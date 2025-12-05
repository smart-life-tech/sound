/*
   Teensy 4.x HYBRID DRUM SYNTH
   - FM attack (with pitch drop)
   - Karplus–Strong resonant body
   - Noise burst transient
   - Flex: selects 1-of-4 notes inside block
   - FSR: attenuation/decay only
   - Piezo ISR + FreeRTOS PlayTask
*/

#include <Arduino.h>
#include <Audio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

#define analogReadFast(pin) analogRead(pin)

// ------------------- PINS -------------------
#define PIEZO_CENTER_PIN A2
#define FLEX_PIN A1
#define FSR_PIN A0

// ------------------- CONFIG -------------------
#define ANALOG_RESOLUTION_BITS 10
#define ADC_MAX 1024

#define PIEZO_THRESHOLD 250
#define PIEZO_DEBOUNCE_MS 2

#define FLEX_MIN 250
#define FLEX_MAX 3800
#define FLEX_SMOOTH_ALPHA 0.24f
#define FLEX_EXPONENT 1.85f

#define PIEZO_SAMPLE_INTERVAL_US 1000000 // 5 kHz ISR

volatile uint16_t sensorC = 0;
volatile uint16_t sensorFSR = 0;
volatile float sensorFlex = 0;
float lastFlex = 0;
float currentFlex = 0;
// ------------------- AUDIO OBJECTS -------------------
// FM attack
AudioSynthWaveform fmMod;
AudioSynthWaveformModulated fmCar;
AudioEffectEnvelope fmEnv;

// Karplus–Strong body
AudioSynthKarplusStrong ks;

// Noise transient
AudioSynthNoiseWhite noise1;
AudioEffectEnvelope noiseEnv;
AudioFilterStateVariable noiseFilter;

// Mixer & output
AudioMixer4 mixer;
AudioOutputI2S audioOut;

AudioConnection patchCord1(fmMod, 0, fmCar, 0);
AudioConnection patchCord2(fmCar, 0, fmEnv, 0);
AudioConnection patchCord3(fmEnv, 0, mixer, 0);

AudioFilterStateVariable ksFilter;
AudioConnection patchCordKS(ks, 0, ksFilter, 0);
AudioConnection patchCordKSOut(ksFilter, 0, mixer, 1); // LP out to mixer

AudioConnection patchCord5(noise1, 0, noiseFilter, 0);
AudioConnection patchCord6(noiseFilter, 0, noiseEnv, 0);
AudioConnection patchCord6b(noiseEnv, 0, mixer, 2);

AudioConnection patchCord7(mixer, 0, audioOut, 0);
AudioConnection patchCord8(mixer, 0, audioOut, 1);

AudioControlSGTL5000 audioShield;

// ------------------- GLOBAL STATE -------------------
IntervalTimer piezoTimer;
TaskHandle_t PlayTaskHandle = NULL;

volatile uint16_t piezoCenterPeak = 0;
volatile uint32_t lastHitMs = 0;
volatile float smoothedFlex = 0.0f;
bool test = true;

// Peak-tracking state for ISR: prevent double-hit on release
volatile bool inPeakState = false; // true when above threshold, false when below

// --- Synth tuning constants (from analyze.py recommendations, tuned for better match) ---
// Talking-drum tuned defaults (midrange, warmer body)
static constexpr float S_MIN_FREQ = 10f;   // avoid overly deep guitar-like body
static constexpr float S_MAX_FREQ = 4.0f;   // cap upper range to talking-drum region
static constexpr int S_FM_DROP_MS = 80;       // pitch drop time (ms)
static constexpr int S_KS_SUSTAIN_MS = 220;   // default sustain (ms) for talking-drum feel
// scale down analyzer's aggressive FM/index and noise suggestions to avoid metallic harshness
// Reduce FM/noise aggressively to avoid metallic timbre
static constexpr float S_FM_INDEX_SCALE = 0.0f; // disable FM content to remove metallic clatter
// noise transient scale — tuned down to remove iron/clatter artifacts
static constexpr float S_NOISE_LEVEL_SCALE = 0.22f; // quieter, filtered transient

float minFreq = S_MIN_FREQ;
float maxFreq = S_MAX_FREQ;
// 8 notes C4 → C5
float gNotes[8] = {
    71.0f,   // C4 261.63f
    293.66f, // D4
    329.63f, // E4
    349.23f, // F4
    392.00f, // G4
    440.00f, // A4
    493.88f, // B4
    523.25f  // C5
};

// Mixer/KS fade state (soft tail cut to avoid metallic clicks)
float gMixerKSGainDefault = 1.6f; // controlled default for KS mixer channel
bool gKsFadeActive = false;
uint32_t gKsFadeStartMs = 0;
const uint32_t G_KS_FADE_MS = 40; // fade duration in ms

// ------------ Pitch Envelope for FM carrier ------------
struct PitchEnvState
{
  bool active;
  float startFreq;
  float endFreq;
  uint32_t startMs;
  uint32_t durMs;
};

PitchEnvState gPitchEnv = {false, 0, 0, 0, 0};

// ks note-off scheduling to enforce a fixed overall decay
volatile uint32_t ksNoteOffMs = 0;

// ------------------- HELPERS -------------------
static inline float clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}

static inline int clampi(int v, int lo, int hi)
{
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}

static inline float mapf(float x, float inMin, float inMax, float outMin, float outMax)
{
  return outMin + (outMax - outMin) * ((x - inMin) / (inMax - inMin));
}

void triggerDrum(uint16_t peakC, float flex, uint16_t fsr);
// 0..1 velocity from peak
float velocityNorm(uint16_t peak)
{
  if (peak <= PIEZO_THRESHOLD)
    return 0.0f;
  float n = (float)(peak - PIEZO_THRESHOLD) / (float)(ADC_MAX - PIEZO_THRESHOLD);
  return clampf(n, 0.0f, 1.0f);
}

// Flex → 0..3 index
int flexToIndex(float flex)
{
  float c = clampf(flex, FLEX_MIN, FLEX_MAX);
  float n = (c - FLEX_MIN) / (FLEX_MAX - FLEX_MIN); // 0..1
  float curved = powf(n, FLEX_EXPONENT);            // exponential feel
  int idx = (int)(curved * 3.999f);                 // 0..3
  return clampi(idx, 0, 3);
}

int flexToIndex7(float flex)
{
  // clamp to ADC range
  float c = clampf(flex, 0.0f, 1024.0f);

  // normalize to 0..1
  float n = c / 1024.0f;

  // exponential feel curve
  float curved = powf(n, FLEX_EXPONENT);

  // scale to 0..7
  int idx = (int)(curved * 7.999f);

  // safety
  return clampi(idx, 0, 7);
}

// FSR → KS damping (0.1 resonant, 0.95 dead), then adjust by velocity for brightness
float fsrToDamping(uint16_t fsr, float velNorm)
{
  float fsrN = (float)fsr / (float)ADC_MAX;
  float baseD = 0.2f + fsrN * 0.75f; // 0.2..0.95
  float bright = velNorm * 0.2f;     // brighter at higher vel
  float d = baseD - bright;          // high vel -> smaller damping -> brighter
  return clampf(d, 0.1f, 0.95f);
}

float fmIndexFromVelocity(float velNorm)
{
  // FM index (brightness) from velocity only
  float base = 5.0f + velNorm * 25.0f; // 5..30
  return base;
}

// Noise amplitude from velocity
float noiseAmpFromVelocity(float velNorm)
{
  // Noise transient amplitude from velocity only
  float base = 0.2f + velNorm * 0.5f; // 0.2..0.7
  return clampf(base, 0.0f, 1.0f);
}
// Controls tone & decay of KS via external lowpass filter
void setKSDamping(uint16_t fsr, float velNorm)
{
  // map FSR (0..1024) → cutoff for bass and damping
  float fsrN = (float)fsr / 1024.0f;
  // More conservative cutoff and resonance mapping to avoid metallic peaks
  // Base cutoff in low-mid range, add gentle brightness from velocity
  float baseCutoff = 150.0f;
  float brightness = velNorm * 200.0f; // up to +200 Hz from velocity
  float finalCutoff = baseCutoff + brightness - fsrN * 80.0f; // pressing lowers cutoff
  finalCutoff = constrain(finalCutoff, 120.0f, 600.0f);
  ksFilter.frequency(finalCutoff);

  // Use mild resonance values (1.5..0.2) to keep body warm and avoid metallic ringing
  float q = 1.5f - fsrN * 1.3f; // higher FSR -> slightly lower Q
  q = clampf(q, 0.2f, 1.5f);
  ksFilter.resonance(q);
}

// Start FM pitch envelope (pitch drop)
void startPitchEnv(float startFreq, float endFreq, uint32_t durMs)
{
  gPitchEnv.active = true;
  gPitchEnv.startFreq = startFreq;
  gPitchEnv.endFreq = endFreq;
  gPitchEnv.startMs = millis();
  gPitchEnv.durMs = durMs;
  fmCar.frequency(startFreq);
}

// Update pitch envelope in loop()
void updatePitchEnv()
{
  if (!gPitchEnv.active)
    return;
  uint32_t now = millis();
  uint32_t elapsed = now - gPitchEnv.startMs;
  if (elapsed >= gPitchEnv.durMs)
  {
    fmCar.frequency(gPitchEnv.endFreq);
    gPitchEnv.active = false;
    return;
  }
  float t = (float)elapsed / (float)gPitchEnv.durMs; // 0..1
  // Exponential pitch drop
  float expTerm = expf(-3.0f * t);
  float f = gPitchEnv.endFreq + (gPitchEnv.startFreq - gPitchEnv.endFreq) * expTerm;
  fmCar.frequency(f);
}

// ------------------- ISR: Piezo + Flex -------------------
void piezoISR()
{
  // digitalToggleFast(13);
  uint16_t c = analogReadFast(PIEZO_CENTER_PIN);

  if (c > piezoCenterPeak)
    piezoCenterPeak = c;

  uint32_t now = millis();
  
  // Peak-tracking: only trigger on rising edge (transition from idle below threshold to above)
  bool aboveThreshold = (c > PIEZO_THRESHOLD);
  
  if (aboveThreshold && !inPeakState && (now - lastHitMs > PIEZO_DEBOUNCE_MS))
  {
    // Rising edge detected: transition from below to above threshold
    inPeakState = true;
    lastHitMs = now;
    
    // Snapshot sensor state (center-only)
    uint16_t peakC;
    float flexLocal;
    noInterrupts();
    peakC = piezoCenterPeak;
    piezoCenterPeak = 0;
    flexLocal = smoothedFlex;
    interrupts();

    uint16_t fsr = analogReadFast(FSR_PIN);
    Serial.print("Hit! C:");
    Serial.print(peakC);
    Serial.print(" Flex:");
    Serial.print(flexLocal, 2);
    Serial.print(" FSR:");
    Serial.print(fsr);
    Serial.println();
    digitalToggleFast(13);
    triggerDrum(peakC, flexLocal, fsr);
  }
  else if (!aboveThreshold)
  {
    // Falling edge: return to idle state
    inPeakState = false;
  }

  // Flex smoothing (runs at ISR rate)
  float flexRaw = analogReadFast(FLEX_PIN);
  smoothedFlex += FLEX_SMOOTH_ALPHA * (flexRaw - smoothedFlex);
  sensorC = c;
  float fsr = analogReadFast(FSR_PIN);
  sensorFSR = fsr;
  // keep a snapshot for the print task
  sensorFlex = smoothedFlex;
}

// ------------------- TRIGGER DRUM -------------------
void triggerDrum(uint16_t peakC, float flex, uint16_t fsr)
{
  uint16_t peak = peakC;
  if (peak < PIEZO_THRESHOLD)
    return;

  float velN = velocityNorm(peak);

  // Block concept retained but not used directly here (flex selects note index)

  // Flex-based pitch mapping (use the passed smoothed flex value)
  int flexIdx = flexToIndex7(flex);
  Serial.print(" NoteIndex:");
  Serial.println(flexIdx);
  Serial.print(" flex read:");
  Serial.println(flex);

  // Map flex (smoothed value passed in) to frequency using a gentle nonlinear curve
  float clamped = constrain(flex, FLEX_MIN, FLEX_MAX);
  float norm = (clamped - FLEX_MIN) / (FLEX_MAX - FLEX_MIN); // 0..1
  // Use an exponent <1 to expand the upper portion (more responsive to flex)
  float curveExp = 0.65f; // tweakable: <1 -> more high pitches, >1 -> more low pitches
  float curved = powf(norm, curveExp);
  float noteFreq = minFreq + curved * (maxFreq - minFreq);
  Serial.print(" NoteFreq:");
  Serial.println(noteFreq);
  // FSR -> damping / decay only
  // float damping = fsrToDamping(fsr, velN);

  // ---------------- KS BODY ----------------
  ks.noteOff(1.0f); // kill previous

  // Set decay/tone (filter + resonance) based on FSR and velocity
  setKSDamping(fsr, velN);

  // Use FSR to control effective sustain only: higher FSR -> shorter sustain
  float fsrN = (float)fsr / (float)ADC_MAX; // 0..1

  // KS amplitude should NOT be reduced by FSR; preserve full hit amplitude
  // Target analysis shows strong presence throughout the decay, so higher velocity helps
  float ksVel = 0.5f + velN * 0.5f; // 0.5..1.0 (increased from 0.3..1.0)
  // restore KS mixer gain and cancel any fade in progress
  mixer.gain(1, gMixerKSGainDefault);
  gKsFadeActive = false;
  ks.noteOn(noteFreq, ksVel);

  // If FSR pressed strongly, cut sustain quickly; otherwise scale sustain inversely
  const float FSR_CUTOFF_IMMEDIATE = 0.25f; // threshold above which we aggressively cut
  uint32_t sustainMs;
  if (fsrN >= FSR_CUTOFF_IMMEDIATE)
  {
    sustainMs = 50; // immediate short cutoff after hit
  }
  else
  {
    // Use tuned KS sustain when FSR not strongly pressed
    sustainMs = (uint32_t)S_KS_SUSTAIN_MS;
  }
  ksNoteOffMs = millis() + sustainMs;

  // ---------------- FM ATTACK ----------------
  fmEnv.noteOff(); // clear old env quickly

  float baseFmFreq = noteFreq * 1.2f; // center-only tuning
  float fmIndex = fmIndexFromVelocity(velN);

  // Do not reduce FM amplitude by FSR; only shorten its release

  fmMod.begin(WAVEFORM_SINE);
  fmCar.begin(WAVEFORM_SINE);
  fmMod.frequency(baseFmFreq * 2.0f); // modulator
  fmMod.amplitude(fmIndex * S_FM_INDEX_SCALE);

  // Pitch envelope: start high and drop down (use measured drop but slightly lengthened)
  float dropAmt = 1.0f + velN * 2.0f; // 1..3x multiplier for pitch rise at start
  float startF = baseFmFreq * dropAmt; // starts high (pitch attack)
  uint32_t envDur = (uint32_t)S_FM_DROP_MS; // use tuned FM drop time
  startPitchEnv(startF, baseFmFreq, envDur);

  // FM envelope: use short attack + FSR-scaled decay/release
  fmEnv.attack(1);
  int fmDecay = 8;
  int fmRelease = (int)max(10.0f, 80.0f * (1.0f - fsrN));
  fmEnv.decay(fmDecay); // small decay
  fmEnv.sustain(0);
  fmEnv.release(fmRelease);
  fmEnv.noteOn();

  // ---------------- NOISE TRANSIENT ----------------
  noiseEnv.noteOff();
  float nAmp = noiseAmpFromVelocity(velN);
  // Keep noise amplitude intact but scale down globally to reduce metallic harshness
  noise1.amplitude(nAmp * S_NOISE_LEVEL_SCALE);
  // Noise transient: keep as short burst, give tiny attack to avoid impulse HF click
  noiseEnv.attack(2);
  int noiseDecay = 12;
  int noiseRelease = (int)max(8.0f, 60.0f * (1.0f - fsrN));
  noiseEnv.decay(noiseDecay);
  noiseEnv.sustain(0);
  noiseEnv.release(noiseRelease);
  // Per-hit noise lowpass to remove metallic high-frequency content
  float noiseCut = noteFreq * 6.0f; // center higher than body
  noiseCut = constrain(noiseCut, 1200.0f, 5000.0f);
  noiseFilter.frequency(noiseCut);
  noiseEnv.noteOn();
}

void SensorPrintTask(void *pvParam)
{
  (void)pvParam;
  while (1)
  {
    uint16_t c, fsr;
    float flex;

    noInterrupts();
    c = sensorC;
    fsr = sensorFSR;
    flex = sensorFlex;
    interrupts();

    Serial.print("C:");
    Serial.print(c);
    Serial.print(" Flex:");
    Serial.print(flex, 1);
    Serial.print(" FSR:");
    Serial.println(fsr);

    vTaskDelay(pdMS_TO_TICKS(100)); // print every 10ms (100Hz)
  }
}

// ------------------- PlayTask -------------------
void PlayTask(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    // Wait for hit notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Snapshot sensor state (center-only)
    uint16_t peakC;
    float flexLocal;
    noInterrupts();
    peakC = piezoCenterPeak;
    piezoCenterPeak = 0;
    flexLocal = smoothedFlex;
    interrupts();

    uint16_t fsr = analogReadFast(FSR_PIN);
    Serial.print("Hit! C:");
    Serial.print(peakC);
    Serial.print(" Flex:");
    Serial.print(flexLocal, 2);
    Serial.print(" FSR:");
    Serial.print(fsr);
    Serial.println();
    digitalToggleFast(13);
    triggerDrum(peakC, flexLocal, fsr);

    // Small yield
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ------------------- SETUP -------------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  // Pins
  pinMode(PIEZO_CENTER_PIN, INPUT);
  pinMode(FLEX_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);

  // Audio
  if (test)
  {
    AudioMemory(32);
    audioShield.enable();
    audioShield.volume(1.0f);

    mixer.gain(0, 0.0f); // FM muted at mixer
    mixer.gain(1, gMixerKSGainDefault); // KS gain (controlled default)
    mixer.gain(2, 0.8f); // Noise present to provide transient 'talking' character
    mixer.gain(3, 0.0f); // unused

    analogReadResolution(ANALOG_RESOLUTION_BITS);
    analogReadAveraging(1);

    smoothedFlex = analogReadFast(FLEX_PIN);

    // initialize noise filter to a conservative lowpass to reduce HF clatter
    noiseFilter.frequency(2500.0f); // 2.5 kHz default
    noiseFilter.resonance(0.6f);

    // Create PlayTask
    BaseType_t res = xTaskCreate(
        PlayTask,
        "PlayTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &PlayTaskHandle);
    if (res != pdPASS)
    {
      Serial.println("ERROR: PlayTask creation failed");
      while (1)
        delay(1000);
    }
    else
    {
      Serial.println("PlayTask created.");
    }
    // Create SensorPrintTask
    res = xTaskCreate(
        SensorPrintTask,
        "SensorPrint",
        2048,
        NULL,
        0, // low priority
        NULL);
    if (res != pdPASS)
    {
      Serial.println("ERROR: SensorPrintTask creation failed");
      while (1)
        delay(1000);
    }
    else
    {
      Serial.println("SensorPrintTask created.");
    }
    // Start piezo ISR

    // piezoTimer.begin(piezoISR, PIEZO_SAMPLE_INTERVAL_US);
  }
  Serial.println("FM + KS Drum Synth Online.");
}

// ------------------- LOOP -------------------
void loop()
{
  // Update pitch envelope at low rate
  if (!test)
  {
    updatePitchEnv();
    vTaskDelay(pdMS_TO_TICKS(10)); // ~500 Hz control rate
  }
  else
  {
    delay(5);
    // uint16_t c = analogRead(PIEZO_CENTER_PIN);
    // Serial.print(" Center tap:");
    // Serial.println(c);
    updatePitchEnv();

    // Continuous FSR damping for post-hit decay control (conservative mapping)
    uint16_t fsr = analogReadFast(FSR_PIN);
    float fsrN = (float)fsr / 1024.0f;
    // Keep resonance in a mild range to avoid metallic ringing
    float q = 1.5f - fsrN * 1.3f; // 1.5 .. ~0.2
    q = clampf(q, 0.2f, 1.5f);
    ksFilter.resonance(q);

    // Call ISR processing (keeps existing behaviour)
    piezoISR();

    // Enforce scheduled KS note-off: start a short soft fade instead of abrupt kill
    if (ksNoteOffMs != 0 && millis() >= ksNoteOffMs)
    {
      // start fade
      gKsFadeActive = true;
      gKsFadeStartMs = millis();
      ksNoteOffMs = 0;
    }

    // Process KS fade if active
    if (gKsFadeActive)
    {
      uint32_t e = millis() - gKsFadeStartMs;
      if (e < G_KS_FADE_MS)
      {
        float t = (float)e / (float)G_KS_FADE_MS;
        float newGain = gMixerKSGainDefault * (1.0f - t);
        mixer.gain(1, newGain);
      }
      else
      {
        mixer.gain(1, 0.0f);
        ks.noteOff(1.0f);
        gKsFadeActive = false;
      }
    }
  }
}

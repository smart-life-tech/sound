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

// Mixer & output
AudioMixer4 mixer;
AudioOutputI2S audioOut;

AudioConnection patchCord1(fmMod, 0, fmCar, 0);
AudioConnection patchCord2(fmCar, 0, fmEnv, 0);
AudioConnection patchCord3(fmEnv, 0, mixer, 0);

AudioFilterStateVariable ksFilter;
AudioConnection patchCordKS(ks, 0, ksFilter, 0);
AudioConnection patchCordKSOut(ksFilter, 0, mixer, 1); // LP out to mixer

AudioConnection patchCord5(noise1, 0, noiseEnv, 0);
AudioConnection patchCord6(noiseEnv, 0, mixer, 2);

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
float minFreq = 80.0f;   // Minimum pitch (F0 Min from analysis ~83 Hz)
float maxFreq = 1000.0f; // Maximum pitch (F0 Max ~1026 Hz from analysis)
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

  float cutoff = 100.0f - fsrN * 50.0f; // much lower base cutoff for maximum bass
  float brightness = velNorm * 100.0f;  // reduced brightness addition

  float finalCutoff = cutoff + brightness;
  if (finalCutoff < 100.0f)
    finalCutoff = 100.0f;
  if (finalCutoff > 1000.0f)
    finalCutoff = 1000.0f;

  ksFilter.frequency(finalCutoff);

  // resonance Q controls decay speed: higher FSR = more damping (less resonance, shorter decay)
  float q = 4.0f - fsrN * 3.5f; // higher FSR = lower Q for shorter decay
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
  if ((c > PIEZO_THRESHOLD) &&
      (now - lastHitMs > PIEZO_DEBOUNCE_MS))
  {
    lastHitMs = now;
    // BaseType_t hpw = pdFALSE;
    //  vTaskNotifyGiveFromISR(PlayTaskHandle, &hpw);
    //  portYIELD_FROM_ISR(hpw);
    //   Snapshot sensor state (center-only)
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

  // Flex smoothing (runs at 5kHz)
  float flexRaw = analogReadFast(FLEX_PIN);
  // Serial.print(" Flex Raw:");
  // Serial.println(flexRaw);
  smoothedFlex += FLEX_SMOOTH_ALPHA * (flexRaw - smoothedFlex);
  sensorC = c;
  float fsr = analogReadFast(FSR_PIN);
  sensorFSR = fsr;
  // keep a snapshot for the print task
  sensorFlex = smoothedFlex;

  // Serial.print("C:");
  // Serial.print(c);
  // Serial.print(" R:");
  // Serial.print(r);
  // Serial.print(" Flex:");
  // Serial.print(flexRaw);
  // Serial.print(" FSR:");
  // Serial.println(fsr);
  // vTaskDelay(pdMS_TO_TICKS(200));
}

// ------------------- TRIGGER DRUM -------------------
void triggerDrum(uint16_t peakC, float flex, uint16_t fsr)
{
  uint16_t peak = peakC;
  if (peak < PIEZO_THRESHOLD)
    return;

  float velN = velocityNorm(peak);

  // Block concept retained but not used directly here (flex selects note index)

  // Flex -> 0..3 inside the block
  int flexIdx = flexToIndex7(flex);
  int noteIndex = flexIdx; // blockStart + flexIdx;
  Serial.print(" NoteIndex:");
  Serial.println(noteIndex);
  Serial.print(" flex read:");
  Serial.println(analogReadFast(FLEX_PIN));
  // noteIndex = clampi(noteIndex, 0, 7);
  // float noteFreq = gNotes[noteIndex];
  // maxFreq = minFreq + maxFreq;
  float newValue = analogRead(FLEX_PIN); // or whatever sensor you're using

  // check if difference is >= 50
  if (abs(newValue - lastFlex) >= 150)
  {
    currentFlex = newValue; // update variable
    lastFlex = newValue;    // remember this value for next time
    Serial.print("last value: ");
    Serial.println(currentFlex);
    Serial.print("Updated value: ");
    Serial.println(currentFlex);
  }
  float noteFreq = map(currentFlex, 0, 1024, minFreq, (maxFreq));
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
    // scale sustain between 500..50 ms when FSR not strongly pressed
    float s = 500.0f * (1.0f - fsrN);
    if (s < 50.0f)
      s = 50.0f;
    sustainMs = (uint32_t)s;
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
  fmMod.amplitude(fmIndex);

  // Pitch envelope: start high and drop down (target audio shows 1000Hz -> 80Hz drop)
  // FM carrier pitch drop simulates the attack-to-decay pitch slide
  float dropAmt = 1.0f + velN * 2.0f; // 1..3x multiplier for pitch rise at start
  float startF = baseFmFreq * dropAmt; // starts high (pitch attack)
  uint32_t envDur = 80 + (uint32_t)(velN * 150.0f); // 80..230ms for slower pitch fall
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
  // Keep noise amplitude intact; shorten release when FSR pressed
  noise1.amplitude(nAmp);

  // Noise transient: keep as short burst, scaled by FSR for release only
  noiseEnv.attack(0);
  int noiseDecay = 12;
  int noiseRelease = (int)max(8.0f, 60.0f * (1.0f - fsrN));
  noiseEnv.decay(noiseDecay);
  noiseEnv.sustain(0);
  noiseEnv.release(noiseRelease);
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

    mixer.gain(0, 0.0f); // FM eliminated for no metallic
    mixer.gain(1, 2.5f); // KS gain increased to compensate for low volume from deep bass filter
    mixer.gain(2, 0.0f); // Noise eliminated to remove any metallic transient
    mixer.gain(3, 0.0f); // unused

    analogReadResolution(ANALOG_RESOLUTION_BITS);
    analogReadAveraging(1);

    smoothedFlex = analogReadFast(FLEX_PIN);

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

    // Continuous FSR damping for post-hit decay control
    uint16_t fsr = analogReadFast(FSR_PIN);
    float fsrN = (float)fsr / 1024.0f;
    float q = 4.0f - fsrN * 3.9f; // higher FSR = lower Q = more damping, down to 0.1
    ksFilter.resonance(q);

    // Call ISR processing (keeps existing behaviour)
    piezoISR();

    // Enforce scheduled KS note-off to limit settling to ~500 ms
    if (ksNoteOffMs != 0 && millis() >= ksNoteOffMs)
    {
      ks.noteOff(1.0f);
      ksNoteOffMs = 0;
    }
  }
}

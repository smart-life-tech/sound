/*
   Teensy 4.x YORUBA TALKING DRUM (GÀNGÀN) - Final Version

   Model:
   - Karplus–Strong = wooden shell/body (≈ 83 Hz from recording)
   - FM synth = talking head (565–1025 Hz, from your F0 analysis)
   - Noise = skin/strike transient
   - Flex (A1) = continuous rope tension → pitch bend (smooth, not stepped)
   - FSR (A0) = damping/choke (controls decay & resonance)
   - Piezo (A2) = hit trigger (velocity from amplitude)

   Output:
   - I2S (e.g. PCM5102 DAC), no SGTL5000 used in this version.
*/

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#define analogReadFast(pin) analogRead(pin)

// ------------------- PINS -------------------
#define PIEZO_PIN A2
#define FLEX_PIN  A1
#define FSR_PIN   A0

// ------------------- CONFIG -------------------
#define ANALOG_RES_BITS   12        // 0..4095
#define ADC_MAX           4095

#define PIEZO_THRESHOLD   220       // hit sensitivity
#define PIEZO_DEBOUNCE_MS 10        // ms debounce between hits

#define FLEX_SMOOTH_ALPHA 0.20f     // smoothing factor for flex (0..1)
#define FLEX_EXPONENT     2.2f      // flex → pitch curve exponent

#define PIEZO_SAMPLE_US   500       // 2 kHz ISR

// Frequencies derived from your real recording:
#define GANGAN_F0_MIN   83.4f       // Hz (body / lowest tension)
#define GANGAN_F0_MED   565.8f      // Hz (median)
#define GANGAN_F0_MAX  1025.6f      // Hz (highest tension)

// ------------------- AUDIO OBJECTS -------------------
// FM talking head
AudioSynthWaveform          fmMod;
AudioSynthWaveformModulated fmCar;
AudioEffectEnvelope         fmEnv;

// Karplus–Strong shell/body
AudioSynthKarplusStrong     ks;
AudioFilterStateVariable    ksFilter;

// Noise transient
AudioSynthNoiseWhite        noise1;
AudioEffectEnvelope         noiseEnv;

// Mixer & output
AudioMixer4                 mixer;
AudioOutputI2S              i2s1;

AudioConnection patchCord1(fmMod, 0, fmCar, 0);
AudioConnection patchCord2(fmCar, 0, fmEnv, 0);
AudioConnection patchCord3(fmEnv, 0, mixer, 0);

AudioConnection patchCord4(ks, 0, ksFilter, 0);
AudioConnection patchCord5(ksFilter, 0, mixer, 1);

AudioConnection patchCord6(noise1, 0, noiseEnv, 0);
AudioConnection patchCord7(noiseEnv, 0, mixer, 2);

AudioConnection patchCord8(mixer, 0, i2s1, 0);
AudioConnection patchCord9(mixer, 0, i2s1, 1);

// If you *do* use the Teensy Audio Shield instead of PCM5102, you can enable this:
// AudioControlSGTL5000 audioShield;

// ------------------- GLOBAL STATE -------------------
IntervalTimer piezoTimer;

// piezo / flex state
volatile uint16_t piezoPeak   = 0;
volatile uint32_t lastHitMs   = 0;
volatile bool     hitPending  = false;
volatile float    smoothedFlex = 0.0f;
volatile float    flexAtHit    = 0.0f;

// pitch envelope state for FM
struct PitchEnvState {
  bool     active;
  float    startFreq;
  float    endFreq;
  uint32_t startMs;
  uint32_t durMs;
};
PitchEnvState gPitchEnv = {false, 0, 0, 0, 0};

// ------------------- HELPERS -------------------
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (outMax - outMin) * ((x - inMin) / (inMax - inMin));
}

// Piezo → velocity 0..1
float velocityNorm(uint16_t peak) {
  if (peak <= PIEZO_THRESHOLD) return 0.0f;
  float n = (float)(peak - PIEZO_THRESHOLD) / (float)(ADC_MAX - PIEZO_THRESHOLD);
  return clampf(n, 0.0f, 1.0f);
}

// Flex ADC (0..4095) → base drum pitch (continuous, uses your real F0 range)
float flexToDrumPitch(float flex) {
  float n = clampf(flex / (float)ADC_MAX, 0.0f, 1.0f);
  float curved = powf(n, FLEX_EXPONENT);  // leather tension curve
  return GANGAN_F0_MIN + curved * (GANGAN_F0_MAX - GANGAN_F0_MIN);
}

// Flex → continuous bend multiplier for live bending while ringing
float flexToBend(float flex) {
  float n = clampf(flex / (float)ADC_MAX, 0.0f, 1.0f);
  float curved = powf(n, FLEX_EXPONENT);
  // 1.0 (no bend) → 2.2 (a bit over an octave)
  return 1.0f + curved * 1.2f;
}

// FSR → KS damping and tone
void setKSDamping(uint16_t fsr, float velNorm, float baseFreq) {
  float fsrN = (float)fsr / (float)ADC_MAX;

  // Body lowpass cutoff relative to base frequency
  float cutoff = baseFreq * (1.5f + (1.0f - fsrN) * 3.0f); // low FSR = ring more
  cutoff = clampf(cutoff, 80.0f, 8000.0f);

  // Q: low FSR → high Q (ring), high FSR → damped
  float q = 3.5f - fsrN * 2.8f;
  q = clampf(q, 0.7f, 3.5f);

  ksFilter.frequency(cutoff);
  ksFilter.resonance(q);
}

// Start FM pitch envelope (for talking “wah”)
void startPitchEnv(float startFreq, float endFreq, uint32_t durMs) {
  gPitchEnv.active    = true;
  gPitchEnv.startFreq = startFreq;
  gPitchEnv.endFreq   = endFreq;
  gPitchEnv.startMs   = millis();
  gPitchEnv.durMs     = durMs;
  fmCar.frequency(startFreq);
}

// ------------------- ISR: Piezo + Flex -------------------
void piezoISR() {
  uint16_t c = analogReadFast(PIEZO_PIN);

  if (c > piezoPeak) piezoPeak = c;

  uint32_t now = millis();
  if (c > PIEZO_THRESHOLD && (now - lastHitMs) > PIEZO_DEBOUNCE_MS) {
    lastHitMs   = now;
    flexAtHit   = smoothedFlex;  // snapshot flex at hit
    hitPending  = true;
    // let piezoPeak hold previous max until processed
  }

  // Flex smoothing at ISR rate
  float rawFlex = (float)analogReadFast(FLEX_PIN);
  smoothedFlex += FLEX_SMOOTH_ALPHA * (rawFlex - smoothedFlex);
}

// ------------------- FM TALKING LAYER -------------------
void triggerFM(float basePitch, float velNorm, float bendAtHit, uint16_t fsr) {
  float fsrN = (float)fsr / (float)ADC_MAX;

  // From your F0 contour: start higher, dip, settle near basePitch
  float startF = basePitch * bendAtHit * (1.8f); // high start
  float endF   = basePitch * bendAtHit;          // settle back

  uint32_t sweepMs = 120 + (uint32_t)(velNorm * 80.0f); // 120–200 ms
  startPitchEnv(startF, endF, sweepMs);

  // Nasal / vowel-like FM depth (strong, like real gangan)
  float fmIndex = 35.0f + velNorm * 85.0f;
  fmMod.begin(WAVEFORM_SINE);
  fmMod.frequency(basePitch * 2.1f);
  fmMod.amplitude(fmIndex);

  fmCar.begin(WAVEFORM_SINE);

  // AM envelope = loud attack, quick decay, short tail
  fmEnv.attack(1);
  fmEnv.decay(80 + (int)(velNorm * 40.0f));
  fmEnv.sustain(0.0f);
  fmEnv.release(40 + (int)((1.0f - fsrN) * 40.0f)); // FSR chokes tail
  fmEnv.noteOn();

  // Noise transient for “skin” texture
  noiseEnv.noteOff();
  float nAmp = 0.15f + velNorm * 0.35f;
  noise1.amplitude(nAmp);
  noiseEnv.attack(0);
  noiseEnv.decay(10 + (int)(velNorm * 15.0f));
  noiseEnv.sustain(0.0f);
  noiseEnv.release(20 + (int)((1.0f - fsrN) * 40.0f));
  noiseEnv.noteOn();
}

// ------------------- KS BODY -------------------
void triggerKS(float basePitch, float velNorm, uint16_t fsr) {
  // Body is mostly fixed shell mode ≈ 83 Hz, not bent by flex
  ks.noteOff(1.0f);

  float bodyFreq = GANGAN_F0_MIN; // deep shell
  setKSDamping(fsr, velNorm, bodyFreq);

  float bodyAmp = 0.25f + velNorm * 0.75f;
  ks.noteOn(bodyFreq, bodyAmp);
}

// ------------------- TRIGGER FULL DRUM -------------------
void triggerDrum(uint16_t peak, float flexAtHitLocal, uint16_t fsr) {
  if (peak < PIEZO_THRESHOLD) return;

  float velN = velocityNorm(peak);

  // Base pitch from flex at the moment of the hit
  float basePitch = flexToDrumPitch(flexAtHitLocal);
  float bendAtHit = flexToBend(flexAtHitLocal);

  // Debug (optional)
  Serial.print("Hit: peak=");
  Serial.print(peak);
  Serial.print(" flexAtHit=");
  Serial.print(flexAtHitLocal, 1);
  Serial.print(" basePitch=");
  Serial.print(basePitch, 1);
  Serial.print(" fsr=");
  Serial.println(fsr);

  // KS body + FM talking head
  triggerKS(basePitch, velN, fsr);
  triggerFM(basePitch, velN, bendAtHit, fsr);
}

// ------------------- UPDATE PITCH ENV + LIVE BEND -------------------
void updatePitchEnv() {
  if (!gPitchEnv.active) return;

  uint32_t now = millis();
  uint32_t elapsed = now - gPitchEnv.startMs;

  // Live flex for continuous bending (rope squeeze while ringing)
  float flexLocal = smoothedFlex;
  float bendLive  = flexToBend(flexLocal);

  if (elapsed >= gPitchEnv.durMs) {
    float finalF = gPitchEnv.endFreq * bendLive;
    fmCar.frequency(finalF);
    gPitchEnv.active = false;
    return;
  }

  float t = (float)elapsed / (float)gPitchEnv.durMs;  // 0..1
  float expTerm = expf(-3.0f * t);
  float f = gPitchEnv.endFreq +
            (gPitchEnv.startFreq - gPitchEnv.endFreq) * expTerm;

  fmCar.frequency(f * bendLive);
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Yoruba GANGAN Synth starting...");

  pinMode(PIEZO_PIN, INPUT);
  pinMode(FLEX_PIN,  INPUT);
  pinMode(FSR_PIN,   INPUT);

  AudioMemory(40);

  // If using Teensy Audio Shield instead of raw PCM5102, uncomment:
  // audioShield.enable();
  // audioShield.volume(0.9f);

  mixer.gain(0, 0.7f);  // FM layer
  mixer.gain(1, 1.0f);  // KS body
  mixer.gain(2, 0.3f);  // Noise
  mixer.gain(3, 0.0f);

  analogReadResolution(ANALOG_RES_BITS);
  analogReadAveraging(1);

  smoothedFlex = (float)analogReadFast(FLEX_PIN);

  // Start piezo sampling ISR
  piezoTimer.begin(piezoISR, PIEZO_SAMPLE_US);

  Serial.println("GANGAN Synth Online.");
}

// ------------------- LOOP -------------------
void loop() {
  // 1. Update FM pitch envelope (with live flex bending)
  updatePitchEnv();

  // 2. Check if a hit is pending from ISR
  static uint16_t lastPeakCopy = 0;
  if (hitPending) {
    noInterrupts();
    uint16_t peakCopy = piezoPeak;   // last peak before reset
    float    flexCopy = flexAtHit;
    hitPending = false;
    piezoPeak  = 0;
    interrupts();

    uint16_t fsr = analogReadFast(FSR_PIN);

    triggerDrum(peakCopy, flexCopy, fsr);
  }

  // small delay to keep USB happy & CPU cool
  delay(1);
}

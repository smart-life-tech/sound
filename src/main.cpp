/*
   Teensy 4.x HYBRID YORUBA TALKING DRUM (GÀNGÀN)
   HYBRID MODE (Option C):
   - Sample attack from your real WAV (AudioPlayMemory, drum_raw.h)
   - FM "talking head" synth layer (fully bendable by flex)
   - Karplus–Strong shell/body (deep wooden resonance)
   - Flex (A1): continuous pitch bend (rope tension)
   - FSR  (A0): damping / choke
   - Piezo (A2): hit trigger + velocity (from peak)

   Output: I2S (e.g. PCM5102 DAC)
*/

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#define analogReadFast(pin) analogRead(pin)

// ------------ PINS ------------
#define PIEZO_PIN A2
#define FLEX_PIN  A1
#define FSR_PIN   A0

// ------------ ADC CONFIG ------------
#define ANALOG_RES_BITS 12    // 0..4095
#define ADC_MAX         4095

#define PIEZO_THRESHOLD   220   // adjust to taste
#define PIEZO_DEBOUNCE_MS 10

#define FLEX_SMOOTH_ALPHA 0.20f  // 0..1
#define FLEX_EXPONENT     2.2f   // exponential curve for tension

#define PIEZO_SAMPLE_US   500    // ISR at ~2 kHz

// From your analysis:
#define GANGAN_F0_MIN   83.4f    // Hz, shell / lowest tension
#define GANGAN_F0_MAX 1025.6f    // Hz, highest tension

// ------------ INCLUDE YOUR SAMPLE ------------
#include "drum_raw.h"   // must define: const unsigned int drum_raw[] PROGMEM;

// ------------ AUDIO OBJECTS ------------
// Sample attack
AudioPlayMemory          playMem;

// FM talking head
AudioSynthWaveform          fmMod;
AudioSynthWaveformModulated fmCar;
AudioEffectEnvelope         fmEnv;

// KS shell/body
AudioSynthKarplusStrong     ks;
AudioFilterStateVariable    ksFilter;

// Noise transient
AudioSynthNoiseWhite        noise1;
AudioEffectEnvelope         noiseEnv;

// Mixer + output
AudioMixer4                 mixer;
AudioOutputI2S              i2s1;

// Patch cords
AudioConnection patchCord1(playMem, 0, mixer, 0);      // sample attack → ch0
AudioConnection patchCord2(fmEnv, 0, mixer, 1);        // FM layer → ch1
AudioConnection patchCord3(ksFilter, 0, mixer, 2);     // KS body → ch2
AudioConnection patchCord4(noiseEnv, 0, mixer, 3);     // noise transient → ch3
AudioConnection patchCord5(mixer, 0, i2s1, 0);
AudioConnection patchCord6(mixer, 0, i2s1, 1);

// If you use the Audio Shield instead of a raw I2S DAC, uncomment these:
// AudioControlSGTL5000 audioShield;

// ------------ GLOBAL STATE ------------
IntervalTimer piezoTimer;

volatile uint16_t piezoPeak    = 0;
volatile uint32_t lastHitMs    = 0;
volatile bool     hitPending   = false;
volatile float    smoothedFlex = 0.0f;
volatile float    flexAtHit    = 0.0f;

// FM pitch envelope tracking
struct PitchEnvState {
  bool     active;
  float    startFreq;
  float    endFreq;
  uint32_t startMs;
  uint32_t durMs;
};
PitchEnvState gPitchEnv = {false, 0, 0, 0, 0};

// ------------ HELPERS ------------
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (outMax - outMin) * ((x - inMin) / (inMax - inMin));
}

float velocityNorm(uint16_t peak) {
  if (peak <= PIEZO_THRESHOLD) return 0.0f;
  float n = (float)(peak - PIEZO_THRESHOLD) / (float)(ADC_MAX - PIEZO_THRESHOLD);
  return clampf(n, 0.0f, 1.0f);
}

// Flex ADC (0..4095) → real gangan pitch range 83–1025 Hz
float flexToDrumPitch(float flex) {
  float n = clampf(flex / (float)ADC_MAX, 0.0f, 1.0f);
  float curved = powf(n, FLEX_EXPONENT); // leather tension feel
  return GANGAN_F0_MIN + curved * (GANGAN_F0_MAX - GANGAN_F0_MIN);
}

// Flex → bend multiplier (for FM live bending)
float flexToBend(float flex) {
  float n = clampf(flex / (float)ADC_MAX, 0.0f, 1.0f);
  float curved = powf(n, FLEX_EXPONENT);
  return 1.0f + curved * 1.2f;  // up to ~1.2 octaves above
}

// FSR → damping / resonance for KS & FM tail
void setKSDamping(uint16_t fsr, float velNorm, float baseFreq) {
  float fsrN = (float)fsr / (float)ADC_MAX;

  // body: low FSR = bright + ringing, high FSR = choked
  float cutoff = baseFreq * (1.5f + (1.0f - fsrN) * 3.0f);
  cutoff = clampf(cutoff, 80.0f, 8000.0f);

  float q = 3.5f - fsrN * 2.8f;  // low FSR → high Q
  q = clampf(q, 0.7f, 3.5f);

  ksFilter.frequency(cutoff);
  ksFilter.resonance(q);
}

// Start FM pitch envelope
void startPitchEnv(float startFreq, float endFreq, uint32_t durMs) {
  gPitchEnv.active    = true;
  gPitchEnv.startFreq = startFreq;
  gPitchEnv.endFreq   = endFreq;
  gPitchEnv.startMs   = millis();
  gPitchEnv.durMs     = durMs;
  fmCar.frequency(startFreq);
}

// ------------ ISR: PIEZO + FLEX ------------
void piezoISR() {
  uint16_t c = analogReadFast(PIEZO_PIN);
  if (c > piezoPeak) piezoPeak = c;

  uint32_t now = millis();

  if (c > PIEZO_THRESHOLD && (now - lastHitMs) > PIEZO_DEBOUNCE_MS) {
    lastHitMs  = now;
    flexAtHit  = smoothedFlex;  // snapshot flex at hit
    hitPending = true;
    // keep piezoPeak for velocity; will be reset after main loop processes
  }

  // Flex smoothing (continuous)
  float rawFlex = (float)analogReadFast(FLEX_PIN);
  smoothedFlex += FLEX_SMOOTH_ALPHA * (rawFlex - smoothedFlex);
}

// ------------ SAMPLE + FM + KS TRIGGERS ------------
void triggerFM(float basePitch, float velNorm, float bendAtHit, uint16_t fsr) {
  float fsrN = (float)fsr / (float)ADC_MAX;

  // high start → drift down to base pitch; your WAV's F0 contour style
  float startF = basePitch * bendAtHit * 1.8f;
  float endF   = basePitch * bendAtHit;

  uint32_t sweepMs = 120 + (uint32_t)(velNorm * 80.0f);  // 120–200ms
  startPitchEnv(startF, endF, sweepMs);

  // nasal, vowel-like FM depth
  float fmIndex = 35.0f + velNorm * 85.0f;  // strong at high velocity

  fmMod.begin(WAVEFORM_SINE);
  fmMod.frequency(basePitch * 2.1f);
  fmMod.amplitude(fmIndex);

  fmCar.begin(WAVEFORM_SINE);

  // AM envelope for FM layer
  fmEnv.attack(1);
  fmEnv.decay(80 + (int)(velNorm * 40.0f));
  fmEnv.sustain(0.0f);
  fmEnv.release(40 + (int)((1.0f - fsrN) * 40.0f)); // FSR chokes tail
  fmEnv.noteOn();

  // Noise burst (skin/attack air)
  noiseEnv.noteOff();
  float nAmp = 0.15f + velNorm * 0.35f;
  noise1.amplitude(nAmp);
  noiseEnv.attack(0);
  noiseEnv.decay(10 + (int)(velNorm * 15.0f));
  noiseEnv.sustain(0.0f);
  noiseEnv.release(20 + (int)((1.0f - fsrN) * 40.0f));
  noiseEnv.noteOn();
}

void triggerKS(float velNorm, uint16_t fsr) {
  ks.noteOff(1.0f);

  float bodyFreq = GANGAN_F0_MIN;  // shell/body around 83 Hz
  setKSDamping(fsr, velNorm, bodyFreq);

  float bodyAmp = 0.25f + velNorm * 0.75f;
  ks.noteOn(bodyFreq, bodyAmp);
}

void triggerDrum(uint16_t peak, float flexAtHitLocal, uint16_t fsr) {
  if (peak < PIEZO_THRESHOLD) return;

  float velN = velocityNorm(peak);

  float basePitch = flexToDrumPitch(flexAtHitLocal);
  float bendAtHit = flexToBend(flexAtHitLocal);

  // Debug:
  Serial.print("Hit peak=");
  Serial.print(peak);
  Serial.print(" flexAtHit=");
  Serial.print(flexAtHitLocal, 1);
  Serial.print(" basePitch=");
  Serial.print(basePitch, 1);
  Serial.print(" fsr=");
  Serial.println(fsr);

  // 1) Sample attack (your real drum recording)
  playMem.play(drum_raw);   // from drum_raw.h

  // 2) KS wooden body
  triggerKS(velN, fsr);

  // 3) FM talking head layer
  triggerFM(basePitch, velN, bendAtHit, fsr);
}

// ------------ UPDATE FM PITCH ENV + BEND ------------
void updatePitchEnv() {
  if (!gPitchEnv.active) return;

  uint32_t now = millis();
  uint32_t elapsed = now - gPitchEnv.startMs;

  float flexLocal = smoothedFlex;
  float bendLive  = flexToBend(flexLocal);

  if (elapsed >= gPitchEnv.durMs) {
    float finalF = gPitchEnv.endFreq * bendLive;
    fmCar.frequency(finalF);
    gPitchEnv.active = false;
    return;
  }

  float t = (float)elapsed / (float)gPitchEnv.durMs;
  float expTerm = expf(-3.0f * t);
  float f = gPitchEnv.endFreq +
            (gPitchEnv.startFreq - gPitchEnv.endFreq) * expTerm;

  fmCar.frequency(f * bendLive);
}

// ------------ SETUP ------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Hybrid GANGAN starting...");

  pinMode(PIEZO_PIN, INPUT);
  pinMode(FLEX_PIN,  INPUT);
  pinMode(FSR_PIN,   INPUT);

  AudioMemory(40);

  // If using Audio Shield, uncomment:
  // audioShield.enable();
  // audioShield.volume(0.9f);

  mixer.gain(0, 0.9f);  // sample
  mixer.gain(1, 0.8f);  // FM
  mixer.gain(2, 1.0f);  // KS body
  mixer.gain(3, 0.4f);  // noise

  analogReadResolution(ANALOG_RES_BITS);
  analogReadAveraging(1);

  smoothedFlex = (float)analogReadFast(FLEX_PIN);

  // Start piezo sampling ISR
  piezoTimer.begin(piezoISR, PIEZO_SAMPLE_US);

  Serial.println("Hybrid GANGAN Online.");
}

// ------------ LOOP ------------
void loop() {
  // Update FM pitch envelope with live bending
  updatePitchEnv();

  // If ISR signaled a hit, handle it here
  if (hitPending) {
    noInterrupts();
    uint16_t peakCopy = piezoPeak;
    float    flexCopy = flexAtHit;
    hitPending = false;
    piezoPeak  = 0;
    interrupts();

    uint16_t fsr = analogReadFast(FSR_PIN);

    triggerDrum(peakCopy, flexCopy, fsr);
  }

  delay(1);
}

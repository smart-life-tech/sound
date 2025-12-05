// main.cpp
// Teensy 4.1: GAN-GAN physical-model drum with sensor modulation
// Output: I2S -> PCM5102
// Sensors:
//   PIEZO_PIN (A2) -> trigger
//   FLEX_PIN  (A1) -> filter cutoff (brightness)
//   FSR_PIN   (A0) -> gain + shorten sustain
//
// Wiring notes (external):
//  - PCM5102 BCLK -> Teensy pin 21
//  - PCM5102 LRCK -> Teensy pin 20
//  - PCM5102 DIN  -> Teensy pin 7
//  - PCM5102 VCC  -> 3.3V, GND -> GND
//
// Author: adapted for your project

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ----------------------------- Sensor pin assignments -----------------------------
#define PIEZO_PIN A2
#define FLEX_PIN A1
#define FSR_PIN A0

// Thresholds and smoothing
#define PIEZO_THRESHOLD 250     // tune this for your piezo sensitivity
#define PIEZO_DEBOUNCE_MS 40    // ignore hits within this window (ms)
#define FLEX_SMOOTH_ALPHA 0.18f // 0..1 (higher = less smoothing)
#define FSR_MIN 30              // raw min / deadzone
#define FSR_MAX 4000            // raw max

// ----------------------------- Audio class (must be defined before objects) -----------------------------
class AudioSynthGanGan : public AudioStream
{
public:
  AudioSynthGanGan() : AudioStream(0, NULL)
  {
    // Exciter defaults
    aStartFreq = 2.0f;
    aStopFreq = 40.0f;
    aStartAmp = 0.1f;
    aStopAmp = 1.0f;
    aDuration = 0.20f;
    aKnee = 0; // 0 linear, 1 expIn, 2 expOut

    // Resonator defaults
    bFreq = 220.0f;
    bSubFreq = 36.0f;
    bAmp = 1.0f;
    bSubStd = 0.05f;

    // Envelope / output
    envDurBase = 0.35f; // base env duration (seconds)
    envDur = envDurBase;
    envAttack = 0.001f;
    outputGain = 0.4f;

    // internal state
    phaseA = phaseB = phaseSub = 0.0f;
    sweepT = 1.0f; // idle
    envT = 1.0f;
    active = false;

    sampleRate = (float)AUDIO_SAMPLE_RATE_EXACT;
    twoPI = 2.0f * (float)M_PI;

    // anti-aliasing/softening multipliers (reduce metallic harshness)
    fmDepthScale = 0.65f;    // reduce mod amplitude
    sweepSpreadScale = 0.6f; // reduce sweep extent influence
  }

  virtual void update(void) override
  {
    audio_block_t *block = allocate();
    if (!block)
      return;

    const float invSR = 1.0f / sampleRate;

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
      float out = 0.0f;

      if (active)
      {
        // progress sweep normalized 0..1
        sweepT += 1.0f / (aDuration * sampleRate);
        if (sweepT > 1.0f)
          sweepT = 1.0f;

        // knee curve
        float k = sweepT;
        if (aKnee == 1)
          k = sweepT * sweepT;
        else if (aKnee == 2)
          k = sqrtf(sweepT);

        // modulator freq & amp (softened)
        float fA = aStartFreq + (aStopFreq - aStartFreq) * (k * sweepSpreadScale);
        float aA = aStartAmp + (aStopAmp - aStartAmp) * k;

        // advance modulator phase
        phaseA += fA * invSR;
        if (phaseA >= 1.0f)
          phaseA -= floorf(phaseA);

        float modA = sinf(twoPI * phaseA) * (aA * fmDepthScale);

        // carrier + jitter
        phaseB += bFreq * invSR;
        if (phaseB >= 1.0f)
          phaseB -= floorf(phaseB);

        phaseSub += bSubFreq * invSR;
        if (phaseSub >= 1.0f)
          phaseSub -= floorf(phaseSub);

        float jitter = sinf(twoPI * phaseSub) * bSubStd;
        float toneB = sinf(twoPI * phaseB) * (bAmp + jitter);

        // envelope timing
        envT += invSR;
        float envNorm = envT / envDur;
        if (envNorm >= 1.0f)
          envNorm = 1.0f;

        // percussive envelope: tiny attack then exponential-ish decay
        float env;
        float atkNorm = envAttack / envDur;
        if (envNorm < atkNorm)
        {
          env = envNorm / atkNorm;
        }
        else
        {
          float t = (envNorm - atkNorm) / (1.0f - atkNorm);
          env = expf(-5.0f * t); // -5 shapes decay speed
        }

        out = (1.0f + modA) * toneB * env * outputGain;

        // voice done
        if (sweepT >= 1.0f && envNorm >= 1.0f)
          active = false;
      }

      // clip to [-1,1]
      if (out > 1.0f)
        out = 1.0f;
      if (out < -1.0f)
        out = -1.0f;

      // convert to signed 16-bit PCM (Audio library block format)
      block->data[i] = (int16_t)(out * 32767.0f);
    }

    transmit(block);
    release(block);
  }

  // -------- control API --------
  // Exciter
  void setAStartFreq(float v) { aStartFreq = v; }
  void setAStopFreq(float v) { aStopFreq = v; }
  void setAStartAmp(float v) { aStartAmp = v; }
  void setAStopAmp(float v) { aStopAmp = v; }
  void setADuration(float v) { aDuration = max(0.005f, v); }
  void setAKnee(uint8_t v) { aKnee = v; }

  // Resonator
  void setBFreq(float v) { bFreq = v; }
  void setBSubFreq(float v) { bSubFreq = v; }
  void setBAmp(float v) { bAmp = v; }
  void setBSubStd(float v) { bSubStd = v; }

  // Envelope & output
  // envDur is set dynamically by external FSR mapping for expressive control
  void setEnvDur(float v) { envDur = max(0.01f, v); }
  void setEnvDurBase(float v)
  {
    envDurBase = max(0.01f, v);
    envDur = envDurBase;
  }
  void setEnvAttack(float v) { envAttack = max(0.0f, v); }
  void setOutputGain(float v) { outputGain = v; }

  // tuning for anti-aliasing
  void setFmDepthScale(float v) { fmDepthScale = v; }
  void setSweepSpreadScale(float v) { sweepSpreadScale = v; }

  // trigger
  void noteOn()
  {
    sweepT = 0.0f;
    envT = 0.0f;
    // small phase jitter for more organic repeats
    phaseB += random(0, 100) * 1e-5f;
    active = true;
  }

  bool isActive() const { return active; }

private:
  // params
  float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
  uint8_t aKnee;

  float bFreq, bSubFreq, bAmp, bSubStd;

  float envDurBase, envDur, envAttack, outputGain;

  // state
  float phaseA, phaseB, phaseSub;
  float sweepT, envT;
  bool active;

  // sample rate
  float sampleRate;
  float twoPI;

  // anti-aliasing tuners
  float fmDepthScale;
  float sweepSpreadScale;
};

// ----------------------------- Audio objects and routing -----------------------------
AudioSynthGanGan ganGan;      // our synth voice (single)
AudioFilterBiquad hpDC;       // high-pass to remove DC / low rumble (biquad)
AudioFilterStateVariable svf; // resonant filter for 'body' (cutoff controlled by flex)
AudioOutputI2S i2sOut;        // I2S output to PCM5102

// routing: ganGan -> hpDC -> svf -> i2sOut (stereo)
AudioConnection patch1(ganGan, 0, hpDC, 0);
AudioConnection patch2(hpDC, 0, svf, 0);
AudioConnection patch3(svf, 0, i2sOut, 0);
AudioConnection patch4(svf, 0, i2sOut, 1);

// ----------------------------- Globals for sensor handling -----------------------------
float smoothedFlex = 0.0f; // filtered flex reading
uint32_t lastHitMs = 0;

// ----------------------------- Setup and loop -----------------------------
void setup()
{
  Serial.begin(115200);
  AudioMemory(12);

  // Biquad high-pass (DC blocker) — cutoff around 60 Hz, Q modest
  hpDC.setHighpass(60.0f, 0.7f);

  // SVF initial setup
  svf.frequency(1200.0f); // default cutoff
  svf.resonance(0.8f);    // Q (0..1-ish, tweak for body)

  // synth defaults (tweak to taste)
  ganGan.setBFreq(46.0f);
  ganGan.setEnvDurBase(0.62f);
  ganGan.setBSubFreq(71.0f);
  ganGan.setBSubStd(0.2f);

  ganGan.setAStartFreq(600.0f);
  ganGan.setAStopFreq(6.0f);
  ganGan.setAStartAmp(1.0f);
  ganGan.setAStopAmp(0.2f);
  ganGan.setADuration(0.21f);
  ganGan.setAKnee(1); // expIn

  // anti-aliasing scales (sensible defaults)
  ganGan.setFmDepthScale(0.65f);
  ganGan.setSweepSpreadScale(0.6f);

  // initialize smoothed flex from a read
  smoothedFlex = analogRead(FLEX_PIN);
  lastHitMs = 0;

  // wait a short while so audio objects settle
  delay(50);
}

void loop()
{
  // read sensors (single-shot reads)
  uint16_t piezo = analogRead(PIEZO_PIN);
  uint16_t fsr = analogRead(FSR_PIN);
  uint16_t flex = analogRead(FLEX_PIN);

  uint32_t now = millis();

  // --- Piezo hit detection (edge) ---
  if (piezo > PIEZO_THRESHOLD && (now - lastHitMs) > PIEZO_DEBOUNCE_MS)
  {
    // Map FSR to velocity-like amplitude influence (0..1)
    float fsrNorm = (float)constrain(fsr, 0, 4095) / 4095.0f;

    // Optional: gate only if fsr above tiny amount? we use for dynamics
    // Trigger note
    ganGan.noteOn();
    lastHitMs = now;
  }

  // --- Smooth flex sensor (exponential) ---
  float flexF = (float)flex;
  smoothedFlex = (1.0f - FLEX_SMOOTH_ALPHA) * smoothedFlex + FLEX_SMOOTH_ALPHA * flexF;

  // Map smoothedFlex to filter cutoff (log scale is nicer; we do simple map with clamp)
  // FLEX_MIN/MAX are approximate ADC ranges for your flex sensor; adjust if needed
  float cutoff = map(smoothedFlex, 250.0f, 3800.0f, 300.0f, 12000.0f); // linear mapping
  cutoff = constrain(cutoff, 300.0f, 12000.0f);
  svf.frequency(cutoff);

  // --- FSR controls output gain and shortens sustain ---
  // Map FSR to 0.25..1.0 gain (more pressure = louder)
  float fsrGain = map(fsr, 0, 4095, 250, 1000) / 1000.0f;
  fsrGain = constrain(fsrGain, 0.25f, 1.0f);
  ganGan.setOutputGain(fsrGain * 0.4f); // scale against original base gain

  // Map FSR to envDur shortening: heavier FSR -> shorter sustain (more damped)
  // We'll make envDur vary between 0.35 * (1..0.6) for higher pressure shorten
  float fsrNorm = (float)constrain(fsr, 0, 4095) / 4095.0f;
  // invert so higher pressure shortens
  float shorten = 1.0f - (0.5f * fsrNorm);                                 // range ~1.0 -> 0.5
  float newEnvDur = ganGan.isActive() ? (ganGan.isActive(), 0.01f) : 0.0f; // avoid unused warning
  // We use base + factor so minimal env >= 0.06s
  newEnvDur = max(0.06f, ganGan.isActive() ? 0.01f : 0.0f); // placeholder to avoid odd edge cases
  // Better approach: use envDurBase and scale
  // Since we didn't expose getter for envDurBase, we can approximate by setting relative to a known base:
  float base = 0.35f;                                  // same as base earlier
  newEnvDur = base * (0.6f + 0.4f * (1.0f - fsrNorm)); // range base*1.0 -> base*0.6
  newEnvDur = constrain(newEnvDur, 0.05f, 1.5f);
  ganGan.setEnvDur(newEnvDur);

  // slight delay (not necessary) - avoid too tight busy loop
  delay(4);
}

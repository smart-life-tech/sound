/*
   Teensy 4.x GAN-GAN SYNTHESIS WITH REAL-TIME SENSOR MODULATION
   - GanGan synth: FM attack + harmonic body + subharmonic jitter
   - Parameters tuned for expressive drum sound
    - Analog Hall effect sensor (A1) → Base frequency modulation (pitch control)
   - FSR sensor (A0) → Output gain (sustain/volume control)
   - Piezo sensor (A2) → Trigger noteOn
*/

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

#define analogReadFast(pin) analogRead(pin)

// ------------------- PINS -------------------
#define PIEZO_CENTER_PIN A2
#define HALL_PITCH_PIN A1
#define FSR_PIN A0

// ------------------- CONFIG -------------------
#define PIEZO_THRESHOLD 200
#define PIEZO_DEBOUNCE_MS 1

#define HALL_MIN_ADC 0.0f
#define HALL_MAX_ADC 1023.0f
#define HALL_SMOOTH_ALPHA 0.20f
#define HALL_FREQ_MIN_HZ 70.0f
#define HALL_FREQ_MAX_HZ 250.0f

#define REEXCITE_WINDOW_MS 1200
#define REEXCITE_COOLDOWN_MS 110
#define HALL_REEXCITE_DELTA_THRESHOLD 12.0f
#define HALL_REEXCITE_DELTA_MAX 180.0f

// =============== GANGAN SYNTH CLASS ===============
class AudioSynthGanGan : public AudioStream
{
public:
  AudioSynthGanGan() : AudioStream(0, NULL)
  {
    aStartFreq = 8.0f;
    aStopFreq = 3.0f;
    aStartAmp = 2.0f;
    aStopAmp = 0.2f;
    aDuration = 0.3f;
    aKnee = 1;

    bFreq = 200.0f;
    bSubFreq = 30.0f;
    bAmp = 1.0f;
    bSubStd = 0.3f;

    envDurBase = 0.32f;
    envDur = envDurBase;
    envAttack = 0.001f;
    outputGain = 0.4f;
    fadeTime = 0.5f;

    phaseA = phaseB = phaseSub = 0.0f;
    sweepT = 1.0f;
    envT = 1.0f;
    active = false;

    sampleRate = AUDIO_SAMPLE_RATE_EXACT;
    twoPI = 2.0f * M_PI;

    fmDepthScale = 0.65f;
    sweepSpreadScale = 0.60f;
  }

  virtual void update(void) override
  {
    audio_block_t *block = allocate();
    if (!block)
      return;
    float invSR = 1.0f / sampleRate;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
      float out = 0.0f;
      if (active)
      {
        sweepT += 1.0f / (aDuration * sampleRate);
        if (sweepT > 1.0f)
          sweepT = 1.0f;
        float k = sweepT;
        if (aKnee == 1)
          k = sweepT * sweepT;
        else if (aKnee == 2)
          k = sqrtf(sweepT);
        float fA = aStartFreq + (aStopFreq - aStartFreq) * (k * sweepSpreadScale);
        float aA = aStartAmp + (aStopAmp - aStartAmp) * k;

        phaseA += fA * invSR;
        if (phaseA >= 1.0f)
          phaseA -= floorf(phaseA);
        float modA = sinf(twoPI * phaseA) * (aA * fmDepthScale);

        phaseB += bFreq * invSR;
        if (phaseB >= 1.0f)
          phaseB -= floorf(phaseB);
        phaseSub += bSubFreq * invSR;
        if (phaseSub >= 1.0f)
          phaseSub -= floorf(phaseSub);
        float jitter = sinf(twoPI * phaseSub) * bSubStd;
        float toneB = sinf(twoPI * phaseB) * (bAmp + jitter);

        envT += invSR;
        float envNorm = envT / envDur;
        if (envNorm > 1.0f)
          envNorm = 1.0f;
        float env;
        float atkNorm = envAttack / envDur;
        if (envNorm < atkNorm)
          env = envNorm / atkNorm;
        else
        {
          float t = (envNorm - atkNorm) / (1.0f - atkNorm);
          env = expf(-5.0f * t);
        }
        float decayRate = 1.0f / fadeTime;
        float fadeFactor = expf(-decayRate * envT);
        out = (1.0f + modA) * toneB * env * outputGain * fadeFactor;
        if (sweepT >= 1.0f && envNorm >= 1.0f)
          active = false;
      }
      if (out > 1.0f)
        out = 1.0f;
      if (out < -1.0f)
        out = -1.0f;
      block->data[i] = (int16_t)(out * 32767.0f);
    }
    transmit(block);
    release(block);
  }

  void setBFreq(float v) { bFreq = v; }
  void setEnvDur(float v) { envDur = max(0.1f, v); }
  void setOutputGain(float v) { outputGain = constrain(v, 0.0f, 1.0f); }
  void setFadeTime(float v) { fadeTime = max(0.01f, v); }
  void noteOn(float velocity = 1.0f)
  {
    sweepT = 0.0f;
    envT = 0.0f;
    phaseB += random(0, 100) * 1e-5f;
    outputGain = velocity;
    active = true;
  }
  void reExcite(float amount)
  {
    if (!active)
      return;

    float intensity = constrain(amount, 0.0f, 1.0f);
    outputGain = constrain(outputGain + 0.18f * intensity, 0.0f, 1.0f);
    envT = max(0.0f, envT - (0.012f + 0.028f * intensity));
    sweepT = max(0.0f, sweepT - (0.012f + 0.018f * intensity));
  }
  bool isActive() const { return active; }

private:
  float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
  uint8_t aKnee;
  float bFreq, bSubFreq, bAmp, bSubStd;
  float envDur, envAttack, outputGain, fadeTime;
  float phaseA, phaseB, phaseSub;
  float sweepT, envT;
  bool active;
  float sampleRate, twoPI;
  float fmDepthScale, sweepSpreadScale;

public:
  float envDurBase;
};

// =============== AUDIO OBJECTS ===============
AudioSynthGanGan ganGan;
AudioOutputI2S i2s1;
AudioControlSGTL5000 audioShield;

// Routing: ganGan → I2S out
AudioConnection patchCord1(ganGan, 0, i2s1, 0);
AudioConnection patchCord2(ganGan, 0, i2s1, 1);

// =============== GLOBAL STATE ===============
volatile uint16_t piezoCenterPeak = 0;
volatile uint32_t lastHitMs = 0;
volatile float smoothedHall = 0.0f;
volatile bool inPeakState = false;
volatile uint16_t currentFSR = 0;
// =============== ISR: PIEZO EDGE DETECTION & GANGAN TRIGGER ===============
void piezoISR()
{
    uint32_t now = millis();
    uint16_t piezoRaw = analogReadFast(PIEZO_CENTER_PIN);
    uint16_t fsrRaw   = analogReadFast(FSR_PIN);
    float    hallRaw  = analogReadFast(HALL_PITCH_PIN);

    // Update shared state
    piezoCenterPeak = piezoRaw;
    currentFSR      = fsrRaw;

    // Hall smoothing for stable pitch modulation.
    smoothedHall = (1.0f - HALL_SMOOTH_ALPHA) * smoothedHall + HALL_SMOOTH_ALPHA * hallRaw;

    // ----------------------------------------------------------------------
    // NEW: PROPER DRUM TRIGGER LOGIC
    // ----------------------------------------------------------------------

    static bool armed = true;
    static uint32_t lastHitTime = 0;

    const int LOWER_RESET = PIEZO_THRESHOLD * 0.3f; // quiet zone
    const int LOCKOUT_MS  = 40;                     // ignore bounce/release

    // 1. Lockout after hit (prevents bounce and release retriggers)
    if (now - lastHitTime < LOCKOUT_MS) {
        armed = false;
    }

    // 2. Re-arm only when piezo is fully quiet (below lower threshold)
    if (!armed) {
        if (piezoRaw < LOWER_RESET) {
            armed = true;
        } else {
            return; // still not quiet → ignore noise
        }
    }

    // 3. Detect valid rising hit
    if (armed && piezoRaw >= PIEZO_THRESHOLD) {

        lastHitTime = now;
        armed = false;

        // Velocity smoothing
        static float piezoSmooth = 0.0f;
        const float PIEZO_ALPHA = 0.1f;

        piezoSmooth = (1.0f - PIEZO_ALPHA) * piezoSmooth + PIEZO_ALPHA * piezoRaw;

        float velocity = (piezoSmooth - PIEZO_THRESHOLD) / (700.0f - PIEZO_THRESHOLD);
        velocity = constrain(velocity, 0.75f, 1.0f);

        ganGan.noteOn(velocity);
        lastHitMs = now;
        delayMicroseconds(30); // debounce
        Serial.printf(">>> HIT   vel: %.2f   piezo: %d   hall:%.0f\n", velocity, piezoRaw, smoothedHall);
        return;
    }

    // no hit, no action
}


// =============== SETUP ===============
void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Teensy 4.x GAN-GAN SYNTH WITH SENSOR MODULATION ===");

  // Audio setup
  AudioMemory(12);
  audioShield.enable();
  audioShield.volume(0.7);

  // GanGan initialization with provided parameters
  ganGan.setBFreq(46.0f);     // base frequency (body)
  ganGan.setEnvDur(0.62f);    // envelope duration (s)
  ganGan.setOutputGain(0.4f); // output gain
  ganGan.setFadeTime(0.5f);   // default fade time (FSR will override)

  // Sensor pins
  pinMode(PIEZO_CENTER_PIN, INPUT);
  pinMode(HALL_PITCH_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);

  // Init state
  piezoCenterPeak = 0;
  lastHitMs = 0;
  smoothedHall = analogReadFast(HALL_PITCH_PIN);
  currentFSR = 0;
  inPeakState = false;

  Serial.println("GAN-GAN Setup complete. Ready for hits!");
  Serial.println("Analog Hall → Base Freq | FSR → Output Gain");
}

// =============== MAIN LOOP: SENSOR POLLING & REAL-TIME MODULATION ===============
void loop()
{
  // Poll sensors continuously
  piezoISR();

  static float prevHall = 0.0f;
  static uint32_t lastReexciteMs = 0;

  const float hallDelta = fabsf(smoothedHall - prevHall);
  prevHall = smoothedHall;

  // 1. Analog Hall → GanGan Base Frequency (bFreq)
  // Map smoothed Hall ADC value to continuous pitch range.
  float hallNorm = (smoothedHall - HALL_MIN_ADC) / (HALL_MAX_ADC - HALL_MIN_ADC);
  hallNorm = constrain(hallNorm, 0.0f, 1.0f);
  float hallFreq = HALL_FREQ_MIN_HZ + (HALL_FREQ_MAX_HZ - HALL_FREQ_MIN_HZ) * hallNorm;
  ganGan.setBFreq(hallFreq);

  // 1b. Controlled re-excitation: fast Hall movement during active tail gives a small secondary boost.
  uint32_t nowMs = millis();
  bool inReexciteWindow = (lastHitMs > 0) && ((nowMs - lastHitMs) <= REEXCITE_WINDOW_MS);
  bool cooldownElapsed = (nowMs - lastReexciteMs) >= REEXCITE_COOLDOWN_MS;
  bool hallMovedFast = hallDelta >= HALL_REEXCITE_DELTA_THRESHOLD;

  if (ganGan.isActive() && inReexciteWindow && cooldownElapsed && hallMovedFast)
  {
    float exciteAmount = (hallDelta - HALL_REEXCITE_DELTA_THRESHOLD) /
                         (HALL_REEXCITE_DELTA_MAX - HALL_REEXCITE_DELTA_THRESHOLD);
    exciteAmount = constrain(exciteAmount, 0.0f, 1.0f);
    ganGan.reExcite(exciteAmount);
    lastReexciteMs = nowMs;
  }

  // 2. FSR → Damping (controls fade-out time after hit)
  // Map FSR [0, 1023] → fade time [0.1s, 2.0s]
  // No pressure = quick decay (0.1s), hard squeeze = long sustain (2.0s)
  float fsrNorm = currentFSR / 1023.0f;
  float fadeTime = 0.1f + (1.9f * fsrNorm);
  ganGan.setFadeTime(fadeTime);

  // Debug output (500ms)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500)
  {
    lastDebugMs = millis();
    Serial.printf("Piezo: %4d | Hall: %4.0f (freq: %6.1f Hz) | FSR: %4d (fade: %.2fs) | Active: %d\n",
            piezoCenterPeak, smoothedHall, hallFreq, currentFSR, fadeTime, ganGan.isActive());
  }

  // delay(2);
}

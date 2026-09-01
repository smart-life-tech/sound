/*
   Teensy 4.x GAN-GAN SYNTHESIS WITH REAL-TIME SENSOR MODULATION
   - GanGan synth: FM attack + harmonic body + subharmonic jitter
   - Parameters tuned for expressive drum sound
    - Analog Hall effect sensor (A1) → Base frequency modulation (pitch control)
   - FSR sensor (A0) → sustain (fade time), with hysteresis mute/cutoff
   - Volume pot (A3) → master volume
   - Treble pot (A4) → brightness (filter cutoff), decoupled from pitch
   - Piezo sensor (A2) → Trigger noteOn (peak-hold velocity capture)
    LATEST working version
*/

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

#define analogReadFast(pin) analogRead(pin)

// ------------------- PINS -------------------
// GPIO usage note:
// - A0 = FSR
// - A1 = Hall / pitch input
// - A2 = piezo hit sensor
// - A3/A4/A5 = analog pots
// - ADXL334/ADXL345 testing must use ONE ADXL on ONE ADC input at a time.
//   The software selector only changes sensitivity calibration; it does NOT change
//   the physical GPIO mapping. Same model on the same GPIO is fine, but do not wire
//   two ADXL parts to the same ADC channel at once.
#define PIEZO_CENTER_PIN A2
#define HALL_PITCH_PIN A1
#define FSR_PIN A0
#define VOLUME_POT_PIN A3
#define TREBLE_POT A4
#define BASS_POT A5
#define ADXL_TEST_PIN A6
// ------------------- CONFIG -------------------
#define PIEZO_THRESHOLD 190
#define PIEZO_DEBOUNCE_MS 1

#define HALL_MIN_ADC 516.0f
#define HALL_MAX_ADC 535.0f
#define HALL_QUANTIZE_THRESHOLD 0.18f // when to snap to discrete steps (0..1 normalized)
#define HALL_QUANTIZE_STEPS 7         // number of intervals (7 -> 8 notes: do..do)
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
    masterVolume = 1.0f;
    fadeTime = 0.5f;

    phaseA = phaseB = phaseSub = 0.0f;
    sweepT = 1.0f;
    envT = 1.0f;
    active = false;

    slapEnv = 0.0f;
    clickEnv = 0.0f;
    enableHighMid = false;
    enableFastHighDecay = false;
    enableSlap = false;
    enableClick = false;
    enableTanh = false;

    sampleRate = AUDIO_SAMPLE_RATE_EXACT;
    twoPI = 2.0f * M_PI;

    fmDepthScale = 0.65f;
    sweepSpreadScale = 0.60f;

    lpState = 0.0f;
    lpCoeff = 1.0f; // pass-through until setBrightness() is called
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
        float tone = toneB;

        if (enableHighMid)
        {
          float highEnv = 1.0f;
          if (enableFastHighDecay)
          {
            highEnv = expf(-25.0f * envT);
          }
          float toneH1 = 0.10f * sinf(twoPI * phaseB * 8.0f) * highEnv;
          float toneH2 = 0.05f * sinf(twoPI * phaseB * 12.0f) * highEnv;
          tone += toneH1 + toneH2;
        }

        if (enableSlap || enableClick)
        {
          float noise = ((float)random(-32768, 32767)) * (1.0f / 32768.0f);
          if (enableSlap)
          {
            tone += noise * 0.08f * slapEnv;
          }
          if (enableClick)
          {
            tone += noise * clickEnv * 0.12f;
          }
          slapEnv *= 0.995f;
          clickEnv *= 0.90f;
        }

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
        out = (1.0f + modA) * tone * env * outputGain * fadeFactor;
        if (sweepT >= 1.0f && envNorm >= 1.0f)
          active = false;
      }
      if (enableTanh)
      {
        out = tanhf(out * 1.4f);
      }
      else
      {
        if (out > 1.0f)
          out = 1.0f;
        if (out < -1.0f)
          out = -1.0f;
      }
      // Brightness: one-pole low-pass, cutoff independent of bFreq (pitch).
      lpState += lpCoeff * (out - lpState);
      out = lpState;
      out *= masterVolume;
      block->data[i] = (int16_t)(out * 32767.0f);
    }
    transmit(block);
    release(block);
  }

  void setBFreq(float v) { bFreq = v; }
  // Brightness (0..1) maps to a fixed cutoff-frequency range, decoupled from pitch (bFreq).
  void setBrightness(float norm)
  {
    norm = constrain(norm, 0.0f, 1.0f);
    float cutoffHz = 800.0f + norm * (8000.0f - 800.0f);
    lpCoeff = 1.0f - expf(-twoPI * cutoffHz / sampleRate);
  }
  void setEnvDur(float v) { envDur = max(0.1f, v); }
  void setOutputGain(float v) { outputGain = constrain(v, 0.0f, 1.0f); }
  void setMasterVolume(float v) { masterVolume = constrain(v, 0.0f, 1.0f); }
  void setFadeTime(float v) { fadeTime = max(0.01f, v); }
  void setHighMid(bool v) { enableHighMid = v; }
  void setFastHighDecay(bool v) { enableFastHighDecay = v; }
  void setSlap(bool v) { enableSlap = v; }
  void setClick(bool v) { enableClick = v; }
  void setTanh(bool v) { enableTanh = v; }
  bool isHighMidEnabled() const { return enableHighMid; }
  bool isFastHighDecayEnabled() const { return enableFastHighDecay; }
  bool isSlapEnabled() const { return enableSlap; }
  bool isClickEnabled() const { return enableClick; }
  bool isTanhEnabled() const { return enableTanh; }
  void noteOn(float velocity = 1.0f)
  {
    sweepT = 0.0f;
    envT = 0.0f;
    phaseB += random(0, 100) * 1e-5f;
    outputGain = velocity;
    slapEnv = 1.0f;
    clickEnv = 1.0f;
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
  float slapEnv, clickEnv;
  bool active;
  float sampleRate, twoPI;
  float fmDepthScale, sweepSpreadScale;
  float masterVolume;
  bool enableHighMid, enableFastHighDecay, enableSlap, enableClick, enableTanh;
  float lpState, lpCoeff;

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

// ADXL test toggle: software-only intensity calibration selector.
// Same GPIO can be reused for the same model, but the physical input still stays fixed.
enum class AdxlModel
{
  ADXL334,
  ADXL345
};

bool gHitTestEnabled = true;
AdxlModel gSelectedAdxlModel = AdxlModel::ADXL345;

const char *adxlModelName(AdxlModel model)
{
  switch (model)
  {
    case AdxlModel::ADXL334:
      return "ADXL334";
    case AdxlModel::ADXL345:
    default:
      return "ADXL345";
  }
}

float adxlSensitivityScale(AdxlModel model)
{
  // ADXL345 is typically more sensitive; use a slight calibration offset for debug output
  return (model == AdxlModel::ADXL334) ? 1.00f : 1.30f;
}

float audioHitIntensity(uint16_t peak, uint16_t fsr)
{
  float vel = constrain((float)(peak - PIEZO_THRESHOLD) / (float)(1023 - PIEZO_THRESHOLD), 0.0f, 1.0f);
  float fsrNorm = constrain((float)fsr / 1023.0f, 0.0f, 1.0f);
  float level = (vel * 0.8f) + (fsrNorm * 0.2f);
  return constrain(level * adxlSensitivityScale(gSelectedAdxlModel), 0.0f, 1.0f);
}

void handleSerialHitTestCommand()
{
  if (Serial.available() == 0)
    return;

  char cmd = Serial.read();
  while (Serial.available())
    Serial.read();

  switch (cmd)
  {
    case 'h':
    case 'H':
      gHitTestEnabled = !gHitTestEnabled;
      Serial.print("Hit test: ");
      Serial.println(gHitTestEnabled ? "ON" : "OFF");
      break;

    case '3':
      gSelectedAdxlModel = AdxlModel::ADXL334;
      Serial.println("ADXL model set to ADXL334");
      break;

    case '4':
      gSelectedAdxlModel = AdxlModel::ADXL345;
      Serial.println("ADXL model set to ADXL345");
      break;
  }
}

// =============== CONTINUOUS PLAYBACK ===============
bool continuousPlayback = false;
uint32_t lastContinuousTriggerMs = 0;
const uint32_t CONTINUOUS_TRIGGER_INTERVAL_MS = 400; // trigger every ~600ms
// =============== ISR: PIEZO EDGE DETECTION & GANGAN TRIGGER ===============
void piezoISR()
{
  uint32_t now = millis();
  uint16_t piezoRaw = analogReadFast(PIEZO_CENTER_PIN);
  uint16_t fsrRaw = analogReadFast(FSR_PIN);
  float hallRaw = analogReadFast(HALL_PITCH_PIN);

  // Update shared state
  piezoCenterPeak = piezoRaw;
  currentFSR = fsrRaw;

  // Hall smoothing for stable pitch modulation.
  smoothedHall = (1.0f - HALL_SMOOTH_ALPHA) * smoothedHall + HALL_SMOOTH_ALPHA * hallRaw;

  // If continuous playback is enabled, skip hit detection
  if (continuousPlayback)
    return;

  // ----------------------------------------------------------------------
  // NEW: PROPER DRUM TRIGGER LOGIC
  // ----------------------------------------------------------------------

  static bool armed = true;
  static uint32_t lastHitTime = 0;

  const int LOWER_RESET = PIEZO_THRESHOLD * 0.3f; // quiet zone
  const int LOCKOUT_MS = 40;                      // ignore bounce/release

  // 1. Lockout after hit (prevents bounce and release retriggers)
  if (now - lastHitTime < LOCKOUT_MS)
  {
    armed = false;
  }

  // 2. Re-arm only when piezo is fully quiet (below lower threshold)
  if (!armed)
  {
    if (piezoRaw < LOWER_RESET)
    {
      armed = true;
    }
    else
    {
      return; // still not quiet → ignore noise
    }
  }

  // 3. Detect valid rising hit
  if (armed && piezoRaw >= PIEZO_THRESHOLD)
  {

    lastHitTime = now;
    armed = false;

    // Peak-hold capture: sample piezo for a short window to catch the true
    // strike peak instead of a single instantaneous ADC read (see
    // docs/SENSOR_AND_SOUND_RESEARCH_NOTES.md §1).
    uint16_t peak = piezoRaw;
    uint32_t captureStart = micros();
    const uint32_t PEAK_CAPTURE_US = 1500; // ~1.5ms peak-hold window
    while (micros() - captureStart < PEAK_CAPTURE_US)
    {
      uint16_t sample = analogReadFast(PIEZO_CENTER_PIN);
      if (sample > peak)
        peak = sample;
    }
    piezoCenterPeak = peak;

    // Velocity now scales across the full peak range instead of being
    // clamped to a narrow band, so hit amplitude actually varies.
    float velocity = (float)(peak - PIEZO_THRESHOLD) / (float)(1023 - PIEZO_THRESHOLD);
    velocity = constrain(velocity, 0.0f, 1.0f);
    velocity = 0.15f + velocity * 0.85f; // audible floor, full dynamic range above it

    float hitIntensity = audioHitIntensity(peak, fsrRaw);
    if (gHitTestEnabled)
    {
      Serial.printf("[HIT_TEST] model=%s peak=%u vel=%.3f intensity=%.3f fsr=%u hall=%.0f\n",
                    adxlModelName(gSelectedAdxlModel),
                    peak,
                    velocity,
                    hitIntensity,
                    fsrRaw,
                    smoothedHall);
    }

    ganGan.noteOn(velocity);
    lastHitMs = now;
    delayMicroseconds(30); // debounce
    Serial.printf(">>> HIT   vel: %.2f   peak: %d   piezo: %d   hall:%.0f\n", velocity, peak, piezoRaw, smoothedHall);
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
  audioShield.volume(1.0);

  // GanGan initialization with provided parameters
  ganGan.setBFreq(46.0f);     // base frequency (body)
  ganGan.setEnvDur(0.62f);    // envelope duration (s)
  ganGan.setOutputGain(0.4f); // output gain
  ganGan.setFadeTime(0.5f);   // default fade time (FSR will override)

  // Sensor pins
  pinMode(PIEZO_CENTER_PIN, INPUT);
  pinMode(HALL_PITCH_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  pinMode(VOLUME_POT_PIN, INPUT);
  pinMode(TREBLE_POT, INPUT);
  pinMode(BASS_POT, INPUT);

  // Init state
  piezoCenterPeak = 0;
  lastHitMs = 0;
  smoothedHall = analogReadFast(HALL_PITCH_PIN);
  currentFSR = 0;
  inPeakState = false;

  Serial.println("GAN-GAN Setup complete. Ready for hits!");
  Serial.println("GPIO map: A0=FSR, A1=Hall, A2=Piezo, A3/A4/A5=Pots.");
  Serial.println("ADXL GPIO note: one ADXL on one ADC pin at a time; software selector 3/4 only changes calibration, not the physical pin.");
  Serial.println("Analog Hall → Base Freq | FSR → Output Gain");
  Serial.println("Press 'c' to toggle continuous playback mode");
  Serial.println("Press 'h' to toggle hit test, '3' ADXL334, '4' ADXL345, '1'=HighMid 2=FastHighDecay 3=Slap 4=Click 5=Tanh");
}

// =============== MAIN LOOP: SENSOR POLLING & REAL-TIME MODULATION ===============
void loop()
{
  // Poll sensors continuously
  piezoISR();
  uint16_t rawHall = analogReadFast(HALL_PITCH_PIN);

  static float prevHall = 0.0f;
  static uint32_t lastReexciteMs = 0;

  const float hallDelta = fabsf((float)rawHall - prevHall);
  prevHall = (float)rawHall;

  // 1. Analog Hall → GanGan pitch range
  // Use smoothed Hall reading for stable transitions. Blend between base and treble ranges.
  float hallVal = smoothedHall; // smoothed by ISR
  float hallNorm = ((float)hallVal - HALL_MIN_ADC) / (HALL_MAX_ADC - HALL_MIN_ADC);
  hallNorm = constrain(hallNorm, 0.0f, 1.0f);

  uint16_t bassRaw = analogReadFast(BASS_POT);
  float bassNorm = constrain((float)bassRaw / 1023.0f, 0.0f, 1.0f);
  float baseFreq = 70.0f + (110.0f - 70.0f) * bassNorm;
  baseFreq = constrain(baseFreq, 70.0f, 110.0f);

  // Treble pot now drives brightness (spectral tone) only, fully decoupled
  // from pitch (see docs/SENSOR_AND_SOUND_RESEARCH_NOTES.md §3). It no longer
  // contributes to the Hall pitch range.
  uint16_t trebleRaw = analogReadFast(TREBLE_POT);
  float trebleNorm = constrain((float)trebleRaw / 1023.0f, 0.0f, 1.0f);
  ganGan.setBrightness(trebleNorm);

  // If magnet is near (hallNorm somewhat high), optionally quantize into discrete steps
  float hallFreq;
  if (hallNorm >= HALL_QUANTIZE_THRESHOLD)
  {
    float stepsF = (float)HALL_QUANTIZE_STEPS;
    float s = roundf(hallNorm * stepsF);
    float qNorm = s / stepsF;
    hallFreq = baseFreq + qNorm * (HALL_FREQ_MAX_HZ - baseFreq);
  }
  else
  {
    hallFreq = baseFreq + hallNorm * (HALL_FREQ_MAX_HZ - baseFreq);
  }
  hallFreq = constrain(hallFreq, 20.0f, 20000.0f);
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

  // 2. FSR → sustain (continuous fade time) with a hysteresis-based mute/cutoff
  // on top, so the cutoff feels sharp and repeatable regardless of the FSR's
  // analog non-linearity (see docs/SENSOR_AND_SOUND_RESEARCH_NOTES.md §2).
  float fsrNorm = currentFSR / 1023.0f;
  static bool fsrMuted = false;
  const float FSR_MUTE_LOW = 0.05f;  // drop below this to engage mute
  const float FSR_MUTE_HIGH = 0.10f; // rise above this to release mute
  if (fsrNorm <= FSR_MUTE_LOW)
    fsrMuted = true;
  else if (fsrNorm >= FSR_MUTE_HIGH)
    fsrMuted = false;

  float fadeTime = fsrMuted ? 0.03f : (0.1f + (1.9f * fsrNorm));
  ganGan.setFadeTime(fadeTime);

  // 2b. Volume pot → master volume
  uint16_t volumeRaw = analogReadFast(VOLUME_POT_PIN);
  float volumeNorm = constrain((float)volumeRaw / 1023.0f, 0.0f, 1.0f);
  ganGan.setMasterVolume(volumeNorm);

  // Debug output (500ms)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500)
  {
    lastDebugMs = millis();
    Serial.printf("Piezo: %4u | rawHall: %4u | smoothHall: %6.1f | selectedFreq: %6.1f Hz | Bass: %4u | base: %4.1f Hz | Treble: %4u | brightness: %.2f | FSR: %4u | fade: %.2fs | muted: %u | Vol: %.2f | Active: %u\n",
                  piezoCenterPeak, rawHall, smoothedHall, hallFreq, bassRaw, baseFreq, trebleRaw, trebleNorm, currentFSR, fadeTime, fsrMuted ? 1u : 0u, volumeNorm, ganGan.isActive() ? 1u : 0u);
  }

  // 3. CONTINUOUS PLAYBACK MODE
  // When enabled, trigger notes at steady intervals, bypassing piezo detection
  if (continuousPlayback)
  {
    uint32_t nowMs = millis();
    if (nowMs - lastContinuousTriggerMs >= CONTINUOUS_TRIGGER_INTERVAL_MS)
    {
      lastContinuousTriggerMs = nowMs;

      // Use FSR for velocity control in continuous mode
      float velocity = constrain(currentFSR / 1023.0f, 0.8f, 1.0f);
      ganGan.noteOn(velocity);

      Serial.printf(">>> CONTINUOUS TRIGGER   vel: %.2f   hall:%.0f Hz   fsr:%d\n",
                    velocity, hallFreq, currentFSR);
    }
  }

  // 4. SERIAL COMMANDS
  handleSerialHitTestCommand();
  if (Serial.available())
  {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C')
    {
      continuousPlayback = !continuousPlayback;
      Serial.printf("Continuous playback: %s\n", continuousPlayback ? "ON" : "OFF");
      if (continuousPlayback)
        Serial.println("  (FSR controls velocity, Hall controls pitch)");
    }
    else if (cmd == 'h' || cmd == 'H')
    {
      gHitTestEnabled = !gHitTestEnabled;
      Serial.printf("Hit test: %s\n", gHitTestEnabled ? "ON" : "OFF");
    }
    else if (cmd == '3')
    {
      gSelectedAdxlModel = AdxlModel::ADXL334;
      Serial.println("ADXL model set to ADXL334");
    }
    else if (cmd == '4')
    {
      gSelectedAdxlModel = AdxlModel::ADXL345;
      Serial.println("ADXL model set to ADXL345");
    }
    else if (cmd == '1')
    {
      ganGan.setHighMid(!ganGan.isHighMidEnabled());
      Serial.printf("High-mid resonance: %s\n", ganGan.isHighMidEnabled() ? "ON" : "OFF");
    }
    else if (cmd == '2')
    {
      ganGan.setFastHighDecay(!ganGan.isFastHighDecayEnabled());
      Serial.printf("Fast high decay: %s\n", ganGan.isFastHighDecayEnabled() ? "ON" : "OFF");
    }
    else if (cmd == '5')
    {
      ganGan.setTanh(!ganGan.isTanhEnabled());
      Serial.printf("Tanh saturation: %s\n", ganGan.isTanhEnabled() ? "ON" : "OFF");
    }
  }

  // delay(2);
}

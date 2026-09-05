/*
   Teensy 4.x GAN-GAN SYNTHESIS WITH REAL-TIME SENSOR MODULATION
   - GanGan synth: FM attack + harmonic body + subharmonic jitter
   - Parameters tuned for expressive drum sound
   - Analog Hall effect sensor (A1) → Base frequency modulation (pitch control)
   - FSR sensor (A0) → sustain (fade time), with hysteresis mute/cutoff
   - Volume pot (A3) → master volume
   - Treble pot (A4) → brightness (filter cutoff), decoupled from pitch
   - Piezo sensor (A2) → Trigger noteOn (peak-hold velocity capture)
   - ADXL345 accelerometer (I2C or SPI) → Optional hit detection
   
   ADXL345 HARDWARE PINS (Teensy 4.x):
   ====================================
   I2C Mode (default):
     SDA (Data)  = Pin 18 (Wire.h default)
     SCL (Clock) = Pin 19 (Wire.h default)
     SD0 (Addr)  = GND for 0x53, VCC for 0x1D
     INT1        = Optional interrupt output
     INT2        = Optional interrupt output
   
   SPI Mode:
     MOSI        = Pin 11 (SPI.h default)
     MISO        = Pin 12 (SPI.h default)
     SCL (Clock) = Pin 13 (SPI.h default)
     CS          = Pin 10 (selectable)
     INT1        = Optional interrupt output
     INT2        = Optional interrupt output
*/

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

static uint16_t readAnalogSettled(uint8_t pin)
{
  analogRead(pin);
  delayMicroseconds(500);
  return analogRead(pin);
}

#define analogReadFast(pin) analogRead(pin)

// =============== ADXL345 CONFIGURATION ===============
// Choose ONE protocol at compile time:
#define ADXL_USE_I2C      1    // Set to 1 for I2C, 0 for SPI
#define ADXL_USE_SPI      0    // Set to 1 for SPI, 0 for I2C (mutually exclusive)

// Choose ONE integration mode at compile time:
#define ADXL_DUAL_INPUT   0    // Set to 1 to use piezo AND ADXL345, 0 for ADXL345 only
#define ADXL_ENABLED      1    // Set to 1 to enable ADXL345, 0 to disable entirely

// ADXL345 I2C Address:
// SD0 pin connected to GND = 0x53 (default)
// SD0 pin connected to VCC = 0x1D (alternate)
#define ADXL_I2C_ADDR_LOW    0x53
#define ADXL_I2C_ADDR_HIGH   0x1D
#define ADXL_I2C_ADDRESS     ADXL_I2C_ADDR_LOW

// ADXL345 SPI Chip Select Pin (if using SPI):
#define ADXL_CS_PIN          10

// ADXL345 Register Map:
#define ADXL_REG_DEVID       0x00
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_BW_RATE     0x2C
#define ADXL_REG_FIFO_CTL    0x38
#define ADXL_REG_X0          0x32  // X LSB
#define ADXL_REG_X1          0x33  // X MSB
#define ADXL_REG_Y0          0x34  // Y LSB
#define ADXL_REG_Y1          0x35  // Y MSB
#define ADXL_REG_Z0          0x36  // Z LSB
#define ADXL_REG_Z1          0x37  // Z MSB
#define ADXL_REG_INT_ENABLE  0x2E
#define ADXL_REG_INT_SOURCE  0x30
#define ADXL_REG_INT_MAP     0x2F

// ADXL345 Configuration values:
#define ADXL_POWER_CTL_MEASURE 0x08  // Enable measurement mode (bit 3)
#define ADXL_DATA_FORMAT_FULL  0x0B  // 13-bit full resolution, +/-16g range
#define ADXL_RATE_200HZ        0x0C

// ------------------- PINS -------------------
// Analog sensors (ADC):
#define PIEZO_CENTER_PIN A2
#define HALL_PITCH_PIN A1
#define FSR_PIN A0
#define VOLUME_POT_PIN A3
#define TREBLE_POT A14
#define BASS_POT A13

// I2C (Wire.h default):
//   SDA = Pin 18
//   SCL = Pin 19

// SPI (SPI.h default):
//   MOSI = Pin 11
//   MISO = Pin 12
//   SCL  = Pin 13
//   CS   = Pin 10 (ADXL_CS_PIN)

// ------------------- CONFIG -------------------
#define PIEZO_THRESHOLD 190
#define PIEZO_DEBOUNCE_MS 1

#define HALL_MIN_ADC 516.0f
#define HALL_MAX_ADC 535.0f
#define HALL_QUANTIZE_THRESHOLD 0.18f
#define HALL_QUANTIZE_STEPS 7
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
    lpCoeff = 1.0f;
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
      lpState += lpCoeff * (out - lpState);
      out = lpState;
      out *= masterVolume;
      block->data[i] = (int16_t)(out * 32767.0f);
    }
    transmit(block);
    release(block);
  }

  void setBFreq(float v) { bFreq = v; }
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

// ADXL345 state:
volatile int16_t adxl_x = 0, adxl_y = 0, adxl_z = 0;
volatile float adxl_magnitude = 0.0f;  // acceleration magnitude (g's)
volatile float adxlRawGX = 0.0f;
volatile float adxlRawGY = 0.0f;
volatile float adxlRawGZ = 0.0f;

// ADXL model selector: software calibration profile only
enum class AdxlModel
{
  ADXL334,
  ADXL345
};

bool gHitTestEnabled = true;
AdxlModel gSelectedAdxlModel = AdxlModel::ADXL345;
float gAdxlThreshold = 0.35f; // Acceleration threshold (g's) for hit detection
float adxlBaselineX = 0.0f;
float adxlBaselineY = 0.0f;
float adxlBaselineZ = 0.0f;
float adxlFilteredDynamic = 0.0f;
bool adxlBaselineReady = false;

const uint32_t ADXL_SAMPLE_INTERVAL_MS = 5;
const uint32_t ADXL_HIT_COOLDOWN_MS = 80;
const uint32_t ADXL_RECALIBRATION_TIME_MS = 10000;
const float ADXL_FILTER_ALPHA = 0.25f;
const float ADXL_REARM_RATIO = 0.45f;
const float ADXL_HIT_MAX_G = 3.0f;
const float ADXL_STATIONARY_DELTA_G = 0.35f;
const float ADXL_AUTO_ORIENTATION_CHANGE_DEG = 90.0f;
const float ADXL_ORIENTATION_HYSTERESIS_DEG = 5.0f;

uint32_t adxlStableSinceMs = 0;
uint32_t adxlStableSamples = 0;
float adxlStableSumX = 0.0f;
float adxlStableSumY = 0.0f;
float adxlStableSumZ = 0.0f;
float adxlPreviousRawX = 0.0f;
float adxlPreviousRawY = 0.0f;
float adxlPreviousRawZ = 0.0f;
bool adxlPreviousRawReady = false;
bool adxlOrientationPending = false;

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
  // ADXL345 is typically 30% more sensitive; use calibration offset for intensity reporting
  return (model == AdxlModel::ADXL334) ? 1.00f : 1.30f;
}

float audioHitIntensity(uint16_t peak, uint16_t fsr)
{
  float vel = constrain((float)(peak - PIEZO_THRESHOLD) / (float)(1023 - PIEZO_THRESHOLD), 0.0f, 1.0f);
  float fsrNorm = constrain((float)fsr / 1023.0f, 0.0f, 1.0f);
  float level = (vel * 0.8f) + (fsrNorm * 0.2f);
  return constrain(level * adxlSensitivityScale(gSelectedAdxlModel), 0.0f, 1.0f);
}

float adxlHitIntensity(float peakG)
{
  // Linear mapping: threshold is silence and 3 g is full hit amplitude.
  float intensity = (peakG - gAdxlThreshold) / (ADXL_HIT_MAX_G - gAdxlThreshold);
  return constrain(intensity, 0.0f, 1.0f);
}

// =============== ADXL345 I2C/SPI COMMUNICATION ===============
void adxlCalibrateBaseline(uint32_t durationMs);

#if ADXL_USE_I2C

byte adxlReadRegister(byte reg)
{
  Wire.beginTransmission(ADXL_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADXL_I2C_ADDRESS, 1, true);
  return Wire.read();
}

void adxlWriteRegister(byte reg, byte value)
{
  Wire.beginTransmission(ADXL_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void adxlReadAccel()
{
  Wire.beginTransmission(ADXL_I2C_ADDRESS);
  Wire.write(ADXL_REG_X0);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)ADXL_I2C_ADDRESS, 6, true) != 6)
    return;
  
  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);
  
  adxl_x = x;
  adxl_y = y;
  adxl_z = z;
  
  // Convert baseline-relative acceleration to g's (256 LSB/g in full-resolution mode).
  float gx = x / 256.0f;
  float gy = y / 256.0f;
  float gz = z / 256.0f;
  adxlRawGX = gx;
  adxlRawGY = gy;
  adxlRawGZ = gz;
  float dx = gx - adxlBaselineX;
  float dy = gy - adxlBaselineY;
  float dz = gz - adxlBaselineZ;
  float dynamicMagnitude = sqrt(dx * dx + dy * dy + dz * dz);
  adxlFilteredDynamic += ADXL_FILTER_ALPHA * (dynamicMagnitude - adxlFilteredDynamic);
  adxl_magnitude = adxlBaselineReady ? adxlFilteredDynamic : dynamicMagnitude;
}

#endif

#if ADXL_USE_SPI

byte adxlReadRegister(byte reg)
{
  digitalWrite(ADXL_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);  // Set MSB for read
  byte val = SPI.transfer(0x00);
  digitalWrite(ADXL_CS_PIN, HIGH);
  return val;
}

void adxlWriteRegister(byte reg, byte value)
{
  digitalWrite(ADXL_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);  // Clear MSB for write
  SPI.transfer(value);
  digitalWrite(ADXL_CS_PIN, HIGH);
}

void adxlReadAccel()
{
  digitalWrite(ADXL_CS_PIN, LOW);
  SPI.transfer(ADXL_REG_X0 | 0x80);  // Read, burst mode
  
  int16_t x = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  int16_t y = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  int16_t z = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  
  digitalWrite(ADXL_CS_PIN, HIGH);
  
  adxl_x = x;
  adxl_y = y;
  adxl_z = z;
  
  // Compute movement relative to the current stationary orientation.
  float gx = x / 256.0f;
  float gy = y / 256.0f;
  float gz = z / 256.0f;
  adxlRawGX = gx;
  adxlRawGY = gy;
  adxlRawGZ = gz;
  float dx = gx - adxlBaselineX;
  float dy = gy - adxlBaselineY;
  float dz = gz - adxlBaselineZ;
  float dynamicMagnitude = sqrt(dx * dx + dy * dy + dz * dz);
  adxlFilteredDynamic += ADXL_FILTER_ALPHA * (dynamicMagnitude - adxlFilteredDynamic);
  adxl_magnitude = adxlBaselineReady ? adxlFilteredDynamic : dynamicMagnitude;
}

#endif

void adxlInit()
{
  #if ADXL_USE_I2C
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C clock
  #endif
  
  #if ADXL_USE_SPI
  pinMode(ADXL_CS_PIN, OUTPUT);
  digitalWrite(ADXL_CS_PIN, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // ~10.5 MHz
  #endif
  
  delay(10);
  
  // Verify ADXL345 presence
  byte devid = adxlReadRegister(ADXL_REG_DEVID);
  Serial.printf("ADXL345 Device ID: 0x%02X (expected 0xE5)\n", devid);
  
  if (devid != 0xE5)
  {
    Serial.println("WARNING: ADXL345 not detected or communication failed!");
    return;
  }
  
  // Reset to defaults via power cycle
  adxlWriteRegister(ADXL_REG_POWER_CTL, 0x00);
  delay(10);
  
  // Configure data format: 13-bit full resolution, ±16g
  adxlWriteRegister(ADXL_REG_DATA_FORMAT, ADXL_DATA_FORMAT_FULL);
  adxlWriteRegister(ADXL_REG_BW_RATE, ADXL_RATE_200HZ);
  delay(1);
  
  // Enable measurement mode
  adxlWriteRegister(ADXL_REG_POWER_CTL, ADXL_POWER_CTL_MEASURE);
  delay(10);
  
  // Disable all interrupts initially
  adxlWriteRegister(ADXL_REG_INT_ENABLE, 0x00);

  Serial.println("Keep the ADXL345 still: calibrating baseline...");
  adxlCalibrateBaseline(1500);
  Serial.println("ADXL345 initialized successfully.");
}

void adxlCalibrateBaseline(uint32_t durationMs)
{
  int32_t sumX = 0;
  int32_t sumY = 0;
  int32_t sumZ = 0;
  uint16_t sampleCount = 0;
  uint32_t calibrationStart = millis();
  while (millis() - calibrationStart < durationMs)
  {
    adxlReadAccel();
    sumX += adxl_x;
    sumY += adxl_y;
    sumZ += adxl_z;
    ++sampleCount;
    delay(ADXL_SAMPLE_INTERVAL_MS);
  }
  if (sampleCount > 0)
  {
    adxlBaselineX = (float)sumX / (float)sampleCount / 256.0f;
    adxlBaselineY = (float)sumY / (float)sampleCount / 256.0f;
    adxlBaselineZ = (float)sumZ / (float)sampleCount / 256.0f;
    adxlFilteredDynamic = 0.0f;
    adxlBaselineReady = true;
    Serial.printf("ADXL345 baseline calibrated from %u samples.\n", sampleCount);
  }
}

void adxlAutoCalibrateOrientation(uint32_t nowMs)
{
  if (!adxlPreviousRawReady)
  {
    adxlPreviousRawX = adxlRawGX;
    adxlPreviousRawY = adxlRawGY;
    adxlPreviousRawZ = adxlRawGZ;
    adxlPreviousRawReady = true;
    return;
  }

  float dx = adxlRawGX - adxlPreviousRawX;
  float dy = adxlRawGY - adxlPreviousRawY;
  float dz = adxlRawGZ - adxlPreviousRawZ;
  float sampleDelta = sqrt(dx * dx + dy * dy + dz * dz);
  adxlPreviousRawX = adxlRawGX;
  adxlPreviousRawY = adxlRawGY;
  adxlPreviousRawZ = adxlRawGZ;

  float baselineDx = adxlRawGX - adxlBaselineX;
  float baselineDy = adxlRawGY - adxlBaselineY;
  float baselineDz = adxlRawGZ - adxlBaselineZ;
  float orientationChange = sqrt(baselineDx * baselineDx +
                                 baselineDy * baselineDy +
                                 baselineDz * baselineDz);
  float currentMagnitude = sqrt(adxlRawGX * adxlRawGX +
                                adxlRawGY * adxlRawGY +
                                adxlRawGZ * adxlRawGZ);
  float baselineMagnitude = sqrt(adxlBaselineX * adxlBaselineX +
                                 adxlBaselineY * adxlBaselineY +
                                 adxlBaselineZ * adxlBaselineZ);
  float orientationAngleDeg = 0.0f;
  if (adxlBaselineReady && currentMagnitude > 0.01f && baselineMagnitude > 0.01f)
  {
    float cosine = (adxlRawGX * adxlBaselineX +
                    adxlRawGY * adxlBaselineY +
                    adxlRawGZ * adxlBaselineZ) /
                   (currentMagnitude * baselineMagnitude);
    cosine = constrain(cosine, -1.0f, 1.0f);
    orientationAngleDeg = acosf(cosine) * (180.0f / M_PI);
  }

  float requiredOrientationAngle = adxlOrientationPending
                                       ? ADXL_AUTO_ORIENTATION_CHANGE_DEG - ADXL_ORIENTATION_HYSTERESIS_DEG
                                       : ADXL_AUTO_ORIENTATION_CHANGE_DEG;
  bool stable = sampleDelta <= ADXL_STATIONARY_DELTA_G &&
                (!adxlBaselineReady || orientationAngleDeg >= requiredOrientationAngle);

  if (!stable)
  {
    // Movement cancels a candidate orientation and immediately restores hit detection.
    adxlOrientationPending = false;
    adxlStableSinceMs = 0;
    adxlStableSamples = 0;
    adxlStableSumX = 0.0f;
    adxlStableSumY = 0.0f;
    adxlStableSumZ = 0.0f;
    return;
  }

  if (adxlStableSinceMs == 0)
  {
    adxlOrientationPending = true;
    adxlStableSinceMs = nowMs;
    Serial.printf("ADXL345 new stationary orientation detected (angle %.1f deg, change %.2fg); waiting %lu seconds.\n",
            orientationAngleDeg,
            orientationChange,
                  (unsigned long)(ADXL_RECALIBRATION_TIME_MS / 1000));
  }
  adxlStableSumX += adxlRawGX;
  adxlStableSumY += adxlRawGY;
  adxlStableSumZ += adxlRawGZ;
  ++adxlStableSamples;

  if (nowMs - adxlStableSinceMs >= ADXL_RECALIBRATION_TIME_MS && adxlStableSamples > 0)
  {
    uint32_t calibratedSamples = adxlStableSamples;
    adxlBaselineX = adxlStableSumX / adxlStableSamples;
    adxlBaselineY = adxlStableSumY / adxlStableSamples;
    adxlBaselineZ = adxlStableSumZ / adxlStableSamples;
    adxlFilteredDynamic = 0.0f;
    adxl_magnitude = 0.0f;
    adxlOrientationPending = false;
    adxlStableSinceMs = 0;
    adxlStableSamples = 0;
    Serial.printf("ADXL345 auto-calibrated orientation from %lu stable samples.\n",
                  (unsigned long)calibratedSamples);
  }
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

    case 'c':
    case 'C':
      Serial.println("Hold ADXL345 still in the new orientation for 10 seconds...");
      adxlCalibrateBaseline(ADXL_RECALIBRATION_TIME_MS);
      Serial.println("ADXL345 recalibration complete; hits now use this orientation as zero.");
      break;

    case '3':
      gSelectedAdxlModel = AdxlModel::ADXL334;
      Serial.println("ADXL model set to ADXL334");
      break;

    case '4':
      gSelectedAdxlModel = AdxlModel::ADXL345;
      Serial.println("ADXL model set to ADXL345");
      break;
      
    case '+':
      gAdxlThreshold += 0.05f;
      gAdxlThreshold = constrain(gAdxlThreshold, 0.1f, 2.0f);
      Serial.printf("ADXL threshold: %.2f g\n", gAdxlThreshold);
      break;
      
    case '-':
      gAdxlThreshold -= 0.05f;
      gAdxlThreshold = constrain(gAdxlThreshold, 0.1f, 2.0f);
      Serial.printf("ADXL threshold: %.2f g\n", gAdxlThreshold);
      break;
  }
}

// =============== CONTINUOUS PLAYBACK ===============
bool continuousPlayback = false;
uint32_t lastContinuousTriggerMs = 0;
const uint32_t CONTINUOUS_TRIGGER_INTERVAL_MS = 400;

// =============== ISR: PIEZO + ADXL345 EDGE DETECTION & GANGAN TRIGGER ===============
void piezoISR()
{
  uint32_t now = millis();
  uint16_t piezoRaw = analogReadFast(PIEZO_CENTER_PIN);
  uint16_t fsrRaw = readAnalogSettled(FSR_PIN);
  float hallRaw = readAnalogSettled(HALL_PITCH_PIN);

  // Update shared state
  piezoCenterPeak = piezoRaw;
  currentFSR = fsrRaw;

  // Hall smoothing for stable pitch modulation
  smoothedHall = (1.0f - HALL_SMOOTH_ALPHA) * smoothedHall + HALL_SMOOTH_ALPHA * hallRaw;

  static uint32_t lastAdxlSampleMs = 0;

  // Read ADXL345 at the same 200 Hz rate as the validated hit-test sketch.
  #if ADXL_ENABLED
  if (now - lastAdxlSampleMs >= ADXL_SAMPLE_INTERVAL_MS)
  {
    lastAdxlSampleMs = now;
    adxlReadAccel();
    adxlAutoCalibrateOrientation(now);
  }
  #endif

  // Suppress hits only while the sensor is actually still in a candidate
  // orientation. Movement cancels this state above.
  if (adxlOrientationPending && adxlStableSinceMs != 0)
    return;

  // If continuous playback is enabled, skip hit detection
  if (continuousPlayback)
    return;

  static bool armed = true;
  static uint32_t lastHitTime = 0;

  const int LOWER_RESET = PIEZO_THRESHOLD * 0.3f;
  const int LOCKOUT_MS = ADXL_HIT_COOLDOWN_MS;

  // 1. Lockout after hit
  if (now - lastHitTime < LOCKOUT_MS)
  {
    armed = false;
  }

  // 2. Re-arm only when piezo is fully quiet
  if (!armed)
  {
    bool piezoReady = (piezoRaw < LOWER_RESET);
    bool adxlReady = (adxl_magnitude <= gAdxlThreshold * ADXL_REARM_RATIO);
    #if ADXL_ENABLED && !ADXL_DUAL_INPUT
    if (adxlReady)
    #else
    if (piezoReady && adxlReady)
    #endif
    {
      armed = true;
    }
    else
    {
      return;
    }
  }

  // 3. Detect valid rising hit (piezo or ADXL based on mode)
  bool hitDetected = false;
  
  #if ADXL_ENABLED && !ADXL_DUAL_INPUT
  // ADXL345 only mode
  hitDetected = (adxl_magnitude >= gAdxlThreshold);
  #elif ADXL_ENABLED && ADXL_DUAL_INPUT
  // Dual input: trigger on either piezo OR ADXL345
  hitDetected = (piezoRaw >= PIEZO_THRESHOLD) || (adxl_magnitude >= gAdxlThreshold);
  #else
  // Piezo only (ADXL disabled)
  hitDetected = (piezoRaw >= PIEZO_THRESHOLD);
  #endif
  
  if (armed && hitDetected)
  {
    lastHitTime = now;
    armed = false;

    #if ADXL_ENABLED
    float initialAdxlIntensity = adxlHitIntensity(adxl_magnitude);
    if (gHitTestEnabled)
    {
      Serial.printf("[HIT_START] accel=%.2fg threshold=%.2fg mapped_volume=%.3f\n",
                    adxl_magnitude,
                    gAdxlThreshold,
                    initialAdxlIntensity);
    }
    #endif

    // Peak-hold capture for piezo
    uint16_t peak = piezoRaw;
    uint32_t captureStart = micros();
    const uint32_t PEAK_CAPTURE_US = 1500;
    while (micros() - captureStart < PEAK_CAPTURE_US)
    {
      uint16_t sample = analogReadFast(PIEZO_CENTER_PIN);
      if (sample > peak)
        peak = sample;
    }
    piezoCenterPeak = peak;

    float velocity;
    float hitIntensity;
    #if ADXL_ENABLED && !ADXL_DUAL_INPUT
    velocity = adxlHitIntensity(adxl_magnitude);
    hitIntensity = velocity;
    #else
    velocity = (float)(peak - PIEZO_THRESHOLD) / (float)(1023 - PIEZO_THRESHOLD);
    velocity = constrain(velocity, 0.0f, 1.0f);
    velocity = 0.15f + velocity * 0.85f;
    hitIntensity = audioHitIntensity(peak, fsrRaw);
    #endif
    if (gHitTestEnabled)
    {
      #if ADXL_ENABLED
      Serial.printf("[HIT_TEST] model=%s peak=%u vel=%.3f intensity=%.3f fsr=%u accel=%.2fg hall=%.0f\n",
                    adxlModelName(gSelectedAdxlModel),
                    peak,
                    velocity,
                    hitIntensity,
                    fsrRaw,
                    adxl_magnitude,
                    smoothedHall);
      #else
      Serial.printf("[HIT_TEST] model=%s peak=%u vel=%.3f intensity=%.3f fsr=%u hall=%.0f\n",
                    adxlModelName(gSelectedAdxlModel),
                    peak,
                    velocity,
                    hitIntensity,
                    fsrRaw,
                    smoothedHall);
      #endif
    }

    ganGan.noteOn(velocity);
    lastHitMs = now;
    delayMicroseconds(30);
    Serial.printf(">>> HIT   vel: %.2f   accel: %.2fg   peak: %d   piezo: %d   hall:%.0f\n", velocity, adxl_magnitude, peak, piezoRaw, smoothedHall);
    return;
  }
}

// =============== SETUP ===============
void setup()
{
  Serial.begin(115200);
  delay(100);
  analogReadResolution(16);
  Serial.println("\n=== Teensy 4.x GAN-GAN SYNTH WITH SENSOR MODULATION ===");

  // Audio setup
  AudioMemory(12);
  audioShield.enable();
  audioShield.volume(1.0);

  ganGan.setBFreq(46.0f);
  ganGan.setEnvDur(0.62f);
  ganGan.setOutputGain(0.4f);
  ganGan.setFadeTime(0.5f);

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
  smoothedHall = readAnalogSettled(HALL_PITCH_PIN);
  currentFSR = 0;
  inPeakState = false;

  Serial.println("GAN-GAN Setup complete. Ready for hits!");
  
  #if ADXL_ENABLED
  #if ADXL_USE_I2C
  Serial.println("ADXL345 protocol: I2C (SDA=18, SCL=19; address=0x53 if SD0 grounded)");
  #elif ADXL_USE_SPI
  Serial.println("ADXL345 protocol: SPI (MOSI=11, MISO=12, SCL=13, CS=10)");
  #endif
  #if ADXL_DUAL_INPUT
  Serial.println("ADXL345 mode: DUAL (piezo AND ADXL345 both trigger)");
  #else
  Serial.println("ADXL345 mode: EXCLUSIVE (ADXL345 only, piezo ignored)");
  #endif
  adxlInit();
  #else
  Serial.println("ADXL345: DISABLED");
  #endif
  
  Serial.println("GPIO: A0=FSR, A1=Hall, A2=Piezo, A3/A4/A5=Pots");
  Serial.println("Analog Hall → Base Freq | FSR → Output Gain");
  Serial.println("Press: c=continuous, h=hit-test, 3=ADXL334, 4=ADXL345, +/-=threshold");
  Serial.println("Press: 1=HighMid, 2=FastHighDecay, 5=Tanh");
}

// =============== MAIN LOOP ===============
void loop()
{
  piezoISR();
  uint16_t rawHall = readAnalogSettled(HALL_PITCH_PIN);

  static float prevHall = 0.0f;
  static uint32_t lastReexciteMs = 0;

  const float hallDelta = fabsf((float)rawHall - prevHall);
  prevHall = (float)rawHall;

  float hallVal = smoothedHall;
  float hallNorm = ((float)hallVal - HALL_MIN_ADC) / (HALL_MAX_ADC - HALL_MIN_ADC);
  hallNorm = constrain(hallNorm, 0.0f, 1.0f);

  uint16_t bassRaw = readAnalogSettled(BASS_POT);
  float bassNorm = constrain((float)bassRaw / 1023.0f, 0.0f, 1.0f);
  float baseFreq = 70.0f + (110.0f - 70.0f) * bassNorm;
  baseFreq = constrain(baseFreq, 70.0f, 110.0f);

  uint16_t trebleRaw = readAnalogSettled(TREBLE_POT);
  float trebleNorm = constrain((float)trebleRaw / 1023.0f, 0.0f, 1.0f);
  ganGan.setBrightness(trebleNorm);

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

  float fsrNorm = currentFSR / 1023.0f;
  static bool fsrMuted = false;
  const float FSR_MUTE_LOW = 0.05f;
  const float FSR_MUTE_HIGH = 0.10f;
  if (fsrNorm <= FSR_MUTE_LOW)
    fsrMuted = true;
  else if (fsrNorm >= FSR_MUTE_HIGH)
    fsrMuted = false;

  float fadeTime = fsrMuted ? 0.03f : (0.1f + (1.9f * fsrNorm));
  ganGan.setFadeTime(fadeTime);

  uint16_t volumeRaw = readAnalogSettled(VOLUME_POT_PIN);
  float volumeNorm = constrain((float)volumeRaw / 1023.0f, 0.0f, 1.0f);
  ganGan.setMasterVolume(volumeNorm);

  // Debug output
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500)
  {
    lastDebugMs = millis();
    #if ADXL_ENABLED
    Serial.printf("Piezo: %4u | accel: %.2fg | Hall raw: %4u | Hall: %.0f Hz | Bass: %4u | Treble: %4u | FSR: %4u | fade: %.2fs | Vol: %.2f | Active: %u\n",
            piezoCenterPeak, adxl_magnitude, rawHall, hallFreq, bassRaw, trebleRaw, currentFSR, fadeTime, volumeNorm, ganGan.isActive() ? 1u : 0u);
    #else
    Serial.printf("Piezo: %4u | Hall raw: %4u | Hall: %.0f Hz | Bass: %4u | Treble: %4u | FSR: %4u | fade: %.2fs | Vol: %.2f | Active: %u\n",
            piezoCenterPeak, rawHall, hallFreq, bassRaw, trebleRaw, currentFSR, fadeTime, volumeNorm, ganGan.isActive() ? 1u : 0u);
    #endif
  }

  // Continuous playback mode
  if (continuousPlayback)
  {
    uint32_t nowMs = millis();
    if (nowMs - lastContinuousTriggerMs >= CONTINUOUS_TRIGGER_INTERVAL_MS)
    {
      lastContinuousTriggerMs = nowMs;
      float velocity = constrain(currentFSR / 1023.0f, 0.8f, 1.0f);
      ganGan.noteOn(velocity);
      Serial.printf(">>> CONTINUOUS TRIGGER   vel: %.2f   hall:%.0f Hz   fsr:%d\n",
                    velocity, hallFreq, currentFSR);
    }
  }

  // Serial commands
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
}

/*
  Arduino Uno + ADXL345 I2C hit-test logger

  Wiring (Arduino Uno):
    ADXL345 VCC  -> 3.3V
    ADXL345 GND  -> GND
    ADXL345 SDA  -> A4 / SDA
    ADXL345 SCL  -> A5 / SCL
    ADXL345 CS   -> 3.3V for I2C mode
    ADXL345 SD0  -> GND for address 0x53, or 3.3V for address 0x1D
    ADXL345 INT1/INT2 are not required by this sketch.

  Serial Monitor: 115200 baud, no line ending required.
  Output is CSV and can be saved for later hit classification/training.

  Commands:
    h       toggle hit-event reporting
    c       recalibrate stationary baseline
    + / -   increase/decrease hit threshold by 0.05 g
    r       print current settings
*/

#include <Wire.h>
#include <math.h>

// Set to 0x1D when SD0 is connected to 3.3V.
#define ADXL345_ADDRESS 0x53

#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_DATAX0      0x32

#define ADXL345_DEVICE_ID       0xE5
#define ADXL345_MEASURE_MODE    0x08
#define ADXL345_FULL_RES_16G    0x0B
#define ADXL345_RATE_200HZ      0x0C

const unsigned long SAMPLE_INTERVAL_MS = 5;
const unsigned long HIT_COOLDOWN_MS = 80;
const unsigned long CALIBRATION_TIME_MS = 1500;
const float LSB_PER_G = 256.0f;

bool reportHits = true;
float hitThresholdG = 0.35f;
float baselineX = 0.0f;
float baselineY = 0.0f;
float baselineZ = 0.0f;
unsigned long lastSampleMs = 0;
unsigned long lastHitMs = 0;
unsigned long sampleNumber = 0;
bool printSerial = true;
struct AccelSample
{
  int16_t x;
  int16_t y;
  int16_t z;
};

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  byte status = Wire.endTransmission();
  if (status != 0)
  {
    Serial.print(F("I2C_WRITE_ERROR,"));
    Serial.println(status);
  }
}

int readRegisters(byte startReg, byte *buffer, byte length)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0)
    return 0;

  byte received = Wire.requestFrom(ADXL345_ADDRESS, length);
  if (received != length)
    return 0;

  for (byte i = 0; i < length; ++i)
    buffer[i] = Wire.read();

  return 1;
}

byte readRegister(byte reg)
{
  byte value = 0;
  if (!readRegisters(reg, &value, 1))
    Serial.println(F("I2C_READ_ERROR"));
  return value;
}

bool readAcceleration(AccelSample &sample)
{
  byte data[6];
  if (!readRegisters(ADXL345_REG_DATAX0, data, sizeof(data)))
    return false;

  sample.x = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
  sample.y = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
  sample.z = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
  return true;
}

void printSettings()
{
  Serial.print(F("# hit_reporting="));
  Serial.println(reportHits ? F("ON") : F("OFF"));
  Serial.print(F("# threshold_g="));
  Serial.println(hitThresholdG, 3);
  Serial.print(F("# baseline_g="));
  Serial.print(baselineX / LSB_PER_G, 3);
  Serial.print(',');
  Serial.print(baselineY / LSB_PER_G, 3);
  Serial.print(',');
  Serial.println(baselineZ / LSB_PER_G, 3);
}

void calibrateBaseline()
{
  Serial.println(F("# Keep the ADXL345 still: calibrating baseline..."));

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  unsigned int count = 0;
  unsigned long startMs = millis();

  while (millis() - startMs < CALIBRATION_TIME_MS)
  {
    AccelSample sample;
    if (readAcceleration(sample))
    {
      sumX += sample.x;
      sumY += sample.y;
      sumZ += sample.z;
      ++count;
    }
    delay(SAMPLE_INTERVAL_MS);
  }

  if (count == 0)
  {
    Serial.println(F("# Calibration failed: no I2C samples."));
    return;
  }

  baselineX = (float)sumX / count;
  baselineY = (float)sumY / count;
  baselineZ = (float)sumZ / count;
  Serial.print(F("# Calibration complete, samples="));
  Serial.println(count);
  printSettings();
}

void handleCommands()
{
  while (Serial.available() > 0)
  {
    char command = Serial.read();
    if (command == 'h' || command == 'H')
    {
      reportHits = !reportHits;
      printSettings();
    }
    else if (command == 'c' || command == 'C')
    {
      calibrateBaseline();
    }
    else if (command == '+')
    {
      hitThresholdG = min(2.0f, hitThresholdG + 0.05f);
      printSettings();
    }
    else if (command == '-')
    {
      hitThresholdG = max(0.05f, hitThresholdG - 0.05f);
      printSettings();
    }
    else if (command == 'r' || command == 'R')
    {
      printSettings();
    }
    else if (command == 't' || command == 'T')
    {
      printSerial = true;
    }
    else if (command == 's' || command == 'S')
    {
      printSerial = false;
    }
  }
}

void printCsvHeader()
{
  Serial.println(F("sample_ms,sample_number,x_raw,y_raw,z_raw,x_g,y_g,z_g,dynamic_g,hit,threshold_g"));
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);

  delay(100);
  byte deviceId = readRegister(ADXL345_REG_DEVID);
  Serial.print(F("# ADXL345_device_id=0x"));
  if (deviceId < 0x10)
    Serial.print('0');
  Serial.println(deviceId, HEX);

  if (deviceId != ADXL345_DEVICE_ID)
  {
    Serial.println(F("# ERROR: expected device ID 0xE5. Check power, SDA/SCL, CS, SD0, and address."));
    while (true)
      handleCommands();
  }

  writeRegister(ADXL345_REG_POWER_CTL, 0x00);
  writeRegister(ADXL345_REG_DATA_FORMAT, ADXL345_FULL_RES_16G);
  writeRegister(ADXL345_REG_BW_RATE, ADXL345_RATE_200HZ);
  writeRegister(ADXL345_REG_POWER_CTL, ADXL345_MEASURE_MODE);
  delay(20);

  calibrateBaseline();
  printCsvHeader();
}

void loop()
{
  handleCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS)
    return;
  lastSampleMs = nowMs;

  AccelSample sample;
  if (!readAcceleration(sample))
    return;

  float xG = sample.x / LSB_PER_G;
  float yG = sample.y / LSB_PER_G;
  float zG = sample.z / LSB_PER_G;
  float dxG = xG - (baselineX / LSB_PER_G);
  float dyG = yG - (baselineY / LSB_PER_G);
  float dzG = zG - (baselineZ / LSB_PER_G);
  float dynamicG = sqrt(dxG * dxG + dyG * dyG + dzG * dzG);

  bool hit = dynamicG >= hitThresholdG && (nowMs - lastHitMs >= HIT_COOLDOWN_MS);
  if (hit)
    lastHitMs = nowMs;
  if (printSerial) {
    // Every sample is logged. The hit column is the label for training data.
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(sampleNumber++);
    Serial.print(',');
    Serial.print(sample.x);
    Serial.print(',');
    Serial.print(sample.y);
    Serial.print(',');
    Serial.print(sample.z);
    Serial.print(',');
    Serial.print(xG, 4);
    Serial.print(',');
    Serial.print(yG, 4);
    Serial.print(',');
    Serial.print(zG, 4);
    Serial.print(',');
    Serial.print(dynamicG, 4);
    Serial.print(',');
    Serial.print(hit ? 1 : 0);
    Serial.print(',');
    Serial.println(hitThresholdG, 3);
  }
  if (hit && reportHits)
  {
    Serial.print(F("# HIT,dynamic_g="));
    Serial.print(dynamicG, 3);
    Serial.print(F(",threshold_g="));
    Serial.println(hitThresholdG, 3);
  }
}

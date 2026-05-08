/*
  Analog Hall Sensor Reader
  - Reads analog Hall sensor output
  - Prints raw ADC value and converted voltage
  - Optional simple moving average filter for smoother readings
*/
#include <Arduino.h>
const int HALL_PIN = A1;            // Analog pin connected to sensor output
const float VREF = 3.3;             // ADC reference voltage (5.0 for most UNO/Nano, 3.3 for some boards)
const int ADC_MAX = 1023;           // 10-bit ADC on AVR boards
const int SAMPLE_DELAY_MS = 50;     // Read interval

// Moving average settings
const int FILTER_SIZE = 10;
int buffer[FILTER_SIZE];
int bufIndex = 0;
long runningSum = 0;
bool filterFilled = false;

void setup() {
  Serial.begin(115200);

  // If your board supports analogReadResolution() (e.g., some ARM boards), adjust logic accordingly.
  pinMode(HALL_PIN, INPUT);

  // Initialize filter buffer
  for (int i = 0; i < FILTER_SIZE; i++) {
    buffer[i] = 0;
  }

  Serial.println("Analog Hall Sensor Reader Started");
  Serial.println("Raw\tFiltered\tVoltage(V)");
}

void loop() {
  int raw = analogRead(HALL_PIN);

  // Update moving average
  runningSum -= buffer[bufIndex];
  buffer[bufIndex] = raw;
  runningSum += buffer[bufIndex];

  bufIndex++;
  if (bufIndex >= FILTER_SIZE) {
    bufIndex = 0;
    filterFilled = true;
  }

  int divisor = filterFilled ? FILTER_SIZE : bufIndex;
  if (divisor == 0) divisor = 1; // safety
  int filtered = runningSum / divisor;

  float voltage = (filtered * VREF) / ADC_MAX;

  Serial.print(raw);
  Serial.print('\t');
  Serial.print(filtered);
  Serial.print('\t');
  Serial.println(voltage, 3);

  delay(SAMPLE_DELAY_MS);
}

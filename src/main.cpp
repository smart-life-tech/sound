#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#include "drum_raw.h"   // your generated header

// Audio objects
AudioPlayMemory playMem;
AudioOutputI2S  i2s1;
AudioConnection patchCord1(playMem, 0, i2s1, 0);
AudioConnection patchCord2(playMem, 0, i2s1, 1);

// If you are using the Teensy Audio Shield (SGTL5000), enable these:
// AudioControlSGTL5000 audioShield;

void setup() {
  AudioMemory(20);

  // If using Audio Shield, uncomment:
  // audioShield.enable();
  // audioShield.volume(0.8);

  delay(100);

  // 👇 This plays your raw sample from program memory
  playMem.play(drum_raw);
  delay(10);  // required for indexing
}

void loop() {
  // Do nothing – sample plays automatically.
}

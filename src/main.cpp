#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// Your generated PCM data
#include "drum_raw.h"

// Audio objects
AudioPlayMemory playMem;
AudioFilterStateVariable ksFilter;
AudioEffectEnvelope env;
AudioOutputI2S i2s1;

AudioMixer4 mix;
AudioConnection c1(playMem, 0, mix, 0);
AudioConnection c2(mix, 0, i2s1, 0);
AudioConnection c3(mix, 0, i2s1, 1);

AudioControlSGTL5000 audioShield;

// Inputs
#define PIEZO_PIN  A2
#define FLEX_PIN   A1
#define FSR_PIN    A0

// Globals
float smoothedFlex = 0;
const float flex_alpha = 0.15;

// Flex → pitch multiplier (continuous bend)
float flexToPitch(float f) {
  float x = constrain(f / 4095.0f, 0, 1);
  float curved = powf(x, 2.2f); // good gangan curve
  return 1.0f + curved * 1.5f;  // up to ~1.5× pitch shift
}

// FSR → damping
void applyFSR(uint16_t fsr) {
  float n = fsr / 4095.0f;
  float cutoff = 800 + (1.0f - n) * 3000;   // low FSR = ring
  float q      = 1.5  + (1.0f - n) * 3.0;   // low FSR = high Q

  ksFilter.frequency(cutoff);
  ksFilter.resonance(q);
}

void triggerDrum() {
  // Read sensors
  uint16_t flex = analogRead(FLEX_PIN);
  uint16_t fsr  = analogRead(FSR_PIN);

  // Smooth flex
  smoothedFlex = smoothedFlex + flex_alpha * (flex - smoothedFlex);

  // Compute pitch bend
  float bend = flexToPitch(smoothedFlex);

  // Play sample
  playMem.play(drum_raw);

  // Apply real-time bending
  playMem.setPlaybackRate(bend);

  // Damping filter
  applyFSR(fsr);

  // Envelope
  env.noteOn();
}

void setup() {
  Serial.begin(115200);

  AudioMemory(20);
  audioShield.enable();
  audioShield.volume(0.9);

  mix.gain(0, 1.0);

  pinMode(PIEZO_PIN, INPUT);
  pinMode(FLEX_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);

  smoothedFlex = analogRead(FLEX_PIN);
}

void loop() {
  uint16_t p = analogRead(PIEZO_PIN);

  if (p > 300) {  // threshold
    triggerDrum();
    delay(120);   // debounce
  }

  // Apply continuous bending while sound plays
  smoothedFlex =
    smoothedFlex + flex_alpha * (analogRead(FLEX_PIN) - smoothedFlex);

  float bend = flexToPitch(smoothedFlex);
  playMem.setPlaybackRate(bend);

  applyFSR(analogRead(FSR_PIN));

  delay(2);
}

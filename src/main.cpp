#include <Audio.h>

AudioSynthWaveform wave;
AudioOutputI2S     i2s1;
AudioConnection    patch(wave, 0, i2s1, 0);

AudioControlSGTL5000 unused; // not used but required

void setup() {
  AudioMemory(8);
  wave.begin(WAVEFORM_SINE);
  wave.frequency(440);
  wave.amplitude(0.6);
}

void loop() {}

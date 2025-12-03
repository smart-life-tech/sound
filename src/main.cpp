#include <Audio.h>

AudioSynthWaveform wave;
AudioOutputI2S i2s1;
AudioConnection patch(wave, 0, i2s1, 0);

AudioControlSGTL5000 sgt; // not used but required

void setup()
{
  AudioMemory(10);
  wave.begin(WAVEFORM_SINE);
  wave.frequency(440);
  sgt.enable();
  sgt.volume(1.0);
  wave.amplitude(0.6);
}

void loop() {}

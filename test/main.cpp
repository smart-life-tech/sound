#include <Audio.h>

AudioSynthWaveform wave;
AudioOutputI2S i2s1;
AudioConnection patch(wave, 0, i2s1, 0);
AudioConnection patch2(wave, 0, i2s1, 1);
AudioControlSGTL5000 sgt; // not used but required

void setup()
{
  AudioMemory(4);
 // wave.begin(WAVEFORM_SINE);
  sgt.enable();
  sgt.volume(1.0);
  wave.frequency(440);
  wave.amplitude(0.6);
}

void loop() {}

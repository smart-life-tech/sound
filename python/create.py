import numpy as np
from scipy.io import wavfile
from scipy import signal

INPUT_WAV  = "drum.wav"
HEADER_OUT = "drum_raw.h"
TARGET_SR = 44100

# -----------------
# Load WAV
# -----------------
sr, audio = wavfile.read(INPUT_WAV)
print("Input sample rate:", sr)

# Convert stereo → mono
if len(audio.shape) == 2:
    audio = audio.mean(axis=1)

# Convert to float for processing
audio = audio.astype(np.float32)

# Normalize
audio /= np.max(np.abs(audio))

# Resample if needed
if sr != TARGET_SR:
    print("Resampling...")
    num = int(len(audio) * TARGET_SR / sr)
    audio = signal.resample(audio, num)

# Convert to int16
audio = (audio * 32767).astype(np.int16)

# PADDING: AudioPlayMemory needs even number of samples
if len(audio) % 2 == 1:
    audio = np.append(audio, 0)

# -----------------
# PACK INTO 32-BIT WORDS
# -----------------
packed = []
for i in range(0, len(audio), 2):
    lo = np.uint16(audio[i])
    hi = np.uint16(audio[i+1])
    word = np.uint32((hi << 16) | lo)
    packed.append(word)

packed = np.array(packed, dtype=np.uint32)

print("Samples:", len(audio))
print("Words:", len(packed))

# -----------------
# GENERATE HEADER FILE
# -----------------
with open(HEADER_OUT, "w") as f:
    f.write("// Auto-generated Teensy AudioPlayMemory header\n")
    f.write("// PCM16  packed into uint32_t words\n\n")
    f.write("#pragma once\n#include <Arduino.h>\n\n")
    f.write("const unsigned int drum_raw_len = {};\n".format(len(packed)))
    f.write("const unsigned int drum_raw[] PROGMEM = {\n")

    for i, w in enumerate(packed):
        f.write("  {}".format(int(w)))
        if i < len(packed)-1:
            f.write(",")
        f.write("\n")

    f.write("};\n")

print("Generated:", HEADER_OUT)

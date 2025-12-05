import numpy as np
from scipy.io import wavfile

INPUT_WAV  = "drum.wav"
HEADER_OUT = "drum_raw.h"

# Load wav
sr, data = wavfile.read(INPUT_WAV)

# Convert to mono if stereo
if len(data.shape) == 2:
    data = data.mean(axis=1)

# Ensure int16
if data.dtype != np.int16:
    data = (data / np.max(np.abs(data)))
    data = (data * 32767).astype(np.int16)

# Pad to even number of samples
if len(data) % 2 == 1:
    data = np.append(data, 0)

# Pack two int16 samples into one uint32 word
packed = []
for i in range(0, len(data), 2):
    lo = np.uint16(data[i])
    hi = np.uint16(data[i+1])
    word = np.uint32((hi << 16) | lo)
    packed.append(word)

packed = np.array(packed, dtype=np.uint32)

print("Samples:", len(data))
print("Words:", len(packed))

# Write header file
with open(HEADER_OUT, "w") as f:
    f.write("// Auto-generated for Teensy AudioPlayMemory\n\n")
    f.write("#pragma once\n\n")
    f.write("const unsigned int drum_raw_len = {};\n".format(len(packed)))
    f.write("const unsigned int drum_raw[] PROGMEM = {\n    ")

    for i, w in enumerate(packed):
        f.write(str(int(w)))
        if i < len(packed) - 1:
            f.write(", ")
        if (i+1) % 8 == 0:
            f.write("\n    ")

    f.write("\n};\n")

print("Generated:", HEADER_OUT)

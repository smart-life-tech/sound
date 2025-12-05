import numpy as np
from scipy.io import wavfile

# ---------------------------
# SETTINGS
# ---------------------------

INPUT_WAV  = "drum.wav"      # your wav file
RAW_FILE   = "drum.raw"
HEADER_OUT = "drum_raw.h"

TARGET_RATE = 44100  # Teensy Audio Library sample rate

# ---------------------------
# LOAD WAV
# ---------------------------

sr, data = wavfile.read(INPUT_WAV)
print("Input sample rate:", sr)

# Convert to mono if stereo
if len(data.shape) == 2:
    print("Stereo detected → converting to mono.")
    data = data.mean(axis=1)

# Normalize & convert to int16
if data.dtype != np.int16:
    print("Converting to int16...")
    data = data / np.max(np.abs(data))
    data = (data * 32767).astype(np.int16)

# ---------------------------
# Optional: resample to 44.1k if needed
# ---------------------------

if sr != TARGET_RATE:
    print("Resampling to 44100 Hz...")
    # Simple but effective resample (no external libraries)
    factor = TARGET_RATE / sr
    new_length = int(len(data) * factor)
    data = np.interp(
        np.linspace(0, len(data), new_length),
        np.arange(len(data)),
        data
    ).astype(np.int16)

# ---------------------------
# SAVE RAW PCM
# ---------------------------

data.tofile(RAW_FILE)
print("Saved RAW file:", RAW_FILE)

# ---------------------------
# GENERATE HEADER FILE
# ---------------------------

with open(HEADER_OUT, "w") as f:
    f.write("// Auto-generated from {}\n".format(INPUT_WAV))
    f.write("// Signed 16-bit PCM, 44100 Hz\n\n")
    f.write("#pragma once\n")
    f.write("#include <Arduino.h>\n\n")

    f.write("const unsigned int drum_raw_len = {};\n".format(len(data)))
    f.write("const int16_t drum_raw[] PROGMEM = {\n    ")

    # Write 16 samples per line for readability
    for i, sample in enumerate(data):
        f.write(str(int(sample)))
        if i < len(data) - 1:
            f.write(", ")

        if (i + 1) % 16 == 0:
            f.write("\n    ")

    f.write("\n};\n")

print("Generated header:", HEADER_OUT)

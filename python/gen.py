#!/usr/bin/env python3
"""
Generate Teensy header files for low-latency drum sampler
Input:  drum_base.wav  (mono, 16-bit PCM, short clean hit)
Output: 15 or 30 .h files in ./headers/
"""

import os, numpy as np, soundfile as sf
from scipy.signal import resample

# ===== User config =====
BASE_WAV = "base.wav"
OUT_DIR = "headers"
VEL_LEVELS = [0.6, 0.85, 1.0]     # soft, medium, hard multipliers
PITCH_STEPS = [0, 2, 4, 7, 12]    # semitones relative to C4
MAKE_SHORT_RELEASE = True
SHORT_RELEASE_MS = 120             # how long short variant lasts

# ===== Utility =====
def semitone_ratio(st):
    return 2 ** (st / 12.0)

def write_header(name, data):
    data_i16 = np.clip(data * 32767, -32768, 32767).astype(np.int16)
    header_path = os.path.join(OUT_DIR, f"{name}.h")
    with open(header_path, "w") as f:
        f.write(f"// Auto-generated from {BASE_WAV}\n")
        f.write(f"#pragma once\n#include <Arduino.h>\nconst int16_t {name}[] PROGMEM = {{\n")

        for i, v in enumerate(data_i16):
            if i % 16 == 0: f.write("    ")
            f.write(f"{int(v)}, ")
            if i % 16 == 15: f.write("\n")
        f.write("\n};\n")
        f.write(f"const unsigned int {name}_len = {len(data_i16)};\n")
    print("Wrote", header_path)

# ===== Main =====
os.makedirs(OUT_DIR, exist_ok=True)
data, fs = sf.read(BASE_WAV)
if data.ndim > 1: data = data[:,0]  # mono

for vi, vscale in enumerate(VEL_LEVELS):
    for pi, pitch in enumerate(PITCH_STEPS):
        # pitch-shift by resampling
        ratio = semitone_ratio(pitch)
        new_len = int(len(data) / ratio)
        pitched = resample(data, new_len)
        # normalize & scale by velocity
        pitched = pitched / np.max(np.abs(pitched)) * vscale

        # Long version
        name_long = f"drum_v{vi}_p{pi}_long"
        write_header(name_long, pitched)

        # Short version (truncated)
        if MAKE_SHORT_RELEASE:
            samples_short = int(fs * SHORT_RELEASE_MS / 1000)
            truncated = pitched[:samples_short]
            name_short = f"drum_v{vi}_p{pi}_short"
            write_header(name_short, truncated)

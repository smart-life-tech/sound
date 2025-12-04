import numpy as np
from scipy.io import wavfile
from scipy.signal import hilbert, find_peaks
import matplotlib.pyplot as plt

# Load WAV
sr, y = wavfile.read("New drum nmo squeeze.wav")
y = y.astype(float)

# Normalize
y = y / np.max(np.abs(y))

# -------------------------------------------------------------------
# 1. Amplitude Envelope (Hilbert transform)
analytic = hilbert(y)
amplitude_envelope = np.abs(analytic)

# -------------------------------------------------------------------
# 2. Fundamental frequency estimation (peak spacing method)
window = int(sr * 0.02)   # 20ms window
hop = int(sr * 0.005)     # 5ms hop

f0_list = []
t_list = []

for start in range(0, len(y)-window, hop):
    frame = y[start:start+window]

    # Autocorrelation
    corr = np.correlate(frame, frame, mode="full")
    corr = corr[len(corr)//2:]

    # Find peaks in autocorrelation
    peaks, _ = find_peaks(corr)

    if len(peaks) < 2:
        f0_list.append(np.nan)
    else:
        # First major peak = period
        lag = peaks[0]
        f0 = sr / lag
        f0_list.append(f0)

    t_list.append(start / sr)

f0_arr = np.array(f0_list)
t_arr = np.array(t_list)

# -------------------------------------------------------------------
# 3. Plot results
plt.figure(figsize=(12,8))

plt.subplot(3,1,1)
plt.title("Waveform")
plt.plot(y)

plt.subplot(3,1,2)
plt.title("Amplitude Envelope")
plt.plot(amplitude_envelope)

plt.subplot(3,1,3)
plt.title("Pitch Contour (Estimated F0)")
plt.plot(t_arr, f0_arr)
plt.ylim(0, 1200)

plt.tight_layout()
plt.show()

# -------------------------------------------------------------------
# Print useful extracted values
valid_f0 = f0_arr[~np.isnan(f0_arr)]
print("F0 Min:", valid_f0.min())
print("F0 Max:", valid_f0.max())
print("F0 Median:", np.median(valid_f0))

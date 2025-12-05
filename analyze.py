"""Enhanced analysis script

This script analyzes a single WAV file and extracts synth-relevant parameters:
- Robust F0 contour (uses librosa.yin() when available)
- Amplitude envelope (attack/decay)
- Pitch-drop metrics (start/end, drop time)
- Spectral centroid and high-frequency energy ratio
- Transient RMS energy

Outputs:
- `synth_params.json` with suggested parameter values
- Prints a C++ constants block for quick paste into `main.cpp`

Usage: place the target WAV in the same folder and set `INPUT_WAV`.
"""

import json
import numpy as np
from scipy.io import wavfile
from scipy.signal import hilbert
import matplotlib.pyplot as plt

INPUT_WAV = "drum.wav"

try:
    import librosa
    HAS_LIBROSA = True
except Exception:
    HAS_LIBROSA = False


def load_wav(path):
    sr, y = wavfile.read(path)
    # convert to mono if stereo
    if y.ndim > 1:
        y = y.mean(axis=1)
    y = y.astype(float)
    # normalize
    if np.max(np.abs(y)) > 0:
        y = y / np.max(np.abs(y))
    return sr, y


def amplitude_envelope(y):
    analytic = hilbert(y)
    return np.abs(analytic)


def robust_f0(y, sr, frame_length=2048, hop_length=512, fmin=50, fmax=1200):
    # Use librosa.yin when available; otherwise fallback to autocorrelation per-frame
    if HAS_LIBROSA:
        f0 = librosa.yin(y, fmin=fmin, fmax=fmax, sr=sr, frame_length=frame_length, hop_length=hop_length)
        times = librosa.frames_to_time(np.arange(len(f0)), sr=sr, hop_length=hop_length)
        f0[np.isnan(f0)] = 0.0
        return f0, times

    # Fallback: simple autocorrelation-based estimator
    f0_list = []
    t_list = []
    for start in range(0, len(y) - frame_length, hop_length):
        frame = y[start:start + frame_length]
        corr = np.correlate(frame, frame, mode="full")
        corr = corr[len(corr) // 2:]
        # find first meaningful peak (ignore lag < 20 samples)
        peak_idx = 0
        # ignore small lags (below 20 samples ~ for sr=44100 that's <2ms)
        for i in range(20, len(corr)):
            if corr[i] > 1e-4 * corr[0]:
                peak_idx = i
                break
        if peak_idx <= 20:
            f0_list.append(0.0)
        else:
            f0_list.append(sr / peak_idx)
        t_list.append(start / sr)
    return np.array(f0_list), np.array(t_list)


def compute_envelope_metrics(env, sr):
    # env assumed normalized 0..1
    peak_idx = np.argmax(env)
    peak_val = env[peak_idx]
    # attack: time from 10% -> 90%
    ten = peak_val * 0.1
    ninety = peak_val * 0.9
    try:
        t10 = np.where(env[:peak_idx] >= ten)[0][0]
    except Exception:
        t10 = 0
    try:
        t90 = np.where(env[:peak_idx] >= ninety)[0][0]
    except Exception:
        t90 = t10
    attack_ms = (t90 - t10) * 1000.0 / sr
    # decay: time from peak -> 37% (~1/e)
    one_over_e = peak_val * np.exp(-1.0)
    try:
        t_decay = np.where(env[peak_idx:] <= one_over_e)[0][0]
        decay_ms = t_decay * 1000.0 / sr
    except Exception:
        decay_ms = (len(env) - peak_idx) * 1000.0 / sr
    return attack_ms, decay_ms, peak_idx


def spectral_metrics(y, sr, n_fft=2048, hop_length=512):
    # use short-time FFT and compute centroid and HF ratio (>3kHz)
    import numpy.fft as fft
    S = []
    for start in range(0, len(y) - n_fft, hop_length):
        frame = y[start:start + n_fft] * np.hanning(n_fft)
        spec = np.abs(fft.rfft(frame))
        S.append(spec)
    S = np.array(S).T  # shape freq x frames
    freqs = np.fft.rfftfreq(n_fft, 1.0 / sr)
    centroid = []
    hf_ratio = []
    for i in range(S.shape[1]):
        spec = S[:, i]
        if spec.sum() <= 0:
            centroid.append(0.0)
            hf_ratio.append(0.0)
            continue
        c = (freqs * spec).sum() / spec.sum()
        centroid.append(c)
        hf = spec[freqs >= 3000].sum()
        hf_ratio.append(hf / spec.sum())
    return np.array(centroid), np.array(hf_ratio)


def transient_rms(y, sr, trans_ms=30):
    n = int(sr * (trans_ms / 1000.0))
    n = min(n, len(y))
    return np.sqrt(np.mean(y[:n] ** 2))


def main():
    sr, y = load_wav(INPUT_WAV)
    # envelope
    env = amplitude_envelope(y)
    env = env / np.max(env)

    # f0
    frame_length = 2048
    hop_length = 512
    f0, times = robust_f0(y, sr, frame_length=frame_length, hop_length=hop_length)
    valid_f0 = f0[f0 > 0]
    if len(valid_f0) == 0:
        print("No F0 estimated")
        return

    f0_min = float(np.min(valid_f0))
    f0_max = float(np.max(valid_f0))
    f0_median = float(np.median(valid_f0))

    # get start/end pitch metrics
    startF = float(valid_f0[0]) if len(valid_f0) > 0 else f0_median
    # endF: median of later frames (skip first few frames)
    end_slice = valid_f0[min(10, len(valid_f0)-1):min(60, len(valid_f0))]
    endF = float(np.median(end_slice)) if len(end_slice) > 0 else f0_median

    # find drop time (time to go from startF to endF)
    drop_time_ms = 0.0
    if startF > 0 and endF > 0 and startF > endF:
        # find first time f0 <= endF + 5% margin
        margin = endF * 1.05
        for i, val in enumerate(f0):
            if val > 0 and val <= margin:
                drop_time_ms = float(times[i] * 1000.0)
                break

    attack_ms, decay_ms, peak_idx = compute_envelope_metrics(env, sr)

    centroid, hf_ratio = spectral_metrics(y, sr, n_fft=2048, hop_length=512)
    centroid_start = float(np.median(centroid[max(0, peak_idx-2):peak_idx+5]))
    hf_start = float(np.median(hf_ratio[max(0, peak_idx-2):peak_idx+5]))

    trans_rms = float(transient_rms(y, sr, trans_ms=30))

    # Suggested synth params mapping
    synth = {
        "minF": max(40.0, f0_min * 0.9),
        "maxF": min(sr/2.0, f0_max * 1.05),
        "medianF": f0_median,
        "startF": startF,
        "endF": endF,
        "pitch_drop_ms": drop_time_ms if drop_time_ms > 0 else max(80.0, decay_ms),
        "attack_ms": attack_ms,
        "decay_ms": decay_ms,
        "transient_rms": trans_rms,
        "spectral_centroid_start": centroid_start,
        "hf_ratio_start": hf_start,
        # suggested controls
        "suggested_ks_sustain_ms": max(50.0, decay_ms * 3.0),
        "suggested_fm_index": min(60.0, 5.0 + hf_start * 50.0),
        "suggested_noise_level": min(1.0, hf_start * 2.0)
    }

    # Write JSON
    with open("synth_params.json", "w") as f:
        json.dump(synth, f, indent=2)

    # Print summary and C++ block
    print("Analysis summary:")
    for k, v in synth.items():
        print(f"  {k}: {v}")

    cpp_block = (
        "// Synth parameters generated by analyze.py\n"
        "static constexpr float S_MIN_FREQ = %0.1ff;\n"
        "static constexpr float S_MAX_FREQ = %0.1ff;\n"
        "static constexpr float S_FM_START = %0.1ff;\n"
        "static constexpr float S_FM_END = %0.1ff;\n"
        "static constexpr int S_FM_DROP_MS = %d;\n"
        "static constexpr int S_KS_SUSTAIN_MS = %d;\n"
        "static constexpr float S_FM_INDEX = %0.2ff;\n"
        "static constexpr float S_NOISE_LVL = %0.2ff;\n"
    ) % (
        synth["minF"], synth["maxF"], synth["startF"], synth["endF"], int(synth["pitch_drop_ms"]), int(synth["suggested_ks_sustain_ms"]), synth["suggested_fm_index"], synth["suggested_noise_level"],
    )

    print("\nC++ snippet to paste into main.cpp:\n")
    print(cpp_block)


if __name__ == '__main__':
    main()

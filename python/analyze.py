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


def harmonic_content(y, sr, n_fft=2048, hop_length=512):
    """Estimate harmonic-to-noise ratio and dominant harmonic using STFT."""
    import numpy.fft as fft
    hh_ratios = []
    for start in range(0, len(y) - n_fft, hop_length):
        frame = y[start:start + n_fft] * np.hanning(n_fft)
        spec = np.abs(fft.rfft(frame))
        # Assume top 5 peaks are harmonics, rest is noise
        top_indices = np.argsort(spec)[-5:]
        harmonic_energy = spec[top_indices].sum()
        total_energy = spec.sum()
        if total_energy > 0:
            hh_ratios.append(harmonic_energy / total_energy)
        else:
            hh_ratios.append(0.0)
    return np.array(hh_ratios)


def damping_rate(env, sr):
    """Estimate exponential damping coefficient (higher = faster decay)."""
    # Find peak index
    peak_idx = np.argmax(env)
    if peak_idx >= len(env) - 10:
        return 0.0
    # Look at tail after peak
    tail = env[peak_idx+5:]
    # Fit to exp(-lambda * t)
    t_samples = np.arange(len(tail))
    t_time = t_samples / sr
    tail_nonzero = tail[tail > 0.01]  # ignore noise floor
    if len(tail_nonzero) < 3:
        return 0.0
    # Simple estimate: time to drop by half (half-life)
    try:
        half_val = tail[0] / 2.0
        half_idx = np.where(tail < half_val)[0]
        if len(half_idx) > 0:
            half_life_ms = (half_idx[0] / sr) * 1000.0
            # lambda = ln(2) / half_life
            lambda_decay = 0.693 / (half_life_ms / 1000.0) if half_life_ms > 0 else 0.0
            return lambda_decay
    except Exception:
        pass
    return 0.0


def ks_resonance_estimate(centroid, trans_rms, hf_ratio):
    """Estimate KS filter resonance from spectral shape."""
    # Higher centroid + high HF ratio suggests resonant peaks
    # Lower trans_rms + high centroid suggests metallic character
    if np.isnan(centroid) or np.isnan(hf_ratio):
        return 0.5
    # Simple heuristic: metallic -> high Q, warm -> low Q
    q = 0.5 + (hf_ratio * 0.8)  # 0.5 .. 1.3
    return min(1.5, q)


def ks_cutoff_estimate(centroid, median_f0):
    """Estimate KS filter cutoff frequency based on spectral centroid."""
    if np.isnan(centroid):
        return median_f0 * 2.0
    # Cutoff should be around 2-4x the body fundamental
    cutoff = max(centroid * 0.8, median_f0 * 1.5)
    return min(1000.0, cutoff)


def noise_color_estimate(hf_ratio):
    """Estimate noise color (white->pink->brown based on HF content)."""
    if np.isnan(hf_ratio):
        return 0.5
    # High HF ratio = brighter noise, low = darker
    return hf_ratio


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
    
    # New advanced metrics
    hh_ratio = harmonic_content(y, sr)
    hh_start = float(np.median(hh_ratio[max(0, peak_idx-2):min(peak_idx+10, len(hh_ratio))]))
    
    damp_rate = damping_rate(env, sr)
    
    ks_q = ks_resonance_estimate(centroid_start, trans_rms, hf_start)
    ks_cut = ks_cutoff_estimate(centroid_start, f0_median)
    noise_color = noise_color_estimate(hf_start)

    # Suggested synth params mapping (improved)
    synth = {
        "minF": max(40.0, f0_min * 0.95),
        "maxF": min(sr/2.0, f0_max * 1.0),
        "medianF": f0_median,
        "startF": startF,
        "endF": endF,
        "pitch_drop_ms": drop_time_ms if drop_time_ms > 0 else max(10.0, decay_ms * 0.2),
        "attack_ms": attack_ms,
        "decay_ms": decay_ms,
        "transient_rms": trans_rms,
        "spectral_centroid_start": centroid_start,
        "hf_ratio_start": hf_start,
        "harmonic_ratio": hh_start,
        "damping_rate": damp_rate,
        "ks_resonance_q": ks_q,
        "ks_filter_cutoff": ks_cut,
        "noise_color_bright": noise_color,
        # Improved suggested controls
        "suggested_ks_sustain_ms": max(30.0, decay_ms * 1.2),  # shorter sustain
        "suggested_fm_index": max(0.1, trans_rms * 40.0),       # FM strength from transient energy
        "suggested_noise_level": max(0.1, trans_rms * 3.0),     # noise from transient RMS
        "suggested_fm_drop_ms": max(5.0, drop_time_ms * 0.8),   # FM pitch drop time
        "suggested_ks_cutoff": ks_cut,
        "suggested_ks_q": ks_q,
    }

    # Write JSON
    with open("synth_params.json", "w") as f:
        json.dump(synth, f, indent=2)

    # Print summary and C++ block
    print("=" * 60)
    print("DETAILED AUDIO ANALYSIS SUMMARY")
    print("=" * 60)
    
    print("\n--- PITCH ANALYSIS ---")
    print(f"  Pitch Range: {synth['minF']:.1f} - {synth['maxF']:.1f} Hz (median: {synth['medianF']:.1f} Hz)")
    print(f"  Pitch Contour: {synth['startF']:.1f} Hz (start)  {synth['endF']:.1f} Hz (end)")
    print(f"  Pitch Drop Time: {synth['pitch_drop_ms']:.1f} ms")
    
    print("\n--- ENVELOPE ANALYSIS ---")
    print(f"  Attack Time: {synth['attack_ms']:.2f} ms")
    print(f"  Decay Time: {synth['decay_ms']:.2f} ms")
    print(f"  Transient RMS Energy: {synth['transient_rms']:.4f}")
    print(f"  Damping Rate (lambda): {synth['damping_rate']:.3f} 1/s")
    
    print("\n--- SPECTRAL ANALYSIS ---")
    print(f"  Spectral Centroid (attack): {synth['spectral_centroid_start']:.1f} Hz")
    print(f"  High-Freq Ratio (>3kHz): {synth['hf_ratio_start']:.3f}")
    print(f"  Harmonic Ratio: {synth['harmonic_ratio']:.3f}")
    print(f"  Noise Color Brightness: {synth['noise_color_bright']:.3f}")
    
    print("\n--- KARPLUS-STRONG FILTER ESTIMATES ---")
    print(f"  Suggested KS Filter Cutoff: {synth['suggested_ks_cutoff']:.1f} Hz")
    print(f"  Suggested KS Resonance (Q): {synth['suggested_ks_q']:.3f}")
    
    print("\n--- SUGGESTED SYNTH PARAMETERS ---")
    print(f"  KS Sustain Time: {synth['suggested_ks_sustain_ms']:.0f} ms")
    print(f"  FM Index (pitchy attack strength): {synth['suggested_fm_index']:.2f}")
    print(f"  FM Pitch Drop Time: {synth['suggested_fm_drop_ms']:.0f} ms")
    print(f"  Noise Level (transient): {synth['suggested_noise_level']:.2f}")
    
    print("\n" + "=" * 60)
    print("ALL METRICS:")
    print("=" * 60)
    for k, v in synth.items():
        if isinstance(v, float):
            print(f"  {k}: {v:.4f}")
        else:
            print(f"  {k}: {v}")

    cpp_block = (
        "\n// ========== SYNTH PARAMETERS FROM analyze.py ==========\n"
        "// Paste this into main.cpp to match the original audio\n"
        "static constexpr float S_MIN_FREQ = %0.1ff;           // Hz (fundamental min)\n"
        "static constexpr float S_MAX_FREQ = %0.1ff;           // Hz (fundamental max)\n"
        "static constexpr int S_FM_DROP_MS = %d;              // ms (pitch drop time)\n"
        "static constexpr int S_KS_SUSTAIN_MS = %d;           // ms (body decay)\n"
        "static constexpr float S_FM_INDEX_SCALE = %0.2ff;     // FM modulation depth (0-1)\n"
        "static constexpr float S_NOISE_LEVEL_SCALE = %0.2ff;  // Transient noise level (0-1)\n"
        "\n// KS FILTER TUNING (update setKSDamping function):\n"
        "// Suggested KS cutoff base: %0.0f Hz\n"
        "// Suggested KS resonance Q: %0.3f (warmer=lower, metallic=higher)\n"
        "// HF Energy: %0.3f (higher = brighter)\n"
        "// ======================================================\n"
    ) % (
        synth["minF"], synth["maxF"], 
        int(synth["suggested_fm_drop_ms"]), 
        int(synth["suggested_ks_sustain_ms"]), 
        synth["suggested_fm_index"], 
        synth["suggested_noise_level"],
        synth["suggested_ks_cutoff"],
        synth["suggested_ks_q"],
        synth["hf_ratio_start"],
    )

    print("\nC++ SNIPPET TO PASTE:\n")
    print(cpp_block)


if __name__ == '__main__':
    main()

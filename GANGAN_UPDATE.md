# play.cpp - GAN-GAN Synthesis Update

## Changes Made

Updated `src/play.cpp` from **Playback-only** approach to **GAN-GAN FM Synthesis** with the new parameters.

### Key Modifications

#### 1. **GanGan Synth Class** (Embedded)
Embedded full `AudioSynthGanGan` class definition with the new parameters you provided:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `aStartFreq` | 600.0f | FM sweep start frequency (Hz) |
| `aStopFreq` | 6.0f | FM sweep stop frequency (Hz) |
| `aStartAmp` | 1.0f | FM start amplitude |
| `aStopAmp` | 0.2f | FM end amplitude |
| `aDuration` | 0.21f | FM sweep duration (s) |
| `bFreq` | 46.0f | Body fundamental frequency (Hz) |
| `bSubFreq` | 71.0f | Subharmonic frequency (Hz) |
| `bAmp` | 1.0f | Body amplitude |
| `bSubStd` | 0.2f | Subharmonic jitter amplitude |
| `envDurBase` | 0.62f | Envelope duration (s) |
| `envAttack` | 0.001f | Attack time (s) |
| `outputGain` | 0.4f | Default output gain |
| `fmDepthScale` | 0.65f | FM modulation depth |
| `sweepSpreadScale` | 0.60f | Pitch sweep spread factor |

#### 2. **Sensor Modulation** (Updated)

**Magnetic Reed Sensor (A1) → Base Frequency Selection**
- Digital open/closed state selects one of two body-frequency anchors
- Open state uses low base frequency, closed state uses high base frequency

**FSR Sensor (A0) → Output Gain**
- Range: [0, 1023] ADC → [0.4, 1.0] gain
- Inverted from playback approach: **pressed = louder**
- Acts as sustain/volume envelope control

**Piezo Sensor (A2) → Trigger**
- Rising-edge detection → calls `ganGan.noteOn()`
- Debounced 50ms between triggers

#### 3. **Reed Pitch Behavior**
The magnetic reed sensor now controls **discrete pitch selection** through base body frequency (`bFreq`) switching. This provides deterministic, low-noise control suitable for magnetic switching hardware.

#### 4. **Audio Memory & Latency**
- Reduced from 20 to 12 blocks (simpler synth, less memory needed)
- ISR polling: ~1-2ms
- Debounce: 50ms

---

## How to Test

1. **Upload** to Teensy 4.1
2. **Hit piezo** → Hear GanGan drum sound (600 Hz FM attack dropping to 6 Hz, 46 Hz harmonic body)
3. **Toggle reed state with magnet movement** → Pitch of drum should switch between low/high base frequencies
4. **Press FSR** → Sound gets louder (sustain/volume control)
5. **Check Serial** for real-time sensor values and active state

---

## If Reed Pitch Spread Needs Adjustment

Try modifying the loop:

### Option A: Map to FM Start Frequency
```cpp
if (reedPitchClosed) {
	ganGan.setBFreq(280.0f);
} else {
	ganGan.setBFreq(80.0f);
}
```

### Option B: Map to Both FM and Body
```cpp
ganGan.aStartFreq = reedPitchClosed ? 500.0f : 280.0f;
ganGan.setBFreq(reedPitchClosed ? 260.0f : 70.0f);
```

### Option C: Steeper Exponential Curve
```cpp
ganGan.setBFreq(reedPitchClosed ? 320.0f : 55.0f);
```

---

## Summary

✅ GanGan synth now running in `play.cpp` with your exact parameters  
✅ Magnetic reed → Base frequency selection (open/closed pitch bands)  
✅ FSR → Output gain (sustain control)  
✅ Piezo → Trigger noteOn  
⚠️ If pitch spread needs to be larger, tune open/closed frequency constants

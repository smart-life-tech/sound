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

**Flex Sensor (A1) → Base Frequency**
- Range: [FLEX_MIN=250, FLEX_MAX=3800] ADC → [40 Hz, 800 Hz]
- Exponential curve for musical pitch spacing
- Formula: `bFreq = 40 * 20^(flexNorm)` creates natural pitch sweep

**FSR Sensor (A0) → Output Gain**
- Range: [0, 1023] ADC → [0.4, 1.0] gain
- Inverted from playback approach: **pressed = louder**
- Acts as sustain/volume envelope control

**Piezo Sensor (A2) → Trigger**
- Rising-edge detection → calls `ganGan.noteOn()`
- Debounced 50ms between triggers

#### 3. **Why Flex Still Minimal on Sound**
The Flex sensor now modulates the **base body frequency (bFreq)** rather than a filter cutoff. To make it more audible:

- **Current mapping**: 40–800 Hz is subtle for a percussive sound (46 Hz is already in this range)
- **To enhance**: Try mapping to a narrower, more perceptible range like **30–100 Hz** or map to `aStartFreq` (FM sweep start) instead for more dramatic tonal change

#### 4. **Audio Memory & Latency**
- Reduced from 20 to 12 blocks (simpler synth, less memory needed)
- ISR polling: ~1-2ms
- Debounce: 50ms

---

## How to Test

1. **Upload** to Teensy 4.1
2. **Hit piezo** → Hear GanGan drum sound (600 Hz FM attack dropping to 6 Hz, 46 Hz harmonic body)
3. **Bend Flex sensor** → Pitch of drum should change (deep = 40 Hz, high = 800 Hz)
4. **Press FSR** → Sound gets louder (sustain/volume control)
5. **Check Serial** for real-time sensor values and active state

---

## If Flex Still Not Dramatic Enough

Try modifying the loop:

### Option A: Map to FM Start Frequency
```cpp
float flexFreq = 300.0f + (300.0f * flexNorm);  // 300–600 Hz FM start
ganGan.aStartFreq = flexFreq;
```

### Option B: Map to Both FM and Body
```cpp
ganGan.aStartFreq = 300.0f + (300.0f * flexNorm);
ganGan.setBFreq(30.0f + (70.0f * flexNorm));  // 30–100 Hz body
```

### Option C: Steeper Exponential Curve
```cpp
float flexFreq = 30.0f * powf(26.67f, flexNorm);  // 30–800 Hz, more aggressive
ganGan.setBFreq(flexFreq);
```

---

## Summary

✅ GanGan synth now running in `play.cpp` with your exact parameters  
✅ Flex → Base frequency (exponential 40–800 Hz)  
✅ FSR → Output gain (sustain control)  
✅ Piezo → Trigger noteOn  
⚠️ Flex modulation subtle; adjust mapping if needed for more obvious effect

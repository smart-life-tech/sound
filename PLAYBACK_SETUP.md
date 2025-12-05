# Playback-Based Drum Instrument Setup

## Architecture Overview

This firmware reconfigures the drum instrument from **synthesis-driven** (FM+KS+Noise) to **playback-driven** with real-time sensor modulation.

### Primary Sound Source
- **AudioPlayMemory** plays `drum_raw` (the authentic original audio sample)
- Triggered on piezo hit (rising-edge detection)
- Prevents overlapping playback (debounced)

### Real-Time Sensor Modulation

| Sensor | Pin | Function | Mapping |
|--------|-----|----------|---------|
| **Piezo** | A2 | Trigger playback | Rising edge → play `drum_raw` |
| **Flex** | A1 | Brightness control | [250, 3800] → Filter cutoff [500 Hz, 4000 Hz] |
| **FSR** | A0 | Sustain control | [0, 1023] → Playback volume [1.0, 0.0] (inverted) |

## Audio Chain

```
Piezo (trigger) → ISR → playMem.play(drum_raw)
                           ↓
                        filter (Flex-modulated cutoff)
                           ↓
                        mixer (FSR-modulated gain)
                           ↓
                        i2s1 (stereo output)
```

## Key Differences from Synthesis

| Aspect | Synthesis | Playback |
|--------|-----------|----------|
| **Sound source** | FM+KS+Noise per hit | Authentic sample per hit |
| **Fidelity** | Approximation | High (original recording) |
| **Computational load** | High (3 synth chains) | Low (simple playback + filter) |
| **Warmth/timbre** | Requires careful tuning | Guaranteed (original preserved) |
| **Control responsiveness** | Fast synth update | Real-time filter/volume modulation |

## Behavior

### On Piezo Hit
1. Edge detector triggers on rising threshold (250 ADC)
2. Plays `drum_raw` sample (6526 samples @ 44.1 kHz ≈ 0.73s)
3. Subsequent hits debounced 50ms
4. Only one playback active at a time (no overlapping)

### Flex Sensor (Brightness)
- **Pressed (lower resistance)** → Dark tone (500 Hz cutoff)
- **Released (higher resistance)** → Bright tone (4000 Hz cutoff)
- Smooth filter sweep via exponential smoothing

### FSR Sensor (Sustain)
- **Not pressed (FSR = 0)** → Full volume (gain = 1.0)
- **Pressed (FSR = 1023)** → Muted (gain = 0.0)
- Acts as **dynamic volume damping** during playback
- Can cut off sound mid-play if pressed hard

## LED/Debug Output

Serial output every 500ms:
```
Piezo: 0345 | Flex: 2450 (cut:  2250 Hz) | FSR: 0123 (gain: 0.88) | Playing: 1
```

Manual trigger commands:
- `p` : Force play `drum_raw`
- `s` : Stop playback
- `?` : Help

## Audio Memory & Latency

- **AudioMemory**: 20 blocks (5ms audio latency at 44.1 kHz)
- **ISR timing**: Called every loop iteration (~1-2ms polling rate)
- **Debounce**: 50ms (minimum time between hits)

## Expected Sound Characteristics

✓ **Authentic original audio timbre** (no metallic artifacts)
✓ **Expressive control** (Flex for tone, FSR for muting)
✓ **Clean single hits** (peak-tracking prevents double-trigger)
✓ **Responsive** (real-time filter/volume modulation)

## Modification Points (if needed later)

1. **Change playback sample**: Replace `drum_raw` reference in `piezoISR()`
2. **Adjust filter range**: Edit `flexCutoff` calculation (currently 500–4000 Hz)
3. **Change FSR behavior**: Modify gain mapping (currently inverted volume)
4. **Add velocity sensitivity**: Map `piezoCenterPeak` to playback speed/amplitude
5. **Looping playback**: Uncomment auto-replay in main loop

## Next Steps

1. **Compile & upload** to Teensy 4.1
2. **Test piezo hit** → Should hear original drum sample
3. **Test Flex sensor** → Tone should brighten/darken
4. **Test FSR sensor** → Sound should fade/mute when pressed
5. **Iterate modulation ranges** if behavior doesn't match expectations

---

**Status**: Playback-focused architecture ready for testing.  
**Replaces**: Synthesis-only approach (FM+KS+Noise).  
**Rationale**: Original audio fidelity + real-time sensor control.

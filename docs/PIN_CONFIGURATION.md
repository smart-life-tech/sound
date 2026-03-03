# Pin Configuration Mapping

## 1. Active Firmware Pin Map (`src/play.cpp`)

| Function | Symbol | MCU Pin | Type | Direction | Notes |
|---|---|---|---|---|---|
| Piezo trigger input | `PIEZO_CENTER_PIN` | `A2` | Analog ADC | Input | Hit detection and velocity estimation |
| Flex sensor input | `FLEX_PIN` | `A1` | Analog ADC | Input | Controls synth base frequency (`bFreq`) |
| FSR sensor input | `FSR_PIN` | `A0` | Analog ADC | Input | Controls synth fade time (`fadeTime`) |
| Audio output (I2S TX) | `i2s1` | Teensy I2S pins (board-defined) | Digital serial | Output | Routed to SGTL5000-based audio path |
| USB serial debug | `Serial` | USB virtual COM | USB CDC | Bi-directional | Debug telemetry at 115200 baud |

## 2. Configuration Constants

| Constant | Current Value | Purpose |
|---|---:|---|
| `PIEZO_THRESHOLD` | `200` | Minimum ADC value to qualify as hit |
| `PIEZO_DEBOUNCE_MS` | `1` | Debounce constant (legacy; lockout logic is primary) |
| `FLEX_MIN` | `0` | Lower bound for flex normalization |
| `FLEX_MAX` | `0.8` | Upper bound for flex normalization (review recommended) |
| `FLEX_SMOOTH_ALPHA` | `0.24` | Exponential smoothing coefficient for flex input |

## 3. Ownership and Change Control

- Pin assignments are currently defined in `src/play.cpp`.
- If pin mappings move to a board profile header, update this file and release notes in the same change.
- Any hardware reroute must include:
  1. pin map update,
  2. BOM alignment update,
  3. deployment validation note.

## 4. Validation Checklist (After Pin Changes)

- Confirm analog channels read expected ranges in Serial telemetry.
- Confirm piezo lockout/re-arm behavior does not cause retrigger or missed hits.
- Confirm I2S output remains audible on both channels.
- Confirm no pin conflicts with manufacturing harnesses.

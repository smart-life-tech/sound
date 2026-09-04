# Pin Configuration Mapping

## 1. Active Firmware Pin Map (`src/play_adxl345.cpp`)

| Function | Symbol | MCU Pin | Type | Direction | Notes |
|---|---|---|---|---|---|
| Piezo trigger input | `PIEZO_CENTER_PIN` | `A2` | Analog ADC | Input | Hit detection and velocity estimation |
| Analog Hall pitch input | `HALL_PITCH_PIN` | `A1` | Analog ADC | Input | Modulates synth base frequency (`bFreq`) |
| FSR sensor input | `FSR_PIN` | `A0` | Analog ADC | Input | Controls synth fade time (`fadeTime`) |
| Volume potentiometer | `VOLUME_POT_PIN` | `A3` | Analog ADC | Input | Master volume |
| Treble potentiometer | `TREBLE_POT` | `A4` | Analog ADC | Input | Brightness/filter cutoff |
| Bass potentiometer | `BASS_POT` | `A5` | Analog ADC | Input | Base frequency |
| ADXL345 SDA | `Wire` | `18` | I2C | Bi-directional | ADXL345 data; I2C mode |
| ADXL345 SCL | `Wire` | `19` | I2C | Output | ADXL345 clock; I2C mode |
| ADXL345 CS | n/a | `3.3V` | Mode select | Input | Must be high for I2C mode |
| ADXL345 SD0 | n/a | `GND` | Address select | Input | Selects I2C address `0x53` |
| ADXL345 INT1/INT2 | n/a | Not connected | Interrupt | Unused | Hit detection is polled over I2C |
| Audio output (I2S TX) | `i2s1` | Teensy I2S pins (board-defined) | Digital serial | Output | Routed to SGTL5000-based audio path |
| USB serial debug | `Serial` | USB virtual COM | USB CDC | Bi-directional | Debug telemetry at 115200 baud |

## 2. Configuration Constants

| Constant | Current Value | Purpose |
|---|---:|---|
| `PIEZO_THRESHOLD` | `200` | Minimum ADC value to qualify as hit |
| `PIEZO_DEBOUNCE_MS` | `1` | Debounce constant (legacy; lockout logic is primary) |
| `HALL_MIN_ADC` | `0.0` | Lower bound for Hall normalization |
| `HALL_MAX_ADC` | `1023.0` | Upper bound for Hall normalization |
| `HALL_SMOOTH_ALPHA` | `0.20` | Exponential smoothing coefficient for Hall input |
| `HALL_FREQ_MIN_HZ` | `70.0` | Minimum mapped base frequency |
| `HALL_FREQ_MAX_HZ` | `250.0` | Maximum mapped base frequency |
| `ADXL_I2C_ADDRESS` | `0x53` | ADXL345 address when SD0 is grounded |
| `ADXL_RATE_200HZ` | `0x0C` | ADXL345 output data rate |
| `gAdxlThreshold` | `0.35 g` | Baseline-relative filtered hit threshold |
| `ADXL_HIT_COOLDOWN_MS` | `80` | Minimum interval between accelerometer hits |
| `ADXL_REARM_RATIO` | `0.45` | Re-arm level as a fraction of hit threshold |

## 3. Ownership and Change Control

- Pin assignments are currently defined in `src/play.cpp`.
- If pin mappings move to a board profile header, update this file and release notes in the same change.
- Any hardware reroute must include:
  1. pin map update,
  2. BOM alignment update,
  3. deployment validation note.

## 4. Validation Checklist (After Pin Changes)

- Confirm Hall ADC field changes smoothly in Serial telemetry.
- Confirm piezo lockout/re-arm behavior does not cause retrigger or missed hits.
- Confirm I2S output remains audible on both channels.
- Confirm no pin conflicts with manufacturing harnesses.

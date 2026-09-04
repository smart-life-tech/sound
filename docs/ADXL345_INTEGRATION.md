# ADXL345 Integration Guide

## File Location
- **New file**: `src/play_adxl345.cpp` — Complete rewrite with ADXL345 support
- **Old file**: `src/play.cpp` — Original piezo-only version (kept for reference)

## Quick Start

### 1. Choose Communication Protocol
Edit the `#define` section at the top of `play_adxl345.cpp`:

```cpp
// Choose ONE protocol:
#define ADXL_USE_I2C      1    // Set to 1 for I2C, 0 for SPI
#define ADXL_USE_SPI      0    // Set to 1 for SPI, 0 for I2C
```

### 2. Choose Integration Mode
```cpp
// Choose ONE integration mode:
#define ADXL_DUAL_INPUT   1    // 1 = use both piezo AND ADXL345, 0 = ADXL345 only
#define ADXL_ENABLED      1    // 1 = enable ADXL345, 0 = disable entirely
```

### 3. I2C Wiring (Teensy 4.x)
```
ADXL345 Pin    Teensy 4.x Pin    Notes
──────────────────────────────────────────
VCC            3.3V              Power
GND            GND               Ground
SDA (Data)     Pin 18 (A4)       I2C Data (Wire.h default)
SCL (Clock)    Pin 19 (A5)       I2C Clock (Wire.h default)
SD0            GND               Slave addr = 0x53 (default)
               or VCC            or Slave addr = 0x1D (alternate)
INT1           (optional)        Interrupt output
INT2           (optional)        Interrupt output
```

### 4. SPI Wiring (Teensy 4.x)
```
ADXL345 Pin    Teensy 4.x Pin    Notes
──────────────────────────────────────────
VCC            3.3V              Power
GND            GND               Ground
MOSI           Pin 11            SPI Data In
MISO           Pin 12            SPI Data Out
SCL (Clock)    Pin 13            SPI Clock
CS             Pin 10            Chip Select (ADXL_CS_PIN)
INT1           (optional)        Interrupt output
INT2           (optional)        Interrupt output
```

## Hardware Pin Summary

### Analog Sensors (ADC)
- **A0** = FSR (Force Sensitive Resistor)
- **A1** = Hall effect sensor (pitch control)
- **A2** = Piezo hit trigger
- **A3** = Volume pot
- **A4** = Treble pot
- **A5** = Bass pot

### I2C (for ADXL345 in I2C mode)
- **Pin 18** = SDA (data)
- **Pin 19** = SCL (clock)

### SPI (for ADXL345 in SPI mode)
- **Pin 11** = MOSI (data in)
- **Pin 12** = MISO (data out)
- **Pin 13** = SCL (clock)
- **Pin 10** = CS (chip select, for ADXL345)

## Serial Commands

### Hit Testing
- **`h`** → Toggle hit-test logging on/off (prints [HIT_TEST] messages)
- **`3`** → Select ADXL334 calibration profile
- **`4`** → Select ADXL345 calibration profile (default)
- **`+`** → Increase ADXL threshold by 0.05g (range: 0.1g to 2.0g)
- **`-`** → Decrease ADXL threshold by 0.05g

### General Controls
- **`c`** → Recalibrate the ADXL345 baseline for the current orientation (10 seconds)
- **`1`** → Toggle high-mid resonance
- **`2`** → Toggle fast high-decay
- **`5`** → Toggle tanh saturation

### Recalibrate for a New Orientation
The hit detector measures movement relative to the device's stationary gravity vector, so it works regardless of which direction the device is pointing.

Automatic recalibration is enabled. When the device is held still at least 90 degrees away from the previous orientation for 10 seconds, the firmware averages that stationary gravity vector and adopts it as the new baseline. It tolerates normal sensor noise up to 0.12g between samples, so leaving the device still or making a small tilt does not recalibrate it.

1. Hold the device still in the desired orientation.
2. Keep it still for the full 10-second calibration. No serial command is required.

The new X/Y/Z gravity vector becomes the baseline. Hits are then calculated from the change away from that orientation, rather than from absolute acceleration. Do not move the device during calibration.

Manual recalibration is also available: press **`c`**, then hold the device still for 10 seconds regardless of how far the orientation changed.

## Dual-Input Mode (ADXL + Piezo)

When `ADXL_DUAL_INPUT = 1`, a hit is triggered if **either**:
- Piezo sensor exceeds `PIEZO_THRESHOLD` (190), **OR**
- ADXL345 acceleration exceeds `gAdxlThreshold` (default 0.5g)

Use `+` and `-` commands to adjust the ADXL threshold in real-time.

## ADXL-Only Mode (ADXL Exclusive)

When `ADXL_DUAL_INPUT = 0`, the piezo is ignored. Hits are triggered only by ADXL345 acceleration.

## Testing Workflow

### Step 1: Verify Communication
1. Upload the code with `ADXL_ENABLED = 1`
2. Open Serial Monitor (115200 baud)
3. Look for output: `ADXL345 Device ID: 0xE5 (expected 0xE5)`
   - If you see `0xE5`, communication is working
   - If you see a different value or `WARNING`, check wiring

### Step 2: Enable Hit Test Logging
1. Press **`h`** in Serial Monitor
2. Output changes to `[HIT_TEST] model=...` format
3. Hit the piezo/drum and watch the log:
   ```
   [HIT_TEST] model=ADXL345 peak=450 vel=0.456 intensity=0.420 fsr=50 accel=1.23g hall=100.0
   ```

### Step 3: Calibrate ADXL Threshold
1. In dual-input mode, tap softly and watch the acceleration value
2. Press **`+`** or **`-`** to adjust threshold until it matches your soft taps
3. Default `0.5g` works for most drum setups

### Step 4: Switch Models
1. Press **`3`** to test ADXL334 profile (1.0x gain)
2. Press **`4`** to test ADXL345 profile (1.3x gain)
3. Compare how hit intensity is calculated differently

## Configuration Guide

### Sensitivity (Model Selection)
- **ADXL334**: `1.00x` gain (less sensitive)
- **ADXL345**: `1.30x` gain (more sensitive, default)

These are calibration profiles only; they don't change the physical sensor. Use `3` and `4` to test different scaling.

### Acceleration Threshold
- **Default**: `0.5g` (good for drum pads)
- **Soft hits**: `0.2g` to `0.4g`
- **Firm hits**: `0.6g` to `1.0g`
- **Hard hits**: `1.0g` to `2.0g`

Adjust with `+` and `-` keys in real-time.

## Advanced Configuration

### Change I2C Address
If your ADXL345 has `SD0` connected to VCC (alternate address):
```cpp
#define ADXL_I2C_ADDRESS     ADXL_I2C_ADDR_HIGH  // 0x1D instead of 0x53
```

### Change SPI Clock Speed
For noisy environments, lower the SPI clock:
```cpp
SPI.setClockDivider(SPI_CLOCK_DIV16);  // ~5.25 MHz (slower, more stable)
```

### Change Hit Test Output Format
Edit the debug print statement in `piezoISR()` to include or exclude fields.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Device ID is not 0xE5 | Check I2C/SPI wiring; verify SD0 pin; check Wire/SPI clock setup |
| No acceleration readings | Verify power supply (3.3V); check SDA/SCL pull-up resistors (≈4.7kΩ typical) |
| Threshold adjustments don't work | Press `+` or `-` and check Serial output for new threshold value |
| Hits detected by ADXL but not piezo | Adjust `PIEZO_THRESHOLD` value; check piezo wiring and sensor condition |
| Random hit triggers | Increase threshold value (`+` key); reduce electrical noise near sensors |

## File Comparison

| Feature | `play.cpp` (Old) | `play_adxl345.cpp` (New) |
|---------|-----------------|------------------------|
| Piezo support | ✓ | ✓ |
| ADXL345 I2C | ✗ | ✓ |
| ADXL345 SPI | ✗ | ✓ |
| Model selector | ✗ | ✓ (3/4 keys) |
| Dual-input mode | ✗ | ✓ |
| Threshold adjust | ✗ | ✓ (+/- keys) |
| Hit test logging | ✗ | ✓ (h key) |

## To Use This New Version

1. **Backup your current `play.cpp`** (already done; it's still there)
2. **Rename** `play_adxl345.cpp` → `play.cpp`
3. **Edit** `platformio.ini` to ensure `src/play.cpp` is compiled
4. **Build and upload** with PlatformIO
5. **Test** using Serial Monitor commands

## Questions / Debugging

Enable ALL debugging with:
```cpp
#define DEBUG_ADXL 1
```

Then add detailed prints in the `adxlReadAccel()` and `piezoISR()` functions.

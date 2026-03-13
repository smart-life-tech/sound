# Debugging and Troubleshooting Guide

## 1. Debug Entry Points

- Serial diagnostics in `src/play.cpp` (`Serial.printf` blocks)
- PlatformIO build output and linker diagnostics
- Runtime sensor telemetry (piezo/Hall/FSR fields)

## 2. Fast Triage Flow

1. **No sound**
   - Confirm firmware upload success.
   - Confirm Audio Shield enabled in setup.
   - Confirm physical audio output path.
2. **No trigger on hit**
   - Observe piezo raw value in telemetry.
   - Tune `PIEZO_THRESHOLD` and check piezo wiring.
3. **Double trigger / false trigger**
   - Verify lockout behavior and quiet-zone re-arm.
   - Improve piezo mechanical damping and wiring noise isolation.
4. **Hall pitch modulation not responding**
   - Validate Hall value changes in telemetry (`Hall: ####`).
   - Verify Hall analog output wiring, power, and magnetic field orientation.
5. **FSR sustain not responsive**
   - Check `currentFSR` changing over full range.
   - Adjust fade mapping limits.

## 3. Build/Toolchain Issues

### Symptom: Linker errors for ARM/CMSIS DSP libraries

- Confirm `lib_ignore` entries in `platformio.ini` are intact.
- Clean and rebuild:

```bash
pio run -t clean
pio run
```

### Symptom: Upload failures

- Verify USB cable/data path.
- Retry with manual boot/program mode.
- Ensure `teensy-cli` upload path is available.

## 4. Runtime Stability Checks

- Ensure `AudioMemory` is adequate for workload.
- Avoid long blocking delays in `loop()`.
- Keep Serial print interval rate-limited.

## 5. Evidence to Capture in Issue Reports

- Firmware version and build type.
- Board revision and sensor configuration.
- Full build log (if compile/link issue).
- 10–20 seconds of Serial telemetry around failure.
- Reproduction steps and expected vs actual behavior.

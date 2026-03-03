# Integration Notes with Hardware Modules

## 1. Hardware Modules in Active Configuration

- Teensy 4.1 MCU board
- Piezo sensor module (trigger source)
- Flex sensor module (pitch/body-frequency modulation)
- FSR module (fade/sustain modulation)
- Audio output chain through Teensy I2S + SGTL5000 control path

## 2. Electrical Integration Constraints

- Sensor grounds must be common with Teensy ground.
- Analog sensor wiring should be short and noise-managed (routing away from high-current switching lines).
- Piezo input conditioning should include practical protection in hardware where needed.
- Use stable power rails for analog sensors to reduce jitter and false triggering.

## 3. Firmware-to-Hardware Coupling Points

- Piezo threshold logic (`PIEZO_THRESHOLD`) is hardware-dependent and may require per-batch calibration.
- Flex smoothing (`FLEX_SMOOTH_ALPHA`) and normalization bounds should be calibrated against installed sensor lot.
- FSR fade mapping (`0.1s` to `2.0s`) should be validated against desired playing feel.

## 4. Integration Test Procedure

1. Boot device and confirm Serial startup logs.
2. Verify idle sensor values are stable.
3. Strike piezo repeatedly and verify one trigger per hit.
4. Sweep flex sensor and verify base frequency changes.
5. Apply FSR pressure and verify fade extension.
6. Confirm stereo output audibility and absence of clipping artifacts.

## 5. Manufacturing/Bring-up Notes

- Keep a per-unit calibration sheet for piezo threshold and sensor baselines.
- Record board revision and firmware version together.
- If hardware substitutions are introduced, update `BOM_ALIGNMENT.md` before production release.

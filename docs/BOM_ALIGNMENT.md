# Bill of Materials (BOM) Alignment

## 1. Objective

Ensure firmware configuration and supported hardware are aligned with the approved BOM, including component dependencies, MCU specs, and substitution controls.

## 2. Core MCU Specification (From Active Build Config)

- Board target: Teensy 4.1 (`board = teensy41`)
- MCU family class: ARM Cortex-M7
- Float ABI: hard-float (`-mfloat-abi=hard`)
- FPU target: `fpv5-d16`
- Build framework: Arduino (Teensy platform)

## 3. Firmware-Dependent Hardware Components

| BOM Group | Component Type | Firmware Dependency |
|---|---|---|
| MCU board | Teensy 4.1 | Required by `env:teensy41` build target |
| Audio output path | SGTL5000-compatible Teensy audio path | Required by `AudioControlSGTL5000` + I2S output |
| Sensor input | Piezo sensor | Required for hit trigger |
| Sensor input | Magnetic reed sensor | Required for pitch/body-frequency selection |
| Sensor input | FSR sensor | Required for fade/sustain modulation |

## 4. Software Component Dependencies

- Arduino core via PlatformIO Teensy platform
- Teensy Audio library APIs (`Audio.h`, `AudioStream`, `AudioOutputI2S`, `AudioControlSGTL5000`)
- No active dependency on CMSIS-DSP runtime libraries in current config (`lib_ignore` excludes CMSIS-related packages)

## 5. Substitution and Revision Tracking

Current status: **No substitutions formally recorded in this document yet.**

If a component substitution is introduced, record:

1. Original BOM part and revision
2. New part and revision
3. Electrical/functional differences
4. Firmware impact (pin map, scaling, timing, thresholds)
5. Validation evidence and sign-off

## 6. Manufacturing-Ready Documentation Support

Before manufacturing release, ensure all of the following are completed:

- [ ] Pin map validated against harness/wiring drawings
- [ ] Sensor calibration envelope documented
- [ ] Firmware version tied to BOM revision
- [ ] Substitutions logged with approval
- [ ] Stable release notes published

# Setup and Deployment Guide

## 1. Prerequisites

- PlatformIO extension or CLI installed.
- Teensy 4.1 connected via USB.
- Project root opened in workspace.

## 2. Environment Configuration

Primary environment: `env:teensy41` in `platformio.ini`.

Key configuration:

- `platform = teensy`
- `board = teensy41`
- `framework = arduino`
- `upload_protocol = teensy-cli`
- custom linker script: `linkerscript.ld`

## 3. Build Process

From project root:

```bash
pio run
```

Expected result:

- successful compile/link for Teensy 4.1,
- generated firmware artifact in `.pio/build/teensy41/`.

## 4. Flash / Upload Process

```bash
pio run -t upload
```

If Teensy is not automatically detected:

- press program button on board,
- re-run upload command.

## 5. Post-Deployment Validation

1. Open monitor:

```bash
pio device monitor -b 115200
```

2. Confirm boot log appears.
3. Hit piezo and verify `HIT` trigger print.
4. Move flex sensor and verify frequency field updates.
5. Apply FSR pressure and verify fade field updates.

## 6. Deployment Record (Required)

For each deployment, record:

- firmware version,
- build type (`stable` or `experimental`),
- board revision,
- deployment date,
- operator,
- smoke-test result summary.

## 7. Rollback Procedure

1. Identify last known-good stable version from release notes.
2. Rebuild that tagged source state.
3. Upload and re-run post-deployment validation.
4. Record rollback in release notes and issue log.

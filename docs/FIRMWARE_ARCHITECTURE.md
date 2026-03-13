# Firmware Architecture Documentation

## 1. Firmware Purpose

The `genesis` firmware runs on Teensy 4.1 and implements a sensor-triggered percussion engine based on a custom `AudioSynthGanGan` synthesis block. It reads sensor inputs (piezo analog, magnetic reed digital, FSR analog), detects hit events, and drives real-time sound generation through the Teensy audio stack.

Primary implementation file: `src/play.cpp`

## 2. Runtime Architecture

### 2.1 Processing Model

- **Control loop (`loop`)**
  - Polls sensor ISR routine (`piezoISR`) in software.
  - Applies modulation mapping:
    - Magnetic reed sensor → base body frequency selection (`bFreq`)
    - FSR sensor → fade time (`fadeTime`)
  - Emits periodic debug telemetry over Serial.

- **Audio engine (`AudioStream::update`)**
  - Runs in audio callback context at Teensy audio block cadence.
  - Generates sample blocks using:
    - FM sweep attack oscillator
    - Body oscillator
    - Subharmonic jitter oscillator
    - Envelope + fade shaping
  - Routes mono synth output to stereo I2S L/R outputs.

### 2.2 Signal Path

1. Piezo threshold crossing is detected.
2. `ganGan.noteOn(velocity)` is triggered.
3. Synth engine generates audio in `update()`.
4. `AudioConnection` routes output to `AudioOutputI2S` channels 0 and 1.
5. Audio Shield codec drives analog output.

## 3. Firmware Modules and Responsibilities

### 3.1 Sensor Front-End

- Reads analog values from:
  - Piezo (`PIEZO_CENTER_PIN`)
  - Magnetic reed (`REED_PITCH_PIN`)
  - FSR (`FSR_PIN`)
- Performs hit-arm/re-arm logic and snapshots reed state.
- Applies lockout to reduce piezo bounce retriggering.

### 3.2 Trigger/Velocity Logic

- Implements threshold-based event generation.
- Maintains `armed` state with quiet-zone reset.
- Computes bounded velocity from smoothed piezo value.

### 3.3 Synthesis Core (`AudioSynthGanGan`)

- Parameters include FM sweep (`aStartFreq`, `aStopFreq`, etc.) and body dynamics (`bFreq`, `bSubFreq`, `bSubStd`).
- Envelope control via `envDur` and `fadeTime`.
- Run-time controls:
  - `setBFreq(float)`
  - `setFadeTime(float)`
  - `noteOn(float velocity)`

### 3.4 Audio Output Layer

- Teensy Audio objects:
  - `AudioOutputI2S i2s1`
  - `AudioControlSGTL5000 audioShield`
- Codec initialization and output volume are handled in `setup()`.

## 4. Build and Platform Architecture

From `platformio.ini`:

- Target board: `teensy41`
- Platform/framework: `teensy` / `arduino`
- Upload protocol: `teensy-cli`
- CPU/FPU flags: Cortex-M7, hard-float, `fpv5-d16`
- Custom linker script: `linkerscript.ld`

## 5. Continuation Notes for New Engineers

When extending firmware:

1. Keep control-path work in `loop()` lightweight and non-blocking.
2. Keep all heavy DSP within `AudioStream::update()`.
3. Preserve lockout + re-arm logic for piezo reliability.
4. Validate sensor mapping ranges after hardware changes.
5. Record every behavior-impacting change in release notes.

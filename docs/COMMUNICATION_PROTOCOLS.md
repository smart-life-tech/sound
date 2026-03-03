# Communication Protocols Documentation

## 1. Protocol Overview

The current firmware uses three interface categories:

1. **Analog sensor acquisition** (piezo, flex, FSR)
2. **Digital audio transport** through Teensy Audio I2S output
3. **USB Serial debug telemetry**

No network stack, BLE, CAN, or UART-based external module protocol is implemented in the active `play.cpp` path.

## 2. Analog Sensor Interface

- Interface type: single-ended analog voltage read through ADC.
- Sampling model: polled via `piezoISR()` from `loop()`.
- Conditioning used in firmware:
  - thresholding,
  - smoothing,
  - lockout and re-arm gating.

## 3. I2S Audio Interface

- Audio graph outputs synthesized signal with `AudioOutputI2S`.
- Mono synth stream is patched to both left and right output channels.
- Intended downstream endpoint: SGTL5000-compatible codec/audio shield path.

## 4. USB Serial Protocol

- Transport: USB CDC Serial.
- Configured monitor speed: `115200` (development monitor setting).
- Payload: human-readable diagnostic lines containing piezo, flex, FSR, frequency, fade, and active state.
- Message framing: newline-delimited plaintext (`Serial.printf`).

## 5. Timing and Reliability Notes

- Hit lockout window is used to avoid bounce retrigger behavior.
- Sensor telemetry is rate-limited in the main loop (~500 ms interval).
- Audio generation is callback-driven and independent of Serial throughput.

## 6. Protocol Change Requirements

Any protocol-level modification must include:

1. update to this document,
2. update to integration notes,
3. regression check results in release notes.

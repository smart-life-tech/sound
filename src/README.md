# Teensy 4.x Hybrid Drum Synth

This project implements a hybrid drum synthesizer on the Teensy 4.x microcontroller, combining FM synthesis for attack transients, Karplus-Strong algorithm for resonant body simulation, and noise bursts for added realism. The synth responds to physical sensors for dynamic, expressive drum sounds.

## Features

- **FM Attack with Pitch Drop**: Frequency modulation synthesis for sharp, bright attack sounds with a decaying pitch envelope.
- **Karplus-Strong Resonant Body**: Plucked string algorithm for sustained, resonant drum body tones.
- **Noise Burst Transient**: White noise envelope for added texture and punch.
- **Center vs Rim Detection**: Different note blocks and timbres based on hit location (center vs rim piezo sensors).
- **Magnetic Reed Pitch Input**: Switches between two base-frequency targets for pitch variation.
- **FSR Sensor Control**: Controls attenuation and decay of the Karplus-Strong body for dynamic damping.
- **Real-Time Processing**: Uses FreeRTOS for task scheduling and an ISR for high-frequency sensor sampling (5 kHz).
- **Audio Output**: Stereo output via I2S to the Teensy Audio Shield.

## Hardware Requirements

- **Teensy 4.x** (e.g., Teensy 4.0 or 4.1)
- **Teensy Audio Shield** (SGTL5000-based)
- **Sensors**:
  - Piezo sensors for center (A0) and rim (A3) detection
  - Magnetic reed sensor (A1) for pitch selection
  - Force Sensitive Resistor (FSR) (A2) for decay control
- **Connections**:
  - Piezo Center: A0
  - Piezo Rim: A3
  - Magnetic reed: A1
  - FSR: A2
- **Power Supply**: Appropriate for Teensy and sensors

## Software Dependencies

- **Arduino IDE** with Teensyduino installed
- **Libraries**:
  - Audio (included in Teensyduino)
  - FreeRTOS (included in Teensyduino)
- **PlatformIO** (optional, for alternative build environment)

## Setup and Installation

1. **Install Teensyduino**: Download and install the Arduino IDE with Teensyduino support from [PJRC](https://www.pjrc.com/teensy/teensyduino.html).

2. **Connect Hardware**: Wire the sensors to the specified analog pins on the Teensy.

3. **Upload Code**:
  - Open `src/play.cpp` in Arduino IDE.
   - Select the correct Teensy board and port.
   - Compile and upload to the Teensy.

4. **Audio Output**: Connect headphones or speakers to the Audio Shield's output.

## How It Works

### Sensor Processing
- **Piezo ISRs**: Samples center and rim piezos at 5 kHz to detect peaks and debounce hits.
- **Reed State Capture**: Latches reed open/closed state for deterministic pitch selection.
- **FSR Reading**: Captures FSR value for decay control during each hit.

### Audio Synthesis
- **Trigger Logic**: On hit detection, snapshots sensor values and triggers synthesis components.
- **Note Selection**: Based on rim/center and flex position, selects from 8 notes (C4 to C5).
- **FM Attack**: Modulated carrier with pitch envelope for dynamic attack.
- **KS Body**: Filtered Karplus-Strong with velocity and FSR-controlled damping.
- **Noise Transient**: Velocity-scaled noise burst for added snap.

### Real-Time Tasks
- **PlayTask**: FreeRTOS task handles synthesis triggering and audio updates.
- **Main Loop**: Updates pitch envelopes at ~500 Hz.

## Configuration

Key constants are defined at the top of `src/play.cpp`:

- **Thresholds and Sensor States**: Adjust `PIEZO_THRESHOLD` and reed active/open/closed constants for calibration.
- **Audio Gains**: Modify mixer gains in `setup()` for balance.
- **Synthesis Parameters**: Tune FM indices, noise amplitudes in helper functions.

## Usage

- **Playing**: Hit the piezos (center/rim) to trigger sounds. Toggle the magnetic reed state to change pitch band. Press the FSR for shorter/longer decay.
- **Calibration**: Monitor Serial output for sensor values and adjust thresholds as needed.
- **Debugging**: Serial prints status and errors.

## File Structure

- `src/play.cpp`: Main synthesizer code with setup, loop, ISR polling, and synthesis functions.

## Notes

- Ensure AudioMemory is set appropriately (currently 32 blocks).
- The synth uses 12-bit ADC resolution for better sensor precision.
- FreeRTOS prioritizes the PlayTask for low-latency response.

## License

This project is open-source. Refer to the repository's license file for details.

## Contributing

Feel free to submit issues or pull requests for improvements.

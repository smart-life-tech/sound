/*
   Teensy 4.x PLAYBACK WITH REAL-TIME SENSOR MODULATION
   - Primary: AudioPlayMemory plays drum_raw (authentic sample)
   - Modulation: 
     * Flex sensor (A1) → Filter cutoff (brightness)
     * FSR sensor (A0) → Playback volume (sustain shortening)
     * Piezo sensor (A2) → Trigger playback on hit
   - Result: Original audio fidelity + expressive sensor control
*/

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

#include "drum_raw.h" // AudioPlayMemory-compatible header

#define analogReadFast(pin) analogRead(pin)

// ------------------- PINS -------------------
#define PIEZO_CENTER_PIN A2
#define FLEX_PIN A1
#define FSR_PIN A0

// ------------------- CONFIG -------------------
#define PIEZO_THRESHOLD 250
#define PIEZO_DEBOUNCE_MS 50

#define FLEX_MIN 250
#define FLEX_MAX 3800
#define FLEX_SMOOTH_ALPHA 0.24f

// =============== AUDIO OBJECTS ===============
AudioPlayMemory playMem;
AudioFilterStateVariable filter;
AudioMixer4 mixer;
AudioOutputI2S i2s1;
AudioControlSGTL5000 audioShield;

// Routing: playMem → filter → mixer → I2S out
AudioConnection patchCord1(playMem, 0, filter, 0);
AudioConnection patchCord2(filter, 0, mixer, 0);
AudioConnection patchCord3(mixer, 0, i2s1, 0);
AudioConnection patchCord4(mixer, 0, i2s1, 1);

// =============== GLOBAL STATE ===============
volatile uint16_t piezoCenterPeak = 0;
volatile uint32_t lastHitMs = 0;
volatile float smoothedFlex = 0.0f;
volatile bool inPeakState = false;
volatile uint16_t currentFSR = 0;
// =============== ISR: PIEZO EDGE DETECTION & PLAYBACK TRIGGER ===============
void piezoISR() {
  uint32_t nowMs = millis();
  uint16_t piezoRaw = analogReadFast(PIEZO_CENTER_PIN);
  uint16_t fsrRaw = analogReadFast(FSR_PIN);
  float flexRaw = analogReadFast(FLEX_PIN);
  
  // Update global state
  piezoCenterPeak = piezoRaw;
  currentFSR = fsrRaw;
  
  // Smooth Flex (exponential moving average)
  smoothedFlex = (1.0f - FLEX_SMOOTH_ALPHA) * smoothedFlex + FLEX_SMOOTH_ALPHA * flexRaw;
  
  // Peak-tracking: only trigger on rising edge
  bool crossedUp = (piezoRaw >= PIEZO_THRESHOLD) && !inPeakState;
  
  if (crossedUp) {
    inPeakState = true;
    
    // Debounce
    if (nowMs - lastHitMs >= PIEZO_DEBOUNCE_MS) {
      lastHitMs = nowMs;
      
      // TRIGGER: Play drum sample if not already playing
      if (!playMem.isPlaying()) {
        playMem.play(drum_raw);
        Serial.println("PLAY");
      }
    }
  } else if (piezoRaw < (PIEZO_THRESHOLD / 2)) {
    // Released
    inPeakState = false;
  }
}

// =============== SETUP ===============
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Teensy 4.x PLAYBACK WITH SENSOR MODULATION ===");
  
  // Audio setup
  AudioMemory(20);
  audioShield.enable();
  audioShield.volume(0.7);
  
  // Initialize filter (low-pass for tone control)
  filter.frequency(4000);  // start neutral (bright)
  filter.resonance(0.7);
  filter.octaveControl(1.0);
  
  // Mixer setup
  mixer.gain(0, 1.0);  // playback channel
  mixer.gain(1, 0.0);  // unused
  mixer.gain(2, 0.0);  // unused
  mixer.gain(3, 0.0);  // unused
  
  // Sensor pins
  pinMode(PIEZO_CENTER_PIN, INPUT);
  pinMode(FLEX_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  
  // Init state
  piezoCenterPeak = 0;
  lastHitMs = 0;
  smoothedFlex = 2500.0f;  // neutral Flex
  currentFSR = 0;
  inPeakState = false;
  
  Serial.println("Setup complete. Ready for hits!");
}

// =============== MAIN LOOP: SENSOR POLLING & REAL-TIME MODULATION ===============
void loop() {
  // Poll sensors continuously
  piezoISR();
  
  // 1. FLEX → Filter cutoff (brightness control)
  // Map smoothed Flex [FLEX_MIN, FLEX_MAX] → cutoff [500Hz, 4000Hz]
  float flexNorm = (smoothedFlex - FLEX_MIN) / (FLEX_MAX - FLEX_MIN);
  flexNorm = constrain(flexNorm, 0.0f, 1.0f);
  
  // Expressive: low = dark (500 Hz), high = bright (4000 Hz)
  float flexCutoff = 500.0f + (3500.0f * flexNorm);
  filter.frequency(flexCutoff);
  
  // 2. FSR → Playback volume (sustain control)
  // Map FSR [0, 1023] → gain [1.0, 0.0]
  float fsrNorm = currentFSR / 1023.0f;
  float playbackGain = 1.0f - fsrNorm;  // inverted: press FSR = lower volume
  mixer.gain(0, playbackGain);
  
  // Debug output (500ms)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500) {
    lastDebugMs = millis();
    Serial.printf("Piezo: %4d | Flex: %4.0f (cut: %5.1f Hz) | FSR: %4d (gain: %.2f) | Playing: %d\n",
                  piezoCenterPeak, smoothedFlex, flexCutoff, currentFSR, playbackGain, playMem.isPlaying());
  }
  
  delay(2);
}
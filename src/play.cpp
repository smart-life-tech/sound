/*
   Teensy 4.x GAN-GAN SYNTHESIS WITH REAL-TIME SENSOR MODULATION
   - GanGan synth: FM attack + harmonic body + subharmonic jitter
   - Parameters tuned for expressive drum sound
   - Flex sensor (A1) → Base frequency (pitch control)
   - FSR sensor (A0) → Output gain (sustain/volume control)
   - Piezo sensor (A2) → Trigger noteOn
*/

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

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

// =============== GANGAN SYNTH CLASS ===============
class AudioSynthGanGan : public AudioStream {
public:
    AudioSynthGanGan() : AudioStream(0, NULL) {
        aStartFreq = 8.0f;
        aStopFreq = 3.0f;
        aStartAmp = 2.0f;
        aStopAmp = 0.2f;
        aDuration = 0.3f;
        aKnee = 1;

        bFreq = 200.0f;
        bSubFreq = 30.0f;
        bAmp = 1.0f;
        bSubStd = 0.3f;

        envDurBase = 0.32f;
        envDur = envDurBase;
        envAttack = 0.001f;
        outputGain = 0.4f;

        phaseA = phaseB = phaseSub = 0.0f;
        sweepT = 1.0f;
        envT = 1.0f;
        active = false;

        sampleRate = AUDIO_SAMPLE_RATE_EXACT;
        twoPI = 2.0f * M_PI;

        fmDepthScale = 0.65f;
        sweepSpreadScale = 0.60f;
    }

    virtual void update(void) override {
        audio_block_t *block = allocate();
        if (!block) return;
        float invSR = 1.0f / sampleRate;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            float out = 0.0f;
            if (active) {
                sweepT += 1.0f / (aDuration * sampleRate);
                if (sweepT > 1.0f) sweepT = 1.0f;
                float k = sweepT;
                if (aKnee == 1) k = sweepT * sweepT;
                else if (aKnee == 2) k = sqrtf(sweepT);
                float fA = aStartFreq + (aStopFreq - aStartFreq) * (k * sweepSpreadScale);
                float aA = aStartAmp + (aStopAmp - aStartAmp) * k;
                
                phaseA += fA * invSR;
                if (phaseA >= 1.0f) phaseA -= floorf(phaseA);
                float modA = sinf(twoPI * phaseA) * (aA * fmDepthScale);
                
                phaseB += bFreq * invSR;
                if (phaseB >= 1.0f) phaseB -= floorf(phaseB);
                phaseSub += bSubFreq * invSR;
                if (phaseSub >= 1.0f) phaseSub -= floorf(phaseSub);
                float jitter = sinf(twoPI * phaseSub) * bSubStd;
                float toneB = sinf(twoPI * phaseB) * (bAmp + jitter);
                
                envT += invSR;
                float envNorm = envT / envDur;
                if (envNorm > 1.0f) envNorm = 1.0f;
                float env;
                float atkNorm = envAttack / envDur;
                if (envNorm < atkNorm) env = envNorm / atkNorm;
                else {
                    float t = (envNorm - atkNorm) / (1.0f - atkNorm);
                    env = expf(-5.0f * t);
                }
                out = (1.0f + modA) * toneB * env * outputGain;
                if (sweepT >= 1.0f && envNorm >= 1.0f) active = false;
            }
            if (out > 1.0f) out = 1.0f;
            if (out < -1.0f) out = -1.0f;
            block->data[i] = (int16_t)(out * 32767.0f);
        }
        transmit(block);
        release(block);
    }

    void setBFreq(float v) { bFreq = v; }
    void setEnvDur(float v) { envDur = max(0.1f, v); }
    void setOutputGain(float v) { outputGain = constrain(v, 0.0f, 1.0f); }
    void noteOn() {
        sweepT = 0.0f;
        envT = 0.0f;
        phaseB += random(0, 100) * 1e-5f;
        active = true;
    }
    bool isActive() const { return active; }

private:
    float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
    uint8_t aKnee;
    float bFreq, bSubFreq, bAmp, bSubStd;
    float envDur, envAttack, outputGain;
    float phaseA, phaseB, phaseSub;
    float sweepT, envT;
    bool active;
    float sampleRate, twoPI;
    float fmDepthScale, sweepSpreadScale;
    
public:
    float envDurBase;
};


// =============== AUDIO OBJECTS ===============
AudioSynthGanGan ganGan;
AudioOutputI2S i2s1;
AudioControlSGTL5000 audioShield;

// Routing: ganGan → I2S out
AudioConnection patchCord1(ganGan, 0, i2s1, 0);
AudioConnection patchCord2(ganGan, 0, i2s1, 1);

// =============== GLOBAL STATE ===============
volatile uint16_t piezoCenterPeak = 0;
volatile uint32_t lastHitMs = 0;
volatile float smoothedFlex = 0.0f;
volatile bool inPeakState = false;
volatile uint16_t currentFSR = 0;
// =============== ISR: PIEZO EDGE DETECTION & GANGAN TRIGGER ===============
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
      
      // TRIGGER: noteOn if not already active
      if (!ganGan.isActive()) {
        ganGan.noteOn();
        Serial.println(">>> GAN-GAN NOTEON <<<");
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
  Serial.println("\n=== Teensy 4.x GAN-GAN SYNTH WITH SENSOR MODULATION ===");
  
  // Audio setup
  AudioMemory(12);
  audioShield.enable();
  audioShield.volume(0.7);
  
  // GanGan initialization with provided parameters
  ganGan.setBFreq(46.0f);        // base frequency (body)
  ganGan.setEnvDur(0.62f);       // envelope duration (s)
  ganGan.setOutputGain(0.4f);    // output gain
  
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
  
  Serial.println("GAN-GAN Setup complete. Ready for hits!");
  Serial.println("Flex → Base Freq | FSR → Output Gain");
}

// =============== MAIN LOOP: SENSOR POLLING & REAL-TIME MODULATION ===============
void loop() {
  // Poll sensors continuously
  piezoISR();
  
  // 1. FLEX → GanGan Base Frequency (bFreq)
  // Map smoothed Flex [FLEX_MIN, FLEX_MAX] → frequency [40Hz, 800Hz]
  float flexNorm = (smoothedFlex - FLEX_MIN) / (FLEX_MAX - FLEX_MIN);
  flexNorm = constrain(flexNorm, 0.0f, 1.0f);
  
  // Expressive: low = deep (40 Hz), high = soprano (800 Hz)
  // Using exponential curve for musical pitch spacing
  float flexFreq = 40.0f * powf(20.0f, flexNorm);  // 40 Hz @ flexNorm=0 to 800 Hz @ flexNorm=1
  ganGan.setBFreq(flexFreq);
  
  // 2. FSR → GanGan Output Gain (volume control)
  // Map FSR [0, 1023] → gain [0.4, 1.0]
  float fsrNorm = currentFSR / 1023.0f;
  float outputGain = 0.4f + (0.6f * fsrNorm);  // FSR pressed = louder (sustain)
  ganGan.setOutputGain(outputGain);
  
  // Debug output (500ms)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500) {
    lastDebugMs = millis();
    Serial.printf("Piezo: %4d | Flex: %4.0f (freq: %6.1f Hz) | FSR: %4d (gain: %.2f) | Active: %d\n",
                  piezoCenterPeak, smoothedFlex, flexFreq, currentFSR, outputGain, ganGan.isActive());
  }
  
  delay(2);
}
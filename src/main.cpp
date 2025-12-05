// GanGanTeensyPCM5102.ino
// Teensy 4.1 + Audio Library + PCM5102 (I2S) output
// Single-voice GAN-GAN FM drum synth as an AudioStream node
// Author: (adapted for you)

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// Audio system objects (create globally)
AudioSynthGanGan    ganGan;          // custom synth (class defined below)
AudioOutputI2S      i2sOut;          // I2S output (connect to PCM5102)
AudioConnection     patch1(ganGan, 0, i2sOut, 0);
AudioConnection     patch2(ganGan, 0, i2sOut, 1);

// ------------------------------
// AudioSynthGanGan class
// ------------------------------
class AudioSynthGanGan : public AudioStream {
public:
    AudioSynthGanGan() : AudioStream(0, NULL) {
        // Default parameters (matches your SC defaults)
        aStartFreq = 2.0f;
        aStopFreq  = 40.0f;
        aStartAmp  = 0.1f;
        aStopAmp   = 1.0f;
        aDuration  = 0.20f;
        aKnee      = 0; // 0=linear,1=expIn,2=expOut

        bFreq      = 220.0f;
        bSubFreq   = 36.0f;
        bAmp       = 1.0f;
        bSubStd    = 0.05f;

        envDur     = 0.35f;
        envAttack  = 0.001f; // seconds (tiny)
        outputGain = 0.4f;

        // internal state
        phaseA = phaseB = phaseSub = 0.0f;
        sweepT = 1.0f; // idle (1=done)
        envT = 1.0f;
        active = false;

        // sample rate constants
        sampleRate = (float)AUDIO_SAMPLE_RATE_EXACT;
        twoPI = 2.0f * M_PI;
        resetIncrements();
    }

    // Required by AudioStream
    virtual void update(void) override {
        audio_block_t *block = allocate();
        if (!block) return;

        // Fill block
        const float invSR = 1.0f / sampleRate;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            float out = 0.0f;

            if (active) {
                // --- sweep normalized time 0..1 (exciter sweep) ---
                sweepT += (1.0f / (aDuration * sampleRate));
                if (sweepT > 1.0f) sweepT = 1.0f;

                // knee curves
                float k;
                if (aKnee == 1) k = sweepT * sweepT;        // expIn
                else if (aKnee == 2) k = sqrtf(sweepT);     // expOut
                else k = sweepT;                            // linear

                // modulator freq & amp
                float fA = aStartFreq + (aStopFreq - aStartFreq) * k;
                float aA = aStartAmp  + (aStopAmp  - aStartAmp ) * k;

                // advance modulator phase (phase in [0,1))
                phaseA += fA * invSR;
                if (phaseA >= 1.0f) phaseA -= floorf(phaseA);

                float modA = sinf(twoPI * phaseA) * aA;

                // carrier + jitter
                phaseB += bFreq * invSR;
                if (phaseB >= 1.0f) phaseB -= floorf(phaseB);

                phaseSub += bSubFreq * invSR;
                if (phaseSub >= 1.0f) phaseSub -= floorf(phaseSub);

                float jitter = sinf(twoPI * phaseSub) * bSubStd;
                float toneB  = sinf(twoPI * phaseB) * (bAmp + jitter);

                // envelope: normalized time envT, linear increment to envDur
                envT += invSR;
                float envNorm = envT / envDur;
                if (envNorm >= 1.0f) envNorm = 1.0f;

                // build percussive envelope: quick attack then exponential-ish decay
                float env;
                if (envNorm < envAttack / envDur) {
                    env = envNorm / (envAttack / envDur);
                } else {
                    // scale remaining portion to 0..1 then exponential fall
                    float t = (envNorm - (envAttack / envDur)) / (1.0f - (envAttack / envDur));
                    env = expf(-5.0f * t); // -5 shapes decay; tweak if needed
                }

                // final sample
                out = (1.0f + modA) * toneB * env * outputGain;

                // deactivate when both sweep and envelope finished (sweepT==1 and envNorm==1)
                if (sweepT >= 1.0f && envNorm >= 1.0f) {
                    active = false; // note complete
                }
            } // active

            // saturate to [-1,1]
            if (out > 1.0f) out = 1.0f;
            if (out < -1.0f) out = -1.0f;

            // convert to 16-bit signed PCM (Audio library expects signed 16-bit in blocks)
            block->data[i] = (int16_t)(out * 32767.0f);
        }

        transmit(block);
        release(block);
    }

    // -------- control methods (exposed parameters) --------
    // Exciter
    void setAStartFreq(float v) { aStartFreq = v; resetIncrements(); }
    void setAStopFreq(float v)  { aStopFreq  = v; resetIncrements(); }
    void setAStartAmp(float v)  { aStartAmp  = v; }
    void setAStopAmp(float v)   { aStopAmp   = v; }
    void setADuration(float v)  { aDuration  = max(0.001f, v); resetIncrements(); }
    void setAKnee(uint8_t k)    { aKnee = k; }

    // Resonator
    void setBFreq(float v)     { bFreq = v; }
    void setBSubFreq(float v)  { bSubFreq = v; }
    void setBAmp(float v)      { bAmp = v; }
    void setBSubStd(float v)   { bSubStd = v; }

    // Envelope / output
    void setEnvDur(float v)    { envDur = max(0.005f, v); }
    void setEnvAttack(float v) { envAttack = max(0.0f, v); }
    void setOutputGain(float v) { outputGain = v; }

    // trigger (like hitting the drum)
    void noteOn() {
        sweepT = 0.0f;   // start sweep
        envT = 0.0f;     // start envelope time (seconds)
        active = true;

        // reset phases slightly to avoid identical repeated hits sounding identical
        // small jitter randomization could be added if desired
    }

    // helper: is the voice currently active?
    bool isActive() const { return active; }

private:
    // parameters (publicly adjustable via setters)
    float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
    uint8_t aKnee;

    float bFreq, bSubFreq, bAmp, bSubStd;

    float envDur, envAttack, outputGain;

    // internal state
    float phaseA, phaseB, phaseSub;
    float sweepT;    // normalized sweep progress 0..1
    float envT;      // envelope elapsed time (seconds)
    bool active;

    // sample-rate helpers
    float sampleRate;
    float twoPI;

    void resetIncrements() {
        // nothing heavy here for now — kept for extension (LUT, increments)
        // reserved for future precomputation (optional)
    }
};

// ------------------------------
// Example usage in setup/loop
// ------------------------------
void setup() {
    AudioMemory(12); // increase if you add more nodes

    // set example parameters (tweak)
    ganGan.setBFreq(46.0f);
    ganGan.setEnvDur(0.62f);
    ganGan.setBSubFreq(71.0f);
    ganGan.setBSubStd(0.2f);

    ganGan.setAStartFreq(600.0f);
    ganGan.setAStopFreq(6.0f);
    ganGan.setAStartAmp(1.0f);
    ganGan.setAStopAmp(0.2f);
    ganGan.setADuration(0.21f);
    ganGan.setAKnee(1); // expIn

    // stereo I2S output via i2sOut (patches already made above)
    // If you use a codec/shield, configure it here. For PCM5102 nothing else needed.
}

void loop() {
    // simple demo: hit the drum every 600 ms
    static uint32_t next = 0;
    if (millis() > next) {
        ganGan.noteOn();
        next = millis() + 600;
    }
    // put other control code here (e.g., changing parameters live)
}

// main.cpp
// Teensy 4.1 FM Physical Drum + PCM5102 I2S Output
// With FULL SERIAL DEBUG of all sensor inputs and event triggers

#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ----------------------------- Sensor Pins -----------------------------
#define PIEZO_PIN A2
#define FLEX_PIN  A1
#define FSR_PIN   A0

#define PIEZO_THRESHOLD      250
#define PIEZO_DEBOUNCE_MS     40
#define FLEX_SMOOTH_ALPHA   0.20f

// -----------------------------------------------------------------------
//  AUDIO CLASS (FM DRUM)
// -----------------------------------------------------------------------
class AudioSynthGanGan : public AudioStream {
public:
    AudioSynthGanGan() : AudioStream(0, NULL)
    {
        // Exciter defaults
        aStartFreq = 600.0f;
        aStopFreq  = 6.0f;
        aStartAmp  = 1.0f;
        aStopAmp   = 0.2f;
        aDuration  = 0.21f;
        aKnee      = 1;

        // Resonator defaults
        bFreq      = 46.0f;
        bSubFreq   = 71.0f;
        bAmp       = 1.0f;
        bSubStd    = 0.20f;

        // Envelope
        envDurBase = 0.62f;
        envDur     = envDurBase;
        envAttack  = 0.001f;
        outputGain = 0.40f;

        // Internal
        phaseA = phaseB = phaseSub = 0;
        sweepT = 1.0f;
        envT   = 1.0f;
        active = false;

        sampleRate = AUDIO_SAMPLE_RATE_EXACT;
        twoPI = 2.0f * (float)M_PI;

        // Anti-metallic scaling
        fmDepthScale      = 0.65f;
        sweepSpreadScale  = 0.60f;
    }

    // =============== PER-SAMPLE SYNTH ===============
    virtual void update(void) override
    {
        audio_block_t *block = allocate();
        if (!block) return;

        float invSR = 1.0f / sampleRate;

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float out = 0.0f;

            if (active)
            {
                // Sweep progress 0–1
                sweepT += 1.0f / (aDuration * sampleRate);
                if (sweepT > 1.0f) sweepT = 1.0f;

                // Knee curve
                float k = sweepT;
                if (aKnee == 1) k = sweepT * sweepT;
                else if (aKnee == 2) k = sqrtf(sweepT);

                // Exciter FM
                float fA = aStartFreq + (aStopFreq - aStartFreq) * (k * sweepSpreadScale);
                float aA = aStartAmp  + (aStopAmp  - aStartAmp) * k;

                // Modulator
                phaseA += fA * invSR;
                if (phaseA >= 1.0f) phaseA -= floorf(phaseA);
                float modA = sinf(twoPI * phaseA) * (aA * fmDepthScale);

                // Resonator
                phaseB += bFreq * invSR;
                if (phaseB >= 1.0f) phaseB -= floorf(phaseB);

                phaseSub += bSubFreq * invSR;
                if (phaseSub >= 1.0f) phaseSub -= floorf(phaseSub);

                float jitter = sinf(twoPI * phaseSub) * bSubStd;
                float toneB  = sinf(twoPI * phaseB) * (bAmp + jitter);

                // Envelope
                envT += invSR;
                float envNorm = envT / envDur;
                if (envNorm >= 1.0f) envNorm = 1.0f;

                float env;
                float atkNorm = envAttack / envDur;

                if (envNorm < atkNorm)
                    env = envNorm / atkNorm;
                else {
                    float t = (envNorm - atkNorm) / (1.0f - atkNorm);
                    env = expf(-5.0f * t);
                }

                out = (1.0f + modA) * toneB * env * outputGain;

                if (sweepT >= 1.0f && envNorm >= 1.0f)
                    active = false;
            }

            // Final clipping
            if (out > 1.0f) out = 1.0f;
            if (out < -1.0f) out = -1.0f;

            block->data[i] = (int16_t)(out * 32767.0f);
        }

        transmit(block);
        release(block);
    }

    // Parameter setters
    void setBFreq(float v){ bFreq = v; }
    void setEnvDur(float v){ envDur = max(0.02f, v); }
    void setOutputGain(float v){ outputGain = v; }

    void noteOn()
    {
        sweepT = 0.0f;
        envT   = 0.0f;
        phaseB += random(0,100)*1e-5f;
        active = true;

        Serial.println(">>> noteOn() fired!");
    }

    bool isActive() const { return active; }

    float envDurBase;

private:
    // Parameters
    float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
    uint8_t aKnee;

    float bFreq, bSubFreq, bAmp, bSubStd;

    float envDur, envAttack, outputGain;

    // State
    float phaseA, phaseB, phaseSub;
    float sweepT, envT;
    bool active;

    float sampleRate, twoPI;

    float fmDepthScale;
    float sweepSpreadScale;
};

// -----------------------------------------------------------------------
//  AUDIO PIPELINE
// -----------------------------------------------------------------------
AudioSynthGanGan ganGan;
AudioFilterBiquad hpDC;
AudioFilterStateVariable svf;
AudioOutputI2S i2sOut;

AudioConnection p1(ganGan, 0, hpDC, 0);
AudioConnection p2(hpDC, 0, svf, 0);
AudioConnection p3(svf, 0, i2sOut, 0);
AudioConnection p4(svf, 0, i2sOut, 1);

// -----------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------
float smoothedFlex = 0.0f;
uint32_t lastHitMs = 0;

// -----------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(800);
    Serial.println("\n=== GAN-GAN FM DRUM START ===");

    AudioMemory(14);

    hpDC.setHighpass(40.0f, 0.7f);
    svf.frequency(1200);   
    svf.resonance(0.8f);

    smoothedFlex = analogRead(FLEX_PIN);
}

// -----------------------------------------------------------------------
//  LOOP — FULL SENSOR DEBUG
// -----------------------------------------------------------------------
void loop()
{
    uint16_t piezo = analogRead(PIEZO_PIN);
    uint16_t fsr   = analogRead(FSR_PIN);
    uint16_t flex  = analogRead(FLEX_PIN);

    uint32_t now = millis();

    // ---------------- Piezo hit debug ----------------
    Serial.print("Piezo=");
    Serial.print(piezo);
    Serial.print("  Hit? ");

    if (piezo > PIEZO_THRESHOLD && (now - lastHitMs) > PIEZO_DEBOUNCE_MS)
    {
        Serial.print("YES  ");
        ganGan.noteOn();
        lastHitMs = now;
    }
    else
        Serial.print("no   ");


    // ---------------- Flex debug ----------------
    smoothedFlex = smoothedFlex*(1.0f-FLEX_SMOOTH_ALPHA) + FLEX_SMOOTH_ALPHA*flex;

    float cutoff = map(smoothedFlex, 200, 3800, 300, 12000);
    cutoff = constrain(cutoff, 300, 12000);
    svf.frequency(cutoff);

    Serial.print("  Flex(raw)=");
    Serial.print(flex);
    Serial.print(" smoothed=");
    Serial.print(smoothedFlex);
    Serial.print(" cutoff=");
    Serial.print(cutoff);


    // ---------------- FSR debug ----------------
    float fsrNorm = (float)constrain(fsr, 0, 4095) / 4095.0f;

    float gain = 0.25f + fsrNorm * 0.75f;
    ganGan.setOutputGain(gain * 0.4f);

    Serial.print("  FSR=");
    Serial.print(fsr);
    Serial.print(" gain=");
    Serial.print(gain);


    // ---------------- Env duration debug ----------------
    float newDur = ganGan.envDurBase * (1.0f - fsrNorm*0.4f);
    newDur = constrain(newDur, 0.05f, 1.5f);
    ganGan.setEnvDur(newDur);

    Serial.print("  envDur=");
    Serial.print(newDur);

    Serial.println();

    delay(20);
}

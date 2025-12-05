#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ---------------------------------------------------------------------------
// 1) CLASS DEFINITION MUST COME FIRST
// ---------------------------------------------------------------------------

class AudioSynthGanGan : public AudioStream {
public:
    AudioSynthGanGan() : AudioStream(0, NULL)
    {
        aStartFreq = 2.0f;
        aStopFreq  = 40.0f;
        aStartAmp  = 0.1f;
        aStopAmp   = 1.0f;
        aDuration  = 0.20f;
        aKnee      = 0;

        bFreq      = 220.0f;
        bSubFreq   = 36.0f;
        bAmp       = 1.0f;
        bSubStd    = 0.05f;

        envDur     = 0.35f;
        envAttack  = 0.001f;
        outputGain = 0.4f;

        phaseA = phaseB = phaseSub = 0.0f;
        sweepT = 1.0f;
        envT   = 1.0f;

        active = false;

        sampleRate = AUDIO_SAMPLE_RATE_EXACT;
        twoPI = 2.0f * (float)M_PI;
    }

    // The per-sample generator
    virtual void update(void)
    {
        audio_block_t *block = allocate();
        if (!block) return;

        const float invSR = 1.0f / sampleRate;

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float out = 0.0f;

            if (active)
            {
                sweepT += 1.0f / (aDuration * sampleRate);
                if (sweepT > 1.0f) sweepT = 1.0f;

                float k = sweepT;
                if (aKnee == 1) k = sweepT * sweepT;
                else if (aKnee == 2) k = sqrtf(sweepT);

                float fA = aStartFreq + (aStopFreq - aStartFreq) * k;
                float aA = aStartAmp  + (aStopAmp  - aStartAmp ) * k;

                phaseA += fA * invSR;
                if (phaseA >= 1.0f) phaseA -= floorf(phaseA);
                float modA = sinf(twoPI * phaseA) * aA;

                phaseB += bFreq * invSR;
                if (phaseB >= 1.0f) phaseB -= floorf(phaseB);

                phaseSub += bSubFreq * invSR;
                if (phaseSub >= 1.0f) phaseSub -= floorf(phaseSub);

                float jitter = sinf(twoPI * phaseSub) * bSubStd;
                float toneB = sinf(twoPI * phaseB) * (bAmp + jitter);

                envT += invSR;
                float envNorm = envT / envDur;
                if (envNorm >= 1.0f) envNorm = 1.0f;

                float env;
                float atkNorm = envAttack / envDur;

                if (envNorm < atkNorm) {
                    env = envNorm / atkNorm;
                } else {
                    float t = (envNorm - atkNorm) / (1.0f - atkNorm);
                    env = expf(-5.0f * t);
                }

                out = (1.0f + modA) * toneB * env * outputGain;

                if (sweepT >= 1.0f && envNorm >= 1.0f)
                    active = false;
            }

            if (out > 1.0f) out = 1.0f;
            if (out < -1.0f) out = -1.0f;

            block->data[i] = (int16_t)(out * 32767.0f);
        }

        transmit(block);
        release(block);
    }

    // Parameter setters
    void setAStartFreq(float v) { aStartFreq = v; }
    void setAStopFreq(float v)  { aStopFreq = v; }
    void setAStartAmp(float v)  { aStartAmp = v; }
    void setAStopAmp(float v)   { aStopAmp = v; }
    void setADuration(float v)  { aDuration = v; }
    void setAKnee(uint8_t v)    { aKnee = v; }

    void setBFreq(float v)     { bFreq = v; }
    void setBSubFreq(float v)  { bSubFreq = v; }
    void setBAmp(float v)      { bAmp = v; }
    void setBSubStd(float v)   { bSubStd = v; }

    void setEnvDur(float v)     { envDur = v; }
    void setOutputGain(float v) { outputGain = v; }

    void noteOn()
    {
        sweepT = 0.0f;
        envT   = 0.0f;
        active = true;
    }

private:
    float aStartFreq, aStopFreq, aStartAmp, aStopAmp, aDuration;
    uint8_t aKnee;
    float bFreq, bSubFreq, bAmp, bSubStd;
    float envDur, envAttack, outputGain;

    float phaseA, phaseB, phaseSub;
    float sweepT;
    float envT;
    bool active;

    float sampleRate;
    float twoPI;
};

// ---------------------------------------------------------------------------
// 2) GLOBAL AUDIO OBJECTS (OK now because class exists above)
// ---------------------------------------------------------------------------

AudioSynthGanGan ganGan;
AudioOutputI2S   i2sOut;
AudioConnection  patchCord1(ganGan, 0, i2sOut, 0);
AudioConnection  patchCord2(ganGan, 0, i2sOut, 1);

// ---------------------------------------------------------------------------
// 3) SETUP
// ---------------------------------------------------------------------------

void setup()
{
    AudioMemory(12);

    ganGan.setBFreq(46.0f);
    ganGan.setEnvDur(0.62f);

    ganGan.setAStartFreq(600.0f);
    ganGan.setAStopFreq(6.0f);
    ganGan.setAStartAmp(1.0f);
    ganGan.setAStopAmp(0.2f);
    ganGan.setADuration(0.21f);
    ganGan.setAKnee(1);
}

// ---------------------------------------------------------------------------
// 4) LOOP
// ---------------------------------------------------------------------------

void loop()
{
    static uint32_t next = 0;
    if (millis() > next) {
        ganGan.noteOn();
        next = millis() + 600;
    }
}

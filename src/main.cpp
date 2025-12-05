#include <Arduino.h>
#include <Audio.h>
#include <math.h>

// ----------------------------- Sensor Pins -----------------------------
#define PIEZO_PIN A2
#define FLEX_PIN  A1
#define FSR_PIN   A0

#define PIEZO_THRESHOLD      250
#define PIEZO_DEBOUNCE_MS     40
#define FLEX_SMOOTH_ALPHA   0.2f

// ----------------------------- Pitch & Gain Constants -----------------------------
const float B_FREQ_MIN = 40.0f;     // min carrier frequency (Hz)
const float B_FREQ_MAX = 200.0f;    // max carrier frequency (Hz)
const float OUTPUT_GAIN_MIN = 0.6f; // min output gain
const float OUTPUT_GAIN_MAX = 1.0f; // max output gain

// ---------------------------------------------------------------------------
// AudioSynthGanGan class (FM drum)
// ---------------------------------------------------------------------------
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
        bSubStd    = 0.2f;

        // Envelope
        envDurBase = 0.62f;
        envDur     = envDurBase;
        envAttack  = 0.001f;
        outputGain = 0.4f;

        // Internal state
        phaseA = phaseB = phaseSub = 0.0f;
        sweepT = 1.0f;
        envT   = 1.0f;
        active = false;

        sampleRate = AUDIO_SAMPLE_RATE_EXACT;
        twoPI = 2.0f * (float)M_PI;

        // Anti-metallic scaling
        fmDepthScale      = 0.65f;
        sweepSpreadScale  = 0.60f;
    }

    virtual void update(void) override
    {
        audio_block_t *block = allocate();
        if (!block) return;

        float invSR = 1.0f / sampleRate;

        for (int i=0; i<AUDIO_BLOCK_SAMPLES; i++)
        {
            float out = 0.0f;

            if(active)
            {
                sweepT += 1.0f / (aDuration * sampleRate);
                if(sweepT > 1.0f) sweepT = 1.0f;

                float k = sweepT;
                if(aKnee==1) k = sweepT*sweepT;
                else if(aKnee==2) k = sqrtf(sweepT);

                float fA = aStartFreq + (aStopFreq - aStartFreq)*(k*sweepSpreadScale);
                float aA = aStartAmp  + (aStopAmp - aStartAmp)*k;

                phaseA += fA*invSR;
                if(phaseA>=1.0f) phaseA -= floorf(phaseA);
                float modA = sinf(twoPI*phaseA)*(aA*fmDepthScale);

                phaseB += bFreq*invSR;
                if(phaseB>=1.0f) phaseB -= floorf(phaseB);

                phaseSub += bSubFreq*invSR;
                if(phaseSub>=1.0f) phaseSub -= floorf(phaseSub);

                float jitter = sinf(twoPI*phaseSub)*bSubStd;
                float toneB = sinf(twoPI*phaseB)*(bAmp+jitter);

                envT += invSR;
                float envNorm = envT/envDur;
                if(envNorm>=1.0f) envNorm=1.0f;

                float env;
                float atkNorm = envAttack/envDur;
                if(envNorm<atkNorm) env = envNorm/atkNorm;
                else {
                    float t = (envNorm - atkNorm)/(1.0f - atkNorm);
                    env = expf(-5.0f*t);
                }

                out = (1.0f + modA)*toneB*env*outputGain;

                if(sweepT>=1.0f && envNorm>=1.0f) active=false;
            }

            if(out>1.0f) out=1.0f;
            if(out<-1.0f) out=-1.0f;

            block->data[i] = (int16_t)(out*32767.0f);
        }

        transmit(block);
        release(block);
    }

    // Setters
    void setBFreq(float v){ bFreq = v; }
    void setEnvDur(float v){ envDur = max(0.1f, v); }
    void setOutputGain(float v){ outputGain = v; }

    void noteOn() {
        sweepT = 0.0f;
        envT   = 0.0f;
        phaseB += random(0,100)*1e-5f;
        active = true;
        Serial.println(">>> noteOn fired");
    }

    bool isActive() const { return active; }
    float envDurBase;

private:
    float aStartFreq,aStopFreq,aStartAmp,aStopAmp,aDuration;
    uint8_t aKnee;
    float bFreq,bSubFreq,bAmp,bSubStd;
    float envDur,envAttack,outputGain;
    float phaseA,phaseB,phaseSub;
    float sweepT,envT;
    bool active;
    float sampleRate,twoPI;
    float fmDepthScale,sweepSpreadScale;
};

// ---------------------------------------------------------------------------
// Audio objects
// ---------------------------------------------------------------------------
AudioSynthGanGan ganGan;
AudioOutputI2S   i2sOut;
AudioConnection  patch1(ganGan,0,i2sOut,0);
AudioConnection  patch2(ganGan,0,i2sOut,1);

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
float smoothedFlex = 0.0f;
uint32_t lastHitMs = 0;

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(800);
    Serial.println("\n=== GAN-GAN FM DRUM START ===");

    AudioMemory(12);

    ganGan.setBFreq(B_FREQ_MIN);
    ganGan.setEnvDur(0.62f);
}

// ---------------------------------------------------------------------------
// Loop with sensors, full serial debug, flex controls pitch
// ---------------------------------------------------------------------------
void loop()
{
    uint16_t piezo = analogRead(PIEZO_PIN);
    uint16_t flex  = analogRead(FLEX_PIN);
    uint16_t fsr   = analogRead(FSR_PIN);

    uint32_t now = millis();

    // --- Piezo trigger ---
    bool hit = (piezo > PIEZO_THRESHOLD) && (now - lastHitMs > PIEZO_DEBOUNCE_MS);
    if(hit) {
        ganGan.noteOn();
        lastHitMs = now;
    }

    // --- Flex sensor smoothing ---
    smoothedFlex = smoothedFlex*(1.0f - FLEX_SMOOTH_ALPHA) + flex*FLEX_SMOOTH_ALPHA;

    // --- Map smoothed flex to carrier frequency ---
    float normFlex = constrain(smoothedFlex, 250.0f, 3800.0f);
    normFlex = (normFlex - 250.0f) / (3800.0f - 250.0f);  // 0..1
    float carrierFreq = B_FREQ_MIN + normFlex*(B_FREQ_MAX - B_FREQ_MIN);
    ganGan.setBFreq(carrierFreq);

    // --- FSR controls output gain and envDur ---
    float fsrNorm = constrain((float)fsr / 4095.0f, 0.0f, 1.0f);
    float gain = OUTPUT_GAIN_MIN + fsrNorm*(OUTPUT_GAIN_MAX - OUTPUT_GAIN_MIN);
    ganGan.setOutputGain(gain);

    float envDur = ganGan.envDurBase*(0.6f + 0.4f*(1.0f - fsrNorm));
    envDur = constrain(envDur,0.1f,1.0f);
    ganGan.setEnvDur(envDur);

    // --- Serial debug ---
    Serial.print("Piezo="); Serial.print(piezo);
    Serial.print(hit ? " HIT! " : " no ");
    Serial.print(" Flex(raw)="); Serial.print(flex);
    Serial.print(" smoothed="); Serial.print(smoothedFlex);
    Serial.print(" CarrierFreq="); Serial.print(carrierFreq);
    Serial.print(" FSR="); Serial.print(fsr);
    Serial.print(" Gain="); Serial.print(gain);
    Serial.print(" EnvDur="); Serial.println(envDur);

    delay(20);
}

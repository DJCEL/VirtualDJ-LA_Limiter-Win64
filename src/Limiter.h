#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

class Limiter
{
public:
    Limiter();
    ~Limiter();
    void prepare(int vdjsampleRate);
    void reset();
    void setThreshold(float db);
    void setOutput(float db);
    void setTruePeak(bool enabled);
    void process(float* buffer, int frames);
    float getGainReductionDb();
    int getLatencySamples();
    int isActive();

private:
    void resizeBuffers();
    float getPeak(float left, float right);
    float detectTruePeak(float left, float right);
    float detectTruePeak_v2(float left, float right);
    float calculateTargetGain(float peak);
    float smoothGain(float target, float envelope);
    float updateEnvelope(float peak);
    float releaseCoefficient(float levelDb);
    float dbToLinear(float db);
    float linearToDb(float value);
    float clamp(float value, float min, float max);

    const float MIN_GAIN = 1.0e-8f;
	const float MIN_GAIN_DB = -120.0f; // Minimum gain in dB.
    const float LOOK_AHEAD_TIME = 0.00125f; // 1.25 ms look - ahead
	const int TRUE_PEAK_SAMPLES = 6; // 6 samples for True Peak detector.
    const double attackEnvelope = 0.00005f; // 0.05 ms
    const double attackTime = 0.00125f; //  1.25 ms corresponds to the look-ahead architecture.

    std::vector<float> delayL;
    std::vector<float> delayR;
    int sampleRate;
    float thresholdDb;
    float outputDb;
    bool is_TruePeak;
    float thresholdLinear;
    float ceilingLinear;
    int lookAheadSamples;
    int delayWrite;
    float gain; // Gain envelope.
    float envelope; // Detector envelope.
    float previousL; // Previous input samples (Left) for True Peak detector.
	float previousR; // Previous input samples (Right) for True Peak detector.
    int isOn;
};
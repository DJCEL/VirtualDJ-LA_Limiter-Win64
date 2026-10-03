#include "Limiter.h"


//----------------------------------------------------------------------------
Limiter::Limiter()
{
    sampleRate = 0;
    thresholdDb = 0.0f;
    outputDb = 0.0f;
    is_TruePeak = false;
    thresholdLinear = 0.0;
    ceilingLinear = 0.0f;
    lookAheadSamples = 0;
    delayWrite = 0;
    gain = 1.0f;
    envelope = 0.0f;
    previousL = 0.0f;
    previousR = 0.0f;
    isOn = 0;
}
//----------------------------------------------------------------------------
Limiter::~Limiter()
{

}
//----------------------------------------------------------------------------
void Limiter::prepare(int vdjsampleRate)
{
    // look-ahead. Examples: 44100 Hz = 55.125 -> 56 / 48000 Hz = 60 / 96000 Hz = 120
    sampleRate = vdjsampleRate;
    int lookAheadSamples_tmp = static_cast<int>(std::ceil(static_cast<double>(sampleRate) * LOOK_AHEAD_TIME));
    lookAheadSamples = std::max(1, lookAheadSamples_tmp);
    resizeBuffers();
    reset();


}
//----------------------------------------------------------------------------
void Limiter::resizeBuffers()
{
    // A few extra samples are useful for ring-buffer indexing and future TruePeak implementation.
    const int size = lookAheadSamples + 16;
    delayL.assign(size, 0.0f);
    delayR.assign(size, 0.0f);
    delayWrite = 0;
}
//----------------------------------------------------------------------------
void Limiter::reset()
{
    std::fill(delayL.begin(),delayL.end(),0.0f);
    std::fill(delayR.begin(),delayR.end(),0.0f);
    delayWrite = 0;
    gain = 1.0f;
    envelope = 0.0f;
    previousL = 0.0f;
    previousR = 0.0f;
}
//----------------------------------------------------------------------------
void Limiter::setThreshold(float db)
{
    thresholdDb = clamp(db, -30.0f, 0.0f);
    thresholdLinear = dbToLinear(thresholdDb);
}
//----------------------------------------------------------------------------
void Limiter::setOutput(float db)
{
    outputDb = clamp(db, -30.0f, 0.0f);
    ceilingLinear = dbToLinear(outputDb);
}
//----------------------------------------------------------------------------
void Limiter::setTruePeak(bool enabled)
{
    if (is_TruePeak == enabled)
        return;

    is_TruePeak = enabled;

    // Reset state when changing detector topology.
    reset();
}
//----------------------------------------------------------------------------
float Limiter::getPeak(float left, float right)
{
	float peak = std::max(std::fabs(left), std::fabs(right));
    return peak;
}
//----------------------------------------------------------------------------
float Limiter::detectTruePeak(float left, float right)
{
    // Simple 4x inter-sample detector. We inspect four points between the previous and current samples.
    float peak = getPeak(left, right);
	float peak_new = 0.0f;
	float t = 0.0f;
	float l = 0.0f;
	float r = 0.0f;
	const int NB_SAMPLES = 4;

    for (int k = 1; k <= NB_SAMPLES; ++k)
    {
        t = static_cast<float>(k) / float(NB_SAMPLES+1);
        l = previousL + (left  - previousL) * t;
        r = previousR + (right - previousR) * t;
		peak_new = getPeak(l,r);
        peak = std::max(peak,peak_new);
    }

    return peak;
}
//----------------------------------------------------------------------------
float Limiter::detectTruePeak_v2(float left, float right)
{
    // TODO: Il faut un suéchantillonage 4x par filtre FIR polyphasé (type ITU-R BS.1770)
}
//----------------------------------------------------------------------------
float Limiter::updateEnvelope(float peak)
{
	float coefficient = 0.0f;

    // Very fast detector attack.
    // The actual look-ahead allows us to know about the peak before it reaches the output.
    if (peak >= envelope)
    {
        coefficient = static_cast<float>(std::exp(-1.0 / (sampleRate * attackEnvelope)));
    }
    else
    {
        // Program-dependent release.
        // Strong/sustained material gets a longer release. Smaller peaks recover faster.
        const float levelDb = linearToDb(std::max(envelope, MIN_GAIN));
        coefficient = releaseCoefficient(levelDb);
    }

    envelope = coefficient * envelope + (1.0f - coefficient) * peak;
    return envelope;
}
//----------------------------------------------------------------------------
float Limiter::releaseCoefficient(float levelDb)
{
    // Approximate program-dependent release. -60 dB -> ~20 ms / 0 dB -> ~250 ms
    // This is intentionally smooth rather than using discrete release times.
	float value = (levelDb + 60.0f) / 60.0f;
    float normalized = clamp(value, 0.0f, 1.0f);
    double releaseTime = 0.020 + 0.230 * normalized;
    float releaseCoeff = static_cast<float>(std::exp(-1.0 / (sampleRate * releaseTime)));
    return releaseCoeff;
}
//----------------------------------------------------------------------------
float Limiter::calculateTargetGain(float peak)
{
    if (peak <= MIN_GAIN)
        return 1.0f;

    // The maximizer concept: threshold controls how hard we drive the limiter.
    // At threshold <= peak, gain reduction is required.
    // At peak below threshold, we can apply makeup gain, but NEVER above the output ceiling.
    const float thresholdGain = thresholdLinear / peak;
    const float ceilingGain = ceilingLinear / peak;

    // Drive the signal toward threshold.
    float target = thresholdGain;

    // But never allow the output to exceed ceiling.
    target = std::min(target, ceilingGain);

    // Never amplify above unity here.
    // The threshold itself is represented by the amount of limiting rather than by arbitrary amplification of silence/very low-level material.
    target = std::min(target,1.0f);
    float TargetGain = clamp(target, MIN_GAIN, 1.0f);

    return TargetGain;
}
//----------------------------------------------------------------------------
float Limiter::smoothGain(float target, float envelope)
{
    // Gain reduction = fast.
    // Gain recovery = program-dependent.
    if (target < gain)
    {
        // Attack.
        isOn = isOn + 1;
        float coefficient = static_cast<float>(std::exp(-1.0 / (sampleRate * attackTime)));
        gain = coefficient * gain + (1.0f - coefficient) * target;
        // Never miss the required gain.
        if (gain < target)
            gain = target;
    }
    else
    {
        // Release. 
        // Use the current envelope to determine how long recovery should take.
		float linear = std::max(envelope, MIN_GAIN);
        float levelDb = linearToDb(linear);
        float coefficient = releaseCoefficient(levelDb);
        gain = coefficient * gain + (1.0f - coefficient) * target;
        if (gain > target)
            gain = target;
    }

    gain = clamp(gain, MIN_GAIN, 1.0f);

    return gain;
}
//----------------------------------------------------------------------------
void Limiter::process(float* buffer, int frames)
{
    if (!buffer || frames <= 0)
        return;

    int bufferSize = static_cast<int>(delayL.size());
	float inputL = 0.0f;
	float inputR = 0.0f;
    float peak = 0.0f;
	int readIndex = 0;
	float delayedL = 0.0f;
	float delayedR = 0.0f;
    float envelope = 0.0f;
    float target = 0.0f;
    float gain = 0.0f;
	float outputL = 0.0f;
	float outputR = 0.0f;
    float outputPeak = 0.0f;

    isOn = 0;

    for (int i = 0; i < frames; ++i)
    {
        inputL = buffer[2 * i];
        inputR = buffer[2 * i + 1];

        // Peak detector
        peak = 0.0f;
        if (is_TruePeak)
        {
            peak = detectTruePeak(inputL,inputR);
        }
        else
        {
            peak = getPeak(inputL,inputR);
        }

        // Look-ahead delay
        delayL[delayWrite] = inputL;
        delayR[delayWrite] = inputR;

        readIndex = delayWrite - lookAheadSamples;
        if (readIndex < 0)
            readIndex += bufferSize;

        delayedL = delayL[readIndex];
        delayedR = delayR[readIndex];

        // Envelope
        envelope = updateEnvelope(peak);

        // Target gain
        target = calculateTargetGain(envelope);

        // Gain smoothing
        gain = smoothGain(target, envelope);

        // Apply
        outputL = delayedL * gain;
        outputR = delayedR * gain;

        buffer[2 * i] = outputL;
        buffer[2 * i + 1] = outputR;

        // Advance
        ++delayWrite;

        if (delayWrite >= bufferSize)
            delayWrite = 0;

        previousL = inputL;
        previousR = inputR;
    }
}
//----------------------------------------------------------------------------
float Limiter::getGainReductionDb()
{
    if (gain <= MIN_GAIN)
        return MIN_GAIN_DB;

    float gainReductionDb = linearToDb(gain);
    return gainReductionDb;
}
//----------------------------------------------------------------------------
int Limiter::getLatencySamples()
{
    int latencySamples = 0;
    if (is_TruePeak)
    {
        latencySamples = lookAheadSamples + TRUE_PEAK_SAMPLES;
    }
    else
    {
        latencySamples = lookAheadSamples;
    }
    return latencySamples;
}
//----------------------------------------------------------------------------
float Limiter::dbToLinear(float db)
{
    float linear = std::pow(10.0f, db / 20.0f);
    return linear;
}
//----------------------------------------------------------------------------
float Limiter::linearToDb(float linear)
{
    float db = 20.0f * std::log10(std::max(linear, MIN_GAIN));
    return db;
}
//----------------------------------------------------------------------------
float Limiter::clamp(float value, float min, float max)
{
    float clampValue = std::max(min, std::min(value, max));
    return clampValue;
}
//----------------------------------------------------------------------------
int Limiter::isActive()
{
    return isOn;
}
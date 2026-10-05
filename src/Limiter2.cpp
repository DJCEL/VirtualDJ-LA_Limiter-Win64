#include "Limiter2.h"

namespace
{
    // Modified Bessel function of the first kind, order 0 (series expansion).
    double besselI0(double x)
    {
        double sum = 1.0, term = 1.0;
        const double q = 0.25 * x * x;
        for (int k = 1; k < 64; ++k)
        {
            term *= q / (static_cast<double>(k) * k);
            sum += term;
            if (term < 1.0e-14 * sum)
                break;
        }
        return sum;
    }
}

//----------------------------------------------------------------------------
// SlidingMin
//----------------------------------------------------------------------------
void CLimiter2::SlidingMin::init(int maxWin)
{
    maxWindow = (std::max)(1, maxWin);
    window = maxWindow;
    uint32_t cap = 1;
    while (cap < static_cast<uint32_t>(maxWindow) + 1)
        cap <<= 1;
    val.assign(cap, 1.0f);
    idx.assign(cap, 0);
    mask = cap - 1;
    clear();
}
//----------------------------------------------------------------------------
void CLimiter2::SlidingMin::clear()
{
    head = tail = 0;
    count = 0;
}
//----------------------------------------------------------------------------
float CLimiter2::SlidingMin::push(float v)
{
    // Drop everything larger or equal: it can never be the minimum again.
    while (tail != head && val[(tail - 1) & mask] >= v)
        --tail;
    val[tail & mask] = v;
    idx[tail & mask] = count;
    ++tail;

    // Drop the elements that left the window [count-window+1, count].
    while (idx[head & mask] + static_cast<uint64_t>(window) <= count)
        ++head;

    ++count;
    return val[head & mask];
}
//----------------------------------------------------------------------------
// MovingAverage
//----------------------------------------------------------------------------
void CLimiter2::MovingAverage::init(int len)
{
    length = (std::max)(1, len);
    invLen = 1.0 / length;
    ring.assign(length, 1.0f);
    clear();
}
//----------------------------------------------------------------------------
void CLimiter2::MovingAverage::clear()
{
    std::fill(ring.begin(), ring.end(), 1.0f);
    sum = static_cast<double>(length);
    pos = 0;
}
//----------------------------------------------------------------------------
float CLimiter2::MovingAverage::push(float v)
{
    sum += static_cast<double>(v) - static_cast<double>(ring[pos]);
    ring[pos] = v;
    if (++pos >= length)
        pos = 0;
    return static_cast<float>(sum * invLen);
}
//----------------------------------------------------------------------------
// Limiter
//----------------------------------------------------------------------------
CLimiter2::CLimiter2()
{
    buildTruePeakKernel();
}
//----------------------------------------------------------------------------
// 4x oversampling: 3 interpolation phases (1/4, 2/4, 3/4), 12 taps each,
// Kaiser-windowed sinc, normalized to unity DC gain.
// Similar to a structure as the ITU-R BS.1770 true-peak interpolator (4x, 48 taps total);
//----------------------------------------------------------------------------
void CLimiter2::buildTruePeakKernel()
{
    const double pi        = 3.14159265358979323846;
    const double beta      = 6.0;
    const double halfWidth = TP_TAPS / 2.0;
    const double i0Beta    = besselI0(beta);

    for (int p = 0; p < TP_PHASES; ++p)
    {
        const double f = (p + 1) / 4.0;
        double k[TP_TAPS];
        double sum = 0.0;

        for (int i = 0; i < TP_TAPS; ++i)
        {
            // Tap i sits at offset (i - (TP_LATENCY-1)) from the centre sample c;
            // we interpolate at c + f.
            const double t = static_cast<double>(i - (TP_LATENCY - 1)) - f;
            const double x = pi * t;
            const double s = (std::sin)(x) / x;
            const double r = t / halfWidth;
            const double w = besselI0(beta * (std::sqrt)((std::max)(0.0, 1.0 - r * r))) / i0Beta;
            k[i] = s * w;
            sum += k[i];
        }
        for (int i = 0; i < TP_TAPS; ++i)
            tpKernel[p][i] = static_cast<float>(k[i] / sum);
    }
}
//----------------------------------------------------------------------------
void CLimiter2::prepare(int vdjSampleRate)
{
    sampleRate = (std::max)(8000, vdjSampleRate);

    latency = (std::max)(MIN_LATENCY,
        static_cast<int>(std::ceil(sampleRate * static_cast<double>(LATENCY_MS) / 1000.0)));

    // The true-peak detector already consumes (TP_LATENCY - 1) samples of the latency budget.
    // Guarantee: audio delay = rampLength + TP_LATENCY - 1  (see header).
    rampLength = latency - (TP_LATENCY - 1);

    // Start-up crossfade: the delay line is empty at start, so the first `latency` output samples
    // would be silence (hard step = click). Fade from the dry input to the processed signal instead.
    // Dry only during the first `latency` samples, then a linear fade over FADE_IN_MS.
    const int fadeRamp = (std::max)(1, static_cast<int>(std::ceil(sampleRate * static_cast<double>(FADE_IN_MS) / 1000.0)));
    fadeLen = latency + fadeRamp;
    invFadeLen = 1.0f / static_cast<float>(fadeRamp);

    delayL.assign(latency, 0.0f);
    delayR.assign(latency, 0.0f);

    slidingMin.init(rampLength);
    holdMin.init(static_cast<int>(std::ceil(sampleRate * 0.1)) + 1);   // up to 100 ms of hold
    movingAvg.init(rampLength);

    driveCoef = static_cast<float>(std::exp(-1.0 / (sampleRate * 0.005)));  // 5 ms
    appliedReleaseMs = -1.0f;                                              // force recompute

    driveSmoothed = driveTarget.load(std::memory_order_relaxed);
    reset();
}
//----------------------------------------------------------------------------
void CLimiter2::reset()
{
    std::fill(delayL.begin(), delayL.end(), 0.0f);
    std::fill(delayR.begin(), delayR.end(), 0.0f);
    delayPos = 0;

    std::fill(std::begin(histL), std::end(histL), 0.0f);
    std::fill(std::begin(histR), std::end(histR), 0.0f);
    histPos = 0;

    slidingMin.clear();
    holdMin.clear();
    movingAvg.clear();

    releaseGain = 1.0f;
    fadePos = 0;
    driveSmoothed = driveTarget.load(std::memory_order_relaxed);

    lastGain.store(1.0f, std::memory_order_relaxed);
    activeFrames.store(0, std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::fadeOut()
{
  // TODO: Gradual fade-out instead of immediate reset ie gradually set gain to 0.
  reset();
}
//----------------------------------------------------------------------------
void CLimiter2::setThreshold(float db)
{
    // Maximizer: lowering the threshold drives the signal harder into the ceiling.
    const float t = std::clamp(db, -30.0f, 0.0f);
    driveTarget.store(dbToLinear(-t), std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::setOutput(float db)
{
    const float c = std::clamp(db, -30.0f, 0.0f);
    ceilingTarget.store(dbToLinear(c), std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::setTruePeak(bool enabled)
{
    // No reset needed: latency is constant and the detector stream is continuous.
    truePeak.store(enabled, std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::setReleaseMs(float ms)
{
    releaseMs.store(std::clamp(ms, 5.0f, 1000.0f), std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::setHoldMs(float ms)
{
    holdMs.store(std::clamp(ms, 0.0f, 100.0f), std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
void CLimiter2::process(float* buffer, int frames)
{
    if (!buffer || frames <= 0 || delayL.empty())
        return;

    // Block-rate parameter snapshot.
    const float ceiling = ceilingTarget.load(std::memory_order_relaxed);
    const float drive   = driveTarget.load(std::memory_order_relaxed);
    const bool  useTP   = truePeak.load(std::memory_order_relaxed);
    const float relMs   = releaseMs.load(std::memory_order_relaxed);

    if (relMs != appliedReleaseMs)
    {
        appliedReleaseMs = relMs;
        releaseCoef = static_cast<float>(std::exp(-1.0 / (0.001 * relMs * sampleRate)));
    }

    holdSamples = (std::max)(1, static_cast<int>(holdMs.load(std::memory_order_relaxed) * 0.001f * sampleRate));
    holdMin.setWindow(holdSamples);

    const float driveStep = 1.0f - driveCoef;
    int   active = 0;
    float g = 1.0f;

    for (int i = 0; i < frames; ++i)
    {
        // 1. Smoothed drive (applied before everything, so the ceiling guarantee holds).
        driveSmoothed += (drive - driveSmoothed) * driveStep;
        const float rawL = buffer[2 * i];
        const float rawR = buffer[2 * i + 1];
        const float inL = rawL * driveSmoothed;
        const float inR = rawR * driveSmoothed;

        // 2. Audio delay line (read the oldest sample, then overwrite it).
        const float dL = delayL[delayPos];
        const float dR = delayR[delayPos];
        delayL[delayPos] = inL;
        delayR[delayPos] = inR;
        if (++delayPos >= latency)
            delayPos = 0;

        // 3. History for the FIR (double-write -> contiguous, oldest..newest).
        histL[histPos] = histL[histPos + TP_TAPS] = inL;
        histR[histPos] = histR[histPos + TP_TAPS] = inR;
        if (++histPos >= TP_TAPS)
            histPos = 0;
        const float* hl = &histL[histPos];
        const float* hr = &histR[histPos];

        // 4. Peak of the centre sample c = n - (TP_LATENCY-1) and, if enabled,
        //    of the 3 interpolated points between c and c+1 (stereo linked).
        float peak = (std::max)(std::fabs(hl[TP_LATENCY - 1]), std::fabs(hr[TP_LATENCY - 1]));
        if (useTP)
        {
            for (int p = 0; p < TP_PHASES; ++p)
            {
                const float* k = tpKernel[p];
                float yl = 0.0f, yr = 0.0f;
                for (int t = 0; t < TP_TAPS; ++t)
                {
                    yl += hl[t] * k[t];
                    yr += hr[t] * k[t];
                }
                peak = (std::max)(peak, (std::max)(std::fabs(yl), std::fabs(yr)));
            }
        }

        // 5. Instantaneous required gain (no smoothing on the detector!).
        const float required = (peak > ceiling) ? (ceiling / peak) : 1.0f;

        // 6. Sliding minimum over the ramp window.
        const float m = slidingMin.push(required);

        // 7. Hold + release stage. The gain may not rise above the minimum of the last holdMs
        //    (keeps the gain constant between the crests of a bass waveform -> no ripple),
        //    then recovers with a one-pole release. Attack is instant here (the moving
        //    average below makes the ramp). Always releaseGain <= holdMin <= m.
        const float hm = holdMin.push(m);
        if (hm < releaseGain)
            releaseGain = hm;
        else
            releaseGain = releaseCoef * releaseGain + (1.0f - releaseCoef) * hm;

        // 8. Moving average = linear attack ramp finishing exactly when the peak exits the delay.
        g = movingAvg.push(releaseGain);

        // 9. Apply + last-resort safety clamp (float rounding, ceiling changes).
        float outL = dL * g;
        float outR = dR * g;

        // 9b. Start-up crossfade dry -> processed (removes the click when the plugin is activated).
        if (fadePos < fadeLen)
        {
            const int   k = fadePos - latency;   // <= 0 while the delay line is still filling
            const float a = (k > 0) ? static_cast<float>(k) * invFadeLen : 0.0f;
            outL = rawL + (outL - rawL) * a;
            outR = rawR + (outR - rawR) * a;
            ++fadePos;
        }

        outL = std::clamp(outL, -ceiling, ceiling);
        outR = std::clamp(outR, -ceiling, ceiling);
        buffer[2 * i]     = outL;
        buffer[2 * i + 1] = outR;

        if (g < 0.999f)
            ++active;
    }

    lastGain.store(g, std::memory_order_relaxed);
    activeFrames.store(active, std::memory_order_relaxed);
}
//----------------------------------------------------------------------------
float CLimiter2::getGainReductionDb() const
{
    const float gain = (std::max)(lastGain.load(std::memory_order_relaxed), MIN_LINEAR);
    return 20.0f * std::log10(gain);
}
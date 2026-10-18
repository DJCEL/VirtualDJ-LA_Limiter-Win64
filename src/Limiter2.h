#pragma once

#include <vector>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <algorithm>

// Look-ahead stereo brick-wall limiter with FIR true-peak detection.
//
// Signal flow (all per sample, interleaved stereo float):
//   in -> drive (smoothed) -> +-> audio delay (LATENCY) ---------------------------> x gain -> clamp -> out
//                              |
//                              +-> true-peak detector (4x: 12 taps/phase, or 8x: 24 taps/phase)
//                                    -> required gain  = min(1, ceiling / peak)
//                                    -> sliding minimum over `rampLength` samples
//                                    -> hold (min over holdMs) + one-pole release
//                                    -> moving average over `rampLength` samples  (= attack ramp)
//
// The sliding-min followed by a moving average of the same length guarantees that
// the gain has fully reached the required value when the peak leaves the delay line.
// Latency is constant (independent of the True Peak switch and of the 4x/8x choice): DEFAULT_LATENCY_MS.
class CLimiter2
{
public:
    CLimiter2();
    void start(int vdjSampleRate); // Allocates. Call from OnStart(), never concurrently with process().
    void stop();
    void process(float* buffer, int frames);  // Audio thread. buffer = interleaved stereo, frames = number of stereo frames.

    // Thread-safe (can be called from the UI thread while process() runs).
    void setThreshold(float db, float minDb, float maxDb);   // -30..0 dB : drive (input gain = -threshold), maximizer style
    void setOutput(float db, float minDb, float maxDb);      // -30..0 dB : output ceiling
    void setTruePeak(bool enabled);
    void setTruePeakOversampling(int factor); // 4 (default, 12 taps/phase) or 8 (24 taps/phase, more accurate near Nyquist)
    void setReleaseMs(float ms, float minMs, float maxMs);   // 5..1000 ms, default 150
    void setHoldMs(float ms, float minMs, float maxMs);      // 0..100 ms, default 30 (>= half a bass period, avoids LF distortion)
    void setFinalSecurity(bool enabled); // last security (clamp between -1.0f and 1.0f)
	void setLookaheadMs(float ms, float minMs, float maxMs); // 0.5..5.0 ms, default 1.25 (affects audio delay and attack ramp length)

    float getGainReductionDb() const;                    // last gain of the last block (<= 0 dB)
    int getLatencySamples() const;
    int isActive() const;

private:
    static constexpr int   TP_TAPS_MAX   = 24;              // history length = taps of the longest (8x) kernel
    static constexpr int   TP_PHASES_MAX = 7;               // 8x oversampling -> 7 interpolated points
    static constexpr int   TP_LATENCY    = TP_TAPS_MAX / 2; // detector delay (samples), identical in 4x and 8x so that latency never changes
    static constexpr float DEFAULT_LATENCY_MS  = 1.25f;            // total latency (audio delay)
	static constexpr float MAX_LATENCY_MS  = 5.0f;            // maximum latency (audio delay)
    static constexpr int   MIN_LATENCY = 16;               // samples
    static constexpr float FADE_IN_MS  = 5.0f;             // start-up crossfade dry -> processed
    static constexpr float FADE_OUT_MS = 5.0f;  // fade-out duration (same as fade-in for consistency)
    static constexpr float MIN_LINEAR  = 1.0e-8f;

    // O(1) amortized sliding-window minimum (monotonic deque).
    struct SlidingMin
    {
        void  init(int maxWindow);          // allocates for windows up to maxWindow
        void  setWindow(int w) { window = (std::max)(1, (std::min)(w, maxWindow)); }
        void  clear();
        float push(float v);

        std::vector<float>    val;
        std::vector<uint64_t> idx;
        uint64_t mask   = 0;
        uint64_t head   = 0;
        uint64_t tail   = 0;
        uint64_t count  = 0;
        int      window = 1;
        int      maxWindow = 1;
    };

    // Running-sum moving average (double accumulator).
    struct MovingAverage
    {
        void  init(int len);
        void  clear();
        float push(float v);

        std::vector<float> ring;
        double sum    = 0.0;
        double invLen = 1.0;
        int    pos    = 0;
        int    length = 1;
    };

    void prepare(int sampleRate);
    void reset();
    void buildTruePeakKernel();
    float dbToLinear(float db);
    double besselI0(double x);

    // Parameters (written by the UI thread, read once per block by the audio thread)
    std::atomic<float> driveTarget   { 1.0f };
    std::atomic<float> ceilingTarget { 1.0f };
    std::atomic<float> releaseMs     { 150.0f };
    std::atomic<float> holdMs        { 30.0f };
    std::atomic<bool>  truePeak      { false };
    std::atomic<int>   truePeakOs    { 4 };      // 4 or 8
    std::atomic<bool>  finalSecurity { false };
    std::atomic<float> lookaheadMs { DEFAULT_LATENCY_MS };

    // Meters (written by the audio thread)
    std::atomic<float> lastGain      { 1.0f };
    std::atomic<int>   activeFrames  { 0 };

    // Configuration (set in prepare)
    int   sampleRate = 48000;
    int   latency    = MIN_LATENCY;   // audio delay in samples
    int   rampLength = 1;             // length of sliding-min and moving-average windows
    float driveCoef  = 0.0f;          // drive smoothing (one-pole)
    float releaseCoef = 0.0f;
    float appliedReleaseMs = -1.0f;
    int   holdSamples = 1;
    int   fadePos = 0;                // start-up crossfade position (samples)
    int   fadeLen = 1;                // start-up: latency + fade ramp (samples)
    float invFadeLen = 1.0f;          // 1 /   fade ramp length
    int   fadeOutPos = 0;              // fade-out position (samples)
    int   fadeOutLen = 1;              // fade-out duration (samples)
    float invFadeOutLen = 1.0f;        // 1 / fade-out length
    bool  isFadingOut = false;         // flag to track fade-out state
	float appliedLookaheadMs = DEFAULT_LATENCY_MS;

    // DSP state
    std::vector<float> delayL, delayR;
    int   delayPos = 0;

    float histL[2 * TP_TAPS_MAX] = {};    // double-written history -> contiguous window
    float histR[2 * TP_TAPS_MAX] = {};
    int   histPos = 0;

    // Interpolation kernels. Both are centred on the same slot (TP_LATENCY-1) of the history window,
    // so switching 4x <-> 8x needs no reset and does not change the latency.
    struct TpKernel
    {
        int   phases = 0;                       // oversampling factor - 1
        int   taps   = 0;                       // taps per phase
        int   first  = 0;                       // index of the first tap inside the history window
        float k[TP_PHASES_MAX][TP_TAPS_MAX] = {};
    };
    TpKernel tpKernels[2];                      // [0] = 4x, [1] = 8x

    SlidingMin    slidingMin;
    SlidingMin    holdMin;      // gain may not rise above the minimum of the last holdMs
    MovingAverage movingAvg1;
    MovingAverage movingAvg2;

    float releaseGain   = 1.0f;
    float driveSmoothed = 1.0f;
};
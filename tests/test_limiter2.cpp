// test_limiter2.cpp - self-contained regression tests for CLimiter2 (no external dependency, C++17+).
//
// Build (MSVC) : cl /std:c++20 /O2 /DNDEBUG /EHsc /I..\src test_limiter2.cpp ..\src\Limiter2.cpp
// Build (g++)  : g++ -O2 -DNDEBUG -std=c++17 -I../src test_limiter2.cpp ../src/Limiter2.cpp -o test_limiter2
// Run          : test_limiter2            (exit code 0 = all tests passed)
//
// All limits below are expressed in dB / dBTP so they are easy to read in the report.
// The true-peak reference is an independent 16x Kaiser-sinc oversampler (128 taps/phase),
// checked against a known signal in test 0 before it is used.

#include "Limiter2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using Sig = std::vector<float>;   // interleaved stereo

static const double PI = 3.14159265358979323846;

//----------------------------------------------------------------------------
// Mini test framework
//----------------------------------------------------------------------------
static int g_checks = 0, g_failed = 0;

static void check(bool ok, const std::string& what, const std::string& detail)
{
    ++g_checks;
    if (!ok) ++g_failed;
    std::printf("  [%s] %-52s %s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.c_str());
}
static void warn(bool ok, const std::string& what, const std::string& detail)
{
    // Quality target: shown in the report, never fails the suite.
    std::printf("  [%s] %-52s %s\n", ok ? "GOAL" : "WARN", what.c_str(), detail.c_str());
}
static std::string fmt(const char* f, double a = 0, double b = 0, double c = 0)
{
    char buf[200];
    std::snprintf(buf, sizeof(buf), f, a, b, c);
    return buf;
}
static double dB(double v) { return 20.0 * std::log10((std::max)(v, 1e-12)); }
static double lin(double db) { return std::pow(10.0, db / 20.0); }

//----------------------------------------------------------------------------
// Deterministic signals
//----------------------------------------------------------------------------
struct Rng
{
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1) {}
    double uni() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s / 4294967296.0) * 2.0 - 1.0; }   // [-1,1)
    double gauss() { return (uni() + uni() + uni() + uni()) * 0.5; }                                   // ~N(0,~0.58)
};

static Sig stereo(const std::vector<double>& m)
{
    Sig s(m.size() * 2);
    for (size_t i = 0; i < m.size(); ++i) s[2 * i] = s[2 * i + 1] = static_cast<float>(m[i]);
    return s;
}

// A named stimulus: silence until `onset`, then the signal, total `seconds`.
struct Stim { std::string name; std::vector<double> x; size_t onset; };

static std::vector<Stim> makeStimuli(int sr)
{
    const size_t N = static_cast<size_t>(sr * 1.5), on = static_cast<size_t>(sr * 0.5);
    std::vector<Stim> v;
    Rng rng(12345);

    auto sine = [&](double f, double amp, double ph) {
        std::vector<double> x(N, 0.0);
        for (size_t i = on; i < N; ++i) x[i] = amp * std::sin(2 * PI * f * i / sr + ph);
        return x; };

    for (double f : { 100.0, 1000.0, 5000.0, 12000.0, 18000.0, 20000.0 })
        for (int k = 0; k < 3; ++k)
            v.push_back({ fmt("sine %.0f Hz (phase %.0f)", f, k), sine(f, 0.95, rng.uni() * PI), on });

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; ++i) x[i] = (std::max)(-1.0, (std::min)(1.0, rng.gauss() * 0.6));
      v.push_back({ "white noise burst", x, on }); }

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; ++i) {
          const double t = double(i - on) / sr, tk = std::fmod(t, 0.5);
          x[i] = 0.9 * std::exp(-tk * 18.0) * std::sin(2 * PI * (55.0 + 90.0 * std::exp(-tk * 30.0)) * tk) + 0.15 * rng.gauss(); }
      v.push_back({ "kick + noise", x, on }); }

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; ++i) x[i] = (std::fmod(200.0 * (i - on) / sr, 1.0) < 0.5) ? 0.9 : -0.9;
      v.push_back({ "square 200 Hz", x, on }); }

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; ++i) x[i] = 0.95 * (2.0 * std::fmod(300.0 * (i - on) / sr, 1.0) - 1.0);
      v.push_back({ "sawtooth 300 Hz", x, on }); }

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; ++i) x[i] = (std::max)(-0.95, (std::min)(0.95, 2.0 * std::sin(2 * PI * 1000.0 * i / sr)));
      v.push_back({ "hard-clipped sine 1 kHz", x, on }); }

    { std::vector<double> x(N, 0.0);
      for (size_t i = on; i < N; i += static_cast<size_t>(sr / 100)) x[i] = 0.95;
      v.push_back({ "impulse train 100 Hz", x, on }); }

    return v;
}

//----------------------------------------------------------------------------
// Limiter runner
//----------------------------------------------------------------------------
struct Cfg
{
    int   sr = 48000;
    float thr = -12.0f;    // drive = -thr dB
    float ceil = -1.0f;
    bool  tp = true;
    int   os = 8;
    float hold = 30.0f, rel = 150.0f, look = 1.25f;
    bool  sec = false;
    bool  autoRel = false;   // Auto release (program-dependent)
};

static void configure(CLimiter2& L, const Cfg& c)
{
    L.setThreshold(c.thr, -30.0f, 0.0f);
    L.setOutput(c.ceil, -30.0f, 0.0f);
    L.setTruePeak(c.tp);
    L.setTruePeakOversampling(c.os);
    L.setHoldMs(c.hold, 0.0f, 100.0f);
    L.setReleaseMs(c.rel, 5.0f, 1000.0f);
    L.setLookaheadMs(c.look, 0.5f, 5.0f);
    L.setFinalSecurity(c.sec);
    L.setAutoRelease(c.autoRel);
}

static Sig run(const Cfg& c, Sig in, int block = 512, int* latency = nullptr)
{
    CLimiter2 L;
    configure(L, c);
    L.start(c.sr);
    if (latency) *latency = L.getLatencySamples();
    const long frames = static_cast<long>(in.size() / 2);
    for (long i = 0; i < frames; i += block)
        L.process(&in[2 * i], static_cast<int>((std::min<long>)(block, frames - i)));
    return in;
}

static double samplePeak(const Sig& y, size_t from, size_t to)
{
    double p = 0;
    for (size_t i = from; i < to; ++i) { p = (std::max)(p, (double)std::fabs(y[2 * i])); p = (std::max)(p, (double)std::fabs(y[2 * i + 1])); }
    return p;
}

//----------------------------------------------------------------------------
// Reference true-peak meter: 16x Kaiser-sinc oversampling (independent of the limiter's detector)
//----------------------------------------------------------------------------
static double bessel0(double x)
{
    double s = 1, t = 1;
    for (int k = 1; k < 60; ++k) { t *= (x / (2.0 * k)) * (x / (2.0 * k)); s += t; if (t < 1e-14 * s) break; }
    return s;
}

static double truePeakRef(const Sig& y, size_t from, size_t to)   // left channel; [from,to) in frames
{
    static std::vector<double> K;                 // [16][128]
    const int OS = 16, HW = 64, TAPS = 2 * HW;
    const double beta = 8.0, i0 = bessel0(beta);
    if (K.empty())
    {
        K.assign(OS * TAPS, 0.0);
        for (int p = 0; p < OS; ++p)
            for (int j = 0; j < TAPS; ++j)
            {
                const double t = (j - (HW - 1)) - p / double(OS);          // tap j sits at offset j-(HW-1)
                const double s = (std::fabs(t) < 1e-12) ? 1.0 : std::sin(PI * t) / (PI * t);
                const double r = t / HW;
                const double w = (std::fabs(r) >= 1.0) ? 0.0 : bessel0(beta * std::sqrt(1.0 - r * r)) / i0;
                K[p * TAPS + j] = s * w;
            }
    }
    double peak = 0;
    const size_t lo = (std::max)(from, (size_t)HW + 1), hi = (std::min)(to, y.size() / 2 - HW - 1);
    for (size_t n = lo; n < hi; ++n)
        for (int p = 0; p < OS; ++p)
        {
            double acc = 0;
            for (int j = 0; j < TAPS; ++j) acc += y[2 * (n + j - (HW - 1))] * K[p * TAPS + j];
            peak = (std::max)(peak, std::fabs(acc));
        }
    return peak;
}

//----------------------------------------------------------------------------
// THD+N of a stationary tone: least-squares sine fit over an integer number of cycles
//----------------------------------------------------------------------------
static double thdPlusNoiseDb(const Sig& y, size_t from, size_t n, double f, int sr)
{
    double mean = 0, a = 0, b = 0;
    for (size_t i = 0; i < n; ++i) mean += y[2 * (from + i)];
    mean /= n;
    for (size_t i = 0; i < n; ++i) {
        const double v = y[2 * (from + i)] - mean, ph = 2 * PI * f * i / sr;
        a += v * std::cos(ph); b += v * std::sin(ph); }
    a *= 2.0 / n; b *= 2.0 / n;
    double res = 0, fund = 0;
    for (size_t i = 0; i < n; ++i) {
        const double v = y[2 * (from + i)] - mean, ph = 2 * PI * f * i / sr;
        const double h = a * std::cos(ph) + b * std::sin(ph);
        res += (v - h) * (v - h); fund += h * h; }
    return dB(std::sqrt(res / fund));
}

//============================================================================
// TESTS
//============================================================================

// 0. The reference meter must itself be right, otherwise every later number is meaningless.
static void test0_reference()
{
    std::printf("\n[0] Self-check of the reference true-peak meter\n");
    const int sr = 48000;
    for (double f : { 3000.0, 12000.0, 18000.0 })
    {
        std::vector<double> x(sr, 0.0);
        for (int i = 0; i < sr; ++i) x[i] = std::sin(2 * PI * f * i / sr + PI / 4);   // true peak = 1.0 (0 dBTP)
        const double tp = dB(truePeakRef(stereo(x), 1000, 20000));
        check(std::fabs(tp) < 0.05, fmt("reference on %.0f Hz sine", f), fmt("%+.3f dBTP (expected 0.000, tol 0.05)", tp));
    }
}

// 1. Latency is exact and does not depend on TP / oversampling.
static void test1_latency()
{
    std::printf("\n[1] Latency (impulse response)\n");
    for (int tp = 0; tp <= 1; ++tp)
        for (int os : { 4, 8 })
        {
            Cfg c; c.thr = 0; c.ceil = 0; c.tp = tp; c.os = os;
            const size_t N = 30000, at = 20000;
            std::vector<double> x(N, 0.0); x[at] = 0.5;
            int lat = 0;
            const Sig y = run(c, stereo(x), 512, &lat);
            size_t idx = 0; float m = 0;
            for (size_t i = 0; i < N; ++i) if (std::fabs(y[2 * i]) > m) { m = std::fabs(y[2 * i]); idx = i; }
            check(idx == at + lat && std::fabs(m - 0.5f) < 1e-6f,
                  fmt("TP=%.0f os=%.0f", tp, os), fmt("delay %.0f samples (reported %.0f)", double(idx - at), lat));
        }
}

// 2. Below the ceiling the limiter must be bit-transparent (pure delay).
static void test2_transparency()
{
    std::printf("\n[2] Transparency below the ceiling\n");
    for (int os : { 4, 8 })
    {
        Cfg c; c.thr = 0; c.ceil = 0; c.tp = true; c.os = os;
        const size_t N = 48000; Rng rng(7); std::vector<double> x(N);
        for (size_t i = 0; i < N; ++i) x[i] = 0.4 * std::sin(2 * PI * 440.0 * i / 48000.0) + 0.2 * rng.gauss() * 0.5;
        int lat = 0; const Sig y = run(c, stereo(x), 512, &lat);
        double err = 0;
        for (size_t i = 2000; i < N; ++i) err = (std::max)(err, std::fabs((double)y[2 * i] - (double)(float)x[i - lat]));
        check(err < 1e-6, fmt("pure delay, os=%.0fx", os), fmt("max error %.2e (limit 1e-6)", err));
    }
}

// 3. Sample-peak ceiling is a hard guarantee when True Peak is off.
static void test3_sample_ceiling()
{
    std::printf("\n[3] Sample-peak ceiling (True Peak OFF), ceiling -1 dB, drive +12 dB\n");
    for (int sr : { 44100, 48000, 96000 })
        for (float look : { 0.5f, 1.25f, 5.0f })
        {
            double worst = -99; std::string wn;
            for (const Stim& s : makeStimuli(sr))
            {
                Cfg c; c.sr = sr; c.tp = false; c.look = look;
                const Sig y = run(c, stereo(s.x));
                const double p = dB(samplePeak(y, s.onset, y.size() / 2));
                if (p > worst) { worst = p; wn = s.name; }
            }
            check(worst <= -1.0 + 0.02, fmt("sr=%.0f look=%.2f ms", sr, look),
                  fmt("worst %+.3f dBFS", worst) + " (limit -0.98) - " + wn);
        }
}

// 4. True-peak ceiling, measured with the independent reference meter.
//    FAIL limits = regression baseline (measured behaviour + margin).
//    GOAL        = what a "pro" limiter should reach (<= 0.3 dB over the ceiling), reported as WARN, never fails.
static void test4_true_peak_ceiling()
{
    std::printf("\n[4] True-peak ceiling (TP ON), ceiling -1 dB, drive +12 dB\n");
    struct Mode { int os; double baselineDb; };
    const double GOAL_DB = 0.3;
    for (const Mode m : { Mode{ 4, 1.3 }, Mode{ 8, 0.7 } })
        for (float look : { 0.5f, 1.25f, 3.0f })
        {
            double worst = -99; std::string wn;
            for (const Stim& s : makeStimuli(48000))
            {
                Cfg c; c.tp = true; c.os = m.os; c.look = look;
                const Sig y = run(c, stereo(s.x));
                const double p = dB(truePeakRef(y, s.onset - 200, (std::min)(s.onset + 12000, y.size() / 2)));
                if (p > worst) { worst = p; wn = s.name; }
            }
            const double over = worst + 1.0;   // dB above the -1 dB ceiling
            check(over <= m.baselineDb, fmt("os=%.0fx look=%.2f ms", m.os, look),
                  fmt("worst %+.2f dB over ceiling (limit %+.2f)", over, m.baselineDb) + " - " + wn);
            warn(over <= GOAL_DB, "   target <= +0.30 dB", fmt("%+.2f dB", over));
        }
    // other sample rates, one broadband stimulus
    for (int sr : { 44100, 96000 })
    {
        const Stim s = makeStimuli(sr)[18];   // white noise burst
        Cfg c; c.sr = sr; c.os = 8;
        const Sig y = run(c, stereo(s.x));
        const double over = dB(truePeakRef(y, s.onset - 200, (std::min)(s.onset + 12000, y.size() / 2))) + 1.0;
        check(over <= 0.7, fmt("os=8x noise, sr=%.0f", sr), fmt("%+.2f dB over ceiling (limit +0.70)", over));
    }
}

// 5. Low-frequency distortion: the hold time is what keeps bass clean.
static void test5_lf_distortion()
{
    std::printf("\n[5] Bass distortion (THD+N), tone 6 dB into the limiter\n");
    const int sr = 48000;
    for (double f : { 40.0, 50.0, 100.0 })
    {
        double th[2] = { 0, 0 };
        int k = 0;
        for (float hold : { 0.0f, 30.0f })
        {
            Cfg c; c.thr = -6; c.ceil = -3; c.hold = hold; c.look = 1.25f;
            std::vector<double> x(sr * 3);
            for (size_t i = 0; i < x.size(); ++i) x[i] = 0.9 * std::sin(2 * PI * f * i / sr);
            const Sig y = run(c, stereo(x));
            const size_t cycles = static_cast<size_t>(f);                  // 1 s = f cycles (integer)
            th[k++] = thdPlusNoiseDb(y, sr, static_cast<size_t>(std::round(cycles * sr / f)), f, sr);
        }
        check(th[1] < -80.0, fmt("%.0f Hz, hold 30 ms", f), fmt("%.1f dB (limit -80)", th[1]));
        std::printf("         (info) %.0f Hz with hold 0 ms: %.1f dB\n", f, th[0]);
    }
}

// 6. Result must not depend on how the host cuts the audio into blocks.
static void test6_block_independence()
{
    std::printf("\n[6] Block-size independence (bit-exact)\n");
    const Stim s = makeStimuli(48000)[18];
    for (int os : { 4, 8 })
    {
        Cfg c; c.os = os;
        const Sig ref = run(c, stereo(s.x), 512);
        for (int block : { 1, 37, 480, 4096 })
        {
            const Sig y = run(c, stereo(s.x), block);
            check(std::memcmp(ref.data(), y.data(), ref.size() * sizeof(float)) == 0,
                  fmt("os=%.0fx block=%.0f vs 512", os, block), "");
        }
    }
}

// 7. Live switching of 4x/8x and TP on/off: no clicks, ceiling still respected.
static void test7_live_switching()
{
    std::printf("\n[7] Live switching 4x<->8x and TP on/off\n");
    const int sr = 48000;
    Cfg c; c.tp = true; c.os = 4;
    Rng rng(3);
    std::vector<double> x(sr * 3);
    for (size_t i = 0; i < x.size(); ++i) x[i] = 0.5 * std::sin(2 * PI * 1000.0 * i / sr) + 0.3 * rng.gauss() * 0.5;
    const Sig in = stereo(x);

    auto maxStep = [](const Sig& y, size_t from) { double m = 0; for (size_t i = from + 1; i < y.size() / 2; ++i) m = (std::max)(m, (double)std::fabs(y[2 * i] - y[2 * (i - 1)])); return m; };
    const Sig fixed4 = run(c, in);

    CLimiter2 L; configure(L, c); L.start(sr);
    Sig y = in; int flip = 0;
    for (long i = 0; i < (long)x.size(); i += 512)
    {
        if ((i / 512) % 8 == 0) { ++flip; L.setTruePeakOversampling((flip & 1) ? 8 : 4); L.setTruePeak((flip % 3) != 0); }
        L.process(&y[2 * i], (int)(std::min<long>)(512, (long)x.size() - i));
    }
    bool finite = true; for (float v : y) if (!std::isfinite(v)) finite = false;
    const double step = maxStep(y, 2000), stepRef = maxStep(fixed4, 2000);
    check(finite, "output finite", "");
    check(step <= stepRef + 0.02, "no click when switching", fmt("max step %.3f vs %.3f static (+0.02)", step, stepRef));
    check(dB(samplePeak(y, 2000, y.size() / 2)) <= -1.0 + 0.6, "ceiling respected while switching", fmt("%+.2f dBFS", dB(samplePeak(y, 2000, y.size() / 2))));
}

// 8. Release time constant (hold + release) behaves as set.
static void test8_release()
{
    std::printf("\n[8] Release timing\n");
    const int sr = 48000;
    for (float rel : { 100.0f, 300.0f })
    {
        Cfg c; c.thr = -12; c.ceil = -1; c.hold = 30; c.rel = rel; c.tp = false;
        const size_t N = sr * 3, burstEnd = sr * 1;
        std::vector<double> x(N);
        for (size_t i = 0; i < N; ++i) x[i] = ((i < burstEnd) ? 0.9 : 0.05) * std::sin(2 * PI * 1000.0 * i / sr);
        int lat = 0; const Sig y = run(c, stereo(x), 512, &lat);
        // gain on the quiet part = |out| / (|in| * drive), measured per 1 ms
        const double drive = lin(12.0);
        std::vector<double> g;
        for (size_t t = burstEnd; t + 48 < N; t += 48)
        {
            double po = 0, pi_ = 0;
            for (size_t i = t; i < t + 48; ++i) { po = (std::max)(po, (double)std::fabs(y[2 * (i + lat)])); pi_ = (std::max)(pi_, std::fabs(x[i]) * drive); }
            g.push_back(po / pi_);
        }
        const double g0 = g.front(), gEnd = g.back();
        size_t k = 0; while (k < g.size() && g[k] < g0 + (gEnd - g0) * 0.632) ++k;
        const double tms = k * 1.0, expect = 30.0 + rel;
        check(std::fabs(tms - expect) / expect < 0.15, fmt("release %.0f ms, hold 30 ms", rel),
              fmt("63%% recovery after %.0f ms (expected ~%.0f, tol 15%%)", tms, expect));
        check(gEnd > 0.97, "gain returns to unity", fmt("final gain %.3f", gEnd));
    }
}

// 9. Final Security: a ceiling lowered abruptly leaves louder samples in the delay line;
//    the clamp must catch them (it clamps to +/-ceiling).
static void test9_final_security()
{
    std::printf("\n[9] Final Security (ceiling lowered 0 -> -12 dB while a loud tone is playing)\n");
    const int sr = 48000; const double newCeil = lin(-12.0);
    double peak[2] = { 0, 0 };
    for (int sec = 0; sec <= 1; ++sec)
    {
        Cfg c; c.thr = 0; c.ceil = 0; c.tp = false; c.sec = sec != 0;
        CLimiter2 L; configure(L, c); L.start(sr);
        const long N = 512 * 100, change = 512 * 50;
        Sig y(2 * N); for (long i = 0; i < N; ++i) y[2 * i] = y[2 * i + 1] = (float)(0.9 * std::sin(2 * PI * 1000.0 * i / sr));
        for (long i = 0; i < N; i += 512)
        {
            if (i == change) L.setOutput(-12.0f, -30.0f, 0.0f);
            L.process(&y[2 * i], 512);
        }
        peak[sec] = samplePeak(y, change, change + 200);   // the few samples that were already in the delay line
    }
    std::printf("         (info) without Final Security: peak %.3f right after the change (ceiling %.3f)\n", peak[0], newCeil);
    check(peak[1] <= newCeil * (1.0 + 1e-6), "with Final Security: never above the ceiling", fmt("peak %.4f (limit %.4f)", peak[1], newCeil));
}

// 10. Stereo link: both channels get the same gain (no image shift).
static void test10_stereo_link()
{
    std::printf("\n[10] Stereo link\n");
    const int sr = 48000; Cfg c; c.thr = -12; c.ceil = -1; c.tp = true; c.os = 8;
    const size_t N = sr * 2; Sig in(2 * N);
    for (size_t i = 0; i < N; ++i) { in[2 * i] = (float)(0.9 * std::sin(2 * PI * 100.0 * i / sr)); in[2 * i + 1] = (float)(0.05 * std::sin(2 * PI * 1700.0 * i / sr)); }
    int lat = 0; const Sig y = run(c, in, 512, &lat);
    double eL = 0, iL = 0, eR = 0, iR = 0;
    for (size_t i = sr; i < N - lat; ++i)
    {
        eL += double(y[2 * (i + lat)]) * y[2 * (i + lat)];   iL += double(in[2 * i]) * in[2 * i];
        eR += double(y[2 * (i + lat) + 1]) * y[2 * (i + lat) + 1]; iR += double(in[2 * i + 1]) * in[2 * i + 1];
    }
    const double gL = std::sqrt(eL / iL), gR = std::sqrt(eR / iR);
    check(std::fabs(dB(gL) - dB(gR)) < 0.05, "L and R gains match", fmt("L %.3f dB, R %.3f dB (tol 0.05)", dB(gL), dB(gR)));
}

// 11. stop(): fades to silence within FADE_OUT_MS.
static void test11_stop()
{
    std::printf("\n[11] stop() fade-out\n");
    const int sr = 48000; Cfg c; c.thr = 0; c.ceil = 0; c.tp = false;
    CLimiter2 L; configure(L, c); L.start(sr);
    Sig y(2 * 4800); for (size_t i = 0; i < 4800; ++i) y[2 * i] = y[2 * i + 1] = (float)(0.5 * std::sin(2 * PI * 200.0 * i / sr));
    L.process(y.data(), 2400);
    L.stop();
    L.process(&y[2 * 2400], 2400);
    const size_t fade = (size_t)std::ceil(sr * 5.0 / 1000.0);
    double mx = 0; for (size_t i = 2400 + fade - 2; i < 2400 + fade; ++i) mx = (std::max)(mx, (double)std::fabs(y[2 * i]));
    bool finite = true; for (float v : y) if (!std::isfinite(v)) finite = false;
    check(finite, "output finite", "");
    check(mx < 0.01, "silent at the end of the fade-out", fmt("residual %.4f (limit 0.01)", mx));
}

// 12. Cost: informational, asserted only for optimised builds.
static void test12_cpu()
{
    std::printf("\n[12] CPU cost\n");
    const int sr = 48000; const size_t N = sr * 10;
    Rng rng(5); Sig in(2 * N); for (auto& v : in) v = (float)(rng.gauss() * 0.4);
    for (int os : { 4, 8 })
    {
        Cfg c; c.tp = true; c.os = os;
        const auto t0 = std::chrono::steady_clock::now();
        const Sig y = run(c, in);
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const double rt = 10.0 / sec;
#ifdef NDEBUG
        check(rt > 50.0, fmt("os=%.0fx", os), fmt("%.0fx real time = %.2f%% of one core (limit: > 50x)", rt, 100.0 / rt));
#else
        std::printf("  [info] os=%dx: %.0fx real time (unoptimised build, not asserted)\n", os, rt);
#endif
        (void)y;
    }
}

// 13. Auto release: brief peaks recover fast, sustained limiting recovers slowly,
//     and nothing else (ceiling, bass distortion, determinism) gets worse.
static double recoveryT90Ms(const Cfg& c, double burstSec)
{
    const int sr = c.sr; const size_t on = sr / 3, end = on + (size_t)(burstSec * sr), N = end + sr * 2;
    std::vector<double> x(N, 0.0);
    for (size_t i = 0; i < N; ++i) x[i] = ((i < on) ? 0.0 : (i < end) ? 0.9 : 0.05) * std::sin(2 * PI * 1000.0 * i / sr);
    int lat = 0; const Sig y = run(c, stereo(x), 512, &lat);
    const double drive = lin(-c.thr);
    const int win = sr / 1000;                      // 1 ms
    std::vector<double> g;
    for (size_t t = end; t + win < N - lat; t += win)
    {
        double po = 0, pi_ = 0;
        for (size_t i = t; i < t + win; ++i) { po = (std::max)(po, (double)std::fabs(y[2 * (i + lat)])); pi_ = (std::max)(pi_, std::fabs(x[i]) * drive); }
        g.push_back(po / pi_);
    }
    const double g0 = g.front();
    size_t k = 0; while (k < g.size() && g[k] < g0 + (1.0 - g0) * 0.9) ++k;
    return static_cast<double>(k);   // 1 window = 1 ms
}

// Gain ripple (max - min of the applied gain, 1 ms resolution) on a train of loud 10 ms hits over a quiet bed.
static double gainRipple(const Cfg& c, double periodSec)
{
    const int sr = c.sr; const size_t N = sr * 5, period = (size_t)(periodSec * sr), hit = sr / 100;
    std::vector<double> x(N);
    for (size_t i = 0; i < N; ++i) x[i] = (((i % period) < hit) ? 0.9 : 0.2) * std::sin(2 * PI * 1000.0 * i / sr);
    int lat = 0; const Sig y = run(c, stereo(x), 512, &lat);
    const double drive = lin(-c.thr); const int win = sr / 1000;
    double gmin = 9, gmax = 0;
    for (size_t t = sr * 3; t + win < N - lat; t += win)
    {
        double po = 0, pi_ = 0;
        for (size_t i = t; i < t + win; ++i) { po = (std::max)(po, (double)std::fabs(y[2 * (i + lat)])); pi_ = (std::max)(pi_, std::fabs(x[i]) * drive); }
        gmin = (std::min)(gmin, po / pi_); gmax = (std::max)(gmax, po / pi_);
    }
    return gmax - gmin;
}

// Mean level (dB) of a steady 1 kHz bed under a kick drum repeating every `period` seconds.
static double bedMeanDb(const Cfg& c, double period)
{
    const int sr = c.sr; const size_t N = sr * 8; std::vector<double> x(N);
    for (size_t i = 0; i < N; ++i) {
        const double t = double(i) / sr, tk = std::fmod(t, period);
        x[i] = 0.25 * std::sin(2 * PI * 1000.0 * t) + 0.9 * std::exp(-tk * 18.0) * std::sin(2 * PI * (55.0 + 90.0 * std::exp(-tk * 30.0)) * tk); }
    int lat = 0; const Sig y = run(c, stereo(x), 512, &lat);
    const int win = sr / 200; const double drive = lin(-c.thr); double sum = 0; int n = 0;
    for (size_t t = sr * 2; t + win < N - lat; t += win) {
        double a = 0, b = 0;
        for (int i = 0; i < win; ++i) { const double ph = 2 * PI * 1000.0 * (t + i) / sr, v = y[2 * (t + i + lat)]; a += v * std::cos(ph); b += v * std::sin(ph); }
        sum += dB(2.0 * std::sqrt(a * a + b * b) / win / (0.25 * drive)); ++n; }
    return sum / n;
}

static void test13_auto_release()
{
    std::printf("\n[13] Auto release (slow end = Release 300 ms, hold 30 ms)\n");
    Cfg m; m.thr = -12; m.ceil = -1; m.tp = false; m.rel = 300; m.hold = 30; m.autoRel = false;
    Cfg a = m; a.autoRel = true;
    const double mShort = recoveryT90Ms(m, 0.04), mLong = recoveryT90Ms(m, 1.5);
    const double aShort = recoveryT90Ms(a, 0.04), aLong = recoveryT90Ms(a, 1.5);
    std::printf("         (info) time to recover 90%% of the gain reduction:\n");
    std::printf("         (info)   manual : 40 ms hit %4.0f ms | 1.5 s passage %4.0f ms\n", mShort, mLong);
    std::printf("         (info)   auto   : 40 ms hit %4.0f ms | 1.5 s passage %4.0f ms\n", aShort, aLong);
    check(aShort < 0.5 * mShort, "brief hit recovers at least 2x faster", fmt("%.0f ms vs %.0f ms manual", aShort, mShort));

    // Dense material (a hit every 100 ms): a fast release pumps, a slow one does not. Auto must behave like the slow one.
    Cfg f = m; f.rel = 20;
    const double rFast = gainRipple(f, 0.1), rSlow = gainRipple(m, 0.1), rAuto = gainRipple(a, 0.1);
    std::printf("         (info) gain ripple, hits every 100 ms: manual 20 ms %.2f | manual 300 ms %.2f | auto %.2f\n", rFast, rSlow, rAuto);
    check(rAuto < 0.6 * rFast, "dense hits: much less pumping than a fast release", fmt("ripple %.2f vs %.2f (limit 60%%)", rAuto, rFast));
    check(rAuto < 0.5 * (rFast + rSlow), "dense hits: below the midpoint fast/slow", fmt("ripple %.2f < %.2f", rAuto, 0.5 * (rFast + rSlow)));
    const double sFast = gainRipple(f, 1.0), sAuto = gainRipple(a, 1.0), sSlow = gainRipple(m, 1.0);
    std::printf("         (info) gain ripple, hits every 1 s:    manual 20 ms %.2f | manual 300 ms %.2f | auto %.2f\n", sFast, sSlow, sAuto);
    check(sAuto > 0.9 * sFast, "sparse hits: recovers fully like a fast release", fmt("ripple %.2f vs %.2f", sAuto, sFast));

    // Calibration: Auto must move from "fast" to "slow" as the tempo goes up, and sit halfway at 120 BPM.
    {
        double pct[3]; const double bpm[3] = { 100.0, 120.0, 140.0 };
        for (int k = 0; k < 3; ++k) {
            const double per = 60.0 / bpm[k];
            const double fastL = bedMeanDb(f, per), slowL = bedMeanDb(m, per), autoL = bedMeanDb(a, per);
            pct[k] = 100.0 * (autoL - fastL) / (slowL - fastL);
        }
        std::printf("         (info) Auto position between fast (0%%) and slow (100%%): 100 BPM %.0f%% | 120 BPM %.0f%% | 140 BPM %.0f%%\n", pct[0], pct[1], pct[2]);
        check(pct[1] >= 35.0 && pct[1] <= 65.0, "120 BPM: Auto sits halfway", fmt("%.0f %% (35..65)", pct[1]));
        check(pct[0] < pct[1] && pct[1] < pct[2], "faster tempo -> slower release", fmt("%.0f < %.0f < %.0f", pct[0], pct[1], pct[2]));
    }

    // ceiling guarantee is untouched (sample peak, TP off)
    double worst = -99; std::string wn;
    for (const Stim& s : makeStimuli(48000))
    {
        Cfg c; c.tp = false; c.autoRel = true;
        const Sig y = run(c, stereo(s.x));
        const double p = dB(samplePeak(y, s.onset, y.size() / 2));
        if (p > worst) { worst = p; wn = s.name; }
    }
    check(worst <= -1.0 + 0.02, "sample-peak ceiling with Auto", fmt("worst %+.3f dBFS", worst) + " - " + wn);

    // bass distortion on a sustained tone
    for (double f : { 40.0, 50.0, 100.0 })
    {
        Cfg c; c.thr = -6; c.ceil = -3; c.hold = 30; c.autoRel = true;
        std::vector<double> x(48000 * 3);
        for (size_t i = 0; i < x.size(); ++i) x[i] = 0.9 * std::sin(2 * PI * f * i / 48000.0);
        const Sig y = run(c, stereo(x));
        const double th = thdPlusNoiseDb(y, 48000, 48000, f, 48000);
        check(th < -80.0, fmt("%.0f Hz sustained, Auto", f), fmt("%.1f dB (limit -80)", th));
    }

    // determinism + live toggling
    const Stim s = makeStimuli(48000)[18];
    Cfg c; c.autoRel = true;
    const Sig ref = run(c, stereo(s.x), 512);
    bool same = true;
    for (int block : { 1, 37, 4096 }) { const Sig y = run(c, stereo(s.x), block); same &= (std::memcmp(ref.data(), y.data(), ref.size() * sizeof(float)) == 0); }
    check(same, "block-size independent (bit-exact)", "");

    CLimiter2 L; Cfg d; configure(L, d); L.start(48000);
    Sig y = stereo(s.x); bool finite = true;
    for (long i = 0, n = (long)(y.size() / 2); i < n; i += 512)
    {
        L.setAutoRelease(((i / 512) % 6) < 3);
        L.process(&y[2 * i], (int)(std::min<long>)(512, n - i));
    }
    for (float v : y) if (!std::isfinite(v)) finite = false;
    check(finite && dB(samplePeak(y, s.onset, y.size() / 2)) <= -1.0 + 0.02, "live Manual<->Auto toggling", fmt("peak %+.3f dBFS", dB(samplePeak(y, s.onset, y.size() / 2))));
}

int main()
{
    std::printf("CLimiter2 regression tests\n==========================\n");
    test0_reference();
    test1_latency();
    test2_transparency();
    test3_sample_ceiling();
    test4_true_peak_ceiling();
    test5_lf_distortion();
    test6_block_independence();
    test7_live_switching();
    test8_release();
    test9_final_security();
    test10_stereo_link();
    test11_stop();
    test12_cpu();
    test13_auto_release();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
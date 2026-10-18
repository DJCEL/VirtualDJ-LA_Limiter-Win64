# Limiter2 regression tests

Self-contained (no dependency). One file: `test_limiter2.cpp`. Exit code 0 = all checks passed.

## Run

Windows (Developer Command Prompt for VS): `tests\run_tests.bat`

Linux / macOS / WSL:

    g++ -O2 -DNDEBUG -std=c++17 -I src tests/test_limiter2.cpp src/Limiter2.cpp -o tests/test_limiter2 && tests/test_limiter2

## What is tested

| # | Test | Limit |
|---|------|-------|
| 0 | Self-check of the reference true-peak meter (16x Kaiser sinc) | within 0.05 dB on 3/12/18 kHz sines |
| 1 | Latency = impulse delay = `getLatencySamples()`, for TP off/on and 4x/8x | exact |
| 2 | Below the ceiling the output is a pure delay | error < 1e-6 |
| 3 | Sample-peak ceiling (TP off), 44.1/48/96 kHz, look-ahead 0.5/1.25/5 ms, 21 stimuli | <= ceiling + 0.02 dB |
| 4 | True-peak ceiling (TP on) measured by the reference meter | regression: 4x <= +1.3 dB, 8x <= +0.7 dB |
| 5 | Bass distortion (THD+N) at 40/50/100 Hz | < -80 dB with hold 30 ms |
| 6 | Output independent of the host block size (1, 37, 480, 4096 vs 512) | bit-exact |
| 7 | Live switching 4x<->8x and TP on/off | no click, finite, ceiling respected |
| 8 | Release time (hold + release, 63 % recovery) | within 15 % |
| 9 | Final Security when the ceiling is lowered abruptly | never above the new ceiling |
| 10 | Stereo link | L/R gains equal within 0.05 dB |
| 11 | `stop()` fade-out | silent at the end of the fade |
| 12 | CPU cost (optimised build only) | > 50x real time |

## Reading the report

- `PASS` / `FAIL`: counted, a FAIL makes the run fail.
- `WARN` / `GOAL`: quality target (true-peak overshoot <= 0.3 dB). Shown, never fails.
  The test-4 limits are a baseline of the measured behaviour: tighten them when the
  detector improves, so any regression is caught.
- `(info)`: extra figure, not checked (e.g. bass distortion with hold = 0 ms).

## Practical note (true peak)

With full-band noise the true peak can exceed the ceiling by up to ~0.6 dB (8x, look-ahead 0.5-1.25 ms)
or ~1.1 dB (4x). For a strict -1 dBTP target, set the ceiling to about -1.5 dB (8x) or -2 dB (4x).

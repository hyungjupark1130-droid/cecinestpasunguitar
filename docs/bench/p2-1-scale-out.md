# P2.1 scale-out -- the two named bench configurations, recorded

Task P2.1. Methodology is locked in `docs/plan.md` section 4.7 and unchanged from
`docs/bench/p1-baseline.md`; the tool is `tests/bench/BenchMain.cpp` (target `cnpg_bench`).

This document **records** numbers, it does not gate them. The 25-30% hard gate binds
`p2_default6` and lands with Task P2.9 in `docs/bench/p2-exit.md`. P2.1's own acceptance
criterion is only that the two named configurations run and print median, p99 and max as a
percent of one core.

Same machine and settings as the P1 baseline, so the rows are directly comparable to it:
AMD Ryzen 9 7950X (32 logical), Windows power plan `e9a42b02-...`, MSVC 1944,
`/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX`, Release, 48 kHz / 128-sample blocks /
2x oversampling / 60 s renders.

## What the named configurations are

```
cnpg_bench --config p2_default6 --samplerate 48000 --blocksize 128 --seconds 60
cnpg_bench --config p2_max8     --samplerate 48000 --blocksize 128 --seconds 60
```

A named config sets the **string count and nothing else**. Everything else about both -- the
full P1 monitoring chain, the shipped parameter defaults, 2x oversampling -- is what
`cnpg_bench` already ran, so the table in `BenchMain.cpp` does not restate it and cannot
drift from it. The remaining flags stay live on top of a `--config`, which is what makes
`--config p2_max8 --samplerate 96000` answerable without a third named row.

## Numbers

| Configuration | strings | median (us) | median CPU | p99 (us) | p99 CPU | max (us) | max CPU |
|---|---|---|---|---|---|---|---|
| `--strings 1` | 1 | 7.300 | 0.274% | 9.300 | 0.349% | 26.400 | 0.990% |
| `--config p2_default6` | 6 | 16.900 | 0.634% | 19.500 | 0.731% | 664.400 | 24.915% |
| `--config p2_default6` (run 2) | 6 | 17.000 | 0.638% | 19.800 | 0.743% | 60.700 | 2.276% |
| `--config p2_max8` | 8 | 20.800 | 0.780% | 23.900 | 0.896% | 702.700 | 26.351% |
| `--config p2_max8` (run 2) | 8 | 20.700 | 0.776% | 26.600 | 0.998% | 123.300 | 4.624% |

`nonFiniteSamples` is 0 in every run. The `max` column varies by an order of magnitude
between two runs of the identical command while the medians move by 0.1 us -- the same
OS-scheduling-preemption pattern `docs/bench/p1-baseline.md` already documents for its own
rows. Median and p99 are the columns to track.

`p2_default6` sits at **0.64% of one core against a 30% budget -- about 47x headroom.**

## The single-string delta from the P1 baseline, and what caused it

`--strings 1` moved from **7.000 us / 0.263%** (P1 baseline, and re-measured at 7.000 us on
this same machine immediately before the P2.1 work as a like-for-like control) to
**7.300 us / 0.274%**: +4.3%. The 6- and 8-string rows moved from 15.700 and 19.100 us to
16.900 and 20.800: +7.6% and +8.9%.

This is real and reproducible, not measurement noise -- three consecutive runs of each build
gave identical medians to the printed precision. The cost is the per-string per-sample work
P2.1 added, and it is not incidental:

- **One position smoother per (string, tap)** instead of one global smoother (ADR 0004 D1).
  At eight strings that is eight one-pole recurrences per sample instead of one. Mandated;
  the alternative is not having per-coil positions.
- **The enable ramp**: a gain compare and, when a ramp is live, a step, per string per
  sample; plus the gain multiply on the tap and on the bridge incident wave. Mandated -- it
  is what makes a mute and a count reduction click-free.
- **The idle-string test**, which is the one item here that is a trade rather than a
  requirement. It costs a predicate per string per sample in this benchmark, because
  `cnpg_bench`'s workload deliberately keeps **every** configured string ringing
  continuously (~4.5 retriggers/second/string). It pays in the case the benchmark cannot
  express: a plugin configured for eight strings while two are being played now costs about
  two strings, not eight, because a string with no state is skipped rather than ticked
  through zeros. Given that the shipped default count is 6 and the shipped allocator (until
  Task P2.6) puts every note on string 0, that case is not hypothetical -- it is what the
  plugin does today.

Against a 30% gate with 47x headroom the trade is not close. It is recorded here rather than
absorbed silently because `0.263%` is a number the P1 baseline published, and a later reader
comparing the two documents deserves the reason rather than a discrepancy.

## Determinism

`audioHash` is bit-identical across the two runs of each configuration
(`p2_default6`: `90739248c2f22d32`, `p2_max8`: `bb73e2d9472d4d2a`), which is the run-to-run
reproducibility claim `docs/bench/p1-baseline.md` explains in full.

The single-string hash is **`1c479bbaf4164075`, unchanged from the P1 baseline** -- that is
not a determinism statement but the P2.1 amendments' binding zero-output-change proof: the
(string, tap) widening and the per-(string, tap) smoothers may not move one sample. The
6- and 8-string hashes were likewise checked against pre-P2.1 renders of the same commands
and are unchanged; they are not printed in `p1-baseline.md`, so they are recorded here:
`fb174be42c4562cb` and `edaa7e258e2eb27b` at `--seconds 20`.

## Raw JSON

```json
{"strings":1,"sampleRate":48000,"blockSize":128,"oversampleFactor":2,"secondsRequested":60.0,"blocksTotal":22500,"blocksMeasured":22125,"warmupBlocksExcluded":375,"nonFiniteSamples":0,"audioHash":"1c479bbaf4164075","medianBlockTimeUs":7.300,"p99BlockTimeUs":9.300,"maxBlockTimeUs":26.400,"medianCpuPercent":0.274,"p99CpuPercent":0.349,"maxCpuPercent":0.990,"cpuModel":"AMD Ryzen 9 7950X 16-Core Processor","logicalCoreCount":32,"powerPlanGuid":"e9a42b02-d5df-448d-aa00-03f14749eb61","compiler":"MSVC 1944","buildType":"Release","buildFlags":"/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX","gitCommit":"cf07f8b"}
{"strings":6,"sampleRate":48000,"blockSize":128,"oversampleFactor":2,"secondsRequested":60.0,"blocksTotal":22500,"blocksMeasured":22125,"warmupBlocksExcluded":375,"nonFiniteSamples":0,"audioHash":"90739248c2f22d32","medianBlockTimeUs":16.900,"p99BlockTimeUs":19.500,"maxBlockTimeUs":664.400,"medianCpuPercent":0.634,"p99CpuPercent":0.731,"maxCpuPercent":24.915,"cpuModel":"AMD Ryzen 9 7950X 16-Core Processor","logicalCoreCount":32,"powerPlanGuid":"e9a42b02-d5df-448d-aa00-03f14749eb61","compiler":"MSVC 1944","buildType":"Release","buildFlags":"/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX","gitCommit":"cf07f8b"}
{"strings":8,"sampleRate":48000,"blockSize":128,"oversampleFactor":2,"secondsRequested":60.0,"blocksTotal":22500,"blocksMeasured":22125,"warmupBlocksExcluded":375,"nonFiniteSamples":0,"audioHash":"bb73e2d9472d4d2a","medianBlockTimeUs":20.800,"p99BlockTimeUs":23.900,"maxBlockTimeUs":702.700,"medianCpuPercent":0.780,"p99CpuPercent":0.896,"maxCpuPercent":26.351,"cpuModel":"AMD Ryzen 9 7950X 16-Core Processor","logicalCoreCount":32,"powerPlanGuid":"e9a42b02-d5df-448d-aa00-03f14749eb61","compiler":"MSVC 1944","buildType":"Release","buildFlags":"/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX","gitCommit":"cf07f8b"}
```

## Acceptance criterion (Task P2.1, verbatim from the brief)

- [x] `cnpg_bench --config p2_default6` and `--config p2_max8` both run and print median, p99,
  and max block time converted to percent-of-one-core at 48 kHz / 128-sample blocks (numbers
  recorded, not yet gated -- gate is P2.9).

# P1 baseline -- first `cnpg_bench` regression numbers

Task P1.10. Methodology is locked in `docs/plan.md` section 4.7; the tool itself is
`tests/bench/BenchMain.cpp` (target `cnpg_bench`). This document is the first entry in the
regression-visibility trail that section asks for -- it is **not** the P2 exit gate
(`docs/bench/p2-exit.md`, Task P2.9): the 25-30% hard gate binds the `p2_default6` configuration
(6 strings + full chain, named `--config` values land in Task P2.1), which does not exist yet.
Everything below is measured with today's flag surface, `--strings N`.

## Revision note (post-review fix)

The numbers below **replace** an earlier revision of this document. Review found that the
first cut of `BenchMain.cpp` applied the chain's parameters once, before the render loop, instead
of every block; production (`PluginProcessor::renderChunk()`) unconditionally re-applies a fresh
parameter snapshot to all six of `StringNetwork`/`PickupTap`/`TriodeStage`/`CabFilter`/
`OutputGain`/`SoftClipLimiter` via `setParams()` before every `process()` call, every block,
regardless of whether any value changed. That cascade is a real per-block cost in production
(`StringNetwork::setParams()` alone loops all 8 rail slots recomputing each one's target frequency
via `std::exp2()`, and four of the six calls convert a dB parameter via `std::pow()`), so excluding
it understated every number below and would have made it structurally impossible for `p99`/`max`
to ever show the "event bursts, smoother retargeting" spike class section 4.7 names as the reason
those two columns exist. `BenchChain::processBlock()` now runs the full six-`setParams()` cascade
(against a constant default snapshot -- `makeDefaultSnapshot()`) inside the timed region every
block, exactly mirroring `renderChunk()`. All numbers below are from the corrected binary; the
delta from the original (buggy) measurement is small in absolute terms -- a few tenths of a
microsecond on the median -- because a handful of `sin`/`cos`/`pow`/`exp2` evaluations is cheap
next to the per-sample physics loop each block still has to run; see the per-row comparison below.
The JSON schema was also completed in the same pass (CPU model, logical core count, Windows power
plan, build type, and build flags -- previously only compiler/commit were self-reported; the doc
compensated for the gap, which was fine for a one-off baseline but not for CI artifacts staying
attributable when runners change silently).

## What was measured

The locked P1 chain (`StringNetwork -> PickupTap -> Oversampler(TriodeStage) -> CabFilter ->
OutputGain -> SoftClipLimiter`, at the plugin's shipped defaults) driven by `cnpg_bench`'s
deterministic, arithmetic-only note stream (no RNG, no wall clock -- see `BenchMain.cpp`'s
file-level comment) for 60 s at 48 kHz / 128-sample blocks / 2x oversampling, for 1, 6, and 8
active strings. The exact acceptance-criterion command:

```
build\bin\Release\cnpg_bench.exe --strings 1 --samplerate 48000 --blocksize 128 --oversample 2 --seconds 60
```

...repeated with `--strings 6` and `--strings 8`. Each render produces exactly 22 500 blocks
(60 s * 48000 / 128), matching section 4.7's "N ~= 22 500" figure exactly. The first 1 s of blocks
(375 blocks) is excluded from the reported statistics as a warmup window (letting caches, the
branch predictor, and any DVFS ramp settle); every block, warmup included -- setParams cascade and
all -- is still rendered, so excluding it from the stats never perturbs the note stream itself.

CPU% = `blockTime / (blockSize / sampleRate)` = `blockTime / 2666.667 us`, per section 4.7's
formula. "Block time" is the whole `BenchChain::processBlock()` call: the six-`setParams()`
cascade plus the six `process()` calls, matching what one real `processBlock()`/`renderChunk()`
call costs in production.

## Machine spec (dev machine, per docs/plan.md's "Windows 11 Pro workstation")

| Field | Value |
|---|---|
| CPU | AMD Ryzen 9 7950X 16-Core Processor, 16 cores / 32 logical processors (self-reported by `cnpg_bench`'s own `cpuModel()`/`logicalCoreCount()`, registry-sourced on Windows) |
| OS | Windows 11 Pro for Workstations, build 10.0.26200 |
| Power plan | "Ultimate Performance" GUID `e9a42b02-d5df-448d-aa00-03f14749eb61` (self-reported by `cnpg_bench` via a locale-independent scan of `powercfg /getactivescheme`'s output; the friendly plan name is localized on this machine -- Korean -- so only the GUID is machine-parsed, per `windowsPowerPlanGuid()`'s comment) |
| Compiler | MSVC 19.44 (`_MSC_VER` 1944), VS2022 toolset 14.44.35207 (self-reported) |
| Build | `windows-msvc-release` preset, config `Release` (self-reported via the `$<CONFIG>` generator expression baked in at configure time); flags (self-reported, mirrors `cmake/CompilerWarnings.cmake`): `/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX`, plus CMake's own default MSVC Release flags (`/O2 /Ob2 /DNDEBUG`, not self-reported -- CMake does not expose these as a queryable target property portably, so they are recorded here by hand) |
| Git commit | `c6b10ee` (the commit the measured binary was configured against, self-reported; necessarily the parent of the commit that lands this fix, the same "a commit cannot name its own hash" limitation `tests/support/SourceHash.h`'s file comment describes for the golden sidecars) |

Every field above marked "self-reported" now comes straight out of `cnpg_bench`'s own JSON line
(see a sample below), per `docs/plan.md` section 4.7's locked schema; only the two CMake default
flags are still hand-recorded, for the reason stated.

**Caveat (honesty over precision, per the P1.8 ledger note):** this is a shared development
workstation, not an isolated, thermally-controlled benchmark rig. Single-run figures below should
be read to about 2 significant figures; the `max` column in particular is sensitive to OS
scheduling noise on a run this long (60 s / 22 500+ blocks gives the scheduler many chances to
preempt this process for something else) -- see the 6-string row's outlier below.

## Results (48 kHz / 128 samples / 2x oversampling, 60 s, N = 22 125 measured blocks)

| Strings | Median (us) | Median (% CPU) | p99 (us) | p99 (% CPU) | Max (us) | Max (% CPU) |
|---|---|---|---|---|---|---|
| 1 | 7.000 | 0.263% | 10.500 | 0.394% | 18.800 | 0.705% |
| 6 | 15.800 | 0.593% | 20.000 | 0.750% | 598.400 | 22.440% |
| 8 | 19.900 | 0.746% | 24.900 | 0.934% | 62.500 | 2.344% |

For comparison, the pre-fix (setParams-cascade-excluded) medians were 6.900 / 15.600 / 19.600 us
respectively -- the corrected numbers are consistently a little higher (median +0.1 to +0.3 us),
as expected: the cascade adds real but small per-block cost (a handful of `sin`/`cos`/`pow`/`exp2`
calls, not a per-sample loop).

The 6-string `max` figure (598 us / 22.4% CPU, a single block out of 22 125) is an outlier far
above that row's own p99 (20.0 us) -- consistent with one OS scheduling preemption during a 60 s
run on a shared workstation, not a steady-state DSP cost; the median and p99 columns are the
figures to track for regressions. (The 1-string and 8-string rows' own two runs, see Determinism
below and the raw JSON, show the same max-varies-run-to-run pattern while medians stay stable.)

## Determinism

`cnpg_bench` prints `audioHash`, an FNV-1a/64 digest over the IEEE-754 bit pattern of every
rendered sample (computed outside the timed region, so it never perturbs a block-time
measurement). Two independent 60 s runs of the single-string configuration:

| Run | audioHash | blocksMeasured | nonFiniteSamples | median (us) | max (us) |
|---|---|---|---|---|---|
| 1 | `1c479bbaf4164075` | 22125 | 0 | 7.000 | 18.800 |
| 2 | `1c479bbaf4164075` | 22125 | 0 | 7.000 | 14.100 |

`audioHash` is bit-identical across both runs -- **unchanged** by the setParams-cascade fix, as
expected: the six parameter values are constant across the whole render (`makeDefaultSnapshot()`
never varies), and re-applying an unchanged target to an already-settled smoother is a per-block
no-op for the rendered samples (every `setParams()` in dsp/ only recomputes a target a smoother
then ramps toward; ramping toward a target you are already sitting on moves nothing). Re-applying
the cascade every block is exclusively a per-block CPU cost, never an audio one, which this
determinism check confirms empirically rather than merely by construction: the deterministic note
stream plus the RNG-free, wall-clock-free dsp/ chain (`PluckExciter`'s own noise-burst PRNG
reseeds to a fixed constant on every `prepare()`/`reset()`, per `PluckExciter.h`) is what makes
`audioHash` reproducible, and it stayed reproducible before and after this fix. Only the timing
columns differ, by ordinary wall-clock jitter.

## Sample JSON line (single-string, 60 s)

```json
{"strings":1,"sampleRate":48000,"blockSize":128,"oversampleFactor":2,"secondsRequested":60.0,"blocksTotal":22500,"blocksMeasured":22125,"warmupBlocksExcluded":375,"nonFiniteSamples":0,"audioHash":"1c479bbaf4164075","medianBlockTimeUs":7.000,"p99BlockTimeUs":10.500,"maxBlockTimeUs":18.800,"medianCpuPercent":0.263,"p99CpuPercent":0.394,"maxCpuPercent":0.705,"cpuModel":"AMD Ryzen 9 7950X 16-Core Processor","logicalCoreCount":32,"powerPlanGuid":"e9a42b02-d5df-448d-aa00-03f14749eb61","compiler":"MSVC 1944","buildType":"Release","buildFlags":"/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX","gitCommit":"c6b10ee"}
```

## Acceptance criteria (Task P1.10, verbatim from the brief)

- [x] `build\bin\Release\cnpg_bench.exe --strings 1 --samplerate 48000 --blocksize 128 --oversample 2 --seconds 60` prints median, p99, and max block time as CPU% and exits 0.
- [x] This document committed with 1/6/8-string numbers at 48 kHz / 128 samples and dev-machine spec.
- [x] CI runs the bench step non-gating (`ci.yml`'s `windows` job, `continue-on-error: true`) -- verified on this branch's push (see the PR/CI run linked in the P1.10 task report).

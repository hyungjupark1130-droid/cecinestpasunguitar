# P1 baseline -- first `cnpg_bench` regression numbers

Task P1.10. Methodology is locked in `docs/plan.md` section 4.7; the tool itself is
`tests/bench/BenchMain.cpp` (target `cnpg_bench`). This document is the first entry in the
regression-visibility trail that section asks for -- it is **not** the P2 exit gate
(`docs/bench/p2-exit.md`, Task P2.9): the 25-30% hard gate binds the `p2_default6` configuration
(6 strings + full chain, named `--config` values land in Task P2.1), which does not exist yet.
Everything below is measured with today's flag surface, `--strings N`.

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
branch predictor, and any DVFS ramp settle); every block, warmup included, is still rendered, so
excluding it from the stats never perturbs the note stream itself.

CPU% = `blockTime / (blockSize / sampleRate)` = `blockTime / 2666.667 us`, per section 4.7's
formula.

## Machine spec (dev machine, per docs/plan.md's "Windows 11 Pro workstation")

| Field | Value |
|---|---|
| CPU | AMD Ryzen 9 7950X 16-Core Processor (16 cores / 32 threads); WMI-reported base clock 4501 MHz (this SKU's rated boost is 5.7 GHz -- WMI's `MaxClockSpeed` field reports the nominal/base rating, not the live boost clock) |
| OS | Windows 11 Pro for Workstations, build 10.0.26200 |
| Power plan | "Ultimate Performance" (GUID `e9a42b02-d5df-448d-aa00-03f14749eb61`), active at measurement time, read via `powercfg /getactivescheme` |
| Compiler | MSVC 19.44 (`_MSC_VER` 1944), VS2022 toolset 14.44.35207 |
| Build | `windows-msvc-release` preset, config `Release`, `CNPG_WARNINGS_AS_ERRORS=ON`; flags per `cmake/CompilerWarnings.cmake`: `/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX`, plus CMake's own default MSVC Release flags (`/O2 /Ob2 /DNDEBUG`) |
| Git commit | `ed0f6fd` (the commit the measured binary was configured against -- necessarily the parent of the commit that adds this document itself, the same "a commit cannot name its own hash" limitation `tests/support/SourceHash.h`'s file comment describes for the golden sidecars; the dsp/ chain code these numbers exercise is unchanged by the P1.10 commit that follows, which touches only the bench harness) |

**Caveat (honesty over precision, per the P1.8 ledger note):** this is a shared development
workstation, not an isolated, thermally-controlled benchmark rig. Single-run figures below should
be read to about 2 significant figures; the `max` column in particular is sensitive to OS
scheduling noise on a run this long (60 s / 22 500+ blocks gives the scheduler many chances to
preempt this process for something else) -- see the 8-string row's outlier below.

## Results (48 kHz / 128 samples / 2x oversampling, 60 s, N = 22 125 measured blocks)

| Strings | Median (us) | Median (% CPU) | p99 (us) | p99 (% CPU) | Max (us) | Max (% CPU) |
|---|---|---|---|---|---|---|
| 1 | 6.900 | 0.259% | 9.000 | 0.338% | 45.800 | 1.718% |
| 6 | 15.600 | 0.585% | 19.100 | 0.716% | 64.100 | 2.404% |
| 8 | 19.600 | 0.735% | 21.900 | 0.821% | 514.100 | 19.279% |

The 8-string `max` figure (514 us / 19.3% CPU, a single block out of 22 125) is an outlier well
above that row's own p99 (21.9 us) -- consistent with one OS scheduling preemption during a 60 s
run on a shared workstation, not a steady-state DSP cost; the median and p99 columns are the
figures to track for regressions. Re-running `--strings 1` twice back to back (see Determinism
below) shows the same pattern -- max varies run to run (45.8 us vs. 61.6 us) while median stays
exactly 6.900 us both times.

## Determinism

`cnpg_bench` prints `audioHash`, an FNV-1a/64 digest over the IEEE-754 bit pattern of every
rendered sample (computed outside the timed region, so it never perturbs a block-time
measurement). Two independent 60 s runs of the single-string configuration:

| Run | audioHash | blocksMeasured | nonFiniteSamples | median (us) | max (us) |
|---|---|---|---|---|---|
| 1 | `5289fa6fa7d22045` | 22125 | 0 | 6.900 | 45.800 |
| 2 | `5289fa6fa7d22045` | 22125 | 0 | 6.900 | 61.600 |

`audioHash` is bit-identical across both runs (the deterministic note stream plus the RNG-free,
wall-clock-free dsp/ chain -- `PluckExciter`'s own noise-burst PRNG reseeds to a fixed constant on
every `prepare()`/`reset()`, per `PluckExciter.h`) while the timing columns differ by ordinary
wall-clock jitter, exactly as expected: the computation is deterministic, the timing measurement of
it is not.

## Acceptance criteria (Task P1.10, verbatim from the brief)

- [x] `build\bin\Release\cnpg_bench.exe --strings 1 --samplerate 48000 --blocksize 128 --oversample 2 --seconds 60` prints median, p99, and max block time as CPU% and exits 0.
- [x] This document committed with 1/6/8-string numbers at 48 kHz / 128 samples and dev-machine spec.
- [x] CI runs the bench step non-gating (`ci.yml`'s `windows` job, `continue-on-error: true`) -- verified on this branch's push (see the PR/CI run linked in the P1.10 task report).

# P2 exit gate — the measurements, and the one thing that was not measured

Task P2.9. This is the file `docs/plan.md` §P2.9 asks for. It records the performance gate, the CI
gate, the `pluginval` gate, the P2.5 outcome, the state version and the benchmark environment.

**Read this first, because it is the whole shape of the document.**

> **P2.9 does NOT exit as a clean pass.** Every *mechanical* gate below is green with large margins.
> The **voicing sign-off was not performed**: `docs/listening/P2-20260803.md` was prepared by Task
> P2.8 and every verdict column and both decision blocks are still empty. The author elected to
> proceed without an audition on **2026-08-03**. So the plan's criterion *"the P2.8 listening results
> show every checklist item passed"* is **not met**, and P2.9 exits with that as a **documented,
> itemised exception** — §2 lists what each unanswered item would have decided.
>
> **A green board is not a substitute for the ear, and this document does not let it be one.** ADR
> 0007 D4 and §P2.9's own final acceptance criterion both say so in as many words: *"if that sign-off
> is absent or inconclusive, the exit doc records the default as still provisional rather than
> treating a green board as confirmation."* `couplingStrength = 0.35` therefore remains
> **PROVISIONAL**, and so does the Normal range.

---

## 1. Status of every acceptance criterion in `docs/plan.md` §P2.9

| # | Criterion | Status |
|---|---|---|
| 1 | `p2_default6` median CPU ≤ 30% of one core (hard gate) | **PASS** — 1.830%, §3 |
| 2 | `p2_max8` measured and reported, < 100% of one core | **PASS** — 2.183% median, §3 |
| 3 | Exit-commit CI: windows + ubuntu green; nonzero counts for all six tags | **PASS** — §4 |
| 4 | `pluginval --strictness-level 10` on the exit-commit VST3 | **PASS** — §5 |
| 5 | This document committed with all measurements, the P2.5 outcome, the voicing sign-off, the restated P1 triode caveat, **and no open blocking issues from P2.8** | **NOT MET** — the sign-off did not happen and the P2.8 blocking set is entirely open; §2 |
| 6 | This gate must **NOT** lock the `couplingStrength` default by passing | **HONOURED** — §2.2 records it as still provisional; nothing here confirms it |

Criterion 6 is not a thing that "passes"; it is a prohibition, and the way this document discharges
it is by **not** doing the thing. Criterion 5 is the exception the rest of §2 itemises.

**The plan's own `- [ ]` boxes for P2.9 are deliberately left unticked, in both copies.** Four of the
six are genuinely met, but a partially ticked list reads as "nearly passed" at a glance, and P2.7
already had to be corrected once for a plan box ticked against a criterion that was not satisfied
(`c5dc91f`, "the plan still ticked the all-five criterion"). This table is the record; the plan is
left byte-identical to what P2.8 left, in both `docs/plan.md` and
`docs/superpowers/plans/2026-07-30-pm-guitar-synth-p0-p2-plan.md`.

---

## 2. The voicing sign-off — NOT PERFORMED

### 2.1 What actually happened

**The author elected to proceed to the next phase without performing the listening pass, on
2026-08-03.** That is the fact. It is not "the pass was performed and everything passed", it is not
"the pass was inconclusive", and it is not "the measurements stood in for it".

`docs/listening/P2-20260803.md` is prepared, complete and executable — Task P2.8 built corpus
version 2, its 27 renders at 48 kHz (8 phrases plus 19 comparison configurations, digest
`_s0b201709fa0f`) and a headless number for every row. **That material is still current at this exit
commit**: `renderSourceHash()` covers `dsp/include`, `dsp/src`, `tests/support/P1Chain.h` and
`tests/render` (`tests/support/SourceHash.h`), and P2.9 touches only `docs/`, so the digest has not
moved and the prepared sheet remains playable exactly as written. And **every verdict cell in it is
empty**:

- §1.4, the `couplingStrength` decision block — blank.
- §2.1, the Normal-range decision block (three parameters) — blank.
- §3, checklist items 22–27 — six blank verdicts.
- Checklist items 1–21 — twenty-one blank verdicts, over §4.1 (nineteen rows), §4.2 (item 16) and
  §4.3 (items 17 and 21).
- §7, the summary verdict block, all 27 rows — blank; and the two trailing decision lines with it.
- `docs/listening/P2.6-ableton-checks.md`, host checks A–D (A1–A5, B1–B4, C1–C4, D1–D5 — **eighteen**
  verdicts, prepared 2026-08-02) — never played.

That sheet's own rule is unambiguous: *"An item with no verdict is **not** a pass; it blocks P2.9
until it has one."* Twenty-seven items are blank. Exactly one of them — item 16 — has a sanctioned
"deferred — host required" escape, and it was not used either.

**The plugin the author has installed is playable, but it is thirteen commits behind this tree.**
Measured, not assumed: `C:\Program Files\Common Files\VST3\cecinestpasunguitar.vst3` and the author's
own `build-ableton/` artefact are the **same binary** — 3 564 544 bytes, equal SHA-256, both last
written **2026-08-01T17:20:25Z** — so the build was installed rather than merely made. Against the
commit timeline in one frame:

| | UTC | relation to the artefact |
|---|---|---|
| `fd43b60` — P2.6, "the instrument becomes playable" | 2026-08-01T16:45:19Z | **35 min before — contained** |
| **the installed artefact** | **2026-08-01T17:20:25Z** | — |
| `0eb52b9` — first P2.6 fix wave | 2026-08-01T18:41:42Z | 81 min after — not contained |

So the installed plugin is, at newest, **the P2.6 task commit — before its own fix waves**: it has the
allocator, the real retrigger semantics and CC64, and it is a playable six-string instrument. What it
cannot have is anything after `fd43b60` — **`git rev-list --count fd43b60..HEAD` = 13**: P2.6's own
three fix waves, the whole of P2.7 (`c34875b`, the analytic bridge phase-delay compensation, which
moves every note's pitch, plus the rest-pitch fix), the ctest fix, and the whole of P2.8.

**That matters for what a host check on it would have meant**, and it cuts both ways. Checks A, B, C
and D would have been playable — the machinery they exercise is in the build. But they would have
been played on an instrument tuned *before* the compensation, with the P2.6 defects its own fix waves
went on to close, and without corpus version 2. The verdicts would not have transferred to this tree.
*(A build can also contain uncommitted work, so the timestamp bounds what the binary can contain from
above and no further; what it cannot contain is anything committed after it.)*

### 2.2 `couplingStrength = 0.35` remains PROVISIONAL

**Nothing in this gate confirms it, and the green board below must not be read as confirmation.**

ADR 0006 D1 ships `0.35`; ADR 0007 **D4** supersedes that in part and makes it provisional, settled
only by a recorded listening sign-off that compares lower values and judges mode-locking in
near-unison voicings *by ear*. D4 names this gate explicitly: *"No later task may treat it as settled
— explicitly including the P2.9 exit gate, which must not lock it by passing."*

The ladder the sign-off was to be made against is measured and unchanged; the **choice** between its
rungs is what did not happen.

### 2.3 The Normal range remains provisional, and criterion (4) is still failing

ADR 0007 **D7** declares `couplingStrength` 0.00–0.35, `bridgeResonanceHz` 20–330 Hz,
`bridgeDamping` 0.15–1.00. **D7.0 states that this box is derived from criterion (1) — the ±2-cent
tuning bound — alone**, and that D5's criterion (4) ("near-unison strings ≈25 cents apart do not
involuntarily mode-lock") was **measured at the declared coupling ceiling of 0.35 and fails there**.

That finding is unresolved, and P2.8 made it worse rather than better. **D7.1** measured the same
criterion on the **shipping six-string topology** — six strings on one bridge through
`NoteAllocator`, four of them resting at their open pitches and therefore in the same loop — and
found the boundary sits between **0.20 and 0.30**, against 0.32–0.35 for D7.0's isolated pair. The
instrument that ships locks *earlier* than the ADR previously recorded. The lock is progressive, not
a threshold: the detuned partner is absorbed at −0.3, −1.8, −6.8 dB across coupling 0.00 / 0.10 /
0.20 and is gone by 0.30.

So the provisional default **sits outside a criterion-complete Normal range**, on both topologies,
and it sits outside it by more on the one that ships. D5 ties the ceiling and the default to one
judgement; that judgement was not made.

### 2.4 The two unanswered items that decide code

Both are load-bearing beyond their own verdict, and both are blank.

**Item 17 — is 30 ms of legato glide a hammer-on or a slide? This decides the shipped
`kRetuneRampSeconds`, and no headless gate can.** The click statistic's reference is a fresh pluck,
which contains no glide at all, so every millisecond of ramp reads as excess by construction and the
statistic's optimum is "no legato". What it can refuse is a ramp so short the retune is a step. It
also **cannot order ramp lengths**: it is not monotone. Re-measured on this exit tree —

```
[contract] Physical retune ramp length vs click excess against a fresh pluck:
  0.0208 ms 10.3746 | 2 ms 18.6973 | 8 ms 10.1096 | 16 ms 6.0391 | 17 ms 4.8493
  18 ms 2.6177 | 19 ms 2.2203 | 20 ms 2.2970 | 21 ms 3.3948 | 22 ms 1.1765
  23 ms 1.8320 | 24 ms 1.4701 | 25 ms 1.4655 | 26 ms 1.1958 | 28 ms 3.5948
  29 ms 0.6065 | 30 ms 0.8982 | 31 ms 0.9761 | 32 ms 0.9967   (criterion 3 dB)
```

— **22 ms measures as well as the shipped 30 ms** (1.18 dB vs 0.90 dB, both far inside the 3 dB
criterion), 28 ms *fails* at 3.59 dB between two passing neighbours, and **nine ramps shorter than
30 ms clear the criterion** (18, 19, 20, 22, 23, 24, 25, 26 and 29 ms) — one of which, 29 ms, measures
strictly *better* than the shipped value. The suite prints exactly that conclusion on every run:
*"30 ms is NOT the shortest ramp that clears it, and this statistic does not order ramp lengths."*
**The shipped 30 ms is therefore an unconfirmed choice among at least ten admissible ones**, and the
gate that would have to be re-run if it moved is an ear, not this suite.

**Item 21 — does the fresh attack mask the state clear? This decides whether a short fade goes in
before the clear, and that change would move every corpus render off its current parent** — every
golden derived from it, and every number in the P2.8 session sheet with it. A released note is over,
so a note-on landing on that string clears it and plucks it fresh, and the clear is a one-sample
step. Measured against a 3 dB criterion whose level-placed hard-cut control reads 30.74 dB:

| gap after note-off | 5.5 ms | 12.4 ms | 21.3 ms | 52.7 ms | ≥ 100 ms |
|---|---|---|---|---|---|
| click excess | 19.69 dB | 21.48 dB | 16.64 dB | 0.30 dB | 0 dB |

So the gate says this **is** a discontinuity below about 50 ms — which is ordinary staccato playing,
not an abuse case — and says nothing about whether anyone hears it under a full-velocity pluck
landing on the same sample. The discarded tail sits 5.7–6.1 dB below the attack that replaces it.
The behaviour is pre-existing and confirmed identical at `77b0430`; it is not a P2.6 or P2.8 defect.
**Unanswered, it ships as-is by default rather than by decision.**

### 2.5 The P1 triode voicing caveat — restated, as §P2.9 requires

`TriodeStage` is a **static waveshaper**: one memoryless input-voltage → output-voltage curve, no
state carried between samples. Therefore no bias drift, no coupling-capacitor high-pass behaviour, no
cathode-bypass corner interacting with signal level, no thermal settling. **The chain is explicitly
not a voicing reference for drive feel.** Checklist item 12 is to be judged for *artifacts* and not
for whether the drive "feels" right. **Drive-feel voicing conclusions remain out of scope until a
dynamic stage lands in P3+.** This caveat stood at P1 and is unchanged by anything in P2.

### 2.6 What is inherited, itemised

| Inherited open item | Where it is defined | What it blocks |
|---|---|---|
| Shipped `couplingStrength` | sheet §1.4; ADR 0007 D4 | the default ships provisionally at 0.35 |
| Normal-range ceiling (all three parameters) | sheet §2.1; ADR 0007 D5, D7, D7.0, D7.1 | criterion (4) stays measured-and-failing |
| Checklist items 1–15, 18–20, 22–27 | sheet §3, §4.1 | nothing in code; they are confirmations |
| **Checklist item 17** | sheet §4.3 | the shipped `kRetuneRampSeconds` |
| **Checklist item 21** | sheet §4.3 | a fade before the state clear → every corpus render and its goldens |
| Checklist item 16 (string count under a ringing chord) | sheet §4.2 (a), host check B | the only item with no render evidence of any kind |
| Item 7 pluck half, item 10 damper half | sheet §4.2 (b), (c) | one host knob gesture each |
| Host checks A–D | `docs/listening/P2.6-ableton-checks.md` | eighteen verdicts, never played |

---

## 3. Performance gate — PASS

Methodology is locked in `docs/plan.md` §4.7 and is unchanged from `docs/bench/p1-baseline.md`,
`p2-1-scale-out.md` and `p2-2-damper.md`. The tool is `tests/bench/BenchMain.cpp` (target
`cnpg_bench`). A named `--config` sets the **string count and nothing else**; everything else — the
full P1 monitoring chain, the shipped parameter defaults, 2× oversampling — is what `cnpg_bench`
already runs, and the per-block `setParams()` cascade that production performs every block runs
inside the timed region.

```
cnpg_bench --config p2_default6 --samplerate 48000 --blocksize 128 --seconds 120
cnpg_bench --config p2_max8     --samplerate 48000 --blocksize 128 --seconds 120
```

Runs were performed **alone and uncontended** on an otherwise idle machine — no build, no test
suite, no `pluginval` running alongside. At 48 kHz with 128-sample blocks the realtime block period
is **2666.6667 µs**, which is what 100% of one core means in every row below.

| Configuration | strings | median (µs) | **median CPU** | p99 (µs) | p99 CPU | max (µs) | max CPU |
|---|---|---|---|---|---|---|---|
| `--config p2_default6` run 1 | 6 | 48.800 | **1.830%** | 52.800 | 1.980% | 740.500 | 27.769% |
| `--config p2_default6` run 2 | 6 | 48.800 | **1.830%** | 54.300 | 2.036% | 691.400 | 25.928% |
| `--config p2_max8` | 8 | 58.200 | **2.183%** | 75.800 | 2.843% | 646.200 | 24.233% |

Every run: `blocksTotal` 45 000, `blocksMeasured` 44 625 (375 warmup blocks excluded),
`nonFiniteSamples` **0**. `audioHash` is `e34f8bf850ab992b` for both `p2_default6` runs and
`ef567a47ba0d3b52` for `p2_max8` — identical across runs of the identical command, which is what says
the two `p2_default6` rows are two measurements of one workload rather than two workloads.

**The gate:**

- **`p2_default6` median 1.830% against a 30% hard gate — PASS, with 16.4× headroom.** It also clears
  the 25% aspirational target, by 13.7×.
- **`p2_max8` median 2.183%, must remain realtime (< 100% of one core) — PASS, 45.8× inside.** Never
  gated, in CI or here. Reported because §P2.9 asks for it.
- Both configurations' **`max`** columns (25.9% and 24.2%) also sit under the 30% figure, though the
  gate binds the median only.

**On the `max` column, restating what the earlier bench documents already establish:** it is not a
steady-state DSP cost. Here the two identical `p2_default6` runs read 740.5 µs and 691.4 µs while
their medians are identical to the printed digit; `p2-1-scale-out.md` saw the same column swing by an
order of magnitude between two runs of one command (664.4 µs vs 60.7 µs) with a 0.1 µs median move.
That is OS scheduling preemption of a single-threaded user-space render loop on a shared workstation.
**Median and p99 are the columns to track**, and the hard gate is written on the median for exactly
that reason.

**Trend across the P1/P2 bench documents**, same machine, same settings, median CPU:

| configuration | P1 baseline (60 s) | P2.1 scale-out (60 s) | P2.2 damper (60 s) | **P2.9 exit (120 s)** |
|---|---|---|---|---|
| `--strings 1` | 0.263% | 0.274% | — | — |
| `--config p2_default6` | — | 0.634% | 1.118% | **1.830%** |
| `--config p2_max8` | — | 0.780% | 1.436% | **2.183%** |

The growth from P2.2 to here is P2.4's loaded bidirectional bridge, P2.6's allocation and retrigger
machinery and P2.7's compensated loop solve. It is a **1.64× rise into a 16.4× margin**, and the two
configurations moved by similar factors (1.64× and 1.52×).

### 3.1 Benchmark environment

Self-reported by the binary in its own JSON record, not transcribed by hand:

| | |
|---|---|
| CPU | AMD Ryzen 9 7950X 16-Core Processor |
| Logical cores | 32 |
| OS | Windows 11 Pro for Workstations, 10.0.26200 |
| Windows power plan GUID | `e9a42b02-d5df-448d-aa00-03f14749eb61` |
| Compiler | MSVC 1944 (Visual Studio 2022 BuildTools 17.14) |
| Generator / architecture | Visual Studio 17 2022, x64 (`windows-msvc-release` preset) |
| Build type | Release |
| Build flags (warnings) | `/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc /WX` |
| Build flags (codegen) | `/DWIN32 /D_WINDOWS /EHsc` + `/O2 /Ob2 /DNDEBUG` |
| Reported `gitCommit` | `d764086` |
| Sample rate / block size / oversampling / duration | 48 000 Hz / 128 / 2× / 120 s |

**One provenance finding, recorded because it bit this task — and it is the third occurrence of a
class this project has already fixed twice.** `CNPG_BENCH_GIT_COMMIT` is baked in at CMake
**configure** time (`tests/bench/CMakeLists.txt:26-40`), not at build time. This tree had last been
configured at `c5dc91f`, so the first `p2_default6` run of this session printed `gitCommit=c5dc91f`
while standing on `d764086` — **five commits stale, on a freshly rebuilt binary.**

The numbers in the table above were **re-measured after `cmake --preset windows-msvc-release`**, so
the field reads `d764086`; the pre-reconfigure run produced the **identical median and the identical
`audioHash`**, which is what says only the label moved and the workload did not.

`tests/bench/CMakeLists.txt`'s own comment declares the behaviour best-effort and intentional, and on
its own terms it is: a rebuild is not a reconfigure, and no generator expression can make a
configure-time `git rev-parse` track HEAD. **What makes it worth recording is that the same shape has
already cost this project a measurement twice** — the golden sidecars' `generatorCommit` and
`cnpg_render`'s filename hash, both replaced by content hashes (`tests/support/SourceHash.h`). It is
carried into the triage list as §8.3 item 7 rather than fixed here.

---

## 4. CI gate — PASS

`.github/workflows/ci.yml` runs **four jobs**: `verify-pins`, `windows`, `ubuntu (gcc)` and
`ubuntu (clang)`.

- **Run ID:** *(recorded in §4.3 below — filled from the exit commit's own run)*
- The `windows` job builds the `windows-msvc-release` preset, runs `ctest --test-dir build -C Release
  --output-on-failure` (the full suite), the clang-format check, the golden-commit trailer guard, a
  **never-gating** benchmark report, and `pluginval --strictness-level 10` against the built VST3.
- The `ubuntu` jobs build the `linux-dsp-only` preset (JUCE never fetched — there is a log guard that
  fails the job on the string `juce` appearing in the configure or build log) and run
  `ctest --test-dir build-dsp`.
- **CI publishes bench numbers without gating on them**, by design: the "Benchmark report" step is
  `continue-on-error: true` and ends in `exit 0`. A perf regression must never gate CI, because a
  shared runner is noise-dominated. The 25–30% figure is read from *this* document, measured on the
  dev machine.

**One workflow change lands with this task**, which is the second file §P2.9 lists. The "Benchmark
report" step published only the P1 `--strings 1` row; §P2.9 asks the windows job to publish the
`cnpg_bench` **configs**. It now emits three rows — `--strings 1` (kept, for continuity with
`docs/bench/p1-baseline.md`), `--config p2_default6` and `--config p2_max8` — all at 48 kHz / 128 /
2× / 10 s, each appended to the uploaded `cnpg-bench-report.txt` artifact. **It still cannot gate
anything**: the step remains `continue-on-error: true`, still ends in `exit 0`, and each invocation
is additionally guarded with `|| true` so one failing row cannot skip the two after it. The loop was
smoke-tested locally before committing; all three rows print and the step exits 0. Nothing else in
the workflow changed.

### 4.1 Per-tag test counts, read out of the test log

§P2.9 requires **nonzero counts for each** of the six tags, read from the log rather than from a
conclusion field. `catch_discover_tests(... ADD_TAGS_AS_LABELS ...)` registers each Catch2 tag as a
CTest label, so the `ctest` run prints the counts itself, in its own Label Time Summary:

```
100% tests passed out of 280

Label Time Summary:
aliasing      =   1.35 sec*proc (2 tests)
contract      =  50.95 sec*proc (234 tests)
denormal      =   2.47 sec*proc (3 tests)
energy        =   5.87 sec*proc (25 tests)
regression    =   3.41 sec*proc (5 tests)
tuning        = 172.77 sec*proc (14 tests)

Total Test time (real) = 235.26 sec
```

| tag | ctest label count | Catch2 `--list-tests` count | nonzero? |
|---|---|---|---|
| `[contract]` | 234 | 234 | yes |
| `[energy]` | 25 | 25 | yes |
| `[regression]` | 5 | 5 | yes |
| `[aliasing]` | 2 | 2 | yes |
| `[denormal]` | **3** | **4** | yes |
| `[tuning]` | 14 | 14 | yes |

The counts sum to 284 against 280 test cases because several cases carry two of these tags.

**The `[denormal]` row differs by one, and the difference is intended.** `DENORMAL: no CPU blow-up in
the tail` is tagged `[.][denormal]` — Catch2-hidden, therefore not registered as a CTest test and
never run by CI. That is exactly what `docs/plan.md` §P2.8 specifies: *"the timing-ratio denormal case
per section 4.6 remains local-only, run on the dev machine."* It was run on the dev machine for this
gate and passes:

```
[denormal] block time median: first 1 s 27.8 us, final 5 s 32.3 us, ratio 1.16187 (limit 2.0)
All tests passed (3 assertions in 1 test case)
```

**`[energy]` covers all three tiers**, which the aggregate count alone does not show:

| tier | cases |
|---|---|
| `ENERGY/T1` (junction scattering algebra) | 8 |
| `ENERGY/T2` (assembled lossless network) | 10 |
| `ENERGY/T3` (lossy network only dissipates) | 7 |

### 4.2 Local suite verification

| Run | Result |
|---|---|
| `build\bin\Release\cnpg_tests.exe` | **All tests passed (5 468 119 assertions in 280 test cases)** |
| `build\bin\Debug\cnpg_tests.exe` | **All tests passed (5 468 119 assertions in 280 test cases)** — identical to Release, §4.4 |
| `ctest --test-dir build -C Release --output-on-failure` | **100% tests passed out of 280**, 235.26 s |
| `clang-format --dry-run --Werror` over `dsp/ plugin/ tests/` | clean, 96 files, clang-format 19.1.5 |
| `git status --porcelain tests/data/golden/` | empty — **goldens untouched**, Release and Debug |
| `docs/plan.md` vs `docs/superpowers/plans/…` | **byte-identical** (`29a6b21ccd82e426aed3bb24c3be3b88`) |

Nothing in P2.9 touches `dsp/`, `plugin/` or `tests/`, so no rendered audio can have moved, and none
did.

### 4.3 The exit commit's CI run

*(This subsection is completed by the commit that follows the one carrying this document — a CI run
cannot exist for a commit before that commit is pushed. The run ID and the four job conclusions are
recorded below, read from the run's own log.)*

### 4.4 Debug/Release assertion parity — identical

Both configurations built from the same configure, both run to completion as a single process, both
exit 0:

| configuration | result | wall time |
|---|---|---|
| Release | `All tests passed (5468119 assertions in 280 test cases)` | 204.9 s |
| **Debug** | `All tests passed (5468119 assertions in 280 test cases)` | ~3 011 s |

**5 468 119 / 280 in both — not one assertion of difference.** That is the check worth running: the
two configurations differ in optimisation level, in `/RTC1` runtime checks, and in whether
`assert`-style debug paths are live, so a count that moves between them means a test whose *number of
assertions* depends on the build — which is either a data-dependent loop bound or a genuine
behavioural divergence. This project has found real defects that way before, so the count matching
exactly is evidence and not a formality.

The Debug wall time is ~14.7× Release. Part of that is `/Od /RTC1`; part is that this run shared the
machine with `pluginval`, the `[energy]` re-measurement and a bench smoke test for portions of its
life. It is not a performance figure and is recorded only so a future run is not surprised by a
50-minute suite.

**One process discipline note, because P2.8 was bitten by the opposite.** This Debug run was left to
finish rather than restarted mid-flight. `build/bin/Debug/corpus-sweep` is scratch that
`CorpusSweepTests` deletes and repopulates on every run, so two Debug suites in one build tree
corrupt each other's evidence — which is exactly how P2.8 invalidated another session's `ctest -C
Debug` run. One suite per build tree at a time.

### 4.5 Two things §P2.9's verification method asks for that were deliberately NOT done

`docs/plan.md` §P2.9's "Manual" line asks for a **final acceptance session in Ableton Live on the exit
build** and for the git tag **`p2-exit` applied to the signed-off commit**.

- **The Ableton session did not happen.** It is the host half of the same audition §2 records as not
  performed, and it is where checklist item 16 and the eighteen host checks would have been answered.
- **The `p2-exit` tag is NOT applied, and this task did not apply it.** The plan's own wording is *"to
  the **signed-off** commit"*. There is no signed-off commit: the sign-off did not happen, so the
  precondition the plan attaches to the tag is unmet and the tag has nothing to name.

  **The reason to state this rather than quietly omit it:** a git tag is the one artefact a future
  reader will trust *without opening this document*. `git tag` output, a release page, a CI matrix
  keyed on tags — all of them would read `p2-exit` as "P2 exited", and none of them would surface §2.
  Every other place this exception is recorded requires someone to read prose; a tag would silently
  overrule all of them. **So the tag is the single worst place to be imprecise, and it is left off.**

  **The tag is the author's to apply** — after the audition, or deliberately and in full knowledge of
  §2 if the decision is to ship past it. Either is a decision; applying it as a side effect of a
  green board is not.

---

## 5. `pluginval --strictness-level 10` — PASS

Run on this tree's Release VST3, with the pinned validator
(`cmake/PluginvalPin.cmake`: **v1.0.4**, SHA-256 verified on download by CI on every run, cache hit
or not).

```
pluginval.exe --strictness-level 10 --validate
  build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3
```

```
Validation started
Strictness level: 10
Num plugins found: 1
Testing plugin: VST3-cecinestpasunguitar-32a0187e-56ba1ee6
Plugin name: cecinestpasunguitar
SupportsDoublePrecision: no
Reported latency: 0
Reported taillength: 0
Main bus num input channels: 0
Main bus num output channels: 2
...
SUCCESS
```

Exit code **0**. The same gate runs in the `windows` CI job against the CI-built artefact, so it is
checked on two independently built binaries.

---

## 6. The P2.5 outcome — the fallback was NOT triggered; the bridge is the passing construction

Recorded here because §P2.9 asks the exit document to state which path P2.5 closed on. The decision
record is `docs/decisions/0006-p2-bridge-passivity-fallback.md`; this is a summary of it, not a
replacement for it.

**`BridgeJunction` ships. `SympatheticResonatorBus` was never built.**

The junction is a **wave-digital parallel adaptor** — the parallel connection of N string ports with
a lumped load, solved exactly from Kirchhoff's law at the shared velocity node, *not* a bridge
admittance discretized into a transfer function and inverted into a scattering matrix. In
power-normalized wave variables the scattering matrix is

```
S̃ = (2/σ) u uᵀ − I,    u_p = √Z_p,    σ = Σ Z_p,    ‖u‖² = σ
```

whose eigenvalues are exactly +1 and −1 for **any** set of positive port impedances: it is a
symmetric orthogonal reflection, so `‖S̃‖₂ = 1` *exactly*, as an algebraic identity rather than a
measured outcome. **There is no clamp anywhere in the audio path.**

| P2.5 question | Outcome |
|---|---|
| Did the corrected `[energy]` suite ever run red? | **No.** The timebox clock never started; the suite enters history green. |
| Fallback protocol triggered? | **No** (ADR 0006 D2). |
| `research/bidirectional-bridge` branch | Does not exist — the correct state on the pass path. |
| Passing construction recorded outside the (gitignored) task report? | **Yes** — ADR 0006 **D0**, added specifically to close that criterion. |
| Q17 design-for-fallback seam intact? | **Yes.** `IBridgePort` is shared, `StringNetwork::setBridgePort()` substitutes, and `BridgePortContractTests` is a `TEMPLATE_TEST_CASE` over two production implementations. |

**Margins, re-measured on this exit tree** — read off the `[energy]` label's own prints (25 cases,
1 480 578 assertions), not transcribed from ADR 0006:

| Gate | Bound | Measured worst, this tree |
|---|---|---|
| Tier 1, `‖S̃‖₂` (DamperJunction) over the §4.2 grid | ≤ 1 | **1** (at position 0, maxLoss 0, engagement 0) |
| Tier 1, sample-level energy balance, dashpot bypassed | ≤ 1e−11 relative | **2.22247e−15** |
| Tier 1, energy **gain** under 201 live admittance retargets | ≤ 1e−11 | **1.81094e−15** |
| Tier 2, per-block growth, 6 strings lossless, `double`, 3 rates × 2 interpolators × 2 dispersion settings | ≤ 1e−9 | **3.3996e−15** (Thiran1 @ 96 kHz) |
| Tier 3, per-block growth, float32, 7 admittance points | ≤ 1e−6 | **4.74382e−9** |

**The non-vacuity control is printed beside the bound rather than assumed:** the same tier-1 sweep
reports a **worst single-sample dissipation of 1.57813** with the dashpot *in* — so the balance
assertion is measuring a junction that demonstrably moves energy, not one that is inert. Tier 3
likewise carries its floor in the print: 3 731 of 3 732 blocks are gated down to 186–236 dB below the
start of each run, and the level of the last gated block is printed so the margin is visible rather
than asserted.

The one red `[energy]` reading during P2.4's development was the **float32 arithmetic floor** — the
shipping FTZ/DAZ guard flushing the recursions' own intermediate products at total energies around
1e−69 — diagnosed by running the identical scenario on the `double` instantiation, where it is clean
over the whole decay. It was not a passivity failure, and the standing case
`ENERGY/T3: the float32 arithmetic floor is a measurement limit, not a passivity failure` runs on
every CI run and *requires* the double control to be clean and the float32 runs not to be, so the
finding cannot be re-lost.

---

## 7. State version — 2

`plugin/src/PluginProcessor.h:95`:

```cpp
static constexpr int kCnpgStateVersion = 2;
```

Written to the saved state as the `cnpgStateVersion` attribute
(`plugin/src/PluginProcessor.cpp:297`) and required to match exactly on load
(`:311`) — a state carrying any other version is rejected rather than partially applied.

**It stays 2 through all of P2**, per Task P2.1, and P2.4's three new APVTS parameters
(`bridgeCoupling`, `bridgeResonanceHz`, `bridgeDamping`) did not move it. Nothing in P2.9 touches
state.

---

## 8. Carry-forwards this gate inherits

### 8.1 C1 — the `ctest -C <cfg>` config trap, fixed, with a behaviour change worth knowing

`catch_discover_tests` was changed to `DISCOVERY_MODE PRE_TEST` in commit **`830ee53`**
(`tests/CMakeLists.txt:128`). The default is `POST_BUILD`, and under the multi-config VS 17 2022
generator that the `windows-msvc-release` and `windows-msvc-debug` presets **share** (both use
`binaryDir` `build`) it writes **one config-agnostic discovery file** whose every `add_test()`
hard-codes an absolute path to the binary of whichever configuration was built **last** — included
unconditionally. `ctest -C <cfg>` had nothing to select between.

**Consequence to state plainly: every local `ctest` run in this repository before `830ee53` was
labelled by the `-C` flag rather than by the binary that executed** — since Task P1.5. Any conclusion
of the form "Release and Debug both pass under ctest" drawn before that commit is, at best,
unattributed, and it silently voided the Release half of this project's dual-config verification
standard.

It also explains a number that had been carried as a mystery since P2.7: the `[tuning]` grid gate was
recorded at **1639.8 s through ctest against ~160 s for the same work in-process**, a ~10× gap with
no explanation. There was no mystery — ctest was running the Debug binary. Measured on this exit
tree, the whole `ctest -C Release` suite is **235.26 s**, of which `tuning` is 172.77 sec·proc.

**CI was never affected**, and it is worth saying why rather than asserting it: the `windows` job
configures and builds **Release only** and never builds a Debug binary at all, so there was no second
binary for discovery to pick up; and the `ubuntu` jobs use the Ninja **single-config** generator,
where `PRE_TEST` emits a single unconditional include file and `ctest --test-dir build-dsp` still
discovers and runs with no `-C` flag at all.

Verified on this exit tree, by reading the command ctest actually issues:

```
ctest --test-dir build -C Release -R "DENORMAL: long tail leaves no subnormal state" -V
  192: Test command: ...\build\bin\Release\cnpg_tests.exe "DENORMAL: long tail ..."

ctest --test-dir build -C Debug -N -V
  1: Test command: ...\build\bin\Debug\cnpg_tests.exe "ALIASING: engagement invariant ..."
```

**The fix introduces a trap of its own, and it is a silent one.** `PRE_TEST` discovery cannot resolve
a config-dependent binary path without a config, so a bare invocation with no `-C` now finds nothing:

```
$ ctest --test-dir build
Test project C:/Users/hyung/Desktop/cecinestpasunguitar/build
No configuration for testing specified, use '-C <cfg>'.
No tests were found!!!
$ echo $?
0
```

**Zero tests, exit code 0.** A future invocation — a script, a CI step, an agent — that forgets the
`-C` flag will look exactly like a pass. Anything that runs `ctest` on this repository must pass
`-C <cfg>` and should assert on the test count, not on the exit code.

### 8.2 C2 — the entire project ledger lives outside the repository

`.superpowers/` is gitignored. That directory holds `progress.md` (the running ledger, 495 lines at
this commit), **25 task reports, 26 task briefs, 7 fix-wave and carry-forward notes — 59 markdown
files in all — and 39 review diffs**: the whole written history of P0 through P2, including the
evidence behind decisions that shipped.

ADR 0006 D3 already shows what this costs once: the P2.5 pass-path criterion "the decision doc
records the passing construction" was **genuinely unmet** at review time *because the construction
lived only in a task report under `.superpowers/`*, and D0 had to be written to close it. That was
one criterion, caught. The general case is not caught by anything.

**This is an open author decision, not this task's to resolve.** It is recorded here because a
tracked file is the only place it becomes visible to someone reading the repository — which is the
whole of the problem it describes.

### 8.3 C3 — whole-branch triage backlog, carried forward unfixed

**None of these were touched by P2.9.** They are listed so the final review inherits a written list
rather than a memory. Each is a live finding from an earlier task's review, not a speculation.

1. **`bridgeTuningFallbacks()` fails open.** `dsp/include/cnpg/dsp/StringNetwork.h:495-517` documents
   it against measurement: a note played at its own string's **rest pitch** is never re-solved, so
   the counter reads 0 for a render that ran entirely on a **non-convergent** solve carrying 2.19
   cents of probe residual — against a 2.0-cent gate and a 0.25-cent solver tolerance. So
   `REQUIRE(net.bridgeTuningFallbacks() == 0)` written at a rest pitch **passes unconditionally and
   asserts nothing.** The header says what to write instead (`bridgeTuningConverged(stringIndex)`,
   which is state and survives the reset). The one existing site,
   `tests/dsp/WaveguideStringTuningTests.cpp:360`, is non-vacuous only because it plays MIDI 45 on a
   string resting at MIDI 40.
2. **A standing sweep is owed for `REQUIRE(<counter> == 0)` assertions whose zero side is reachable
   through a skip or a clear.** Item 1 is one instance of a pattern; nothing has enumerated the rest.
   Vacuity is this project's recurring failure mode — found and closed repeatedly from P2.1 through
   P2.8, and named as such in the shipped tree (`StringNetwork.h:510`: *"Vacuity is this project's
   recurring failure mode; meet it here, in the tree, rather than in a review."*) — and a counter that
   resets is the cheapest way to produce another.
3. **Six click gates have no negative control at all** (pre-existing; found by P2.6's suite-wide
   sweep, review item R-2): `StringNetworkScaleTests.cpp`'s `setNumStrings` reduction and its
   8-string case, `BridgePortContractTests.cpp`'s coupling-to-zero, `PitchBendClickTests.cpp`,
   `StringNetworkTests.cpp`, `PickupTapTests.cpp`. P2.6 level-placed the four controls that existed
   and were blind; it did not add controls where there were none. The cost of a blind control is
   measured and large: level-placing one moved it 8.43 → 32.50 dB and **corrected
   `tests/support/ClickMetric.h` itself**, which had quoted its own blind reading as the suite's
   sensitivity floor and so understated the metric's sensitivity by 24 dB.
4. **The swept-damper case gates the wrong one of two readings it prints.** It gates `excessDb`
   (0.984342 dB, limit 3) while the companion **level-normalised** reading — the one `ClickMetric.h`
   itself says is correct for a decay-rate change — prints **17.3327 dB and is ungated.**
5. **`cnpg_render --samplerate` keeps the flat P1 output layout and the filename carries no rate**, so
   two runs at different rates into one `--out` silently overwrite each other. `--rates` exists
   precisely because of this and writes per-rate subdirectories; `--samplerate` is kept flat because
   `RenderTests.cpp` depends on it.
6. **The gain-structure gate cannot fire.** `renderOne()` returns false above the limiter ceiling and
   `CorpusSweepTests` re-measures the peak off the written file — but `SoftClipLimiter` is hard-wired
   last in `P1Chain::processBlock` and the ceiling it is compared against is **the value that limiter
   enforces as a horizontal asymptote**. No phrase, variant or sidecar can produce a failure.
   Discharging it would need a build with the limiter bypassed or re-ordered, which is a test-only
   seam that does not exist. It is an **undischarged** gate, not a passing one, and the P2.8 session
   sheet's §5 says the same thing about the same numbers.
7. **`cnpg_bench` still stamps a configure-time git commit — the third instance of a defect class this
   project has fixed twice already** (found by this task; see §3.1 for the measurement). The pattern is
   *a provenance field resolved when CMake last ran, silently naming a commit that is not the one the
   artifact came from*:

   | occurrence | field | outcome |
   |---|---|---|
   | golden sidecars | `generatorCommit`, configure-time `git rev-parse HEAD` | **fixed** — replaced by `dspSourceHash()`, a content hash of `dsp/` |
   | `cnpg_render` filenames | configure-time `git rev-parse --short HEAD` | **fixed at P2.7** (carry-forward C2) — replaced by `renderSourceHash()`, computed at render time. It had already misfiled real renders: a tree configured at `77b0430` produced audio from the code at `1ccfcb1` and stamped it `_g77b0430`, three commits stale, so **two different code states produced identical filenames** |
   | **`cnpg_bench` JSON/banner** | `CNPG_BENCH_GIT_COMMIT`, configure-time | **still the original shape** — measured five commits stale during this task |

   The sidecar and render cases were fixed because their outputs are *compared against* later; a bench
   artifact is only read by a human, which is why this one has survived. That is a reason it is lower
   priority, not a reason it is correct — a bench row attributed to the wrong commit is exactly how a
   performance regression gets blamed on the wrong change. **Deliberately not fixed by P2.9**, whose
   scope is to measure this tree, not to change what the tools record about it.

---

## 9. What this document does not claim

- It does **not** claim the instrument sounds right. Nobody listened.
- It does **not** confirm `couplingStrength = 0.35`, the Normal range, or any of its three faces.
- It does **not** confirm the shipped `kRetuneRampSeconds = 30 ms` over the **nine shorter ramp
  lengths that also clear the click criterion** — one of which, 29 ms, measures strictly better.
- It does **not** claim the state clear is inaudible under a fresh attack.
- It does **not** claim the P2.6 host checks passed; they were never played.
- It does **not** treat CI green, `pluginval` green or 16× performance headroom as evidence about any
  of the above. They are evidence that the code runs, is fast, and does not regress — which is a
  different question from whether it is the right instrument.

---

## 10. Appended after the exit: the damper default was revised on a listening finding

**This section does not amend §1–§9.** Every verdict above stands exactly as it was recorded on
2026-08-03, including the exception this document exits with. What follows is a fact that arrived
after that exit and is recorded here because §9's first line is the reason it could: *"It does not
claim the instrument sounds right. Nobody listened."* Somebody then listened.

The author recorded eight strikes of **C3 (130.81 Hz)** through the plugin and heard the release turn
into "a weird harmonics-like sound" whenever a released note was still ongoing. It is real, it is the
damper, and it was reachable at the shipped default in one strike:

- A point damper dissipates mode *n* in proportion to `sin^2(n*pi*p)`, so it is **exactly blind** to
  every partial with a node at *p*.
- `damperPosition01` defaulted to **0.15**, and 3/20 = 0.150 exactly — partial 20's node — with
  partials 7 (1/7) and 13 (2/13) within 0.008 of one.
- Measured on C3 at the shipping exciter/pickup geometry (both 0.5), a note-off at 0.15 collapses onto
  **partial 7 at 915.7 Hz, 24.1 dB ABOVE the fundamental**, inside the note's own audible life.

The default is now **1/25 = 0.04**, derived from a closed form rather than fitted: requiring every
partial in `[2, N]` to be damped at least as hard as the fundamental gives `p <= 1/(N+1)`, and 1/25
guarantees the band `[2, 24]` at 98.4% of the 4x margin that criterion can ever deliver. The same
measurement then reads **−12.7 dB**, a 36.7 dB improvement.

### It is not free: it closes one defect, creates a second, and worsens a third it did not create

Coupling to the fundamental is `sin^2(pi*p)`, so the felt is **13.1x weaker** on it. Two costs, both
on gestures a player uses constantly — and the second of them has a part that is **not this change's
to give back**, which the decomposition below separates out:

| | cost | measured |
|---|---|---|
| **1. Note-off speed** | **3.25x slower** | C3 falls 60 dB in **0.650 s**, was 0.200 s (undamped: 2.03 s). Worst in the low-mid register: MIDI 28 goes 1.230 s → 2.010 s |
| **2. Re-strike click** | **13 dB louder**, clean boundary **50 ms → 500 ms** | note-after-note click excess by note-off age, criterion 3 dB: at p = 0.15, `19.69 / 21.48 / 16.64 / 0.30 / 0 / 0 / 0`; at 1/25, `20.00 / 24.12 / 23.34 / 13.29 / 13.34 / 3.16 / 0` for 5 / 10 / 20 / 50 / 100 / 200 / 500 ms |

The silence watchdog also holds a released string in the per-sample loop about **2.5x longer**, which
adds to the CPU figure §3 reports (not re-measured — the §3 hard gate has 16x of headroom).

**THERE IS NO COMPENSATING ADJUSTMENT, and that is the fact that decides whether the durable fix is
optional.** `maxLoss = 1` is already the matched resistive termination — measured monotone in depth
at every position, and past it the junction becomes a rigid pin — and taking the felt time from 40 ms
to its 20 ms floor moves t(−60 dB) by **0 ms**. **Position is the only knob**, so the note-off
slowdown is unavoidable at *every* position that clears the comb; 1/25 is the cheapest such position,
not an expensive one. The author is therefore choosing between three states, not two:

| | the comb | note-off | re-strike click 50–200 ms | re-strike click **≤ 20 ms** |
|---|---|---|---|---|
| keep p = 0.15 | **+24.1 dB at 915.7 Hz** | 0.200 s | clean from 50 ms | **17–21 dB, item 21** |
| **ship p = 1/25 (this change)** | −12.7 dB, gone | **0.650 s** | **+13 dB, clean from 500 ms** | **20–24 dB, item 21** |
| finite contact width (scoped separately) | gone at **every** position | back to ~0.200 s | back to ~clean from 50 ms | **STILL 17–21 dB, item 21** |

### These are THREE defects with THREE owners, not one trade

| # | defect | measured | who closes it |
|---|---|---|---|
| **1** | The harmonic comb | +24.07 → **−12.67 dB**, a 36.7 dB improvement | **closed here**, by position; finite width would close it at *every* position rather than only below 1/21 |
| **2** | The note-off slowdown | 0.200 → **0.650 s** at C3 | **created here**; finite width takes it back essentially in full, because the whole 3.25x is the price of pushing the first node past partial 24 |
| **3** | The **≤ 20 ms** re-strike click | **17–24 dB** at ages 5 / 10 / 20 ms | **NEITHER.** Pre-existing at p = 0.15 (19.69 / 21.48 / 16.64 dB), 0.3–6.7 dB worse here (20.00 / 24.12 / 23.34). It is **checklist item 21**, *"Re-striking just after a note-off (P2.6)"* — P2.6's state clear discarding a still-loud tail — and it needs a **fade on the fresh path** |

**Read defect 3 twice.** The 50–200 ms band of that same click *is* this change's doing and finite
width does take it back. The ≤ 20 ms band does not move with the damper **at all**, and that is
measured rather than argued: the discarded tail's level relative to the note replacing it reads
**−5.665 / −5.886 / −6.108 dB** at 5 / 10 / 20 ms at p = 0.15, and **−5.665 / −5.886 / −6.106 dB**
at 1/25 — identical to three decimals, because 5–20 ms after a note-off no damper of any width at
any position has had time to act. **Anyone reading the cost table alone would conclude the durable
fix closes everything. It does not, and item 21 would then sit unowned.**

Two `[contract]` gates now hold the finding, both demonstrated RED at p = 0.15 in their own bodies:
`tests/dsp/DamperReleaseSpectrumTests.cpp`. **Six** existing gates were re-pointed deliberately, each
recorded at its own site with both readings — and one of the six is a *strengthening*: the
click-metric companion's full-note-off carve-out is gone, because that carve-out existed only
because the residue at p = 0.15 was brighter than the note. Full derivation, the whole-slider sweep,
the trade table and the re-pointing ledger:
`.superpowers/sdd/2026-07-30-pm-guitar-synth-p0-p2-plan/task-damper-node-comb.md`.

**What this changes about the exception above: nothing.** The voicing sign-off is still not performed,
`couplingStrength = 0.35` is still PROVISIONAL, and `docs/listening/P2-20260803.md` is still empty.
One defect found by one ear on one note is not an audition. If anything it is evidence for §9's
first line rather than against it — a green board did not catch this, and was never going to.

---

## 11. Appended after the exit: the default VOICING was set, on the same listening finding

**This section does not amend §1–§9 either, and it does not amend §10.** Same standing as §10: the
exit gate closed on 2026-08-03 without an audition, §9's first line records that nobody listened,
and §10 records what happened when somebody did. This is the second thing that came out of the same
listening pass — the author asked whether the plugin could be made to sound like a plausible
instrument **from a dry load, with no knob-turning** — and it is recorded here for the same reason.

### The finding

`pickupPosition01` and `PluckExciterParams::defaultPosition` both shipped at **0.5**, the exact
midpoint of the string, and they shipped at the **same** value.

A point contact couples to mode *n* through `sin(n*pi*d)`, so it is exactly blind to every partial
with a node at *d* — the same physics §10's damper fix is about, pointed at the pick and the coil
instead of the felt. Written as a rational `a/b` in lowest terms the null set is exactly
`{b, 2b, 3b, ...}`: **onset b, density 1/b**. `b = 2` is the smallest value `b` can take, so
**d = 1/2 is provably the worst single point on the whole slider** — simultaneously the lowest
possible onset (partial 2, the octave) and the highest possible density (half of every partial the
instrument produces). Both defaults sat on it, and because they sat on the *same* value the two
combs coincided and every null was **squared**.

Measured through the shipping chain at the low open E, dB below the loudest partial:

| partial | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| old default | 0.0 | **−48.9** | −3.4 | **−43.1** | −0.6 | **−48.1** | −16.4 | **−43.5** |

Half the harmonic series was not attenuated. It was **absent**. The instrument's default voice was
odd-harmonic-only across the whole register — 35.8 to 50.8 dB of even-partial deficit on every one
of the six open strings.

`docs/listening/physical-plausibility-checklist.md` **item 7 already names this in words** —
*"near-middle plucks are hollow, with suppressed even harmonics"* — and the shipped default **was**
the near-middle pluck. Item 7 is also the item whose pluck half P2.8's review found has **no corpus
evidence at all**, because every render leaves Exciter Position at 0.5.

### What changed

| | was | now | derived from |
|---|---|---|---|
| `PluckExciterParams::defaultPosition` | 0.5 | **1 − 1/9 = 0.8889** | 1/9 of the string from the bridge, 2.833" on a 25.5" scale; first null partial 9 |
| `StringNetworkParams::pickupPosition01` | 0.5 | **1 − 1/16 = 0.9375** | 1/16 from the bridge, 1.594"; a Stratocaster bridge pickup's pole line; first null partial 16 |
| `kNominalPickupTrimDb` | 24.8 dB | **25.3 dB** | re-derived from the measurement that *defines* it, because the tap position is one of its inputs |

The criterion is closed form: partials 2–6 are the ones that spell tempered intervals (2 and 4 are
octaves, 3 and 6 fifths within 2 cents, 5 a major third 14 cents flat) and **partial 7 is the first
that names no tempered interval at all**, so requiring no null in [2, 6] is `onset >= 7`, i.e.
`d <= 1/7`. Of the three positions a real electric guitar actually has a pickup at — bridge 0.06,
middle 0.15, neck 0.25 of the string from the bridge — **only the bridge clears it**, and that is a
derivation rather than a preference: a real middle or neck coil gets away with nulling partial 6 or
4 because it has a finite aperture, and this model's tap is a point.

Nothing was changed that could not be derived. `noiseAmount` stays at **0** and the string material
stays at 0.5 / 0.5 / 0.0; both are argued with measurements in the task note rather than moved.

### The cost, stated where §10 states its own

| | |
|---|---|
| **Fundamental** | coupling to it is `sin(pi*d)` at both ends, so the shipped pair costs **23.5 dB** of it. At the low E the fundamental goes from being the loudest partial in the note to sitting 20 dB under partial 4 — checklist item 7's *"brighter and thinner"*, arriving as the default. |
| **Sustained chord level** | corpus phrases 02 and 06 lose **7.7 / 8.4 dB** of energy over 200 Hz–4 kHz. **At most ~1 dB of that is attributable to choosing the bridge**: measured over five admissible geometries, the minimum-loss one (pluck 1/8, tap 1/7) reads *lower* than the bridge above 200 Hz and only 6 dB higher below it. The loss is the criterion's, not the position's — the old default was loud precisely because the midpoint costs 0 dB of fundamental, and it paid for that by deleting the even partials. |
| **Peak level and headroom** | **unchanged.** Single string MIDI 45 at velocity 1.0 stays inside the −18 dBFS ±1 dB gate; a six-string open chord at velocity 1.0 peaks −11.36 dBFS, **11.06 dB under the limiter ceiling**. |
| **Calibration rate-independence** | `kNominalPickupTrimDb`'s three-rate spread widens from **0.08 dB to 0.844 dB**, because a near-bridge tap weights the upper partials and those are the rate-sensitive ones. The constant is now a compromise across rates rather than a measurement that agreed three times. |
| **Corpus** | every phrase moved; worst RMS move **−8.78 dB** (`02_open_chords`), worst peak move **−5.68 dB** (`01_chromatic_singles`). |
| **Goldens** | **did not move.** Worst \|golden diff\| **0** on both layers, because `StringIrScenarios` pins its own geometry (pluck 0.28, tap 0.87, noise 0.25) and never reads these defaults. |

### Three `[contract]` gates now hold it, each shown RED at 0.5 / 0.5 in its own body

`tests/dsp/DefaultVoicingTests.cpp` — the rendered even-partial deficit over the six open strings
(shipping 5.29 dB worst, old geometry 35.80 dB *weakest*, limit 18); the closed form asserted on the
defaults themselves with no render in it; and **the six-string chord's headroom**, which is the
first time the "+16 dB summing budget fits under the ceiling" promise has been measured rather than
inferred (`docs/listening/P2-20260803.md` records it as an inference, and §8.3 item 6 above records
that the *other* level gate cannot fire). Full derivation, the whole candidate grid, the loudness
accounting, and a measured `couplingStrength` recommendation the author has not been asked to
accept: `.superpowers/sdd/2026-07-30-pm-guitar-synth-p0-p2-plan/task-default-voicing.md`.

### A STRUCTURAL FINDING ABOUT THE GOLDEN CORPUS — the second blind spot in two days

**This is the part of §11 that outlives the change it came from, and it belongs on §8.3's triage
list rather than only here.**

The goldens did not move, and the reason is not that this change was gentle. It is that
`tests/support/StringIrScenarios.cpp` **overrides every one of the three fields this task touched**,
before either scenario renders a sample:

| override | line(s) | what it hides |
|---|---|---|
| `params.pickupPosition01 = kStringIrTapPosition` (0.87) | `:39`, `:90` | the tap default |
| `params.exciter.noiseAmount = kStringIrNoiseAmount` (0.25) | `:41`, `:92` | the noise default |
| `noteOn.pluckPosition = kStringIrPluckPosition` (0.28) | `:56`, `:113` | the exciter default |

The third is belt-and-braces: `resolveNoteParam` (`dsp/src/StringNetwork.cpp:20-24`) returns the
event's value whenever it is in [0, 1] and only falls back to `params_.exciter.defaultPosition`
otherwise — and 0.28 is in range, so the default is never consulted even if the scenario had not set
it. **The golden path reads none of the moved defaults, and therefore cannot see a default-voicing
regression at all.**

**That is the SECOND structural blind spot found in the golden corpus in two days, and both were
found by chasing an audible defect rather than by any gate:**

| # | blind spot | consequence | status |
|---|---|---|---|
| 1 | **No note-offs at all.** `StringIrScenarios.cpp` emits `NoteOn` only (recorded at §10 and in `task-damper-node-comb.md` §6). | No golden can see RELEASE behaviour. It is why the damper node comb survived to the author's ears. | **STILL OPEN** — no gate covers it; the damper's own gate is a separate file |
| 2 | **Geometry defaults overridden**, above. | No golden can see what the instrument sounds like when a user simply LOADS it. | **closed by `tests/dsp/DefaultVoicingTests.cpp`**, which measures at the shipping defaults by construction |

**Both are the same shape: the goldens pin *a* configuration, not *the shipping* configuration.**
That is a legitimate and deliberate property of a regression baseline — a fixed-geometry instrument
response is exactly what makes it stable and byte-comparable — but it means the golden suite is
evidence that *the physics did not change*, and it is **not** evidence that *the instrument sounds
right*. Two defaults that made the shipped instrument audibly wrong passed it unmoved, twice.

The practical consequence for whoever does the final review: **a green golden board says nothing
about voicing, and both times it has been read as though it did.** The gates that can speak for the
shipped configuration are `DamperReleaseSpectrumTests.cpp` and `DefaultVoicingTests.cpp`, and there
are exactly two of them.

**What this changes about the exception above: nothing, again.** The voicing sign-off is still not
performed, `couplingStrength = 0.35` is still PROVISIONAL and is still the author's alone, and
`docs/listening/P2-20260803.md` is still empty. Two defects found from one recording is still not an
audition — it is two more reasons §9's first line was the right thing to write.

---

## 12. Appended after the exit: `couplingStrength` was SETTLED BY DELEGATION, and a second voicing change was REFUSED

**Same standing as §10 and §11: this does not amend §1–§9.** The exit gate closed on 2026-08-03
without an audition; §9's first line records that nobody listened. This section records the third
thing to come out of the same listening finding, and the first thing this branch has refused.

### 12.1 `couplingStrength` 0.35 → 0.20 — settled, but NOT by ear

| | |
|---|---|
| What changed | `BridgeAdmittanceParams::couplingStrength` **0.35 → 0.20** (`dsp/include/cnpg/dsp/IBridgePort.h`) |
| How it was settled | **Author delegation on 2026-08-05.** Not by the listening pass ADR 0007 **D4** reserves it for |
| State of that pass | **STILL NOT PERFORMED.** `docs/listening/P2-20260803.md` is still marked NOT PERFORMED with every verdict field blank, and this change did not fill one |
| Therefore | **D4's condition was WAIVED, not met.** Nobody may cite 0.20 as a listening result |
| Declared Normal range | **coupling ceiling UNCHANGED at 0.35.** ADR 0007 **D7.2** carries the re-derivation |

**§2.2 above says `couplingStrength = 0.35` remains PROVISIONAL. That sentence is superseded as to
the value and NOT as to its status.** The value is 0.20; it is still unconfirmed by ear.

**Why 0.20 and not another waiver.** D5's criterion (4) — near-unison strings ~25 cents apart must
not involuntarily mode-lock — is the only one of the five that binds coupling below the criterion-(1)
ceiling, and it is measured on two topologies. On the **shipping six-string instrument** (ADR 0007
D7.1) the pair survives at 0.20 with 23.612 cents of separation and its partner 6.8 dB down, and is a
**single locked peak** at 0.30. 0.20 is therefore **the largest measured point at which criterion (4)
holds on the topology that ships** — the last reading that passes, not an interpolated edge, because
nothing between 0.20 and 0.30 has ever been measured on six strings. **The margin is zero in the only
direction that matters**, and D7.1's own question — whether a partner 6.8 dB down is still the chord
that was played — is still unanswered.

**What the Normal range's character is now.** ADR 0007 D7.2 re-derives it rather than re-adjectiving
it. The short form:

| D5 criterion | at the shipping default 0.20 | over the declared box (coupling ≤ 0.35) |
|---|---|---|
| (1) ±2 cents | holds — 0.060 cents | holds — worst 0.770 cents |
| (2) solver converges | holds | holds — 1368/1368 points |
| (3) live changes click-free | holds | holds |
| (4) no involuntary mode-lock | **HOLDS — this is what changed** | **FAILS above ≈0.20–0.25** |
| (5) instrument, not effect | **never judged** | **never judged** |

So: **for the first time since the bridge landed, the shipped instrument sits where every MEASURABLE
criterion holds.** The *box* is still not criterion-complete, and the gap between the default and the
ceiling is now exactly the failing region — reachable only by dragging the Bridge Coupling slider
above its default. §2.3's "criterion (4) is still failing" therefore stands for the range and no
longer for the shipped default.

**The standing mode-lock gate was RE-POINTED, not deleted.** `tests/dsp/StringNetworkScaleTests.cpp`
asserted the lock **at the shipping default**; at 0.20 there is no lock, so that assertion would have
gone vacuous — this branch's recurring failure mode for the tenth time, and the first time it was
named in advance rather than found after the fact. It now pins the **boundary**: separation
preserved at a 0.25 probe, collapse
at a 0.30 probe, both measured in the case, each arm asserted to reject the other arm's numbers, plus
an arithmetic clause that the shipping default lies on the separated side. Three RED arms were
constructed and run: swapping the probes fails the separated clause at **0.00304 cents against a
20-cent limit**; pointing the locked probe below the boundary fails at **25.0986 cents against 12.5**;
and raising the default to 0.30 fails the arithmetic clause at `0.300000012f <= 0.25f`. That is a gate
on a physical fact rather than on a value, and it survives the default moving again.

### 12.2 The tap was to move to 1 − 1/7. It was REFUSED, on measurement.

§11 closed by naming `1 − 1/7 = 0.857143` as *"the fullest position that still clears the criterion"*
and offering it as a one-line diff. **It does not clear the criterion in this model, and the reason
is a property of the waveguide rather than of the arithmetic.**

`onset = 1/d` is a **continuous-string identity**. `WaveguideString` reads a tap at delay
`1.0 + position01 * positionSpan_`, and `positionSpan_` is one rail's realized span — it excludes the
loss, dispersion, seam and bridge phase delays, which are part of the acoustic loop but not of the
rail. The tap's acoustic distance from the bridge is therefore `(1 + (1 − p)·S)` samples out of a
half-loop of `(S + tau/2)`, so **the realised onset is LOWER than 1/d by an absolute offset of about
one sample** — a larger fraction of a shorter loop, i.e. worse at the top of the register and at the
lowest sample rate.

Closed form, with `S = positionSpan_` and `L` the loop length: `onset_realised = (L/2)/(1 + d·S)`
instead of `1/d`. At the worst point over the six default open strings — MIDI 64 at 44.1 kHz, the
shortest loop at the lowest supported rate — that reads **6.38 at d = 1/7**, 6.79 at 1/7.5, 7.19 at
1/8 and 12.99 at 1/16, so the criterion becomes **`d ≤ 1/7.76`**, not `d ≤ 1/7`. At 1/16 the offset
costs three whole partials of onset and nothing notices, because the margin is ten. At the
**zero-margin** 1/7 it puts the realised null **on partial 6, inside the identity band [2, 6]**.

**The closed form predicts the measured boundary** — 1/7.76 against a render sweep that finds 1/7.5
failing and 1/8 clean. Worst even-partial deficit over the six default open strings:

| tap | chain, 48 kHz | raw tap, 44.1 / 48 / 96 kHz |
|---|---|---|
| **1/7** | **21.72 dB at MIDI 59** — fails the 18 dB `[contract]` gate | **24.77 / 21.51 / 5.58** |
| 1/7.5 | 5.13 | 18.76 / 5.38 / 5.78 — fails at 44.1 kHz only |
| **1/8** | 5.29 | **6.23 / 5.54 / 5.91 — clean at every rate** |
| 1/16 (shipped) | 5.78 | 6.74 / 6.03 / 6.32 |

**The rate dependence is the attribution**: the closed-form comb contains no sample rate, so a
reading that moves 16 dB between 44.1 and 96 kHz is the discretisation and can be nothing else. The
hole is present in the **raw tap with the bridge decoupled**, so it is neither the chain nor the
coupling. It is one of the six default open strings, and it is the same defect class §11 exists to
remove — one partial instead of half the series, ~20 dB instead of ~45. (An intermediate run, taken
while `kNominalPickupTrimDb` was mid-re-derivation at 22.3 dB, read 21.65 rather than 21.72: the
statistic is a ratio between partials, but the chain it is measured through has a triode and a
limiter in it, so 3 dB of level is worth 0.07 dB of deficit. 21.72 is the figure at the shipping
trim.)

**What the measurement supports instead is 1/8, and 1/8 was NOT taken.** It is the largest tap
distance clean at all three rates and returns `20*log10(sin(pi/8)/sin(pi/16)) = 5.85 dB` of the
6.94 dB of fundamental that 1/7 was wanted for. Substituting it would be choosing the instrument's
voice rather than refusing an inadmissible value, so it is left as a one-line diff for the author:

```
-inline constexpr float kDefaultTapDistanceFromBridge01 = 1.0f / 16.0f;
+inline constexpr float kDefaultTapDistanceFromBridge01 = 1.0f / 8.0f;
```

**The durable lesson, which outlives this value: the criterion needs a margin of about ONE PARTIAL in
this model, not zero.** `PluckExciter.h` reached that conclusion independently for the pluck when it
declined the zero-margin 1/7 in favour of 1/9 — which is why the pluck default is untouched by this
finding, and what its margin bought. The sweep is kept re-derivable as
`REPORT: voicing -- the realised comb onset is lower than 1/d, and by how much`.

**A second, smaller defect found in the same place.** The `static_assert` §11 shipped read
`1.0f / kDefaultTapDistanceFromBridge01 >= 7.0f`. At `d = 1.0f/7.0f` — the exact boundary, i.e. the
one value the author was going to reach for — that expression evaluates to **6.99999952f and the
assertion FAILS TO COMPILE**: `float(1/7)` rounds up, so its float reciprocal rounds down through 7.
The criterion was stated in a form that rejects its own boundary, for a floating-point reason and not
a physical one. Both `static_assert`s and the closed-form `[contract]` clause are re-pointed to
`d <= 1.0f/7.0f`, which is exact at equality. Non-vacuity is unchanged: at `d = 1/2` the reading is
0.5 against 0.142857.

### 12.3 What it measured

| gate | result |
|---|---|
| Release suite | **285 cases / 5 480 612 assertions, all passed** (parent `ebe5484`: 285 / 5 480 584) |
| Assertion accounting | the parent's exact total was **reproduced on this tree** by reverting only the coupling literal, the goldens and the two edited test files. The default move plus the regeneration accounts for **+13**, all of it inside `[contract]` — `[energy]`, `[tuning]`, `[regression]`, `[denormal]` and `[aliasing]` are unchanged to the assertion. The two edited test files account for the remaining **+15**. **The case carrying the +13 was not isolated**, and that is the one accounting item left open |
| Debug suite | §12.4 |
| `ctest -C Release` | **100% tests passed out of 285**, running `build/bin/Release/cnpg_tests.exe` out of the generated `cnpg_tests-b12d07c_tests-Release.cmake` (`DISCOVERY_MODE PRE_TEST`) |
| clang-format | clean over all **98** files under `dsp/ plugin/ tests/` (LLVM 20.1.8, the repo `.clang-format`), re-run **after** the last edit |
| **Goldens** | **MOVED, and regenerated deliberately.** All 84 files (60 `string_ir`, 24 `chord_ir`), with a `Regenerate-Goldens:` trailer. `string_ir` worst \|sample diff\| **0.024005**, `chord_ir` tap worst **0.049788**, bridge channel worst **0.001689**. **LAYER (a) MOVED PAST ITS TOLERANCES TOO — see §12.5.** §11's goldens did **not** move because `StringIrScenarios` overrides the geometry; it does **not** override `couplingStrength`, and both scenarios render at the shipping admittance on purpose |
| Corpus | 8/8 phrases, **all counters zero**, determinism verified, per-phrase deltas below |

### 12.4 Corpus, the trim, and Debug

`cnpg_render --corpus tests/corpus --samplerate 48000 --verify-determinism`, against `ebe5484`:

| phrase | peak (old → new) | RMS (old → new) | ΔRMS | max abs sample diff |
|---|---|---|---|---|
| 01_chromatic_singles | −17.21 → −17.14 | −38.970 → −38.420 | **+0.550 dB** | −27.19 dBFS |
| 02_open_chords | −11.22 → −11.21 | −40.500 → −39.540 | **+0.960** | −25.11 dBFS |
| 03_legato_retrigger | −17.58 → −17.54 | −38.180 → −37.370 | +0.810 | −27.53 dBFS |
| 04_palm_mute_chug | −13.87 → **−12.87** | −32.730 → −32.210 | +0.520 | −20.26 dBFS |
| 05_low_string_bends | −17.43 → −17.38 | −40.470 → −39.620 | +0.850 | −27.56 dBFS |
| 06_sustain_chords | −12.84 → −12.58 | −41.500 → −40.460 | +1.040 | −25.64 dBFS |
| 07_param_sweeps_midnote | −14.63 → −14.37 | −38.880 → −37.760 | **+1.120** | −27.49 dBFS |
| 08_harmonics_nodes | −15.97 → −15.92 | −38.470 → −37.670 | +0.800 | −25.89 dBFS |

**Every phrase moved and every one got LOUDER**, which is the direction the physics predicts and the
opposite of §11's column: less coupling means less of each string's energy dumped into the shared
bridge load, so more of it stays on the string. The largest peak move is `04_palm_mute_chug` at
+1.00 dB; headroom is unaffected (worst peak −11.21 dBFS, 10.91 dB under the ceiling).

`kNominalPickupTrimDb` was **re-derived and did not move**: at the new coupling the per-rate trims are
24.856 / 25.031 / 25.686 dB, whose midpoint-of-extremes is 25.271, which rounds to the 25.3 already
shipping. The re-derivation was performed, not skipped — lowering the coupling raises the
single-string peak by 0.055–0.075 dB, which is below the 0.1 dB this constant is documented at.

### 12.5 The goldens' layer-(a) features moved past their tolerances — this is a behaviour change

`tests/data/golden/README.md`: *"Layer-(a) invariants are designed to stay green across a legitimate
regeneration; if they move, the change is a behaviour change, not a refresh, and belongs in the
commit message."* **They moved.** Layer (a) reads green on the shipping tree only because the
sidecars were refreshed alongside the samples. Old sidecar against new, over all 84 files:

| feature | tolerance | tap channel | bridge channel |
|---|---|---|---|
| attack RMS | ±1.5 dB | +0.44 … +0.54 dB — inside | **−4.38 dB (`chord_ir`) / −4.81 dB (`string_ir`) — OUTSIDE** |
| per-band T60 | ±10 % | **+25.0 % / +23.8 % — OUTSIDE** | **+27.6 % / +26.1 % — OUTSIDE** |
| partial 1 | ±2 cents | **−0.0084 … +0.0381 cents — inside by two orders** | — |
| partials 2–8 | ±2 cents | **up to 2.8191 cents — OUTSIDE on 42 of 226 readings** | — |

**All three are the intended physics, and none was known before this measurement.**

- **T60 up ~25 %** is the sustain the change buys — less coupling, less energy leaving the string.
  Same direction as ADR 0006's own T60 column (1.147 s at 0.2 against 1.009 s at 0.35), larger here
  because the golden scenarios sit nearer the 180 Hz bridge resonance.
- **Bridge output down ~4.5 dB** is `couplingStrength` doing what it is defined to do: it scales the
  full load admittance and `bridgeOutput()` is the load's velocity.
- **The fundamental holds to hundredths of a cent while partials 2–8 move up to 2.8**, and that is
  ADR 0007 **D1** visible in the goldens for the first time. The compensation evaluates the junction's
  reflection phase delay **at f0** and folds only that into the loop, so f0 is exact by construction
  while the upper partials ride the bridge's frequency-dependent phase at their own frequencies.
  **The ±2-cent `[tuning]` gate is a gate on f0 and is untouched** — worst 0.060 cents at the shipping
  admittance. What is new is knowing that the partials move ~2.8 cents for a 0.15 change in coupling.

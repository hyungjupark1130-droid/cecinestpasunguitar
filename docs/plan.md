# cecinestpasunguitar — P0–P2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build phases P0–P2 of a physical-modeling guitar-deconstruction synthesizer (VST3): a walking skeleton, a playable mono vertical slice (exciter → string → pickup → triode monitoring chain), and the complete physics core (6–8 bidirectionally coupled strings with position-based dampers, a passive-by-construction bridge scattering junction, and two note-allocation modes).

**Architecture:** Two-domain split — a sample-domain physics core (`StringNetwork`: exciter → strings ⇄ bridge, per-sample loop, no inter-module block delay) feeding a block-domain unidirectional chain (pickup tap + RLC → Koren triode → monitoring filters), with the domain boundary crossed via per-block per-string tap buffers computed inside the sample loop. `dsp/` is a JUCE-free, headless-testable C++20 static library; `plugin/` is a thin JUCE 8 wrapper with APVTS as the single parameter source.

**Tech Stack:** C++20 · JUCE 8 (FetchContent-pinned, GPL path) · CMake + CMakePresets · Catch2 v3 · pluginval · GitHub Actions (Windows full build + Linux dsp-only portability job).

## Global Constraints

- License: **GPLv3, public GitHub repo from P0**. No GPL-incompatible dependencies.
- Plugin identity (permanent, pinned in P0): product/display name `cecinestpasunguitar` (working title, display rename safe), vendor **Hyung Ju Park**, manufacturer code `Hjpk`. State-version integer embedded from P1; **no session-compatibility guarantee before P5**.
- Formats: VST3 + Standalone. Stereo output bus (duplicated mono until P4) + **disabled-by-default sidechain input bus declared in P0** (frozen bus layout). `supportsDoublePrecisionProcessing() == false` in v1.
- `dsp/` is 100% JUCE-free; only `plugin/` links JUCE. All heavy offline work is C++ in-repo through P2 — **no Python/LTspice anywhere in P0–P2** (`tools/` stays a stub until P3).
- Realtime discipline: no allocation or locks on the audio thread; single audio thread through P2; all physics parameters smoothed; float32 realtime path with FTZ/DAZ guard; float64 offline references; sample-domain classes templated on `SampleT` (float/double) — tier-2 energy tests run the double instantiation.
- Pitch design envelope MIDI 21–108; strings configurable 1..8 (default 6), preallocation sized for 8 at MIDI 21 against `max(hostSampleRate, kMaxDesignRateHz)`.
- Performance gate (P2 exit): median CPU ≤ 30% of one core (25% aspirational) at 48 kHz / 128-sample blocks, `p2_default6` config, measured by `cnpg_bench` on the dev machine (Windows 11 Pro workstation); `p2_max8` reported, never gated. Native 44.1–96 kHz; 192 kHz best-effort. CI reports but never gates performance. **The benchmark target lands in P1.**
- Reference acceptance host: **Ableton Live on Windows 11**; pluginval (high strictness) is the automated gate; all manual acceptance criteria are phrased against Live.
- Oversampling factors: {2, 4, 8} only; default 2×; aliasing gate −60 dBc on the triode in P1.
- No MPE in P0–P2 (P5); the `NoteAllocator` keeps a clean channel=string seam.
- Capacity assumption: solo developer, full-time (35+ h/wk); timeboxes in calendar weeks.

## 0. Locked decision register

Resolutions from the pre-plan Q&A (2026-07-30). These are settled; reopening any of them is a user decision, not an implementer's.

| # | Decision |
|---|----------|
| Q1 | Exciter = one-shot pluck/pick only through P2: pluck position + hardness user-facing, velocity → amplitude + mild hardness increase, small noise-burst component. Sustained excitation deferred to P4 feedback bus. |
| Q2 | Damper position and pickup position are continuously modulatable while a note rings (click-free machinery in P2, amplitude-complementary linear crossfades); exciter position fixed at note-on. |
| Q3 | Strings monophonic, count 1..8 (default 6). Global `retriggerMode`: **Physical** (same-pitch retrigger plucks over ringing state; pitch change = quick damper choke → retune ramp → re-excite) / **Synth** (fast fade, reset, instant re-init). Audible-slide mode deferred. |
| Q4 | No MPE in P0–P2. P2 allocation modes: (a) guitar-emulation fingering logic, (b) user-defined free zones. Channel=string seam reserved for P5. |
| Q5 | Global pitch bend ±2 st from P1; CC64 sustain (defers damper engage) in P2; mod-wheel vibrato LFO deferred to P5. |
| Q6 | Pitch design envelope MIDI 21–108 (A0–C8). |
| Q7 | Monitoring chain: PickupTap → TriodeStage (bypassable) → CabFilter ~5 kHz LP (bypassable) → OutputGain → SoftClipLimiter. No tone stack before P4. |
| Q8 | Pickup in P1 = position tap + linear RLC biquad; magnetic nonlinearity deferred to P4. |
| Q9 | One global shared string-physics parameter set + per-string block (tuning offset, mute/enable); generic DAW editor through P2. |
| Q10 | Single audio thread through P2; perf gate as in Global Constraints; benchmark lands P1. |
| Q11 | Disabled sidechain input bus declared in P0; implemented post-P2. |
| Q12 | Identity as in Global Constraints; state-version from P1; no session compat before P5. |
| Q13 | GPLv3, public repo. |
| Q14 | Ableton Live reference host + pluginval + Standalone target. |
| Q15 | GitHub Actions from P0: Windows (build+tests+pluginval) + Ubuntu (dsp-only tests). |
| Q16 | Hybrid milestone gates: CI objective proxies + author listening pass over the versioned corpus (with mandated abuse cases) against the written physical-plausibility checklist. |
| Q17 | Bridge fallback = design-for-fallback: `BridgeJunction` and `SympatheticResonatorBus` share `IBridgePort`; trigger = corrected energy suite still red after a 1–2 week timebox in P2 (task P2.5). |
| Q18 | Full-time solo capacity; parallel workstreams acceptable. |
| RC1 | P2 tuning calibration = C++ headless in-repo target (`cnpg_calibrate`) measuring the actual dsp/ filters. No Python. |
| RC2 | P1 triode is a static Koren waveshaper (no bias drift): the P1 chain is **not a voicing reference for drive feel**; supply-voltage/heater THD hooks exist as signature-level contracts in the P1 drafts, implemented P3+. |

---

# 1. Repository layout + CMake strategy

## 1.1 Annotated file tree

Every path below is created in P0 unless tagged with the phase in which it lands. Names of CMake targets, directories, and namespaces follow the locked naming conventions exactly.

```
cecinestpasunguitar/
├── CMakeLists.txt                  # Top-level: project(), C++20, options, adds subdirs, FetchContent includes
├── LICENSE                         # GPLv3 full text (locked license; enables JUCE GPL/no-splash path)
├── README.md                       # Vision summary, build instructions (MSVC + Linux dsp-only), phase status table
├── .gitignore                      # Ignores build*/, out/, .vs/, .vscode/, .cache/, CMakeUserPresets.json, *.user
├── .clang-format                   # LLVM-derived style: 4-space indent, 120 cols, pointer-left; enforced by CI format check
├── CMakePresets.json               # Presets: windows-msvc-release, windows-msvc-debug, linux-dsp-only (CI parity locally)
├── cmake/
│   ├── Dependencies.cmake          # All FetchContent declarations with pinned GIT_TAGs + commit SHAs (single pin point)
│   ├── CompilerWarnings.cmake      # cnpg_set_warnings(): MSVC /permissive- /W4, GCC/Clang -Wall -Wextra; /WX|-Werror toggle
│   └── PluginvalPin.cmake          # Pinned pluginval release URL + SHA-256 used by CI download step (P0 pin-verify target)
├── dsp/                            # JUCE-free physics + block DSP core -> static lib cnpg_dsp
│   ├── CMakeLists.txt              # Defines cnpg_dsp (STATIC), include dirs, warnings; links NOTHING external
│   ├── include/cnpg/dsp/           # Public headers, one per interface subsection (namespace cnpg::dsp)
│   │   ├── Common.h                # Sample/Sample64 aliases, kMaxStrings/kMinMidiNote/kMaxMidiNote/kMaxDesignRateHz/kMaxOversampling, lifecycle contract comment (§2.1)
│   │   ├── EventQueue.h            # NoteEventType, NoteEvent, EventQueue<Capacity>, BlockEventQueue — header-only template (§2.2)
│   │   ├── MidiTranslation.h       # (P1) pure JUCE-free translation: raw (status, data1, data2, sampleOffset) tuples -> NoteEvent/RawMidiEvent streams; tested headlessly in cnpg_tests
│   │   ├── PluckExciter.h          # PluckExciterParams, PluckExciter (§2.3)
│   │   ├── WaveguideString.h       # FractionalDelayKind, StringMaterialParams, WaveguideStringParams, WaveguideString (§2.4)
│   │   ├── DamperJunction.h        # DamperJunctionParams, DamperJunction (§2.5)
│   │   ├── IBridgePort.h           # (P1) IBridgePort interface + trivial passive reflective termination used until P2's BridgeJunction (§2.6)
│   │   ├── BridgeJunction.h        # (P2) BridgeAdmittanceParams, BridgeJunction (§2.6)
│   │   ├── SympatheticResonatorBus.h # (P2, only if fallback triggers) unidirectional IBridgePort implementation decl (§2.6)
│   │   ├── StringNetwork.h         # RetriggerMode, StringNetworkParams, StringTapBuffers, StringNetwork (§2.7)
│   │   ├── PickupTap.h             # PickupTapParams, PickupTap (§2.8)
│   │   ├── TriodeStage.h           # KorenTriodeParams, TriodeStageParams, TriodeStage incl. TransferTableView + THD hooks (§2.9)
│   │   ├── Oversampler.h           # Oversampler with processWrapped/upsample/downsample (§2.10)
│   │   ├── CabFilter.h             # CabFilterParams, CabFilter (§2.11)
│   │   ├── SoftClipLimiter.h       # SoftClipLimiterParams, SoftClipLimiter (§2.11)
│   │   ├── OutputGain.h            # OutputGainParams, OutputGain (§2.11)
│   │   ├── NoteAllocator.h         # RawMidiEvent, AllocationMode, StringZone, NoteAllocatorParams, NoteAllocator (§2.12)
│   │   ├── ModuleGraph.h           # IBlockModule, ModuleGraph (§2.13)
│   │   ├── ScopedFtzDazGuard.h     # (P1) JUCE-free RAII FTZ/DAZ MXCSR guard, instantiated first in processBlock (dsp-side; there is no plugin DenormalGuard.h)
│   │   └── generated/
│   │       └── TuningCalibrationData.h # (P2) checked-in generated constexpr per-rate cents tables, regenerated by cnpg_calibrate
│   ├── data/
│   │   └── calibration/            # (P2) cnpg_calibrate output: checked-in, human-diffable measured tuning tables
│   │       ├── tuning_cal_44100.csv
│   │       ├── tuning_cal_48000.csv
│   │       └── tuning_cal_96000.csv
│   └── src/                        # Implementation files; SoA layout lives inside StringNetwork.cpp
│       ├── PluckExciter.cpp        # (P1) burst synthesis, velocity->amplitude+hardness mapping
│       ├── WaveguideString.cpp     # (P1) rails, termination filters, fractional delay, tuning compensation
│       ├── DamperJunction.cpp      # (P2) linear 2-port scatter + felt-time-constant engagement smoother
│       ├── BridgeJunction.cpp      # (P2) N-port scattering, positive-real admittance clamp, S-matrix hook
│       ├── SympatheticResonatorBus.cpp # (P2, only if fallback triggers) unidirectional IBridgePort implementation
│       ├── StringNetwork.cpp       # (P1 single-string; P2 full) per-sample loop, event consumption, tap buffers, energy estimate
│       ├── PickupTap.cpp           # (P1) tap-buffer summing + RLC biquad
│       ├── TriodeStage.cpp         # (P1) Koren ECC83 waveshaper, publishedEcc83(), table-loading validation
│       ├── Oversampler.cpp         # (P1) IIR halfband cascades, latency accounting
│       ├── CabFilter.cpp           # (P1) fixed ~5 kHz lowpass body
│       ├── SoftClipLimiter.cpp     # (P1) soft-knee ceiling body
│       ├── OutputGain.cpp          # smoothed-gain body (first module, lands with P0.3)
│       ├── NoteAllocator.cpp       # (P1 trivial single-string; P2 modes + CC64) allocation logic
│       └── ModuleGraph.cpp         # (P1) topo sort, freeze(), buffer plan; exercised by contract tests through P2
├── plugin/                         # JUCE layer -> cnpg_plugin (VST3 + Standalone)
│   ├── CMakeLists.txt              # juce_add_plugin() config (see §1.4), links cnpg_dsp, JUCE modules
│   └── src/
│       ├── PluginProcessor.h       # AudioProcessor: bus layout pin, APVTS owner, processBlock orchestration
│       ├── PluginProcessor.cpp     # ScopedFtzDazGuard engage, APVTS snapshot->param structs, domain wiring, state-version I/O
│       ├── Parameters.h            # APVTS layout builder, parameter IDs, state-version integer constant (from P1)
│       ├── Parameters.cpp          # createParameterLayout(), per-block atomic snapshot into plain structs
│       ├── MidiConverter.h         # Thin adapter: juce::MidiBuffer -> raw (status, data1, data2, sampleOffset) tuples for cnpg::dsp MidiTranslation (sample-accurate, channel carried opaquely)
│       ├── MidiConverter.cpp       # Adapter body; tuples flow through MidiTranslation into NoteAllocator::allocate
│       └── PluginEditor.cpp        # Returns juce::GenericAudioProcessorEditor (generic DAW editor through P2; GUI is P5)
├── tests/                          # All headless verification -> cnpg_tests, cnpg_bench, cnpg_render, cnpg_calibrate
│   ├── CMakeLists.txt              # Defines the four executables, catch_discover_tests, cnpg_regen_goldens target
│   ├── dsp/                        # Catch2 v3 sources; tags: [contract] [energy] [regression] [aliasing] [denormal] [tuning]
│   │   ├── OutputGainTests.cpp     # [contract] unity-gain bit-exactness, 12 dB step ramp, reset, numSamples 1..maxBlockSize (first P0 test)
│   │   ├── ScopedFtzDazGuardTests.cpp # (P1) [contract] MXCSR FTZ/DAZ set inside scope, restore on exit
│   │   ├── EventQueueTests.cpp     # (P1) [contract] capacity, ordering assert, droppedCount
│   │   ├── MidiTranslationTests.cpp # (P1) [contract] raw (status, data1, data2, sampleOffset) tuple -> RawMidiEvent translation, headless
│   │   ├── NoteAllocatorTests.cpp  # (P1) [contract] monophonic P1 allocation, velocity mapping, note-range rejection, channel opacity
│   │   ├── PluckExciterTests.cpp   # (P1) [contract] latch semantics, determinism, burst termination
│   │   ├── WaveguideStringTuningTests.cpp # (P1) [tuning] analytic-compensation path: MIDI 33-96 ±2-cent gate, 21-32/97-108 report-only table
│   │   ├── WaveguideStringRegressionTests.cpp # (P1) [regression] two-layer IR suite: feature invariants (first 8 partials) + float64 goldens (atol ~1e-7)
│   │   ├── StringNetworkTests.cpp  # (P1/P2) [contract] event consumption at offsets, tap-buffer validity
│   │   ├── PitchBendClickTests.cpp # (P1) [contract] click-free ±2 st bend under continuous wheel motion
│   │   ├── DenormalTests.cpp       # (P1) [denormal] 60 s decay tail; state-inspection case (both CI jobs) + timing-ratio case (local-only)
│   │   ├── PickupTapTests.cpp      # (P1) [contract] summing, RLC response
│   │   ├── TriodeStageTests.cpp    # (P1) [contract] bypass bit-exact, THD monotonicity, deferred hooks inert, table validation
│   │   ├── AliasingGateTests.cpp   # (P1) [aliasing] 1244.5/4186 Hz gate, -60 dBc folded threshold, factor 4/8 reports; oversampler latency [contract]
│   │   ├── MonitoringChainTests.cpp# (P1) [contract] CabFilter/SoftClipLimiter/OutputGain bypass paths, limiter-last ceiling enforcement
│   │   ├── ModuleGraphTests.cpp    # (P1) [contract] cycle rejection, freeze, latency summation, chain equivalence vs direct calls
│   │   ├── StringNetworkScaleTests.cpp # (P2) [contract] 1..8 strings, per-string tuning offsets, enable ramp
│   │   ├── DamperEnergyTests.cpp   # (P2) [energy] tier-1 damper |S| grid sweep (§4.2 grids)
│   │   ├── DamperNodeSuppressionTests.cpp # (P2) [contract] node-suppression proxy
│   │   ├── DamperFeltTimeTests.cpp # (P2) [contract] felt time-constant NoteOff envelope
│   │   ├── MovingPositionClickTests.cpp # (P2) [contract] click metric under pickup/damper position sweeps
│   │   ├── MovingJunctionEnergyTests.cpp # (P2) [energy] bounded-growth criterion under junction/tap motion
│   │   ├── BridgeEnergyTierOneTests.cpp # (P2) [energy] tier-1 bridge grid spectral norm, positive-real clamp
│   │   ├── NetworkEnergyTierTwoTests.cpp # (P2) [energy] tier-2 lossless non-increase (1e-9, double instantiation)
│   │   ├── NetworkEnergyTierThreeTests.cpp # (P2) [energy] tier-3 float32 monotone-decreasing energyEstimate envelope
│   │   ├── CoupledStringsTests.cpp # (P2) [contract] sympathetic response, beat rate at couplingStrength 0.1, Weinreich two-stage decay
│   │   ├── BridgePortContractTests.cpp # (P2) [contract] IBridgePort interface suite, parameterized over implementations
│   │   ├── NoteAllocatorFingeringTests.cpp # (P2) [contract] GuitarFingering assignment + steal policy
│   │   ├── NoteAllocatorZonesTests.cpp # (P2) [contract] FreeZones overlap/LRU, unassignable-note drop
│   │   ├── SustainPedalTests.cpp   # (P2) [contract] CC64 deferral, stale-NoteOff drop after steal
│   │   ├── RetriggerModeTests.cpp  # (P2) [contract] Physical/Synth retrigger semantics
│   │   ├── TuningAccuracyTests.cpp # (P2) [tuning] calibration-table gate: ±2 cents, all 88 notes x 3 rates
│   │   └── CorpusSweepTests.cpp    # (P2) [contract] corpus render click/NaN/denormal scans
│   ├── data/
│   │   └── golden/                 # Committed float64 golden IRs: golden/<scenario>/<variant>/<rate>/<name>.f64 + .json sidecar (locked layout, scenario-first)
│   │       └── README.md           # Golden regeneration policy + commit-message requirement (see §1.6)
│   ├── corpus/                     # Versioned MIDI phrase corpus (locked location, flat, append-only), consumed by cnpg_render
│   │   ├── corpus.json             # Corpus version, per-phrase description, expected render duration
│   │   ├── 01_chromatic_singles.mid       # (P1)
│   │   ├── 02_open_chords.mid             # (P2)
│   │   ├── 03_legato_retrigger.mid        # (P1 default retrigger mode; per-mode variant renders added in P2)
│   │   ├── 04_palm_mute_chug.mid          # (P2)
│   │   ├── 05_low_string_bends.mid        # (P1)
│   │   ├── 06_sustain_chords.mid          # (P2)
│   │   ├── 07_param_sweeps_midnote.mid    # (P1)
│   │   ├── 07_param_sweeps_midnote.json   # (P1) automation-lane sidecar
│   │   ├── 08_harmonics_nodes.mid         # (P2)
│   │   └── 08_harmonics_nodes.json        # (P2) automation-lane sidecar
│   ├── bench/
│   │   └── BenchMain.cpp           # (P1, locked) cnpg_bench: [--config <name> | --strings N] --samplerate R --blocksize B --oversample F --seconds S; median/p99/max block time -> CPU%; P2 adds named configs p2_default6/p2_max8
│   ├── render/
│   │   ├── RenderMain.cpp          # (P1) cnpg_render: corpus MIDI -> WAV for listening pass; also golden regeneration driver
│   │   ├── MidiFileReader.h        # (P1) Minimal JUCE-free SMF parser (corpus files only; created by P1.11)
│   │   ├── MidiFileReader.cpp      # (P1) Parser body
│   │   ├── WavWriter.h             # (P1) Minimal JUCE-free RIFF/WAV float writer (created by P1.11)
│   │   └── WavWriter.cpp           # (P1) Writer body
│   └── calibrate/
│       └── CalibrateMain.cpp       # (P2) cnpg_calibrate: measures actual dsp/ loop/dispersion filters -> dsp/data/calibration CSVs + generated/TuningCalibrationData.h
├── tools/                          # Stub until P3 (locked) — no build targets reference this directory before P3
│   └── README.md                   # States: intentionally empty; activates in P3 for BOTH the Python modal-extraction pipeline AND the C++ headless triode-table generator (never Python in P0-P2)
├── docs/
│   ├── plan.md                     # This plan document, committed verbatim (created by P0.1)
│   ├── listening/
│   │   ├── physical-plausibility-checklist.md # (P1, created by P1.11) written checklist for per-milestone author listening pass; P2 appends items
│   │   └── P<phase>-<yyyymmdd>.md  # (P1 onward) per-milestone listening reports, one per pass
│   ├── bench/
│   │   ├── p1-baseline.md          # (P1) first cnpg_bench regression numbers + machine spec
│   │   └── p2-exit.md              # (P2) P2 exit-gate measurements, P2.5 outcome, voicing sign-off
│   └── decisions/                  # In-repo decision records (numbered by creation order, authored when each concludes)
│       ├── README.md               # Record format: context / options / measurement / decision / date
│       ├── 0001-pluginval-baseline.md    # pluginval version + command baseline (P0.6)
│       ├── 0002-fractional-delay.md      # (P1) Lagrange3 vs Thiran1 spike result
│       ├── 0003-adaa-vs-oversampling.md  # (P1) timeboxed triode antialiasing spike result
│       └── 0006-p2-bridge-passivity-fallback.md # (P2) bridge passivity outcome or fallback trigger record
└── .github/
    ├── workflows/
    │   └── ci.yml                  # Two jobs: windows (full) + ubuntu (dsp-only); see §1.5
    └── scripts/
        ├── check-golden-commit.sh  # Fails CI if a commit touches tests/data/golden/ without a "Regenerate-Goldens:" trailer
        └── verify-pins.cmake       # P0 pin-verify: compares git ls-remote JUCE/Catch2 tag SHAs against the SHAs recorded as comments in cmake/Dependencies.cmake, fails on drift
```

## 1.2 Top-level CMake layout

`CMakeLists.txt` (root) does, in order:

1. `cmake_minimum_required(VERSION 3.24)` — needed for `FetchContent` with `OVERRIDE_FIND_PACKAGE` and good MSVC preset support.
2. `project(cecinestpasunguitar VERSION 0.1.0 LANGUAGES CXX)`.
3. Global language settings: `CMAKE_CXX_STANDARD 20`, `CMAKE_CXX_STANDARD_REQUIRED ON`, `CMAKE_CXX_EXTENSIONS OFF`. No fast-math anywhere (`/fp:precise` MSVC default kept) — float64 goldens and the 1e-9 energy tolerance depend on predictable arithmetic; FTZ/DAZ is a runtime RAII concern, not a compiler flag.
4. Output layout: sets `CMAKE_RUNTIME_OUTPUT_DIRECTORY` (per-config, established by P0.2) so every executable lands in `build/bin/<Config>/` — all verification commands throughout the plan therefore use `build\bin\Release\<exe>.exe`.
5. Options:
   - `CNPG_BUILD_PLUGIN` (default `ON`) — gates JUCE FetchContent and `plugin/`. The Linux CI job sets `OFF`, so JUCE is never even downloaded there.
   - `CNPG_BUILD_TESTS` (default `ON`) — gates `tests/` and `enable_testing()`.
   - `CNPG_WARNINGS_AS_ERRORS` (default `ON`) — policy: warnings are errors everywhere, always, including local builds; the option exists solely as an escape hatch for dependency-upgrade triage and is never set `OFF` in CI. Warning flags apply only to cnpg targets, never to FetchContent'd dependencies (JUCE/Catch2 build with their own settings via `SYSTEM` includes).
6. `include(cmake/CompilerWarnings.cmake)` — provides `cnpg_set_warnings(<target>)`: MSVC `/permissive- /W4 /Zc:__cplusplus /utf-8` (+ `/WX`); GCC/Clang `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` (+ `-Werror`). Both toolchains must stay clean: MSVC is the dev machine, GCC and Clang are exercised by the ubuntu CI job as the portability guard.
7. `include(cmake/Dependencies.cmake)` — all pins in one file (below).
8. `add_subdirectory(dsp)`, then conditionally `add_subdirectory(plugin)` and `add_subdirectory(tests)`.

## 1.3 Dependency pinning (FetchContent)

`cmake/Dependencies.cmake` contains exactly two `FetchContent_Declare` blocks:

```cmake
include(FetchContent)

# JUCE 8 — latest stable 8-series tag as of authoring: 8.0.12 (Dec 2025).
# JUCE 9.0.0 exists but JUCE 8 is the locked major. GPL path, no splash.
# Tag commit SHA recorded here by the first P0 pin-verify run; verify-pins.cmake fails on drift.
FetchContent_Declare(JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG        8.0.12          # annotated release tag — stays a tag (GIT_SHALLOW requires it)
    GIT_SHALLOW    TRUE)

# Catch2 v3 — latest stable as of authoring: v3.10.0.
# Tag commit SHA recorded here by the first P0 pin-verify run; verify-pins.cmake fails on drift.
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.10.0         # annotated release tag — stays a tag (GIT_SHALLOW requires it)
    GIT_SHALLOW    TRUE)

if(CNPG_BUILD_PLUGIN)
    FetchContent_MakeAvailable(JUCE)
endif()
if(CNPG_BUILD_TESTS)
    FetchContent_MakeAvailable(Catch2)
endif()
```

P0 pin-verify step (mandatory, cheap): the pins stay human-readable annotated release tags as `GIT_TAG` with `GIT_SHALLOW TRUE` — SHA pins must never be combined with shallow clones. Immutability is enforced instead by `.github/scripts/verify-pins.cmake`: the first P0 CI run records each tag's `git ls-remote`-resolved commit SHA as a comment in `cmake/Dependencies.cmake`, and every subsequent CI run re-runs `git ls-remote` and compares the live tag SHAs against those recorded comment SHAs, failing on drift (tags are movable; the recorded SHAs pin them down). Any later dependency bump is a deliberate commit touching only `Dependencies.cmake`. No chowdsp_wdf in P0–P2 (re-evaluate at P4); no other third-party code.

## 1.4 Target graph and JUCE-freedom of cnpg_dsp

| Target | Kind | Links | JUCE? |
|---|---|---|---|
| `cnpg_dsp` | static lib | nothing external | **never** |
| `cnpg_plugin` | `juce_add_plugin` (VST3 + Standalone) | `cnpg_dsp`, JUCE modules | yes |
| `cnpg_tests` | Catch2/CTest executable | `cnpg_dsp`, `Catch2::Catch2WithMain` | no |
| `cnpg_bench` | executable (lands P1, locked) | `cnpg_dsp` | no |
| `cnpg_render` | executable (lands P1) | `cnpg_dsp` | no |
| `cnpg_calibrate` | executable (lands P2) | `cnpg_dsp` | no |

`cnpg_dsp` JUCE-freedom is enforced structurally, not by review: (1) `dsp/CMakeLists.txt` declares no dependency, so JUCE headers are simply not on its include path — any `#include <juce_...>` fails to compile on every platform; (2) the ubuntu CI job configures with `-DCNPG_BUILD_PLUGIN=OFF`, so JUCE is never fetched, proving `cnpg_dsp`, `cnpg_tests`, `cnpg_bench`, `cnpg_render`, and `cnpg_calibrate` build and pass on a JUCE-less GCC/Clang toolchain. All five consumers link `cnpg_dsp` via `target_link_libraries(... PRIVATE cnpg_dsp)` and see only `dsp/include/`. Support code the headless executables need (SMF parsing, WAV writing) lives in `tests/render/` as minimal JUCE-free implementations rather than pulling JUCE into the test domain.

`plugin/CMakeLists.txt` core configuration:

```cmake
juce_add_plugin(cnpg_plugin
    PRODUCT_NAME              "cecinestpasunguitar"   # working title; display rename is safe later
    COMPANY_NAME              "Hyung Ju Park"          # permanent vendor identity
    BUNDLE_ID                 com.hyungjupark.cecinestpasunguitar
    PLUGIN_MANUFACTURER_CODE  Hjpk                     # pinned in P0, never changes
    PLUGIN_CODE               Cnpg                     # pinned in P0, never changes
    FORMATS                   VST3 Standalone          # Standalone = fast edit-listen loop (locked)
    IS_SYNTH                  TRUE
    NEEDS_MIDI_INPUT          TRUE
    NEEDS_MIDI_OUTPUT         FALSE
    VST3_CATEGORIES           Instrument Synth
    COPY_PLUGIN_AFTER_BUILD   FALSE)
target_compile_definitions(cnpg_plugin PRIVATE
    JUCE_DISPLAY_SPLASH_SCREEN=0   # legitimate under the GPLv3 JUCE license path
    JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0)
```

Both 4-char codes follow the one-uppercase-then-lowercase convention so they stay valid if AU is ever added. The bus layout is frozen in `PluginProcessor`: `BusesProperties().withOutput("Output", stereo, true).withInput("Sidechain", stereo, false)` — stereo out from P0 carrying duplicated mono until P4; the sidechain input is declared but disabled by default and unused until post-P2. `isBusesLayoutSupported()` accepts main out == stereo AND sidechain ∈ {`AudioChannelSet::disabled()`, stereo}; every other layout is rejected, so the identity is stable for hosts from the first release. The processor also reports `supportsDoublePrecisionProcessing() == false` (locked: no VST3 double-precision path in v1; pinned in P0.4).

## 1.5 CI workflows (`.github/workflows/ci.yml`)

**Job `windows`** (`windows-latest`, MSVC):
1. Checkout; restore FetchContent cache (below).
2. Configure: `cmake --preset windows-msvc-release` (sets `CNPG_WARNINGS_AS_ERRORS=ON`).
3. Build all targets: `cmake --build --preset windows-msvc-release`.
4. Test: `ctest --test-dir build -C Release --output-on-failure` (full suite: all tags including `[energy]`, `[regression]`, `[aliasing]`, `[denormal]`, `[tuning]`; the `windows-msvc-release` preset's binary dir is `build`).
5. Golden-commit guard: run `.github/scripts/check-golden-commit.sh` over the push/PR commit range.
6. Benchmark report: run `cnpg_bench`, upload its report as an artifact. CI **reports** these numbers and never gates on them (locked) — the 25–30 % single-core gate is a manual milestone criterion read from this artifact.
7. pluginval: download the pinned pluginval Windows release named in `cmake/PluginvalPin.cmake` (Tracktion's official GitHub release binary, pin = version **v1.0.4** + SHA-256 checksum verified before execution; the P0 pin-verify step confirms this is the latest stable and updates if not), then run `pluginval --strictness-level 10 --validate <build>/cnpg_plugin_artefacts/Release/VST3/cecinestpasunguitar.vst3`. Failure fails the job.
8. Upload artifacts: VST3 bundle, Standalone exe, bench report, pluginval log.

**Job `ubuntu`** (`ubuntu-latest`, GCC; a second matrix leg uses Clang):
1. Checkout; restore Catch2 cache.
2. Configure: `cmake --preset linux-dsp-only` (the preset sets `CNPG_BUILD_PLUGIN=OFF` and `CNPG_WARNINGS_AS_ERRORS=ON`) — JUCE never fetched.
3. Build `cnpg_dsp cnpg_tests cnpg_bench cnpg_render cnpg_calibrate`; run `ctest --test-dir build-dsp --output-on-failure` (the `linux-dsp-only` preset's binary dir is `build-dsp`). This job is the portability guard for the headless dsp/ domain only (locked); it produces no plugin.

**Caching:** `actions/cache` on `build/_deps/*-src` keyed by `hashFiles('cmake/Dependencies.cmake')` with the OS as prefix — a dependency bump invalidates exactly one key, and shallow clones make cold misses cheap. The pluginval binary is cached under a key derived from `PluginvalPin.cmake`.

## 1.6 Regenerate-goldens target

`tests/CMakeLists.txt` exposes:

```cmake
add_custom_target(cnpg_regen_goldens
    COMMAND $<TARGET_FILE:cnpg_render> --regenerate-goldens
            --out ${CMAKE_SOURCE_DIR}/tests/data/golden
    DEPENDS cnpg_render
    COMMENT "Rewrites float64 golden IRs for every scenario/algorithm variant at 44.1/48/96 kHz")
```

`cnpg_render --regenerate-goldens` runs the float64 reference path per scenario per algorithm variant per sample rate and overwrites `tests/data/golden/<scenario>/<variant>/<rate>/` (each `<name>.f64` with its `.json` sidecar). The locked "explanatory commit message" requirement is mechanized: `.github/scripts/check-golden-commit.sh` fails CI whenever a commit modifies anything under `tests/data/golden/` unless its message body contains a `Regenerate-Goldens:` trailer followed by a non-empty explanation of *why* the reference output legitimately changed. `tests/data/golden/README.md` documents the workflow: build `cnpg_regen_goldens`, inspect the diff, commit with the trailer. Feature-invariant tests (partials ±2 cents, per-band T60 ±10 %, RMS bounds) remain green across regeneration by design; only the tight-tolerance (atol ≈ 1e-7) golden comparisons are refreshed.

*(Version pins confirmed against [JUCE releases](https://github.com/juce-framework/JUCE/releases) — [8.0.12](https://github.com/juce-framework/JUCE/releases/tag/8.0.12) is the newest 8-series tag — and [Catch2 releases](https://github.com/catchorg/Catch2/releases), newest v3 tag v3.10.0.)*

## 2. dsp/ core interface drafts (signature level)

All types below live in `namespace cnpg::dsp`, are 100% JUCE-free, and compile headless into the `cnpg_dsp` static library. Public headers live in `dsp/include/cnpg/dsp/`: §2.2 is `EventQueue.h` (NoteEvent + EventQueue + BlockEventQueue), §2.6 splits into `IBridgePort.h`, `BridgeJunction.h`, and `SympatheticResonatorBus.h`, §2.11 splits into `CabFilter.h`, `SoftClipLimiter.h`, and `OutputGain.h` (MonitoringChain.h does not exist), §2.14 is `MidiTranslation.h`; every other subsection maps to a single identically named header. Declarations are signature-level with contract comments; bodies are implementation work assigned in the task sections. Task sections reference these names exactly as written here.

### 2.1 Unified module contract, Sample aliases, parameter pattern

Every dsp/ module follows one lifecycle: `prepare()` on the message thread (may allocate; sizes all state for the worst case — 8 strings, MIDI 21, `maxBlockSize` — against `max(hostSampleRate, kMaxDesignRateHz)`, so a 192 kHz best-effort host never under-allocates delay rails), `reset()` realtime-safe state clear, `process(...)` realtime-safe (no allocation, no locks, no exceptions, no I/O, no denormal traps — FTZ/DAZ is engaged by the caller's RAII guard in `processBlock`). Parameters arrive as plain aggregate structs snapshotted from APVTS atomics once per block; each module owns its smoothers — per-sample ramps inside the sample domain, per-block ramps elsewhere. `setParams()` is realtime-safe and only retargets smoothers.

```cpp
namespace cnpg::dsp {

using Sample   = float;   // realtime path (float32, locked)
using Sample64 = double;  // offline references, goldens, energy accounting in tests

// Design-envelope constants; all preallocation in prepare() is sized against these.
inline constexpr int kMaxStrings        = 8;      // active count configurable 1..8, default 6
inline constexpr int kMinMidiNote       = 21;     // A0
inline constexpr int kMaxMidiNote       = 108;    // C8
inline constexpr double kMaxDesignRateHz = 96000.0; // 44.1-96 kHz guaranteed; 192 kHz best-effort
inline constexpr int kMaxOversampling   = 8;      // Oversampler factor upper bound; valid factors {2, 4, 8} only

// Module lifecycle convention (informal concept; every module below conforms):
//   void prepare(double sampleRate, int maxBlockSize);   // message thread, may allocate
//   void reset() noexcept;                                // realtime-safe, clears state
//   void setParams(const <Module>Params&) noexcept;       // realtime-safe, retargets smoothers
//   process(...) noexcept;                                 // realtime-safe, no alloc/locks
```

The sample-domain classes — `PluckExciter`, `WaveguideString`, `DamperJunction`, `BridgeJunction`, `SympatheticResonatorBus`, and `StringNetwork` (plus the `IBridgePort` seam and `StringTapBuffers` view they share) — are `template <typename SampleT>`, with explicit instantiations for `float` and `double` compiled into `cnpg_dsp` (each header carries the matching `extern template` declarations). Rationale: the realtime path is locked to the `float` instantiation, but the tier-2 `[energy]` tests run the `double` instantiation — the 1e-9 per-block non-increase bound on the Lyapunov storage function (lossless configuration) is only justifiable in float64 arithmetic, since float32 rounding alone produces relative energy fluctuations orders of magnitude above that bound. Templating yields both precisions from a single source of truth for the physics; block-domain modules are not templated and use the `Sample` (float) alias.

### 2.2 NoteEvent and fixed-capacity EventQueue

Note events are sample-accurate within a block and pre-assigned to a string by `NoteAllocator`. The queue is preallocated at fixed capacity, alloc-free, and consumed in non-decreasing `sampleOffset` order by `StringNetwork::process`. Push order must be non-decreasing in `sampleOffset`; violation is a caller bug (assert in debug builds).

```cpp
enum class NoteEventType : std::uint8_t {
    NoteOn,     // velocity, pluckPosition, hardness valid; retrigger semantics per RetriggerMode
    NoteOff     // engages damper with felt time constant (unless deferred by sustain upstream)
};

struct NoteEvent {
    NoteEventType type;
    std::int32_t  sampleOffset;   // 0..numSamples-1, offset within the current block
    std::uint8_t  stringIndex;    // 0..kMaxStrings-1, assigned by NoteAllocator
    std::uint8_t  channel;        // carried opaquely through P2; MPE seam for P5
    std::uint8_t  midiNote;       // kMinMidiNote..kMaxMidiNote
    float         velocity;       // 0..1; scales amplitude, adds mild hardness increase
    float         pluckPosition;  // 0..1 fraction of string length; latched at note-on
    float         hardness;       // 0..1 exciter hardness at note-on
};

template <std::size_t Capacity>
class EventQueue {
public:
    bool push(const NoteEvent& e) noexcept;      // false if full (event dropped, counted); never allocates
    const NoteEvent* peek() const noexcept;      // earliest remaining event, nullptr if empty
    void pop() noexcept;                          // precondition: !empty()
    void clear() noexcept;
    std::size_t size() const noexcept;
    bool empty() const noexcept;
    std::uint32_t droppedCount() const noexcept;  // diagnostics; reset by clear()
    static constexpr std::size_t capacity() noexcept;
};

using BlockEventQueue = EventQueue<256>;          // the queue type passed into StringNetwork::process
```

### 2.3 PluckExciter

One-shot pluck/pick excitation only (P0-P2). `trigger()` latches position and hardness at note-on (exciter position is not modulatable while ringing); velocity maps to amplitude plus a mild hardness increase, and a small noise-burst component is mixed in. `renderSample()` is called from inside the StringNetwork per-sample loop; output is injected into the string rails at the latched position.

```cpp
struct PluckExciterParams {
    float defaultPosition;   // 0..1, used when the note event carries no explicit position
    float defaultHardness;   // 0..1
    float noiseAmount;       // 0..1, small noise-burst component mixed into the pluck shape
};

template <typename SampleT>
class PluckExciter {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const PluckExciterParams& p) noexcept;

    // Starts a one-shot excitation; position/hardness latched for the life of the burst.
    void trigger(float velocity, float position01, float hardness01) noexcept;

    SampleT renderSample() noexcept;     // next excitation sample; 0 after the burst completes
    float  latchedPosition01() const noexcept;   // injection point for StringNetwork
    bool   isActive() const noexcept;
};

extern template class PluckExciter<float>;   // realtime path
extern template class PluckExciter<double>;  // tier-2 [energy] tests
```

### 2.4 WaveguideString

Dual-rail bidirectional digital waveguide; loss and dispersion are consolidated into termination filters (frequency-dependent loop loss filter + dispersion allpass chain). Fractional delay is Lagrange or Thiran, selected in P1 and recorded in-repo. Material (steel/nylon) is a morph of `StringMaterialParams`. Tuning uses analytic group-delay compensation at f0 in P1; P2 swaps in a calibration table generated by `cnpg_calibrate` from the actual filter implementations. f0 and bend retune continuously and click-free (per-sample smoothed internally). Called only from inside the StringNetwork per-sample loop. All moving-tap and moving-junction crossfades are amplitude-complementary linear (g1 + g2 = 1): adjacent taps a fraction of a sample apart are strongly correlated, so an equal-power sine/cosine law would add a +3 dB overlap bump and comb coloration.

```cpp
enum class FractionalDelayKind : std::uint8_t { Lagrange3, Thiran1 };  // P1 spike decides; fixed at prepare

struct StringMaterialParams {           // material = preset/morph of loss + dispersion
    float lossGainLow;                  // loop loss at low frequencies, 0..1
    float lossGainHigh;                 // loop loss at high frequencies, 0..1
    float dispersionAmount;             // 0..1 scaling of allpass-chain coefficients
};

struct WaveguideStringParams {
    float f0Hz;                         // target fundamental (MIDI 21..108 mapped upstream)
    float bendSemitones;                // continuous retune contribution; click-free under constant modulation
    StringMaterialParams material;
};

template <typename SampleT>
class WaveguideString {
public:
    void prepare(double sampleRate, int maxBlockSize, FractionalDelayKind kind);  // sizes rails for MIDI 21 at
                                                                                  // max(hostSampleRate, kMaxDesignRateHz)
    void reset() noexcept;
    void setParams(const WaveguideStringParams& p) noexcept;   // per-sample smoothing owned here

    // f0 compensation hook. Exactly one source is active:
    //   P1: analytic group-delay correction (samples) computed from filter formulas.
    //   P2: per-note cents-correction table measured from the real dsp/ filters by cnpg_calibrate.
    void setAnalyticTuningCompensation(float delaySamplesCorrection) noexcept;
    void loadCalibrationTable(const float* centsByMidiNote, int firstMidiNote, int count); // message thread

    // Rail access for the per-sample loop:
    void    injectAt(float position01, SampleT excitation) noexcept; // adds into both rails at position
    SampleT readTapAt(float position01) const noexcept;              // fractional tap; amplitude-complementary
                                                                     // linear crossfade (g1+g2=1) for
                                                                     // click-free position motion (P2)
    // 2-port insertion seam for DamperJunction at position p (moving-junction crossfade internal,
    // amplitude-complementary linear, g1+g2=1):
    void readJunctionInputs(float position01, SampleT& fromNut, SampleT& fromBridge) const noexcept;
    void writeJunctionOutputs(float position01, SampleT toBridge, SampleT toNut) noexcept;

    // Bridge port coupling (power-normalized wave variables):
    SampleT railOutgoingAtBridge() const noexcept;    // incident wave presented to the bridge port
    void    railAcceptFromBridge(SampleT reflected) noexcept;
    float  portImpedance() const noexcept;            // reference impedance for power normalization

    void  tick() noexcept;                            // advance rails + termination filters one sample
    void  setLossBypassed(bool bypass) noexcept;      // tier-2 energy test: all intentional losses lossless
    float currentF0Hz() const noexcept;
};

extern template class WaveguideString<float>;   // realtime path
extern template class WaveguideString<double>;  // tier-2 [energy] tests
```

### 2.5 DamperJunction

Variable-loss, strictly linear 2-port scattering junction at position p on the string; `|S| <= 1` for every parameter value (tier-1 energy test target). Position is continuously modulatable while a note rings (crossfade machinery — amplitude-complementary linear, g1 + g2 = 1 — lives in `WaveguideString`'s junction seam; the junction itself is memoryless apart from its loss smoother). Note-off calls `engage()`, ramping loss in with the felt time constant (20-100 ms); `release()` ramps it out. At `engagement == 0` the junction is a bit-exact pass-through (`toBridge == fromNut`, `toNut == fromBridge`).

```cpp
struct DamperJunctionParams {
    float position01;          // junction position on the string; continuously modulatable
    float maxLoss;             // 0..1 loss depth when fully engaged
    float feltTimeConstantMs;  // 20..100 ms engage/release ramp
};

template <typename SampleT>
class DamperJunction {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const DamperJunctionParams& p) noexcept;   // per-sample smoothed internally

    void engage() noexcept;    // note-off: ramp engagement -> 1 with felt time constant
    void release() noexcept;   // ramp engagement -> 0
    void setEngagementImmediate(float engagement01) noexcept; // Synth retrigger / reset paths

    // Linear passive 2-port scatter: incident (fromNut, fromBridge) -> (toBridge, toNut).
    // At engagement 0 this is bit-exact pass-through: toBridge == fromNut, toNut == fromBridge.
    void scatter(SampleT fromNut, SampleT fromBridge, SampleT& toBridge, SampleT& toNut) noexcept;

    float currentEngagement() const noexcept;
    float currentPosition01() const noexcept;
    // Tier-1 test hook: writes the instantaneous 2x2 scattering matrix, losses bypassed.
    void copyScatteringMatrix(double* rowMajorS2x2) const;
};

extern template class DamperJunction<float>;   // realtime path
extern template class DamperJunction<double>;  // tier-2 [energy] tests
```

### 2.6 IBridgePort, BridgeJunction, SympatheticResonatorBus

`IBridgePort` is the single port-level seam shared by the primary bidirectional bridge and the fallback bus — StringNetwork holds exactly one `IBridgePort&` and cannot tell them apart. Wave variables are power-normalized against per-port impedances supplied at prepare. `BridgeJunction` is an N-port scattering junction, passive by construction, loaded (P2) with a tunable 2nd-order admittance whose coefficients are positive-real-constrained at set time. `SympatheticResonatorBus` implements the identical interface unidirectionally: outgoing waves are pure passive terminations and string energy drives internal resonators (built only if the P2 passivity timebox triggers the fallback). All three are templated on the shared `SampleT` (the port seam must match the `StringNetwork` instantiation that holds it), and the three types ship in separate headers: `IBridgePort.h` (also carrying the trivial passive reflective termination), `BridgeJunction.h`, `SympatheticResonatorBus.h`. `couplingStrength` scales the FULL load admittance — conductance and susceptance alike: at 0 the strings are fully decoupled AND `bridgeOutput()` is identically 0 (the P3 body feed therefore requires `couplingStrength > 0`).

```cpp
template <typename SampleT>
class IBridgePort {
public:
    virtual ~IBridgePort() = default;
    // portImpedances: one reference impedance per port (per string), length numPorts.
    virtual void prepare(double sampleRate, int maxBlockSize,
                         int numPorts, const float* portImpedances) = 0;
    virtual void reset() noexcept = 0;
    // Per-sample exchange: one incident wave in, one outgoing wave out, per port.
    virtual void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept = 0;
    virtual SampleT bridgeOutput() const noexcept = 0; // mono bridge signal for pickup/body chain;
                                                       // == 0 when couplingStrength == 0
    virtual void setLossBypassed(bool bypass) noexcept = 0;  // tier-2 energy test support
};

struct BridgeAdmittanceParams {          // tunable positive-real 2nd-order load (P2)
    float resonanceHz;                   // load resonance
    float damping;                       // >= 0; positive-real constraint enforced at set time
    float couplingStrength;              // 0..1; scales the FULL load admittance (conductance AND
                                         // susceptance): 0 = strings fully decoupled AND
                                         // bridgeOutput() == 0 (P3 body feed requires > 0)
};

template <typename SampleT>
class BridgeJunction final : public IBridgePort<SampleT> {
public:
    void prepare(double sampleRate, int maxBlockSize,
                 int numPorts, const float* portImpedances) override;
    void reset() noexcept override;
    void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept override;
    SampleT bridgeOutput() const noexcept override;
    void setLossBypassed(bool bypass) noexcept override;

    void setAdmittance(const BridgeAdmittanceParams& p) noexcept;  // clamps into positive-real region
    // Tier-1 test hook: instantaneous NxN scattering matrix with losses bypassed;
    // spectral norm must satisfy ||S||_2 <= 1 + 1e-12.
    void copyScatteringMatrix(double* rowMajorS, int maxPorts) const;
};

extern template class BridgeJunction<float>;   // realtime path
extern template class BridgeJunction<double>;  // tier-2 [energy] tests

template <typename SampleT>
class SympatheticResonatorBus final : public IBridgePort<SampleT> {
public:
    void prepare(double sampleRate, int maxBlockSize,
                 int numPorts, const float* portImpedances) override;
    void reset() noexcept override;
    void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept override;
    SampleT bridgeOutput() const noexcept override;
    void setLossBypassed(bool bypass) noexcept override;

    void setAdmittance(const BridgeAdmittanceParams& p) noexcept;  // same tuning surface as BridgeJunction
};

extern template class SympatheticResonatorBus<float>;   // realtime path
extern template class SympatheticResonatorBus<double>;  // tier-2 [energy] tests
```

### 2.7 StringNetwork

The sample-domain physics core: owns up to `kMaxStrings` `WaveguideString`s, `DamperJunction`s, and `PluckExciter`s plus one `IBridgePort&`, and runs the entire bidirectional section as a per-sample loop (no inter-module block delay). State is laid out structure-of-arrays across strings for SIMD-friendly access. `process()` consumes the sample-accurate event queue, applies retrigger semantics, and fills per-block per-string tap buffers (smoothed fractional pickup taps computed per-sample) plus the bridge output buffer — the domain boundary artifacts consumed by the block domain.

```cpp
enum class RetriggerMode : std::uint8_t {
    Physical,   // same pitch: pluck over ringing state; new pitch: damper choke -> retune ramp -> re-excite
    Synth       // fast fade, full state reset, instant re-init at new pitch
};

struct StringNetworkParams {
    RetriggerMode retriggerMode;
    float pitchBendSemitones;          // global bend, +/-2; click-free under constant modulation (P1)
    float pickupPosition01;            // tap position; continuously modulatable while ringing (P2)
    float damperPosition01;            // junction position; continuously modulatable while ringing (P2)
    StringMaterialParams material;     // one global shared physics set (P2 parameter surface)
    DamperJunctionParams damper;       // shared damper behavior (position01 mirrored above)
    BridgeAdmittanceParams bridge;
    PluckExciterParams exciter;
    struct PerString {                 // small per-string block (P2)
        float tuningOffsetCents;
        bool  enabled;                 // mute/enable
    };
    std::array<PerString, kMaxStrings> perString;
};

// View over the per-block per-string tap buffers filled by StringNetwork::process.
// Storage is SoA: one contiguous SampleT run per string, owned by StringNetwork.
template <typename SampleT>
struct StringTapBuffers {
    const SampleT* channel(int stringIndex) const noexcept; // numSamples() contiguous samples
    bool isActive(int stringIndex) const noexcept;           // string enabled and ringing this block
    int numStrings() const noexcept;
    int numSamples() const noexcept;
};

template <typename SampleT>
class StringNetwork {
public:
    // Preallocates for kMaxStrings strings at MIDI 21, sized against
    // max(hostSampleRate, kMaxDesignRateHz), regardless of active count — prepare may
    // allocate, so a 192 kHz best-effort host never under-allocates delay rails.
    void prepare(double sampleRate, int maxBlockSize, FractionalDelayKind kind);
    void reset() noexcept;
    // 1..8; no allocation (capacity preallocated). Count REDUCTION routes through the
    // per-string enable-ramp: the removed string ramps silent first, then the count
    // actually drops on a later block. Count increase is immediate (new string starts silent).
    void setNumStrings(int numStrings) noexcept;
    int  numStrings() const noexcept;
    void setParams(const StringNetworkParams& p) noexcept;
    void setBridgePort(IBridgePort<SampleT>& port) noexcept;  // message thread; BridgeJunction or fallback bus

    // Per-block entry point. Consumes (pops) events at their sample offsets inside the
    // per-sample loop; fills internal tap buffers and the bridge output buffer.
    void process(BlockEventQueue& events, int numSamples) noexcept;

    const StringTapBuffers<SampleT>& tapBuffers() const noexcept;  // valid until next process() call
    const SampleT* bridgeOutputBuffer() const noexcept;       // numSamples of bridge signal (body/pickup feed)

    // P4 feedback bus seam (declared now, implemented P4): power-amp output re-excites the
    // strings; the block delay is physically the speaker-to-string air path.
    void injectFeedback(const SampleT* buffer, int numSamples, float airDelayMs, float gain) noexcept;

    // Energy-test hooks (tiers 2 and 3):
    void setLosslessTestMode(bool lossless) noexcept;         // forwards to strings/dampers/bridge
    // Discrete Lyapunov storage function (Bilbao NSS-style), NOT rail energy alone:
    // impedance-weighted rail energy PLUS the closed-form quadratic storage of every
    // state-bearing element — dispersion allpass states, loss-filter states, fractional-delay
    // interpolator states, and bridge admittance biquad states. Tier-2 [energy] tests assert
    // the 1e-9 per-block non-increase (lossless config) on the double instantiation.
    Sample64 energyEstimate() const noexcept;
};

extern template struct StringTapBuffers<float>;
extern template struct StringTapBuffers<double>;
extern template class StringNetwork<float>;   // realtime path
extern template class StringNetwork<double>;  // tier-2 [energy] tests
```

### 2.8 PickupTap

Block-domain consumer of the domain-boundary tap buffers: sums the active strings' tap channels into mono, then applies a single linear RLC resonance biquad (loaded pickup model — R, L, C mapped to resonant frequency and Q). Magnetic nonlinearity is deferred to P4 and does not appear here. Coefficients update per-block with smoothing owned by the module.

```cpp
struct PickupTapParams {
    float resonanceHz;      // RLC resonant frequency
    float q;                // resonance Q (loading)
    float outputGainDb;     // post-sum trim toward the -18 dBFS per-string nominal structure
};

class PickupTap {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const PickupTapParams& p) noexcept;
    // Sums taps.channel(i) for all active strings, applies the RLC biquad, writes mono out.
    // Block domain is not templated: it consumes the float (realtime) instantiation.
    void process(const StringTapBuffers<Sample>& taps, Sample* out, int numSamples) noexcept;
};
```

### 2.9 TriodeStage

Koren-equation static waveshaper of a fixed classic ECC83 input stage (100k plate load, bypassed cathode bias, 1M next-stage load, soft grid-current clamp) with published Koren ECC83 parameters hardcoded via `publishedEcc83()`. User-facing P1 parameters: drive and output trim, plus bypass (raw-string audition). Runs inside the `Oversampler` wrapper. Required caveat (user-mandated): as a static waveshaper it has no bias drift — the P1 chain is explicitly not a voicing reference for drive feel; that judgment is deferred until a dynamic stage lands (P3+). The offline table-loading contract and supply/heater THD hooks are declared now, implemented P3+ (no Python/LTspice in P0-P2).

```cpp
struct KorenTriodeParams {     // single authoritative definition; published ECC83 values hardcoded
    double mu;                 // amplification factor
    double ex;                 // Koren exponent
    double kg1;                // grid-1 constant
    double kp;                 // knee parameter
    double kvb;                // knee volt-boost
    double rgi;                // grid-current onset resistance (soft grid clamp)
};

struct TriodeStageParams {
    float drive;               // input gain into the waveshaper, calibrated against +16 dB summing headroom
    float outputTrimDb;        // post-stage trim
    bool  bypass;              // triode-bypass switch: audition the raw string (P1 monitoring chain)
};

class TriodeStage {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const TriodeStageParams& p) noexcept;
    static KorenTriodeParams publishedEcc83() noexcept;   // the pinned published parameter set

    // Static waveshaping; caller wraps this in Oversampler::processWrapped at the chosen factor.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

    // Offline table-loading contract (signature locked now; tables produced by a C++ headless
    // in-repo tool when tools/ activates in P3 — never Python/LTspice in P0-P2).
    struct TransferTableView {
        const float* values;   // uniformly sampled transfer curve
        int   size;            // number of samples, >= 2
        float inputMin;        // domain lower bound (volts at grid)
        float inputMax;        // domain upper bound
    };
    bool loadTransferTable(const TransferTableView& table);  // message thread; validates domain; false on reject

    // REQUIRED deferred hooks (contract methods, P3+ implementation): supply-voltage and
    // heater-dependent THD behavior. Through P2 these store state and have no audible effect.
    void setSupplyVoltage(float plateSupplyVolts) noexcept;   // P3+: shifts operating point / THD
    void setHeaterVoltage(float heaterVolts) noexcept;        // P3+: emission-dependent THD contract
};
```

### 2.10 Oversampler

Reusable fixed-factor oversampling wrapper (ships P1) around nonlinear stages only. Filters are IIR halfband cascades, so the factor is constrained to {2, 4, 8} only (default 2), asserted/clamped in `prepare` and fixed thereafter; total latency is queryable for host reporting. The wrapped callback runs at `factor * sampleRate` on a preallocated buffer. The P1 ADAA-vs-oversampling spike compares against this wrapper and records its decision in-repo.

```cpp
class Oversampler {
public:
    void prepare(double sampleRate, int maxBlockSize, int factor);  // factor in {2, 4, 8} only;
                                                                    // asserted/clamped here (kMaxOversampling = 8)
    void reset() noexcept;
    int  factor() const noexcept;
    int  latencySamples() const noexcept;      // at base rate; reported to host

    // Wraps one nonlinear stage: upsample -> fn(buffer, numUpsampled) -> downsample.
    // fn must be alloc/lock-free; buffer capacity maxBlockSize*factor is preallocated.
    template <typename NonlinearFn>            // callable: void(Sample* buf, int numUpsampled)
    void processWrapped(const Sample* in, Sample* out, int numSamples, NonlinearFn&& fn) noexcept;

    // Split API for chains needing separate up/down legs:
    int  upsample(const Sample* in, int numSamples, Sample* upBuffer) noexcept;   // returns numUpsampled
    void downsample(const Sample* upBuffer, int numUpsampled, Sample* out) noexcept;
};
```

### 2.11 Monitoring-chain helpers: CabFilter, SoftClipLimiter, OutputGain

The P1 monitoring chain after the triode: a bypassable fixed 2nd-order ~5 kHz lowpass as a crude speaker stand-in (no tone stack before P4), then output gain, then the soft-clip safety limiter hard-wired LAST (`CabFilter -> OutputGain -> SoftClipLimiter`) so ceiling-based acceptance (rendered peak <= ceilingDb) is enforceable. All three are trivial block modules following the unified contract; gain changes ramp per-block. Each class ships in its own header (`CabFilter.h`, `SoftClipLimiter.h`, `OutputGain.h`) with matching sources in `dsp/src/`; MonitoringChain.h/.cpp do not exist.

```cpp
struct CabFilterParams { bool bypass; };       // cutoff is fixed (~5 kHz, 2nd order) by design

class CabFilter {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const CabFilterParams& p) noexcept;
    void process(const Sample* in, Sample* out, int numSamples) noexcept;
};

struct SoftClipLimiterParams { float ceilingDb; };   // safety ceiling; soft-knee shape fixed

class SoftClipLimiter {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const SoftClipLimiterParams& p) noexcept;
    void process(const Sample* in, Sample* out, int numSamples) noexcept;
};

struct OutputGainParams { float gainDb; };

class OutputGain {
public:
    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;
    void setParams(const OutputGainParams& p) noexcept;   // per-block smoothed ramp
    void process(const Sample* in, Sample* out, int numSamples) noexcept;
};
```

### 2.12 NoteAllocator

Maps raw host MIDI to string-assigned `NoteEvent`s (strings are monophonic). Two modes in P2: guitar-emulation fingering-logic assignment and user-defined free zones per string. CC64 sustain (P2) is handled here by deferring NoteOff emission while the pedal is down. Stealing invalidates the displaced note's (string, note) ownership: a later NoteOff — including a CC64-held NoteOff — for a note that no longer owns its string is dropped and engages no damper (NoteOn A -> steal via NoteOn B on the same string -> NoteOff A produces nothing; NoteOff B engages the damper). The channel field is carried opaquely end-to-end so P5 MPE can switch assignment to channel=string without touching `StringNetwork` — this seam is the only MPE provision in P0-P2.

```cpp
struct RawMidiEvent {              // minimal host-MIDI carrier; built by the plugin layer
    std::int32_t sampleOffset;     // 0..numSamples-1
    std::uint8_t status;           // note on/off, CC (CC64 consumed here)
    std::uint8_t data1;
    std::uint8_t data2;
    std::uint8_t channel;          // opaque through P2; MPE key in P5
};

enum class AllocationMode : std::uint8_t {
    GuitarFingering,   // emulation fingering logic over the configured open-string tuning
    FreeZones          // user-defined note zone per string
};

struct StringZone { std::uint8_t lowNote; std::uint8_t highNote; };  // inclusive, FreeZones mode

struct NoteAllocatorParams {
    AllocationMode mode;
    std::array<std::uint8_t, kMaxStrings> openStringMidiNote;  // GuitarFingering tuning reference
    std::array<StringZone, kMaxStrings>   zones;               // FreeZones assignment table
};

class NoteAllocator {
public:
    void prepare(int numStrings);            // 1..kMaxStrings; fixed-capacity internal state
    void reset() noexcept;
    void setParams(const NoteAllocatorParams& p) noexcept;

    // Consumes one block of raw MIDI, pushes string-assigned NoteEvents (sample-accurate,
    // non-decreasing offsets) into outEvents. CC64 down: NoteOffs are held; CC64 up: held
    // NoteOffs are emitted at the pedal-release sample offset. Stale NoteOffs — the note no
    // longer owns its string after a steal — are dropped, whether direct or CC64-held.
    void allocate(const RawMidiEvent* events, int numEvents, BlockEventQueue& outEvents) noexcept;

    int  stringForNote(std::uint8_t channel, std::uint8_t midiNote) const noexcept;  // -1 if unassigned
    bool sustainActive() const noexcept;
};
```

### 2.13 ModuleGraph

Block-domain node graph, drafted now at interface level; through P2 the audio chain is hard-wired (`PickupTap -> Oversampler(TriodeStage, bypassable) -> CabFilter (bypassable) -> OutputGain -> SoftClipLimiter` — the safety clip is LAST so ceiling-based acceptance, rendered peak <= ceilingDb, is enforceable) and `ModuleGraph` is exercised only by contract tests. Free routing, the body node, transformer/power-amp nodes, and the feedback-bus edge arrive P4. Graph mutation is message-thread-only and must be followed by `freeze()`; `process()` is realtime-safe over the frozen topology. Cycles are rejected — the P4 feedback path is an explicit delay-bearing node, not a graph cycle.

```cpp
class IBlockModule {               // adapter interface block modules implement to join the graph
public:
    virtual ~IBlockModule() = default;
    virtual void prepare(double sampleRate, int maxBlockSize) = 0;
    virtual void reset() noexcept = 0;
    virtual void process(const Sample* in, Sample* out, int numSamples) noexcept = 0;
    virtual int  latencySamples() const noexcept = 0;
};

class ModuleGraph {
public:
    using NodeId = int;                              // stable for the life of the graph; -1 invalid

    void prepare(double sampleRate, int maxBlockSize);
    void reset() noexcept;

    NodeId addNode(IBlockModule& module);            // message thread; before freeze()
    bool   connect(NodeId from, NodeId to);          // message thread; false on cycle or invalid id
    void   setInputNode(NodeId node);
    void   setOutputNode(NodeId node);
    void   freeze();                                  // topological order + preallocated buffer plan

    // Realtime-safe over the frozen topology; mono in/out through P4's routing expansion.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;
    int  totalLatencySamples() const noexcept;        // summed along the frozen input->output path
    bool isFrozen() const noexcept;
};
```

### 2.14 MidiTranslation

`MidiTranslation.h` keeps the entire MIDI-interpretation path JUCE-free and headless-testable: a pure, stateless translation from raw (status, data1, data2, sampleOffset) tuples into the dsp-side carriers — `RawMidiEvent` streams, over which `NoteAllocator::allocate` then produces string-assigned `NoteEvent` streams — all tested headlessly in `cnpg_tests`. The plugin-side `plugin/src/MidiConverter.h/.cpp` is a thin `juce::MidiBuffer` -> tuples adapter with no logic of its own.

```cpp
// Pure function: one raw host-MIDI tuple in, one RawMidiEvent out; the channel field is
// derived from the status byte's low nibble. Stateless, allocation-free, JUCE-free.
RawMidiEvent translateRawMidi(std::uint8_t status, std::uint8_t data1, std::uint8_t data2,
                              std::int32_t sampleOffset) noexcept;

} // namespace cnpg::dsp
```

# 3a. Task breakdown — P0 and P1

Conventions for every task below: all commands run from the repo root on the Windows 11 dev machine unless a task states otherwise; the CMake build directory is `build` (MSVC multi-config, configured via `cmake --preset windows-msvc-release`, `--config Release`); Catch2 tags are mirrored to CTest labels via `catch_discover_tests`, so `ctest -L <tag>` selects by tag. Interface names refer exactly to the drafts in section 2. Phase gates: P0 ends at Task P0.8; P1 ends at Task P1.11.

## Task P0.1: Repository scaffold, license, and formatting baseline

**Files (create):** `LICENSE`, `README.md`, `.gitignore`, `.gitattributes`, `.clang-format`, `docs/plan.md` (the locked plan document), `dsp/README.md`, `plugin/README.md`, `tools/README.md` (stub: states tools/ activates in P3 with BOTH the Python modal-extraction pipeline AND the C++ headless triode-table generator; never Python in P0-P2), `tests/README.md`, `docs/README.md`, `docs/decisions/README.md`
**Depends on:** —
**Steps:**
1. Create the public GitHub repository `cecinestpasunguitar` under the user's account; default branch `main`.
2. Add the full GPLv3 text as `LICENSE`; `README.md` states product working title, vendor `Hyung Ju Park`, license, and the two-domain architecture in three sentences.
3. Add `.clang-format` (LLVM base, 4-space indent, 120-column limit, pointer-left) and `.gitignore` covering `build/`, MSVC artifacts, and `renders/`.
4. Create the locked top-level layout: `dsp/`, `plugin/`, `tools/`, `tests/`, `docs/`, `.github/workflows/`, each with a one-line README stating its purpose.
**Acceptance criteria:**
- [ ] Repository is public on GitHub and clonable anonymously.
- [ ] `LICENSE` byte-for-byte matches the canonical GPLv3 text.
- [ ] `clang-format --dry-run --Werror` over all tracked C++ files exits 0 (vacuously at this point; the command is wired into CI in P0.7).
- [ ] All six locked top-level directories exist in the initial commit.
**Verification:** `git clone https://github.com/<account>/cecinestpasunguitar` from a clean directory; `git ls-files` shows the layout; `clang-format --version` >= 17 and the dry-run command exits 0.

## Task P0.2: CMake skeleton and dependency pins

**Files (create):** `CMakeLists.txt`, `dsp/CMakeLists.txt`, `plugin/CMakeLists.txt`, `tests/CMakeLists.txt`, `cmake/Dependencies.cmake`, `cmake/CompilerWarnings.cmake`, `cmake/PluginvalPin.cmake`, `CMakePresets.json`
**Depends on:** P0.1
**Steps:**
1. Top-level `CMakeLists.txt`: C++20 (`CMAKE_CXX_STANDARD 20`, required), project name `cecinestpasunguitar`, option `CNPG_BUILD_PLUGIN` (default ON) so the ubuntu CI job can configure dsp+tests only; set `CMAKE_RUNTIME_OUTPUT_DIRECTORY` so every executable lands in `build/bin/<Config>/`; include `cmake/CompilerWarnings.cmake` (shared warning flags for all cnpg targets) and `cmake/PluginvalPin.cmake` (pinned pluginval version consumed by P0.6 and CI).
2. `cmake/Dependencies.cmake`: FetchContent for JUCE 8 pinned by exact annotated release `GIT_TAG` (with `GIT_SHALLOW TRUE`) to the latest stable 8.x release at execution time (GPL path, no splash) and Catch2 v3 pinned by exact annotated release `GIT_TAG` (with `GIT_SHALLOW TRUE`) to the latest v3.x release; record both tags and their resolved commit SHAs in comments with pin date (the SHAs are consumed by `.github/scripts/verify-pins.cmake` in P0.7; never combine SHA pins with shallow clones — the pins stay tags).
3. Guard JUCE fetch behind `CNPG_BUILD_PLUGIN`; Catch2 is always fetched (dsp tests need it headless).
4. Declare empty-for-now subdirectory wiring for `cnpg_dsp`, `cnpg_plugin`, `cnpg_tests` (populated in P0.3/P0.4).
5. `CMakePresets.json` with presets `windows-msvc-release`, `windows-msvc-debug`, and `linux-dsp-only` (sets `CNPG_BUILD_PLUGIN=OFF`, binary dir `build-dsp`); all configure commands go through presets — never raw `-G` invocations.
**Acceptance criteria:**
- [ ] `cmake --preset windows-msvc-release` configures without error on MSVC.
- [ ] `cmake --preset linux-dsp-only` configures without downloading JUCE (verify no JUCE directory under `build-dsp/_deps`).
- [ ] Both dependency pins are exact annotated release tags (not branches) with `GIT_SHALLOW TRUE`, and their commit SHAs are recorded in comments.
**Verification:** run both preset configure commands; inspect `build-dsp/_deps` listing; `git grep GIT_TAG cmake/Dependencies.cmake` shows two pinned tags.

## Task P0.3: cnpg_dsp static library, first module, first [contract] test

**Files (create):** `dsp/include/cnpg/dsp/Common.h` (Sample/Sample64 aliases, `kMaxStrings`, `kMinMidiNote`, `kMaxMidiNote`, `kMaxDesignRateHz`, `kMaxOversampling`), `dsp/include/cnpg/dsp/OutputGain.h`, `dsp/src/OutputGain.cpp`, `tests/dsp/OutputGainTests.cpp`; **modify:** `dsp/CMakeLists.txt`, `tests/CMakeLists.txt`
**Depends on:** P0.2
**Steps:**
1. Implement `OutputGain` exactly as drafted (`prepare`/`reset`/`setParams(const OutputGainParams&)`/`process`), with a per-block smoothed gain ramp; namespace `cnpg::dsp`; zero JUCE includes anywhere under `dsp/`.
2. Build `cnpg_dsp` as a static library from `dsp/src`, public include dir `dsp/include`.
3. Build `cnpg_tests` (Catch2 v3 + CTest) linking only `cnpg_dsp`; wire `catch_discover_tests` with tag-to-label mapping so `ctest -L contract` works.
4. Write `[contract]` tests: unity gain passes a buffer bit-exactly after smoothing settles; a 12 dB step reaches target within one block without exceeding it; `reset()` clears ramp state; `process` handles `numSamples` from 1 to `maxBlockSize`.
**Acceptance criteria:**
- [ ] `cnpg_dsp` compiles with zero JUCE dependencies (`git grep -l juce dsp/` returns nothing).
- [ ] `ctest --test-dir build -C Release -L contract` runs >= 4 assertions-bearing test cases, all pass.
- [ ] The same tests build and pass under the `linux-dsp-only` preset (`CNPG_BUILD_PLUGIN=OFF`).
**Verification:** `cmake --build build --config Release --target cnpg_tests`; `ctest --test-dir build -C Release -L contract --output-on-failure`; repeat in `build-dsp`.

## Task P0.4: cnpg_plugin shell — identity, frozen buses, state-version scaffold, sine tone

**Files (create):** `plugin/src/PluginProcessor.h`, `plugin/src/PluginProcessor.cpp`; **modify:** `plugin/CMakeLists.txt`
**Depends on:** P0.3
**Steps:**
1. `juce_add_plugin` target `cnpg_plugin`, `FORMATS VST3 Standalone`, `PRODUCT_NAME "cecinestpasunguitar"`, `COMPANY_NAME "Hyung Ju Park"`, `PLUGIN_MANUFACTURER_CODE Hjpk`, `PLUGIN_CODE Cnpg`, `IS_SYNTH TRUE`, `NEEDS_MIDI_INPUT TRUE`; pin these codes permanently in a comment.
2. Bus layout, frozen from P0: one stereo output bus; one stereo sidechain input bus declared `disabled by default`; `isBusesLayoutSupported` accepts main out == stereo AND sidechain in {`AudioChannelSet::disabled()`, stereo}; everything else rejected. The processor overrides `supportsDoublePrecisionProcessing()` to return `false` (locked: no VST3 double path in v1).
3. `processBlock` renders a continuous 440 Hz sine at -18 dBFS peak, duplicated mono to both output channels.
4. State scaffold: `getStateInformation`/`setStateInformation` write/read an XML root carrying integer attribute `cnpgStateVersion = 1`; unknown versions are rejected to defaults (full APVTS state arrives in P1.1).
5. Use `juce::GenericAudioProcessorEditor` (generic DAW editor through P2).
**Acceptance criteria:**
- [ ] VST3 builds: `build\bin\Release\VST3\cecinestpasunguitar.vst3` exists.
- [ ] `supportsDoublePrecisionProcessing()` returns `false`.
- [ ] Plugin reports 1 stereo output bus enabled and 1 sidechain input bus disabled at instantiation (asserted via pluginval log in P0.6).
- [ ] Saved state blob contains `cnpgStateVersion="1"` (dump state in the Standalone build and inspect).
**Verification:** `cmake --build build --config Release --target cnpg_plugin`; file-existence check on the `.vst3`; state inspection via Standalone save/load in P0.5.

## Task P0.5: Standalone target run check

**Files (modify):** none (target created in P0.4)
**Depends on:** P0.4
**Steps:**
1. Build and launch the Standalone artefact; select the machine's default audio device at 48 kHz.
2. Confirm the sine tone is audible and the app survives device sample-rate switches 44.1/48/96 kHz.
**Acceptance criteria:**
- [ ] `build\bin\Release\Standalone\cecinestpasunguitar.exe` launches, emits the 440 Hz sine, and closes cleanly at all three sample rates.
**Verification:** manual: run the exe, cycle sample rates in its audio settings panel, listen, close; no crash dialog, exit code 0.

## Task P0.6: pluginval gate

**Files (create):** `docs/decisions/0001-pluginval-baseline.md` (records pluginval version and command line)
**Depends on:** P0.4
**Steps:**
1. Install pluginval (the pinned version recorded in `cmake/PluginvalPin.cmake`) on the dev machine; record version.
2. Run at high strictness against the built VST3; fix any failures (bus negotiation, state round-trip, threading) before closing the task.
**Acceptance criteria:**
- [ ] `pluginval --strictness-level 10 --validate build\bin\Release\VST3\cecinestpasunguitar.vst3` exits 0.
**Verification:** the exact command above; exit code 0; log archived as CI artifact from P0.7 onward.

## Task P0.7: GitHub Actions CI — windows and ubuntu jobs green

**Files (create):** `.github/workflows/ci.yml`, `.github/scripts/check-golden-commit.sh`, `.github/scripts/verify-pins.cmake`
**Depends on:** P0.3, P0.4, P0.6
**Steps:**
1. `windows` job: checkout, configure via `cmake --preset windows-msvc-release`, `cmake --build build --config Release`, `ctest --test-dir build -C Release --output-on-failure`, download the pluginval version pinned in `cmake/PluginvalPin.cmake`, run the P0.6 command, upload pluginval log artifact.
2. `ubuntu` job: configure via `cmake --preset linux-dsp-only`, build the headless target list `cnpg_dsp`, `cnpg_tests`, `cnpg_bench`, `cnpg_render`, `cnpg_calibrate` (each as it lands), run full ctest — dsp/ portability guard, no JUCE.
3. Add a `clang-format --dry-run --Werror` step (windows job) over `dsp/ plugin/ tests/`.
4. CI reports performance numbers when `cnpg_bench` lands (P1.10) but NEVER gates on them; encode this as a non-failing step from the start.
5. Add a pin-immutability step running `.github/scripts/verify-pins.cmake`: compares `git ls-remote` tag SHAs against the SHAs recorded as comments in `cmake/Dependencies.cmake`, failing on drift.
6. Add `.github/scripts/check-golden-commit.sh` as a step failing any commit that touches `tests/data/golden/**` without a git trailer line `Regenerate-Goldens: <reason>` (goldens land in P1.4; the guard is wired from P0).
**Acceptance criteria:**
- [ ] Both jobs green on `main` for the P0 head commit.
- [ ] ubuntu job log contains no JUCE fetch or compile lines.
- [ ] pluginval log uploaded as a build artifact.
**Verification:** GitHub Actions UI: both jobs green; inspect ubuntu log for absence of `juce`; download artifact.

## Task P0.8: Manual acceptance — Ableton Live load check

**Files:** none
**Depends on:** P0.5, P0.6
**Steps:**
1. Copy the VST3 to the system VST3 folder; rescan plugins in Ableton Live on Windows 11.
2. Insert `cecinestpasunguitar` on a MIDI track; open the generic editor; play the set.
**Acceptance criteria:**
- [ ] Live's browser lists the plugin under vendor `Hyung Ju Park`; insertion succeeds; track meter shows the -18 dBFS sine on both channels; no crash across save/reload of the Live set; plugin state (version attribute) survives set reload.
**Verification:** named manual check "P0 Live smoke": perform steps in Ableton Live, Windows 11; record pass/fail and Live version in `docs/decisions/0001-pluginval-baseline.md` appendix.

## Task P1.1: APVTS plumbing, parameter snapshot structs, FTZ/DAZ guard

**Files (create):** `plugin/src/Parameters.h`, `plugin/src/Parameters.cpp` (APVTS layout + snapshot), `dsp/include/cnpg/dsp/ScopedFtzDazGuard.h`, `tests/dsp/ScopedFtzDazGuardTests.cpp`; **modify:** `plugin/src/PluginProcessor.h/.cpp`
**Depends on:** P0.8
**Steps:**
1. Create the APVTS as the single parameter source; P1 surface: exciter (`defaultPosition`, `defaultHardness`, `noiseAmount`), material (`lossGainLow`, `lossGainHigh`, `dispersionAmount`), pickup (`resonanceHz`, `q`, `outputGainDb`, `pickupPosition01`), triode (`drive`, `outputTrimDb`, `bypass`), `CabFilterParams::bypass`, `SoftClipLimiterParams::ceilingDb`, `OutputGainParams::gainDb`, retriggerMode.
2. Implement a once-per-block snapshot function reading APVTS atomics into the plain structs `PluckExciterParams`, `StringNetworkParams`, `PickupTapParams`, `TriodeStageParams`, `CabFilterParams`, `SoftClipLimiterParams`, `OutputGainParams`; modules own all smoothing.
3. Implement `ScopedFtzDazGuard` (JUCE-free RAII, saves/sets/restores MXCSR FTZ+DAZ bits); instantiate it first in `processBlock`.
4. Embed `cnpgStateVersion` in APVTS-backed saved state (replaces the P0 scaffold blob).
**Acceptance criteria:**
- [ ] `[contract]` test: guard sets FTZ and DAZ bits inside scope and restores prior MXCSR on exit (read MXCSR directly).
- [ ] No-alloc assurance per the locked plugin-testability design: the snapshot is a trivial atomic-read adapter; every dsp param struct carries `static_assert(std::is_trivially_copyable_v<...>)`, and all testable MIDI/parameter translation logic lives JUCE-free in dsp (`MidiTranslation.h`, tested headlessly in `cnpg_tests` from P1.2); no test links JUCE.
- [ ] pluginval at strictness 10 still exits 0 (state round-trip with APVTS).
- [ ] Saved Live set from P0.8 loads without error (version handled).
**Verification:** `ctest --test-dir build -C Release -L contract`; P0.6 pluginval command; manual reload of the P0 Live set.

## Task P1.2: NoteEvent, EventQueue, MIDI translation

**Files (create):** `dsp/include/cnpg/dsp/EventQueue.h` (NoteEvent, NoteEventType, `EventQueue`, `BlockEventQueue`), `dsp/include/cnpg/dsp/MidiTranslation.h` (pure JUCE-free translation of raw MIDI tuples into NoteEvent/`RawMidiEvent` streams), `dsp/include/cnpg/dsp/NoteAllocator.h`, `dsp/src/NoteAllocator.cpp`, `plugin/src/MidiConverter.h/.cpp` (thin `juce::MidiBuffer` -> tuples adapter), `tests/dsp/EventQueueTests.cpp`, `tests/dsp/MidiTranslationTests.cpp`, `tests/dsp/NoteAllocatorTests.cpp`
**Depends on:** P1.1
**Steps:**
1. Implement `EventQueue<Capacity>` exactly as drafted: fixed-capacity ring, alloc-free `push`/`peek`/`pop`, `droppedCount`, debug assert on decreasing `sampleOffset` push order.
2. Implement `MidiTranslation.h` in dsp: a pure function building NoteEvent/`RawMidiEvent` streams from raw (status, data1, data2, sampleOffset) tuples, tested headlessly in `cnpg_tests`; `plugin/src/MidiConverter.h/.cpp` is the thin `juce::MidiBuffer` -> tuples adapter, preserving sample offsets and channel.
3. Implement `NoteAllocator` at P1 scope: `prepare(1)`, all NoteOns assigned `stringIndex = 0`, monophonic last-note priority, NoteOffs matched to the sounding note; `channel` carried opaquely; CC64 ignored until P2 (`sustainActive()` returns false); `AllocationMode` values accepted but full GuitarFingering/FreeZones logic is P2 work.
4. `[contract]` tests: FIFO ordering, capacity-256 overflow increments `droppedCount` without corruption, offsets non-decreasing out of `allocate`, velocity mapping 0..1, notes outside `kMinMidiNote..kMaxMidiNote` rejected.
**Acceptance criteria:**
- [ ] All `[contract]` tests above pass headless on windows and ubuntu CI jobs.
- [ ] `allocate` performs no allocation (counting-new test).
- [ ] A 300-event block into `BlockEventQueue` yields exactly 256 queued + 44 dropped, `droppedCount() == 44`.
**Verification:** `ctest --test-dir build -C Release -L contract --output-on-failure` on both CI jobs.

## Task P1.3: PluckExciter

**Files (create):** `dsp/include/cnpg/dsp/PluckExciter.h`, `dsp/src/PluckExciter.cpp`, `tests/dsp/PluckExciterTests.cpp`
**Depends on:** P1.2
**Steps:**
1. Implement `PluckExciter` per the draft: `trigger()` latches position/hardness for the burst lifetime; velocity scales amplitude and adds a mild hardness increase; small noise-burst component scaled by `noiseAmount`; `renderSample()` returns 0 after completion; `latchedPosition01()`/`isActive()` as specified.
2. Shape: short raised-cosine displacement burst whose spectral tilt follows hardness; burst length bounded (< 10 ms at 96 kHz) so preallocation is trivial.
3. `[contract]` tests: latching (changing `PluckExciterParams` mid-burst does not move position), determinism with `noiseAmount = 0`, silence and `isActive() == false` after burst end, peak amplitude monotone in velocity.
**Acceptance criteria:**
- [ ] All `[contract]` tests pass; burst peak at velocity 1.0, hardness 0.5 is within ±1 dB of the level calibrated for -18 dBFS per-string nominal downstream.
- [ ] `renderSample()` measured alloc-free and NaN-free over 10^6 triggers with random parameters.
**Verification:** `ctest --test-dir build -C Release -L contract`.

## Task P1.4: WaveguideString — single string, fractional delay decision, termination filters, analytic tuning, IR baselines

**Files (create):** `dsp/include/cnpg/dsp/WaveguideString.h`, `dsp/src/WaveguideString.cpp`, `tests/dsp/WaveguideStringTuningTests.cpp`, `tests/dsp/WaveguideStringRegressionTests.cpp`, `tests/data/golden/string_ir/{lagrange3,thiran1}/<rate>/<name>.f64` + `.json` sidecars (scenario-first layout per section 4.3; goldens per fractional-delay variant per rate 44.1/48/96 kHz, float64), `docs/decisions/0002-fractional-delay.md`
**Depends on:** P1.3
**Steps:**
1. Implement the dual-rail bidirectional waveguide with loss and dispersion consolidated at terminations: one-pole-plus-zero frequency-dependent loop loss filter parameterized by `lossGainLow`/`lossGainHigh`; dispersion allpass chain (4 cascaded first-order allpasses) scaled by `dispersionAmount`; rails sized in `prepare` for MIDI 21 against `max(hostSampleRate, kMaxDesignRateHz)`; the class is `template <typename SampleT>` with explicit `float` and `double` instantiations (realtime path float; tier-2 energy tests run double).
2. Implement both `FractionalDelayKind::Lagrange3` and `Thiran1`; run a 2-working-day comparison (tuning error across MIDI 21-108, decay-tail smoothness under continuous retune, cost per sample); record the chosen default with data tables in `docs/decisions/0002-fractional-delay.md`; the loser remains compiled and golden-covered.
3. Implement `setAnalyticTuningCompensation`: group-delay of loss filter + dispersion chain + interpolator at f0, computed from filter formulas, subtracted from the delay-line length; `loadCalibrationTable` stores the table and is a no-op source until P2 selects it.
4. Implement `injectAt`, `readTapAt`, `readJunctionInputs`/`writeJunctionOutputs` (seam only; DamperJunction is P2), bridge-port methods (`railOutgoingAtBridge`, `railAcceptFromBridge`, `portImpedance`), `tick`, `setLossBypassed`, `currentF0Hz`; f0/bend retune per-sample smoothed.
5. Build the `[tuning]` test with the canonical estimator: render 7 s, discard the first 0.5 s, analyze 2^18 samples (2^19 at 96 kHz) with a Blackman-Harris window, x4 zero-padding, parabolic peak interpolation, search restricted to ±80 cents around nominal; include the mandatory estimator self-calibration case (synthetic known-f0 tones proving estimator error << 0.5 cents); convert to cents against target; sweep MIDI 21-108 at 44.1/48/96 kHz.
6. Build the `[regression]` layer: (a) feature invariants — first 8 partial frequencies within ±2 cents of measured baseline, per-octave-band T60 within ±10%, RMS within recorded bounds; (b) float64 goldens (atol 1e-7) per `FractionalDelayKind` per sample rate; add CMake target `cnpg_regen_goldens` that regenerates and overwrites goldens; commits touching `tests/data/golden/**` must carry the git trailer `Regenerate-Goldens: <reason>`, enforced by `.github/scripts/check-golden-commit.sh` (wired in P0.7).
**Acceptance criteria:**
- [ ] `[tuning]`: with analytic compensation, absolute error <= 2 cents for MIDI 33-96 at all three rates; MIDI 21-32 and 97-108 emitted as a report-only error table in the test log (the full 88-note x 3-rate ±2-cent assertion binds only the P2 calibration-table case, per the locked tuning design).
- [ ] `[regression]`: all feature invariants and all 6 golden sets (2 kinds x 3 rates) pass at atol 1e-7.
- [ ] `docs/decisions/0002-fractional-delay.md` exists, names the chosen `FractionalDelayKind`, and contains the comparison tables.
- [ ] Lossless mode (`setLossBypassed(true)`) pluck on the `double` instantiation shows per-block non-increase of the storage-function `energyEstimate()` (impedance-weighted rail energy plus closed-form quadratic storage of dispersion-allpass, loss-filter, and fractional-delay interpolator states) over 10 s within 1e-9 relative tolerance (early tier-2 check on the isolated string).
**Verification:** `ctest --test-dir build -C Release -L tuning`; `ctest --test-dir build -C Release -L regression`; `cmake --build build --config Release --target cnpg_regen_goldens` (dry check); CI `check-golden-commit.sh` trailer step green.

## Task P1.5: StringNetwork — one string, per-sample loop, tap buffers, click-free global pitch bend

**Files (create):** `dsp/include/cnpg/dsp/StringNetwork.h`, `dsp/src/StringNetwork.cpp` (includes `StringTapBuffers`, `RetriggerMode`, `StringNetworkParams`), `dsp/include/cnpg/dsp/IBridgePort.h` (interface + a trivial passive reflective termination used until P2's `BridgeJunction`), `tests/dsp/StringNetworkTests.cpp`, `tests/dsp/PitchBendClickTests.cpp`, `tests/dsp/DenormalTests.cpp`
**Depends on:** P1.4
**Steps:**
1. Implement `StringNetwork` per the draft: preallocate for `kMaxStrings` at MIDI 21 against `max(hostSampleRate, kMaxDesignRateHz)` in `prepare` regardless of active count; SoA layout across strings; P1 runs with `setNumStrings(1)`.
2. `process(BlockEventQueue&, int)`: per-sample loop consuming events at their offsets; NoteOn triggers the string's `PluckExciter` at the event's `pluckPosition`/`hardness`; NoteOff starts a fixed fast release in P1 (DamperJunction lands P2); same-note retrigger plucks over ringing state; `retriggerMode` is accepted and stored, with full Physical/Synth semantics completing in P2.
3. Compute the smoothed fractional-position pickup tap per-sample via `readTapAt`, filling the SoA tap buffers exposed by `tapBuffers()`; fill `bridgeOutputBuffer()` from the termination port.
4. Global pitch bend: MIDI pitch-wheel mapped ±2 semitones into `StringNetworkParams::pitchBendSemitones`, applied through the string's per-sample-smoothed retune path.
5. Declare `injectFeedback` (stores nothing audible; P4 seam), `setLosslessTestMode`, `energyEstimate` (discrete Lyapunov storage function: impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element, per the locked energy design) as drafted.
6. Click-free bend test (`[contract]`, the stated test): render a 5 s MIDI 40 note at 48 kHz with `pitchBendSemitones` driven by a 2 Hz full-depth ±2 st sine; assert (a) no NaN/Inf anywhere in the tap buffer, and (b) excluding the first 50 ms, the maximum absolute sample-to-sample difference during the bend never exceeds 4x the maximum observed for the same note rendered with static pitch at equal amplitude.
7. Denormal regression test (`[denormal]`, canonical): render a pluck and let the tail decay 60 s at 44.1 kHz with losses enabled inside a `ScopedFtzDazGuard`; two named cases: state-inspection (fpclassify probes plus NaN/Inf scan) runs in BOTH CI jobs; timing-ratio (median per-block time over the final 5 s <= 2x the median of the first 1 s) is LOCAL-ONLY (flaky on shared runners).
**Acceptance criteria:**
- [ ] `[contract]`: events at offsets 0, 63, 127 in a 128-sample block excite the string starting at exactly those samples (impulse-alignment check on the tap buffer).
- [ ] `[contract]` click-free bend test passes as specified in step 6.
- [ ] `[denormal]` test passes as specified in step 7 (state-inspection case in both CI jobs; timing-ratio case local-only).
- [ ] `process` verified alloc-free (counting-new test wrapping a 1000-block run).
**Verification:** `ctest --test-dir build -C Release -L contract`; `ctest -L denormal` (same `--test-dir build -C Release` form; the timing-ratio case is run locally only).

## Task P1.6: PickupTap with RLC resonance

**Files (create):** `dsp/include/cnpg/dsp/PickupTap.h`, `dsp/src/PickupTap.cpp`, `tests/dsp/PickupTapTests.cpp`
**Depends on:** P1.5
**Steps:**
1. Implement `PickupTap` per the draft: sum `taps.channel(i)` over strings where `isActive(i)`, apply one linear RLC resonance biquad (resonanceHz, q mapped to loaded-pickup pole pair), then `outputGainDb` trim toward the -18 dBFS per-string nominal structure; per-block coefficient smoothing owned here. Magnetic nonlinearity is absent by design (P4).
2. `[contract]` tests: sine sweep through the biquad matches the analytic RLC magnitude response within ±0.5 dB at 20 log-spaced frequencies; inactive strings contribute exactly zero; parameter steps produce no NaN and no sample-to-sample jump > 6 dB in a steady sine.
**Acceptance criteria:**
- [ ] All `[contract]` tests pass at 44.1/48/96 kHz.
- [ ] Resonance peak frequency measured within ±1% of `resonanceHz` for q in {0.7, 2, 6}.
**Verification:** `ctest --test-dir build -C Release -L contract`.

## Task P1.7: TriodeStage — Koren ECC83 static waveshaper with deferred hooks

**Files (create):** `dsp/include/cnpg/dsp/TriodeStage.h` (includes `KorenTriodeParams`, `TriodeStageParams`, `TransferTableView`, and the deferred hooks `setSupplyVoltage`/`setHeaterVoltage`), `dsp/src/TriodeStage.cpp`, `tests/dsp/TriodeStageTests.cpp`
**Depends on:** P1.1
**Steps:**
1. Implement the Koren-equation static waveshaper of the fixed classic ECC83 input stage: 100k plate load, bypassed cathode bias, 1M next-stage load, soft grid-current clamp via `rgi`; `publishedEcc83()` returns the published Koren ECC83 parameter set, hardcoded and cited to the Koren reference in a comment.
2. Solve the static plate curve offline-at-prepare into an internal uniform lookup with cubic interpolation; `drive` is input gain calibrated against the +16 dB summing headroom convention; `outputTrimDb` post-stage; `bypass` passes input through untouched.
3. Declare and implement to store-only: `setSupplyVoltage`, `setHeaterVoltage` (contract methods, no audible effect through P2, implemented P3+); implement `loadTransferTable` validation (domain sanity, `size >= 2`, monotone domain) with tables produced only by a future C++ headless tool — no Python/LTspice in P0-P2.
4. State the mandated caveat in the header comment and in this task's rationale: a static waveshaper has no bias drift (no coupling-cap/cathode-bypass dynamics), so the P1 chain is explicitly NOT a voicing reference for drive feel; drive-feel voicing conclusions are deferred until a dynamic stage lands (P3+).
5. `[contract]` tests: bypass is bit-exact; zero input -> DC-removed zero output; monotone THD increase across drive {0.25, 0.5, 1.0} on a 220 Hz sine; both deferred hooks callable and audibly inert (output golden-equal before/after calls).
**Acceptance criteria:**
- [ ] All `[contract]` tests pass; `publishedEcc83()` values match the cited published set exactly (test asserts the literals).
- [ ] Header contains the deferred THD hook signatures and the not-a-voicing-reference caveat verbatim.
- [ ] `loadTransferTable` rejects malformed tables (returns false) in tests.
**Verification:** `ctest --test-dir build -C Release -L contract`; `git grep "NOT a voicing reference" dsp/include/cnpg/dsp/TriodeStage.h` returns the caveat line.

## Task P1.8: Oversampler, [aliasing] gate, ADAA spike

**Files (create):** `dsp/include/cnpg/dsp/Oversampler.h`, `dsp/src/Oversampler.cpp`, `tests/dsp/AliasingGateTests.cpp`, `docs/decisions/0003-adaa-vs-oversampling.md`
**Depends on:** P1.7
**Steps:**
1. Implement `Oversampler` per the draft: factor fixed at `prepare`, restricted to {2, 4, 8} (assert/clamp in `prepare`; `kMaxOversampling` stays 8), default 2; IIR halfband cascades for up/down legs; `latencySamples()` at base rate reported to the host by the plugin layer; `processWrapped` and the split `upsample`/`downsample` API; all buffers preallocated at `maxBlockSize * factor`.
2. Build the `[aliasing]` gate exactly as locked: fixed sines 1244.5 Hz and 4186 Hz, at nominal and max drive through `Oversampler`-wrapped `TriodeStage`, 48 kHz host rate; FFT with Blackman-Harris window, >= 2^18 points; classify bins as harmonic vs folded; PASS = worst folded component <= -60 dBc at the default factor (2x); run and report (non-gating) the same measurement at factors 4 and 8.
3. ADAA spike, timeboxed to 3 working days: implement first-order ADAA on the same Koren transfer in a test-only branch of the harness; compare against 2x/4x oversampling on aliasing (same gate metric), CPU (per-sample cost), and step-response smearing; record the decision, tables, and rationale in `docs/decisions/0003-adaa-vs-oversampling.md`; the shipped path is whatever that document concludes, wired through the same `Oversampler` seam.
4. If the -60 dBc gate fails at the 2x default, apply the locked remedy hierarchy: (1) adopt ADAA from the spike if it passes at equal CPU; (2) steepen the halfband transition/order at 2x; (3) soften the grid-clamp knee (voicing-neutral per the static-stage caveat); (4) escalate the default factor — the outcome recorded in `docs/decisions/0003-adaa-vs-oversampling.md`. A red gate converts to a recorded decision, not a stall.
**Acceptance criteria:**
- [ ] `[aliasing]` gate passes: worst folded component <= -60 dBc at factor 2 for both test frequencies at both drive settings; factors 4 and 8 results printed in the test log.
- [ ] `latencySamples()` equals the measured impulse delay through `processWrapped` with an identity nonlinearity (test asserts equality).
- [ ] `docs/decisions/0003-adaa-vs-oversampling.md` exists with measurement tables and an explicit chosen approach; spike branch merged or deleted within the 3-day timebox.
**Verification:** `ctest --test-dir build -C Release -L aliasing --output-on-failure`; file check on `docs/decisions/0003-adaa-vs-oversampling.md`; latency `[contract]` test via `ctest -L contract`.

## Task P1.9: Monitoring chain helpers, gain staging, full chain integration

**Files (create):** `dsp/include/cnpg/dsp/CabFilter.h`, `dsp/src/CabFilter.cpp`, `dsp/include/cnpg/dsp/SoftClipLimiter.h`, `dsp/src/SoftClipLimiter.cpp`, `dsp/include/cnpg/dsp/ModuleGraph.h`, `dsp/src/ModuleGraph.cpp`, `tests/dsp/MonitoringChainTests.cpp`, `tests/dsp/ModuleGraphTests.cpp`; **modify:** `plugin/src/PluginProcessor.cpp` (wire the hard-wired P1 chain), `plugin/src/Parameters.cpp`
**Depends on:** P1.5, P1.6, P1.8
**Steps:**
1. Implement `CabFilter` (bypassable fixed 2nd-order ~5 kHz lowpass; cutoff fixed by design, no tone stack before P4) and `SoftClipLimiter` (fixed soft-knee shape, `ceilingDb` parameter) per the drafts; `OutputGain` exists from P0.3.
2. Wire the locked hard-wired chain in `processBlock`: `NoteAllocator::allocate` -> `StringNetwork::process` -> `PickupTap` -> `Oversampler::processWrapped(TriodeStage)` (bypassable) -> `CabFilter` (bypassable) -> `OutputGain` -> `SoftClipLimiter` (safety clip LAST so ceiling-based acceptance is enforceable) -> duplicated mono to the stereo bus; `ScopedFtzDazGuard` first; APVTS snapshot once per block; triode `bypass` auditions the raw string; report `Oversampler::latencySamples()` to the host.
3. Calibrate gain staging: single string at velocity 1.0 peaks at -18 dBFS at the pickup sum; triode `drive` range calibrated against the ~+16 dB summing headroom convention (documented in a header comment with the measured mapping).
4. `[contract]` tests: CabFilter -3 dB point within ±10% of its fixed design cutoff; SoftClipLimiter output never exceeds `ceilingDb` + 0.1 dB for +12 dB overshoot input; full-chain NaN-free 60 s random-parameter sweep at 44.1/48/96 kHz.
5. Implement `ModuleGraph` (`dsp/include/cnpg/dsp/ModuleGraph.h` + `dsp/src/ModuleGraph.cpp`) with `tests/dsp/ModuleGraphTests.cpp` covering the freeze-discipline battery: cycle rejection, freeze, latency summation, and chain-equivalence vs direct calls.
**Acceptance criteria:**
- [ ] All `[contract]` tests pass; measured single-string peak = -18 dBFS ± 1 dB at the pickup output (automated test).
- [ ] `ModuleGraphTests` freeze-discipline battery passes: cycle rejection, freeze, latency summation, chain-equivalence vs direct calls.
- [ ] pluginval strictness 10 exits 0 with the full chain and reported latency.
- [ ] Manual check "P1 playability smoke" in Ableton Live: plugin sounds a plucked-string note per MIDI note-on, pitch tracks the keyboard, pitch-bend wheel bends ±2 st without clicks, triode bypass switch audibly changes timbre.
**Verification:** `ctest --test-dir build -C Release -L contract`; P0.6 pluginval command; named manual check in Ableton Live, Windows 11.

## Task P1.10: cnpg_bench — headless benchmark lands with first regression numbers

**Files (create):** `tests/bench/BenchMain.cpp`, `tests/bench/CMakeLists.txt` (target `cnpg_bench`), `docs/bench/p1-baseline.md`; **modify:** `.github/workflows/ci.yml` (non-gating bench step)
**Depends on:** P1.9
**Steps:**
1. Build `cnpg_bench`: headless, links `cnpg_dsp` only; constructs the full dsp chain (StringNetwork + PickupTap + Oversampler(TriodeStage) + CabFilter + OutputGain + SoftClipLimiter), plays a 60 s deterministic corpus-derived stream, measures median, p99, and max per-block processing time, converted to CPU% via blockTime/(B/R); CLI: `cnpg_bench [--config <name> | --strings N] --samplerate R --blocksize B --oversample F --seconds S` (named configs arrive in P2).
2. Record first regression numbers for 1 string (P1 configuration) and, since capacity is preallocated, also run `--strings 6` and `--strings 8` and report; commit all numbers with machine spec to `docs/bench/p1-baseline.md`. The 25-30% hard gate binds the 6-string default configuration at the P2 milestone; from P1 the numbers exist for regression visibility (user-mandated).
3. Add the CI step running `cnpg_bench` and publishing its output to the job log and an artifact — reported, never gating.
**Acceptance criteria:**
- [ ] `build\bin\Release\cnpg_bench.exe --strings 1 --samplerate 48000 --blocksize 128 --oversample 2 --seconds 60` prints median, p99, and max block time as CPU% and exits 0.
- [ ] `docs/bench/p1-baseline.md` committed with 1/6/8-string numbers at 48 kHz / 128 samples and dev-machine spec.
- [ ] CI shows the bench step output on both a passing and a deliberately slow debug run without changing job status (non-gating confirmed).
**Verification:** run the exact bench command above; inspect CI artifact; `git log --stat` shows the baseline doc commit.

## Task P1.11: cnpg_render, initial MIDI corpus, P1 listening pass — playable mono instrument gate

**Files (create):** `tests/render/RenderMain.cpp`, `tests/render/MidiFileReader.h/.cpp`, `tests/render/WavWriter.h/.cpp`, `tests/render/CMakeLists.txt` (target `cnpg_render`), `tests/corpus/` (flat, versioned MIDI phrases: `01_chromatic_singles.mid`; `03_legato_retrigger.mid` — default retrigger mode only; `05_low_string_bends.mid` — aggressive ±2 st bends on the lowest playable note; `07_param_sweeps_midnote.mid` + `.json` — automation of drive/pickup resonance mid-note; 02/04/06/08 plus the per-mode variant renders of 03 join the corpus in P2 when CC64 and DamperJunction land), `tests/corpus/corpus.json` (manifest), `tests/corpus/README.md` (corpus versioning rules), `docs/listening/physical-plausibility-checklist.md`, `docs/listening/P1-<yyyymmdd>.md`
**Depends on:** P1.10
**Steps:**
1. Build `cnpg_render`: headless MIDI-to-WAV renderer over the same dsp chain as `cnpg_bench`; flags `--midi <file> --out <wav> --samplerate R --blocksize B`; deterministic output for fixed inputs.
2. Author the P1 corpus files above (01, 03 in default retrigger mode only, 05, 07 with its `.json` automation sidecar) plus the `corpus.json` manifest; corpus is versioned in-repo and only grows (README states append-only rule and the P2 additions 02/04/06/08 plus per-mode variants of 03).
3. Write the physical-plausibility checklist (attack character, decay realism, pitch stability under bend, absence of clicks/zipper noise, triode drive character within the stated static-waveshaper caveat).
4. Render the full corpus at 48 kHz; author performs the listening pass against the checklist; record item-by-item pass/fail with notes in `docs/listening/P1-<yyyymmdd>.md`.
5. Perform the P1 milestone manual gate in Ableton Live, Windows 11: play the instrument live from a MIDI keyboard for 15 minutes covering the corpus gestures by hand; the instrument must be playable as a mono plucked-string instrument with working bend, velocity response, and triode bypass audition.
**Acceptance criteria:**
- [ ] `build\bin\Release\cnpg_render.exe --midi tests\corpus\05_low_string_bends.mid --out renders\05_low_string_bends.wav --samplerate 48000 --blocksize 128` exits 0 and two consecutive runs produce bit-identical WAVs.
- [ ] All four P1 corpus files render NaN-free and clip-free (peak <= `SoftClipLimiterParams::ceilingDb`) — automated post-render scan wired as a `[contract]` test invoking `cnpg_render`.
- [ ] Every checklist item in `docs/listening/P1-<yyyymmdd>.md` is marked pass, or has a filed GitHub issue linked inline; zero unexplained items.
- [ ] Named manual check "P1 playable instrument gate" in Ableton Live passes: note-on latency subjectively immediate at 128-sample buffer, no clicks under continuous bend wheel motion, no stuck notes across 15 minutes of playing.
**Verification:** the exact `cnpg_render` command above run twice plus `fc /b` on the two WAVs; `ctest --test-dir build -C Release -L contract`; manual gate performed in Ableton Live on Windows 11 and recorded in `docs/listening/P1-<yyyymmdd>.md`; CI green on the P1 head commit.

## 3b. Task breakdown — P2 (physics core complete)

P2 turns the P1 single-string vertical slice into the full sample-domain physics core: 1..8 strings with per-string damper junctions, bidirectional bridge coupling behind `IBridgePort`, allocation and retrigger modes, CC64 sustain, click-free moving damper/pickup positions, measured tuning calibration, and the P2 voicing gate. Everything below references the fixed interface drafts in section 2 by exact name.

Conventions used by every Verification method in this section:

- Build directory is `build/`, configured via the P0 preset: `cmake --preset windows-msvc-release` (never raw `-G` invocations). All executable targets emit to `build\bin\Release\` (P0 CMake convention); the VST3 bundle emits to `build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3`.
- Catch2 tag filters run against the test binary directly, e.g. `build\bin\Release\cnpg_tests.exe "[energy]"`. The ubuntu CI job runs the identical dsp/ tests via `ctest --test-dir build-dsp --output-on-failure` (the `linux-dsp-only` preset's binary dir is `build-dsp`).
- Golden regeneration uses the regenerate-goldens target established in P1 (`cmake --build build --config Release --target cnpg_regen_goldens`); every commit touching `tests/data/golden/` must carry a git trailer line `Regenerate-Goldens: <reason>` (enforced by `.github/scripts/check-golden-commit.sh`) stating which algorithm change invalidated which goldens.
- Manual acceptance is always phrased against Ableton Live on the Windows 11 dev machine.

Dependency order: P2.1 → P2.2 → P2.3 → P2.4 (P2.5 is a protocol shadowing P2.4) → P2.6 → P2.7 → P2.8 → P2.9. P2.6 and P2.7 may run as parallel workstreams once P2.4 lands; P2.5's timebox clock starts the day P2.4's energy suite first runs red.

---

### P2.1 — StringNetwork scale-out: N = 1..8 strings, SoA layout, per-string parameters, bench configs

**Files:**
- `dsp/include/cnpg/dsp/StringNetwork.h` (implementation state only; drafted signatures unchanged)
- `dsp/src/StringNetwork.cpp`
- `dsp/src/WaveguideString.cpp` (per-string `tuningOffsetCents` applied inside f0 smoothing)
- `plugin/src/Parameters.cpp` (APVTS: `numStrings`, per-string `tuningOffsetCents` and `enabled` for 8 slots)
- `plugin/src/PluginProcessor.cpp` (snapshot into `StringNetworkParams::perString`)
- `tests/dsp/StringNetworkScaleTests.cpp`
- `tests/bench/BenchMain.cpp` (add configs `p2_default6`, `p2_max8`)

**Depends on:** P1 complete (all P1 tasks and the P1 exit gate); interface drafts 2.4, 2.7.

**Steps:**
Replace the P1 single-string ownership inside `StringNetwork` with structure-of-arrays storage across `kMaxStrings` strings: delay-rail sample storage, termination-filter state, smoother state, and tap accumulators are each laid out as one contiguous array per field spanning all strings (not an array of per-string structs), so the per-sample loop iterates field-wise and is SIMD-friendly. `prepare()` preallocates all 8 strings at MIDI 21 against `max(hostSampleRate, kMaxDesignRateHz)` regardless of active count; `setNumStrings()` never allocates: a count increase takes effect immediately (the new string starts silent), while a count reduction routes through the per-string enable-ramp — the removed string ramps silent first, then leaves the loop trip count on a later block. Apply `StringNetworkParams::perString[i].tuningOffsetCents` as an additive cents term inside each `WaveguideString`'s per-sample f0 smoothing (same smoother as bend, so offset changes are click-free), and implement `perString[i].enabled` as: disabled strings are skipped by the loop, report `StringTapBuffers::isActive() == false`, and present a zero incident wave at their bridge port slot. Extend `StringTapBuffers` filling to all active strings (SoA channel runs). Wire the APVTS additions and bump the state-version integer to 2 (it stays 2 through all of P2). Add the two named bench configurations to `cnpg_bench` as named `--config` values extending the P1.10 CLI (`cnpg_bench [--config <name> | --strings N] --samplerate R --blocksize B --oversample F --seconds S`): `p2_default6` (6 strings + full P1 monitoring chain at default 2x oversampling) and `p2_max8` (8 strings, same chain). Define the click metric here, once, for all of P2 (later tasks reference this definition): click metric = max over 10 ms windows of (peak |first difference| / whole-render median |first difference|), compared against a reference render without the state change; the criterion is that the metric stays within 3 dB of the reference, with no NaN and no denormal-guard trips.

**Acceptance criteria:**
- [ ] `setNumStrings(n)` for every n in 1..8: `cnpg_tests` contract test triggers a note per string and observes n active tap channels and silence on disabled ones; no allocation after `prepare()` (verified by the P1 allocation-guard hook in debug builds).
- [ ] Per-string `tuningOffsetCents = +25` on one string of a unison pair produces a measured f0 ratio within ±2 cents of 25 cents (FFT phase method, 48 kHz).
- [ ] Toggling `enabled` on a ringing string ramps it silent with no click (click metric and criterion as defined in this task's steps — the canonical P2 definition).
- [ ] `setNumStrings` reduction `[contract]` while ringing: the removed string ramps silent through its enable-ramp before the trip count drops on a later block; the click metric criterion holds across the count change; a count increase starts the new string silent immediately.
- [ ] Saved state round-trips with state-version 2; loading a version-1 (P1) state does not crash (values default; no compatibility guarantee, per plan).
- [ ] `cnpg_bench --config p2_default6` and `--config p2_max8` both run and print median, p99, and max block time converted to percent-of-one-core at 48 kHz / 128-sample blocks (numbers recorded, not yet gated — gate is P2.9).

**Verification method:**
```
cmake --build build --config Release --target cnpg_tests cnpg_bench
build\bin\Release\cnpg_tests.exe "[contract]"
build\bin\Release\cnpg_bench.exe --config p2_default6 --samplerate 48000 --blocksize 128 --seconds 60
build\bin\Release\cnpg_bench.exe --config p2_max8   --samplerate 48000 --blocksize 128 --seconds 60
```
Manual: in Ableton Live, load the VST3, set numStrings 6, play 6-note chords from a MIDI clip; every voice sounds; disable strings 5–6 and confirm those notes fall silent. `pluginval.exe --strictness-level 10 --validate "build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3"` passes.

---

### P2.2 — DamperJunction: linear passive 2-port at position p, felt note-off, node-suppression CI test

**Files:**
- `dsp/include/cnpg/dsp/DamperJunction.h` (bodies for drafted signatures)
- `dsp/src/DamperJunction.cpp`
- `dsp/src/WaveguideString.cpp` (static-position `readJunctionInputs` / `writeJunctionOutputs` seam; motion arrives in P2.3)
- `dsp/src/StringNetwork.cpp` (one `DamperJunction` per string; NoteOff → `engage()`; retrigger/reset paths call `setEngagementImmediate()`)
- `tests/dsp/DamperEnergyTests.cpp`
- `tests/dsp/DamperNodeSuppressionTests.cpp`
- `tests/dsp/DamperFeltTimeTests.cpp`

**Depends on:** P2.1.

**Steps:**
Implement `DamperJunction::scatter()` as a strictly linear, memoryless 2-port scattering operation whose only state is the per-sample smoothed engagement and loss values: at `engagement == 0` the junction is bit-exact transparent (`toBridge = fromNut`, `toNut = fromBridge`); at full engagement the scattering coefficients realize a resistive loss of depth `maxLoss` at the junction, constructed so the 2x2 matrix satisfies `||S||_2 <= 1` for every `(position01, maxLoss, engagement)` triple — derive coefficients from a positive junction conductance so passivity holds by construction, not by clamping. `engage()`/`release()` drive a one-pole ramp with time constant `feltTimeConstantMs` (validated into 20..100 ms). Insert the junction into the per-sample loop through the `WaveguideString` seam at a static `position01` (fractional read/write via the same interpolation kind fixed at `prepare`). Implement `copyScatteringMatrix()` for the tier-1 test. `StringNetwork` maps NoteOff events (post-allocator, so CC64 deferral in P2.6 composes upstream) to `engage()` on the target string.

**Acceptance criteria:**
- [ ] Tier-1 `[energy]`: over the canonical damper tier-1 grid defined in section 4.2 (referenced, not restated here), spectral norm of the 2x2 matrix from `copyScatteringMatrix` (losses bypassed) ≤ 1 + 1e-12.
- [ ] Transparency `[contract]`: unit test that `DamperJunction::scatter()` at engagement 0 is bit-exact pass-through (`toBridge == fromNut`, `toNut == fromBridge`). The junction stays permanently in-line — no compiled-out comparison exists, because the fractional seam interpolates even at zero engagement; instead, an engagement-0 string impulse response is compared to the no-damper reference via the layer-(a) feature invariants (first 8 partials ±2 cents, T60 ±10%) plus a stated non-zero waveform tolerance, and the P2.7 calibration table absorbs the seam's group delay.
- [ ] Node suppression `[contract]`: pluck at position 0.28, damper at position01 = 0.5, `engage()` at t = 1.0 s with maxLoss 1.0: within 250 ms of full engagement the fundamental partial drops ≥ 24 dB from its pre-engage level while the 2nd harmonic drops ≤ 6 dB; fitted T60(fundamental) ≤ 0.25 × T60(2nd harmonic).
- [ ] Felt time: NoteOff→silence envelope reaches −60 dBFS within a window consistent with `feltTimeConstantMs` at 20 ms and at 100 ms (measured decay time monotone in the parameter, factor ≥ 3 between the two settings).
- [ ] Tier-3 `[energy]` still passes with dampers engaged: monotone-decreasing block-RMS envelope of `energyEstimate()` — the discrete Lyapunov storage functional (impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element), float32 — no growth. Tier-2 runs on the `double` instantiation of the sample-domain classes.

**Verification method:**
```
cmake --build build --config Release --target cnpg_tests
build\bin\Release\cnpg_tests.exe "[energy]"
build\bin\Release\cnpg_tests.exe "DamperNodeSuppression*"
```
Manual: in Ableton Live, hold a note and release — audible felt-like stop, no click, no thump; set damper position to 0.5 in the generic editor and confirm released notes leave a faint octave-up remnant (2nd harmonic surviving).

---

### P2.3 — Moving-position machinery: click-free damper and pickup position modulation while ringing

**Files:**
- `dsp/src/WaveguideString.cpp` (dual-anchor crossfade inside `readTapAt`, `readJunctionInputs`, `writeJunctionOutputs`)
- `dsp/src/StringNetwork.cpp` (per-sample smoothing of `pickupPosition01` and `damperPosition01` from block-snapshotted targets)
- `tests/dsp/MovingPositionClickTests.cpp`
- `tests/dsp/MovingJunctionEnergyTests.cpp`

**Depends on:** P2.2.

**Steps:**
Technique (stated per mandate): dual-anchor amplitude-complementary linear crossfade (g1 + g2 = 1) over per-sample smoothed fractional positions — not equal-power sine/cosine: taps 1/32 of string length apart are strongly correlated, so an equal-power law would give a +3 dB bump and comb coloration. For the pickup tap, `readTapAt` maintains an anchor tap at the last committed position; the incoming position is per-sample smoothed, and while the smoothed value stays within 1/32 of string length of the anchor, plain fractional interpolation follows it directly. When the smoothed position departs beyond that threshold, a second tap opens at the smoothed position and an amplitude-complementary linear crossfade of 128 samples at 48 kHz (duration scaled with sample rate) transfers the output to it, after which it becomes the new anchor; crossfades re-arm immediately so continuous fast sweeps become a chain of overlapping amplitude-complementary segments. The damper junction seam uses the same dual-anchor amplitude-complementary linear law on both the read side (`readJunctionInputs`) and the write side (`writeJunctionOutputs`). Exciter position remains latched at note-on and gets no motion machinery.

**Acceptance criteria:**
- [ ] Click test `[contract]`, pickup: render a 6-string sustained chord for 10 s while `pickupPosition01` sweeps 0.05→0.95 sinusoidally at 2 Hz; the click metric defined in P2.1 is within 3 dB of the same render with position frozen at 0.5; no NaN, no denormal-guard trips.
- [ ] Click test `[contract]`, damper: same render with `damperPosition01` swept 0.1→0.9 at 2 Hz with engagement held at 0.5; same criterion.
- [ ] Combined worst case: both positions swept at 5 Hz simultaneously during a `retriggerMode = Physical` retrigger; click metric criterion holds.
- [ ] Moving-junction `[energy]`: tier-2 network impulse test on the `double` instantiation of the sample-domain classes (all losses lossless via `setLosslessTestMode(true)`) with the damper position swept 0.1→0.9 over the full render: bounded growth — cumulative `energyEstimate()` (the discrete Lyapunov storage functional: impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element) never exceeds its post-excitation maximum, and per-block growth stays within a stated tolerance derived from the crossfade overlap. The strict 1e-9 per-block non-increase criterion applies to static-position runs only.
- [ ] IR regression layer (a) feature invariants still pass at frozen positions (partials ±2 cents, per-octave-band T60 ±10%, RMS bounds); goldens regenerated only if the interpolation change altered frozen-position output, with a `Regenerate-Goldens: <reason>` trailer.

**Verification method:**
```
cmake --build build --config Release --target cnpg_tests
build\bin\Release\cnpg_tests.exe "MovingPosition*"
build\bin\Release\cnpg_tests.exe "[energy]"
build\bin\Release\cnpg_tests.exe "[regression]"
```
Manual: in Ableton Live, map a MIDI controller knob to pickup position and another to damper position; sweep both aggressively while a chord rings — no zipper noise, no clicks, audible comb/brightness change tracks the knob.

---

### P2.4 — BridgeJunction: passive N-port scattering + positive-real admittance load; three-tier energy suite; coupled-string sanity

**Files:**
- `dsp/include/cnpg/dsp/BridgeJunction.h` (bodies for drafted signatures; `IBridgePort` in `dsp/include/cnpg/dsp/IBridgePort.h`)
- `dsp/src/BridgeJunction.cpp`
- `dsp/src/StringNetwork.cpp` (per-sample gather of `railOutgoingAtBridge()` across strings → `IBridgePort::scatter` → `railAcceptFromBridge()`; `bridgeOutputBuffer` fill; `energyEstimate()` as a discrete Lyapunov storage function — impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element: dispersion allpass states, loss-filter states, fractional-delay interpolator states, bridge admittance biquad states)
- `tests/dsp/BridgeEnergyTierOneTests.cpp`
- `tests/dsp/NetworkEnergyTierTwoTests.cpp`
- `tests/dsp/NetworkEnergyTierThreeTests.cpp`
- `tests/dsp/CoupledStringsTests.cpp`
- `tests/dsp/BridgePortContractTests.cpp` (interface-level suite any `IBridgePort` must pass — written now, exercised against `BridgeJunction`)
- `tests/data/golden/` (new coupled-network goldens per algorithm variant per rate)

**Depends on:** P2.1, P2.2 (P2.3 recommended landed first so energy tests cover motion).

**Steps:**
Implement `BridgeJunction` as an N-port scattering junction in power-normalized wave variables against the per-port impedances supplied at `prepare` (from `WaveguideString::portImpedance()`). Passivity by construction: the junction is a parallel connection of the N string ports into a lumped 2nd-order bridge admittance; the scattering matrix is derived from positive port conductances plus the discretized admittance, never from free matrix entries. `setAdmittance()` maps `BridgeAdmittanceParams` (`resonanceHz`, `damping`, `couplingStrength`) to the 2nd-order load coefficients and clamps at set time into the positive-real region (damping ≥ a strictly positive floor; resonance clamped below Nyquist margin); `couplingStrength` scales the load conductance seen by the ports, controlling inter-string energy transfer and brightness. Implement `bridgeOutput()` as the load velocity/force signal (mono feed for `PickupTap` context and the future body node), `setLossBypassed()` (load made lossless for tier-2), and `copyScatteringMatrix()` for tier-1. `StringNetwork::setBridgePort()` accepts the junction; disabled strings present zero incident waves. Update IR regression: regenerate layer-(b) float64 goldens for the coupled 6-string network at 44.1/48/96 kHz with a `Regenerate-Goldens: <reason>` trailer; extend layer-(a) invariants to the coupled case.

**Acceptance criteria:**
- [ ] Tier-1 `[energy]`: over the canonical bridge tier-1 grid defined in section 4.2 (referenced, not restated here — it covers every port count 1..8, the resonance/damping sweep, and the couplingStrength sweep), `copyScatteringMatrix` with losses bypassed has spectral norm ≤ 1 + 1e-12; deliberately illegal `setAdmittance` inputs (negative damping, resonance ≥ Nyquist) are clamped and still yield a passive S.
- [ ] Tier-2 `[energy]`: 6-string network impulse, all intentional losses set lossless (`setLosslessTestMode(true)` forwarding to strings, dampers, bridge), run on the `double` instantiation of the sample-domain classes: `energyEstimate()` — the discrete Lyapunov storage functional (impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element) — non-increasing per block within 1e-9 relative tolerance.
- [ ] Tier-3 `[energy]`: losses enabled, float32: monotone-decreasing block-RMS envelope of `energyEstimate()` (the storage functional — never of the bridge output signal) over a 10 s decay, no growth at any tested admittance setting.
- [ ] Sympathetic response `[contract]`: strings 0 and 1 in unison, couplingStrength 0.5, pluck string 0 only: string 1's tap channel rises above −60 dBFS within 1 s.
- [ ] Beating `[contract]`: string 1 detuned +4 cents, both plucked, couplingStrength 0.1 (where the uncoupled beat prediction holds; at higher coupling, normal-mode splitting per Weinreich exceeds the naive prediction): bridge-output envelope shows amplitude modulation at the predicted beat rate f0·(2^(4/1200) − 1) within ±20%.
- [ ] Weinreich-style two-stage decay (qualitative check per Weinreich, JASA 1977, made testable): unison pair, pluck one string; a two-exponential fit to the Schroeder-integrated (or windowed-max over at least one beat period) bridge-output decay envelope — never raw block-RMS monotonicity on the bridge signal, which beats periodically — beats a single-exponential fit by ≥ 6 dB residual and the two fitted decay rates differ by ≥ 2×.
- [ ] IR regression: layer-(a) invariants pass on the coupled network; layer-(b) goldens regenerated at all three rates (atol ~1e-7 in float64) with a `Regenerate-Goldens: <reason>` trailer.

**Verification method:**
```
cmake --build build --config Release --target cnpg_tests
build\bin\Release\cnpg_tests.exe "[energy]"
build\bin\Release\cnpg_tests.exe "CoupledStrings*"
build\bin\Release\cnpg_tests.exe "[regression]"
cmake --build build --config Release --target cnpg_regen_goldens   (only when invalidated; `Regenerate-Goldens: <reason>` trailer required)
```
Manual: in Ableton Live, play and hold a low E, then staccato notes on other strings — audible sympathetic shimmer on the held string; raise couplingStrength and confirm brighter, more coupled behavior.

---

### P2.5 — Bridge passivity fallback protocol (timeboxed) and SympatheticResonatorBus contingency path

**Files:**
- `docs/decisions/0006-p2-bridge-passivity-fallback.md` (written in all outcomes)
- `dsp/include/cnpg/dsp/SympatheticResonatorBus.h`, `dsp/src/SympatheticResonatorBus.cpp` (implemented only if triggered)
- `tests/dsp/BridgePortContractTests.cpp` (parameterized over `IBridgePort` implementations)

**Depends on:** P2.4 in progress (this task is a protocol that shadows it; the clock starts the day P2.4's corrected `[energy]` suite — tier-2 on the `double` instantiation asserting the storage-functional `energyEstimate()`, tier-3 its float32 block-RMS envelope — first runs red).

**Steps:**
This is a design-for-fallback protocol, not speculative implementation. Timebox: 1–2 calendar weeks of focused effort on making `BridgeJunction` pass the three-tier `[energy]` suite, counted from the first red run of that suite, hard ceiling two calendar weeks. During the timebox, all fixes stay within the positive-real-by-construction framing (discretization method, normalization, coefficient mapping) — no ad-hoc energy clamps in the audio path. Exit criteria (either ends the timebox early): (pass) all P2.4 tier-1/2/3 acceptance boxes green across the full parameter grid at 44.1/48/96 kHz → record the passing construction and discretization choice in the decision doc, close this task; (fail) timebox expires with any tier still red → trigger the fallback. Fallback procedure: (1) create branch `research/bidirectional-bridge` from the failing state, preserving the failing tests and all diagnostic work; bidirectional coupling development continues only on that branch; (2) on `main`, implement `SympatheticResonatorBus` behind the identical `IBridgePort` interface — outgoing waves are pure passive terminations (per-port reflection with |r| ≤ 1) and the incident string energy drives an internal bank of resonators tuned by the same `setAdmittance` surface (`BridgeAdmittanceParams` reused verbatim so the APVTS surface is unchanged); `bridgeOutput()` sums the resonator bank; (3) `StringNetwork::setBridgePort()` is repointed at the bus — `StringNetwork` holds exactly one `IBridgePort&` and must not be able to tell the difference; (4) the bus must pass `BridgePortContractTests` and energy tiers 1–3 (trivially, being unidirectional); the two-string beating and Weinreich boxes of P2.4 are re-scoped on `main` to "sympathetic response present" (string-1 audibility box only) and the bidirectional-only boxes move to the research branch; (5) decision doc records trigger date, failing evidence, and re-entry criteria for merging the research branch back (its energy suite green).

**Acceptance criteria:**
- [ ] `docs/decisions/0006-p2-bridge-passivity-fallback.md` exists and records either the passing construction (pass path) or the full trigger record and re-scope (fail path); no TBD text.
- [ ] `BridgePortContractTests` `[contract]` passes for every `IBridgePort` implementation compiled into `cnpg_tests` (one on the pass path, two on the fail path).
- [ ] Fail path only: `SympatheticResonatorBus` passes `[energy]` tiers 1–3 and the sympathetic-response box; `research/bidirectional-bridge` branch exists and CI runs its dsp/ tests.
- [ ] The timebox was respected: repository history shows ≤ 2 calendar weeks between the first red `[energy]` run on `BridgeJunction` and either green tests or the fallback trigger commit.

**Verification method:**
```
build\bin\Release\cnpg_tests.exe "BridgePortContract*"
build\bin\Release\cnpg_tests.exe "[energy]"
git log --oneline -- docs/decisions/0006-p2-bridge-passivity-fallback.md
```
Review: decision doc read in the P2.9 exit review; on the fail path, confirm in Ableton Live that `main` still produces sympathetic shimmer via the bus.

---

### P2.6 — NoteAllocator modes, retrigger semantics, CC64 sustain

**Files:**
- `dsp/include/cnpg/dsp/NoteAllocator.h` (bodies for drafted signatures)
- `dsp/src/NoteAllocator.cpp`
- `dsp/src/StringNetwork.cpp` (`RetriggerMode` semantics at event consumption)
- `plugin/src/Parameters.cpp` (APVTS: `AllocationMode`, `openStringMidiNote[8]`, `zones[8]`, `retriggerMode`)
- `plugin/src/PluginProcessor.cpp` (build `RawMidiEvent`s from the host MIDI buffer; `NoteAllocator::allocate` → `BlockEventQueue` → `StringNetwork::process`)
- `tests/dsp/NoteAllocatorFingeringTests.cpp`
- `tests/dsp/NoteAllocatorZonesTests.cpp`
- `tests/dsp/SustainPedalTests.cpp`
- `tests/dsp/RetriggerModeTests.cpp`

**Depends on:** P2.1, P2.2 (damper needed for Physical retrigger and pedal-deferred note-offs); P2.4 for full-network renders in tests.

**Steps:**
Implement `AllocationMode::GuitarFingering` with this assignment algorithm: for an incoming NoteOn, the candidate set is every enabled string i with `openStringMidiNote[i] <= midiNote <= openStringMidiNote[i] + 24` (24-fret playable range over the configured open tuning, default EADGBE = {40, 45, 50, 55, 59, 64} for 6 strings). Among idle candidates, pick the string with the lowest fret position (`midiNote - openStringMidiNote[i]` minimal); ties break least-recently-used. If no candidate is idle, apply the steal policy: steal the candidate holding the oldest note-on (least-recently-triggered); the steal emits the new NoteOn onto that string and the ringing note is displaced under the active `RetriggerMode` (no separate NoteOff is synthesized). Implement `AllocationMode::FreeZones`: candidates are strings whose inclusive `StringZone` `[lowNote, highNote]` contains the note; overlapping zones are legal and resolved by the same idle-LRU-then-steal-LRU policy; a note contained in no zone is unassignable — it is dropped silently, an allocator-level diagnostics counter increments, and `stringForNote` returns −1. Implement CC64 in `allocate()`: CC64 value ≥ 64 is pedal-down; while down, NoteOffs are held per (string, note) instead of emitted; on pedal-up they are emitted at the pedal-release sample offset (non-decreasing offset order preserved); a new NoteOn arriving on a string with a pending held NoteOff for that note cancels the held NoteOff and retrigger semantics apply. In `StringNetwork`, implement both `RetriggerMode` states at event consumption: Physical — same pitch plucks over the ringing state; pitch change performs damper choke (fast `engage()`), a retune ramp completing within 30 ms, then re-excitation with rail state preserved (emergent legato); Synth — a ≤ 5 ms fade via `setEngagementImmediate`-assisted gain, full state reset, instant re-init at the new pitch. Stealing invalidates the displaced note's (string, note) ownership: a later NoteOff — including a CC64-held NoteOff — for a note that no longer owns its string is dropped.

**Acceptance criteria:**
- [ ] Fingering `[contract]`: with default EADGBE, E2 (40) → string 0 at fret 0; a C-major open chord (48, 52, 55, 60, 64) lands on five distinct strings at minimal fret positions; a 7th simultaneous note steals the least-recently-triggered candidate.
- [ ] Zones `[contract]`: overlapping zones alternate via LRU under repeated notes; a note outside all zones produces no `NoteEvent`, increments the diagnostics counter, and `stringForNote` returns −1.
- [ ] CC64 `[contract]`: NoteOn → CC64 down → NoteOff produces no damper engagement (tap decay remains slow); CC64 up at sample offset k emits the held NoteOff at offset k; re-striking a held-off note cancels its pending NoteOff; queue order stays non-decreasing in `sampleOffset`.
- [ ] Stale NoteOff `[contract]`: NoteOn A → steal via NoteOn B on the same string → NoteOff A produces no damper engagement (A no longer owns the string); NoteOff B engages the damper.
- [ ] Physical retrigger `[contract]`: same-pitch restrike passes the P2.1 click metric and string RMS never drops below −60 dBFS between the two attacks; cross-pitch restrike shows f0 reaching the new pitch within 30 ms with no click.
- [ ] Synth retrigger `[contract]`: within 100 ms after restrike at a new pitch, no old-pitch partial exceeds −60 dBc in the string's tap spectrum.
- [ ] All allocator paths are alloc-free after `prepare` (debug allocation guard) and drop-counted, never blocking.

**Verification method:**
```
cmake --build build --config Release --target cnpg_tests
build\bin\Release\cnpg_tests.exe "NoteAllocator*,SustainPedal*,RetriggerMode*"
```
Manual: in Ableton Live, play a strummed open-chord clip with sustain pedal held — all six strings ring past their note-offs and damp together on pedal release; toggle retriggerMode and confirm Physical gives connected same-string legato while Synth gives clean instant re-attacks.

---

### P2.7 — cnpg_calibrate: measured tuning-calibration tables, file format, runtime loading, [tuning] gate

**Files:**
- `tests/calibrate/CalibrateMain.cpp` (target `cnpg_calibrate`; links `cnpg_dsp` only — headless, JUCE-free, no Python/LTspice)
- `tests/calibrate/CMakeLists.txt`
- `dsp/data/calibration/tuning_cal_44100.csv`, `tuning_cal_48000.csv`, `tuning_cal_96000.csv` (generated, checked in, human-diffable)
- `dsp/include/cnpg/dsp/generated/TuningCalibrationData.h` (generated constexpr arrays compiled into `cnpg_dsp`)
- `dsp/src/WaveguideString.cpp` (`loadCalibrationTable` body; rate selection/interpolation)
- `plugin/src/PluginProcessor.cpp` (table load at prepare on the message thread)
- `tests/dsp/TuningAccuracyTests.cpp`

**Depends on:** P2.1 (per-string offsets must compose with calibration); P2.3/P2.4 recommended landed so the measured loop matches shipping topology.

**Steps:**
`cnpg_calibrate` measures the actual dsp/ implementation, not formulas: for each sample rate in {44100, 48000, 96000} and each MIDI note 21..108, it instantiates a single `WaveguideString` through `StringNetwork` at the default `StringMaterialParams`, triggers a `PluckExciter` pluck, renders 3.5 s, discards the first 0.25 s, and estimates f0 from the phase difference of the fundamental bin across two overlapping 2^16-sample Blackman-Harris FFT frames (hop 2^15) — sub-cent resolution. Cents error = 1200·log2(f_measured / f_target); the table entry is the negated error. The tool then applies the correction via `loadCalibrationTable` and re-measures once; the residual after this second pass must be < 0.5 cents (a claim stated for exactly these render/frame/hop sizes) or the tool exits nonzero. File format (both emitted): (a) CSV per rate — header line `# cnpg tuning calibration v1, rate=<Hz>, firstMidiNote=21, count=88`, then one `midiNote,centsCorrection` row per note; (b) the generated header containing `constexpr float` arrays (one per rate) plus rate constants, regenerated in place. Regeneration policy mirrors goldens: rerunning `cnpg_calibrate` and committing changed tables requires an explanatory commit message. Runtime loading: at `prepare`, the plugin (message thread) calls `WaveguideString::loadCalibrationTable` with the embedded array for the exact host rate if it is one of the three measured rates; for other rates, cents values are linearly interpolated between the two nearest measured rates (clamped at the ends; 192 kHz best-effort uses the 96 kHz table); the P1 analytic `setAnalyticTuningCompensation` path remains as the pre-load default so the plugin never runs uncorrected. Per-note calibration, per-string `tuningOffsetCents`, and bend compose additively in cents inside the f0 smoother.

**Acceptance criteria:**
- [ ] `cnpg_calibrate --rates 44100,48000,96000 --out dsp/data/calibration` runs headless to completion on the dev machine and in the ubuntu CI job, exit code 0, residual report < 0.5 cents for every note/rate.
- [ ] `[tuning]` gate: with tables loaded, rendered f0 is within ±2 cents of equal-temperament target for every MIDI note 21..108 at 44.1, 48, and 96 kHz (all 264 note/rate points asserted, none sampled).
- [ ] Composition: calibration + `tuningOffsetCents = +30` + bend −1.0 semitone lands within ±2 cents of the analytically combined target at MIDI 40, 69, 96 (48 kHz).
- [ ] CSV and generated header agree bit-for-bit in value; a follow-up `cnpg_calibrate` run on an unchanged tree produces zero diff (determinism).
- [ ] Table load path allocates nothing on the audio thread (load is message-thread `prepare` only; debug allocation guard clean).

**Verification method:**
```
cmake --build build --config Release --target cnpg_calibrate cnpg_tests
build\bin\Release\cnpg_calibrate.exe --rates 44100,48000,96000 --out dsp/data/calibration
git diff --stat dsp/data/calibration dsp/include/cnpg/dsp/generated
build\bin\Release\cnpg_tests.exe "[tuning]"
```
Manual: in Ableton Live at 48 kHz and again at 96 kHz, play against a reference tuner track across the range (A0, E1, A2, A4, C6, C8) — no audible detuning; ubuntu CI job runs the same `[tuning]` suite headless.

> **Amendment (author decision, 2026-08-01, ADR 0007) — this
> task's mechanism changes. Read §4.5's ADR 0007 amendment with it.** The steps above describe measuring the shipped
> implementation into a per-note table. P2.4 established that the tuning residual is *not* a per-note constant: it is a
> function of `couplingStrength`, `bridgeResonanceHz` and `bridgeDamping`, all live APVTS parameters, and it reverses
> sign across the resonance sweep. A one-dimensional note-indexed table cannot represent that.
>
> **What replaces it.** Analytic bridge phase-delay compensation, computed from the junction's admittance at each
> string's fundamental and recomputed on parameter change. Consequently:
> - the **`[tuning]` gate becomes a sweep over (note × rate × bridge-parameter grid)**, not (note × rate) — the ±2-cent
>   criterion binds across the declared normal range, not only at the default admittance;
> - the parameter-change recompute must route through **P2.3's dual-anchor crossfade** and carry its own click gate,
>   because retuning a ringing string is a delay-length change on a ringing string;
> - the solve is **self-referential** (phase delay at f0 changes the loop length changes f0) and needs a fixed-point
>   iteration with a stated convergence criterion, offline at parameter-change time;
> - `cnpg_calibrate` is retained, re-scoped from *mechanism* to **verification across the grid** plus an optional
>   residual trim. The file-format, determinism, CSV/header-agreement and no-audio-thread-allocation criteria above
>   still apply to whatever it emits.
>
**Additional acceptance criteria (author decision 2026-08-01, ADR 0007 D5/D6):**
- [ ] **`[tuning]` grid gate.** ±2 cents over the supported note range at 44.1/48/96 kHz, across the **provisional Normal range** of (`couplingStrength` × `bridgeResonanceHz` × `bridgeDamping`) — not only at the default admittance. Grid points, not samples.
- [ ] **The Normal range is derived, not declared.** It is the **largest contiguous region** of the three-parameter space in which *all five* hold at once: (1) tuning within ±2 cents across the supported note range; (2) the fixed-point solver converges reliably; (3) live parameter changes remain click-free; (4) near-unison strings **≈25 cents apart do not involuntarily mode-lock** under ordinary playing conditions; (5) the bridge still behaves as an instrument component rather than an overt resonant effect. Four are measurable here; (5) is P2.8's judgement. P2.7 derives and records a **provisional** region; **P2.8 confirms or revises it** alongside the `couplingStrength` default. Criterion (4) is what ties the two together — mode-locking is a coupling phenomenon, so the range ceiling and the default are the same measurement.
- [ ] **Extended (Effect) range declared.** Settings outside the Normal range remain available and carry **no tuning guarantee**; the boundary is declared in the ADR and enforced in whatever the UI/parameter surface reports, never merely discovered by a user.
- [ ] **Solver contract declared *and tested*:** convergence tolerance in cents (tighter than ±2 by a stated margin), maximum iteration count, and **defined fallback behaviour for non-convergent or pathological settings**. A test drives the solver into non-convergence deliberately and asserts the fallback, rather than assuming it is unreachable — the P2.4 review found exactly that assumption false about a different "unreachable" branch.
- [ ] **Dedicated live-parameter gate `[contract]`:** `couplingStrength`, `bridgeResonanceHz` and `bridgeDamping` each changed **while strings are sounding**, asserting no clicks, no discontinuities, and **no unstable pitch transitions**. All compensation changes route through **P2.3's dual-anchor crossfade machinery** — retuning a ringing string is a delay-length change on a ringing string. The pitch-stability half is not implied by the click half: a converging solver can be perfectly smooth and still audibly hunt.
- [ ] **Steep phase-slope region around bridge resonance measured early** and reported, before the grid gate is finalized — it is the risk item for this task. A **residual trim** is applied *only if* the analytic solution leaves a small systematic error there; it is a fallback, not part of the design.
- [ ] `cnpg_calibrate` retained and re-scoped to the **verification harness** over the note × bridge-parameter grid (plus the optional residual trim). Its determinism, CSV/header agreement, and no-audio-thread-allocation criteria above apply to whatever it emits.
- [ ] **`couplingStrength` is not confirmed here.** It remains provisional per ADR 0007 D4; this task must not record it as the shipping default.

---

### P2.8 — Corpus expansion with mandated abuse cases + full P2 listening pass

**Files:**
- `tests/corpus/02_open_chords.mid` (full open-string chords)
- `tests/corpus/04_palm_mute_chug.mid` (rapid palm-mute chugging: low-position damper partially engaged, fast retriggers)
- `tests/corpus/06_sustain_chords.mid` (chords held under sustain pedal, ringing past their note-offs)
- `tests/corpus/08_harmonics_nodes.mid` + `tests/corpus/08_harmonics_nodes.json` (harmonics at string node positions; the `.json` sidecar drives the render configuration)
- `tests/corpus/corpus.json` (append-only manifest, updated with the P2 entries)
- `tests/render/RenderMain.cpp` (cnpg_render: P2 parameter-automation lanes, per-corpus render configs, per-retriggerMode variant renders of `03_legato_retrigger.mid`)
- `tests/dsp/CorpusSweepTests.cpp` (automated click/NaN/denormal sweeps over corpus renders)
- `docs/listening/physical-plausibility-checklist.md` (P2 items appended; versioned)
- `docs/listening/P2-<yyyymmdd>.md`

**Depends on:** P2.1–P2.7 (renders exercise the complete P2 feature set); P2.5 outcome determines which bridge implementation is rendered on `main`.

**Steps:**
Extend the versioned MIDI corpus with exactly the four P2 additions listed above (`02_open_chords.mid`, `04_palm_mute_chug.mid`, `06_sustain_chords.mid`, `08_harmonics_nodes.mid` + `.json`), update the append-only `corpus.json` manifest, and add the per-retriggerMode variant renders of `03_legato_retrigger.mid` (render configurations, not a new MIDI file). Extend `cnpg_render` to drive P2 parameter automation (positions, coupling, material, retriggerMode switches) alongside MIDI so the sweep cases are reproducible artifacts, not manual knob rides. Add `CorpusSweepTests`, tagged `[contract]`: it renders every corpus entry headlessly through the full default chain at 44.1/48/96 kHz and asserts the CI-enforced objective proxies — the P2.1 click metric, zero NaN/Inf samples, and the state-inspection denormal case (no FP_SUBNORMAL state, no NaN/Inf) extended to 6 coupled strings with dampers; the timing-ratio denormal case per section 4.6 remains local-only, run on the dev machine. Append the P2 items to the written physical-plausibility checklist (sympathetic coupling audible and musical; palm-mute chug damps convincingly without machine-gun artifacts; bends stay in tune and click-free at the extremes; pedal-held chords ring and release together; mid-note sweeps morph without zipper noise; Physical vs Synth retrigger characters distinct). Perform the author listening pass: render the entire corpus to WAV with `cnpg_render`, audition against the checklist, and record pass/fail with notes per item in `docs/listening/P2-<yyyymmdd>.md`. Any checklist failure files an issue and blocks P2.9.

**Acceptance criteria:**
- [ ] The four new corpus entries (02, 04, 06, 08 with its `.json` sidecar) and the updated `corpus.json` manifest exist, are committed (versioned), and render headlessly to WAV at all three rates without error; the per-retriggerMode variant renders of 03 are produced.
- [ ] `CorpusSweepTests` `[contract]` passes: the P2.1 click metric within criterion on every corpus render; zero NaN/Inf.
- [ ] `[denormal]` state-inspection case passes on the 6-string coupled long-decay render (no FP_SUBNORMAL state, no NaN/Inf) in CI; the timing-ratio case (section 4.6) passes locally on the dev machine.
- [ ] Checklist updated in the same commit series as the corpus (versioned together); every P2 checklist item has a recorded pass in `docs/listening/P2-<yyyymmdd>.md`, or a linked blocking issue.
- [ ] Rendered WAVs peak below the SoftClipLimiter ceiling on default settings (gain-structure sanity, −18 dBFS per-string nominal respected).
- [ ] **`couplingStrength` sign-off (author decision 2026-08-01, ADR 0007 D4).** ADR 0006's `0.35` is **provisional**. This pass must render and compare **lower coupling values** and judge **mode-locking in near-unison voicings** by ear, then confirm or replace the default. At 0.35 two strings 25 cents apart mode-lock — both peak at 111.297 Hz for nominals 110.00/111.60, a **+20.286 cent pull** on the string nobody detuned, separation collapsing to 0.0029 cents — while beat depth falls with coupling (10.08 / 3.59 / 2.28 / 1.62 dB at 0.1 / 0.35 / 0.5 / 1.0). The setting trades sympathetic richness against pitch integrity and where that sits is a musical judgement, not a measurement.
- [ ] **Provisional *normal parameter range* confirmed or revised** for the three bridge parameters, per §4.5's ADR 0007 amendment — the range over which the ±2-cent tuning gate binds. P2.7 sets it provisionally from measured data; this is the session that judges it by ear.

**Verification method:**
```
cmake --build build --config Release --target cnpg_render cnpg_tests
build\bin\Release\cnpg_render.exe --corpus tests\corpus --rates 44100,48000,96000 --out renders\p2
build\bin\Release\cnpg_tests.exe "CorpusSweep*"
build\bin\Release\cnpg_tests.exe "[denormal]"
```
Manual: author listening pass over `renders\p2\*.wav` in Ableton Live (also replaying the corpus MIDI live through the plugin), judged against `docs/listening/physical-plausibility-checklist.md`; results committed.

---

### P2.9 — P2 exit gate: performance gate, full CI green, voicing sign-off

**Files:**
- `docs/bench/p2-exit.md`
- `.github/workflows/` (windows job additionally runs `cnpg_bench` configs and publishes numbers in the job log — reported, never gating, per plan)

**Depends on:** P2.1–P2.8 all closed (P2.5 closed on either path).

**Steps:**
Run the complete gate on the dev machine (Windows 11 Pro workstation) and record every number in `docs/bench/p2-exit.md`. Performance gate: `cnpg_bench --config p2_default6` at 48 kHz / 128-sample blocks — the full P2 default configuration (6 strings + monitoring chain at default oversampling) must measure a median CPU ≤ 30% of one core (hard gate; 25% is the aspirational target to note), and `--config p2_max8` is measured and reported (median, p99, max) and must remain realtime (< 100% of one core) — never gated in CI; the hard gate applies only to the median on the 6-string default. CI gate: windows job (VST3 build + unit tests + pluginval high strictness) and ubuntu job (JUCE-free dsp/ tests) both green on the exit commit, with all P2 suites present: `[contract]`, `[energy]` (all three tiers), `[regression]`, `[aliasing]`, `[denormal]`, `[tuning]`. Voicing sign-off: the P2.8 listening results show every checklist item passed (the P1 triode caveat stands — drive-feel voicing conclusions remain out of scope until a dynamic stage lands in P3+, and the sign-off document restates this). The exit document also records the P2.5 outcome (bridge construction or fallback), the state-version (2), and the benchmark environment (CPU model, compiler, build flags).

**Acceptance criteria:**
- [ ] `p2_default6` at 48 kHz / 128: median CPU ≤ 30% of one core on the dev machine — hard gate (25% target noted alongside the measured median, p99, and max figures).
- [ ] `p2_max8` measured and reported (median, p99, max); < 100% of one core (realtime); never gated in CI.
- [ ] Exit-commit CI: windows and ubuntu jobs green; test log shows nonzero test counts for each of `[contract]`, `[energy]`, `[regression]`, `[aliasing]`, `[denormal]`, `[tuning]`; CI publishes bench numbers without gating on them.
- [ ] `pluginval --strictness-level 10` passes on the exit-commit VST3.
- [ ] `docs/bench/p2-exit.md` committed with all measurements, the P2.5 outcome, the voicing sign-off referencing `docs/listening/P2-<yyyymmdd>.md`, and the restated P1 triode voicing caveat; no open blocking issues from P2.8.
- [ ] **This gate must NOT lock the `couplingStrength` default by passing** (ADR 0007 D4). The value is settled only by P2.8's recorded listening sign-off; if that sign-off is absent or inconclusive, the exit doc records the default as still provisional rather than treating a green board as confirmation.

**Verification method:**
```
cmake --build build --config Release
build\bin\Release\cnpg_bench.exe --config p2_default6 --samplerate 48000 --blocksize 128 --seconds 120
build\bin\Release\cnpg_bench.exe --config p2_max8   --samplerate 48000 --blocksize 128 --seconds 120
build\bin\Release\cnpg_tests.exe
pluginval.exe --strictness-level 10 --validate "build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3"
```
Manual: final acceptance session in Ableton Live on the exit build — load, play the full corpus live, toggle allocation and retrigger modes, sweep positions, hold pedal chords; git tag `p2-exit` applied to the signed-off commit; CI links and bench logs pasted into `docs/bench/p2-exit.md`.

# 4. Test strategy

All automated tests are Catch2 v3 cases in the `cnpg_tests` target, driven by CTest, and use only the JUCE-free `cnpg_dsp` library unless stated otherwise. Tags used are exactly: `[contract]`, `[energy]`, `[regression]`, `[aliasing]`, `[denormal]`, `[tuning]`. Section 4.9 maps every tag and tool to its CI job. Wherever a test needs double-precision arithmetic (energy accounting, golden comparison, FFT analysis) it uses `Sample64`; the device-under-test runs the shipping `Sample = float` path, except that the tier-2 `[energy]` cases run the `double` instantiation of the sample-domain classes (see 4.2).

## 4.1 Contract tests — `[contract]`

Every dsp/ module conforms to the unified lifecycle (`prepare` / `reset` / `setParams` / `process`). A single templated harness (Catch2 `TEMPLATE_TEST_CASE` over a per-module adapter struct that knows how to construct, feed silence, and feed a canonical excitation) applies the same battery to: `PluckExciter`, `WaveguideString`, `DamperJunction`, `BridgeJunction`, `SympatheticResonatorBus` (once built), `StringNetwork`, `PickupTap`, `TriodeStage`, `Oversampler`, `CabFilter`, `SoftClipLimiter`, `OutputGain`, `NoteAllocator`, `BlockEventQueue`, and `ModuleGraph`.

Named cases (each parameterized across 44.1/48/96 kHz and block sizes {32, 128, 512}):

- **`CONTRACT: prepare is re-entrant`** — call `prepare(44100, 512)`, process one block, then `prepare(96000, 128)` and process again. Second run must be indistinguishable from a freshly constructed instance prepared at 96 kHz (compared sample-exact against a fresh instance). No crash, no stale-state leakage, all internal sizing re-derived from the worst case (`kMaxStrings`, `kMaxDesignRateHz`, `kMinMidiNote`).
- **`CONTRACT: reset is idempotent and complete`** — excite the module (canonical pluck / driven sine per adapter), process ≥ 1 s, call `reset()`, then process silence. Output must be sample-identical to a fresh prepared instance processing silence. Calling `reset()` twice in a row must equal calling it once. For `StringNetwork`, additionally assert `energyEstimate() == 0.0` immediately after `reset()`.
- **`CONTRACT: silence in, silence out before excitation`** — after `prepare()` + `reset()`, processing zero-filled buffers yields exactly 0.0f on every output sample. This applies to nonlinear stages too: `TriodeStage` must be internally DC-compensated at its quiescent operating point so that zero input produces zero output (with and without `bypass`).
- **`CONTRACT: block-size invariance`** — full split-invariance (processing N samples in one call versus the same N samples split across arbitrary sub-block boundaries, including size-1 blocks, yields sample-identical output, including under active event streams) is asserted only for the per-sample-smoothed sample domain (`StringNetwork` and its members); for `StringNetwork` the event queue is split so each `NoteEvent::sampleOffset` is rebased into the correct sub-block. Block-domain modules are tested with parameters held constant since `prepare()` (smoothers settled) — per-block parameter ramps are inherently split-dependent.
- **`CONTRACT: process allocates nothing`** — the `cnpg_tests` binary installs a counting replacement for global `operator new`/`delete`; the counter is snapshotted around every `reset()`, `setParams()`, and `process()` call in the harness and must not change. `prepare()` is exempt by contract.
- **`CONTRACT: EventQueue ordering and overflow`** — `EventQueue<256>` accepts 256 events, reports `push` failure and increments `droppedCount()` on the 257th, `peek`/`pop` return events in push order, `clear()` zeroes both size and drop count, and a debug-build death test asserts on decreasing `sampleOffset` push order.
- **`CONTRACT: ModuleGraph freeze discipline`** — `connect()` returns false on a cycle and on invalid `NodeId`; `process()` on the frozen hard-wired P2 chain (`PickupTap -> Oversampler(TriodeStage) -> CabFilter -> OutputGain -> SoftClipLimiter` wrapped as `IBlockModule` adapters) is sample-identical to calling the modules directly in sequence; `totalLatencySamples()` equals the sum of the members' reported latencies.

## 4.2 Energy and passivity — `[energy]`, three tiers

The tier separation exists to make failures diagnosable. Tier 1 proves the *scattering algebra* of each junction is passive pointwise in parameter space, with no filters or delay lines involved — a failure here is a math error in the junction design. Tier 2 proves the *assembled network* (delay rails, fractional interpolation, moving-junction crossfades, power normalization across `portImpedance()` values) creates no energy when every intentional loss is disabled — a tier-2 failure with tier 1 green points at interpolation gain > 1, crossfade energy injection, or impedance-normalization mistakes, never at the junction algebra. Tier 3 re-enables the intentional losses and confirms they only ever remove energy — a tier-3 failure with tiers 1–2 green points at a loss or dispersion filter whose magnitude response exceeds unity somewhere. Without this separation, a single "energy grew" assertion cannot distinguish an active junction from a bad loop filter.

**Energy functional.** `energyEstimate()` is a discrete Lyapunov storage function: the impedance-weighted rail energy PLUS the closed-form quadratic storage of every state-bearing element — dispersion allpass states, loss-filter states, fractional-delay interpolator states, and bridge admittance biquad states (Bilbao NSS-style). Rail-only energy cannot back these gates: energy migrates between the rails and the filter states within a block, so rail-only accounting fluctuates at ~1e-3 relative even in a perfectly passive network and can never meet a 1e-9 bound. The sample-domain classes (`PluckExciter`, `WaveguideString`, `DamperJunction`, `BridgeJunction`, `SympatheticResonatorBus`, `StringNetwork`) are `template <typename SampleT>` with explicit instantiations for `float` and `double`; the realtime path uses the `float` instantiation, and the tier-2 cases below run the `double` instantiation, where the 1e-9 per-block bound is justified.

The tier-1 parameter grids below are the canonical definitions, stated here once — tasks reference them and never restate the numbers.

- **Tier 1 — `ENERGY/T1: DamperJunction scattering matrix is passive`.** Instantiates one `DamperJunction` in isolation, losses bypassed. Sweeps the canonical grid: `position01` ∈ {0.0, 0.1, …, 1.0} × engagement ∈ {0, 0.25, 0.5, 0.75, 1} (via `setEngagementImmediate`) × `maxLoss` ∈ {0, 0.5, 1}. For every grid point, `copyScatteringMatrix()` yields the 2×2 S; the test computes its spectral norm in double precision (closed-form 2×2 SVD) and asserts `‖S‖₂ ≤ 1 + 1e-12`. Additionally asserts exact transparency (S = anti-diagonal pass-through) at engagement 0.
- **Tier 1 — `ENERGY/T1: BridgeJunction scattering matrix is passive`.** Instantiates one `BridgeJunction` for every port count 1..8 with representative impedance sets (equal impedances, and a 4:1 spread). Sweeps `BridgeAdmittanceParams` over the canonical grid: `resonanceHz` ∈ {80, 400, 2000, 8000} × `damping` ∈ {0 (exercises the positive-real clamp), 0.1, 1, 10} × `couplingStrength` ∈ {0, 0.5, 1}. For each point, `copyScatteringMatrix()` with losses bypassed; spectral norm via Jacobi SVD in double; assert `‖S‖₂ ≤ 1 + 1e-12`. The same case runs against `SympatheticResonatorBus` once the fallback exists (identical `IBridgePort` seam, identical assertion).
- **Tier 2 — `ENERGY/T2: lossless network impulse conserves energy`.** Instantiates a full `StringNetwork` (6 strings, `BridgeJunction` attached via `setBridgePort`), calls `setLosslessTestMode(true)` (which forwards `setLossBypassed`/lossless configuration to strings, dampers, and bridge). Injects a single-sample unit impulse into string 0 via a NoteOn carrying a hardness-1, noise-0 excitation (and a variant that calls `WaveguideString::injectAt` directly on an isolated string). Processes 10 s in 128-sample blocks; after the excitation block, samples `energyEstimate()` (the Lyapunov storage functional defined above) once per block. The case runs the `double` instantiation of the sample-domain classes — the 1e-9 bound is justified only there. Runs at 44.1/48/96 kHz, for both `FractionalDelayKind::Lagrange3` and `FractionalDelayKind::Thiran1`, and in three motion scenarios: static positions, where the test asserts strict per-block non-increase `E[k+1] ≤ E[k] * (1 + 1e-9)`; `damperPosition01` swept 0.1→0.9 over 5 s; and `pickupPosition01` swept likewise (tap reads must never inject energy). The motion scenarios assert bounded growth instead: cumulative `energyEstimate()` never exceeds the post-excitation maximum and per-block growth stays within a stated tolerance derived from the crossfade overlap — the strict non-increase applies to static-position runs only.

> **Amendment (P2.3, 2026-08-01) — the damper-motion scenario as written above is vacuous, and its replacement is stronger.** `setLosslessTestMode(true)` makes `DamperJunction` *transparent* (S becomes the anti-diagonal pass-through), so sweeping `damperPosition01` under lossless mode moves a junction that is doing nothing: the scenario passes identically against an implementation with no crossfade at all. The vacuity is a property of lossless mode, not of the crossfade, and no dual-anchor read/write construction repairs it — depositing a crossfaded difference through the far anchor breaks passivity (an absorbing junction with anti-correlated rails *creates* energy) while still depositing zero under a transparent junction, and re-reading at the far anchor makes the seam an energy transporter, which destroys the bit-exact seam contract P2.2 established. **The shipped scenario instead runs a *dissipative* damper on a *lossless string*.** That is a strictly stronger gate than the text above: the moving seam becomes the only element in the system that can move energy at all, whereas the configuration above buries it behind six strings' worth of loop losses. Passivity of the moving seam is additionally proven structurally rather than only measured — `railDeposit` is the exact transpose of `railInterpolate`, so with `r = g1·r_A + g2·r_B` the deposit-read composition is a contraction at every fade position, given `‖r‖ ≤ g1 + g2 = 1`. That bound **requires the amplitude-complementary linear crossfade law**; under an equal-power law it becomes √2 and the proof does not hold. The locked crossfade law and the passivity proof are the same decision.
- **Tier 3 — `ENERGY/T3: lossy network only dissipates`.** Same network, lossless mode off, default `StringMaterialParams`, running the shipping `float` (float32) instantiation. Pluck all 6 strings (staggered NoteOns), then after the last excitation sample track the per-block block-RMS envelope of `energyEstimate()` and assert it is monotone decreasing — no block may exceed its predecessor by more than a 1e-6 relative tolerance (absorbing float32 state rounding). The monotonicity assertion applies to `energyEstimate()`, never to the raw bridge output signal: coupled strings beat, and beating is periodic envelope growth, so a companion check on `bridgeOutputBuffer()` instead fits a decay from Schroeder backward integration (or a windowed-max envelope over at least one beat period) and requires the fitted decay slope to be negative after excitation. Repeated with the damper fully engaged (`engage()` on all strings) to confirm accelerated, still-monotone decay.


> **Amendment (P2.4, 2026-08-01) — tier 3's monotonicity assertion has a stated dynamic range, because float32 does.**
> The bound above ("no block may exceed its predecessor by more than a 1e-6 relative tolerance") is written without a
> floor, and a *relative* tolerance cannot absorb a phenomenon whose cause is *absolute*. Bidirectional coupling (P2.4)
> gives the decay a tail the uncoupled network never had — once the strings are damped, the bridge resonator keeps
> re-driving them out of its own residual instead of the silence watchdog clearing them — and run far enough down, the
> shipping float32 path stops being able to represent its own recursions. There are two floors, at different depths.
> **Without** the FTZ/DAZ guard the state itself goes subnormal (~1e-44, quantized to multiples of 1.4e-45) and the
> functional wanders by tens of per cent at total energies of ~1e-87; that configuration ships nowhere, since
> `ScopedFtzDazGuard` has been the first thing `processBlock` constructs since P1.1, so **every tier-3 render engages
> it**. **With** the guard, the recursions multiply the state by coefficients as small as ~1e-2, and a product below the
> smallest *normal* float (1.18e-38) is flushed to zero — which does not merely round the state, it **changes the
> recursion**: an allpass whose `a·x` term has vanished is no longer an allpass, and the closed-form storage derived for
> it is no longer its storage. With ~10³ stored values that begins at |x| ~ 1e-36, i.e. a total energy of ~1e-69.
>
> **The monotonicity assertion is therefore gated down to an energy of 1e-40 and reported below it** — 29 decades above
> where the phenomenon begins, and 186–236 dB below the start of every run, with the level of the last gated block
> printed so the margin is visible rather than asserted. This is a statement about the range over which float32 can be
> measured, not a tolerance: the *identical* scenario on the `double` instantiation is clean over its whole run (worst
> growth 0, following the decay to 1.5e-160), which is what says the junction is passive and the float32 reading was
> arithmetic. `ENERGY/T3: the float32 arithmetic floor is a measurement limit, not a passivity failure` runs all three
> configurations on every CI run and *requires* the double control to be clean and the float32 runs not to be, so the
> finding cannot be re-lost.

## 4.3 Impulse-response regression — `[regression]`

Two layers, exactly as locked. Layer (a) survives algorithm swaps; layer (b) freezes the exact numerical behavior of the current implementation.

**Captured IRs.** Each scenario renders a single-string `StringNetwork` (1 active string, damper transparent, default `BridgeAdmittanceParams`), excited by a `PluckExciter` NoteOn with velocity 0.8, `pluckPosition` 0.28, `hardness` 0.5, and `noiseAmount` 0.25 with the exciter's noise PRNG seeded to a fixed constant recorded in the sidecar. Two signals are captured per scenario, both 3.0 s long: the string's tap channel from `StringTapBuffers::channel(0)` at `pickupPosition01 = 0.87`, and `bridgeOutputBuffer()`. Scenario notes: MIDI {21, 45, 69, 93, 108}. A sixth scenario captures the full 6-string network playing one open E-major chord (tests bridge coupling regression).

**Golden format.** Raw little-endian `float64` samples (`.f64`, the float32 render widened to double at capture time) plus a JSON sidecar of the same basename containing: `schemaVersion`, generator git commit, generation date, dsp state-version integer, `sampleRate`, algorithm variant, MIDI note, excitation parameters and noise seed, tap position, length in samples, comparison `atol`, and the layer-(a) reference features (partial frequencies 1–8 in Hz, per-octave-band T60 values, attack-window RMS in dBFS).

**Directory scheme.** `tests/data/golden/<scenario>/<variant>/<rate>/<name>.f64|.json`, where `<variant>` ∈ {`lagrange3`, `thiran1`} (and later `sympathetic_bus` if the fallback triggers) and `<rate>` ∈ {`44100`, `48000`, `96000`}. Example: `tests/data/golden/string_ir/lagrange3/48000/midi069_pluck28_tap87.f64`.

- **Layer (a) — `REGRESSION/A: feature invariants`.** Re-renders each scenario and extracts: partial frequencies 1–8 (Blackman-Harris FFT + quadratic peak interpolation, the section-4.5 estimator) — each must lie within ±2 cents of the sidecar reference; per-octave-band T60 from 63 Hz to 8 kHz (band-filtered Schroeder backward integration) — each within ±10% relative; RMS of the first 100 ms within ±1.5 dB of reference. These assertions compare against sidecar *features*, not waveforms, so they survive an interpolator or filter-topology swap.
- **Layer (b) — `REGRESSION/B: float64 golden exactness`.** Re-renders and compares sample-wise against the `.f64` file with absolute tolerance 1e-7. This is effectively bit-stability of the float32 path and is toolchain-sensitive; goldens are generated on the pinned MSVC toolchain of the dev machine, so layer (b) runs only in the Windows CI job and locally (the Linux job runs layer (a) only — see 4.9).

> **Amendment (P2.4, 2026-08-01) — "two signals per scenario" is captured at two different resolutions, on purpose.**
> The `bridgeOutputBuffer()` channel above was identically zero through P2.3 (P1's bridge is the rigid termination, so
> a golden of it would have been 3 s of zeros per scenario) and P2.4 makes it a real signal. It is captured as follows:
> - **`chord_ir`** — the coupled 6-string scenario P2.4 adds — captures **both channels as full `.f64` goldens**,
>   because inter-string coupling appears in no other scenario and nothing else in the golden set can gate it.
> - **`string_ir`** — the five single-string scenarios — captures the tap channel as a `.f64` and the bridge channel in
>   the **sidecar**, as its layer-(a) features (the eight band T60s and attack RMS) plus an FNV-1a checksum over the
>   float64 bridge render, asserted in `REGRESSION/A` (the checksum under the same MSVC-only condition as layer (b)).
>   On a single string the bridge output is one more linear functional of a state the tap channel already pins
>   sample-exactly at 1e-7, so a second `.f64` set would add ~45 MB of committed binaries per regeneration, for ever, to
>   gate nothing the tap does not already gate — while the sidecar form gates the decay behaviour a bridge load actually
>   changes, plus sample-exactness, for a few kB. Sidecar schema v3 carries the new fields.

**Regenerate-goldens workflow.** A scripted CMake target `cnpg_regen_goldens` re-renders every scenario, overwrites `.f64` files, refreshes sidecars, and prints a drift report (max abs sample diff and per-feature deltas versus the previous goldens). Rule: any commit touching `tests/data/golden/` must carry a git trailer line `Regenerate-Goldens: <reason>`, where the reason states (1) which variants/rates changed, (2) the algorithm change motivating regeneration, and (3) that layer-(a) invariants pass on the new renders. `.github/scripts/check-golden-commit.sh` enforces the trailer mechanically in the Windows CI job: if the pushed commit modifies files under `tests/data/golden/` and lacks the `Regenerate-Goldens:` trailer, the job fails.

## 4.4 Aliasing gate — `[aliasing]`

Device under test: `Oversampler` wrapping `TriodeStage::process` via `processWrapped`, exactly as composed in the P1 monitoring chain, at 48 kHz host rate.

- **Stimulus.** Fixed sines at 1244.5 Hz and 4186 Hz (chosen so harmonic and fold frequencies do not collide), each at two drive settings: nominal (default APVTS drive with a −18 dBFS peak input, the per-string nominal level) and maximum drive.
- **Measurement.** Discard 8192 warm-up samples, capture 2^18 samples, apply a Blackman-Harris window, FFT ≥ 2^18 points.
- **Bin classification.** Enumerate predicted harmonic frequencies k·f0 well beyond the oversampled Nyquist — until k·f0 exceeds 4 × (factor·24 kHz) or the predicted amplitude falls below the noise floor. Harmonics with k·f0 ≤ 24 kHz are *harmonic*; every other k is classified by its folded image f_fold = |k·f0 − round(k·f0 / 48000)·48000| whenever that image lands in-band — aliasing generated inside the oversampled domain is not exempt from the gate. A spectral peak within ±2 bins of a harmonic prediction is classified harmonic; a peak within ±2 bins of a fold prediction (and not coinciding with a harmonic bin) is classified folded; residual bins are noise floor and ignored by the gate.
- **Gate.** `ALIASING: triode folded components under -60 dBc` — at the default oversampling factor (2×), the worst folded component must be ≤ −60 dB relative to the strongest harmonic component, for all four stimulus/drive combinations. FAIL otherwise.
- **Reporting.** The same case additionally runs factors 4 and 8 and prints a table (factor × stimulus × drive → worst folded dBc, `latencySamples()`), emitted to the CTest log and uploaded as a CI artifact. Non-default factors are report-only, never gated. The P1 ADAA-vs-oversampling spike reuses this exact harness for its recorded in-repo comparison.

## 4.5 Tuning — `[tuning]`

**Acceptance:** measured fundamental within ±2 cents of equal temperament (A4 = 440 Hz) at 44.1, 48, and 96 kHz, in the shipping single-string topology. The P1 case gates MIDI 33–96, with MIDI 21–32 and 97–108 emitted as a report-only error table; the full 88-note (21–108) × 3-rate ±2-cent assertion binds only the P2 calibration-table case.

- **Render.** For each (note, rate): `StringNetwork` with 1 active string, damper transparent, default bridge admittance attached (tuning is accepted against the shipping topology, which is also what `cnpg_calibrate` measures in P2); NoteOn velocity 0.8, `pluckPosition` 0.28, `hardness` 0.5, `noiseAmount` 0. Render 7 s of the tap channel; discard the first 0.5 s of attack (so the 2^18-sample analysis window fits at 44.1 kHz).
- **Estimator (concrete).** Windowed FFT + quadratic (parabolic) interpolation: take 2^18 analysis samples at 44.1/48 kHz (2^19 at 96 kHz, keeping bin spacing ≈ 0.18 Hz), Blackman-Harris window, zero-pad ×4, FFT, locate the fundamental partial's peak bin (search restricted to ±80 cents around the target f0 so dispersion-sharpened upper partials are never picked), then fit a parabola through the log-magnitude of the peak and its neighbors to refine the frequency. Convert to cents against the target.
- **Accuracy floor vs the 2-cent criterion.** 2 cents at MIDI 21 (27.5 Hz) is 31.8 mHz. Bin spacing 0.183 Hz with ×4 zero-padding and quadratic log-magnitude interpolation on a BH window gives worst-case interpolation bias well under 0.5% of a bin ≈ 0.9 mHz — over 30× below the criterion. This floor is not taken on faith: **`TUNING: estimator self-calibration`** feeds the estimator synthetic exponentially decaying sinusoids (with low-level harmonics added) at known frequencies spanning 27.5 Hz–4186 Hz and requires estimator error ≤ 0.2 cents everywhere — a 10× guard band under the 2-cent gate. The sweep test is invalid (and fails loudly) if self-calibration fails.
- **Named cases.** `TUNING: P1 analytic compensation sweep` (P1: `setAnalyticTuningCompensation` active) and `TUNING: P2 calibration-table sweep` (P2: table from `cnpg_calibrate` loaded via `loadCalibrationTable`) — the P1 case gates MIDI 33–96 within ±2 cents (MIDI 21–32 and 97–108 report-only); the full 88-note × 3-rate ±2-cent assertion binds only the P2 calibration-table case. A third case, `TUNING: static bend accuracy`, applies constant `pitchBendSemitones` = ±2 on MIDI 40 and requires the settled pitch within ±2 cents of the bent target.


> **Amendment (P2.4, 2026-08-01) — the P1 analytic case measures the shipping topology and reports; the ±2-cent
> assertion binds P2.7, as this section already says.** Through P2.3 the `TUNING:` cases rendered an *isolated*
> `WaveguideString` rather than the `StringNetwork` topology the recipe above specifies. That was harmless only by
> accident: `couplingStrength` defaulted to 0.0, where `BridgeJunction` reduces bit-exactly to the rigid termination an
> isolated string applies internally, so the two topologies were the same object. P2.4's nonzero shipping default
> (`docs/decisions/0006`) ends the equivalence, and all three named cases now render the shipping topology.
>
> **What that measures.** The bridge seam's own one extra sample is *not* the issue — `WaveguideString::setBridgePortDriven`
> subtracts it from the loop-length solve, measured at 5.6e-6 cents residual against an uncompensated −17.19 cents at
> MIDI 69 / 44.1 kHz. What remains is the **load's phase response**: a bridge resonance pulls the partials near it,
> which is physics (the same mechanism that puts dead spots on a real instrument) and not an error. Measured at the
> shipping admittance: **worst 4.90 cents**, over MIDI 33–96 at all three rates, worst at MIDI 45.
>
> The P1 **analytic** compensation is a closed-form solve over the loop's own filters, derived for a rigid termination,
> and it is exact for one (0.00028 cents). It cannot absorb a load it was derived without. So the P1 cases assert a
> **documented sanity bound of ±12 cents** — set from the measured worst with ~2.4× headroom, and non-vacuous in both
> directions (the residual must be > 0.5 cents, or the render has stopped going through the bridge) — and the ±2-cent
> criterion binds the P2 calibration-table case, exactly as the "Named cases" paragraph above already assigns it.
> A ±2-cent assertion additionally survives on the bend's own *arithmetic*, as the difference between the bent and
> unbent residuals. It lives in `PitchBendClickTests.cpp` (**not** in `TUNING: static bend accuracy`, which asserts only
> the ±12-cent bound). The bridge load does move that difference — measured **+0.379 / −0.351 cents**, about 19% of the
> budget — so the correct claim is that it moves it by *far less than the gate*, leaving ~5× headroom, which is what
> makes the assertion still catch a bend landing off-target.
>
> **BINDING ENTRY CONDITION ON TASK P2.7.** P2.7 owns the ±2-cent gate over the full 88 notes × 3 rates in the shipping
> coupled topology. It also inherits a scope question this task surfaced and deliberately did not solve: the residual is
> a function of three **live APVTS parameters**, not of the MIDI note alone — coupling 0.00/0.35/1.00 → 0.000/−4.855/
> −14.056 cents; resonance 80/110/180/2000 Hz → +4.461/+0.001/−4.855/−0.527 cents; damping 0.01/0.50/10.0 →
> −0.188/−4.855/−0.494 cents — **and the sign reverses across resonance**. A calibration table indexed by MIDI note
> structurally cannot represent that, so P2.7 must either re-scope its mechanism or the parameter surface must change.
> Measured by `TUNING: the coupled residual is a function of three LIVE parameters`.

> **Amendment (author decision, 2026-08-01, `docs/decisions/0007-bridge-tuning-compensation.md`) — the
> scope question above is answered: P2.7 re-scopes its mechanism, and the default instrument stays in tune.**
> The frozen per-note calibration table is **not** the mechanism. P2.7 is redesigned around **analytic bridge
> phase-delay compensation**: the junction's contribution to the loop delay is computed in closed form from its own
> admittance at the string's fundamental, per string, on every parameter change — O(1), no table, correct at every
> point in the parameter space rather than at one frozen admittance. This is the P1.4 phase-delay ruling (phase delay,
> never group delay, governs tuning compensation) applied one element further down the loop.
>
> **The ±2-cent acceptance therefore binds on the shipping coupled topology across a declared *normal range* of the
> three bridge parameters** — not merely at their defaults. Outside that range, bounded physical detuning may be
> retained later as an optional, declared advanced behaviour; it is not a licence for the default instrument to drift
> and it is not in P2.7's scope unless the analytic correction proves unable to hold ±2 cents across the normal range.
>
> Two consequences that are easy to miss. **The recompute lands inside P2.3's machinery** — changing tuning
> compensation while a string rings *is* changing its delay length while it rings, the exact case the dual-anchor
> crossfade exists for — so the parameter-change path routes through it and needs its own click gate. **The correction
> is self-referential** — the phase delay is evaluated at f0, but changing the loop length changes f0 — so P2.7 needs a
> fixed-point iteration with a declared convergence criterion, offline at parameter-change time, never on the audio
> path. `cnpg_calibrate` survives as the *verification* harness across the parameter grid and as a possible residual
> trim; it is no longer the mechanism of record.
>
> **Owed by the author before P2.7's gate can be written:** the definition of the *normal parameter range* — a musical
> answer, the range over which the bridge is a bridge rather than an effect. ADR 0007 records two further open
> questions (the bound for extreme settings, and whether the analytic correction holds near the load resonance where
> the phase slope is steepest — the risk item, to be measured early rather than discovered at the gate).

## 4.6 Denormal robustness — `[denormal]`

- **`DENORMAL: long tail leaves no subnormal state`** — pluck a 6-string `StringNetwork` + full monitoring chain once, then process 60 s of silence-in decay in 128-sample blocks with the same FTZ/DAZ RAII guard the plugin's `processBlock` uses engaged around each block. After the tail: no sample anywhere in the output history is NaN/Inf; sampling the final output blocks and the exposed state probes (`energyEstimate()`, final tap buffers) shows no value classified `FP_SUBNORMAL` via `std::fpclassify`.
- **`DENORMAL: no CPU blow-up in the tail`** — same render, wall-clock timed per block. Assert the median block time over the final 5 s of the tail ≤ 2× the median block time of the first (loud) 1 s. This catches denormal-induced slowdowns that FTZ/DAZ should have eliminated. Because the assertion is timing-based it is flaky on shared runners and therefore local-only (see 4.9); the state-inspection case above runs everywhere.

## 4.7 `cnpg_bench` methodology

`cnpg_bench` is a headless benchmark executable over `cnpg_dsp` (lands in P1, per the locked mandate — regression visibility from the first playable build).

- **What it measures.** Renders a fixed, deterministic 60 s workload (dense corpus-derived note stream, all strings active, default oversampling) at 48 kHz / 128-sample blocks, timing each `process` call with a steady high-resolution clock. Reports median, p99, and max block time over all N ≈ 22 500 blocks, converted to CPU%: `blockTime / (128/48000)`. Median is the headline; p99 and max are reported to expose spikes (event bursts, smoother retargeting).
- **Gate vs report.** The hard gate is the 6-string default configuration (`p2_default6`) + full monitoring chain at default oversampling, 48 kHz / 128-sample blocks: median CPU ≤ 30% of one core on the dev machine (Windows 11 Pro workstation) is the hard gate, with 25% as the aspirational target. The 8-string configuration (`p2_max8`) is always measured and reported and must remain realtime (< 100%), but is never gated in CI — the pass/fail gate applies only to `p2_default6`. Additional reported (never gated) rows: 44.1 and 96 kHz, block sizes 64 and 512, oversampling 4×.
- **Machine documentation.** Every run emits JSON containing: CPU model and core count, Windows power plan, compiler and version, build type and flags, git commit, sample rate, block size, string count, oversampling factor, N blocks, median/p99/max block time, derived CPU%. Gate results are only meaningful against the documented dev machine; the JSON makes every historical number attributable.
- **Why CI never gates on performance.** CI runners are shared, virtualized, thermally uncontrolled, and change hardware generations silently — block-time numbers on them are noise-dominated and would make the gate flaky in both directions (false failures on contended runners, false passes on fast ones). Therefore both CI jobs *run* `cnpg_bench` and upload the JSON as an artifact for trend inspection, but no CI step compares numbers against thresholds. The binding gate is executed manually on the documented dev machine at each milestone close and its JSON is committed to `docs/bench/`.

## 4.8 `cnpg_render`, the MIDI corpus, and the listening pass

`cnpg_render` is a headless MIDI→WAV renderer over `cnpg_dsp` (JUCE-free, so it runs in both CI jobs), consuming a MIDI file plus an optional JSON automation sidecar (parameter lanes with sample-stamped breakpoints — required for the mid-note parameter-sweep phrases).

**Corpus.** Versioned under `tests/corpus/` with a `corpus.json` manifest carrying an integer `corpusVersion`, per-phrase render settings (params, `RetriggerMode`, seed), and the checklist items each phrase exercises. Contents:

| Phrase | Phase | Exercises |
|---|---|---|
| `01_chromatic_singles.mid` | P1 | full-range single notes, decay character |
| `02_open_chords.mid` | P2 | 6-voice bridge coupling, summing headroom |
| `03_legato_retrigger.mid` | P1 (default mode only; per-mode variant renders land in P2) | `RetriggerMode::Physical` vs `Synth` (rendered once per mode) |
| `04_palm_mute_chug.mid` | P2 | **abuse:** rapid palm-mute chugging, 16th-notes at 180 BPM, partial damper engagement |
| `05_low_string_bends.mid` | P1 | **abuse:** aggressive ±2-semitone bends on the lowest open string, overlapping with retriggers |
| `06_sustain_chords.mid` | P2 | **abuse:** full open-string chords repeatedly struck under CC64 sustain |
| `07_param_sweeps_midnote.mid` + `.json` | P1 | **abuse:** extreme `pickupPosition01`, `damperPosition01`, drive, and material sweeps while notes ring |
| `08_harmonics_nodes.mid` + `.json` | P2 | damper parked at p = 1/2 and 1/3 while strings ring (natural-harmonic emergence) |

**Determinism.** Renders are single-threaded, float32, dither-free 32-bit-float WAV; every stochastic component (exciter noise burst) draws from a PRNG seeded from the manifest. **`RENDER: determinism smoke`** (tagged `[contract]`) renders one phrase twice in-process and once out-of-process and requires byte-identical files (same binary; cross-toolchain identity is not claimed). Render filenames embed corpus version + git hash so listening notes are attributable.

**Physical-plausibility checklist (versioned at `docs/listening/physical-plausibility-checklist.md`; the actual items, one verdict each: pass / concern / fail):**

1. **Harmonics at nodes** — damper at p = 1/2 suppresses the fundamental and leaves the 2nd harmonic ringing (octave harmonic); p = 1/3 yields the 12th. (Also CI-enforced as an objective proxy: FFT of phrase 08 shows fundamental ≥ 20 dB below the 2nd harmonic at p = 1/2.)
2. **Palm-mute character** — muted notes are short, pitched thumps, not clicks or unpitched thuds; chugging at speed stays tight with no energy buildup between hits.
3. **Retrigger, Physical** — same-pitch repluck sounds like picking an already-sounding string; pitch changes choke briefly, then re-excite with plausible legato; no pops.
4. **Retrigger, Synth** — instant clean restart at the new pitch, no click, no residual old pitch.
5. **Bend cleanliness** — bends glide smoothly; no zipper noise, stepping, or clicks anywhere in phrase 05, including at the bend extremes.
6. **Chord density under sustain** — phrase 06 stays stable and separable; no runaway level, output stays below the `SoftClipLimiter` ceiling without audible pumping.
7. **Pluck position audibility** — near-bridge plucks are noticeably brighter/thinner; near-middle plucks are hollow with suppressed even harmonics.
8. **Velocity/hardness response** — harder and faster playing gets brighter as well as louder, not merely scaled.
9. **Decay realism** — high notes die faster than low notes; nothing terminates abruptly or rings implausibly forever.
10. **Moving positions mid-note (P2)** — pickup and damper position sweeps in phrase 07 are click-free and continuous.
11. **Silence hygiene** — gaps between phrases decay to digital silence; no denormal fizz, no stuck resonance.
12. **Abuse survival** — nothing in phrases 04–07 produces NaN blasts, stuck notes, or level runaways.

**Listening-pass procedure (per milestone, P1 and P2).** (1) Run `cnpg_render` over the full corpus at 48 kHz, default parameters plus the manifest's variant renders. (2) The author listens on documented monitoring (device named in the report) and fills the checklist, citing WAV filename + timestamp for every "concern"/"fail". (3) The report is committed to `docs/listening/P<phase>-<yyyymmdd>.md` together with the corpus version and git hash. (4) Any "fail" blocks milestone closure until fixed and re-rendered, or explicitly re-scoped with rationale in the same report. The CI-enforced objective proxies (pitch ±2 cents, damper-at-node harmonic suppression, click/NaN/denormal-free automated parameter sweeps) run continuously regardless; the listening pass is the human gate layered on top, per the locked hybrid design.

## 4.9 Test-to-CI mapping

| Test / tool | Tag | Windows CI (VST3 + tests + pluginval) | Ubuntu CI (dsp/ headless) | Local-only |
|---|---|---|---|---|
| Contract battery, EventQueue, ModuleGraph, render determinism smoke | `[contract]` | run + gate | run + gate | — |
| Energy tiers 1–3 | `[energy]` | run + gate | run + gate | — |
| IR regression layer (a) feature invariants | `[regression]` | run + gate | run + gate | — |
| IR regression layer (b) float64 goldens | `[regression]` | run + gate (pinned MSVC toolchain) | skipped (goldens are toolchain-pinned) | regen via `cnpg_regen_goldens` |
| Golden commit-trailer guard (`Regenerate-Goldens:` via `.github/scripts/check-golden-commit.sh`) | — | gate (only when `tests/data/golden/` changes) | — | — |
| Aliasing gate + factor report | `[aliasing]` | run + gate | run + gate | — |
| Tuning sweeps + estimator self-calibration | `[tuning]` | run + gate | run + gate | — |
| Denormal state inspection | `[denormal]` | run + gate | run + gate | — |
| Denormal timing ratio | `[denormal]` | skipped (timing noise) | skipped (timing noise) | run + gate on dev machine |
| `cnpg_bench` | — | run, upload JSON artifact, **never gate** | run, upload JSON artifact, **never gate** | hard gate on documented dev machine per milestone |
| pluginval (high strictness) | — | run + gate | — | also run in Ableton Live acceptance |
| Corpus render + listening pass | — | renders may be produced as artifacts | renders may be produced as artifacts | manual, per milestone, report in `docs/listening/` |
| Ableton Live manual acceptance criteria | — | — | — | manual on Windows 11 dev machine |

Both CI jobs run `ctest` over all non-skipped tags on every push and pull request; the Ubuntu job builds only `cnpg_dsp`, `cnpg_tests`, `cnpg_bench`, `cnpg_render`, and `cnpg_calibrate` (no JUCE), serving as the portability guard for the headless core.

## 5. Risks and fallback paths

Each risk lists Likelihood / Impact (H/M/L), the early-warning signal that should be watched deliberately (not discovered by accident), the mitigation already built into the plan, and the pre-committed fallback path. Fallbacks never relax a locked test gate; they trade scope or approach.

### R1. Bridge passivity failure (bidirectional coupling)
- **Likelihood / Impact:** M / H
- **Early warning:** Tier-1 spectral-norm test on `BridgeJunction::copyScatteringMatrix` exceeding `1 + 1e-12` for corner values of `BridgeAdmittanceParams`; tier-2 `[energy]` tests (run on the double instantiation, `setLosslessTestMode(true)`) showing per-block increase of the storage-functional `StringNetwork::energyEstimate` — impedance-weighted rail energy plus closed-form quadratic storage of every state-bearing element — beyond the 1e-9 non-increase tolerance; audible low-level growth in long corpus renders.
- **Mitigation:** Passive-by-construction scattering formulation with power-normalized wave variables; positive-real constraint enforced at `setAdmittance` time (clamping, not trusting the caller); the corrected three-tier `[energy]` suite (tier-2 on the double instantiation of the storage functional; tier-3 float32 monotone-decreasing block-RMS envelope of `energyEstimate`) runs in CI from the moment `BridgeJunction` exists.
- **Fallback (locked protocol):** Design-for-fallback only until triggered. `BridgeJunction` and `SympatheticResonatorBus` share the single port-level seam `IBridgePort`; `StringNetwork` holds exactly one `IBridgePort&` and cannot tell them apart. Trigger = the tier-1 passivity grid or the corrected tier-2 `[energy]` tests (double instantiation, storage-functional `energyEstimate`) still failing after a 1-2 calendar-week timebox inside P2. On trigger: implement `SympatheticResonatorBus` (passive terminations + driven internal resonators, unidirectional), ship P2 on it, and move bidirectional coupling to a research branch. No interface churn anywhere else.

### R2. Moving-junction / moving-tap clicks (P2)
- **Likelihood / Impact:** H / M
- **Early warning:** CI click/NaN parameter-sweep proxies failing on `pickupPosition01` / `damperPosition01` ramps; audible zipper or thumps in the corpus abuse case `07_param_sweeps_midnote.mid`.
- **Mitigation:** Crossfade machinery lives inside `WaveguideString` (`readTapAt` internally crossfaded; the `readJunctionInputs`/`writeJunctionOutputs` seam owns moving-junction crossfades); `DamperJunction` itself stays memoryless apart from its loss smoother; per-sample smoothing on all sample-domain positions.
- **Fallback:** Slew-limit position modulation rate inside `StringNetwork` (bounded-rate motion, parameter surface unchanged); if a residual click floor persists, widen crossfade windows and accept slightly softened position transitions — continuous modulation is preserved, only its maximum slew is bounded.

### R3. Per-sample loop CPU blowout (including 8-string config)
- **Likelihood / Impact:** M / H
- **Early warning:** `cnpg_bench` lands in P1 (locked) precisely for this: single-string P1 cost × 8 extrapolation already near the 25-30% budget; per-sample virtual dispatch through `IBridgePort::scatter` showing up in profiles.
- **Mitigation:** Structure-of-arrays layout across strings from the interface draft onward; termination filters consolidated (one loss filter + allpass chain per string, not per segment); the concrete `BridgeJunction`/`SympatheticResonatorBus` are `final` so the hot path can devirtualize; CI reports benchmark numbers every build (never gates).
- **Fallback:** In order: SIMD across the string dimension (SoA enables this without redesign); reduce dispersion allpass order for low notes where it dominates; lower default `Oversampler` factor is already 2x. The hard gate stays on the 6-string default; the 8-string config remains "measured and reported, must stay realtime". String count stays user-configurable 1..8 with the `kMaxStrings = 8` preallocation intact; if 7-8 strings run hot, ship a documented performance advisory for those counts — imposing a hard cap below 8 would reopen a locked decision and requires taking it back to the user.

### R4. Dispersion-vs-tuning interaction breaks the ±2-cent gate
- **Likelihood / Impact:** H / M
- **Early warning:** P1 analytic compensation (`setAnalyticTuningCompensation`) drifting past ±2 cents as `dispersionAmount` rises, worst at MIDI 21 and 108; per-note error curves from the tuning test trending with material morph.
- **Mitigation:** The P2 upgrade is designed for exactly this: `cnpg_calibrate` measures the actual dsp/ loss and dispersion implementations (not formulas) and emits the per-note cents table consumed by `loadCalibrationTable`; acceptance covers MIDI 21-108 at 44.1/48/96 kHz.
- **Fallback:** Calibrate on a grid over `dispersionAmount` and interpolate tables at material changes (message thread); if extremes still fail, clamp the usable `dispersionAmount` range at the offending end of the pitch range and document the clamp — the gate itself does not move.

### R5. ADAA-vs-oversampling spike inconclusive (P1)
- **Likelihood / Impact:** M / L
- **Early warning:** Spike timebox half-spent with ADAA and 2x oversampling within measurement noise of each other on the aliasing gate and CPU.
- **Mitigation:** The spike is timeboxed inside P1 and the `Oversampler` wrapper ships regardless; the aliasing gate (-60 dBc folded, fixed sines 1244.5/4186 Hz) is the objective referee.
- **Fallback:** Default to the `Oversampler` path (already the shipping architecture), record "oversampling by default; ADAA re-evaluate alongside the P4 nonlinear stages" in `docs/decisions/0003-adaa-vs-oversampling.md`. No schedule impact.

### R6. Denormal CPU spikes in decay tails
- **Likelihood / Impact:** M / M
- **Early warning:** The locked denormal test (60 s decay tail; state-inspection case in both CI jobs, timing-ratio case local-only); tail-end CPU spikes in `cnpg_bench`; divergence between the Windows job (plugin FTZ guard active) and the Linux dsp/-only job (tests must engage FTZ themselves).
- **Mitigation:** `ScopedFtzDazGuard` (RAII FTZ/DAZ) in `processBlock`; headless tests set FTZ explicitly so the Linux portability job exercises the same numeric regime; the test's state-inspection case (fpclassify probes + NaN/Inf scan) runs in both CI jobs, while the timing-ratio case stays local-only (flaky on shared runners).
- **Fallback:** Inject an inaudible alternating-sign LSB-scale offset inside the termination loss filters (standard anti-denormal dither), verified against the float64 goldens' atol so it never perturbs regressions.

### R7. JUCE 8 / MSVC toolchain breakage
- **Likelihood / Impact:** M / M
- **Early warning:** Windows CI job failing after a runner image or MSVC toolset update; FetchContent resolving differently than the pinned state.
- **Mitigation:** JUCE pinned by GIT_TAG (locked); Catch2 pinned via FetchContent; the Linux dsp/-tests job acts as a compiler-diversity guard so MSVC-specific code never accumulates silently in `cnpg_dsp`.
- **Fallback:** Pin the MSVC toolset version explicitly in the workflow; if FetchContent itself misbehaves, vendor JUCE as a submodule at the identical tag. Toolchain upgrades happen deliberately between phases, never mid-milestone.

### R8. Ableton Live host quirks
- **Likelihood / Impact:** H / M
- **Early warning:** Behavior differing between the Standalone target and Live; pluginval passing while Live misbehaves; zipper noise appearing only under Live automation.
- **Mitigation:** No code assumes a constant block size — `maxBlockSize` is a capacity bound only, and `StringNetwork::process` takes `numSamples` per call (Live delivers irregular and small blocks, especially with automation active); all modules own their smoothers, so Live's block-rate automation resolution cannot produce steps; pluginval at high strictness gates CI; plugin scanning is de-risked by pinned plugin IDs/codes from P0 and a frozen bus layout (declared sidechain input never changes shape later).
- **Fallback:** Reproduce every Live-specific issue in pluginval or a headless harness before fixing (keeps fixes testable); use the Standalone target for iteration when Live's scanner cache misbehaves; document known Live workarounds in docs/ so manual acceptance criteria stay executable.

### R9. Golden-file churn during algorithm swaps
- **Likelihood / Impact:** H / L
- **Early warning:** Frequent regenerate-goldens commits; goldens failing on changes that leave the feature invariants (partials ±2 cents, per-octave T60 ±10%, RMS bounds) green.
- **Mitigation:** The two-layer design absorbs this: layer (a) invariants are the development-time referee and survive algorithm swaps; layer (b) float64 goldens are keyed per algorithm-variant per sample rate, so a swap adds new goldens rather than thrashing old ones; commits touching `tests/data/golden/` must carry a `Regenerate-Goldens: <reason>` trailer, enforced by `.github/scripts/check-golden-commit.sh` (audit trail).
- **Fallback:** Retire goldens with their variant when a variant is deleted; if a golden set churns more than once per phase without an algorithm change, treat it as a determinism bug (uninitialized state, order-of-summation) and fix the cause — never widen atol to make churn pass.

### R10. GPL dependency contamination
- **Likelihood / Impact:** L / H
- **Early warning:** Any new FetchContent entry or copied source file entering the tree without a license note in the commit.
- **Mitigation:** Dependency surface is deliberately tiny and locked: JUCE (GPL path, no splash), Catch2 (BSL-1.0, GPL-compatible), VST3 SDK via JUCE's GPLv3-compatible path. No chowdsp_wdf in P0-P2 (re-evaluated P4 with license review as part of that evaluation). Rule: every new dependency gets a license check before the first commit that references it.
- **Fallback:** Remove or reimplement the offending dependency; because dsp/ is JUCE-free and self-contained, contamination risk is confined to the plugin layer where substitutes exist.

### R11. Solo-developer bus factor and schedule slip
- **Likelihood / Impact:** M / H
- **Early warning:** Any phase gate slipping more than two calendar weeks; a timebox (bridge passivity, ADAA spike) expiring without its decision recorded; corpus listening passes being skipped "temporarily".
- **Mitigation:** Public GitHub repo from P0 with decisions recorded in-repo (nothing lives only in one head); CI-enforced objective proxies keep quality measurable without a second reviewer; timeboxes carry pre-committed fallbacks (R1 is the template) so a stall converts to a decision, not a drift; parallel workstreams are allowed but phases gate serially.
- **Fallback:** Cut scope only along the already-staged deferral lines (P3+ items slip further out); extend calendar rather than skipping listening passes or weakening gates; if work must pause, the phase-gate discipline guarantees the repo is resumable at the last green gate.

### R12. float32 precision at pitch-range extremes
- **Likelihood / Impact:** M / M
- **Early warning:** Tuning failures concentrated at MIDI 21 (96 kHz: ~3500-sample loop) and MIDI 105-108 (44.1 kHz: ~10-sample loop where one sample of group-delay error is tens of cents); goldens diverging between sample rates.
- **Mitigation:** Delay lengths, compensation, and calibration tables are computed in `Sample64` and only the signal path runs float32 (locked numerics split); the calibration acceptance explicitly covers both extremes at all three rates.
- **Fallback:** Double-precision filter coefficients with float32 states in the termination filters at the high extreme; at the low extreme, verify the fractional-delay kind chosen in P1 (`FractionalDelayKind`) against worst-case coefficient sensitivity before fixing it at `prepare`.

## 6. Open questions

Only items that are genuinely open remain here; everything else in this plan is locked. Each entry states why deferral is safe, the deadline, and the decision owner (solo project: the developer decides, but each entry names the evidence that binds the decision).

1. **Lagrange3 vs Thiran1 final call.** Safe to defer because `FractionalDelayKind` is fixed at `prepare` and both variants satisfy the same `WaveguideString` interface — no downstream code changes either way. Must close: inside P1, before the P1 tuning acceptance test is frozen (its compensation formula depends on the choice). Decided by: the developer, bound by the P1 spike's measured cents error across MIDI 21-108, click-free behavior under continuous `bendSemitones` modulation, and CPU cost; rationale recorded in-repo (locked requirement).
2. **Exact felt time-constant default.** The 20-100 ms range is locked and the parameter is user-facing, so any in-range default is shippable; the default is a voicing judgment needing real renders. Must close: P2 voicing gate (author listening pass). Decided by: the developer via the physical-plausibility checklist over the versioned MIDI corpus, especially the palm-mute chug abuse case `04_palm_mute_chug.mid`.
3. **Guitar-emulation fingering heuristic tuning.** The `NoteAllocator` interface and `AllocationMode::GuitarFingering` contract are fixed, and the P2.6 fingering algorithm is the committed initial implementation; what remains open is tuning its heuristics (string preference order, hand-position window, stealing policy for monophonic strings) from corpus playing before the P2 allocation acceptance — the heuristic internals are invisible to every other module. Must close: end of P2 (allocation-mode acceptance). Decided by: the developer via corpus phrases plus manual playing in Ableton Live against the acceptance criteria for chord and legato passages.
4. **Material morph parameter ranges.** `StringMaterialParams` fields are locked; the numeric endpoints for the steel and nylon presets (loss gains, dispersion scaling) need measurement, not speculation. Must close: P2, alongside `cnpg_calibrate` (ranges and calibration interact per R4). Decided by: the developer, bound by per-octave-band T60 measurements from the IR feature-invariant layer and the corpus listening pass.
5. **Dedicated 8-string performance gate.** Locked position through P2: 8-string is measured and reported, must remain realtime, hard gate on the 6-string default. Whether the 8-string config gets its own numeric gate is open because it depends on measured headroom, unknowable before `cnpg_bench` history exists. Must close: at the P2 exit review, as an input to P3 planning. Decided by: the developer from accumulated `cnpg_bench` data on the dev machine.
6. **P4 oversampled-island boundary for magnetic pickup nonlinearity.** Open because pickup nonlinearity is P4 scope and the `Oversampler` split API (`upsample`/`downsample`) already supports both candidate shapes — one shared island spanning pickup nonlinearity and triode, or separate wrapped stages. Latency, CPU, and aliasing trade-offs cannot be evaluated until the P4 stages exist. Must close: during P4 planning, before the first P4 nonlinear stage is implemented. Decided by: the developer, bound by the aliasing gate methodology (folded components ≤ -60 dBc) extended across the P4 chain and the host-reported latency budget.

## 7. P3-P5 outlines (non-binding)

Capabilities, the fixed interfaces they plug into, and principal risks. No task breakdowns; tasks are authored when each phase is planned in detail.

### 7.1 P3 — Body modal bank and wood parameter mapping

**Capabilities.** A modal resonator bank (parallel second-order resonators, preallocated maximum mode count) processing the bridge signal — the "wooden body" module. Wood-species parameter mapping from published material data (Wegst 2006; Bucur; Brémaud; Obataya) onto modal frequency/damping/gain scalings, so body character is a continuous material control, not a fixed IR. `tools/` activates with BOTH deliverables: the Python modal-extraction pipeline (first Python permitted in the repo), which extracts modal tables offline from measured or synthesized body IRs, and the C++ headless triode-table generator specified at signature level in P1. The triode's deferred contracts begin landing: a dynamic stage supersedes the static-waveshaper caveat, `TriodeStage::setSupplyVoltage` / `setHeaterVoltage` become audible, and `loadTransferTable` receives tables produced by the C++ headless triode-table generator. Drive-feel voicing judgments, explicitly deferred out of P1, become admissible here.

**Modal table format.** A versioned table (state-version integer discipline extends to it): per-mode records of frequency (Hz, sample-rate independent), T60, gain, plus a table-level normalization reference; validated on the message thread at load, rejected tables leave prior state intact — mirroring the `loadCalibrationTable` and `loadTransferTable` loading conventions already drafted.

**Plugs into.** The body module implements `IBlockModule` and consumes `StringNetwork::bridgeOutputBuffer()`; its output mixes with the `PickupTap` path ahead of `TriodeStage` (exact mix topology is a P3 planning decision — the chain remains hard-wired until P4, so this is an insertion into the fixed chain, not a `ModuleGraph` activation). IR regression layer (a) invariants extend naturally: modal frequencies within cents tolerance, per-band T60 within ±10%.

**Main risks.** Modal bank CPU at high mode counts colliding with the P2 performance budget (mitigation lineage: `cnpg_bench` already reports every build); sparse or inconsistent published wood data producing an unconvincing material morph; phase interaction between body and pickup paths when mixed; Python pipeline reproducibility (pin the environment, commit generated tables — CI never runs Python).

### 7.2 P4 — Transformer, power amp, routing, feedback bus

**Capabilities.** Output transformer as a flux-domain Jiles-Atherton model (LTspice Chan variant as the comparative reference); push-pull power amplifier with supply sag dynamics; magnetic pickup nonlinearity finally landing (deferred from P1); free block-domain routing; the feedback bus closing the loop from power-amp output back to string re-excitation — the only backwards domain crossing, its block delay physically justified as speaker-to-string air propagation with a variable ms delay parameter. chowdsp_wdf is re-evaluated here (locked: excluded P0-P2), with license compatibility review as part of the evaluation.

**Plugs into.** `ModuleGraph` activates for real: transformer, power amp, and body become `IBlockModule` nodes; `addNode`/`connect`/`freeze` replace the hard-wired chain; `totalLatencySamples` feeds host latency reporting. The feedback path is an explicit delay-bearing node feeding `StringNetwork::injectFeedback(buffer, numSamples, airDelayMs, gain)` — never a `ModuleGraph` cycle (cycles are rejected by contract). Pickup nonlinearity extends the `PickupTap` stage and the oversampled island per open question 6, using the `Oversampler` split API. `KorenTriodeParams` and the P3 dynamic stage carry forward unchanged.

**Main risks.** Feedback-loop runaway (needs an energy monitor and hard safety limiting on the injection path — the tier-3 monotone-energy machinery is reusable as a watchdog); Jiles-Atherton solver stability and per-block cost; aliasing budget across three cascaded nonlinear stages (the P1 aliasing gate methodology must be re-run per stage and end-to-end); latency bookkeeping errors across the frozen graph; sag time constants interacting with feedback delay to produce unintended low-frequency oscillation.

### 7.3 P5 — MPE, presets, GUI

**Capabilities.** MPE via the locked channel=string seam: `NoteAllocator` gains a channel-keyed assignment mode; per-note pitch bend, pressure, and timbre map onto per-string parameters; `StringNetwork` is untouched except for widening the per-string parameter block (per-string bend joins the existing global `pitchBendSemitones`). Mod-wheel vibrato (deferred from P1) lands. A preset system with factory content; the state-version integer embedded since P1 becomes a real compatibility contract — the "no session-compatibility guarantee" policy ends, and saved states migrate forward across versions from P5 on. A custom GUI replaces the generic DAW editor.

**Plugs into.** `NoteEvent::channel` (carried opaquely end-to-end since P0 precisely for this); `NoteAllocator::stringForNote(channel, midiNote)`; `StringNetworkParams::perString`; APVTS as the single parameter source with the versioned state chunk.

**Main risks.** Per-note modulation stressing smoothing paths designed for one global bend (the per-string extension must preserve the click-free guarantees proven in P1-P2, re-running the same sweep proxies); GUI scope creep against solo-developer capacity (mitigation: the instrument is fully playable through a generic editor, so GUI cuts are always available); the onset of preset backward-compatibility as a permanent maintenance tax — every post-P5 parameter change now needs a migration, which is exactly why the guarantee was withheld until here.

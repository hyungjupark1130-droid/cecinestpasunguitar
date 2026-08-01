# `tests/corpus/` — the versioned MIDI phrase corpus

The corpus is the fixed set of musical inputs every milestone listening pass is judged on
(`docs/plan.md` section 4.8). It is consumed by `cnpg_render`, which turns it into WAVs, and by
`tests/dsp/RenderTests.cpp`, which renders it headlessly in CI and scans the result. It lives here
flat (locked location), with one manifest, `corpus.json`.

## The rules

1. **Append-only.** A phrase that has been committed is never edited and never deleted. Its bytes
   are frozen the moment a listening report cites a render made from it — otherwise "phrase 05
   sounded wrong at 0:14" stops meaning anything, because *which* phrase 05 is no longer knowable
   from the report. New material is added as new files.
2. **Versioned.** `corpus.json` carries an integer `corpusVersion`. It increments by one whenever a
   phrase is added. Because of rule 1, a corpus version identifies exactly one set of file
   contents — not merely one set of filenames.
3. **Render filenames carry that version, and the git hash.** `cnpg_render --corpus` writes
   `<phrase stem>__cv<corpusVersion>_g<git hash>.wav`, so a WAV on disk names both the input corpus
   and the build that rendered it (`docs/plan.md` section 4.8: "Render filenames embed corpus
   version + git hash so listening notes are attributable"). The git hash is resolved at CMake
   configure time and therefore names the parent of the commit that landed the binary — close
   enough to attribute a listening note to a build, and deliberately *not* the self-consistent
   content hash the golden sidecars need (see `tests/support/SourceHash.h` for that distinction).
4. **The manifest is checked, not trusted.** `durationSeconds` is verified against the actual
   rendered length on every `--corpus` run (10 ms tolerance) and the render fails if they disagree,
   so the manifest cannot quietly go stale against the files it describes.

### Current version: 1 (P1 — phrases 01, 03, 05, 07)

Task P2.8 appends `02_open_chords.mid`, `04_palm_mute_chug.mid`, `06_sustain_chords.mid`, and
`08_harmonics_nodes.mid` + `.json`, and bumps `corpusVersion` to 2. It also adds the
per-`RetriggerMode` variant renders of `03_legato_retrigger.mid` — those are *render
configurations*, not new MIDI files, so they do not change the corpus itself. Phrase 03 is rendered
in the default (Physical) mode only in P1 because the full Physical/Synth semantics need
`DamperJunction`, which is Task P2.2.

## Manifest fields

| Field | Meaning |
|---|---|
| `corpusVersion` | Integer; see rule 2. |
| `phrases[].file` | MIDI filename, relative to this directory. |
| `phrases[].phase` | The milestone that added the phrase. Documentation only. |
| `phrases[].description` | What is actually played. |
| `phrases[].exercises` | What the phrase is *for*. |
| `phrases[].checklistItems` | The `docs/listening/physical-plausibility-checklist.md` item numbers this phrase is the primary evidence for. |
| `phrases[].retriggerMode` | `"Physical"` or `"Synth"`; sets `StringNetworkParams::retriggerMode` for the render. |
| `phrases[].seed` | See the honesty note below. |
| `phrases[].sidecar` | Automation sidecar filename, or `null`. |
| `phrases[].params` | Constant parameter overrides for the whole phrase, keyed by the plugin's own APVTS parameter IDs (`plugin/src/Parameters.h`, namespace `ID`). Empty means "the shipped defaults". |
| `phrases[].durationSeconds` | The render's length: last MIDI event + a fixed 2.0 s tail. Verified per rule 4. |

**Honesty note on `seed`.** `docs/plan.md` section 4.8 specifies that "every stochastic component
(exciter noise burst) draws from a PRNG seeded from the manifest", and the field is here for that.
It is **inert in P1**: `PluckExciter` seeds its noise PRNG from a fixed, non-time-based compile-time
constant on every `prepare()`/`reset()` (`dsp/src/PluckExciter.cpp`, `kNoiseSeed`) and exposes no
seeding API, so nothing reads this value yet. Renders are byte-identical regardless — that fixed
constant is *why* — and at the shipped default `PluckExciterParams::noiseAmount` of 0 the noise path
contributes nothing to a corpus render at all. The values recorded here are per-phrase constants
ready for the phase that adds a seeding API; a corpus author should not expect changing one to
change a P1 render.

## Automation sidecars

A sidecar is a JSON document with a `lanes` array; each lane names a parameter (same vocabulary as
`params` above) and a list of `{ "time": <seconds>, "value": <number> }` breakpoints. Values are
linearly interpolated between breakpoints and held flat before the first and after the last, and the
lane is evaluated once per render block at that block's first sample — which is exactly the
granularity a host automates at (`docs/plan.md` Task P1.1: the APVTS snapshot is read once per
block, and every module's own smoother turns a per-block step into a continuous ramp).

Times are in **seconds**, not raw sample indices, even though `docs/plan.md` section 4.8 calls these
"sample-stamped breakpoints" — the stamps the renderer applies *are* sample-exact, resolved at the
render rate. Seconds is the authoring unit because the same sidecar has to render correctly at
44.1, 48 and 96 kHz (Task P2.8 renders the whole corpus at all three); a fixed sample index would
name a different musical moment at each rate and would silently slide the sweep out of sync with the
phrase it is sweeping over.

Not every accepted parameter name does something in P1. `damperPosition01` in particular is stored
by `StringNetworkParams` and is **inert** until `DamperJunction` lands in Task P2.2
(`dsp/include/cnpg/dsp/StringNetwork.h`, "P1 SCOPE"): `cnpg_render` accepts it so a P2 phrase does
not need a renderer change, and a P1 render will not respond to it.

## How these files were authored

Every file is SMF **format 0**, one track, **480 ticks per quarter note**, with a single leading
set-tempo meta of 500 000 µs per quarter (120 BPM) — so one second is exactly 960 ticks, and at
48 kHz one tick is exactly 50 samples. Running status is not used (every event carries its own
status byte); every message is on channel 0; each file ends with one End of Track meta. Note-offs
are real `0x8n` messages with velocity 0, never a zero-velocity note-on.

The phrases are committed artifacts, deliberately without a regeneration target: unlike the golden
IRs (`tests/data/golden/README.md`), which *must* be rewritable by the same renderer that later
compares against them, a corpus phrase that could be rewritten in place would violate rule 1. The
specification below is complete — every note, velocity and wheel movement — so any phrase can be
re-derived byte-for-byte from it plus the encoding conventions above, without the files themselves
being the only record of what they contain.

Pitch-wheel values map through `cnpg::dsp::pitchWheelToSemitones` (`MidiTranslation.h`): 8192 is
centre, 16383 is +2 semitones, 0 is −2 semitones. A normalized wheel value `v` in [−1, 1] below
encodes as `8192 + round(v · 8191)` for `v ≥ 0` and `8192 + round(v · 8192)` for `v < 0` — the
asymmetry is MIDI's, not a choice here (there is no exact 14-bit representation of full-scale up).

Wheel gestures are emitted on a uniform grid over their stated window, **endpoints inclusive**:
`n = round((t1 − t0) / 0.005)` intervals, so `n + 1` messages at `t0 + (t1 − t0)·i/n`.

- **wheel ramp** `from → to` — linear in `i/n`.
- **wheel sine** `depth, f` — `depth · sin(2π·f·(t − t0))`, so it starts at 0.
- **wheel triangle** `depth, f` — phased to start at 0 and rise: with
  `p = ((t − t0)·f + 0.25) mod 1`, the value is `depth · (1 − 4·|p − 0.5|)`, i.e.
  0 → +1 → 0 → −1 → 0 per cycle.

Both of those phasings are load-bearing rather than incidental. Phrase 05 exists to answer
checklist item 5 — "no zipper noise, **stepping**, or clicks anywhere" — so every wheel gesture in
it has to begin and end exactly where the previous one left the wheel. A triangle starting at its
own peak would have stepped the wheel instantaneously from centre to +2 semitones at 18.00 s, and
the vibrato's original 5.5 Hz over 3.30 s (18.15 cycles) would have ended mid-swing at
+0.81 semitones and made the wheel-to-centre message at 11.60 s an audible step. Measured after
the fix: across all seven wheel-only (note-on-free) windows in the phrase, the largest
sample-to-sample step sits a uniform 25.2–27.5 dB below that window's own peak — no window is an
outlier, which is what "no discontinuities" looks like objectively.

**Every phrase's first event is at 0.20 s, never at sample 0**, and that is deliberate. The first
block of any render is the one in which `TriodeStage`'s output-trim ramp travels from the struct
default (0 dB) to the shipped `kUnityGainOutputTrimDb` (−13.98 dB); that is production's real
cold-start behaviour, analysed at length in `docs/bench/p1-baseline.md`, and a note-on at sample 0
therefore passes through roughly 14 dB of extra gain for that one block. Measured: an earlier cut of
phrase 01 that started at 0.00 s peaked its first note at −11.1 dBFS against −20.2 dBFS for every
one of its neighbours. That behaviour is real and worth knowing about, but it is not what these
phrases exist to measure — a corpus whose first note is 9 dB hotter than the next one would corrupt
exactly the level and brightness comparisons checklist items 8 and 9 ask the author to make.

### `01_chromatic_singles.mid` — 60.500 s render, 198 events

- **A (0.20–31.00 s).** MIDI 21 through 108 inclusive, one note each: note-on at
  `0.20 + 0.35 · (note − 21)`, velocity 100, note-off 0.28 s later.
- **B (31.00–50.00 s).** MIDI 28, 45, 62, 79, 96 in turn: note-on velocity 100, note-off 3.0 s
  later, next note 3.8 s after the previous note-on.
- **C (50.00–59.00 s).** MIDI 45 six times: velocities 16, 40, 64, 88, 112, 127; note-on, note-off
  1.0 s later, next note 1.5 s after the previous note-on.

### `03_legato_retrigger.mid` — 23.100 s render, 42 events

- **A.** Note-on MIDI 45 velocity 100 at 0.20 s; same-pitch replucks (note-on, no note-off) at
  0.70/88, 1.10/104, 1.45/72, 1.75/112, 2.00/64, 2.20/96, 2.35/80; note-off at 4.60 s.
- **B (ascending slur, no note-offs between).** Note-on 45/96 at 5.50; then 47, 48, 50, 52 at
  velocity 90 at 5.90, 6.30, 6.70, 7.10; note-off 52 at 9.10.
- **C (descending slur).** Note-on 64/96 at 10.00; then 62, 60, 59, 57, 55 at velocity 88 at 10.30,
  10.60, 10.90, 11.20, 11.50; note-off 55 at 13.00.
- **D (trill).** 14 note-ons at velocity 92, 0.11 s apart from 13.60 s, alternating 52, 53, 52, …
  (the 14th is 53); note-off 53 at 16.83.
- **E.** Note-on 33/104 at 17.60; replucks 33/96 at 18.10, 18.60, 19.10; note-off at 21.10.

### `05_low_string_bends.mid` — 23.600 s render, 2683 events

Wheel centred at 0.00 s.

- **A.** Note-on 21/110 at 0.20. Wheel ramp 0 → +1 over 1.00–2.20; ramp +1 → 0 over 2.60–3.40;
  repluck 21/100 at 3.00; ramp 0 → −1 over 3.60–4.80; repluck 21/100 at 5.00; ramp −1 → 0 over
  5.20–6.00; note-off at 7.00.
- **B.** Note-on 28/104 at 7.50. Wheel sine, ±0.5 (±1 semitone) at 5.0 Hz, 7.70–11.00 (exactly
  16.5 cycles, so it ends at 0 as well as starting there); repluck 28/96 at 9.00; note-off at
  11.50; wheel to centre at 11.60 (a no-op reassertion, by construction).
- **C.** Note-on 40/108 at 12.20. Wheel ramp 0 → −1 over 12.50–14.00; repluck 40/100 at 14.50; ramp
  −1 → 0 over 15.00–16.50; note-off at 17.20.
- **D.** Note-on 21/110 at 17.80. Wheel triangle, full ±1 at 3 Hz, 18.00–21.00 (exactly 9 cycles,
  starting and ending at 0); note-off at 21.50;
  wheel to centre at 21.60.

### `07_param_sweeps_midnote.mid` + `.json` — 33.000 s render, 27 events

Four held notes, each a note-on at velocity 104, a run of same-pitch replucks at velocity 92, and a
note-off:

| Note | On | Replucks | Off | Sidecar sweep over it |
|---|---|---|---|---|
| MIDI 45 | 0.50 | 2.00, 3.50, 5.00, 6.50 | 8.00 | `pickupPosition01` 0.5 → 0.02 → 0.98 → 0.5 |
| MIDI 52 | 8.50 | 10.00, 11.50, 13.00, 14.50 | 16.00 | `pickupResonanceHz` 2500 → 300 → 8000 → 2500 with `pickupQ` 2 → 0.7 → 8 → 2 |
| MIDI 33 | 16.50 | 18.00, 19.50, 21.00, 22.50 | 24.00 | `triodeDrive` 0.5 → 0 → 1 → 0.5 |
| MIDI 57 | 24.50 | 25.30, 26.10, 26.90, 27.70, 28.50, 29.30, 30.10 | 31.00 | `materialLossGainLow`/`High` swept in opposition, `materialDispersionAmount` 0 → 1 → 0 |

The repluck spacing is per-section rather than uniform because the sweeps do not all leave the
string ringing for the same length of time. The material section in particular takes
`materialLossGainLow` down to 0.05, which at MIDI 57 kills the string in a few hundred
milliseconds; without the tighter repluck spacing the "extreme material" half of that sweep would be
judged on silence. Measured with one repluck per section, this phrase fell to −137 dBFS RMS in the
middle of the material sweep and −112 dBFS in the middle of the resonance sweep; with the spacing
above it stays between −20 and −57 dBFS RMS from 0.5 s to 31 s, then decays to digital silence in
the tail.

Exact breakpoint times and values are in `07_param_sweeps_midnote.json`, which is the authoritative
copy; the table above is a summary.

## Rendering the corpus

```
build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p1 --samplerate 48000 --blocksize 128
```

`renders/` is git-ignored: the WAVs are build products of the corpus plus a commit, reproducible at
any time from either.

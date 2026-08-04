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
3. **Render filenames carry that version, and a render-time content hash.** `cnpg_render --corpus`
   writes `<phrase stem>__cv<corpusVersion>_s<source hash>.wav`, the hash being 12 hex characters —
   `01_chromatic_singles__cv1_s3cca978b000b.wav` on the tree that landed Task P2.7's second fix wave
   — so a WAV on disk names both the input corpus and the exact code that rendered it
   (`docs/plan.md` section 4.8, as amended by Task P2.7's carry-forward C2: "Render filenames embed
   corpus version + a **render-time content hash** so listening notes are attributable").
   `cnpg_render` also prints the digest in its startup banner, so it is readable without parsing a
   filename.

   **The prefix is `_s`, never `_g`, and the change is not cosmetic.** The field used to be a
   configure-time `git rev-parse --short HEAD`, resolved when CMake last ran rather than when the
   binary was built or run — and that failed in the ordinary way, not a contrived one. Measured on
   this repository: a `build/` tree configured at `77b0430` produced renders of the code at
   `1ccfcb1` and filed them as `..._cv1_g77b0430.wav`, three commits stale. **Two different code
   states, one filename** — precisely the confusion this field exists to prevent, and the reason the
   argument that once stood here (that the configure-time hash "names the parent of the commit that
   landed the binary — close enough") is withdrawn: it names neither reliably.

   What replaced it is a SHA-256 digest computed at **render** time over `dsp/include`, `dsp/src`,
   `tests/support/P1Chain.h` and `tests/render` — the bytes a render is actually a function of —
   truncated to its first 12 hex characters. Two code states that differ in those bytes get
   different names *by construction*, which is exactly what the configure-time hash could not
   promise; what remains is only the ordinary birthday risk of a 48-bit digest, not a systematic
   collision every un-reconfigured tree walks into. It is verifiable from any checkout with no
   repository history and no git at all, and the recipe is published in `tests/support/SourceHash.h`
   (which also covers the golden sidecars' digest, computed by the same primitive over a smaller
   root set). If the source tree cannot be read the field reads `unknown` rather than failing the
   render. Deliberately **not** covered: the corpus itself — it has its own version in the same
   filename, and folding it in would rename every render whenever a phrase was added.

   **Any example of that digest goes stale, and that is the feature, not a defect in the example.**
   It is a digest of the *source*, not of the audio, so it changes whenever a covered byte changes —
   including a comment. Observed across Task P2.7's two fix waves, whose `dsp/` changes were comments
   only: `59d6b426d3f4` → `3cca978b000b`, with no audio-path byte moved and no golden moved — and the
   audio measured unchanged rather than assumed, all four phrases reproducing their RMS at the wave's
   parent commit to the printed digit (−34.29 / −32.63 / −38.65 / −34.95 dBFS, 48 kHz). The field is
   therefore a conservative
   over-approximation: **a changed digest means "different code", never by itself "different
   sound".** The direction it guarantees is the useful one — the same digest means the same covered
   bytes, which is what makes a listening note reproducible.

   A render filename does **not** carry a commit, and no listening note should claim it does. If a
   report wants one, record `git rev-parse HEAD` separately at render time.
4. **The manifest is checked, not trusted.** `durationSeconds` is verified against the actual
   rendered length on every `--corpus` run (10 ms tolerance) and the render fails if they disagree,
   so the manifest cannot quietly go stale against the files it describes.

### Current version: 2 (P1 — phrases 01, 03, 05, 07; P2 — phrases 02, 04, 06, 08)

Task P2.8 appended `02_open_chords.mid`, `04_palm_mute_chug.mid`, `06_sustain_chords.mid`, and
`08_harmonics_nodes.mid` + `.json`, and bumped `corpusVersion` to 2. It also added the
per-`RetriggerMode` variant renders of `03_legato_retrigger.mid` and the coupling / near-unison /
Normal-range comparison sets — those are *render configurations*, not new MIDI files, so they do not
change the corpus itself. Phrase 03 is rendered in the default (Physical) mode only in P1 because
the full Physical/Synth semantics need `DamperJunction`, which is Task P2.2.

### Per-phrase render configuration, and why the DEFAULTS are the P1 ones

A manifest entry may carry `numStrings` (1..8) and `allocationMode` (`"GuitarFingering"` or
`"FreeZones"`) alongside `retriggerMode` and `params`. **Both default to the P1 single-string,
full-range-`FreeZones` configuration, and that default is load-bearing rather than nostalgic.**
Rule 1 freezes a committed phrase's bytes, and an entry's fields are part of what "cv1 identifies
exactly one set of inputs" means, so the P1 entries do not state a count and the absence has to keep
meaning what it meant when they were written:

- **Phrase 01** is a chromatic run from MIDI 21 to 108. The shipped 6-string EADGBE fingering table
  spans 40..88, so under it 40 of those notes are **unassignable** and `cnpg_render` fails the
  render rather than quietly dropping them.
- **Phrase 03**'s legato slurs are retriggers *only while every note lands on the same string*. On
  six strings each slur note takes a free string instead, and there is no legato left to judge —
  which is exactly what the phrase exists for.

The P2 entries state their configuration explicitly, and for 02, 04 and 06 it is the plugin's
shipped default: **6 strings, `GuitarFingering`** over the default open notes 40/45/50/55/59/64.

**Phrase 08 is the exception, and the reason is a measurement.** It renders on **one** string with
the tap at `pickupPosition01` 0.87, because the item it exists for — checklist item 1, "the damper
at p = 1/2 suppresses the fundamental and leaves the octave" — is a *single-string* physical
statement that six coupled strings make unmeasurable. The neighbours ring at exactly the partials
under test — string 5's open E4 (329.6 Hz) **is** the 4th partial of the low E, and string 0's open
E2 is the 2nd partial of E3 — and an idle string's damper is *released*, because it never had a note
to release. Measured on the six-string cut of this phrase, second harmonic over fundamental at
p = 1/2, 1/3, 1/4, 1/5 and the 0.15 control:

| | p = 1/2 | 1/3 | 1/4 | 1/5 | 0.15 |
|---|---|---|---|---|---|
| six strings, MIDI 52 | +7.2 dB | +8.0 | +8.2 | +8.4 | +8.7 |
| six strings, MIDI 40 | **+1.9 dB** | +12.4 | +0.1 | +15.5 | −0.7 |
| **one string, tap 0.87, MIDI 40** | **+100.1 dB** | +18.2 | +9.0 | −30.0 | −31.7 |

The six-string MIDI 52 row varies by **1.5 dB across the whole sweep and shows no node structure at
all** — what it measures is the neighbours, not the node.

The tap moves off the default 0.5 for a second, independent reason: a tap at the midpoint sits on a
node of *every even partial*, so the octave the item asks for is suppressed by the pickup before the
damper is ever consulted. On one string with the tap at 0.87 the phrase reads the fundamental
**100.1 dB** below the 2nd harmonic at p = 1/2 and **120.2 dB** below the 3rd at p = 1/3, with the
loudest surviving partial being H4, H3, H4 and H5 at p = 1/2, 1/3, 1/4 and 1/5 respectively — each
one exactly the partial whose node the damper is sitting on — and the 0.15 control leaving the
fundamental loudest.

> **The 0.15 lane is a control, and it is no longer the shipping default.** It was when this phrase
> was authored, and the lane is retained unchanged at 0.15 so corpus version 2 stays re-derivable
> byte-for-byte — this phrase renders bit-identically before and after the default moved. What the
> lane demonstrates is unaffected: at this phrase's resolution 0.15 has no node below partial 20 and
> leaves the fundamental loudest. It is not nodeless, though — partial 20's node sits at
> 3/20 = 0.150 exactly — and that is the finding that moved the default to 1/25. See
> `.superpowers/sdd/2026-07-30-pm-guitar-synth-p0-p2-plan/task-damper-node-comb.md`.
>
> The same note applies to `04_palm_mute_chug`, which pins `damperPosition01 = 0.92`: by the
> symmetry of `sin^2(n*pi*p)` that is the mirror of 0.08, whose first node is partial 12.5. It too
> renders bit-identically and is deliberately left alone here.

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

Three parameter names have **two accepted spellings**. Task P2.1 renamed the APVTS ids
`materialLossGainLow` / `materialLossGainHigh` / `materialDispersionAmount` to
`stringMaterialLossGainLow` / `stringMaterialLossGainHigh` / `stringMaterialDispersionAmount`
(ADR 0004: `Material`/`Wood` is reserved for the *body* module a later phase adds). Corpus v1's
`07_param_sweeps_midnote.json` still carries the original spelling and **was deliberately not
rewritten** — the versioning rule above says an existing phrase's bytes never change, which is what
makes `cv1` in a render filename identify exactly one set of inputs. `cnpg_render` therefore
accepts both spellings (`tests/render/RenderMain.cpp`, `kParamNames`); new phrases use the
`stringMaterial*` form.

Not every accepted parameter name does something in P1. `damperPosition01` in particular is stored
by `StringNetworkParams` and was **inert** until `DamperJunction` landed in Task P2.2:
`cnpg_render` accepted it so a P2 phrase would not need a renderer change, and a P1 render did not
respond to it. From P2.2 it is live, and `08_harmonics_nodes.json` is the phrase that drives it.

Task P2.8 added the P2 half of the vocabulary, all of them shipped APVTS ids: `damperMaxLoss`,
`damperFeltTimeMs`, `bridgeCoupling`, `bridgeResonanceHz`, `bridgeDamping`, and
`stringTuningOffsetCents0` … `stringTuningOffsetCents7`.

**`retriggerMode` and `numStrings` are deliberately NOT lanes**, and the reason is recorded in
`tests/render/RenderMain.cpp` as well as here, because a stepped-lane implementation of both was
built during Task P2.8 and then removed. A `retriggerMode` lane has no technical obstacle but nothing
would drive it — the plan's deliverable is per-mode *variant renders*, which `kRenderVariants`
produces — and a `numStrings` lane's whole gesture (a count reduction under a held chord) **cannot**
be a corpus render at all: it makes the held notes' note-offs undeliverable, `NoteAllocator` counts
them on `unaddressableNoteOffCount()` by design, and `cnpg_render` fails the render on a non-zero
reading, also by design. Neither behaviour should be weakened to manufacture a listening artifact,
so that gesture stays a host check (`docs/listening/P2.6-ableton-checks.md` check B, which is
checklist item 16's only evidence). Both remain per-phrase manifest fields, which is where a value
that does not move belongs — as do `triodeBypass` and `cabBypass`, for the reason the P1 note above
gives.

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

### `02_open_chords.mid` — 46.000 s render, 120 events

Rendered on **6 strings, `GuitarFingering`**. Every chord below is a real open-position shape and
the allocator puts it where a player's hand would: E major arrives as 022100, G major as 320003.

- **A (0.20–20.50 s) — seven strummed open chords.** Each chord's notes are struck low to high,
  **0.012 s apart**, velocity **100**, and each note is released **2.30 s after its own note-on**.
  One chord every 3.00 s from 0.20 s:

  | at | chord | notes |
  |---|---|---|
  | 0.20 | E major (022100) | 40, 47, 52, 56, 59, 64 |
  | 3.20 | A major (x02220) | 45, 52, 57, 61, 64 |
  | 6.20 | D major (xx0232) | 50, 57, 62, 66 |
  | 9.20 | G major (320003) | 43, 47, 50, 55, 59, 67 |
  | 12.20 | C major (x32010) | 48, 52, 55, 60, 64 |
  | 15.20 | A minor (x02210) | 45, 52, 57, 60, 64 |
  | 18.20 | E minor (022000) | 40, 47, 52, 55, 59, 64 |

- **B (21.00–29.20 s) — sympathetic shimmer.** Note-on MIDI 40, velocity 108, at 21.00; note-off at
  29.20. Under it, twelve staccato notes at velocity **96**, each **0.12 s** long, starting at
  22.00 s and **0.60 s** apart: 64, 71, 67, 74, 59, 76, 62, 71, 79, 67, 83, 64.
- **C (30.00–40.00 s) — the near-unison pair.** Note-on 45/100 at 30.00 and 46/100 at 30.03, both
  released at 36.00; then note-on 46/104 at 36.60, released at 40.00. MIDI 45 lands on string 1
  (fret 0) and MIDI 46 on string 0 (fret 6) — the pair ADR 0007 D7.0 measured. The second half is
  the *purely sympathetic* case: string 0 alone, with string 1 resting at its open pitch.
- **D (41.00–44.00 s) — the steal.** Six notes at 41.00 + 0.012·i, velocity 100: 40, 47, 52, 56, 59,
  64. Then note-on 69/100 at 42.50, which finds every candidate string owned and displaces the
  least-recently-triggered one. All seven note-offs at 44.00; the displaced note's is stale and is
  dropped, by design.

### `04_palm_mute_chug.mid` — 23.830 s render, 234 events

Rendered on **6 strings, `GuitarFingering`**, with `damperPosition01` **0.92**, `damperMaxLoss`
**0.45** and `damperFeltTimeMs` **20**. That combination *is* the palm mute on this instrument: the
damper sits near the bridge, is only partly absorbing (so a muted note is a short **pitched** thump
rather than a dead click), and engages fast.

- **A (0.20 s).** 16 note-ons on MIDI 40, velocity 110, **0.16 s** apart, each released **0.055 s**
  after its own note-on.
- **B (3.20 s).** 24 note-ons on MIDI 40, velocity 104, **0.09 s** apart, each 0.045 s long.
- **C (6.00 s).** 12 power-chord chugs: MIDI 40 **and** 47 together at velocity 108, **0.18 s**
  apart, each 0.060 s long.
- **D (8.60 s) — gallop.** Eight repetitions of long-short-short on MIDI 40, velocity 106: note
  lengths 0.075 / 0.045 / 0.045 s at step sizes 0.24 / 0.12 / 0.12 s.
- **E (12.60 s) — the re-strike gap ladder (checklist item 21).** On MIDI 43, velocity 108, notes
  0.180 s long. Four notes at each of five gaps between one note-off and the next note-on —
  **0.005, 0.012, 0.021, 0.053, 0.100 s** — with 0.50 s of silence between groups. Those are the
  four ages the headless gate measured (19.69 / 21.48 / 16.64 / 0.30 dB of click excess) plus the
  100 ms point where it reads 0.
- **F (19.00 s) — chugs into a let-ring.** 8 chugs on MIDI 45, velocity 108, 0.14 s apart, 0.050 s
  long; then note-on 45/112 at 20.33, released at 21.83.

### `06_sustain_chords.mid` — 31.000 s render, 98 events

Rendered on **6 strings, `GuitarFingering`**, shipped defaults otherwise. Every chord is strummed
low to high **0.012 s** apart at the stated velocity, and every note of a chord is released on the
one stated instant. CC64 is controller 64 on channel 0; `NoteAllocator` reads ≥ 64 as pedal-down.

- **A.** CC64 = 127 at 0.20. Chord {40, 47, 52, 56, 59, 64} velocity 100 at 0.40, keys released at
  1.60 (held by the pedal). CC64 = 0 at 5.00.
- **B.** CC64 = 127 at 6.00. {45, 52, 57, 60, 64} velocity 100 at 6.20, keys up at 7.20;
  {48, 52, 55, 60, 64} velocity 100 at 8.00 **over** it, keys up at 9.20. CC64 = 0 at 12.00.
- **C.** CC64 = 127 at 13.00. Note-on 45/104 at 13.20, note-off at 14.00 (held); note-on 45/104
  again at 14.60, which **cancels** that held note-off. CC64 = 0 at 16.50 — the note must not damp.
  Note-off 45 at 18.00.
- **D — the half-pedal sweep.** CC64 = 127 at 18.90; chord {50, 57, 62, 66} velocity 100 at 19.00,
  keys up at 20.20. Then CC64 stepped **down** from 21.00 s, 0.05 s apart, values 120, 112, …, 0
  (16 messages, `120 − 8·i`), and **up** from 22.00 s, same spacing, values 0, 8, …, 120; CC64 = 0 at
  23.20. Exactly one threshold crossing each way.
- **E.** CC64 = 127 at 24.50. Chord {40, 47, 52, 56, 59, 64} velocity 104 at 24.60, keys up at
  26.00. CC64 = 0 at 29.00 — six dampers landing on one sample.

### `08_harmonics_nodes.mid` + `.json` — 31.900 s render, 20 events

Rendered on **one** string with `pickupPosition01` **0.87** — see "Per-phrase render configuration"
above for the measurement that forced both.

Ten notes, velocity **104**, each released **0.90 s** after its own note-on, one every **3.20 s**
from 0.20 s. The first five are MIDI **40**, the second five MIDI **45**. The sidecar drives
`damperPosition01` through **1/2, 1/3, 1/4, 1/5, 0.15** for each note in that order, moving between
values with a **50 ms** ramp that ends **0.30 s before** the note-on — i.e. entirely inside a silent
gap, so the damper is already at the node when the note-off engages it and this phrase never sweeps
a position under a ringing string (that is phrase 07's job). The lane holds flat everywhere else;
that is why each node has *two* breakpoints rather than one.

`08_harmonics_nodes.json` is the authoritative copy of the times and values.

## Rendering the corpus

```
build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p1 --samplerate 48000 --blocksize 128
build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p2 --rates 44100,48000,96000
build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p2 --samplerate 48000 --variants
```

**`--samplerate` writes flat into `--out`; `--rates` writes `<out>/<rate>/`.** A render filename
carries the phrase, the variant, the corpus version and the source digest, and deliberately not the
sample rate (rule 3) — so three rates in one directory would be three renders with one name and only
the last would survive. Measured on the first cut of `--rates`: it produced 8 files, all of them
96 kHz.

`--variants` adds the built-in P2 comparison configurations (`tests/render/RenderMain.cpp`,
`kRenderVariants`): the per-`RetriggerMode` pair over phrase 03, the `couplingStrength` ladder and
the near-unison ladder over phrase 02, and the settings at and outside the provisional Normal range.
Their filenames carry the variant name ahead of the `__cv<n>_s<hash>` tail, which stays terminal so
the checklist's step 4 still reads the corpus version and digest off the end.

`--bridge-phase-blind` attaches a bridge port that forwards everything to a real `BridgeJunction` and
reports **zero** reflection phase delay — Task P2.4's instrument, the negative control for P2.7's
compensation. It is not the instrument and no listening verdict may be recorded against it.

`renders/` is git-ignored: the WAVs are build products of the corpus plus a commit, reproducible at
any time from either.

# Physical-plausibility checklist

The human half of the hybrid milestone gate (`docs/plan.md` Q16 and section 4.8). CI enforces the
objective proxies — pitch within ±2 cents, damper-at-node harmonic suppression, click/NaN/denormal-
free automated parameter sweeps — continuously and regardless of this document. This checklist is
the layer on top: the questions a measurement cannot answer, asked of the actual rendered audio, by
the author, once per milestone.

**Versioned, and appended to rather than rewritten.** This is the P1 revision, created by Task
P1.11. Task P2.8 appends its own items (sympathetic coupling, palm-mute chug, pedal-held chords,
retrigger-mode contrast). Items are never renumbered: `corpus.json` cites them by number, and so do
past listening reports.

## Procedure

1. Render the corpus:
   `build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p1 --samplerate 48000 --blocksize 128`
2. Listen on monitoring you name in the report — the device matters to the verdicts and an unnamed
   device makes a "concern" unreproducible.
3. Give **every** item exactly one verdict: **pass** / **concern** / **fail**. Cite the WAV filename
   and a timestamp for every concern and every fail. An item with no verdict is not a pass; it
   blocks the milestone until it has one.
4. Commit the filled report to `docs/listening/P<phase>-<yyyymmdd>.md`, recording the corpus version
   and the git hash (both are in the render filenames).
5. Any **fail** blocks milestone closure until it is fixed and re-rendered, or explicitly re-scoped
   with written rationale in the same report. Any **concern** needs either a linked GitHub issue or
   a sentence saying why it is acceptable to ship past.

## Read this before judging drive (RC2, user-mandated)

`TriodeStage` is a **static waveshaper**: one memoryless input-voltage → output-voltage curve, no
state carried between samples, therefore no bias drift, no coupling-cap high-pass behaviour, no
cathode-bypass corner interacting with signal level, no thermal settling
(`dsp/include/cnpg/dsp/TriodeStage.h`, and `docs/plan.md` line 50). **The P1 chain is explicitly not
a voicing reference for drive feel.** Judge item 12 for artifacts — is the coloration smooth,
click-free, and stable as drive moves — and not for whether the amp "feels" right. Drive-feel
conclusions are deferred until a dynamic stage lands in P3+. This paragraph is restated verbatim in
every listening report and in the P2 exit sign-off.

## Items

Items 1–12 are the locked P1 set (`docs/plan.md` section 4.8). Items 1, 2, 4, 6 and 10 depend on
machinery that does not exist in P1 — `DamperJunction` (P2.2), `BridgeJunction` (P2.4), CC64 sustain
and the Physical/Synth retrigger split (P2.6), multi-string allocation (P2.1) — so in a P1 report
they are marked **n/a (P2)** rather than passed. They are listed here anyway because the checklist is
one document across milestones, not one per milestone, and a P2 reader must be able to see that a P1
report deliberately did not answer them.

| # | Item | What to listen for | First judged |
|---|---|---|---|
| 1 | **Harmonics at nodes** | Damper at p = 1/2 suppresses the fundamental and leaves the 2nd harmonic ringing (octave); p = 1/3 yields the 12th. Also CI-enforced: FFT of phrase 08 shows the fundamental ≥ 20 dB below the 2nd harmonic at p = 1/2. | P2 |
| 2 | **Palm-mute character** | Muted notes are short, *pitched* thumps — not clicks, not unpitched thuds. Chugging at speed stays tight with no energy buildup between hits. | P2 |
| 3 | **Retrigger, Physical** | A same-pitch repluck sounds like picking an already-sounding string. A pitch change chokes briefly, then re-excites with plausible legato. No pops. | P1 |
| 4 | **Retrigger, Synth** | Instant clean restart at the new pitch. No click, no residual old pitch. | P2 |
| 5 | **Bend cleanliness** | Bends glide smoothly. No zipper noise, no stepping, no clicks anywhere in phrase 05 — including at the bend extremes and through the direction reversals. | P1 |
| 6 | **Chord density under sustain** | Phrase 06 stays stable and separable. No runaway level; output stays below the `SoftClipLimiter` ceiling without audible pumping. | P2 |
| 7 | **Pluck position audibility** | Near-bridge plucks are noticeably brighter and thinner; near-middle plucks are hollow, with suppressed even harmonics. | P1 |
| 8 | **Velocity/hardness response** | Harder and faster playing gets *brighter* as well as louder — not merely scaled in level. | P1 |
| 9 | **Decay realism** | High notes die faster than low notes. Nothing terminates abruptly, and nothing rings implausibly forever. | P1 |
| 10 | **Moving positions mid-note** | Pickup and damper position sweeps in phrase 07 are click-free and continuous. (The pickup half is judgeable in P1; the damper half is P2.) | P1 (pickup) / P2 (damper) |
| 11 | **Silence hygiene** | Gaps between phrases decay to digital silence. No denormal fizz, no stuck resonance, no low-level hum or DC thump at note boundaries. | P1 |
| 12 | **Abuse survival** | Nothing in phrases 04–07 produces NaN blasts, stuck notes, or level runaways. Coloration under the drive sweep stays smooth and stable — see the RC2 note above for what this item does *not* ask. | P1 |

### Items added after the locked P1 set

These are not in `docs/plan.md` section 4.8's list of twelve. They were added when the task that
created the behaviour also created a question a measurement cannot answer, and each names the task
that added it. First judged at the P2 listening pass (Task P2.8).

| # | Item | What to listen for | Added by |
|---|---|---|---|
| 13 | **Slow pickup automation continuity** | The effective read position is hysteretically quantised to ~1/32 of the string, so a slow pickup sweep moves the comb in ~32 discrete crossfaded steps rather than continuously. Does it sound continuous, or stepped? | P2.3 |
| 14 | **Sympathetic resonance** | Hold a low note, play staccato notes on other strings: the held string should shimmer sympathetically. Raise `Bridge Coupling` and it should get brighter and more coupled — and shorter. Is the shipping default (0.35) the right amount of instrument, or does it want more or less? | P2.4 |
| 15 | **Unison-adjacent voicings and mode locking** | Two strings a few cents apart on a shared bridge PULL TOGETHER: at the shipping coupling a 25-cent detune collapses to a measured 0.003 cents of separation, and the string that was *not* detuned is dragged +20.3 cents off its own nominal. That is real coupled-string physics (Weinreich) and it is also, potentially, an instrument that will not stay where the tuner put it. Play a deliberately spread unison and a close voicing: does it sound like a real instrument locking, or like a bug? | P2.4 |
| 16 | **String count and decay** | A string that is disabled or idle presents a zero wave at its bridge port, which makes it a perfect absorber there — so the bridge's effective damping depends on how many strings are switched on. Does changing the string count audibly change the decay of the strings that stay? | P2.4 |

## Which P1 phrase is the evidence for which item

| Item | Primary evidence in the P1 corpus |
|---|---|
| 3 | `03_legato_retrigger` — replucks at 0.7–2.4 s, slurs at 5.5–13.0 s, trill at 13.6–15.0 s |
| 5 | `05_low_string_bends` — whole file; the 3 Hz full-range triangle at 18.0–21.0 s is the worst case |
| 7 | `07_param_sweeps_midnote` — 0.5–8.0 s (`pickupPosition01` 0.5 → 0.02 → 0.98 → 0.5) |
| 8 | `01_chromatic_singles` — 50.0–59.0 s (one pitch at six velocities) |
| 9 | `01_chromatic_singles` — 31.0–50.0 s (five sustained anchors) |
| 10 | `07_param_sweeps_midnote` — 0.5–8.0 s; the damper half of the item waits for P2 |
| 11 | `01_chromatic_singles` gaps throughout; every phrase's own 2 s tail |
| 12 | `05_low_string_bends` and `07_param_sweeps_midnote` in full |

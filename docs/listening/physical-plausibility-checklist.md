# Physical-plausibility checklist

The human half of the hybrid milestone gate (`docs/plan.md` Q16 and section 4.8). CI enforces the
objective proxies — pitch within ±2 cents, damper-at-node harmonic suppression, click/NaN/denormal-
free automated parameter sweeps — continuously and regardless of this document. This checklist is
the layer on top: the questions a measurement cannot answer, asked of the actual rendered audio, by
the author, once per milestone.

**Versioned, and appended to rather than rewritten.** Created by Task P1.11 as the P1 revision;
**this is the P2 revision, appended by Task P2.8** (items 22-27, plus the corpus-v2 evidence table
at the end). Items are never renumbered: `corpus.json` cites them by number, and so do past
listening reports.

## Procedure

1. Render the corpus. P1 used one command; from corpus version 2 there are three, and the third is
   what the P2 comparison items (15, 22, 26, 27) are judged from:

   ```
   build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p1 --samplerate 48000 --blocksize 128
   build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p2 --rates 44100,48000,96000
   build\bin\Release\cnpg_render.exe --corpus tests\corpus --out renders\p2 --samplerate 48000 --variants
   ```

   `--samplerate` writes flat into `--out`; `--rates` writes one subdirectory per rate, because the
   filename carries no rate (`tests/corpus/README.md`, "Rendering the corpus").
2. Listen on monitoring you name in the report — the device matters to the verdicts and an unnamed
   device makes a "concern" unreproducible.
3. Give **every** item exactly one verdict: **pass** / **concern** / **fail**. Cite the WAV filename
   and a timestamp for every concern and every fail. An item with no verdict is not a pass; it
   blocks the milestone until it has one.
4. Commit the filled report to `docs/listening/P<phase>-<yyyymmdd>.md`, recording the corpus version
   and the **render-time source hash**. Both are in the render filenames, as `__cv<version>_s<hash>`
   — a render named `01_chromatic_singles__cv1_s3cca978b000b.wav` gives corpus version `1` and
   source hash `3cca978b000b`. `cnpg_render` prints the same digest in its startup banner, so it can
   be copied from the console rather than reconstructed from a filename.

   **That field is a content digest over the DSP sources, not a commit** (`_s`, never `_g`). Task
   P2.7 removed the configure-time git hash that used to sit there: it named the commit CMake last
   ran at rather than the code that produced the audio, and was measured filing renders of `1ccfcb1`
   under the name `_g77b0430` — two code states, one filename (`tests/corpus/README.md` rule 3,
   `tests/support/SourceHash.h`). The digest is the stronger provenance, because it is checkable
   from any checkout with no repository history at all. A render whose source tree could not be read
   stamps `unknown`; if a report shows that, the provenance is missing and the render should be
   redone from a real checkout. If the report also wants a commit, record `git rev-parse HEAD`
   separately at render time — **the filename does not carry one.**

   **Do not read a changed digest as changed audio.** It covers source bytes, comments included, so
   it is a deliberate over-approximation. Across Task P2.7's two fix waves it moved `59d6b426d3f4` →
   `3cca978b000b` on changes to `dsp/` that were **comments only**, and the audio was measured rather
   than assumed unchanged: all four corpus phrases reproduce their RMS at the wave's parent commit to
   the printed digit (−34.29 / −32.63 / −38.65 / −34.95 dBFS at 48 kHz), no golden moved, and the
   `[regression]` suite that re-renders against them is green. A *matching* digest is the strong
   claim (identical covered bytes); a differing one says only that the code differs somewhere, not
   that the instrument sounds different.
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

**THE SENTENCE THAT USED TO STAND HERE IS OBSOLETE AS OF CORPUS VERSION 2, AND IT MATTERED.** It
read: "Items 1, 2, 4, 6 and 10's P2 halves, and items 14–20, are all judged from a HOST rather than
from the corpus: the corpus renders through `cnpg_render`, which is still the single-string P1
chain, so nothing in it exercises allocation, chords, the pedal, or the coupled bridge under six
real voices." Task P2.8 made that false: phrases 02, 04 and 06 render on **six strings through
`GuitarFingering`** with the loaded bridge and real dampers, so allocation, chords, the pedal and
sympathetic coupling are all in the corpus now.

**Exactly one of those items still needs a host, and it is item 16** — a string-count change under a
ringing chord. Nothing in the corpus moves `numStrings` (the renderer can automate it, but no phrase
does), so `docs/listening/P2.6-ableton-checks.md` check B remains the only evidence for it. Items
18, 19 and 21 are now judgeable from a render *as well as* from a host, and the host version is
still the better one for 18 (a moving progression is a gesture, not a phrase).
`docs/listening/P2.6-ableton-checks.md` is the executable form of that session — the two manual
checks deferred since Task P2.1 plus the retrigger and pedal items — and it should be run before or
alongside the P2.8 pass rather than after it.

| # | Item | What to listen for | Added by |
|---|---|---|---|
| 13 | **Slow pickup automation continuity** | The effective read position is hysteretically quantised to ~1/32 of the string, so a slow pickup sweep moves the comb in ~32 discrete crossfaded steps rather than continuously. Does it sound continuous, or stepped? | P2.3 |
| 14 | **Sympathetic resonance** | Hold a low note, play staccato notes on other strings: the held string should shimmer sympathetically. Raise `Bridge Coupling` and it should get brighter and more coupled — and shorter. Is the shipping default (0.35) the right amount of instrument, or does it want more or less? | P2.4 |
| 15 | **Unison-adjacent voicings and mode locking** | Two strings a few cents apart on a shared bridge PULL TOGETHER: at the shipping coupling a 25-cent detune collapses to a measured 0.003 cents of separation, and the string that was *not* detuned is dragged +20.3 cents off its own nominal. That is real coupled-string physics (Weinreich) and it is also, potentially, an instrument that will not stay where the tuner put it. Play a deliberately spread unison and a close voicing: does it sound like a real instrument locking, or like a bug? | P2.4 |
| 16 | **String count and decay** | A string that is disabled or idle presents a zero wave at its bridge port, which makes it a perfect absorber there — so the bridge's effective damping depends on how many strings are switched on. Does changing the string count audibly change the decay of the strings that stay? | P2.4 |
| 17 | **Legato glide length** | A pitch-changing retrigger in Physical mode keeps the rails and glides f0 to the new note over 30 ms — the plan's own figure. **This item decides the number, and the headless gate does not.** The click statistic's reference is a fresh pluck, which contains no glide at all, so every millisecond of glide reads as excess by construction and a longer ramp always scores better: its optimum is "no legato". It refuses a ramp so short the retune is a step (2 ms reads 18.7 dB, 8 ms reads 10.1 dB, 16 ms reads 6.0 dB against a 3 dB criterion) and says nothing useful above that: 22 ms reads 1.18 dB and **passes**, 28 ms reads 3.59 dB and **fails**, 30 ms reads 0.90 dB — the statistic is not monotone and cannot order ramp lengths. (Every figure here moved slightly at fixes wave 3, when the re-strike stopped being struck at a round block boundary and started being placed on the loudest sample of the old note's cycle; the shape and the conclusions are unchanged.) So: does 30 ms read as a hammer-on, or as a slide? If it is a slide, anything from about 18 ms up is available without re-opening the click question, and the knob is `StringNetwork::setRetuneRampSeconds` (deliberately not on the APVTS surface yet). | P2.6 |
| 18 | **Fingering plausibility** | With `Allocation Mode` = Guitar Fingering, do chords land where a player's hand would put them? The rule is "lowest fret among strings that own no note, ties to least-recently-used", and on an open C major it produces x32010 unprompted. Does a moving chord progression stay plausible, or does it wander onto implausible strings as notes are held and released? | P2.6 |
| 19 | **Steals** | Play more simultaneous notes than there are strings. A steal displaces the note struck longest ago, with no synthesized note-off and no click. Does that sound like a guitarist running out of strings, or like a bug? Also: a note the fingering table cannot reach (below the low E, above the 24th fret) is DROPPED — silence, not a transposition. Is silence the right answer to play? | P2.6 |
| 20 | **Sustain pedal** | With CC64 held, notes ring past their key releases and damp together on pedal-up. Six dampers landing on one sample measure −1.07 dB of click excess against a 3 dB criterion — is that inaudible in a dense held chord on a coupled bridge, where the six released strings are all still feeding each other? | P2.6 |
| 21 | **Re-striking just after a note-off (P2.6)** | A released note is over: a NoteOn landing on a string whose note was released CLEARS that string and plucks it fresh. If it lands soon after the note-off, the tail it discards is still loud, and the clear is a one-sample step. **Measured, worst case within half a cycle of each age: 5.5 ms → 19.69 dB of click excess, 12.4 ms → 21.48 dB, 21.3 ms → 16.64 dB, 52.7 ms → 0.30 dB, and 0 dB from 100 ms up, all against a 3 dB criterion whose level-placed hard-cut control reads 30.74 dB.** So the headless gate says this IS a discontinuity below about 50 ms and says nothing about whether anyone hears it under a fresh full-velocity pluck landing on the same sample. Play fast repeated notes and a staccato line with short gaps: does a re-strike 5–20 ms after a release tick, or does the new attack mask it? **This is pre-P2.6 behaviour that nothing ever measured, not a new defect** — but it is now measured, and if the answer is "it ticks", the fix is a short fade before the clear on the fresh path over a string that still has state, at the cost of moving the corpus renders off the P1 parent. | P2.6 |

### The six P2 items (`docs/plan.md` Task P2.8), added by Task P2.8

These are the six the P2.8 task definition names, in its words. Each of them **overlaps an earlier
item and is not a duplicate of it**, and the "what it adds" column says exactly what the overlap is,
because judging the same thing twice is how a checklist stops being read. Items 22–27 are judged
from the **corpus renders**, which as of corpus version 2 are six real strings on a loaded bridge —
so unlike items 14–20 they do not need a host.

| # | Item | What to listen for | What it adds over the earlier item | Added by |
|---|---|---|---|---|
| 22 | **Sympathetic coupling is audible and musical — and at what strength** | The `coupling*` ladder over phrase 02: 0.00 / 0.10 / 0.20 / 0.30 / 0.32 / 0.35. Section B (21–29 s) is the held low E under staccato notes. Which value makes the instrument sound like an instrument? Headless: beat depth 10.08 / 3.59 / 2.28 / 1.62 dB at coupling 0.1 / 0.35 / 0.5 / 1.0 — richness FALLS as coupling rises. | Item 14 asks "is 0.35 right?" with a knob in a host and no anchor. This is a fixed six-render A/B whose values are all measured boundary points, and **it is the item that decides the shipped default** (ADR 0007 D4). | P2.8 |
| 23 | **Palm-mute chug damps convincingly without machine-gun artifacts** | Phrase 04, sections A–D. Do 16 identical note-ons sound like sixteen *picks*, or like one sample retriggered? Listen for identical attack transients, and for level creeping up across a run (energy the damper failed to remove). | Item 2 asks whether a muted note is a pitched thump. This asks whether a *sequence* of them is playable — the sameness artifact only exists in the repetition, and item 2 can be passed by a single good chug. | P2.8 |
| 24 | **Bends stay in tune and click-free at the extremes** | Phrase 05, whole file, on the P2 tree. The bend extremes now sit on a bridge whose phase-delay compensation **re-solves every block** as the wheel moves (ADR 0007 D9), and the corpus render's pitch moved up to ~5 cents at P2.7. Does the wheel still glide, and does the pitch still arrive where the wheel says? | Item 5 was judged in P1 on one string against a rigid termination, where nothing re-solved. The compensation under a moving wheel is new machinery on old material. | P2.8 |
| 25 | **Pedal-held chords ring and release together** | Phrase 06, all five sections. Section E is six dampers landing on one sample (headless: −1.07 dB of click excess against a 3 dB criterion). Section C is a re-strike under the pedal that must survive the pedal-up. Section D is the half-pedal sweep: exactly one damp each way, no stutter. | Item 20 is the host check with a real pedal. This is the recorded artifact, so a concern here is reproducible from a file rather than from a gesture. | P2.8 |
| 26 | **Mid-note sweeps morph without zipper noise** | Phrase 07 (pickup, resonance, drive, material) **plus** phrase 08's damper-position steps **plus** the `range*` renders' bridge settings. Any stepping, any grain, any change that arrives as an event rather than as a motion. | Item 10's damper half was deferred to P2 and item 13's hysteretic pickup quantisation was never judged. The bridge parameters are new at P2.7 and their live-change gate measured −0.057 / −0.033 / +0.115 dB — clean by the metric, unjudged by ear. | P2.8 |
| 27 | **Physical vs Synth retrigger characters are distinct** | The two `retrig*` renders of phrase 03, back to back. Sections B and C are the slurs. Could a listener tell which is selected without being told? | Items 3 and 4 judge each mode **alone**, and each can pass while the pair is indistinguishable. This is the contrast, and it is the one thing two separate verdicts cannot express. | P2.8 |

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

## …and which corpus-v2 render is the evidence for which P2 item (Task P2.8)

| Item | Primary evidence |
|---|---|
| 1 | `08_harmonics_nodes` — ten note-offs onto p = 1/2, 1/3, 1/4, 1/5 and the 0.15 control |
| 2, 23 | `04_palm_mute_chug` — A/B/C/D at 0.2–12.4 s; F (19.0–21.8 s) is the mute against a let-ring |
| 3, 4, 27 | `03_legato_retrigger__retrigPhysical` and `…__retrigSynth` — slurs at 5.5–13.0 s |
| 6, 20, 25 | `06_sustain_chords` — A 0.2–5.0 s, B 6.0–12.0 s, C 13.0–18.0 s, D 18.9–23.2 s, E 24.5–29.0 s |
| 14, 22 | `02_open_chords__coupling000/010/020/030/032/035` — section B, 21.0–29.2 s |
| 15 | `02_open_chords__unison000/010/020/030/032/035` — section C, 30.0–40.0 s |
| 16 | Host only (`P2.6-ableton-checks.md` check B). No corpus render moves the string count. |
| 17 | `03_legato_retrigger__retrigPhysical` — section B, 5.5–9.1 s |
| 18, 19 | `02_open_chords` — section A for the fingering, section D at 41.0–44.0 s for the steal |
| 21 | `04_palm_mute_chug` — section E, 12.6–18.5 s, the 5/12/21/53/100 ms gap ladder |
| 24 | `05_low_string_bends` — whole file, on the P2 tree |
| 26 | `07_param_sweeps_midnote`, `08_harmonics_nodes`, and the `02_open_chords__range*` renders |

### The P1-set items, re-stated against corpus version 2 (appended by Task P2.8 fixes wave 1)

The table two sections up is the **P1 corpus**'s mapping and stays as written. These are the same
items against the **version 2** renders, added because `docs/listening/P2-20260803.md` §7 makes all
of 1–27 blocking and eight of them had no row in that sheet. Nothing here renumbers or rewrites an
item; it names evidence.

| Item | Primary evidence in corpus version 2 |
|---|---|
| 5 | `05_low_string_bends` — whole file; section D (18.0–21.0 s), the 3 Hz full-range triangle, is the worst case. Same file as item 24, different question |
| 7 | `07_param_sweeps_midnote` — 0.5–8.0 s, for the **pickup** half. **See the note below for the pluck half** |
| 8 | `01_chromatic_singles` — 50.0–59.0 s (MIDI 45 at six velocities) |
| 9 | `01_chromatic_singles` — 31.0–50.0 s (five 3 s anchors, MIDI 28/45/62/79/96) |
| 10 | `07_param_sweeps_midnote` — 0.5–8.0 s, for the **pickup** half. **The damper half has NO corpus evidence — see the note below** |
| 11 | Every phrase's own 2 s tail; `01_chromatic_singles`'s 87 gaps of 70 ms and four gaps of 800 ms |
| 12 | `04`, `05`, `06`, `07` in full |
| 13 | `07_param_sweeps_midnote` — 0.5–8.0 s. **First judged at the P2 pass; it has never been judged before** |

**TWO HALVES OF TWO ITEMS HAVE NO CORPUS EVIDENCE AT ALL, and the P2 revision above was wrong to
imply otherwise.** Item 10's text says "pickup and damper position sweeps in phrase 07"; phrase 07's
sidecar has **no damper lane** (its four lanes are `pickupPosition01`, `pickupResonanceHz`/`pickupQ`,
`triodeDrive`, and the material trio), and phrase 08 moves `damperPosition01` only **inside silent
gaps**, deliberately, so that the position is already settled when the note-off engages it. Item 7 is
titled "pluck position" but every render leaves `Exciter Position` at its 0.5 default and no variant
overrides it; what the corpus sweeps is the **pickup**. Both halves are therefore host gestures — one
knob each — and `docs/listening/P2-20260803.md` §4.2 (b) and (c) write them out. **The rest of both
items is judgeable from a render.**

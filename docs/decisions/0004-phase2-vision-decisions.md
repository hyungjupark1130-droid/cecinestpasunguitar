# ADR 0004 — Post-P2 vision: locked decisions

- **Status:** Accepted
- **Date:** 2026-08-01
- **Decided by:** project author, in response to the multi-lens vision audit
- **Supersedes:** the non-binding P3/P4/P5 outlines in the approved P0–P2 plan (§7)

## Context

The author specified a large feature vision (pickup types, wood/material, resonance chamber,
ADSR, six amp voicings with variable tube count and type, cabinets, optical/FET compressors,
a two-tab monochrome GUI, a preset bank). A five-lens audit against the shipped code produced
40 candidate questions, 51 conflict findings and 67 observations, consolidated to 13 decisions
and 15 real collisions. The decisions below are locked; the roadmap they imply is P3–P10.

## Decisions

### D1 — Pickup cardinality
Widen the sample→block domain boundary to `(string, tap)` with **`kMaxTapsPerString = 4`
preallocated and exactly one active at the P2 exit gate**. Each string gets its own position
smoother, replacing the single global `pickupSmoothed_`. **The widening must not alter any
current output sample.** Mirrors the proven `kMaxStrings = 8` / `setNumStrings(1)` pattern.

The humbucker is eventually a **true two-coil construction** — two spatial string taps with
real coil spacing, aperture, polarity and per-coil behaviour — **not** a downstream voicing
preset. Consequence: the comb null at `f = v/2d` is emergent and is a free acceptance criterion.

*Rationale:* the audit corrected an error in the controller's briefing — `StringNetwork` calls
`readTapAt()` exactly once per string per sample and `StringTapBuffers::channel()` carries no
tap index. This is the only vision item that is **cheaper inside the in-flight plan**: P2.1 is
already rewriting that storage into SoA and P2.3 is already writing the click-free crossfade.

### D2 — Envelope semantics differ by engine mode
**One** Envelope module whose *destination* changes with the engine mode. **No** third
retrigger mode, and no redefinition of the shipped `RetriggerMode::Synth` parameter state.

- **Physical mode:** envelope drives **loop loss and damper engagement**. Sustain is the
  held-stage **decay rate**, not a level (on a decaying string a level-Sustain is inert).
- **Synth mode:** conventional ADSR over an **output VCA**.

Locked Q1 (one-shot pluck; sustained excitation deferred to the feedback bus) and locked Q3
(two retrigger states) both remain intact. Physical-mode routing requires amending locked Q9's
*shape*: `StringNetworkParams::PerString` gains at minimum an envelope-scaling scalar, because
strings are independently triggered and a note-on must not alter other ringing strings' decay.
A faster parameter path into `WaveguideString` is required (existing smoothers are 8–25 ms and
would erase a snappy attack) and needs its own click test.

### D3 — Six amp directions, shared components
Retain **six distinct amp directions**; they are part of the intended sound. They do **not**
require six DSP codebases: stage components are shared, but each model may define its own
stage count, interstage networks, tone-stack topology **and placement**, feedback,
phase-inverter behaviour and power-stage configuration.

Product-derived names never appear in UI, parameter IDs, preset names or saved state
(see D3a). Scope note carried forward: the binding budget is **author listening time**, one
voicing at a time — not CPU (~40× headroom measured).

### D3a — Naming policy (binding, applies to all future work)
Shipped labels use the author's neutral system: `NAME — CATEGORY / YEAR`, e.g.
`GLASS CURRENT — US CLEAN / 65`, `GREEN CONE 25`. Tube type designations (12AX7, 12AU7, ECC83)
and functional control names ("Peak Reduction", "Attack", "All Buttons") stay verbatim —
generic and already in use. **Product designations appear only in source comments and design
notes as archetype provenance**, never as a claim of emulation. A NOTICE file disclaims
affiliation. Rationale: GPLv3 grants no trademark license, the repo is public, and the vendor
identity is a named individual; preset names persist into saved state, so this is free now and
a migration later.

### D4 — Unlock Q7 (tone stack) by ADR at P6
Tone-stack topology and placement is the largest differentiator among the amp directions.
Q7 ("no tone stack before P4") is unlocked deliberately at the valve-cascade phase, recorded
in its own ADR. Until then, no amp-voicing listening pass is admissible — the same discipline
RC2 already applies to drive feel. Tone-stack placement is a routing **index inside the
composite amp node**, not free graph routing.

### D5 — Stereo now
Widen the processing and graph domain (`IBlockModule`, `ModuleGraph` buffer plan, adapters) to
carry a **channel count now**, while the per-module adapters are still test-only. Individual
physical modules may remain internally mono where appropriate. The architecture must support
true stereo for cabinet/driver behaviour, spatial routing, effects and later MPE.
**Default behaviour remains mono-compatible.**

*Scheduling:* lands as the **opening task of P3**, before any new block-domain module is
written (P3 transducer, P5 compressors, P6 valve, P7 cabinet). After the P9 graph cutover it
would be a rewrite of the buffer plan, every adapter, the fan-in rule and the sample-identical
chain-equivalence property the cutover is verified against.

### D6 — Tube count semantics
- Preamp "count" = **active gain stages**; `N_max = 6` preamp **+ 1 phase-inverter slot**.
- Power "count" = **push-pull output-pair count** (cascading push-pull stages is physically
  meaningless; pair count scales max power, sag depth, crossover behaviour, transformer loading).
- Tube substitutions are **gain-true**: a 12AU7 really is ~20 dB quieter and really does drive
  the next stage less.

**Required consequence:** remove or revise the per-stage unity normalization at
`TriodeStage.cpp:232-240`, where `buildTransferTable()` divides the curve by its own
small-signal slope — this erases µ (12AX7 = 100, 12AT7 ≈ 60, 12AU7 ≈ 17), the exact term that
decides how hard the next stage is driven. Replace with a **volts-domain inter-stage
convention** (each stage emits real plate volts; only the amp's input and output convert to
and from sample domain). Budget the gain-staging re-calibration and regression refresh as part
of that change; makeup is defeatable and applied only at the **amp output**.

### D7 — Hybrid preset/parameter architecture
Structural and automatable choices (coil count, tube count, driver count, routing order) remain
**APVTS parameters**. Pure coefficient sets and voicing targets remain **preset data**.
Consequence: the preset system splits out of the old P5 bundle and lands **before** the first
voicing ships (P8), because a voicing that cannot be shipped as data must otherwise be baked
into the parameter surface.

### D8 — Compressor control surface
Gear-familiar primary controls with a **secondary advanced layer** exposing the underlying cell
and detector physics. This is the standing pattern for every "bring the controls from the
original gear" item — the same resolution `TriodeStage` already uses (musical controls plus
contractually reserved physical hooks).

### D9 — Analytic cabinet only
No IR loading, in any form, in the initial implementation. The analytic model gets **sufficient
scope to be a real model**, not a static EQ: meaningful driver count, cabinet structure,
resonant behaviour, inter-driver interaction (path-length comb via the existing Lagrange3
primitive), and **level-dependent power compression** — the one behaviour a fixed IR
structurally cannot produce, and what makes driver count physically meaningful.

### D10 — Accepted deferrals
WebView GUI; animated string visualisation (ship a **static monochrome schematic** over
low-rate snapshots); neck/fretboard wood (couples through nut/fret terminations — a different
physical path, a second module); IR loading; per-stage oversampling islands; runtime
`ModuleGraph` mutation as the mechanism for tube count (the class disclaims locks, requires
host suspension, and allocates in `freeze()`). A **native monochrome JUCE interface** using
custom LookAndFeel + `juce::Path` is preferred — no new dependency, no CI change.

## Consequent engineering positions (controller, not requiring author sign-off)

- **Variable structure** is expressed as **max-N always-instantiated slots with bit-exact
  bypass** plus an active-count parameter — generalising the shipped `renderChunk` precedent
  (the triode stays inside the oversampled island whether bypassed or not, so real latency
  never changes with a parameter the host was not told about). Compressor pre/post is a
  two-element order swap in a hard-wired chain, **not** a routing problem.
- **One oversampling island** spans the whole amp, factor fixed per `prepare()`. Inter-stage
  coupling highpasses are the **physical** DC fix (closing the pending ledger item) and
  band-limit before each successive nonlinearity. The `[aliasing]` gate re-binds to shipped
  default + one locked abuse config; everything else report-only. This is an **entry gate** of
  the valve phase, not a task inside it.
- **Body coupling is bidirectional**: a low-order (4–8 mode) positive-real modal admittance
  loads `BridgeJunction` in the sample domain (dead spots, wolf notes, uneven partial decay,
  feedback tendency), plus a high-order radiated bank downstream in the block domain. A
  one-way body would leave every note's envelope bit-identical and ship as an EQ curve wearing
  a physics name. The "chamber" ships as a **continuum on `couplingStrength`**, not a toggle,
  with a declared colour-only degradation path if the P2.5 passivity timebox fires.
- **Table metadata extends now**, before the generator's format freezes: a table record carries
  Koren parameters **plus circuit constants plus the solved operating point**, because a 12AU7
  dropped into a 12AX7's 100k/1.5k circuit is a badly biased 12AU7. `loadTransferTable` extends
  from validate-only to store-plus-atomic-swap with a short crossfade.

## Amendments to the in-flight P0–P2 plan

These ride inside tasks already scheduled to rewrite the same lines; **zero output-sample change**:

1. **P2.1** — `(string, tap)` storage widening, `kMaxTapsPerString = 4`, one active; per-string
   position smoothers; rename `material*` → `stringMaterial*` ("String Loss", "String
   Stiffness") reserving Material/Wood for the body, in the task that already bumps the state
   version; `PerString` gains the envelope-scaling scalar reserved for D2.
2. **P2.3** — the click-free crossfade is written against the widened tap arity.
3. **P2.4** — decide and record a **nonzero** shipping default for
   `BridgeAdmittanceParams::couplingStrength` at the gate (at 0.0 the strings are fully
   decoupled and `bridgeOutput()` is identically zero, which would silently disable every
   later body/chamber feature).
4. **P1.3/P1.5 seam** — exciter position read sample-accurately at the note-on event rather
   than block-quantized, so fast tremolo picking gives each repluck its own position. This is
   the whole of "picking position adjustable through slide": the control already ships; there
   is no physical referent for sliding a pick after its 3 ms burst has ended.

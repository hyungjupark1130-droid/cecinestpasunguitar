# ADR 0007 — Bridge tuning compensation is analytic; the default instrument stays in tune

- **Status:** Accepted
- **Date:** 2026-08-01
- **Decided by:** project author, on the P2.4 review's finding I1
- **Amends:** the P2.7 task definition in the approved P0–P2 plan (§4.5 and the P2.7 task section)
- **Amends:** ADR 0006's `couplingStrength` shipping default, which becomes **provisional**

## Context

P2.4 attached a loaded `BridgeJunction` to the shipping single-string topology. The review
established two facts that together invalidate P2.7's planned mechanism.

**Fact 1 — the shipping topology is measurably out of tune.** At the P2.4 default admittance the
worst residual over the gated band (MIDI 33–96) is **4.90 cents at MIDI 45**, at all three sample
rates, against a project gate of ±2 cents. This is not a defect in the junction; it is the physical
consequence of terminating a string on a compliant load instead of a rigid one. The load's phase
response near its resonance shifts the reflection, and the loop length changes with it.

**Fact 2 — the residual is not a per-note constant.** It is a function of three parameters the
user can turn, and it **reverses sign** across the resonance sweep (all at MIDI 45, 48 kHz, other
parameters at their defaults):

| `couplingStrength` | cents | `resonanceHz` | cents | `damping` | cents |
|---|---|---|---|---|---|
| 0.00 | +0.000 | 80 | +4.461 | 0.01 | −0.188 |
| 0.35 | −4.855 | 110 | +0.001 | 0.50 | −4.855 |
| 1.00 | −14.056 | 180 | −4.855 | 10.00 | −0.494 |
| | | 2000 | −0.527 | | |

A note scan at `couplingStrength` 1.0 spans **−14.056 cents at MIDI 45 to +13.407 at MIDI 64**.

All three are live APVTS parameters: `bridgeCoupling` ∈ [0, 1], `bridgeResonanceHz` ∈
[`kBridgeMinResonanceHz`, 2000] Hz, `bridgeDamping` ∈ [`kBridgeMinDamping`, 4]
(`plugin/src/Parameters.cpp:92-102`).

P2.7 as planned builds a calibration table **indexed by MIDI note**, loaded through
`loadCalibrationTable`. A per-note table is a one-dimensional object. It structurally cannot
represent a correction that also depends on three continuous parameters, and no amount of
regeneration fixes that — the plan's mechanism does not cover the problem it was scheduled to fix.

## Decision

### D1 — P2.7 is redesigned around **analytic bridge phase-delay compensation**

The frozen per-note calibration table is **not** the mechanism. The junction's contribution to the
loop delay is analytically computable from its own admittance, so it is computed rather than
tabulated.

The string sees the junction as a reflection whose phase varies with frequency. Evaluating that
reflection's **phase delay at the string's fundamental** and folding it into the delay-line length
is the same kind of correction the P1 analytic compensation already applies for the loss and
dispersion filters — the bridge simply adds another term with a known closed form. It is O(1) per
string per parameter change, needs no table, and is correct at every point in the parameter space
rather than at one frozen admittance.

*Rationale:* this is the phase-delay ruling from P1.4 applied one element further down the loop.
That ruling — phase delay, never group delay, governs tuning compensation — is what makes this
tractable, and it must be honoured here too.

### D2 — the ±2-cent gate binds on the shipping topology across the normal parameter range

The **default instrument is in tune.** The ±2-cent acceptance across MIDI 33–96 at 44.1/48/96 kHz
binds against the shipping `StringNetwork` with the bridge attached, over a declared **normal
range** of the three bridge parameters — not merely at their default values, and not on the
isolated-string topology that P2.4's review found the gate had drifted onto.

### D3 — extreme bridge settings may keep bounded physical detuning, later and opt-in

Outside the normal range, retaining real detuning is legitimate: a radically compliant bridge
*should* pull pitch, and erasing that erases physics the instrument exists to expose. But this is
an **optional advanced behaviour, deferred**, and it must be **bounded** and declared. It is not a
licence for the default instrument to drift, and it is not in P2.7's scope unless P2.7 finds the
analytic correction cannot hold ±2 cents across the normal range.

### D4 — `couplingStrength = 0.35` is **provisional, not shipped**

ADR 0006's default is **not final** and must not be treated as settled by any later task,
including the P2.9 exit gate. It is confirmed or replaced only after the **P2.8 listening pass**,
which must compare lower values and judge mode-locking in near-unison voicings by ear.

> **SUPERSEDED IN PART, 2026-08-05 — see D7.2.** The default is now **0.20**, and it was settled by
> **author delegation, not by ear**. The listening pass this section reserves the decision for has
> still never been performed: `docs/listening/P2-20260803.md` remains marked NOT PERFORMED with
> every verdict field blank. **D4's condition was WAIVED, not met.** What remains in force is the
> prohibition above — no later task may treat the value as settled *by measurement*, and a green
> board is still not a sign-off.

Evidence motivating this: at 0.35, two strings tuned 25 cents apart **mode-lock** — both taps peak
at 111.297 Hz for nominals 110.00/111.60, a **+20.286 cent pull** on the string nobody detuned, with
the separation collapsing from 25 cents to 0.0029. Beat depth also falls sharply with coupling
(10.08 / 3.59 / 2.28 / 1.62 dB at 0.1 / 0.35 / 0.5 / 1.0), so the setting trades sympathetic
richness against pitch integrity, and where that trade should sit is a musical judgement.

## Consequences

- **P2.7's acceptance criteria change.** The `[tuning]` gate becomes a sweep over (note × rate ×
  bridge parameter grid) rather than (note × rate). The grid must cover the normal range declared
  under **Q1** below.
- **The correction is recomputed on parameter change, and that lands in the middle of P2.3's
  machinery.** Changing tuning compensation while a string rings means changing its delay length
  while it rings — exactly the click-free case P2.3 built the dual-anchor crossfade for. The
  recompute must route through that path and needs its own click gate. This coupling is easy to
  miss and expensive to retrofit.
- **The correction is self-referential and needs a fixed point.** The bridge phase delay is
  evaluated *at f0*, but changing the loop length changes f0. P2.7 must iterate (or Newton-solve)
  to convergence at parameter-change time, and declare the convergence criterion. It is offline
  work, not audio-path work.
- **`cnpg_calibrate` does not necessarily disappear.** It remains useful for *verifying* the
  analytic correction across the grid, and as a fallback residual trim at the default admittance.
  What changes is that it is no longer the mechanism of record.
- **ADR 0006's claim that this sits "inside what Task P2.7's calibration table is scheduled to
  absorb" is withdrawn** — true only at a frozen admittance, which is not the shipping condition.
- **P2.9 must not lock the coupling default.** The exit gate records it as provisional pending
  P2.8, per D4.

### D5 — the Normal range is **derived, not declared**, and there are two ranges

*(author decision, same day, answering this ADR's original open question 1)*

The **Normal range** is the **largest contiguous region** of the three-parameter space in which
*all five* of the following hold simultaneously:

1. tuning stays within **±2 cents** across the supported note range;
2. the fixed-point solver (D6) **converges reliably**;
3. **live parameter changes remain click-free**;
4. near-unison strings **≈25 cents apart do not involuntarily mode-lock** under ordinary playing
   conditions;
5. the bridge still behaves as an **instrument component rather than an overt resonant effect**.

This is a definition P2.7 can *execute* — four of the five conditions are measurable and the fifth
is the listening judgement P2.8 supplies. P2.7 derives a **provisional** region from measured data
so it can proceed; **P2.8 confirms or revises it** in the same session that settles the
`couplingStrength` default. Criterion 4 is the one that ties the range to D4: mode-locking is a
coupling phenomenon, so the coupling default and the range ceiling are the same measurement.

**Read D7.0 before D7's table, D7.1 after it, and D7.2 last.** The region D7 declares is derived
from criterion (1) alone, and **criterion (4) is measured to FAIL at its coupling ceiling** — so
D7's box is not the five-criteria region this section defines. D7.1 adds the same measurement on the
**shipping six-string topology**, where the boundary is **lower still** (between 0.20 and 0.30,
against 0.32–0.35 on an isolated pair). **D7.2 (2026-08-05) records what changed when the *default*
moved to 0.20: the box still is not criterion-complete, but the shipped instrument no longer sits in
the part of it that fails.** The listening pass has still not been performed.

Settings **outside** the Normal range remain available as an **Extended (Effect) range**. They
carry **no tuning guarantee** — this is where D3's bounded physical detuning lives, and it is a
legitimate place for the instrument to be an effect, provided the boundary is declared rather than
discovered.

### D6 — the solver is specified, not merely required

*(author decision, same day, answering original open questions 2 and 3)*

The fixed point runs **outside the audio path**, on parameter change. P2.7 must **declare and
test**, not merely implement:

- the **convergence tolerance** (in cents, tighter than the ±2-cent gate by a stated margin);
- the **maximum iteration count**;
- the **fallback behaviour for non-convergent or pathological settings** — what the instrument does
  when the solve does not land, which must be defined behaviour rather than whatever the loop
  happens to leave behind.

**All compensation changes caused by live bridge parameters route through P2.3's dual-anchor
crossfade machinery**, and P2.7 carries a **dedicated gate** proving that `couplingStrength`,
`bridgeResonanceHz` and `bridgeDamping` can each be changed *while strings are sounding* without
clicks, discontinuities, or unstable pitch transitions. Note the third of those is new: click-free
is not sufficient here, because a converging solver can be smooth and still audibly hunt.

`cnpg_calibrate` is retained as the **verification harness** across the MIDI-note × bridge-parameter
grid. The **steep phase-slope region around bridge resonance is measured early** — it is the risk
item, and discovering it at the gate is the failure mode to avoid. A **residual trim** is used
*only if* the analytic solution leaves a small systematic error there; it is a fallback, not part
of the design.

### D7 — the provisional Normal range, DERIVED (Task P2.7, 2026-08-01)

*Added by Task P2.7, which executed D5's definition against measurement. Everything here is
**provisional** and is confirmed or revised by the P2.8 listening pass, per D5.*

| parameter | Normal range | shipped slider |
|---|---|---|
| `couplingStrength` | **0.00 – 0.35** | 0 – 1 |
| `bridgeResonanceHz` | **20 – 330 Hz** | 20 – 2000 Hz |
| `bridgeDamping` | **0.15 – 1.00** | 0.01 – 4.0 |

**Everything outside that box is the Extended (Effect) range and carries NO TUNING GUARANTEE.** It
remains fully available; what it loses is the ±2-cent promise, and that is exactly D3's bounded
physical detuning — a radically compliant or radically sharp bridge *should* pull pitch.

#### D7.0 — this box is derived from criterion (1) ALONE, and criterion (4) FAILS at its coupling ceiling

*Stated first because it qualifies everything below it. Added by the P2.7 review, on measurement.*

D5 defines the Normal range as the largest contiguous region in which **all five** criteria hold
**simultaneously**. **This box does not meet that definition.** It is the largest box inside
criterion (1) — the ±2-cent tuning bound — with criterion (2) checked and found never to fire and
criterion (3) measured by the live-parameter click gate. **Criterion (4) — "near-unison strings ≈25
cents apart do not involuntarily mode-lock" — is NOT satisfied at the coupling ceiling of 0.35.** It
was not merely left unmeasured; it was measured, and it fails.

**Read the topology label on every table in this section.** There are now two independent
measurements of criterion (4) — P2.7's, on an **isolated pair** of strings driven directly, and
P2.8's, on the **shipping six-string instrument** through `NoteAllocator` and the monitoring chain.
They do not agree about where the boundary is, and **they are deliberately not reconciled into one
number**: they are two configurations, both real, and the disagreement is itself the finding. P2.7's
is below; P2.8's is in D7.1.

**The measurement (TOPOLOGY: two strings, isolated pair, direct `NoteEvent` injection).** Two strings
at MIDI 45, string 1 offset +25 cents via `tuningOffsetCents`, the shipping bridge otherwise, both
plucked, 48 kHz, 7 s, sustain material, peak of each string's own tap inside ±80 cents of its own
nominal:

| `couplingStrength` | string 0 | string 1 | separation | pull on string 0 |
|---|---|---|---|---|
| 0.20 | 109.9956 Hz | 111.5984 Hz | **25.045 cents** | −0.07 |
| 0.25 | 109.9922 Hz | 111.5985 Hz | **25.099 cents** | −0.12 |
| 0.30 | 111.5991 Hz | 111.5993 Hz | **0.003 cents** | **+24.99** |
| 0.35 | 111.6013 Hz | 111.6015 Hz | **0.003 cents** | **+25.02** |

The whole 25-cent separation collapses and lands on the string nobody detuned. Three further facts,
because each of them closes an escape route:

- **It is not an artifact of the sustain material.** At the **default** string material the boundary
  sits between 0.32 and 0.35 — separation 25.14 cents at coupling 0.30 and 25.18 at 0.32, then
  **0.33 cents at 0.35**, a +24.67-cent pull. The ceiling is inside the locked region at both
  materials.
- **It is not an artifact of plucking both strings.** Plucking only string 0 — the purely
  sympathetic case, which is the ordinary one — locks at the same ceiling: 25.19 cents separation at
  coupling 0.30, **−0.01 cents at 0.35** with a +25.04-cent pull.
- **It is worse, not better, where the load resonates.** At MIDI 53 (on the 180 Hz bridge resonance)
  coupling 0.35 compresses the pair from 25 cents to **15.10 cents** with a **+5.05-cent** pull on
  string 0 — audible detuning without a full lock, which is the harder case to dismiss.

**Consistency with ADR 0006, which is what says this is the same phenomenon and not a new one.**
ADR 0006 recorded the locked frequency as 111.297 Hz. The same configuration on P2.7's tree locks at
**111.6013 Hz — +4.73 cents**, which is P2.7's own compensation at MIDI 45 (+4.83 cents, the
`string_ir` golden drift). **The lock is unchanged in character; P2.7 moved it onto the correct
pitch.** `tests/dsp/StringNetworkScaleTests.cpp` already gates it — separation < 12.5 cents and a
pull larger than the whole decoupled error budget — so this is a standing measurement in the suite,
not a one-off.

**Why the ceiling is NOT lowered here.** Criterion (4) is a musical judgement, and D5 already ties it
to D4: mode-locking is a coupling phenomenon, so **the coupling default and the range ceiling are the
same measurement**, and D4 reserves that measurement for the P2.8 listening pass. Lowering the
ceiling now would pre-empt a decision that session exists to make. What P2.7 owes instead is the
statement of fact, which is this section.

**The consequence, stated so P2.8 does not have to discover it: the provisional `couplingStrength`
default sits OUTSIDE a criterion-complete Normal range.** It is inside the criterion-(1) box declared
above, and outside the region where all five of D5's criteria hold. P2.8 settles both, together.

> **THAT LAST PARAGRAPH IS NO LONGER TRUE OF THE SHIPPED DEFAULT, AND IS LEFT STANDING BECAUSE IT IS
> STILL TRUE OF THE BOX.** As of 2026-08-05 the default is 0.20 and is measured to satisfy criterion
> (4) on both topologies; the ceiling is unchanged at 0.35 and criterion (4) still fails there. The
> gap between the default and the ceiling is now the whole of the failing region, and it is reachable
> only by a user dragging the Bridge Coupling slider. See D7.2.

*The rest of this section is D7's own derivation: how the box in the table above was arrived at, face
by face. D7.1 below adds the second topology, and is placed after the derivation rather than before
it so that the derivation stays contiguous with the box it derives.*

**Measured worst |error| inside the box: 0.770 cents**, over MIDI 21–96 at 44.1/48/96 kHz at each of
six grid points (the four coupled corners of the box, the decoupled control, and the shipping
default) — 2.6× inside the criterion. At the shipping default it is 0.060 cents.

**What binds each face of the box, and it is not the same thing on each:**

- The **coupling ceiling** and the **resonance ceiling** trade against each other; the boundary is a
  curved surface and the box is the largest one inside it. Measured worst over the three notes
  nearest each resonance (48 kHz, worst over damping 0.15–1.0):

  | | 20 Hz | 60 | 110 | 180 | 250 | 330 | 400 | 500 |
  |---|---|---|---|---|---|---|---|---|
  | coupling 0.20 | 0.06 | 0.06 | 0.06 | 0.06 | 0.06 | 0.06 | 0.06 | 0.13 |
  | coupling 0.35 | 0.18 | 0.20 | 0.20 | 0.21 | 0.33 | 0.63 | 0.63 | 7.11 |
  | coupling 0.50 | 0.38 | 0.41 | 0.58 | 1.31 | 12.06 | 9.96 | 15.79 | 79.9 |
  | coupling 0.60 | 0.56 | 0.80 | 3.57 | 4.86 | 17.26 | 11.24 | 79.12 | 79.3 |

  Readings at 79–80 are the estimator's ±80-cent search boundary: the fundamental is no longer
  anywhere near where it was solved for.
- The **damping ceiling of 1.00** is a separate mechanism and was found at the grid gate rather than
  in the map above. At damping ≥ 2 the load is dashpot-dominated over a wide band and the worst note
  moves to the TOP of the range: measured 2.92 / 4.93 / 7.38 cents at damping 2 / 3 / 4 (coupling
  0.35, resonance 180 Hz, 44.1 kHz, worst over MIDI 21–96, worst note 85–95). At damping 1.0 the same
  sweep reads 0.03.
- The **damping floor of 0.15** is where the margin becomes comfortable rather than where the gate
  breaks (0.05 still reads 1.39 cents at resonance 330, i.e. inside the criterion with 1.4× headroom).
- The **coupling ceiling of 0.35 is a measured criterion-(1) boundary that coincides with the
  provisional default, not the default wearing a different hat.** It is written as a literal in the
  gate, never as `BridgeAdmittanceParams{}.couplingStrength`, and the gate additionally asserts that
  the shipping default lies inside the range — so if P2.8 raises the default past the boundary the
  gate fails and the range must be re-derived, which is what D5 already schedules that session to do.
  D4's expectation is that P2.8 compares *lower* values, all of which are inside. **Criterion (4) is
  a different boundary and it is BELOW this one — see D7.0.**
- **The coupling face's evidence above is coarser than the boundary it reports, and the headroom is
  therefore smaller than "0.35 vs 0.50" reads.** The map samples 0.35 and 0.50, but the criterion-(1)
  failure at the worst corner (resonance 330 Hz, ζ 0.15, 44.1 kHz, worst over MIDI 21–96) is at
  coupling **≈0.405**, not 0.50: measured **0.7697 / 1.6281 / 1.8575 / 2.5556 / 3.4742** cents at
  coupling 0.35 / 0.38 / 0.40 / **0.41** / 0.45. The ceiling has roughly **16 %** of criterion-(1)
  headroom on this face, not the ~43 % the coarse table suggests. That does not move the ceiling —
  0.35 is inside — but a reader sizing the margin from the table alone would over-estimate it.

**Criterion (2) — solver convergence — never fires inside the box.** The contraction ratio is
μ/(2πζ) with μ = `couplingStrength × kBridgeMaxMobilityRatio`; at coupling 0.35 it stays under 0.28
at every admissible damping and the solver converges in one iteration at every one of the 1368
gated points. The convergence boundary was measured at damping **0.0197** at coupling 1.0 — i.e. it
is reachable, but only in the Extended range, with two shipped sliders at their stops.

#### D7.1 — the SHIPPING SIX-STRING topology locks EARLIER than D7.0's isolated pair (Task P2.8)

*Added by Task P2.8, which built the listening material D4 reserves and measured the phenomenon on
the configuration that actually ships. It records a fact and proposes nothing: the ceiling and the
default remain the author's, per D4 and D5.*

D7.0's tables are measured on **two strings, driven directly**. The instrument ships **six strings on
one bridge**, allocated by `NoteAllocator`, with the four strings nobody is playing resting at their
open tuning (P2.7's rest-pitch fix) and therefore *in the same loop*. That is a different system, and
it does not have the same boundary.

**The measurement (TOPOLOGY: six strings, `GuitarFingering`, the shipping default chain).** Corpus
phrase `02_open_chords.mid` section C, rendered by `cnpg_render --variants` as
`02_open_chords__unison<NNN>`. MIDI 45 lands on string 1 (fret 0) and MIDI 46 on string 0 (fret 6);
`stringTuningOffsetCents1` = +25 and `stringTuningOffsetCents0` = −50 put the two strings at
**111.60 Hz and 113.22 Hz — 25.00 cents apart**, which is D7.0's pair on D7.0's strings. Both struck,
30 ms apart, as a player would. 48 kHz, FFT over 31–36 s, peaks inside 109–116 Hz, levels relative to
the stronger peak:

| `couplingStrength` | what survives in the band | separation |
|---|---|---|
| 0.00 | 111.603 Hz @ −0.3 dB, 113.205 Hz @ 0.0 dB | **24.677 cents** |
| 0.10 | 111.626 @ −1.8, 113.205 @ 0.0 | **24.322 cents** |
| 0.20 | 111.671 @ −6.8, 113.205 @ 0.0 | **23.612 cents** |
| 0.30 | 113.182 only | **ONE PEAK — locked** |
| 0.32 | 113.182 only | **ONE PEAK — locked** |
| 0.35 | 113.182 only | **ONE PEAK — locked** |

**On this topology the criterion-(4) boundary sits between 0.20 and 0.30**, against **0.32–0.35** for
D7.0's isolated pair at the same default string material. The shipping instrument locks *earlier*
than the ADR previously recorded.

Two further facts, because each changes what the listening pass is listening for:

- **The lock is progressive, not a threshold.** The detuned partner is not present-then-absent; it is
  absorbed — **−0.3, −1.8, −6.8 dB** — and then gone. So at coupling 0.20 the question is not "did it
  lock" (it did not) but whether a partner 6.8 dB down is still the chord that was played. D7.0's
  isolated-pair tables cannot show this, because they report only the peak frequencies.
- **Nothing here contradicts D7.0.** Six strings on one bridge is a more strongly coupled system than
  two: the four unplayed strings resting at their open pitches are additional loads sharing the same
  junction, and the bridge's effective admittance seen by any one string is a function of how many
  strings are live (which is checklist item 16's whole subject). A lower boundary on the busier
  topology is the expected direction; what was not known before this measurement is **how much**
  lower, and that it crosses below 0.30.

**What this does NOT do.** It does not lower the declared ceiling, propose a default, or re-derive
D7's box. D5 makes criterion (4) a musical judgement and D4 reserves it for the P2.8 listening pass,
which at the time of writing **has not been performed** — `docs/listening/P2-20260803.md` is prepared
with every verdict column empty. If the author's ear agrees with this table, the coupling ceiling
moves below 0.30 and D7's box is re-derived rather than edited; if it does not, this table stands as
the measurement the judgement was made against. Either way the ADR now carries the topology that
ships alongside the one that was convenient to measure.

> **WHAT HAPPENED NEXT, 2026-08-05.** The author settled the **default** at **0.20** — the largest
> value in the table above at which the pair survives on this topology — **by delegation, without
> holding the listening pass**. The **ceiling** did not move and this table is unchanged. D7.2
> carries the derivation, and the sentence above about what would follow "if the author's ear agrees"
> is still awaiting an ear.

#### D7.2 — the DEFAULT moves to 0.20 by author delegation; the CEILING does not move (2026-08-05)

*Added by the post-exit default-diffs task. It records a decision and a re-derivation. It proposes
nothing and it settles nothing by measurement.*

**HOW THIS WAS SETTLED, first, because it is the part most likely to be misread later.** The author
delegated the value on **2026-08-05** and the implementer applied it. **It was NOT settled by ear.**
D4 reserves this value for a recorded listening sign-off and reserved it across three separate
rulings; that sign-off has still never been performed, `docs/listening/P2-20260803.md` is still
marked NOT PERFORMED with every verdict column blank, and nothing in this section fills it. **D4's
condition was waived, not met.** Anyone citing 0.20 must cite it as a delegated decision resting on
the measurements below, never as a listening result.

**WHY 0.20 AND NOT ANOTHER WAIVER — the derivation.** D5's criterion (4) is the only one of the five
that binds on coupling below the criterion-(1) ceiling, and it is measured on two topologies:

| topology | largest measured coupling at which the 25-cent pair SURVIVES | first measured coupling at which it LOCKS |
|---|---|---|
| isolated pair, sustain material (D7.0) | **0.25** — separation 25.099 cents, pull −0.12 | 0.30 — separation 0.003, pull +24.99 |
| isolated pair, default material (D7.0) | 0.32 — separation 25.18 | 0.35 — separation 0.33, pull +24.67 |
| **shipping six strings on one bridge (D7.1)** | **0.20** — separation 23.612 cents, partner at −6.8 dB | 0.30 — ONE PEAK |

**The binding topology is the one that ships**, so the criterion-(4)-satisfying region has a coupling
ceiling of **0.20**, and the default is placed exactly on it. That placement is deliberate and its
weakness is stated rather than smoothed: the true boundary lies somewhere in **(0.20, 0.30)** and no
point inside that interval has ever been measured on six strings, so 0.20 is the **last reading that
passes**, not an edge. **The margin is zero in the only direction that matters.**

**WHAT THE CEILING IS NOW, and what binds it.** `kBridgeNormalCouplingMax` is **unchanged at 0.35**,
and three things bind that decision rather than one:

1. **Criterion (1) still holds there and is still measured there.** The ±2-cent tuning guarantee is
   what D2 declares and what the grid gate enforces; its coupling face is at ≈0.405 (D7's own
   measurement) and 0.35 sits inside it with about 16 % of headroom. Lowering the declared box to
   0.20 would *withdraw* a tuning guarantee that is measured to hold — a different and unrequested
   change, and a misleading one, because "Extended range" reads as *may detune* and 0.20–0.35 does
   not detune.
2. **The delegation named the default and only the default.** D5 ties the ceiling and the default to
   the same *measurement*; it does not make them the same *decision*, and the standing project ruling
   on this ADR is explicit that the ceiling is not to be lowered ahead of the ear.
3. **Criterion (5) — "the bridge still behaves as an instrument component rather than an overt
   resonant effect" — has never been judged at all.** A ceiling re-derived without it would be the
   same category error D7.0 exists to name.

**IS THE NORMAL RANGE CRITERION-COMPLETE NOW? At the default, yes on the four measurable criteria;
as a box, no.** Precisely:

| | at the shipping default 0.20 | over the declared box (coupling ≤ 0.35) |
|---|---|---|
| (1) ±2 cents | **holds** — 0.060 cents measured | **holds** — worst 0.770 cents |
| (2) solver converges | **holds** — one iteration | **holds** — 1368/1368 gated points |
| (3) live changes click-free | **holds** | **holds** — `BridgeTuningClickTests.cpp` |
| (4) no involuntary mode-lock at ~25 cents | **HOLDS — this is what changed** | **FAILS above ≈0.20–0.25** |
| (5) instrument component, not an effect | **NEVER JUDGED** | **NEVER JUDGED** |

So the correct statement, and the one that replaces D7.0's closing paragraph for the shipped
instrument: **the default now sits inside the region where every measurable criterion holds, for the
first time since the bridge landed.** The box does not, and the difference between the two is now
exactly the interval a user crosses by dragging Bridge Coupling above its default.

**What it costs.** Coupling is the mechanism of sympathetic resonance and the feed ADR 0004's later
body/chamber rides on, so less of it is less of one string in the others. The two measurements that
exist both run the other way: on a six-C3 unison stack the 1–3 s tail is **8.3 dB louder** at 0.20
than at 0.35 (−80.4 against −88.7 dBFS), and ADR 0006's beat depth is **10.08 dB at 0.10 against
3.59 at 0.35**. Neither is an ear, and D7.1's own question stands unanswered: at 0.20 the detuned
partner is present but **6.8 dB down**, and whether that is still the chord that was played is a
listening judgement.

**The standing gate moved with it, and was re-pointed rather than deleted.**
`tests/dsp/StringNetworkScaleTests.cpp` asserted the lock **at the shipping default**; at 0.20 there
is no lock, so that assertion would have gone vacuous. It now pins the **boundary** instead —
separation preserved at a probe of 0.25, collapse at a probe of 0.30, both measured in the case, each
arm asserted to reject the other's numbers, plus an arithmetic clause that the shipping default lies
on the separated side. That is a gate on a physical fact rather than on a value, and it survives the
default moving again.

### D8 — the gated note band moves in both directions (Task P2.7)

§4.5 assigned "the full 88-note (21–108) × 3-rate ±2-cent assertion" to this task. P2.7 **widens** it
downward and **narrows** it upward, both on measurement:

- **MIDI 21–32 is now GATED** (it was report-only through P1). Worst |error| at the shipping
  admittance across all three rates: **0.001 cents**.
- **MIDI 97–108 stays report-only**, under a widened ±4-cent sanity bound, because what limits it is
  not the bridge. At 48 kHz the DECOUPLED control on the identical render is *worse* than the coupled
  one — MIDI 108 reads −0.262 cents coupled against −1.552 decoupled — and at 96 kHz every note in
  the band reads 0.000 in both arms. The cause is the loop being ~13 samples long at MIDI 105 /
  48 kHz, where the integer rail read and the interpolator's admissible range stop tiling the reals
  finely enough: a P1 fractional-delay property (ADR 0002), not one a bridge compensation can reach.
  The gate asserts that attribution rather than merely stating it.

### D9 — the solve is a closed form; the fixed point is about UNIQUENESS, not about the value

*A correction to this ADR's own D1/D6 framing, recorded because the difference is load-bearing.*

D6 says "P2.7 must iterate (or Newton-solve) to convergence". The derivation says otherwise for the
value: a waveguide loop resonates where its round-trip phase delay equals `fs/f`, the loop-length
solve chooses the rail span so that this holds **at the target**, and adding `tau_port(f_target)` to
that sum makes `f_target` exactly a root. **One closed-form evaluation, no iteration.** Measured: the
solver reports one iteration at every note at the shipping admittance, and at that admittance the
**rendered pitch error** is 0.060 cents against 4.90 uncompensated.

**Two different residuals live in this task and they must not be conflated.** The **probe residual**
is what the solver itself reports (`BridgeTuningSolution::residualCents`) — the worst |cents| the
fixed-point iterate still carried at its last step, against a 0.25-cent tolerance; its worst over the
240 084-point dense sweep is **0.063 cents**. The **rendered pitch error** is what the §4.5 estimator
measures off 7 seconds of audio; at the shipping admittance it is **0.060 cents** and over the whole
Normal range **0.770**. The two are close at the shipping point by coincidence of magnitude, not by
construction: one is a property of an arithmetic iterate, the other of the instrument.

The self-reference D6 anticipated is real but it is about **uniqueness**. Given a committed
compensation, the frequency the string sings at is the fixed point of
`Phi(f) = fs / (fs/f_target − tau_c + tau_port(f))`, whose derivative is `(f²/fs)·|dtau_port/df|`.
Where that approaches 1 the root stops being isolated — the string's fundamental and the bridge mode
enter an avoided crossing, the loop acquires three phase-zero crossings instead of one, and the pitch
that comes out is not the pitch that was solved for. **That** is what the iteration tests, and it is
the same thing as D6's "steep phase-slope region". The solver therefore produces the value in closed
form and spends its iterations establishing that the value is one the instrument can hold.

#### D9.1 — the exact root is a PHASE ZERO; the sounding pitch is a POLE, and the gap is the whole residual

*Recorded in the ADR rather than only in the task report, because it is what the Normal range's
resonance and damping ceilings are actually cut from.*

The closed form above places the loop's **round-trip phase zero** exactly on the target — exactly, at
every admittance, which is why the ±2-cent gate's residual is hundredths of a cent and not tenths.
But **what a listener and an FFT both measure is the loop's POLE**, and a pole sitting on a
frequency-dependent loop gain is displaced from the unit-circle phase zero by roughly

    Δω  ≈  a₀ · (d ln|L|/dω) / (dφ/dω)²

where `L` is the round-trip loop gain, `a₀` its magnitude deficit from 1, and `dφ/dω` is the loop
delay in samples, `D`. **The displacement therefore scales as 1/D², i.e. as f₀².** Two consequences,
both of which the Normal range is shaped by:

1. **This displacement IS the residual.** The 0.060 cents at the shipping admittance and the 0.770 at
   the box's worst corner are not solver error and not estimator noise — they are the pole–zero gap.
   No tighter solve reduces them, which is why D6's licensed "residual trim" was measured to be
   unnecessary rather than merely skipped.
2. **It is why the ceilings are on the RESONANCE and on the DAMPING rather than on the note.** A
   resonance sitting on a high note pulls far harder than the same resonance on a low one, because
   `D` is smaller there. It is also what the high-damping corner failure turned out to be: at ζ ≥ 2
   the load is dashpot-dominated over a wide band, nothing is localised at the resonance any more,
   and the worst note migrates to the **top** of the range (2.92 / 4.93 / 7.38 cents at ζ = 2 / 3 / 4,
   against 0.03 at ζ = 1). The two mechanisms are told apart by *which note fails*, and the gate
   asserts that migration rather than narrating it.

## Open questions this ADR does not settle

None outstanding as a *question*. The three original open questions were answered by the author on the
same day and are recorded above as D5 and D6; D7–D9 record what Task P2.7 measured against them.

What remains is a scheduled confirmation with a **known defect in it**: the **provisional** Normal
range (D7) and the **provisional** `couplingStrength` default are both settled by ear at P2.8, and
**D7's box is criterion-(1)-complete but not D5-complete** — criterion (4) fails at its coupling
ceiling, measured, per D7.0. P2.8 therefore does not merely confirm the range; it has an established
failure to resolve, and D5 makes resolving it the same act as choosing the default.

**As of Task P2.8 (2026-08-03) that confirmation is still outstanding, and the defect is now known to
be larger than D7.0 recorded.** P2.8 built the comparison material D4 reserves — the coupling ladder,
the near-unison ladder, and settings at and outside all three faces of the box — and measured
criterion (4) on the **shipping six-string topology**, where the boundary sits between **0.20 and
0.30** rather than D7.0's 0.32–0.35 (D7.1). It recorded **no verdict**:
`docs/listening/P2-20260803.md` is the prepared session sheet and every verdict column in it,
including the `couplingStrength` value and the Normal range, is deliberately empty. Both remain
provisional, and P2.9 must not treat a green board as confirmation of either (D4).

**As of 2026-08-05 the DEFAULT is settled at 0.20 and the RANGE is not.** The default was settled by
**author delegation, with the listening pass still unheld** — the sheet above is still marked NOT
PERFORMED — so D4's condition was waived rather than discharged (D7.2). The declared box is unchanged
at coupling ≤ 0.35 and is **still not D5-complete**: criterion (4) still fails above roughly 0.20–0.25
and criterion (5) has still never been judged. What changed is that **the shipped instrument no longer
sits in the failing part of its own declared range** — every measurable criterion holds at the
default. The remaining work is unchanged in kind: an ear, and a re-derivation of the box against it.

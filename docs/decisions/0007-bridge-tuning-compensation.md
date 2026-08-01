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

## Open questions this ADR does not settle

1. **What is the "normal parameter range"?** The ±2-cent gate cannot bind until it is defined.
   It needs a musical answer, not an arithmetic one — the range over which the bridge is a bridge
   rather than an effect. Owed before P2.7's gate can be written.
2. **What bound applies to extreme settings under D3** — a hard cents ceiling, a soft warning, or
   simply "declared and measured"?
3. **Does the analytic correction hold ±2 cents across the whole normal range**, or does a residual
   trim remain necessary near the load resonance, where the phase slope is steepest and the
   sensitivity to every parameter is highest? This is the risk item for P2.7 and it should be
   measured early rather than discovered at the gate.

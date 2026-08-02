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
- The **coupling ceiling of 0.35 is a measured boundary that coincides with the provisional default,
  not the default wearing a different hat.** It is written as a literal in the gate, never as
  `BridgeAdmittanceParams{}.couplingStrength`, and the gate additionally asserts that the shipping
  default lies inside the range — so if P2.8 raises the default past the boundary the gate fails and
  the range must be re-derived, which is what D5 already schedules that session to do. D4's
  expectation is that P2.8 compares *lower* values, all of which are inside.

**Criterion (2) — solver convergence — never fires inside the box.** The contraction ratio is
μ/(2πζ) with μ = `couplingStrength × kBridgeMaxMobilityRatio`; at coupling 0.35 it stays under 0.28
at every admissible damping and the solver converges in one iteration at every one of the 1368
gated points. The convergence boundary was measured at damping **0.0197** at coupling 1.0 — i.e. it
is reachable, but only in the Extended range, with two shipped sliders at their stops.

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
solver reports one iteration at every note at the shipping admittance, and the residual it leaves is
0.060 cents against 4.90 uncompensated.

The self-reference D6 anticipated is real but it is about **uniqueness**. Given a committed
compensation, the frequency the string sings at is the fixed point of
`Phi(f) = fs / (fs/f_target − tau_c + tau_port(f))`, whose derivative is `(f²/fs)·|dtau_port/df|`.
Where that approaches 1 the root stops being isolated — the string's fundamental and the bridge mode
enter an avoided crossing, the loop acquires three phase-zero crossings instead of one, and the pitch
that comes out is not the pitch that was solved for. **That** is what the iteration tests, and it is
the same thing as D6's "steep phase-slope region". The solver therefore produces the value in closed
form and spends its iterations establishing that the value is one the instrument can hold.

## Open questions this ADR does not settle

None outstanding. The three original open questions were answered by the author on the same day and
are recorded above as D5 and D6; D7–D9 record what Task P2.7 measured against them. What remains is
not a question but a scheduled confirmation: the **provisional** Normal range (D7) and the
**provisional** `couplingStrength` default are both settled by ear at P2.8.

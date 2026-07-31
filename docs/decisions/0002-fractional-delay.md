# 0002 — Fractional-delay interpolator for `WaveguideString`

**Date:** 2026-07-31
**Task:** P1.4 (WaveguideString)
**Status:** Accepted

## Decision

**`FractionalDelayKind::Lagrange3` is the shipping default.** `Thiran1` remains fully implemented,
compiled, contract-tested, energy-tested and golden-covered, and is selectable at `prepare()`.

## Context

`docs/plan.md` section 2.4 leaves the fractional-delay interpolator open ("Fractional delay is
Lagrange or Thiran, selected in P1 and recorded in-repo") and Task P1.4 asks for a comparison on
three axes: tuning error across MIDI 21–108, decay-tail smoothness under continuous retune, and
cost per sample. Both candidates are implemented as the *rail read itself* (one per rail), so the
comparison is like-for-like: same topology, same termination chain, same analytic tuning solve,
same tests.

Two structural facts shape the comparison:

- **Lagrange-3 is an FIR** with `|H(w)| <= 1` only for design delay `D` in `[1, 2]` (already 1.0013
  at Nyquist for `D = 2.001`). Its phase delay is *exactly* 1 at `D = 1` and *exactly* 2 at `D = 2`
  at every frequency, so the reachable per-rail spans tile the reals with no gaps. It has no state
  of its own — its "states" are rail samples — so nothing is carried across a coefficient change.
- **Thiran-1 is an allpass** (`|H| = 1` everywhere, for every `D > 0`), so it adds no loss at all,
  but it *has* state, and its phase delay is compressed relative to `D` by an amount that grows
  with frequency. At the shortest supported loop the nominal `D in [0.5, 1.5]` covers only 0.929
  samples of phase delay; the range had to be widened to `[0.4, 1.8]` to close that gap (worth
  ~24 cents at MIDI 108 if left unhandled).

## Measurement

All numbers produced by the in-repo harnesses on the dev machine (Windows 11, MSVC, Release):

- Tuning: `cnpg_tests "TUNING: full comparison table (report generator)"` — the canonical section-4.5
  estimator (7 s render, 0.5 s discard, 2^18 samples / 2^19 at 96 kHz, Blackman-Harris, ×4 zero
  padding, parabolic peak interpolation, ±80-cent search), all 88 notes × 3 rates × both kinds.
- Retune smoothness and cost: `cnpg_tests "SPIKE: fractional-delay comparison (report generator)"`.

### 1. Tuning error (cents), MIDI 21–108 at 44.1 / 48 / 96 kHz

| variant | rate | band | worst abs error | mean abs error |
|---|---|---|---|---|
| lagrange3 | 44100 Hz | gated (33–96) | 0.0003 | 0.00006 |
| lagrange3 | 44100 Hz | report-only (21–32, 97–108) | 1.0358 | 0.09039 |
| lagrange3 | 48000 Hz | gated (33–96) | 0.0003 | 0.00007 |
| lagrange3 | 48000 Hz | report-only (21–32, 97–108) | 0.7210 | 0.05920 |
| lagrange3 | 96000 Hz | gated (33–96) | 0.0003 | 0.00007 |
| lagrange3 | 96000 Hz | report-only (21–32, 97–108) | 0.0005 | 0.00015 |
| thiran1 | 44100 Hz | gated (33–96) | 0.0003 | 0.00006 |
| thiran1 | 44100 Hz | report-only (21–32, 97–108) | 0.0005 | 0.00015 |
| thiran1 | 48000 Hz | gated (33–96) | 0.0003 | 0.00007 |
| thiran1 | 48000 Hz | report-only (21–32, 97–108) | 0.0005 | 0.00015 |
| thiran1 | 96000 Hz | gated (33–96) | 0.0003 | 0.00007 |
| thiran1 | 96000 Hz | report-only (21–32, 97–108) | 0.0005 | 0.00015 |

Both kinds are *identical* inside the gated band (0.0003 cents worst — 6600× inside the ±2-cent
gate) and both are inside ±2 cents across the entire 88-note range at all three rates. The only
difference is at MIDI 104–108 at 44.1/48 kHz, where Lagrange-3 reaches ~1 cent and Thiran-1 stays
at 0.0005 cents worst across its whole report-only band. That difference is not a tuning-solve
difference — the solve realizes the requested loop phase delay to better than 1e-6 cents for both (`CONTRACT: WaveguideString realizes
the requested loop period exactly`, 1584 grid points). It is the interpolator's magnitude response:
Lagrange-3's lowpass droop damps the top notes' fundamental enough to broaden the resonance, and a
broader peak biases slightly under the pluck's spectral slope. Thiran-1, being exactly allpass,
leaves a needle-sharp peak.

**Verdict on axis 1: a slight edge to Thiran-1, entirely outside the gated band, and both pass.**

### 2. Decay-tail smoothness under continuous retune

Metric definitions (both from the spike harness, 110 Hz note, ±2-semitone continuous glide over
6 s, loss bypassed, `double` instantiation):

- *bend energy growth* — worst per-block relative growth of `energyEstimate()` during the glide. A
  stateful interpolator whose coefficients jump when the integer part of the rail read steps
  injects or destroys stored energy; a stateless one cannot.
- *bend transient ratio* — peak |second difference| of the tap signal divided by its RMS, expressed
  as a **ratio against the identical measurement on a static note**. 1.0 means the bend added no
  impulsive content whatsoever.

| variant | bend energy growth | bend transient ratio |
|---|---|---|
| Lagrange3 | 3.94e-4 | **1.25** |
| Thiran1 | 1.53e-2 | **2477** |

This is the decisive axis and it is not close. Thiran-1's allpass state is stale every time the
integer part of the rail read steps — which during a ±2-semitone bend on a 110 Hz string happens
dozens of times — and each step lands as an impulsive discontinuity three orders of magnitude above
the signal's own sample-to-sample behaviour. Lagrange-3, holding no state of its own, glides at
1.25x the static baseline, i.e. essentially clean.

Thiran-1's figure carries a second, structural contribution that is worth naming because it is not
a tuning matter and would otherwise look like a free choice. Its allpass sits OUTSIDE the rail, so
the rail is consumed at an integer delay and nothing past it is ever read again; the position ->
delay map therefore has to run over the integer live window, and it steps with that integer during
a bend. Lagrange-3's interpolator IS the rail read, so its map runs over the continuous realized
span and never steps. Mapping Thiran-1 over the continuous span instead does not avoid the problem,
it hides it: taps and injections near the bridge then address already-consumed slots -- at MIDI 108
/ 44.1 kHz the entire dn-rail half of a p = 0.28 pluck lands past the read point and is silently
discarded. See the `positionSpan_` note in `WaveguideString.h`.

`docs/plan.md` locks "click-free global pitch
bend" (Task P1.5) and lists bend cleanliness as physical-plausibility item 5 with corpus phrase
`05_low_string_bends.mid` as an explicit abuse case; R2's early-warning signal is exactly this.

**Verdict on axis 2: decisive win for Lagrange-3.**

### 3. Cost per sample

| variant | static f0 (ns/sample) | continuously bending (ns/sample) |
|---|---|---|
| Lagrange3 | 17.1 | 161.0 |
| Thiran1 | 14.2 | 171.0 |

Thiran-1 is ~17 % cheaper at rest (one allpass multiply-add per rail versus four taps) and slightly
*more* expensive while bending. In budget terms, at 48 kHz one static string costs 0.082 % of a
core for Lagrange-3 versus 0.068 % for Thiran-1 — a difference of 0.014 percentage points per
string, i.e. **0.085 points across the 6-string default configuration**, against a 30 % gate.

**Verdict on axis 3: a small edge to Thiran-1, immaterial against the budget.**

## Decision rationale

Thiran-1 wins two axes by margins that do not matter (a fraction of a cent, entirely outside the
gated band; 0.085 percentage points of a 30 % CPU budget). Lagrange-3 wins the one axis that is a
locked product requirement, by a factor of about 2000. Cheap and marginally sharper is not worth an
audible click on every bend, and no amount of smoothing fixes it — a stale allpass state and an
integer-stepping position map are both structural.

Lagrange-3 also composes better with the rest of the locked design: it is a *read*, so both rail
spans stay continuous reals and every tap/injection position derived from them moves continuously
(see the topology note in `WaveguideString.h`), and its extra loss is dissipative, which the
tier-2 `[energy]` gate confirms (worst per-block growth exactly 0 for Lagrange-3 across both
dispersion settings and all three rates, versus 2–4e-15 float64 noise for Thiran-1).

## Consequences

- `FractionalDelayKind::Lagrange3` is what `StringNetwork` (P1.5) and the plugin will pass to
  `prepare()`. `Thiran1` stays a first-class, tested, golden-covered alternative — it is the
  natural fallback if a future stage needs a strictly allpass loop.
- Goldens are per-variant: `tests/data/golden/string_ir/{lagrange3,thiran1}/<rate>/`. Both sets are
  maintained; the losing variant is not allowed to bit-rot.
- The Thiran-1 `D` range is `[0.4, 1.8]`, not the textbook `[0.5, 1.5]`, for the reachability reason
  documented in `WaveguideString.cpp`. Narrowing it back would reopen a ~24-cent hole at MIDI 108.
- Thiran-1's position -> delay map runs over the integer live rail window, Lagrange-3's over the
  continuous realized span (`positionSpan_`). Anything that later moves the interpolator back into
  the termination chain, for either kind, has to revisit that map.
- P2.7 (`cnpg_calibrate`) measures whichever variant is configured; the residual it has to correct
  for Lagrange-3 at MIDI 104–108 is the ~1 cent above, well inside its own < 0.5-cent second-pass
  requirement.

## Related decision recorded here: phase delay, not group delay

`docs/plan.md` section 2.4 and the P1.4 brief both describe the analytic tuning compensation as
"group-delay". Implemented literally that misses the gate: a loop resonates where its round-trip
**phase** is a multiple of 2π, so the quantity that must equal `fs / f0` is the total loop **phase**
delay. With `dispersionAmount = 1` at MIDI 96 / 44.1 kHz each dispersion allpass has group delay
1.4599 samples but phase delay 1.4865 samples; across the four-allpass chain that is 0.106 samples
of a 21.07-sample loop — an 8.7-cent error, over four times the gate, before anything else
contributes, and worth roughly 60 cents at MIDI 108. Every delay quantity in `WaveguideString` is
therefore a closed-form phase delay evaluated at the current f0. This is a correction to the
wording of the plan, not a change to its design: the compensation is still analytic, still computed
from the filter formulas, and still the source `setAnalyticTuningCompensation` selects.

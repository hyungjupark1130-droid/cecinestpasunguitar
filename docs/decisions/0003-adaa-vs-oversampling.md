# 0003 — Antialiasing the triode: oversampling vs ADAA

**Date:** 2026-07-31
**Task:** P1.8 (`Oversampler`, `[aliasing]` gate, ADAA spike)
**Status:** Accepted

## Decision

**Oversampling is the shipping antialiasing path: `Oversampler` at the locked default factor 2×,
wrapping `TriodeStage::process` through `processWrapped`.** First-order ADAA is measured, recorded
below, and **not adopted**; it is to be re-evaluated alongside the P4 nonlinear stages, if at all.

The `[aliasing]` gate is **GREEN at 2×**: the worst folded component across both locked tones at
both locked drive settings is **−64.93 dBc**, 4.93 dB inside the −60 dBc limit.

## Context

`docs/plan.md` section 4.4 fixes the measurement; Task P1.8 asks for a timeboxed spike comparing
first-order antiderivative antialiasing against the `Oversampler` wrapper on aliasing, CPU and
step-response smearing, and for the shipped path to be whatever this document concludes.

The device under test is the shipped `TriodeStage` (P1.7) as-is — a static Koren ECC83 waveshaper
whose soft grid-current clamp has a slowly-decaying harmonic tail at high drive (P1.7's review
ledgered ~3 dB per odd harmonic through H15). That tail is what makes this a real measurement and
not a formality: at 2× the binding folded components below are *not* decimation-filter leakage at
all, they are harmonics above the **oversampled** Nyquist wrapping inside the 96 kHz domain, which
no filter in the decimator can touch.

## Measurement method

Exactly `docs/plan.md` section 4.4, implemented in `tests/dsp/AliasingGateTests.cpp`:

- 48 kHz host rate, fixed sines at 1244.5 Hz and 4186 Hz.
- Discard 8192 warm-up samples, capture 2^18, remove the mean (the triode is asymmetric and carries
  a real DC offset), Blackman-Harris window, ×4 zero-padded 2^20-point FFT.
- Enumerate predicted harmonics `k·f0` until `k·f0 > 4 × (factor × 24 kHz)`. Harmonics at or below
  24 kHz are *harmonic*; every other `k` is classified by its folded image
  `|k·f0 − round(k·f0 / 48000) · 48000|`. That one formula covers both fold mechanisms —
  reflection folding about a multiple of the base rate is the composition of folding about a
  multiple of the oversampled rate with the decimator's own fold — so **in-oversampled-domain
  aliasing is measured by the same rule and is not exempt**.
- Readout radius 1 analysis bin (0.183 Hz); coincidence radius 8 analysis bins (1.465 Hz, the
  Blackman-Harris main-lobe width — the smallest radius at which a harmonic's own main lobe cannot
  be misreported as a fold). No amplitude-based early stop on the enumeration: enumerating *more*
  folds than necessary can only make the gate stricter.
- `worst dBc` is the strongest classified fold relative to the strongest classified harmonic. The
  measurement noise floor (median bin magnitude, 100 Hz–20 kHz, same reference) is reported beside
  every row so no number can be mistaken for the harness running out of dynamic range.

Three drive conditions. Two are **gated**, exactly as locked: *nominal* (default
`TriodeStageParams::drive` = 0.5, −18 dBFS peak input — the per-string nominal of `docs/plan.md`
section 1.9) and *max* (drive 1.0, same input level). The third, *headroom*, is **report-only and
is this task's own addition**: drive 1.0 with a −2 dBFS input, i.e. the per-string nominal plus the
whole +16 dB multi-string summing budget of section 2.9. It is the hardest thing the shipped chain
can actually be asked to do, and leaving it out would have made the gate flattering. It is
explicitly *not* treated as a gate — substituting a self-invented stimulus for a locked one is not
a decision this document is allowed to make.

## 1. Aliasing: `Oversampler(TriodeStage)`, factors 2 / 4 / 8

Worst folded component, dBc relative to the strongest harmonic. Dev machine, Windows 11 / MSVC /
Release. `cnpg_tests "[aliasing]"`.

| factor | latency | tone | drive | worst dBc | at Hz | noise floor dBc | folds | status |
|---|---|---|---|---|---|---|---|---|
| 2 | 3 | 1244.5 | nominal | −145.32 | 1418.0 | −178.83 | 135 | **GATED** |
| 2 | 3 | 1244.5 | max | −88.82 | 21330.0 | −178.36 | 135 | **GATED** |
| 2 | 3 | 1244.5 | headroom | −61.14 | 21330.0 | −172.07 | 135 | report |
| 2 | 3 | 4186.0 | nominal | −149.24 | 3908.0 | −176.43 | 40 | **GATED** |
| 2 | 3 | 4186.0 | max | **−64.93** | 16466.0 | −175.55 | 40 | **GATED** |
| 2 | 3 | 4186.0 | headroom | −42.13 | 20652.0 | −172.62 | 40 | report |
| 4 | 4 | 1244.5 | nominal | −124.93 | 1229.5 | −178.19 | 289 | report |
| 4 | 4 | 1244.5 | max | −113.47 | 21503.5 | −177.89 | 289 | report |
| 4 | 4 | 1244.5 | headroom | −75.94 | 21503.5 | −173.22 | 289 | report |
| 4 | 4 | 4186.0 | nominal | −149.51 | 3908.0 | −178.52 | 86 | report |
| 4 | 4 | 4186.0 | max | −77.50 | 20374.0 | −177.80 | 86 | report |
| 4 | 4 | 4186.0 | headroom | −54.56 | 16188.0 | −173.25 | 86 | report |
| 8 | 4 | 1244.5 | nominal | −124.93 | 1259.5 | −177.93 | 598 | report |
| 8 | 4 | 1244.5 | max | −124.93 | 1259.5 | −177.75 | 598 | report |
| 8 | 4 | 1244.5 | headroom | −88.08 | 23095.0 | −173.98 | 598 | report |
| 8 | 4 | 4186.0 | nominal | −121.11 | 4194.0 | −179.96 | 178 | report |
| 8 | 4 | 4186.0 | max | −102.48 | 19818.0 | −179.16 | 178 | report |
| 8 | 4 | 4186.0 | headroom | −68.42 | 3074.0 | −174.35 | 178 | report |

**Gate verdict: PASS.** Worst gated row −64.93 dBc ≤ −60 dBc. Factors 4 and 8 are report-only per
the plan and are never asserted.

### Where the binding folds come from — and what would *not* have fixed them

The worst gated row is the 79 534 Hz 19th harmonic of 4186 Hz (`19 × 4186 = 79 534`). At 2× the
oversampled Nyquist is 48 kHz, so that harmonic aliases *inside the 96 kHz domain* to
`|79 534 − 96 000| = 16 466 Hz` — which is exactly where the measurement found it. The same is true
of the worst headroom row (H18 at 75 348 Hz → 20 652 Hz) and of the 1244.5 Hz rows (folds at
21 330 Hz and 23 110 Hz likewise trace to harmonics past 48 kHz).

This matters for the remedy hierarchy: **steepening the stage-0 halfband (remedy 2) would have
bought nothing.** The decimation filter is not what is leaking. The only levers on this term are a
higher factor (which moves the oversampled Nyquist), a nonlinearity with a faster-decaying harmonic
series (remedy 3), or an antialiasing method that does not rely on rate at all (remedy 1). Since
the gate is green, none of the four remedies was applied; the analysis is recorded here so that a
future red gate is not spent re-deriving it.

### The headroom condition (report-only) is the honest weak spot

At 2× with the full +16 dB summing budget applied, the worst fold is −42.13 dBc. 4× brings that to
−54.56 dBc and 8× to −68.42 dBc. This is *not* a gate failure — the locked gate is the nominal and
max drive rows and both pass — but it is the number a listening pass should be aimed at, and it is
the reason `Oversampler`'s factor stays user-visible and re-selectable. It is carried forward as an
open item for P1.10/P1.11 (bench + listening), where the real chain levels out of `PickupTap` are
measurable rather than assumed. See "Consequences" below.

## 2. ADAA spike

First-order ADAA on the shipped Koren transfer:

`y[n] = (F(x[n]) − F(x[n−1])) / (x[n] − x[n−1])`, falling back to `f((x[n] + x[n−1])/2)` when the
denominator is below 1e-6.

The transfer `f` is recovered from `TriodeStage` **through its own public `process()`** on a
uniform 65 537-point grid over x ∈ [−8, 8] at the drive under test, so the spike can never drift
away from what the plugin ships. Between grid nodes `f` is linear, which makes `F` an exactly
integrable piecewise quadratic — the "antiderivative of the table is computable analytically per
segment" the brief asks for, without hard-coding any of `TriodeStage`'s private table constants.
The spike lives in `tests/dsp/AliasingGateTests.cpp` as the hidden report generator
`SPIKE: ADAA vs oversampling comparison (report generator)` (`[.][report]`, the same house pattern
as P1.4's fractional-delay spike). No spike branch was created; nothing to merge or delete.

### 2a. Aliasing — same stimulus, same classifier

| path | tone | drive | worst dBc | at Hz | status |
|---|---|---|---|---|---|
| naive (no antialiasing) | 1244.5 | nominal | −145.64 | 1418.0 | reference |
| naive | 1244.5 | max | −64.96 | 23110.0 | reference |
| naive | 1244.5 | headroom | −42.74 | 23110.0 | reference |
| naive | 4186.0 | nominal | −127.66 | 22884.0 | reference |
| naive | 4186.0 | max | −46.59 | 22884.0 | reference |
| naive | 4186.0 | headroom | −20.04 | 18698.0 | reference |
| ADAA1 | 1244.5 | nominal | −145.72 | 1418.0 | gated-equivalent |
| ADAA1 | 1244.5 | max | −69.89 | 23110.0 | gated-equivalent |
| ADAA1 | 1244.5 | headroom | −47.00 | 23110.0 | report |
| ADAA1 | 4186.0 | nominal | −133.85 | 22884.0 | gated-equivalent |
| ADAA1 | 4186.0 | max | **−64.30** | 22884.0 | gated-equivalent |
| ADAA1 | 4186.0 | headroom | −26.46 | 18698.0 | report |

Head to head on the four gated combinations: **oversampling 2× worst −64.93 dBc, ADAA1 worst
−64.30 dBc.** Both would pass the gate; oversampling is 0.63 dB better. On the report-only headroom
condition the gap opens to **15.7 dB** (2× −42.13 vs ADAA1 −26.46) at 4186 Hz and 14.1 dB at
1244.5 Hz. ADAA1's improvement over doing nothing at all is 5–18 dB depending on the row — real,
but roughly one order of antialiasing, with no knob to turn for more.

### 2b. CPU — nanoseconds per base-rate sample

Dev machine, Windows 11 Pro workstation / MSVC / Release, 512-sample blocks, 4000 blocks after a
64-block warm-up.

| path | ns / base sample | × naive |
|---|---|---|
| naive 1× (shipped cubic LUT) | 6.84 | 1.00 |
| ADAA1 1× (linear LUT + antiderivative) | 5.97 | 0.87 |
| Oversampler 2× + TriodeStage | 27.13 | 3.97 |
| Oversampler 4× + TriodeStage | 55.15 | 8.06 |
| Oversampler 8× + TriodeStage | 113.14 | 16.54 |

ADAA1 is **4.5× cheaper** than 2× oversampling. Two caveats that flatter ADAA and are stated here
rather than buried: the spike's ADAA runs against a *linear* interpolation of the transfer (one
multiply-add per lookup) where the shipped path evaluates a Catmull-Rom cubic, and it runs in
`double` where a shipping implementation would have to face the difference quotient's catastrophic
cancellation in `float` (dx ≈ 1e-5 in float leaves roughly 2 significant digits; the ~1e-6 epsilon
used here is safe only because the spike is `double`). A production-grade float ADAA on the cubic
LUT would land materially above 5.97 ns.

Absolute scale check: 27.13 ns/sample at 48 kHz is 0.13 % of one core for the mono triode stage.
Against the 30 % CPU gate of `docs/plan.md` section 4.7 the difference between the two paths is
noise.

### 2c. Passband transparency — the disqualifying axis

In the small-signal limit the difference quotient collapses to `f((x[n] + x[n−1])/2)`, i.e. the
waveshaper behind the two-tap FIR `(1 + z^-1)/2`, whose magnitude response is `|cos(ω/2)|`. Measured
against the static stage at default drive, −18 dBFS:

| tone | ADAA1 (dB) | Oversampler 2× (dB) | `|cos(ω/2)|` predicted (dB) |
|---|---|---|---|
| 1 000 Hz | −0.0184 | −0.0000 | −0.019 |
| 4 186 Hz | −0.3258 | 0.0000 | −0.330 |
| 10 000 Hz | −1.9886 | −0.0000 | −2.006 |
| 15 000 Hz | −5.0642 | 0.0000 | −5.100 |
| 20 000 Hz | −11.6850 | −0.0000 | −11.738 |

Measurement matches theory to within 0.05 dB at every point, which is also a useful check that the
spike's ADAA is implemented correctly rather than merely quiet.

**A guitar preamp stage cannot ship −2 dB at 10 kHz and −11.7 dB at 20 kHz.** Compensating it with
a matching shelf would re-amplify precisely the aliasing ADAA was added to suppress, in the same
band. Oversampling has no equivalent term (0.00 dB across the table, as the halfband's
0.001 dB-ripple passband requires).

### 2d. Step-response smearing

10–90 % rise, in base-rate samples, on a −18 dBFS → +18 dBFS step at drive 1.0:

| path | rise (base samples) |
|---|---|
| static 1× (reference) | 0.000 |
| ADAA1 1× | 1.000 |
| Oversampler 2× | 1.000 |
| Oversampler 4× | 1.000 |

No separation on this axis at base-rate resolution: both methods smear the step by one base sample,
which is the smallest quantity this measurement can resolve. It is recorded because the brief asks
for it, not because it discriminates.

## Decision rationale

1. **The gate is green at the locked default.** Nothing in the remedy hierarchy is triggered, and
   the plan's own fallback for this spike ("default to the `Oversampler` path — already the
   shipping architecture") is therefore the live branch.
2. **ADAA does not win the axis it would have to win.** It is cheaper, but the plan's adoption
   criterion is "passes at equal CPU" *for a red gate*; with a green gate the tiebreakers are
   quality and architecture, and ADAA loses both. It is 0.6 dB worse on the gated rows, 15.7 dB
   worse on the realistic headroom row, and it carries an −11.7 dB-at-20-kHz passband droop that
   oversampling does not.
3. **ADAA does not survive P3.** First-order ADAA requires an analytic antiderivative of a
   *memoryless* map. `TriodeStage` is memoryless only through P2 — `TriodeStage.h`'s locked caveat
   says a dynamic stage with coupling-cap and cathode-bypass state supersedes it at P3+, and the P4
   transformer and power-amp stages have state by construction. `Oversampler` wraps anything;
   adopting ADAA would mean shipping an antialiasing mechanism with a known expiry date, and
   maintaining two mechanisms in the meantime.
4. **`Oversampler` ships regardless.** It is in the P1.9 hard-wired chain and its split
   `upsample`/`downsample` API is what `docs/plan.md` open question 6 (the P4 pickup-nonlinearity
   island) is written against. ADAA would be an addition, not a replacement.
5. **The cost is affordable.** 0.13 % of one core at 2×.

## Consequences

- The shipped chain is `Oversampler(factor = 2) → TriodeStage`, wired at P1.9, reporting
  `Oversampler::latencySamples()` = **3** samples at 2× (4 at 4×, 4 at 8×) to the host.
- The `[aliasing]` gate has **4.93 dB of margin** and it is driven by the triode's own harmonic
  tail, not by the filters. Any change to `TriodeStage`'s circuit constants — which are explicitly
  unpinned — can move it. That is the gate working as intended, and it is the reason the gate runs
  in CI on every push rather than at milestones.
- **Open item carried to P1.10/P1.11:** at the full +16 dB summing headroom the 2× path reaches
  −42.1 dBc. If the bench's real `PickupTap` output levels put the triode anywhere near that
  regime, the choices are (a) raise the default factor to 4× (−54.6 dBc, 2.0× the CPU), (b) trim
  the pre-triode gain staging, or (c) accept it as a transient-only artefact above 16 kHz. This is
  a product decision on measured levels, not one this document can settle from synthetic tones, and
  raising the default factor unilaterally would reopen a locked value.
- ADAA is **not** re-evaluated again before P4. If it returns, it returns as second-order ADAA
  against a stage that is still memoryless, and the first thing to re-measure is section 2c.
- The spike is preserved as a runnable report generator, not deleted: `cnpg_tests "SPIKE: ADAA vs
  oversampling comparison (report generator)"` regenerates every number in section 2. There is no
  spike branch outstanding.

## Related decision recorded here: what `latencySamples()` reports, and why it is not the impulse peak

These are IIR halfbands, so there is no single exact integer delay — only a group delay that is flat
across the passband and peaks inside the transition band. `latencySamples()` reports the **passband
(DC) group delay of the whole up → down round trip**, at the base rate, rounded to the nearest
integer, computed in closed form from the designed coefficients (one section in `z^-2` contributes
`2(1−a)/(1+a)`; one stage's round trip contributes `D − 0.5` at its own input rate; stage `s`
contributes `(D_s − 0.5)/2^s` at the base rate). That gives 2.98 / 3.88 / 4.32 base samples at
2× / 4× / 8×, reported as 3 / 4 / 4.

The P1.8 acceptance criterion asks that this equal "the measured impulse delay through
`processWrapped` with an identity nonlinearity". Measured with a *bare* impulse it would not: a unit
impulse excites the transition-band group-delay peak as hard as the passband, and its response peak
lands a sample late (index 5 rather than 4 at 2×, with the two candidate samples within 0.7 % of
each other — a coin flip, not a latency). `OversamplerTests.cpp` therefore drives a band-limited
pulse (Blackman-windowed sinc, cutoff 0.2 × base rate) and takes the parabolically-refined
cross-correlation lag, which measures 3.106 / 4.019 / 4.470 base samples — each rounding to exactly
the reported integer, with the fractional residual asserted under 0.5. A host's integer latency
report is a passband-alignment number; that is what is measured and that is what is reported.

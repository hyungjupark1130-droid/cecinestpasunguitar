# ADR 0006 — Bridge coupling: the shipping `couplingStrength` default, and the state of the P2.5 fallback

- **Status:** Accepted
- **Date:** 2026-08-01
- **Task:** P2.4 (`BridgeJunction`)
- **Closes:** ADR 0004 "Amendments to the in-flight P0–P2 plan", item 3
- **Relates to:** locked decision Q17 (design-for-fallback), plan task P2.5 (timeboxed fallback protocol)

## Context

`BridgeJunction` is the point at which six independent `WaveguideString`s become one instrument.
It is also the task the plan shadows with a timeboxed fallback protocol (P2.5), because a
bidirectionally coupled network is where a passivity error stops being local and starts being an
oscillator.

Two decisions had to be made here and recorded outside a task report:

1. **`BridgeAdmittanceParams::couplingStrength` shipped as `0.0f` through P1–P2.3.** At `0.0` the
   junction reduces exactly to the rigid termination: the strings are fully decoupled *and*
   `bridgeOutput()` is identically zero. Every later body, chamber and pickup-feed feature reads
   that signal. Leaving the default at `0.0` would therefore have disabled all of them by a
   default rather than by a decision, and the symptom — silence on a channel nobody was listening
   to yet — is invisible until the phase that needs it.
2. **Whether the P2.5 fallback protocol is triggered**, which turns on whether the `[energy]` suite
   can be made green inside the positive-real-by-construction framing.

## Decision

### D1 — `couplingStrength` is **provisionally 0.35**

> **SUPERSEDED IN PART, 2026-08-01 — this default is PROVISIONAL, not shipped.** By author decision
> (`docs/decisions/0007-bridge-tuning-compensation.md`, D4) the value below is confirmed or replaced only after the
> **P2.8 listening pass**, which must compare *lower* coupling values and judge **mode-locking in near-unison
> voicings** by ear. **No later task may treat it as settled — explicitly including the P2.9 exit gate, which must not
> lock it by passing.** Motivating evidence: at 0.35 two strings tuned 25 cents apart mode-lock, both peaking at
> 111.297 Hz for nominals 110.00/111.60 — a **+20.286 cent pull on the string nobody detuned**, with the separation
> collapsing to 0.0029 cents — while beat depth falls with coupling (10.08 / 3.59 / 2.28 / 1.62 dB at 0.1 / 0.35 / 0.5
> / 1.0). The setting trades sympathetic richness against pitch integrity; where that trade sits is a musical
> judgement, and the measurements below establish the *range*, not the *choice*.
>
> The measurement-derived reasoning that follows remains valid and is what the listening pass should be read against.

Chosen by measurement, not by taste. The number the knob maps to is a dimensionless *peak mobility
ratio* against the string impedance,

    mu = Y(omega_0) * Z_ref = couplingStrength * kBridgeMaxMobilityRatio,   kBridgeMaxMobilityRatio = 0.05

so `couplingStrength = 1` means the bridge, at its resonance, is 5% as mobile as a matched
(perfectly absorbing) string termination. At the shipping default `mu = 0.0175`.

Measured at MIDI 53 (F3, 174.6 Hz — on the default 180 Hz bridge resonance, where the coupling is
strongest), 48 kHz, unison pair, string 0 plucked
(`tests/dsp/CoupledStringsTests.cpp`, "CoupledStrings: the couplingStrength default is a measured
choice"):

| `couplingStrength` | unplucked string's peak within 1 s | bridge-output peak | plucked string's 250 Hz-band T60 |
|---|---|---|---|
| 0.0 | exactly 0 (silent) | exactly 0 | 1.554 s |
| 0.05 | −69.7 dBFS | 2.25e−4 | 1.390 s |
| 0.1 | −63.8 dBFS | 4.46e−4 | 1.284 s |
| 0.2 | −58.0 dBFS | 8.80e−4 | 1.147 s |
| **0.35 (shipped)** | **−53.5 dBFS** | **1.51e−3** | **1.009 s** |
| 0.5 | −50.7 dBFS | 2.11e−3 | 0.907 s |
| 1.0 | −45.6 dBFS | 4.06e−3 | 0.688 s |

**Why 0.35:**

- It clears the plan's own sympathetic-response criterion (−60 dBFS within 1 s) **by 6.5 dB on its
  own**, without relying on the 0.5 the criterion happens to name. A default that only just met the
  criterion at a coupling the user has to dial in would be a default that ships the feature off.
- It costs **35% of the uncoupled sustain** in the band around the bridge resonance (T60 1.01 s vs
  1.55 s). That is the correct direction and the correct order: on a real instrument the bridge *is*
  the dominant loss for low partials, and a 1.0 s T60 at 250 Hz is an ordinary guitar figure. At
  `1.0` the same note loses 56% of its sustain, which reads as a damped instrument rather than a
  responsive one.
- It leaves the knob **useful in both directions** — audibly drier below, audibly more coupled and
  shorter above — which is what ADR 0004 means by shipping the chamber as a continuum rather than a
  toggle.
- Its cost in tuning is bounded and known: the load's phase response pulls partials near the bridge
  resonance by at most **4.90 cents** across MIDI 33–96 at all three sample rates (measured;
  `tests/dsp/WaveguideStringTuningTests.cpp` and `tests/dsp/BridgePortContractTests.cpp`).

### D1a — the tuning residual is NOT something a note-indexed calibration table can absorb

*(Added 2026-08-01 after the P2.4 review. The original text of D1 claimed the residual sat "inside
what Task P2.7's calibration table is scheduled to absorb". That is true only at the frozen default
admittance, and stating it without that qualifier misrepresents what P2.7 inherits.)*

The residual is a function of three **live APVTS parameters**, not of the MIDI note alone. Measured
at MIDI 45 / 48 kHz (`TUNING: the coupled residual is a function of three LIVE parameters`):

| swept parameter | values | residual |
|---|---|---|
| `couplingStrength` | 0.00 / 0.35 / 1.00 | 0.000 / −4.855 / −14.056 cents |
| `resonanceHz` | 80 / 110 / 180 / 2000 Hz | +4.461 / +0.001 / −4.855 / −0.527 cents |
| `damping` | 0.01 / 0.50 / 10.0 | −0.188 / −4.855 / −0.494 cents |

**The sign reverses across resonance**, and all three parameters are user-reachable while playing. A
table indexed by MIDI note can represent a residual that is a function of the note; it structurally
cannot represent one that also depends on three continuous controls and changes sign along one of
them.

Task P2.4 deliberately does **not** solve this — it measures it, prints it, and pins the shape with
an assertion, so that P2.7's scope decision is made against numbers. The options P2.7 faces (a
parameter-dependent correction, a restricted admittance range, or accepting a documented residual
and re-scoping the ±2-cent gate) are a design question for the author, not an implementation
detail, and the plan's §4.5 amendment records it as a binding entry condition on that task.

`kBridgeMaxMobilityRatio = 0.05` is the design constant behind the knob's top end. It is set so that
`couplingStrength = 1` is *strongly* coupled without being a matched termination: at `mu = 0.05` a
partial sitting on the bridge resonance loses roughly `4 mu = 20%` of its energy per round trip.

### D2 — the P2.5 fallback protocol is **NOT triggered**

The trigger defined in the plan and in the P2.4 carry-forward is "P2.4's corrected `[energy]` suite
first runs red" — tier 2 on the `double` instantiation asserting the storage-functional
`energyEstimate()`, tier 3 its float32 envelope. **Both are green, on the first corrected run, with
no clamp anywhere in the audio path**, and the margins are not marginal:

| Gate | Bound | Measured worst |
|---|---|---|
| Tier 1, `‖S̃‖₂` over the §4.2 grid × 3 rates (2304 points) | ≤ 1 + 1e−12 | 1.0 (to 4.4e−16) |
| Tier 1, sample-level energy balance, dashpot bypassed | ≤ 1e−11 relative | 2.2e−15 |
| Tier 1, energy **gain** under 201 live admittance retargets | ≤ 1e−11 | 1.8e−15 |
| Tier 2, per-block growth, 6 strings lossless, `double`, 3 rates × 2 interpolators | ≤ 1e−9 | 6.4e−15 |
| Tier 3, per-block growth, float32, 10 s, 7 admittance points | ≤ 1e−6 | 4.7e−9 |

One red `[energy]` reading did occur during development and is recorded in the P2.4 report for
completeness; it was **not** a passivity failure and did not start the timebox clock. It was the
float32 arithmetic floor — the shipping FTZ/DAZ guard flushing the recursions' own intermediate
products at total energies around 1e−69 — diagnosed by running the identical scenario on the
`double` instantiation, where it is clean over the whole decay. See the report and the standing
case `ENERGY/T3: the float32 arithmetic floor is a measurement limit, not a passivity failure`.

Q17's design-for-fallback stays intact regardless: `BridgeJunction` and the (unbuilt)
`SympatheticResonatorBus` share `IBridgePort`, `StringNetwork::setBridgePort()` substitutes one for
the other, and `tests/dsp/BridgePortContractTests.cpp` is written as an interface-level suite that a
fallback bus would inherit by adding one line.

## Consequences

- `BridgeAdmittanceParams::couplingStrength` defaults to `0.35f`; `bridgeOutput()` is a real signal
  at the shipping default, which is what the P3 body node needs.
- Three new APVTS parameters ship: `bridgeCoupling`, `bridgeResonanceHz`, `bridgeDamping`. The state
  version stays **2** (it stays 2 through all of P2, per Task P2.1).
- The single-string `string_ir` goldens were regenerated: the shipping single-string topology now
  terminates on a loaded bridge, which changes both the waveform and the band T60s.
- A sixth golden scenario, `chord_ir`, was added — the 6-string open-E chord on both the summed tap
  channel and `bridgeOutputBuffer()` — because inter-string coupling appears in no other scenario.
- Task P2.7 inherits a measured, bounded tuning residual (≤ 4.90 cents at the default admittance)
  rather than an unmeasured one, and inherits it *without* the bridge seam's own sample, which the
  loop-length solve already subtracts — **plus the open scope question in D1a**.
- The `[tuning]` suite renders the shipping coupled topology from this task on. Its P1 analytic cases
  assert a documented ±12 cent sanity bound and report; the ±2-cent criterion binds P2.7's
  calibration-table case, as plan §4.5 already assigned it. Recorded as an amendment in both plan
  copies.
- **Mode locking is a new audible behaviour**, not only a measurement one: two strings 25 cents apart
  on the shared bridge pull together, and the string that was *not* detuned is dragged **+20.29
  cents** off its own nominal while the measured separation collapses from 25 cents to 0.003. It is
  gated in `TUNING: a per-string tuning offset…` and is on the P2.8 listening checklist as its own
  item, because whether a unison-adjacent voicing sounds like an instrument or like a bug is an ear
  question.
- `SympatheticResonatorBus` is **not** built. If a later phase wants a one-way colour path it is
  still available at the same seam, but it is no longer a contingency.

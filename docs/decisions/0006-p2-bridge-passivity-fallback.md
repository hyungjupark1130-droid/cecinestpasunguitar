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

### D0 — the passing construction: a **wave-digital parallel adaptor**, passive by algebra

*This section exists because P2.5's pass-path criterion requires the decision record — not the task
report — to carry the construction and discretization that passed. Independently re-derived from
Kirchhoff's law and checked against the shipped code during the P2.4 review.*

**Not** a bridge admittance discretized into a transfer function and inverted into a scattering
matrix — that route makes passivity an empirical property of the resulting coefficients, which is
where these designs usually fail. Instead the junction is the parallel connection of N string ports
with a lumped load, solved exactly.

Kirchhoff at the shared velocity node gives

    v = 2 * sum_p (Z_p * a_p) / sigma,      b_p = v - a_p,      sigma = sum_p Z_p

In **power-normalized** wave variables `â_p = sqrt(Z_p) * a_p` this is `b̂ = S̃ â` with

    S̃ = (2/sigma) * u u^T - I,     u_p = sqrt(Z_p),     ||u||^2 = sigma

`(2/sigma) u u^T` has eigenvalue 2 on `span(u)` and 0 on its orthogonal complement, so `S̃` has
eigenvalues +1 and −1: it is a **symmetric orthogonal reflection**, and `||S̃||_2 = 1` *exactly*,
for **any** set of positive port impedances. Passivity is therefore an algebraic identity, not a
measured outcome, and there is **no clamp anywhere in the audio path** — the only `clamp` calls in
the module are parameter validation in `setAdmittance` and a bound on the port count.

**Discretization of the load elements.** The lumped load is three wave-digital one-ports joined at
the same node: a **mass** with `a[n+1] = b[n]`, a **spring** with `a[n+1] = -b[n]`, and a matched
**dashpot** with `a_R ≡ 0`, which contributes its `Z_R` to `sigma` and dissipates `Z_R * v^2`. The
element states are stored already normalized (`sM = sqrt(Z_M) * a_M`), so `sM^2` *is* that port's
stored energy and no conversion is needed at the energy boundary. The rigid termination is the
exact `v = 0` substitution rather than a stiff approximation of it. `copyScatteringMatrix` exposes
the **power-normalized** `2*sqrt(Z_i Z_j)/sigma - delta_ij`; the raw `2*Z_j/sigma - delta_ij` has
norm exceeding 1 at unequal impedances and would have been the wrong thing to publish.

**What the matrix norm does *not* carry.** A correct matrix wrapped in wrong element impedances is
still wrong, so the binding gate measures the **full energy account on `scatter()` itself, every
sample**, over ~1500 configurations (port counts 1/2/6/8, equal impedances and a 4:1 spread):
`sum Z a^2 >= sum Z b^2 + dE_stored`. Worst gain **2.22e-15**; exact balance with the dashpot
bypassed; worst single-sample dissipation **1.578** as the non-vacuity control.

**Deriving the balance found two state-bearing elements nobody had counted** — the junction's own
mass and spring (11–12% of total network energy; omitting them misses the passivity bound by
~1.28e8×), and a per-string seam register that only *becomes* state once a port drives that string.
Both are now in `energyEstimate()`. A test would have reported only that the bound failed; the
derivation is what said why.

**Measured against the three tiers:** spectral norm 1.0 to within **4.4e-16** over 2304 grid points
at three sample rates; tier 2 worst per-block growth **6.4e-15** against a 1e-9 bound; tier 3
**4.7e-9** against 1e-6.

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
>
> **REPLACED, 2026-08-05 — the default is now `0.20`** (`docs/decisions/0007-bridge-tuning-compensation.md`, **D7.2**).
> It was settled by **author delegation, NOT by the listening pass** this note reserves it for: that pass has still
> never been held and `docs/listening/P2-20260803.md` is still marked NOT PERFORMED. **D4's condition was waived, not
> met**, and the prohibition above still stands — no later task may treat 0.20 as settled *by measurement*. The table
> below is unchanged and is still the evidence: 0.20 is its `−58.0 dBFS` row, and the criterion-(4) measurement that
> selected it is ADR 0007 D7.1's, on the shipping six-string topology.

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

### D3 — Task P2.5's acceptance criteria, verified rather than re-executed

P2.5 is a *protocol*, not an implementation task, and the pass path closes it. Each criterion
checked against the tree at `5c37baa` rather than assumed:

| P2.5 criterion | State | Evidence |
|---|---|---|
| Decision doc exists, records the passing construction, no TBD text | **Met** | This file. **D0 was added specifically to close this** — it was the one criterion genuinely unmet, because the construction lived only in the task report under the gitignored `.superpowers/`. Grep for TBD/TODO/FIXME: none. |
| `BridgePortContractTests` `[contract]` passes for every `IBridgePort` implementation | **Met, and exceeded** | The suite is `TEMPLATE_TEST_CASE`-parameterized over **two** production implementations — `BridgeJunction<double>` and `RigidBridgeTermination<double>` — where the pass path anticipated one. That it is genuinely parameterized, rather than a single-implementation test wearing a contract name, is what makes the fallback's "`StringNetwork` must not be able to tell the difference" claim checkable. |
| Fail-path items (`SympatheticResonatorBus`, `research/bidirectional-bridge`) | **N/A** | Fallback not triggered. Branch list confirms no `research/bidirectional-bridge` exists, which is the correct state. |
| Timebox ≤ 2 calendar weeks from first red `[energy]` run | **Met vacuously** | The clock never started: the energy suite enters history green in `7339697`, and no red `[energy]` run was ever committed or pushed. The one red reading during development is recorded in the task report (float32 subnormal, ~1e-87 total energy) and was diagnosed rather than accommodated. |

**One judgement recorded rather than waved through.** A third type satisfies `IBridgePort` in the
compiled test binary: `RecordingPort` in `tests/dsp/StringNetworkTests.cpp`. It is a deliberate
test spy — a rigid reflection scaled by −0.5, passive by construction, existing so that "the
substituted port is the one actually being used" is observable. It is **exempt** from the shared
contract: it is not a shipped implementation, and requiring a stand-in to satisfy a physical
junction's contract would defeat the reason it exists. The criterion's intent is that no
*production* `IBridgePort` ships without passing the contract, and that holds.

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

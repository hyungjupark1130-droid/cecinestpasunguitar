#pragma once

#include <array>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/IBridgePort.h"

// BridgeJunction -- see docs/plan.md section 2.6. The N-port scattering junction that turns N
// independent WaveguideStrings into ONE coupled instrument: every string terminates on the same
// bridge point, the bridge point has a finite mechanical mobility, and therefore every string's
// motion reaches every other string. Task P2.4 lands it. Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// THE PHYSICS (derived, not fitted -- this is what makes passivity structural)
// ---------------------------------------------------------------------------------------------
// N strings are clamped to one massless bridge point. Write, for string i, `a_i` for the wave
// arriving at the bridge and `b_i` for the wave leaving it, in the SAME velocity-wave convention
// DamperJunction already uses: the transverse velocity of the string end is
//
//     v_i = a_i + b_i                                    (velocity continuity)
//
// and the transverse force that string i delivers into the junction is
//
//     F_i = Z_i (a_i - b_i)                              (Z_i = string characteristic impedance)
//
// All the string ends are the same point, so v_i = v for every i, and the forces balance against
// whatever the bridge absorbs:
//
//     sum_i Z_i (a_i - b_i) = F_load,     v = Y(s) F_load
//
// with Y the bridge's driving-point MOBILITY (velocity per unit force). Eliminating F_load,
//
//     v = 2 Y (sum_i Z_i a_i) / (1 + Y sigma),   sigma = sum_i Z_i
//     b_i = v - a_i
//
// Y = 0 is a rigid bridge (v = 0, b_i = -a_i, every string decoupled -- exactly
// RigidBridgeTermination, and exactly the couplingStrength == 0 limit). Y > 0 is a bridge that
// moves, and a bridge that moves is a bridge that carries string 0's motion into string 1.
//
// The load itself is a single lumped mechanical resonator across the bridge point -- mass m,
// spring k and dashpot r, all sharing the bridge velocity, so their FORCES add:
//
//     Z_load(s) = F_load / v = m s + k / s + r,     Y(s) = 1 / Z_load(s) = s / (m s^2 + r s + k)
//
// which is the classic single-mode driving-point mobility: zero at DC (the spring holds), zero at
// infinity (the mass holds), a real peak 1/r at omega_0 = sqrt(k/m). It is POSITIVE REAL for any
// m, k, r > 0 -- Re Y(j omega) = r omega^2 / |Z_load|^2 >= 0 -- which is the whole reason the
// parameter clamps below insist on strictly positive m, k and r rather than merely finite ones.
//
// ---------------------------------------------------------------------------------------------
// THE DISCRETIZATION: a wave-digital parallel adaptor, so passivity survives sampling
// ---------------------------------------------------------------------------------------------
// Discretizing Y and then inverting it would give a scattering matrix whose passivity depends on
// the algebra of the coefficients. Instead the load is discretized ELEMENT BY ELEMENT with the
// bilinear transform and connected as three more ports of the same junction, which is a wave
// digital filter: the bilinear transform maps a positive-real one-port onto a discrete one-port
// with |reflectance| <= 1, and a scattering junction of positive port impedances is lossless
// EXACTLY (it is Kirchhoff, nothing else). Passivity is then a property of the interconnection,
// not of a coefficient.
//
// Each grounded element becomes a port whose reference impedance is chosen so its reflectance is
// trivial (s -> (2/T)(1 - z^-1)/(1 + z^-1) throughout):
//
//     mass     Z_elem = m s      port impedance Z_M = 2m/T       reflectance  z^-1   (delay)
//     spring   Z_elem = k / s    port impedance Z_K = k T / 2    reflectance -z^-1   (negated delay)
//     dashpot  Z_elem = r        port impedance Z_R = r          reflectance  0      (matched)
//
// so the junction has N + 3 ports, the three element ports carry ONE stored sample each (the
// dashpot none -- it is matched, i.e. it absorbs everything it is sent, which is what makes it the
// only dissipative element in the whole construction), and the scattering is
//
//     sigma_total = sum_i Z_i + Z_M + Z_K + Z_R
//     v           = 2 (sum_i Z_i a_i + sqrt(Z_M) sM + sqrt(Z_K) sK) / sigma_total
//     b_i         = v - a_i
//     sM'         = sqrt(Z_M) v - sM          (mass port:   a[n] =  b[n-1])
//     sK'         = sK - sqrt(Z_K) v          (spring port: a[n] = -b[n-1])
//
// where sM, sK are the element waves held in POWER-NORMALIZED form (s = sqrt(Z) * wave). Storing
// them normalized is not a micro-optimization: it is what makes the storage functional below a
// plain sum of squares even while the coefficients are being smoothed, and what keeps the states
// numerically sane when a weak coupling drives Z_M into the millions.
//
// ---------------------------------------------------------------------------------------------
// PASSIVITY, WRITTEN OUT
// ---------------------------------------------------------------------------------------------
// Power-normalize every port, x_hat = sqrt(Z_p) x. The junction above becomes
//
//     b_hat = S~ a_hat,   S~ = (2 / sigma_total) u u^T - I,   u_p = sqrt(Z_p)
//
// S~ is real symmetric with ||u||^2 = sigma_total, so its eigenvalues are exactly
//
//     +1  on span(u)                (2 sigma_total / sigma_total - 1 = 1)
//     -1  on u's orthogonal complement
//
// i.e. S~ is an orthogonal REFLECTION and ||S~||_2 == 1 exactly, for every set of positive port
// impedances -- there is no parameter value, no sample rate and no coefficient rounding that can
// make it larger, because the only way to break it is to make some Z_p negative and every Z_p here
// is a smoothed convex combination of positive numbers. Nothing in the audio path clamps a
// scattering coefficient. (This is the same structure DamperJunction proved for N = 2 with a
// dashpot: there ||S||_2 = max(|1 - 2g|, 1) = 1.)
//
// Restricted to the N STRING ports the same matrix is
//
//     S~_ij = 2 sqrt(Z_i Z_j) / sigma_total - delta_ij
//
// whose largest singular value is max(|2 sigma / sigma_total - 1|, 1) = 1, since
// 0 < sigma <= sigma_total. That N x N block is what copyScatteringMatrix() writes and what the
// tier-1 [energy] grid measures.
//
// ---------------------------------------------------------------------------------------------
// THE STORAGE FUNCTIONAL (closed form, and why it is exactly this)
// ---------------------------------------------------------------------------------------------
// The adaptor conserves sum_p Z_p (a_p^2 - b_p^2) = 0 identically. Summed over time, the energy
// that has flowed into the mass port telescopes:
//
//     sum_{k<=n} Z_M (b_M[k]^2 - a_M[k]^2) = sum_{k<=n} (sM[k+1]^2 - sM[k]^2) = sM[n+1]^2
//
// because a_M[n] = b_M[n-1] means exactly sM[n] = (previous b_hat). The spring is identical up to
// the sign, which squares away. So the bridge's stored energy is the plain sum of squares of the
// two normalized states, and the dashpot's per-sample dissipation is Z_R v^2 >= 0. Both statements
// survive time-varying coefficients unchanged, which is why setAdmittance() may smooth.
//
// storageEnergy() returns that sum in the SAME units WaveguideString::energyEstimate() uses (which
// weights a wave sample x by x^2 / (2 Z)), so StringNetwork::energyEstimate() can add the two.
//
// ---------------------------------------------------------------------------------------------
// WHAT couplingStrength ACTUALLY SCALES
// ---------------------------------------------------------------------------------------------
// docs/plan.md section 2.6: "couplingStrength scales the FULL load admittance -- conductance AND
// susceptance". Scaling Y by c is scaling (m, k, r) by 1/c, which leaves omega_0 and the damping
// ratio zeta untouched and moves only the mobility. The knob is therefore expressed as a
// dimensionless PEAK MOBILITY RATIO against the string impedance,
//
//     mu = Y(omega_0) * Z_ref = Z_ref / r = couplingStrength * kBridgeMaxMobilityRatio
//
// so "couplingStrength = 1" means "at its resonance the bridge is kBridgeMaxMobilityRatio as
// mobile as a matched string termination would be". mu is what decides how much energy leaves a
// string per round trip near the bridge resonance (1 - |reflectance|^2 ~= 4 mu) and therefore how
// strong the sympathetic coupling is; see docs/decisions/0006 for the measured shipping default.
//
// ---------------------------------------------------------------------------------------------
// THE ONE EXTRA SAMPLE (Task P2.7 must not rediscover this)
// ---------------------------------------------------------------------------------------------
// StringNetwork gathers every string's outgoing wave, scatters, and hands the result back before
// the next tick(), so routing a string through this junction inserts EXACTLY ONE sample into its
// loop -- at every sample rate, for every note, because scatter() is memoryless in the incident
// waves (the load's own dynamics live in sM/sK, which add to the reflection rather than delaying
// it). WaveguideString::setBridgePortDriven(true) subtracts that sample from the loop-length
// solve, so the seam costs no tuning; what remains is the load's own phase response, which is
// physics (a bridge resonance pulls the partials near it) and is what P2.7's calibration table
// measures. tests/dsp/BridgePortContractTests.cpp reports the residual in cents at all three rates.

namespace cnpg::dsp {

// ---- validation window for BridgeAdmittanceParams (all enforced in setAdmittance) --------------

// Resonance floor, and the Nyquist margin the ceiling is expressed as. The floor is well below the
// lowest body/bridge resonance anything guitar-shaped has; the ceiling keeps the prewarped analog
// prototype finite (tan(pi * 0.45) is 6.31, tan(pi * 0.5) is not a number).
inline constexpr float kBridgeMinResonanceHz = 20.0f;
inline constexpr float kBridgeResonanceNyquistFraction = 0.45f;

// Damping-ratio window. The floor is STRICTLY POSITIVE and that is the positive-real constraint
// itself: zeta = 0 is a lossless resonator with poles exactly on the imaginary axis, whose discrete
// image sits exactly on the unit circle -- passive, but marginally so, and it makes Re Y(j omega)
// identically zero, i.e. a bridge that stores string energy forever and never lets go of it. 0.01
// is a Q of 50, which is a sharp but perfectly ordinary body mode.
inline constexpr float kBridgeMinDamping = 0.01f;
inline constexpr float kBridgeMaxDamping = 10.0f;

// couplingStrength = 1 means the bridge, at its resonance, is this fraction as mobile as a matched
// (perfectly absorbing) string termination. See "WHAT couplingStrength ACTUALLY SCALES" above and
// docs/decisions/0006-p2-bridge-passivity-fallback.md for the measurements behind the number.
inline constexpr float kBridgeMaxMobilityRatio = 0.05f;

// Below this mobility ratio the load is treated as exactly rigid (v == 0, b_i == -a_i,
// bridgeOutput() == 0). Two reasons: couplingStrength == 0 has a CONTRACT to be exactly rigid
// (docs/plan.md section 2.6), and the element impedances scale as 1/mu, so an unbounded mu -> 0
// would run them to infinity.
//
// THIS BRANCH IS USER-REACHABLE, and earlier text here said the opposite. `bridgeCoupling` is an
// APVTS parameter over the full 0..1 unit range, so its MINIMUM is exactly the value that enters
// this branch: dragging that slider to its stop while the instrument rings discards whatever the
// junction was holding, every time. The claim that it "fires at a value no listener and no test can
// reach" was false, and it is why the transition went unmeasured through the network for a round.
//
// It is measured now, and it is benign -- but the reason is a decomposition, not an assertion.
// Decoupling is a legitimate timbral change, so a raw click reading across the gesture measures two
// things at once. The control is a gesture to couplingStrength = 1e-6, which is acoustically the
// same decoupling but stays LOADED, so the store decays instead of being discarded: the difference
// between the two readings is the store release's own contribution, measured at <= 0.009 dB across
// four configurations including full coupling into a high-Q mode landed late in the decay.
// "CONTRACT: dragging Bridge Coupling to zero is click-free through the network"
// (tests/dsp/BridgePortContractTests.cpp) is that measurement.
inline constexpr double kBridgeMinMobilityRatio = 1.0e-9;

// Below this stored energy the junction is treated as quiescent. A THRESHOLD, not an exact-zero
// test, and the reason is that the states decay GEOMETRICALLY: a `double` reaches exact zero only by
// underflow, ~700 dB down, which for a lightly-damped bridge mode is minutes of silence. An
// exact-zero isQuiescent() therefore never became true in practice, which meant
// StringNetwork::process's `live` predicate never went false again, which meant the idle-string skip
// never fired again after the first note -- the exact "eight strings tick for ever after one note"
// outcome the silence watchdog exists to prevent.
//
// The value is the bridge-side counterpart of StringNetwork's kSilenceFloor (1e-5, i.e. -100 dBFS on
// a wave amplitude): a wave of that amplitude carries x^2/2 = 5e-11 of storage in these units, so a
// junction holding less than that is holding less than the level at which a whole STRING is
// declared silent and cleared. Judging it is the same class of decision, made at the same level.
inline constexpr double kBridgeQuiescentEnergy = 5.0e-11;

// Per-sample smoothing time for admittance changes, matching the 8 ms convention WaveguideString,
// StringNetwork and DamperJunction already use. The smoothed quantities are sqrt(Z_M), sqrt(Z_K)
// and Z_R, so sigma_total is recomputed from the SAME numbers the reflection uses and the junction
// is a legal passive adaptor at every sample of the glide rather than only at the endpoints.
inline constexpr double kBridgeSmoothingSeconds = 0.008;

// Per docs/plan.md section 2.1 every sample-domain class is template <typename SampleT> with
// explicit float/double instantiations compiled into cnpg_dsp -- the realtime path uses float, the
// tier-2 [energy] tests run double.
template <typename SampleT> class BridgeJunction final : public IBridgePort<SampleT> {
  public:
    // Message thread. Allocates nothing (storage is a fixed kMaxStrings array), computes the
    // smoother coefficient for `sampleRate`, re-derives the load coefficients at the new rate, and
    // calls reset(). `numPorts` is clamped to [1, kMaxStrings]; `portImpedances` may be null, in
    // which case every port is given unit impedance.
    void prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) override;

    // Realtime-safe. Clears both element states, snaps the coefficient smoothers onto their
    // targets, and zeroes the published bridge velocity. setLossBypassed() is a test-mode
    // configuration and survives reset, exactly as WaveguideString::setLossBypassed does.
    void reset() noexcept override;

    // THE per-sample entry point. `numPorts` may be smaller than the prepared count -- StringNetwork
    // hands over only the strings currently in its loop -- and the junction then loads against
    // exactly those ports.
    void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept override;

    // The bridge point's VELOCITY: the signal a body node is driven by and a bridge pickup would
    // read. Identically 0 when couplingStrength is 0, because a rigid bridge does not move.
    SampleT bridgeOutput() const noexcept override { return velocity_; }

    // Tier-2 [energy] hook. Removes the dashpot port entirely (rather than setting r = 0, which
    // would be a matched-to-nothing port with a delay-free self-loop), leaving a lossless mass +
    // spring resonator. The junction still COUPLES the strings -- which is the point, and the
    // difference from DamperJunction, where lossless mode makes the junction transparent and the
    // tier-2 scenario vacuous (docs/plan.md section 4.2, P2.3 amendment).
    void setLossBypassed(bool bypass) noexcept override;

    // Realtime-safe; only retargets the smoothers. Every field is clamped into the positive-real
    // region at SET TIME (damping onto a strictly positive floor, resonance under the Nyquist
    // margin, couplingStrength into 0..1), and a NaN in any field resolves to that field's lower
    // bound rather than propagating into the audio path.
    void setAdmittance(const BridgeAdmittanceParams& p) noexcept override;

    // True when the junction stores nothing, i.e. when zero incident waves imply zero outgoing
    // waves forever. StringNetwork's idle-string skip needs this: under bidirectional coupling a
    // string with no state of its own can still be driven THROUGH the bridge, so "nothing to do"
    // has to include the junction's own state (see StringNetwork.cpp's `live` predicate).
    bool isQuiescent() const noexcept override;

    // The Lyapunov storage of the two reactive element states, in the same units as
    // WaveguideString::energyEstimate(). Summed into StringNetwork::energyEstimate().
    Sample64 storageEnergy() const noexcept override;

    // Tier-1 [energy] hook (docs/plan.md section 4.2). Writes the POWER-NORMALIZED N x N scattering
    // matrix over the string ports, row-major with row stride `maxPorts`:
    //
    //     rowMajorS[i * maxPorts + j] = 2 sqrt(Z_i Z_j) / sigma_total - delta_ij
    //
    // Power-normalized because that is the basis the passivity claim is made in (docs/plan.md
    // section 2.6: "wave variables are power-normalized against per-port impedances supplied at
    // prepare"); the raw-port matrix 2 Z_j / sigma_total - delta_ij is similar to it but its
    // spectral norm exceeds 1 whenever the impedances differ, which would make the tier-1 bound a
    // statement about a coordinate system rather than about energy.
    //
    // In double regardless of SampleT. Writes nothing if `rowMajorS` is null or `maxPorts` is
    // smaller than the prepared port count. Not realtime-safe by intent (it is a test hook), though
    // it does in fact allocate nothing.
    //
    // NOTE the port count it uses: the PREPARED one (`lastScatterPorts()` reports what scatter() was
    // last handed, which StringNetwork varies with its trip count). The two agree in every
    // configuration this is called from -- the tier-1 grid prepares exactly the port count it
    // sweeps -- and where they would not, the prepared count is the right answer for a matrix
    // describing the junction rather than one particular block's loop.
    void copyScatteringMatrix(double* rowMajorS, int maxPorts) const;

    // ---- diagnostics (tests, and later cnpg_calibrate) -----------------------------------------

    // The VALIDATED parameters actually in force, i.e. after the positive-real clamps. Exposed so a
    // test can assert the validation directly instead of inferring it from a measured response, and
    // so every case can assert it is in the state it claims to exercise.
    float currentResonanceHz() const noexcept { return resonanceHz_; }
    float currentDamping() const noexcept { return damping_; }
    float currentCouplingStrength() const noexcept { return couplingStrength_; }

    // The instantaneous (memoryless) mobility the ports see, i.e. 1 / (Z_M + Z_K + Z_R): the
    // "positive load conductance" the scattering matrix is built from. Exactly 0 in the rigid
    // limit. This is the quantity whose positivity carries ||S~||_2 <= 1.
    double instantaneousMobility() const noexcept;

    // Number of ports the junction is currently loading against, i.e. the `numPorts` of the most
    // recent scatter() call. A junction that is never ticked reports 0, which is what makes a
    // mis-wired port fail loudly instead of degrading into a rigid reflection nobody notices.
    int lastScatterPorts() const noexcept { return lastScatterPorts_; }

    // Total number of scatter() calls since reset(). The other half of the liveness assertion.
    unsigned long long scatterCount() const noexcept { return scatterCount_; }

    // Mean of the port impedances supplied at prepare -- the Z_ref the mobility ratio mu is
    // expressed against, and the scale storageEnergy() converts through. Exposed so an energy
    // test can convert between the junction's own (adaptor) units, where a wave x on a port of
    // impedance Z carries Z x^2, and WaveguideString's, without re-deriving the conversion.
    double referenceImpedance() const noexcept { return referenceImpedance_; }

  private:
    void refreshTargets() noexcept; // maps the validated parameters onto the element impedances
    void snapSmoothers() noexcept;
    void advanceSmoothers() noexcept;
    double stringImpedanceSum(int numPorts) const noexcept;

    // Below this distance a smoother snaps onto its target rather than asymptoting forever -- the
    // same reason DamperJunction has one: a value crawling toward its target eventually becomes a
    // denormal, and "fully arrived" has to be a reachable state for a test to assert it. Relative,
    // because the impedances span many orders of magnitude across the parameter range.
    static constexpr double kSmootherSettleRelative = 1.0e-12;

    double sampleRate_ = 44100.0;
    int numPorts_ = 1;
    std::array<double, kMaxStrings> impedance_{};    // Z_i, as supplied at prepare
    std::array<double, kMaxStrings> impedanceSum_{}; // prefix sums of Z_i, so scatter() adds none
    double referenceImpedance_ = 1.0;                // mean Z_i; the scale mu is expressed against

    // Validated parameters (what currentResonanceHz() and friends report).
    float resonanceHz_ = 180.0f;
    float damping_ = 0.5f;
    float couplingStrength_ = 0.0f;

    // Targets and smoothed values of the three element quantities. rootZM_/rootZK_ are sqrt of the
    // port impedances, NOT the impedances: sigma_total is then formed as rootZM^2 + rootZK^2 + ZR
    // from the very same numbers scatter() multiplies by, so the adaptor cannot be inconsistent
    // with itself mid-glide -- which is what makes passivity hold DURING a parameter move and not
    // merely at its endpoints.
    double rootZMTarget_ = 0.0;
    double rootZKTarget_ = 0.0;
    double resistanceTarget_ = 0.0;
    double rootZM_ = 0.0;
    double rootZK_ = 0.0;
    double resistance_ = 0.0;
    bool rigid_ = true; // couplingStrength below kBridgeMinMobilityRatio: the exact Y == 0 limit
    bool rigidTarget_ = true;

    double smoothingCoeff_ = 0.0;

    // The whole of the junction's state: two normalized element waves.
    double massState_ = 0.0;
    double springState_ = 0.0;

    SampleT velocity_ = SampleT(0);
    bool lossBypassed_ = false;

    int lastScatterPorts_ = 0;
    unsigned long long scatterCount_ = 0;
};

extern template class BridgeJunction<float>;  // realtime path
extern template class BridgeJunction<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

#include "cnpg/dsp/BridgeJunction.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

constexpr double kPi = 3.141592653589793;

// Clamp that resolves NaN onto `lo` rather than propagating it, exactly as DamperJunction's does
// and for the same reason: every comparison against a NaN is false, so std::clamp returns the NaN
// untouched and the audio path inherits it. Written as a rejection of "is it inside the range", so
// the NaN case falls out of the first test rather than needing its own branch.
double sanitize(float value, double lo, double hi) noexcept {
    const auto v = static_cast<double>(value);
    if (!(v >= lo))
        return lo;
    return (v > hi) ? hi : v;
}

double smootherCoefficient(double timeConstantSeconds, double sampleRate) noexcept {
    if (!(timeConstantSeconds > 0.0) || !(sampleRate > 0.0))
        return 1.0; // degenerate configuration: snap rather than divide by zero
    return 1.0 - std::exp(-1.0 / (timeConstantSeconds * sampleRate));
}

// One smoothing step with a RELATIVE settle snap. Relative rather than absolute because the
// element impedances span many orders of magnitude across the parameter range (Z_M runs from
// single digits to millions as the coupling weakens), so a fixed epsilon would either never fire at
// the top of the range or snap the whole glide at the bottom.
void advanceSmoother(double& value, double target, double coefficient, double relativeEpsilon) noexcept {
    const double distance = target - value;
    if (std::fabs(distance) <= relativeEpsilon * (std::fabs(target) + 1.0)) {
        value = target;
        return;
    }
    value += coefficient * distance;
}

} // namespace

// ------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void BridgeJunction<SampleT>::prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) {
    // Accepted for lifecycle uniformity only (docs/plan.md section 2.1): the junction owns no
    // buffers, because a memoryless-in-the-incident-waves N-port has nothing to buffer.
    (void)maxBlockSize;

    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    numPorts_ = std::clamp(numPorts, 1, kMaxStrings);

    double total = 0.0;
    for (int port = 0; port < kMaxStrings; ++port) {
        double z = 1.0;
        if (portImpedances != nullptr && port < numPorts_) {
            // A non-positive or NaN impedance would break the passivity argument at its root (the
            // eigen-decomposition of S~ needs u_p = sqrt(Z_p) real and the reflection needs
            // sigma_total > 0), so it resolves to the normalized unit impedance rather than
            // reaching the scattering coefficients.
            const auto supplied = static_cast<double>(portImpedances[port]);
            z = (supplied > 0.0) ? supplied : 1.0;
        }
        impedance_[static_cast<std::size_t>(port)] = z;
        if (port < numPorts_)
            total += z;
        impedanceSum_[static_cast<std::size_t>(port)] = total; // inclusive prefix sum
    }
    referenceImpedance_ = total / static_cast<double>(numPorts_);

    smoothingCoeff_ = smootherCoefficient(kBridgeSmoothingSeconds, sampleRate_);
    refreshTargets(); // the mapping is sample-rate dependent (bilinear), so re-run it here
    reset();
}

template <typename SampleT> void BridgeJunction<SampleT>::reset() noexcept {
    massState_ = 0.0;
    springState_ = 0.0;
    velocity_ = SampleT(0);
    lastScatterPorts_ = 0;
    scatterCount_ = 0;
    snapSmoothers();
    // lossBypassed_ deliberately survives: it is a test-mode configuration, exactly as
    // WaveguideString::setLossBypassed and DamperJunction::setLossBypassed are.
}

template <typename SampleT> void BridgeJunction<SampleT>::setLossBypassed(bool bypass) noexcept {
    lossBypassed_ = bypass;
}

// ------------------------------------------------------------------------------------------
// parameter mapping -- BridgeAdmittanceParams -> positive element impedances
// ------------------------------------------------------------------------------------------

template <typename SampleT> void BridgeJunction<SampleT>::setAdmittance(const BridgeAdmittanceParams& p) noexcept {
    // Every clamp is HERE, at set time, and nothing downstream re-checks: that is the difference
    // between "passive because the parameters were validated" and "passive because a coefficient
    // was clamped in the audio path". The audio path below multiplies by whatever these produce.
    const double nyquistCeiling = std::max(static_cast<double>(kBridgeMinResonanceHz),
                                           static_cast<double>(kBridgeResonanceNyquistFraction) * sampleRate_);
    resonanceHz_ =
        static_cast<float>(sanitize(p.resonanceHz, static_cast<double>(kBridgeMinResonanceHz), nyquistCeiling));
    // The clamp is computed in double and STORED in float, and float rounding is to nearest -- so
    // the stored value can land just ABOVE the ceiling it was clamped to (0.45f * 48000 is
    // 21599.99942..., whose nearest float is exactly 21600). Stepping back down keeps the invariant
    // "the value in force is <= the ceiling" exactly true, which is what makes it assertable rather
    // than approximately true, and costs one ulp of a parameter nobody can hear.
    while (static_cast<double>(resonanceHz_) > nyquistCeiling && resonanceHz_ > kBridgeMinResonanceHz)
        resonanceHz_ = std::nextafter(resonanceHz_, 0.0f);
    damping_ = static_cast<float>(
        sanitize(p.damping, static_cast<double>(kBridgeMinDamping), static_cast<double>(kBridgeMaxDamping)));
    couplingStrength_ = static_cast<float>(sanitize(p.couplingStrength, 0.0, 1.0));
    refreshTargets();
}

template <typename SampleT> void BridgeJunction<SampleT>::refreshTargets() noexcept {
    const double mu = static_cast<double>(couplingStrength_) * static_cast<double>(kBridgeMaxMobilityRatio);
    if (mu < kBridgeMinMobilityRatio) {
        // The exact Y == 0 limit: an immovable bridge. Kept as its own branch rather than as a very
        // large impedance, because docs/plan.md section 2.6 gives couplingStrength == 0 a CONTRACT
        // (strings fully decoupled AND bridgeOutput() identically 0) that "very nearly rigid" does
        // not satisfy, and because the element impedances scale as 1/mu.
        rigidTarget_ = true;
        rootZMTarget_ = 0.0;
        rootZKTarget_ = 0.0;
        resistanceTarget_ = 0.0;
        return;
    }
    rigidTarget_ = false;

    // r is fixed by the peak mobility ratio: Y(omega_0) = 1/r = mu / Z_ref.
    const double r = referenceImpedance_ / mu;

    // Bilinear PREWARPING, so the discrete resonance lands on the requested frequency rather than
    // on its arctangent-compressed image. The analog prototype is designed at
    // omega_a = (2/T) tan(pi f0 / fs); at that omega the mass and spring reactances cancel exactly
    // in the discretized load and the mobility peak is exactly 1/r.
    const double t = 1.0 / sampleRate_;
    // resonanceHz_ is already clamped under kBridgeResonanceNyquistFraction * fs, so the tangent
    // argument never reaches pi/2 and `warped` is always finite and positive.
    const double warped = (2.0 / t) * std::tan(kPi * static_cast<double>(resonanceHz_) / sampleRate_);
    const double zeta = static_cast<double>(damping_);

    // m = r / (2 zeta omega_0), k = m omega_0^2. Port impedances for the bilinear-discretized
    // grounded elements (see the header): Z_M = 2m/T, Z_K = k T / 2, Z_R = r.
    const double mass = r / (2.0 * zeta * warped);
    const double zM = 2.0 * mass / t;
    const double zK = mass * warped * warped * t * 0.5;

    // sqrt is taken HERE, once per parameter change, and the smoothers then run on the roots --
    // scatter() never calls sqrt, and sigma_total is rebuilt from the same roots it multiplies by.
    rootZMTarget_ = std::sqrt(zM);
    rootZKTarget_ = std::sqrt(zK);
    resistanceTarget_ = r;
}

template <typename SampleT> void BridgeJunction<SampleT>::snapSmoothers() noexcept {
    rootZM_ = rootZMTarget_;
    rootZK_ = rootZKTarget_;
    resistance_ = resistanceTarget_;
    rigid_ = rigidTarget_;
}

template <typename SampleT> void BridgeJunction<SampleT>::advanceSmoothers() noexcept {
    // The rigid/loaded distinction is a topology change, not a coefficient, so it is taken
    // immediately rather than glided. It can only fire below kBridgeMinMobilityRatio of full
    // coupling -- 180 dB under the shipping default -- where both element states are numerically
    // zero and there is nothing for the change to step.
    if (rigid_ != rigidTarget_) {
        rigid_ = rigidTarget_;
        rootZM_ = rootZMTarget_;
        rootZK_ = rootZKTarget_;
        resistance_ = resistanceTarget_;
        if (rigid_) {
            // ENTERING THE RIGID BRANCH RELEASES THE STORE, and that is a considered reversal of
            // what this file said at first. The rigid branch is the Y == 0 limit: v is identically
            // 0, so the mass state merely alternates sign and the spring state never moves, and the
            // energy is conserved FOR EVER. Leaving it there is not conservative, it is a leak in
            // the other direction -- it froze isQuiescent() false permanently, and turning the
            // coupling back up released the whole store into the strings from zero incident
            // (measured by review: 0.1864 of storage held unchanged over 500 000 silent samples,
            // then a peak outgoing wave of 0.0173 out of nothing).
            //
            // Zeroing it is NOT a clamp. A clamp is an in-loop limiter that bounds a signal every
            // sample; this is a one-time state release on a topology change, which is the same
            // thing clearStringState() does to a ringing string's rails and the same thing the
            // silence watchdog does when it decides a tail is over. It only ever runs below
            // kBridgeMinMobilityRatio of full coupling -- 180 dB under the shipping default -- where
            // the store it releases is whatever the user asked to disconnect.
            massState_ = 0.0;
            springState_ = 0.0;
        }
        return;
    }
    if (rigid_)
        return;
    advanceSmoother(rootZM_, rootZMTarget_, smoothingCoeff_, kSmootherSettleRelative);
    advanceSmoother(rootZK_, rootZKTarget_, smoothingCoeff_, kSmootherSettleRelative);
    advanceSmoother(resistance_, resistanceTarget_, smoothingCoeff_, kSmootherSettleRelative);
}

template <typename SampleT> double BridgeJunction<SampleT>::stringImpedanceSum(int numPorts) const noexcept {
    if (numPorts <= 0)
        return 0.0;
    const int last = std::min(numPorts, kMaxStrings) - 1;
    return impedanceSum_[static_cast<std::size_t>(last)];
}

// ------------------------------------------------------------------------------------------
// scattering
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void BridgeJunction<SampleT>::scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept {
    const int ports = std::clamp(numPorts, 0, kMaxStrings);
    lastScatterPorts_ = ports;
    ++scatterCount_;

    if (incident == nullptr || outgoing == nullptr || ports == 0) {
        velocity_ = SampleT(0);
        advanceSmoothers();
        return;
    }

    if (rigid_) {
        // v == 0: every string sees its own wave inverted, which IS the rigid termination. The
        // element states were released on the way into this branch (advanceSmoothers()), so there
        // is nothing here to conserve or to discard -- they are already 0 and the recurrences below
        // keep them there.
        for (int port = 0; port < ports; ++port)
            outgoing[port] = -incident[port];
        massState_ = -massState_;
        velocity_ = SampleT(0);
        advanceSmoothers();
        return;
    }

    // sum_i Z_i a_i, plus the two element ports' normalized states. The dashpot contributes
    // nothing to the numerator because it is MATCHED: it reflects zero, which is the whole reason
    // it costs no state.
    double weighted = 0.0;
    for (int port = 0; port < ports; ++port)
        weighted += impedance_[static_cast<std::size_t>(port)] * static_cast<double>(incident[port]);
    weighted += rootZM_ * massState_ + rootZK_ * springState_;

    // sigma_total is rebuilt from the same rootZM_/rootZK_/resistance_ the reflection uses, so the
    // adaptor is a legal positive-impedance parallel junction at EVERY sample of a parameter glide.
    double sigma = stringImpedanceSum(ports) + rootZM_ * rootZM_ + rootZK_ * rootZK_;
    if (!lossBypassed_)
        sigma += resistance_;

    const double v = (sigma > 0.0) ? (2.0 * weighted / sigma) : 0.0;

    for (int port = 0; port < ports; ++port)
        outgoing[port] = static_cast<SampleT>(v) - incident[port];

    // Element port updates. Mass reflects with a plain unit delay, spring with a negated one; both
    // written as "new normalized state = f(v, old state)" so the stored quantity is always the
    // power-normalized wave and the storage functional stays a plain sum of squares.
    const double massOut = rootZM_ * v - massState_;
    const double springOut = rootZK_ * v - springState_;
    massState_ = massOut;
    springState_ = -springOut;

    velocity_ = static_cast<SampleT>(v);
    advanceSmoothers();
}

template <typename SampleT> bool BridgeJunction<SampleT>::isQuiescent() const noexcept {
    // A THRESHOLD on the stored energy, not an exact-zero test on the states -- see
    // kBridgeQuiescentEnergy for why the exact test could not become true and what that cost.
    return storageEnergy() <= kBridgeQuiescentEnergy;
}

template <typename SampleT> Sample64 BridgeJunction<SampleT>::storageEnergy() const noexcept {
    // (sM^2 + sK^2) is the stored energy in the junction's own (adaptor) units, where a wave x on a
    // port of impedance Z carries Z x^2. WaveguideString::energyEstimate() weights a wave sample by
    // x^2 / (2 Z) instead, so the two agree only after dividing by 2 Z_ref^2 -- which is exactly
    // 1/2 at the unit impedance every string currently publishes. Written out rather than hardcoded
    // to 1/2 so a future per-string impedance (a gauge/tension model) changes one expression here
    // and not the meaning of the tier-2 bound. NOTE: a NON-UNIFORM impedance set would additionally
    // need WaveguideString's own 1/(2 Z) weighting revisited -- see the P2.4 report.
    const double scale = 2.0 * referenceImpedance_ * referenceImpedance_;
    return (massState_ * massState_ + springState_ * springState_) / scale;
}

template <typename SampleT> double BridgeJunction<SampleT>::instantaneousMobility() const noexcept {
    if (rigid_)
        return 0.0;
    double load = rootZM_ * rootZM_ + rootZK_ * rootZK_;
    if (!lossBypassed_)
        load += resistance_;
    return (load > 0.0) ? (1.0 / load) : 0.0;
}

template <typename SampleT> void BridgeJunction<SampleT>::copyScatteringMatrix(double* rowMajorS, int maxPorts) const {
    if (rowMajorS == nullptr || maxPorts < numPorts_)
        return;

    if (rigid_) {
        for (int i = 0; i < numPorts_; ++i)
            for (int j = 0; j < numPorts_; ++j)
                rowMajorS[static_cast<std::size_t>(i) * static_cast<std::size_t>(maxPorts) +
                          static_cast<std::size_t>(j)] = (i == j) ? -1.0 : 0.0;
        return;
    }

    double sigma = stringImpedanceSum(numPorts_) + rootZM_ * rootZM_ + rootZK_ * rootZK_;
    if (!lossBypassed_)
        sigma += resistance_;
    if (!(sigma > 0.0))
        return;

    for (int i = 0; i < numPorts_; ++i) {
        const double ui = std::sqrt(impedance_[static_cast<std::size_t>(i)]);
        for (int j = 0; j < numPorts_; ++j) {
            const double uj = std::sqrt(impedance_[static_cast<std::size_t>(j)]);
            const double kronecker = (i == j) ? 1.0 : 0.0;
            rowMajorS[static_cast<std::size_t>(i) * static_cast<std::size_t>(maxPorts) + static_cast<std::size_t>(j)] =
                2.0 * ui * uj / sigma - kronecker;
        }
    }
}

template class BridgeJunction<float>;  // realtime path
template class BridgeJunction<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

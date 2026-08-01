#include "cnpg/dsp/DamperJunction.h"

#include <cmath>

namespace cnpg::dsp {

namespace {

// Clamp that resolves NaN onto `lo` rather than propagating it. std::clamp cannot: every
// comparison against a NaN is false, so it returns the NaN untouched and the audio path inherits
// it. Written as a rejection of "is it inside the range" so the NaN case falls out of the first
// test rather than needing its own branch.
double sanitize(float value, double lo, double hi) noexcept {
    const auto v = static_cast<double>(value);
    if (!(v >= lo))
        return lo;
    return (v > hi) ? hi : v;
}

// One-pole per-sample smoothing coefficient for a time constant in seconds.
double smootherCoefficient(double timeConstantSeconds, double sampleRate) noexcept {
    if (!(timeConstantSeconds > 0.0) || !(sampleRate > 0.0))
        return 1.0; // degenerate configuration: snap rather than divide by zero
    return 1.0 - std::exp(-1.0 / (timeConstantSeconds * sampleRate));
}

// One smoothing step, with the settle snap described in DamperJunction.h.
void advanceSmoother(double& value, double target, double coefficient, double epsilon) noexcept {
    const double distance = target - value;
    if (std::fabs(distance) < epsilon) {
        value = target;
        return;
    }
    value += coefficient * distance;
}

} // namespace

// ------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------

template <typename SampleT> void DamperJunction<SampleT>::prepare(double sampleRate, int maxBlockSize) {
    // Accepted for lifecycle uniformity only (docs/plan.md section 2.1): this class owns no
    // buffers, because a memoryless two-port has nothing to buffer.
    (void)maxBlockSize;

    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    lossCoeff_ = smootherCoefficient(kLossSmoothingSeconds, sampleRate_);
    // Re-derived from the felt time the current parameters carry, so a prepare() at a new rate
    // does not leave the ramp running at the old rate's speed.
    engagementCoeff_ = smootherCoefficient(static_cast<double>(feltTimeConstantMs_) * 0.001, sampleRate_);
    reset();
}

template <typename SampleT> void DamperJunction<SampleT>::reset() noexcept {
    engagement_ = 0.0;
    engagementTarget_ = 0.0;
    // Snapped onto the parameter, not zeroed: reset() must leave the instance indistinguishable
    // from a freshly prepared one CARRYING THE SAME PARAMETERS, and the loss depth is a parameter.
    lossDepth_ = lossTarget_;
    // lossBypassed_ deliberately survives: it is a test-mode configuration, exactly as
    // WaveguideString::setLossBypassed is.
}

template <typename SampleT> void DamperJunction<SampleT>::setParams(const DamperJunctionParams& p) noexcept {
    position01_ = static_cast<float>(sanitize(p.position01, 0.0, 1.0));
    lossTarget_ = sanitize(p.maxLoss, 0.0, 1.0);

    feltTimeConstantMs_ = static_cast<float>(sanitize(p.feltTimeConstantMs, static_cast<double>(kFeltTimeConstantMinMs),
                                                      static_cast<double>(kFeltTimeConstantMaxMs)));
    engagementCoeff_ = smootherCoefficient(static_cast<double>(feltTimeConstantMs_) * 0.001, sampleRate_);
}

// ------------------------------------------------------------------------------------------
// engagement
// ------------------------------------------------------------------------------------------

template <typename SampleT> void DamperJunction<SampleT>::engage() noexcept { engagementTarget_ = 1.0; }

template <typename SampleT> void DamperJunction<SampleT>::release() noexcept { engagementTarget_ = 0.0; }

template <typename SampleT> void DamperJunction<SampleT>::setEngagementImmediate(float engagement01) noexcept {
    engagement_ = sanitize(engagement01, 0.0, 1.0);
    engagementTarget_ = engagement_;
}

template <typename SampleT> void DamperJunction<SampleT>::setLossBypassed(bool bypass) noexcept {
    lossBypassed_ = bypass;
}

// ------------------------------------------------------------------------------------------
// scattering
// ------------------------------------------------------------------------------------------

template <typename SampleT> double DamperJunction<SampleT>::lossCoefficient() const noexcept {
    if (lossBypassed_)
        return 0.0;
    // s = engagement * lossDepth in [0, 1]; R = 2 Z0 s; g = R / (R + 2 Z0) = s / (s + 1).
    // Both factors are convex combinations of values in [0, 1] and therefore stay in [0, 1]
    // WITHOUT a clamp, so s >= 0 -- the positive junction conductance the passivity argument in
    // DamperJunction.h needs -- holds by construction. s == 0 gives exactly +0.0.
    const double s = engagement_ * lossDepth_;
    return s / (s + 1.0);
}

template <typename SampleT>
void DamperJunction<SampleT>::scatter(SampleT fromNut, SampleT fromBridge, SampleT& toBridge, SampleT& toNut) noexcept {
    const auto g = static_cast<SampleT>(lossCoefficient());

    // At g == 0 `common` is a signed zero and both lines below are exact copies -- the bit-exact
    // pass-through the transparency contract asserts. Subtracting (rather than adding a negated
    // coefficient) is what keeps that true for a negative-zero operand as well.
    const SampleT common = g * (fromNut + fromBridge);
    toBridge = fromNut - common;
    toNut = fromBridge - common;

    // Advanced AFTER the sample: see the declaration. engage() consumed at sample n leaves sample
    // n bit-exactly transparent and starts the damping at n + 1.
    advanceSmoother(engagement_, engagementTarget_, engagementCoeff_, kSmootherSettleEpsilon);
    advanceSmoother(lossDepth_, lossTarget_, lossCoeff_, kSmootherSettleEpsilon);
}

template <typename SampleT> void DamperJunction<SampleT>::copyScatteringMatrix(double* rowMajorS2x2) const {
    if (rowMajorS2x2 == nullptr)
        return;
    const double g = lossCoefficient();
    rowMajorS2x2[0] = -g;      // S_nut,nut
    rowMajorS2x2[1] = 1.0 - g; // S_nut,bridge
    rowMajorS2x2[2] = 1.0 - g; // S_bridge,nut
    rowMajorS2x2[3] = -g;      // S_bridge,bridge
}

template class DamperJunction<float>;  // realtime path
template class DamperJunction<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

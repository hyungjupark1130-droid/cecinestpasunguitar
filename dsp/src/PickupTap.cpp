#include "cnpg/dsp/PickupTap.h"

#include "cnpg/dsp/StringNetwork.h" // full StringTapBuffers<Sample> definition

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// Keeps w0 = 2*pi*resonanceHz/sampleRate comfortably clear of both 0 and Nyquist -- sin(w0) is
// alpha's numerator, and letting it approach 0 at either end would push the biquad toward the
// degenerate all-zero-numerator case -- regardless of what a caller (APVTS automation, a
// malformed preset, ...) hands setParams().
constexpr float kMinResonanceHz = 20.0f;
constexpr float kMaxResonanceHzRatio = 0.45f; // ceiling expressed as a fraction of sampleRate
constexpr float kMinQ = 0.05f;
constexpr float kMaxQ = 50.0f;

float clampFinite(float v, float lo, float hi) noexcept {
    if (!(v == v)) // NaN guard: NaN compares unequal to itself
        return lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

float dbToLinear(float gainDb) noexcept { return std::pow(10.0f, gainDb / 20.0f); }

} // namespace

PickupTap::Coeffs PickupTap::computeCoeffs(float resonanceHz, float q, double sampleRate) noexcept {
    const double rate = (sampleRate > 0.0) ? sampleRate : 44100.0;
    const auto maxHz = static_cast<float>(kMaxResonanceHzRatio * rate);
    const float safeHz = clampFinite(resonanceHz, kMinResonanceHz, std::max(kMinResonanceHz, maxHz));
    const float safeQ = clampFinite(q, kMinQ, kMaxQ);

    // RBJ-style constant SKIRT-gain bandpass (Audio EQ Cookbook conventions -- peak gain == q, not
    // a fixed 0 dB): the bilinear discretization of the displacement->EMF transfer function
    // s*w0^2/(s^2+(w0/Q)s+w0^2), a pole pair placed from (safeHz, safeQ) with the numerator's DC
    // zero doing the differentiation the magnetic transducer itself performs -- see the header
    // comment. The CONSTANT-PEAK-GAIN variant (b0 = alpha/a0) was tried first and rejected: its
    // numerator scales with alpha = sin(w0)/(2*safeQ), so the whole low/mid register would have
    // scaled as 1/(q*resonanceHz) -- tens of dB across the user-facing q/resonanceHz range, which
    // both parameters are -- contradicting the -18 dBFS nominal structure. b0 = sin(w0)/2/a0 has
    // no q or resonanceHz dependence beyond w0 itself, so peak gain is q by construction (as the
    // physical transfer function's own numerator magnitude at resonance requires) and nothing else
    // moves with loading.
    const double w0 = kTwoPi * static_cast<double>(safeHz) / rate;
    const double alpha = std::sin(w0) / (2.0 * static_cast<double>(safeQ));
    const double a0 = 1.0 + alpha;

    Coeffs c;
    c.b0 = std::sin(w0) / 2.0 / a0;
    c.b1 = 0.0;
    c.b2 = -c.b0;
    c.a1 = (-2.0 * std::cos(w0)) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

double PickupTap::biquadTick(double x, const Coeffs& c) noexcept {
    const double y = c.b0 * x + c.b1 * x1_ + c.b2 * x2_ - c.a1 * y1_ - c.a2 * y2_;
    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;
    return y;
}

void PickupTap::prepare(double sampleRate, int maxBlockSize) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(1, maxBlockSize);
    scratchMono_.assign(static_cast<std::size_t>(maxBlockSize_), Sample(0));

    const PickupTapParams defaults{};
    targetCoeffs_ = computeCoeffs(defaults.resonanceHz, defaults.q, sampleRate_);
    targetGainLinear_ = dbToLinear(defaults.outputGainDb);
    currentCoeffs_ = targetCoeffs_;
    currentGainLinear_ = targetGainLinear_;

    reset();
}

void PickupTap::reset() noexcept {
    x1_ = x2_ = y1_ = y2_ = 0.0;
    currentCoeffs_ = targetCoeffs_;
    currentGainLinear_ = targetGainLinear_;
}

void PickupTap::setParams(const PickupTapParams& p) noexcept {
    targetCoeffs_ = computeCoeffs(p.resonanceHz, p.q, sampleRate_);
    targetGainLinear_ = dbToLinear(p.outputGainDb);
}

void PickupTap::runBlock(const Sample* in, Sample* out, int numSamples) noexcept {
    const bool coeffsSettled = currentCoeffs_.b0 == targetCoeffs_.b0 && currentCoeffs_.b1 == targetCoeffs_.b1 &&
                               currentCoeffs_.b2 == targetCoeffs_.b2 && currentCoeffs_.a1 == targetCoeffs_.a1 &&
                               currentCoeffs_.a2 == targetCoeffs_.a2;
    const bool gainSettled = currentGainLinear_ == targetGainLinear_;

    if (coeffsSettled && gainSettled) {
        for (int n = 0; n < numSamples; ++n) {
            const double y = biquadTick(static_cast<double>(in[n]), currentCoeffs_);
            out[n] = static_cast<Sample>(y * static_cast<double>(currentGainLinear_));
        }
        return;
    }

    // Linear ramp from the settled coefficients/gain to the newly targeted ones across this
    // block's samples (the OutputGain convention), landing exactly on target at the last sample
    // regardless of numSamples -- see OutputGain::process for the same identity spelled out.
    const double invN = 1.0 / static_cast<double>(numSamples);
    for (int n = 0; n < numSamples; ++n) {
        const bool isLastSample = (n == numSamples - 1);
        Coeffs c;
        float gain;
        if (isLastSample) {
            c = targetCoeffs_;
            gain = targetGainLinear_;
        } else {
            const double t = static_cast<double>(n + 1) * invN;
            c.b0 = currentCoeffs_.b0 + (targetCoeffs_.b0 - currentCoeffs_.b0) * t;
            c.b1 = currentCoeffs_.b1 + (targetCoeffs_.b1 - currentCoeffs_.b1) * t;
            c.b2 = currentCoeffs_.b2 + (targetCoeffs_.b2 - currentCoeffs_.b2) * t;
            c.a1 = currentCoeffs_.a1 + (targetCoeffs_.a1 - currentCoeffs_.a1) * t;
            c.a2 = currentCoeffs_.a2 + (targetCoeffs_.a2 - currentCoeffs_.a2) * t;
            gain = static_cast<float>(
                static_cast<double>(currentGainLinear_) +
                (static_cast<double>(targetGainLinear_) - static_cast<double>(currentGainLinear_)) * t);
        }
        const double y = biquadTick(static_cast<double>(in[n]), c);
        out[n] = static_cast<Sample>(y * static_cast<double>(gain));
    }

    currentCoeffs_ = targetCoeffs_;
    currentGainLinear_ = targetGainLinear_;
}

void PickupTap::process(const StringTapBuffers<Sample>& taps, Sample* out, int numSamples) noexcept {
    // Bounded by taps.numSamples() as well as maxBlockSize_: taps' own fill can be shorter than
    // either (a short last block, or a network prepared with a smaller maxBlockSize than this
    // PickupTap's own), and each string's channel() is only valid for taps.numSamples() samples --
    // reading further walks into the next string's stride, or past StringNetwork's whole tap
    // storage once every string's slot is exhausted. Samples beyond count are left untouched in
    // `out`, matching the rest of this module's [0, count) convention.
    const int count = std::clamp(numSamples, 0, std::min(maxBlockSize_, taps.numSamples()));
    if (count <= 0)
        return;

    const int numStrings = taps.numStrings();
    std::array<const Sample*, kMaxStrings> channels{};
    int activeCount = 0;
    for (int s = 0; s < numStrings && s < kMaxStrings; ++s) {
        if (!taps.isActive(s))
            continue;
        const Sample* channel = taps.channel(s);
        if (channel != nullptr)
            channels[static_cast<std::size_t>(activeCount++)] = channel;
    }

    for (int n = 0; n < count; ++n) {
        Sample sum = Sample(0);
        for (int i = 0; i < activeCount; ++i)
            sum += channels[static_cast<std::size_t>(i)][n];
        scratchMono_[static_cast<std::size_t>(n)] = sum;
    }

    runBlock(scratchMono_.data(), out, count);
}

void PickupTap::processMono(const Sample* in, Sample* out, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);
    if (count <= 0)
        return;
    runBlock(in, out, count);
}

} // namespace cnpg::dsp

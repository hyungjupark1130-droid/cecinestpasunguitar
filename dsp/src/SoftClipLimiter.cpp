#include "cnpg/dsp/SoftClipLimiter.h"

#include <algorithm>
#include <cmath>

namespace cnpg::dsp {

namespace {

float dbToLinear(float gainDb) noexcept { return std::pow(10.0f, gainDb / 20.0f); }

} // namespace

void SoftClipLimiter::prepare(double sampleRate, int maxBlockSize) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(0, maxBlockSize);

    // Seed BOTH the settled and the target ceiling from the documented default, so the first
    // process() call after prepare() enforces that ceiling immediately instead of ramping onto it
    // from an unrelated starting point (the PickupTap/TriodeStage prepare() convention).
    const SoftClipLimiterParams defaults{};
    targetCeilingLinear_ = dbToLinear(std::clamp(defaults.ceilingDb, kMinCeilingDb, kMaxCeilingDb));
    currentCeilingLinear_ = targetCeilingLinear_;
}

void SoftClipLimiter::reset() noexcept { currentCeilingLinear_ = targetCeilingLinear_; }

void SoftClipLimiter::setParams(const SoftClipLimiterParams& p) noexcept {
    if (!std::isfinite(p.ceilingDb))
        return; // keep the current target rather than manufacturing a non-finite ceiling

    targetCeilingLinear_ = dbToLinear(std::clamp(p.ceilingDb, kMinCeilingDb, kMaxCeilingDb));
}

// The fixed soft-knee shape (see SoftClipLimiter.h for the algebra and for why each branch is
// written the way it is): identity below the knee, tanh-asymptotic to the ceiling above it.
float SoftClipLimiter::clipOne(Sample x, float ceilingLinear) noexcept {
    // The chain's last line of defence: a non-finite sample becomes silence rather than reaching
    // the host. Checked first, before any arithmetic that would propagate it.
    if (!std::isfinite(x))
        return 0.0f;

    const double ceiling = static_cast<double>(ceilingLinear);
    const double magnitude = std::fabs(static_cast<double>(x));
    const double kneeMagnitude = kKneeStart * ceiling;

    // Below the knee the stage is the IDENTITY, returned bit-exactly -- not multiplied by a unity
    // gain, not round-tripped through a division by the ceiling. See the header's property 1.
    if (magnitude <= kneeMagnitude)
        return x;

    const double u = magnitude / ceiling; // input in units of the ceiling; > kKneeStart here
    const double excess = (u - kKneeStart) / (1.0 - kKneeStart);
    const double shaped = kKneeStart + (1.0 - kKneeStart) * std::tanh(excess);

    // shaped < 1 strictly (tanh < 1), so this product is strictly below the ceiling before
    // rounding, and rounding to float can land on the ceiling but never past it.
    const double limited = ceiling * shaped;
    return static_cast<float>(x < 0.0f ? -limited : limited);
}

void SoftClipLimiter::process(const Sample* in, Sample* out, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);
    if (count <= 0)
        return;

    if (currentCeilingLinear_ == targetCeilingLinear_) {
        for (int n = 0; n < count; ++n)
            out[n] = clipOne(in[n], currentCeilingLinear_);
        return;
    }

    // Per-block linear ramp of the ceiling itself, the OutputGain convention: computed in double so
    // the interpolation fraction (i+1)/N is exact at the last sample, which is assigned the target
    // directly regardless -- guaranteeing bit-exact settling and guaranteeing every intermediate
    // ceiling is a convex combination of the two endpoints (so the block's bound is
    // max(old, new), exactly as the header documents, and never something outside that pair).
    const double startCeiling = static_cast<double>(currentCeilingLinear_);
    const double endCeiling = static_cast<double>(targetCeilingLinear_);
    const double span = endCeiling - startCeiling;
    const double invN = 1.0 / static_cast<double>(count);

    for (int n = 0; n < count; ++n) {
        const bool isLastSample = (n == count - 1);
        const float ceiling = isLastSample
                                  ? targetCeilingLinear_
                                  : static_cast<float>(startCeiling + span * (static_cast<double>(n + 1) * invN));
        out[n] = clipOne(in[n], ceiling);
    }

    currentCeilingLinear_ = targetCeilingLinear_;
}

} // namespace cnpg::dsp

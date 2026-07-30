#include "cnpg/dsp/OutputGain.h"

#include <cmath>

namespace cnpg::dsp {

namespace {

float dbToLinear(float gainDb) noexcept { return std::pow(10.0f, gainDb / 20.0f); }

} // namespace

void OutputGain::prepare(double /*sampleRate*/, int /*maxBlockSize*/) {
    // No buffers to size: this module's only state is the pair of scalar gains below.
    currentGainLinear_ = 1.0f;
    targetGainLinear_ = 1.0f;
}

void OutputGain::reset() noexcept { currentGainLinear_ = targetGainLinear_; }

void OutputGain::setParams(const OutputGainParams& params) noexcept { targetGainLinear_ = dbToLinear(params.gainDb); }

void OutputGain::process(const Sample* in, Sample* out, int numSamples) noexcept {
    if (currentGainLinear_ == targetGainLinear_) {
        for (int i = 0; i < numSamples; ++i)
            out[i] = in[i] * currentGainLinear_;
        return;
    }

    // Linear ramp from currentGainLinear_ to targetGainLinear_ across this block's
    // samples, computed in double so the per-sample interpolation fraction (i+1)/N is
    // exact at i == N-1 (t == 1.0). The last sample is assigned targetGainLinear_
    // directly regardless, which both guarantees bit-exact settling (no reliance on
    // floating-point round-trip through double) and guarantees the ramp never exceeds
    // the target: every earlier sample's gain is a convex combination of current and
    // target (t in [0, 1)), so it lies strictly between them.
    const double startGain = static_cast<double>(currentGainLinear_);
    const double endGain = static_cast<double>(targetGainLinear_);
    const double span = endGain - startGain;
    const double invN = 1.0 / static_cast<double>(numSamples);

    for (int i = 0; i < numSamples; ++i) {
        const bool isLastSample = (i == numSamples - 1);
        const float gain = isLastSample ? targetGainLinear_
                                        : static_cast<float>(startGain + span * (static_cast<double>(i + 1) * invN));
        out[i] = in[i] * gain;
    }

    currentGainLinear_ = targetGainLinear_;
}

} // namespace cnpg::dsp

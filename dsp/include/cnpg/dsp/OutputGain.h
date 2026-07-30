#pragma once

#include "cnpg/dsp/Common.h"

// OutputGain -- see docs/plan.md section 2.11 ("Monitoring-chain helpers: CabFilter,
// SoftClipLimiter, OutputGain"). Trivial block-domain gain stage; gain changes ramp
// per-block. Zero JUCE includes.

namespace cnpg::dsp {

struct OutputGainParams {
    float gainDb = 0.0f;
};

// A per-block-smoothed output gain. setParams() retargets the gain in decibels; the next
// process() call ramps the internal linear gain linearly from its last settled value to
// the new target across that call's samples, landing exactly on the target at the last
// sample -- so the ramp always completes within a single block (never spilling into a
// later block) regardless of numSamples, and linear interpolation between two endpoints
// can never overshoot either one. If no new target was set since the last process()
// call, the gain is already settled and process() is a plain per-sample multiply (exact
// for unity gain, since multiplying an IEEE-754 float by 1.0f is bit-exact).
class OutputGain {
  public:
    // Message thread; may allocate. Nothing to allocate here (no buffers, only two
    // scalar gains), but every module keeps this signature per the unified contract.
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Collapses any pending ramp: the currently-configured target
    // (set via setParams(), or unity if setParams() was never called) becomes the
    // settled value immediately, so the very next process() call starts already at
    // that target rather than ramping up/down to it. Does not alter the target itself.
    void reset() noexcept;

    // Realtime-safe. Only retargets the ramp; does not touch currentGainLinear_.
    void setParams(const OutputGainParams& params) noexcept;

    // Realtime-safe; never allocates or locks. Valid for numSamples in [1, maxBlockSize].
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

  private:
    float currentGainLinear_ = 1.0f; // last settled linear gain
    float targetGainLinear_ = 1.0f;  // linear gain requested by the most recent setParams()
};

} // namespace cnpg::dsp

#include "cnpg/dsp/CabFilter.h"

#include <algorithm>
#include <cmath>

namespace cnpg::dsp {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

} // namespace

void CabFilter::prepare(double sampleRate, int maxBlockSize) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(0, maxBlockSize);

    // See CabFilter.h: the fixed 5 kHz design cutoff is clamped only if the host rate is so low
    // that 5 kHz would sit at or above Nyquist. Idle at 44.1 kHz and up.
    effectiveCutoffHz_ = std::min(kDesignCutoffHz, kMaxCutoffFraction * sampleRate_);

    // RBJ cookbook 2nd-order lowpass, bilinear-transformed with the standard prewarping, so the
    // digital response at effectiveCutoffHz_ equals the analog prototype's at its own w0 -- which
    // at kDesignQ is exactly -3.01 dB, at every sample rate (see the header).
    const double w0 = kTwoPi * effectiveCutoffHz_ / sampleRate_;
    const double cosW0 = std::cos(w0);
    const double alpha = std::sin(w0) / (2.0 * kDesignQ);

    const double a0 = 1.0 + alpha;
    b0_ = ((1.0 - cosW0) * 0.5) / a0;
    b1_ = (1.0 - cosW0) / a0;
    b2_ = b0_;
    a1_ = (-2.0 * cosW0) / a0;
    a2_ = (1.0 - alpha) / a0;

    reset();
}

void CabFilter::reset() noexcept {
    x1_ = 0.0;
    x2_ = 0.0;
    y1_ = 0.0;
    y2_ = 0.0;
}

void CabFilter::setParams(const CabFilterParams& p) noexcept { bypass_ = p.bypass; }

void CabFilter::process(const Sample* in, Sample* out, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);

    for (int n = 0; n < count; ++n) {
        // Read the input BEFORE writing the output so in == out (in-place) is safe.
        const Sample sample = in[n];

        // The biquad runs whether or not bypass is engaged -- see the header's "Bypass" section
        // for why the cycles buy state continuity across the switch. A non-finite input sample is
        // substituted with silence before it reaches the recursion: without this one NaN would
        // poison y1_/y2_ permanently and every later sample with it, which for a stage sitting
        // downstream of an oversampled nonlinearity is a failure mode worth costing one compare.
        const double x = std::isfinite(sample) ? static_cast<double>(sample) : 0.0;
        const double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

        x2_ = x1_;
        x1_ = x;
        y2_ = y1_;
        y1_ = y;

        out[n] = bypass_ ? sample : static_cast<Sample>(y);
    }
}

} // namespace cnpg::dsp

#pragma once

#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"

// PickupTap -- see docs/plan.md section 2.8. Task P1.1 landed PickupTapParams only (already its
// final shape per the draft). This task (P1.6) adds the PickupTap class: a block-domain consumer
// of StringNetwork's per-string tap buffers (StringTapBuffers, fully declared in
// StringNetwork.h -- only forward-declared below, so this header stays light) that sums the
// active strings' fractional-position taps into mono, applies one linear RLC resonance biquad,
// then trims with outputGainDb toward the -18 dBFS per-string nominal structure. Magnetic
// nonlinearity is deliberately absent -- it is P4's. Block domain, NOT templated: it consumes
// only the float/Sample realtime tap instantiation. Zero JUCE includes.
//
// RLC model. The coil's inductance (L), its parasitic capacitance (C), and the pot/cable/amp
// loading (R) form a resonant tank; resonanceHz and q place a complex pole pair for that tank
// (w0 = 2*pi*resonanceHz/sampleRate; pole radius set from q), realized as an RBJ-style constant
// 0 dB peak-gain bandpass biquad. A resonant LOWPASS -- the topology "voltage across the load"
// suggests at first glance -- is deliberately NOT used: that damping convention only produces a
// magnitude peak once q exceeds 1/sqrt(2) =~ 0.707, so the acceptance sweep's q = 0.7 case (a
// heavily loaded pickup, just under that boundary) would have no resonance left to measure. The
// bandpass realization peaks at exactly resonanceHz for every q > 0 -- what the q in {0.7, 2, 6}
// acceptance sweep needs -- while remaining a legitimate driven-RLC-tank reading of the same two
// physical parameters.
//
// Smoothing. Coefficients and the post-filter output-gain trim are both smoothed with a
// per-block linear ramp, the convention OutputGain uses for its own gain: setParams() only
// retargets; the next process()/processMono() call ramps the last SETTLED coefficients/gain to
// the new targets across that call's samples, landing exactly on target at the last sample, so a
// ramp always completes within the block it starts in (never spilling into a later one).

namespace cnpg::dsp {

template <typename SampleT> struct StringTapBuffers; // full definition: StringNetwork.h

struct PickupTapParams {
    float resonanceHz = 2500.0f; // RLC resonant frequency
    float q = 2.0f;              // resonance Q (loading)
    float outputGainDb = 0.0f;   // post-sum trim toward the -18 dBFS per-string nominal structure
};

static_assert(std::is_trivially_copyable_v<PickupTapParams>,
              "PickupTapParams must stay trivially copyable for the realtime APVTS snapshot path.");

class PickupTap {
  public:
    // Message thread; may allocate. Sizes the internal mono scratch buffer for maxBlockSize and
    // seeds both the settled and target coefficients/gain from PickupTapParams{}'s own defaults,
    // so a process() call before any setParams() runs a sane, already-settled filter rather than
    // a degenerate one. Calls reset().
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Clears the biquad's own memory (as if no signal had ever passed through it)
    // and collapses any pending coefficient/gain ramp onto its target immediately, so a reset
    // instance is indistinguishable from a freshly prepared one carrying the same parameters.
    void reset() noexcept;

    // Realtime-safe; only retargets the ramp. resonanceHz and q are clamped into a safe range
    // (away from 0 Hz and from Nyquist; q away from 0) before being placed as a pole pair, so a
    // malformed or extreme parameter value can move the response but never yields NaN or an
    // unstable filter.
    void setParams(const PickupTapParams& p) noexcept;

    // Realtime-safe; never allocates. Sums taps.channel(i) for every string reporting
    // taps.isActive(i) into an internal mono scratch buffer, then runs that sum through the same
    // stage processMono() runs. Clamped to [0, maxBlockSize].
    void process(const StringTapBuffers<Sample>& taps, Sample* out, int numSamples) noexcept;

    // Test-only seam. docs/plan.md section 2.8 locks process(taps, out, numSamples) as the real
    // consumer entry point, but a StringTapBuffers can only be produced by a real StringNetwork
    // driving actual waveguide-string physics -- there is no way to hand it a synthetic sine for
    // the [contract] frequency-response measurement. processMono() runs the identical RLC-biquad
    // + output-gain stage process() uses, directly over an already-mono buffer, so that
    // measurement (and the parameter-step / no-NaN check) can drive a controlled signal through
    // the exact code process() itself runs. Realtime-safe; never allocates.
    void processMono(const Sample* in, Sample* out, int numSamples) noexcept;

  private:
    struct Coeffs {
        double b0 = 0.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
    };

    static Coeffs computeCoeffs(float resonanceHz, float q, double sampleRate) noexcept;
    double biquadTick(double x, const Coeffs& c) noexcept;
    void runBlock(const Sample* in, Sample* out, int numSamples) noexcept;

    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;

    Coeffs currentCoeffs_{}; // last settled coefficients
    Coeffs targetCoeffs_{};  // coefficients requested by the most recent setParams()
    float currentGainLinear_ = 1.0f;
    float targetGainLinear_ = 1.0f;

    // Direct-Form-I biquad memory, kept in double for headroom at high q.
    double x1_ = 0.0;
    double x2_ = 0.0;
    double y1_ = 0.0;
    double y2_ = 0.0;

    std::vector<Sample> scratchMono_; // sized maxBlockSize_ in prepare(); process()'s active sum
};

} // namespace cnpg::dsp

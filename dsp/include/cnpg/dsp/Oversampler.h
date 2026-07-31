#pragma once

#include <vector>

#include "cnpg/dsp/Common.h"

// Oversampler -- see docs/plan.md section 2.10 and Task P1.8. A reusable fixed-factor oversampling
// wrapper around nonlinear stages only (in P1: TriodeStage). Zero JUCE includes.
//
// -----------------------------------------------------------------------------------------------
// Why the factor is restricted to {2, 4, 8}.
// -----------------------------------------------------------------------------------------------
//
// Every rate change here is a 2x polyphase IIR halfband -- the cheapest structure that gets a
// >90 dB stopband out of ~5 one-multiply sections per branch. A halfband only ever doubles or
// halves, so the reachable factors are exactly the powers of two, and `kMaxOversampling` (= 8,
// Common.h) caps the cascade at three stages. `prepare()` therefore snaps any requested factor
// DOWN to the largest valid power of two in [2, 8] (1 -> 2, 3 -> 2, 5/6/7 -> 4, >= 9 -> 8) rather
// than rejecting it: a host or a future preset handing in a nonsense value must still leave the
// plugin in a working, documented state. The factor is fixed for the life of one prepare().
//
// -----------------------------------------------------------------------------------------------
// The halfband filters (elliptic, polyphase, one multiply per section).
// -----------------------------------------------------------------------------------------------
//
// Each stage is the classic two-branch polyphase decomposition of an odd-order elliptic halfband:
//
//   H(z) = 0.5 * ( A0(z^2) + z^-1 * A1(z^2) ),   A_b(z^2) = product of (a_i + z^-2)/(1 + a_i z^-2)
//
// with the designed coefficients sorted ascending and dealt alternately into A0 (even indices) and
// A1 (odd indices) -- the assignment that makes both branches carry the same DC group delay, which
// is what makes the two branches sum to a lowpass rather than cancel. Because the sections are
// first-order in z^-2, each one runs at the LOW rate inside the polyphase structure: upsampling by
// two costs one pass of A0 and one of A1 per INPUT sample (producing the even and odd output
// samples respectively), and downsampling by two costs one pass of each per OUTPUT sample. There is
// no zero-stuffing and no discarded work anywhere in either leg.
//
// `designHalfbandCoefficients()` computes the coefficients from the elliptic-filter theta-function
// series (Jacobi nome `q` from the selectivity `k`, then sqrt(k)*sn evaluated as a ratio of theta
// series) -- i.e. nothing in this file is a pasted magic constant; the whole filter is derived
// in-repo from its two design parameters and pinned by `OversamplerTests.cpp`. The design
// parameters per stage are:
//
//   stage 0 (base rate <-> 2x): 10 coefficients, transition bandwidth 0.0125
//   stage 1 (2x <-> 4x)       :  4 coefficients, transition bandwidth 0.10
//   stage 2 (4x <-> 8x)       :  4 coefficients, transition bandwidth 0.10
//
// A `transitionBandwidth` of t places the passband edge at (0.25 - t/2) and the stopband edge at
// (0.25 + t/2) of the stage's own (upsampled) rate. Only stage 0 has to be sharp: it is the one
// whose stopband edge decides how much of the spectrum immediately above the BASE Nyquist gets
// folded back into the audio band. At a 48 kHz host rate stage 0 runs at 96 kHz, so t = 0.0125
// puts its passband edge at 23.4 kHz and its stopband edge at 24.6 kHz with >= 91 dB of stopband
// rejection. Stages 1 and 2 sit an octave and two octaves higher; the only content that can fold
// from them into the final 0..24 kHz band lies above 0.375 of their own rate, which is deep inside
// even a lazy t = 0.10 stopband -- so they are deliberately cheap (4 coefficients, ~70 dB), because
// spending sections there would buy nothing measurable at the output.
//
// Residual, and deliberately so: content between the stage-0 passband and stopband edges (23.4 to
// 24.6 kHz at a 48 kHz host) is neither passed nor rejected, so nonlinear products landing in that
// window fold back into 23.4..24 kHz at up to ~-6 dB. That is a property of every halfband
// oversampler ever shipped -- the transition band has to go somewhere, and above 23 kHz is the only
// place it can go at a 48 kHz host rate. docs/decisions/0003-adaa-vs-oversampling.md records the
// measured consequence.
//
// -----------------------------------------------------------------------------------------------
// Latency (`latencySamples()`), and what P1.9 reports to the host.
// -----------------------------------------------------------------------------------------------
//
// These filters are IIR, so there is no single exact integer delay to report -- only a group delay
// that varies across frequency (flat across the passband, rising into a peak at the transition
// band, exactly as an elliptic filter's does). `latencySamples()` reports the PASSBAND (DC) group
// delay of the complete up -> down round trip, expressed at the base rate and rounded to the
// nearest integer, which is the only number a host's integer `setLatencySamples` can carry and the
// one that actually aligns low-frequency material.
//
// It is computed in closed form, not measured: one first-order allpass section (a + z^-1)/(1 + a
// z^-1) has group delay (1-a)/(1+a) at DC, so a section in z^-2 has 2(1-a)/(1+a); a whole branch is
// the sum over its sections; and one up+down round trip through a stage whose halfband has DC group
// delay D contributes exactly (D - 0.5) samples at that stage's INPUT rate (the up leg adds D at
// the doubled rate, the down leg adds D-1 at the doubled rate because its polyphase form reads the
// filtered stream at odd full-rate indices). Cascaded, stage s contributes (D_s - 0.5) / 2^s at the
// base rate. For the shipped specs that is 2.98 samples at 2x, 3.88 at 4x and 4.32 at 8x, reported
// as 3, 4 and 4.
//
// `OversamplerTests.cpp` holds this honest by measuring the actual delay of a band-limited pulse
// through `processWrapped` with an identity nonlinearity and asserting it rounds to exactly this
// number. The pulse is band-limited on purpose: a bare unit impulse excites the transition-band
// group-delay peak as hard as the passband, so its response peak sits a sample later than anything
// a listener or a host would call "the latency".
//
// -----------------------------------------------------------------------------------------------
// Buffers and realtime discipline.
// -----------------------------------------------------------------------------------------------
//
// `prepare()` preallocates every buffer at `maxBlockSize * factor` samples (three of them: the
// `processWrapped` work buffer plus two cascade ping-pong scratches). `processWrapped`,
// `upsample`, `downsample` and `reset` allocate nothing, lock nothing and are safe on the audio
// thread; the wrapped callback must be too. Filter state and coefficients are `double` while the
// buffers stay `Sample` (float): the state is where a near-unit-circle allpass recursion would
// accumulate error, and it is only ~10 scalars per branch, so the accuracy is free.
//
// Wiring note for P1.9: the wrapped nonlinearity runs at `factor * sampleRate` on blocks of up to
// `maxBlockSize * factor` samples, so it must be prepared for THAT rate and block size --
// `triode.prepare(sampleRate * factor, maxBlockSize * factor)` -- not for the host's.

namespace cnpg::dsp {

class Oversampler {
  public:
    // Shipped default factor (docs/plan.md line 21: "Oversampling factors: {2, 4, 8} only;
    // default 2x").
    static constexpr int kDefaultFactor = 2;

    // Longest cascade `kMaxOversampling` allows: 2 -> 1 stage, 4 -> 2, 8 -> 3.
    static constexpr int kMaxStages = 3;

    // Design parameters of one 2x halfband stage. See the file-level comment for what
    // `transitionBandwidth` means and why stage 0 is the only sharp one.
    struct HalfbandStageSpec {
        int numCoefficients;
        double transitionBandwidth;
    };

    // Message thread; may allocate. `factor` is snapped down to the largest valid power of two in
    // [2, kMaxOversampling] (see the file-level comment) and fixed until the next prepare(). Calls
    // reset() before returning, so a process call before any other setup runs settled state.
    void prepare(double sampleRate, int maxBlockSize, int factor = kDefaultFactor);

    // Realtime-safe. Zeroes every halfband's filter memory in both legs of every stage. Does not
    // touch the factor, the buffers' sizes, or the reported latency.
    void reset() noexcept;

    // The factor actually in force (post-snap), 2 until the first prepare().
    int factor() const noexcept { return factor_; }

    // Passband (DC) group delay of the whole up -> down round trip, at the base rate, rounded to
    // the nearest integer. See the file-level comment. Reported to the host by the plugin layer.
    int latencySamples() const noexcept { return latencySamples_; }

    // Capacity, in oversampled samples, of the buffers prepare() allocated -- i.e.
    // maxBlockSize * factor. The split API's caller must provide a buffer at least this large.
    int oversampledCapacity() const noexcept { return oversampledCapacity_; }

    // Wraps one nonlinear stage: upsample -> fn(buffer, numUpsampled) -> downsample, over the
    // internally preallocated work buffer. `fn` must be alloc/lock-free and is called exactly once
    // per process call with the whole oversampled block. `numSamples` is clamped to the prepared
    // maxBlockSize like every other module in this repo; in-place use (in == out) is fine.
    template <typename NonlinearFn>
    void processWrapped(const Sample* in, Sample* out, int numSamples, NonlinearFn&& fn) noexcept {
        const int count = clampBlock(numSamples);
        if (count <= 0)
            return;
        const int numUpsampled = upsample(in, count, workBuffer_.data());
        fn(workBuffer_.data(), numUpsampled);
        downsample(workBuffer_.data(), numUpsampled, out);
    }

    // Split API for chains that need the two legs separately (docs/plan.md open question 6: the P4
    // pickup-nonlinearity island). `upBuffer` must hold at least oversampledCapacity() samples and
    // must not alias `in`. Returns the number of oversampled samples written.
    int upsample(const Sample* in, int numSamples, Sample* upBuffer) noexcept;

    // Inverse leg. `numUpsampled` must be what the matching upsample() returned (it is clamped to
    // oversampledCapacity() and rounded down to a whole number of base-rate samples regardless).
    void downsample(const Sample* upBuffer, int numUpsampled, Sample* out) noexcept;

    // Design parameters of stage `stageIndex` (0 = the base-rate <-> 2x stage). Pure function of
    // no state; safe before prepare(). Out-of-range indices return stage kMaxStages-1's spec.
    static HalfbandStageSpec stageSpec(int stageIndex) noexcept;

    // The elliptic halfband coefficient design (see the file-level comment). Returns
    // `numCoefficients` allpass coefficients in ascending order, all strictly inside (0, 1); the
    // caller deals even indices to branch A0 and odd indices to A1. Message thread; allocates.
    // Exposed so OversamplerTests.cpp can pin the design's stopband rejection and passband ripple
    // directly, rather than only observing them through a prepared instance.
    static std::vector<double> designHalfbandCoefficients(int numCoefficients, double transitionBandwidth);

  private:
    // One branch of one halfband: a chain of first-order-in-z^-2 allpass sections, run at the low
    // rate inside the polyphase structure (so the section is first-order in z^-1 here).
    struct AllpassBranch {
        std::vector<double> coefficients;
        std::vector<double> lastInput;
        std::vector<double> lastOutput;

        void configure(const std::vector<double>& coefs);
        void clear() noexcept;
        double process(double x) noexcept;
    };

    // One 2x rate-change stage: independent filter memory for the up leg and the down leg, because
    // they filter different signals.
    struct Stage {
        AllpassBranch upEven;   // A0 -> even-indexed oversampled samples
        AllpassBranch upOdd;    // A1 -> odd-indexed oversampled samples
        AllpassBranch downEven; // A1 applied to the even-indexed input samples
        AllpassBranch downOdd;  // A0 applied to the odd-indexed input samples
        double dcGroupDelay = 0.0;

        void configure(const std::vector<double>& coefs);
        void clear() noexcept;
    };

    int clampBlock(int numSamples) const noexcept;
    static void upsampleStage(Stage& stage, const Sample* in, int numSamples, Sample* out) noexcept;
    static void downsampleStage(Stage& stage, const Sample* in, int numOut, Sample* out) noexcept;

    double sampleRate_ = 44100.0; // base (host) rate; kept for lifecycle-contract completeness
    int maxBlockSize_ = 0;
    int factor_ = kDefaultFactor;
    int numStages_ = 1;
    int latencySamples_ = 0;
    int oversampledCapacity_ = 0;

    Stage stages_[kMaxStages];

    std::vector<Sample> workBuffer_; // processWrapped's oversampled block
    std::vector<Sample> scratchA_;   // cascade ping-pong
    std::vector<Sample> scratchB_;
};

} // namespace cnpg::dsp

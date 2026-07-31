#pragma once

#include <type_traits>

#include "cnpg/dsp/Common.h"

// CabFilter -- see docs/plan.md section 2.11. Task P1.1 landed CabFilterParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft: cutoff is fixed by
// design, not user-facing); Task P1.9 (this file's current state) adds the CabFilter class itself.
// Zero JUCE includes.
//
// -----------------------------------------------------------------------------------------------
// What this is, and what it deliberately is NOT.
// -----------------------------------------------------------------------------------------------
//
// A crude speaker stand-in: one fixed 2nd-order Butterworth lowpass at kDesignCutoffHz (5 kHz),
// bypassable. It exists so the P1 monitoring chain does not present the triode's raw upper
// harmonics straight to the listener -- a real guitar cabinet rolls off hard above ~5 kHz, and
// without any such roll-off the P1 instrument sounds nothing like one. It is NOT a cabinet
// simulation, NOT an IR convolution, and NOT a tone stack: docs/plan.md section 2.11 locks
// "no tone stack before P4", and the cutoff is deliberately absent from CabFilterParams so no
// preset or automation lane can move it before the real cabinet/tone work lands. The only
// user-facing control is bypass.
//
// -----------------------------------------------------------------------------------------------
// The filter, and why Butterworth specifically.
// -----------------------------------------------------------------------------------------------
//
// An RBJ-cookbook bilinear-transformed 2nd-order lowpass with Q = 1/sqrt(2) (kDesignQ), i.e. a
// Butterworth response: maximally flat passband, and -- the property the [contract] test keys on --
// its -3.01 dB point sits at exactly the design cutoff, for every sample rate. That is a property
// of Q = 1/sqrt(2) alone: |H(w0)| = 1/(2Q) at the pole frequency for this topology, which is
// 1/sqrt(2) only at that Q, and the bilinear transform's own prewarping is what carries the
// identity across sample rates unchanged (the RBJ formulas evaluate the analog prototype exactly at
// w0). Any other Q would put the -3 dB point somewhere else and make "the -3 dB point is at the
// design cutoff" an approximation to be tolerated rather than an identity to be asserted.
//
// Very low sample rates. The design cutoff is a fixed 5 kHz, so a host running below ~11.1 kHz
// would place it at or above Nyquist, where the RBJ formulas degenerate (w0 >= pi). The effective
// cutoff is therefore clamped to kMaxCutoffFraction of the sample rate; at every rate this project
// supports (44.1 kHz and up, docs/plan.md section 1.2) the clamp never engages and the effective
// cutoff is exactly kDesignCutoffHz. effectiveCutoffHz() reports what actually got placed, so the
// [contract] test measures against the real design point rather than assuming the clamp is idle.
//
// -----------------------------------------------------------------------------------------------
// Bypass.
// -----------------------------------------------------------------------------------------------
//
// bypass is an immediate, unramped switch (the TriodeStage::bypass convention): while bypassed,
// process() copies input to output bit-exactly. It does NOT stop running the biquad -- the filter
// keeps consuming the same input and updating its own memory while bypassed, and bypass only
// selects which of the two results is written out. That costs the same five multiplies per sample
// either way, and buys a property worth more than the cycles in a monitoring chain: re-engaging the
// filter resumes from state consistent with the signal that just went past, not from memory frozen
// at whatever sample was playing when bypass was switched on however many seconds ago. The
// switch's audible discontinuity is then only the genuine dry-vs-filtered difference, and never
// additionally an artifact of how long the bypass had been engaged.

namespace cnpg::dsp {

struct CabFilterParams {
    bool bypass = false; // cutoff is fixed (~5 kHz, 2nd order) by design
};

static_assert(std::is_trivially_copyable_v<CabFilterParams>,
              "CabFilterParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Bypassable fixed 2nd-order Butterworth lowpass; the P1 monitoring chain's speaker stand-in.
// Block domain, NOT templated: Sample is always float (the realtime instantiation).
class CabFilter {
  public:
    // Fixed by design (see the file-level comment); not a parameter, not automatable, not moved
    // before P4's real cabinet/tone work.
    static constexpr double kDesignCutoffHz = 5000.0;

    // Butterworth: 1/sqrt(2), the only Q at which the -3.01 dB point coincides with the pole
    // frequency. Spelled out rather than computed so the constant is greppable and cannot drift.
    static constexpr double kDesignQ = 0.7071067811865475244;

    // Cutoff ceiling as a fraction of the sample rate, so the bilinear placement stays well inside
    // the unit circle even at absurdly low rates (see the file-level comment). Idle at every rate
    // this project supports.
    static constexpr double kMaxCutoffFraction = 0.45;

    // Message thread; may allocate. Nothing to allocate here (five coefficients and four state
    // scalars), but every module keeps this signature per the unified contract. Places the biquad
    // for `sampleRate` and calls reset().
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Clears the biquad's memory, as if no signal had ever passed through it, so a
    // reset instance is indistinguishable from a freshly prepared one. Does not touch the
    // coefficients (they depend only on the sample rate) or the bypass switch.
    void reset() noexcept;

    // Realtime-safe. Updates the bypass switch immediately -- no ramp; see the file-level comment.
    void setParams(const CabFilterParams& p) noexcept;

    // Realtime-safe; never allocates. Valid for numSamples in [1, maxBlockSize]; clamps internally
    // like every other module in this repo. In-place use (in == out) is fine.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

    // The cutoff actually placed for the prepared sample rate: kDesignCutoffHz at every supported
    // rate, lower only if the kMaxCutoffFraction clamp engaged (see the file-level comment).
    double effectiveCutoffHz() const noexcept { return effectiveCutoffHz_; }

    // Latency of this stage, in samples: zero. A biquad is not delay-compensated (it is minimum
    // phase, with a frequency-dependent group delay and no bulk delay to report), and reporting a
    // nonzero integer here would misalign the host rather than help it. Present so CabFilter can be
    // wrapped as an IBlockModule (ModuleGraph.h) without the adapter inventing a number.
    static constexpr int latencySamples() noexcept { return 0; }

  private:
    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;
    double effectiveCutoffHz_ = kDesignCutoffHz;

    // Direct-Form-I biquad, coefficients and memory in double (the PickupTap convention): the
    // state is where a resonant recursion would accumulate error and it is four scalars, so the
    // accuracy is free.
    double b0_ = 1.0;
    double b1_ = 0.0;
    double b2_ = 0.0;
    double a1_ = 0.0;
    double a2_ = 0.0;

    double x1_ = 0.0;
    double x2_ = 0.0;
    double y1_ = 0.0;
    double y2_ = 0.0;

    bool bypass_ = false;
};

} // namespace cnpg::dsp

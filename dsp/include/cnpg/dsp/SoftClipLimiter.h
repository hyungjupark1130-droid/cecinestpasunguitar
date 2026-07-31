#pragma once

#include <type_traits>

#include "cnpg/dsp/Common.h"

// SoftClipLimiter -- see docs/plan.md section 2.11. Task P1.1 landed SoftClipLimiterParams only
// (the APVTS-backed P1 parameter surface, already its final shape per the draft: the soft-knee
// shape is fixed, not user-facing); Task P1.9 (this file's current state) adds the SoftClipLimiter
// class itself. Zero JUCE includes.
//
// -----------------------------------------------------------------------------------------------
// What this is: a hard bound, reached softly. Not a dynamics processor.
// -----------------------------------------------------------------------------------------------
//
// docs/plan.md section 2.11 hard-wires this stage LAST in the monitoring chain
// (`... -> OutputGain -> SoftClipLimiter`) precisely so that "rendered peak <= ceilingDb" is an
// enforceable acceptance criterion for everything upstream of it. That makes the class's real
// contract a mathematical one, not a taste one: for ANY finite input, of any magnitude, the output
// magnitude is <= the ceiling. Not "usually", not "for the +12 dB overshoot the acceptance test
// happens to feed it" -- the shape below has the ceiling as a horizontal ASYMPTOTE, so there is no
// input level, however absurd, that can push a sample past it.
//
// It is a memoryless waveshaper: no lookahead, no attack/release envelope, no gain-reduction
// state. A peak limiter with a release contour would be a musical decision about how the
// instrument's dynamics behave; this is a safety clip whose entire job is that nothing downstream
// ever sees a sample above the ceiling, and whose second job is to be inaudible until it has to be
// audible. Real dynamics processing, if it ever ships, is a different module in a different phase.
//
// -----------------------------------------------------------------------------------------------
// The fixed soft-knee shape.
// -----------------------------------------------------------------------------------------------
//
// With C = the linear ceiling and u = |x| / C (input in units of the ceiling):
//
//   u <= kKneeStart              ->  y = x                                     (identity, bit-exact)
//   u >  kKneeStart              ->  y = sign(x) * C * (k + (1-k) * tanh((u - k) / (1 - k)))
//
// where k = kKneeStart. Three properties this specific arrangement has, all load-bearing:
//
//   1. BELOW THE KNEE IT IS THE IDENTITY, BIT-EXACTLY -- `out[n] = in[n]`, not `in[n] * 1.0f` and
//      not `C * (x/C)`. A safety clip that is engaged 100% of the time at nominal levels (the
//      -18 dBFS per-string nominal sits ~24 dB below a -0.3 dBFS ceiling's knee) would be a
//      permanent, unmeasurable colouration on everything upstream. This one is provably absent
//      until the signal actually approaches the ceiling, which is also what makes
//      "no colouration below the knee" a testable claim rather than a hope.
//   2. C1-CONTINUOUS AT THE KNEE. tanh(0) = 0 puts the curve at exactly k*C where the identity
//      leaves it, and d/du[(1-k) tanh((u-k)/(1-k))] = sech^2(0) = 1 at u = k matches the identity's
//      own unit slope. So there is no slope discontinuity at the knee to radiate a harmonic edge.
//   3. THE CEILING IS AN ASYMPTOTE, NOT A CLAMP. tanh < 1 strictly, so |y| < C strictly for every
//      finite input; the equality |y| == C is only ever reachable through float rounding of a value
//      already below C, which rounds to C and never past it. No std::clamp appears anywhere in this
//      module: a clamp would produce a corner (infinite harmonic content, the exact artifact a soft
//      clip exists to avoid) and would make the bound a property of the clamp rather than of the
//      shape.
//
// kKneeStart = 0.5 puts the knee 6.02 dB below the ceiling: gentle enough that the onset is not
// itself an audible edge, tight enough that the stage stays out of the way of a chain whose nominal
// level is far below it. It is fixed by design and deliberately absent from SoftClipLimiterParams.
//
// -----------------------------------------------------------------------------------------------
// Ceiling changes, and the one-block window where the bound is the OLD ceiling.
// -----------------------------------------------------------------------------------------------
//
// ceilingDb is smoothed with the house per-block linear ramp (the OutputGain/PickupTap/TriodeStage
// convention): setParams() only retargets; the next process() call ramps the last SETTLED linear
// ceiling to the new one across that call's samples, landing exactly on target at the last sample.
// A ramp therefore always completes inside the block it starts in. During that one block the
// enforced bound is the ramp's own instantaneous ceiling, so the block's peak is bounded by
// max(oldCeiling, newCeiling), NOT by the new ceiling alone. That is inherent to smoothing a
// ceiling rather than jumping it (jumping it would zipper), it lasts exactly one block, and
// reset() collapses the ramp immediately for callers that need the new bound to hold from the very
// next sample.
//
// -----------------------------------------------------------------------------------------------
// Non-finite input: the chain's last line of defence.
// -----------------------------------------------------------------------------------------------
//
// A NaN or +/-Inf sample is replaced by silence rather than propagated. This module is the last
// stage before the host buffer (docs/plan.md section 2.11), and a non-finite sample reaching a DAW
// is the one output defect that can damage speakers and ears rather than merely sounding wrong --
// so the stage whose name is "safety" is the right place to stop it. It is a backstop, not an
// excuse: every upstream module guards its own arithmetic, and the full-chain [contract] test
// asserts finiteness at the PRE-limiter probe point as well, so this guard cannot mask an upstream
// regression by silently making the chain-output assertion pass.

namespace cnpg::dsp {

struct SoftClipLimiterParams {
    float ceilingDb = -0.3f; // safety ceiling; soft-knee shape fixed
};

static_assert(std::is_trivially_copyable_v<SoftClipLimiterParams>,
              "SoftClipLimiterParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Memoryless soft-knee safety clip, hard-wired LAST in the P1 monitoring chain. Block domain, NOT
// templated: Sample is always float (the realtime instantiation).
class SoftClipLimiter {
  public:
    // Knee onset as a fraction of the ceiling (see the file-level comment): 0.5, i.e. 6.02 dB below
    // the ceiling. Fixed by design, not a parameter.
    static constexpr double kKneeStart = 0.5;

    // Defensive clamp on ceilingDb before it is converted to a linear ceiling. The APVTS range is
    // -12..0 dB (plugin/src/Parameters.cpp); this wider window is what a malformed preset or a
    // future automation range can hand in without producing a degenerate (zero or non-finite)
    // ceiling.
    static constexpr float kMinCeilingDb = -60.0f;
    static constexpr float kMaxCeilingDb = 12.0f;

    // Message thread; may allocate. Nothing to allocate here (two scalar ceilings), but every
    // module keeps this signature per the unified contract. Seeds both the settled and the target
    // ceiling from SoftClipLimiterParams{}'s own default, so a process() call before any
    // setParams() enforces the documented default ceiling already settled, not a degenerate one.
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Collapses any pending ceiling ramp onto its target immediately, so the target
    // ceiling bounds the very next sample rather than the block after it. There is no filter memory
    // to clear -- this is a memoryless waveshaper.
    void reset() noexcept;

    // Realtime-safe; only retargets the ramp. ceilingDb is clamped into [kMinCeilingDb,
    // kMaxCeilingDb] and a non-finite value is rejected (the current target is kept), so no
    // parameter value can produce a non-finite or zero ceiling.
    void setParams(const SoftClipLimiterParams& p) noexcept;

    // Realtime-safe; never allocates. Valid for numSamples in [1, maxBlockSize]; clamps internally
    // like every other module in this repo. In-place use (in == out) is fine.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

    // The settled linear ceiling currently enforced (i.e. after any pending ramp has completed).
    // Exposed so a test can assert the bound it is measuring against without re-deriving the dB
    // conversion.
    float ceilingLinear() const noexcept { return targetCeilingLinear_; }

    // Latency of this stage, in samples: zero. A memoryless waveshaper has none -- there is no
    // lookahead here by design (see the file-level comment). Present so SoftClipLimiter can be
    // wrapped as an IBlockModule (ModuleGraph.h) without the adapter inventing a number.
    static constexpr int latencySamples() noexcept { return 0; }

  private:
    static float clipOne(Sample x, float ceilingLinear) noexcept;

    double sampleRate_ = 44100.0; // kept for lifecycle-contract completeness; a memoryless
                                  // waveshaper has no sample-rate dependence
    int maxBlockSize_ = 0;

    float currentCeilingLinear_ = 1.0f; // last settled linear ceiling
    float targetCeilingLinear_ = 1.0f;  // requested by the most recent setParams()
};

} // namespace cnpg::dsp

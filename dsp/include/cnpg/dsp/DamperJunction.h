#pragma once

#include <type_traits>

// DamperJunction -- see docs/plan.md section 2.5. A strictly LINEAR, MEMORYLESS two-port
// scattering junction inserted at position p on a WaveguideString, through that class's
// readJunctionInputs / writeJunctionOutputs seam. Task P2.2 lands it; the moving-position
// crossfade machinery on the seam itself is Task P2.3, so p is static here. Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// THE PHYSICS (derived, not fitted -- this is what makes passivity structural)
// ---------------------------------------------------------------------------------------------
// A damper is a lumped mechanical resistance R (a dashpot to ground) touching the string at p.
// Cut the string there: the nut-side section delivers the arriving wave `fromNut`, the
// bridge-side section delivers `fromBridge`, and the junction point is massless, so
//
//     (velocity continuity)  v_J = fromNut + toNut = fromBridge + toBridge
//     (force balance)        Z0 (fromNut - toNut) - Z0 (toBridge - fromBridge) - R v_J = 0
//
// with Z0 the string's characteristic impedance (normalized to 1 here, matching
// WaveguideString::portImpedance()). Solving those two:
//
//     v_J      = 2 (fromNut + fromBridge) / (2 + R)
//     toBridge = v_J - fromBridge
//     toNut    = v_J - fromNut
//
// which this class evaluates in the algebraically identical but numerically nicer form
//
//     g        = R / (R + 2)                       (the junction's loss coefficient, 0 <= g < 1)
//     common   = g * (fromNut + fromBridge)
//     toBridge = fromNut  - common
//     toNut    = fromBridge - common
//
// Two things fall out of that form, and both are load-bearing:
//
//   1. R == 0 gives g == 0 EXACTLY (0/2 in IEEE-754 is exactly +0), so `common` is a signed zero
//      and the junction is a BIT-EXACT pass-through: toBridge == fromNut, toNut == fromBridge.
//      WaveguideString::writeJunctionOutputs deposits only the DIFFERENCE between the junction's
//      outputs and the waves it just read, so a bit-exact pass-through deposits exactly 0.0 into
//      both rails and the permanently in-line seam costs the ringing string nothing at all --
//      not "a tolerance", nothing. That is why "CONTRACT: StringNetwork renders the isolated
//      string bit-exactly" (tests/dsp/StringNetworkTests.cpp) still holds after this task, and it
//      is why the seam has no group delay for P2.7's calibration table to absorb.
//
//   2. Passivity is a property of the CONSTRUCTION, never of a clamp. In port order -- port 1 the
//      nut side (incident fromNut, outgoing toNut), port 2 the bridge side (incident fromBridge,
//      outgoing toBridge) -- the scattering matrix is
//
//          S = [ -g     1-g ]        (toNut, toBridge)^T = S (fromNut, fromBridge)^T
//              [ 1-g   -g   ]
//
//      S is real symmetric, so its spectral norm IS its largest |eigenvalue|, and its
//      eigenvectors are fixed: (1, 1)/sqrt(2) with eigenvalue 1 - 2g, and (1, -1)/sqrt(2) with
//      eigenvalue -1. Hence
//
//          ||S||_2 = max(|1 - 2g|, 1) = 1   for every g in [0, 1]
//
//      and g in [0, 1) follows from R >= 0 alone -- a POSITIVE junction conductance. Nothing in
//      the audio path clamps a scattering coefficient; the only clamps are on the parameters at
//      set time, and even a parameter clamp is not what carries the bound. Feed this class any
//      non-negative R and it is passive.
//
//      The unit eigenvalue is not slack in the bound, it is the physics: the (1, -1) mode is the
//      wave pair that produces ZERO displacement at p, so it does not move the dashpot and the
//      dashpot cannot dissipate it. That is exactly the node-suppression behaviour the P2.2
//      contract test measures -- a damper at p = 1/2 annihilates the fundamental (antinode) and
//      leaves the 2nd harmonic (node) untouched.
//
// ---------------------------------------------------------------------------------------------
// WHAT maxLoss MEANS (why R is mapped onto [0, 2 Z0] and not onto [0, infinity))
// ---------------------------------------------------------------------------------------------
// The dissipated fraction of the symmetric (displacement-carrying) mode is 1 - (1 - 2g)^2, which
// rises monotonically from 0 at R = 0 to exactly 1 at R = 2 Z0 -- the MATCHED resistance, where
// the junction absorbs the whole symmetric mode and reflects none of it -- and then FALLS again
// for larger R, because R -> infinity is a rigid pin: a fret, not a felt, reflecting everything
// with inversion and dissipating nothing. So the monotone-loss branch is R in [0, 2 Z0] and the
// full-depth end of it is the matched termination:
//
//     R = 2 * Z0 * maxLoss * engagement,   hence   g = s / (s + 1) with s = maxLoss * engagement
//
// giving g in [0, 0.5]: transparent at s = 0, perfectly absorbing at s = 1. `maxLoss` is
// therefore a genuine loss DEPTH (fraction of the maximum dissipation physically available at a
// point contact), not an arbitrary knob normalization.
//
// ---------------------------------------------------------------------------------------------
// STATE (all of it)
// ---------------------------------------------------------------------------------------------
// Two per-sample one-pole smoothers and nothing else: the engagement ramp (driven by engage() /
// release() with the felt time constant) and the loss depth (so a maxLoss automation move on an
// engaged damper does not zipper). Position carries no smoother in P2.2 -- the junction is
// memoryless in p, and the click-free motion machinery is a property of WaveguideString's seam,
// which Task P2.3 gives the dual-anchor crossfade.

namespace cnpg::dsp {

// Felt time-constant validation window, docs/plan.md section 2.5 ("20-100 ms engage/release
// ramp"). setParams() clamps into this range: a 0 ms felt would be a hard mute -- the exact
// discontinuity the ramp exists to prevent -- and anything past 100 ms stops reading as a damper
// coming down and starts reading as a slow fade.
inline constexpr float kFeltTimeConstantMinMs = 20.0f;
inline constexpr float kFeltTimeConstantMaxMs = 100.0f;

struct DamperJunctionParams {
    // Junction position on the string, 0 = nut, 1 = bridge. Continuously modulatable while a note
    // rings from Task P2.3; static within a block here. StringNetwork mirrors its own
    // StringNetworkParams::damperPosition01 into this field (docs/plan.md section 2.7), so on the
    // network's surface that one is the single source of truth.
    float position01 = 0.15f;

    // Loss depth when fully engaged, 0..1. 1 is the matched resistive termination (see above):
    // the deepest loss a point contact can produce, not an arbitrary maximum.
    float maxLoss = 1.0f;

    // engage()/release() ramp time constant, validated into [kFeltTimeConstantMinMs,
    // kFeltTimeConstantMaxMs]. The default is the centre of that window and is deliberately the
    // same 40 ms the P1 placeholder release envelope used, so replacing that envelope with this
    // junction (Task P2.2) is not also a change of speed.
    float feltTimeConstantMs = 40.0f;
};

static_assert(std::is_trivially_copyable_v<DamperJunctionParams>,
              "DamperJunctionParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Per docs/plan.md section 2.1 every sample-domain class is template <typename SampleT> with
// explicit float/double instantiations compiled into cnpg_dsp -- the realtime path uses float,
// the tier-2 [energy] tests run double.
template <typename SampleT> class DamperJunction {
  public:
    // Message thread. Allocates nothing (this class owns no buffers -- maxBlockSize is accepted
    // only because the module lifecycle in docs/plan.md section 2.1 is uniform), computes the
    // smoother coefficients for `sampleRate`, and calls reset().
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Engagement to 0 (target and value), loss depth SNAPPED onto its parameter
    // target, so a reset instance is indistinguishable from a freshly prepared one carrying the
    // same parameters. setLossBypassed() is a test-mode configuration and survives reset, exactly
    // as WaveguideString::setLossBypassed does.
    void reset() noexcept;

    // Realtime-safe; only retargets. position01 and maxLoss are clamped into 0..1 and
    // feltTimeConstantMs into the 20..100 ms window; a NaN in any field resolves to that field's
    // lower bound rather than propagating into the audio path.
    void setParams(const DamperJunctionParams& p) noexcept;

    // Note-off: ramp engagement -> 1 with the felt time constant.
    void engage() noexcept;

    // Ramp engagement -> 0.
    void release() noexcept;

    // Snap engagement (value AND target) to `engagement01`, clamped into 0..1. This is a
    // DISCONTINUITY in the scattering coefficients by construction, so StringNetwork calls it
    // only where the string's state is being cleared in the same breath (re-init at a new pitch,
    // reset, the enable ramp landing on silence, the release watchdog clearing a decayed tail) --
    // scattering coefficients that jump while zeros are travelling produce no step at all.
    void setEngagementImmediate(float engagement01) noexcept;

    // THE per-sample entry point. Linear passive 2-port scatter: incident (fromNut, fromBridge)
    // -> (toBridge, toNut). At engagement 0 (or with the loss bypassed) this is a bit-exact
    // pass-through: toBridge == fromNut, toNut == fromBridge.
    //
    // The smoothers are advanced AFTER the sample is computed, so the sample on which engage() is
    // consumed is still exactly transparent and the damping starts on the next one. That is what
    // makes a note-off's first sample bit-identical to the same render without it.
    void scatter(SampleT fromNut, SampleT fromBridge, SampleT& toBridge, SampleT& toNut) noexcept;

    float currentEngagement() const noexcept { return static_cast<float>(engagement_); }
    float currentPosition01() const noexcept { return position01_; }

    // The instantaneous loss depth actually in force (the smoothed maxLoss), i.e. the second half
    // of this class's entire state. Diagnostics and tests.
    float currentLossDepth() const noexcept { return static_cast<float>(lossDepth_); }

    // The felt time constant AS VALIDATED, i.e. after the 20..100 ms clamp. Exposed so a test can
    // assert the validation directly instead of inferring it from a measured ramp speed -- the
    // measured ramp is a separate, weaker observation and both are worth having.
    float currentFeltTimeConstantMs() const noexcept { return feltTimeConstantMs_; }

    // Tier-1 [energy] test hook (docs/plan.md section 4.2). Writes the instantaneous 2x2
    // scattering matrix in PORT order, row-major:
    //
    //     rowMajorS2x2 = { S_nut,nut, S_nut,bridge, S_bridge,nut, S_bridge,bridge }
    //     (toNut, toBridge)^T = S (fromNut, fromBridge)^T
    //
    // In double regardless of SampleT, since the tier-1 case computes a closed-form 2x2 SVD from
    // it. At engagement 0 S is the exact anti-diagonal pass-through {0, 1, 1, 0}.
    // Not realtime-safe by intent (it is a test hook), though it does in fact allocate nothing.
    void copyScatteringMatrix(double* rowMajorS2x2) const;

    // Energy-test hook, forwarded by StringNetwork::setLosslessTestMode (docs/plan.md section 2.7:
    // "forwards to strings/dampers/bridge"). The junction's resistive loss is its ONLY intentional
    // loss, so bypassing it makes the junction transparent regardless of engagement. The
    // engagement smoother keeps running underneath, so switching the bypass back off resumes
    // exactly where the ramp had got to. Realtime-safe.
    void setLossBypassed(bool bypass) noexcept;

  private:
    // Loss coefficient g = R / (R + 2 Z0) in force for the NEXT scatter() call. See the file
    // header: s = engagement * lossDepth, R = 2 Z0 s, hence g = s / (s + 1) in [0, 0.5].
    double lossCoefficient() const noexcept;

    // Below this distance a one-pole smoother is snapped onto its target instead of asymptoting
    // toward it forever. Two reasons, both real: a value crawling toward 0 eventually becomes a
    // denormal, and "fully engaged"/"fully released" have to be reachable states for anything to
    // be able to assert them. dg/ds = 1 / (1 + s)^2 <= 1, so the induced step in the loss
    // coefficient g is no larger than this number itself -- about one ulp of float32 around 0.5,
    // and ~120 dB under the smallest step the ramp takes on its own first sample at the slowest
    // felt time.
    static constexpr double kSmootherSettleEpsilon = 1.0e-9;

    // Per-sample smoothing time for maxLoss changes. Not the felt time constant: this one is a
    // plain parameter de-zipper for an automation move, and it matches the 8 ms convention
    // WaveguideString and StringNetwork already use for their parameter smoothers.
    static constexpr double kLossSmoothingSeconds = 0.008;

    double sampleRate_ = 44100.0;

    float position01_ = 0.15f;
    float feltTimeConstantMs_ = 40.0f; // the VALIDATED value, not the raw parameter

    double engagement_ = 0.0;       // the ramp's current value, 0..1
    double engagementTarget_ = 0.0; // 0 (released) or 1 (engaged)
    double engagementCoeff_ = 0.0;  // one-pole coefficient from feltTimeConstantMs

    double lossDepth_ = 1.0;  // smoothed maxLoss, 0..1
    double lossTarget_ = 1.0; // the parameter itself
    double lossCoeff_ = 0.0;  // one-pole coefficient from kLossSmoothingSeconds

    bool lossBypassed_ = false;
};

extern template class DamperJunction<float>;  // realtime path
extern template class DamperJunction<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"

// WaveguideString -- see docs/plan.md section 2.4. Task P1.1 landed StringMaterialParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft); Task P1.4 adds
// FractionalDelayKind, WaveguideStringParams, and the WaveguideString class itself to this same
// header. Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// TOPOLOGY (locked, docs/plan.md section 2.4)
// ---------------------------------------------------------------------------------------------
// Dual-rail bidirectional digital waveguide. Two delay rails carry the travelling-wave
// components of the string's displacement:
//
//     nut (p = 0)                                                          bridge (p = 1)
//        |----------- up rail, railSpan samples (fractional read) ------------>|
//        |                                                                     | [loop chain]
//        |<---------- dn rail, railSpan samples (fractional read) -------------|
//        ^ rigid, lossless, inverting (-1) reflection
//
// The frequency-dependent termination is CONSOLIDATED into a single chain at the bridge
// (docs/plan.md R3 mitigation: "one loss filter + allpass chain per string, not per segment"):
//
//     4 x first-order dispersion allpass  ->  one-pole-plus-zero loop loss filter
//
// The nut is a pure -1 reflection with no filter. Two inversions per round trip give a
// non-inverting loop, so the resonances sit at every integer multiple of f0 -- the correct
// modal series for a string clamped at both ends.
//
// The fractional delay is NOT a separate stage in that chain; it is the rail read itself, and
// each rail carries its own so both rail spans stay continuous reals. That placement is what
// makes retune zipper-free: a lumped fractional stage forces the rails to integer lengths, so
// every time the loop period crosses an integer during a bend the tap-position mapping steps by
// up to a full sample. With the fractional part inside the rail read, the integer part can step
// while the realized span -- and therefore every tap and injection position derived from it --
// moves continuously.
//
// ---------------------------------------------------------------------------------------------
// TUNING: PHASE delay, not group delay
// ---------------------------------------------------------------------------------------------
// The loop resonates where the round-trip phase is a multiple of 2*pi, i.e. where the total
// loop PHASE delay equals fs / f0. The task brief and docs/plan.md section 2.4 both say "group
// delay"; that is loose wording and using it literally is not merely imprecise, it misses the
// +/-2-cent gate. Worked example, dispersionAmount = 1 (allpass coefficient a = -0.2), MIDI 96
// (2093 Hz) at 44.1 kHz: each allpass has group delay 1.4599 samples but phase delay 1.4865
// samples at f0. Over the four-allpass chain that is 0.106 samples against a loop period of
// 21.07 samples -- a 8.7-cent tuning error before anything else contributes. At MIDI 108 the
// same mismatch is worth ~60 cents. Every delay quantity in this class is therefore a PHASE
// delay evaluated at the current (per-sample smoothed) f0, computed in closed form from the
// filter coefficients. See docs/decisions/0002-fractional-delay.md.
//
// The loop-length solve run whenever the smoothed f0 or a material coefficient moves is:
//
//     P            = fs / f0                                    (target loop period, samples)
//     E(w0)        = phaseDelay(dispersion) + phaseDelay(loss)   (closed form, this file's .cpp)
//     railSpan     = (P - E) / 2                                 (per rail, a continuous real)
//     base         = round(railSpan - Dmid)                      (integer part of the rail read)
//     solve D from phaseDelay(interpolator, D) = railSpan - base (fixed point, ~1e-13 samples)
//
// so the realized loop phase delay equals fs / f0 to solver precision at every note. That is
// what `setAnalyticTuningCompensation` selects; `loadCalibrationTable` only stores the P2 table
// (Task P2.7 makes it the active source).
//
// ---------------------------------------------------------------------------------------------
// ENERGY (docs/plan.md section 4.2 tier 2)
// ---------------------------------------------------------------------------------------------
// `energyEstimate()` is a discrete Lyapunov storage function: impedance-weighted rail energy
// (rail samples at delays 1..base, the ones still travelling) plus a closed-form quadratic
// storage term for every state-bearing element. Per tick each rail hands the chain exactly the
// energy it drops, so the whole string is dissipative as long as each element has a storage V
// with V(next) - V(now) <= in^2 - out^2. For the dispersion allpasses and for the Thiran-1
// fractional delay that storage is s^2 / (1 - a^2) (equality -- they are lossless). For
// Lagrange-3 -- an FIR, for which NO diagonal storage exists once ||h||_1 > 1, which is the case
// for every D strictly inside (1, 2) -- it is the quadratic form built from the filter's lossless
// orthogonal embedding, whose "states" are simply the rail samples at delays base+1..base+3; see
// WaveguideString.cpp. `energyEstimate()` is a diagnostic/test entry point, NOT a realtime path:
// it may run that closed-form factorization on first call after a coefficient change.
//
// ---------------------------------------------------------------------------------------------
// MOVING POSITIONS: the dual-anchor amplitude-complementary crossfade (Task P2.3)
// ---------------------------------------------------------------------------------------------
// Every position-mapped access this class offers -- the pickup tap and the two-port junction seam
// -- reads the rails through a CROSSFADED PAIR of anchors rather than at a position that slides
// with the parameter. The read position is therefore piecewise constant, and every step it takes is
// an amplitude-complementary linear crossfade between the position it was reading and the position
// it is going to:
//
//     anchor A = the committed position; anchor B = the position being faded in
//     g2 = the crossfade's progress in [0, 1]; g1 = 1 - g2      (so g1 + g2 = 1 by construction)
//     read = g1 * r(A) + g2 * r(B)
//
// While |requested - A| stays within kPositionAnchorThreshold01 (1/32 of the string) nothing moves
// at all: a 1/32 position error is a timbral nuance, and holding it is what keeps a slowly drifting
// parameter from resampling the rails at all. When the request departs beyond that threshold, B
// opens at the requested position, g2 ramps 0 -> 1 over kPositionCrossfadeSeconds (128 samples at
// 48 kHz, scaled with the sample rate), and B then BECOMES A. Because B opens with g2 == 0, the
// output at the first sample of a fade is exactly r(A) -- the same value it was the sample before
// -- and at the last it is exactly r(B): the read is continuous across the whole staircase. A fade
// that completes re-arms on the very next sample, so a fast sweep becomes a chain of overlapping
// amplitude-complementary segments rather than one long fade or a zipper.
//
// LINEAR, NOT EQUAL-POWER, and that is a decision rather than a simplification. Two taps a
// thirty-second of a string apart are strongly correlated: a constant-POWER law (g1^2 + g2^2 = 1)
// applied to correlated signals produces up to +3 dB mid-fade plus comb coloration -- the exact
// artifact this machinery exists to prevent. Amplitude-complementary weights sum the two reads to a
// convex combination, which can never exceed the larger of them.
//
// PASSIVITY OF THE MOVING SEAM, structurally rather than by measurement. Write r for the (linear)
// functional the crossfade reads a rail with, i.e. r = g1 r_A + g2 r_B where each r_p is the
// two-slot linear interpolation at p. railDeposit is the EXACT TRANSPOSE of railInterpolate (same
// two slots, same two weights), so writeJunctionOutputs deposits through r^T. The seam is then
// x' = x + r^T (S (r x) - r x), and with m = ||r||^2 the energy identity is
//
//     ||x'||^2 = ||x||^2 + (2 - 2m) y.u + (m - 2) ||y||^2 + m ||u||^2,   y = r x, u = S y
//
// which is <= ||x||^2 for every m <= 1 whenever ||u|| <= ||y||. And m <= 1 always: each ||r_p|| is
// sqrt((1-f)^2 + f^2) <= 1, so ||g1 r_A + g2 r_B|| <= g1 + g2 = 1 by the triangle inequality. The
// moving seam cannot create energy no matter how far apart the two anchors sit or where the fade
// has got to -- which is what the tier-2 moving-junction [energy] case measures rather than
// assumes.
//
// TRANSPARENCY SURVIVES THE MOTION. writeJunctionOutputs deposits the DIFFERENCE between the
// junction's outputs and the waves it just read through the SAME crossfaded functional, so a
// junction that returns its inputs unchanged deposits exactly 0.0 at BOTH anchors, mid-fade
// included. A transparent damper therefore still costs a ringing string nothing while its position
// is swept -- and, as the P2.3 [energy] case records, that is also exactly why a damper-position
// sweep under setLosslessTestMode(true) is bit-exactly inert: lossless mode is what makes the
// junction transparent in the first place.
//
// The exciter is deliberately NOT part of this: injectAt's position is latched at note-on and never
// modulated, so it needs no anchor and gets none.

namespace cnpg::dsp {

// The moving-position machinery's two constants, stated once here so tests reference them rather
// than restating numbers. The threshold is in string lengths; the crossfade duration is in seconds
// and is converted to a whole number of samples at prepare() time (128 samples at 48 kHz).
inline constexpr float kPositionAnchorThreshold01 = 1.0f / 32.0f;
inline constexpr double kPositionCrossfadeSeconds = 128.0 / 48000.0;

enum class FractionalDelayKind : std::uint8_t { Lagrange3, Thiran1 }; // P1 spike decides; fixed at prepare

struct StringMaterialParams {      // material = preset/morph of loss + dispersion
    float lossGainLow = 0.5f;      // loop loss at low frequencies, 0..1
    float lossGainHigh = 0.5f;     // loop loss at high frequencies, 0..1
    float dispersionAmount = 0.0f; // 0..1 scaling of allpass-chain coefficients
};

static_assert(std::is_trivially_copyable_v<StringMaterialParams>,
              "StringMaterialParams must stay trivially copyable for the realtime APVTS snapshot path.");

struct WaveguideStringParams {
    float f0Hz = 440.0f;        // target fundamental (MIDI 21..108 mapped upstream)
    float bendSemitones = 0.0f; // continuous retune contribution; click-free under constant modulation
    StringMaterialParams stringMaterial;
};

static_assert(std::is_trivially_copyable_v<WaveguideStringParams>,
              "WaveguideStringParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Normalized-knob -> physical-coefficient mapping for StringMaterialParams. docs/plan.md open
// question 4 ("Material morph parameter ranges ... need measurement, not speculation") assigns
// the final steel/nylon endpoints to P2, bound by the IR feature-invariant T60 measurements;
// these are the P1 working endpoints, chosen so the default 0.5 / 0.5 / 0.0 knob position is a
// plausible plain-steel-string decay and so the whole knob range stays strictly passive.
// The knob-1.0 endpoints sit very close to (but strictly below) unity on purpose: that end of
// the range is the "sustain" extreme, and the [tuning] sweep needs it, because the estimator
// mandated by docs/plan.md section 4.5 analyses 2^18 samples starting 0.5 s after the pluck and
// at the DEFAULT material MIDI 108 has a T60 of ~65 ms -- nothing would be left to measure.
inline constexpr float kLossGainLowMin = 0.95f;    // lossGainLow = 0 -> round-trip DC gain
inline constexpr float kLossGainLowMax = 0.99999f; // lossGainLow = 1 -> round-trip DC gain
inline constexpr float kLossGainHighMin = 0.70f;   // lossGainHigh = 0 -> round-trip Nyquist gain
inline constexpr float kLossGainHighMax = 0.9999f; // lossGainHigh = 1 -> round-trip Nyquist gain
inline constexpr float kLossFilterPoleZ = 0.3f;    // fixed pole of the one-pole-plus-zero loss filter
inline constexpr float kDispersionMaxCoeff = 0.2f; // dispersionAmount = 1 -> allpass coefficient -0.2

// Dual-rail bidirectional digital waveguide for one string. Called only from inside the
// StringNetwork per-sample loop (StringNetwork lands in Task P1.5). Per docs/plan.md section
// 2.1, every sample-domain class is template <typename SampleT> with explicit float/double
// instantiations compiled into cnpg_dsp -- the realtime path uses float, the tier-2 [energy]
// tests run double.
template <typename SampleT> class WaveguideString {
  public:
    // Message thread; allocates the rails. Sizes them for kMinMidiNote (minus the +/-2-semitone
    // bend range) against max(sampleRate, kMaxDesignRateHz), so a 192 kHz best-effort host never
    // under-allocates. `kind` is fixed for the life of the prepared instance. Calls reset().
    void prepare(double sampleRate, int maxBlockSize, FractionalDelayKind kind);

    // Realtime-safe. Zeroes both rails and every filter state and snaps all per-sample smoothers
    // onto their targets, so a reset instance is indistinguishable from a freshly prepared one.
    void reset() noexcept;

    // Realtime-safe; only retargets the per-sample smoothers (f0 including bend, loss gains,
    // dispersion amount). Never resizes and never touches rail contents.
    void setParams(const WaveguideStringParams& p) noexcept;

    // f0 compensation hook. Exactly one source is active:
    //   P1: analytic phase-delay correction computed internally from the filter coefficients at
    //       f0 (see the file header). `delaySamplesCorrection` is an ADDITIONAL caller-supplied
    //       offset in samples subtracted from the solved loop length -- 0 is the shipping value;
    //       cnpg_calibrate (P2.7) uses it to probe residuals. Calling this selects the analytic
    //       source, i.e. deactivates any table loaded by loadCalibrationTable.
    //   P2: per-note cents-correction table measured from the real dsp/ filters by cnpg_calibrate.
    // P1 has exactly one source, so "selects" is currently vacuous -- Task P2.7 adds the selector
    // together with the table's first real consumer.
    void setAnalyticTuningCompensation(float delaySamplesCorrection) noexcept;

    // Message thread; may allocate. Stores the table and nothing else -- the analytic source
    // stays active until Task P2.7 adds the selector, exactly as the P1.4 brief specifies
    // ("loadCalibrationTable stores the table and is a no-op source until P2 selects it").
    // Passing nullptr or count <= 0 clears any stored table.
    void loadCalibrationTable(const float* centsByMidiNote, int firstMidiNote, int count);

    // ---- rail access for the per-sample loop -------------------------------------------------

    // Adds `excitation` into the string's displacement at position01 (0 = nut, 1 = bridge),
    // split evenly between the two rails and deposited with amplitude-complementary linear
    // weights (g1 + g2 = 1) across the two adjacent rail slots.
    void injectAt(float position01, SampleT excitation) noexcept;

    // Fractional tap, read through the dual-anchor amplitude-complementary crossfade described in
    // the file header, so `position01` may be modulated continuously while the string rings. NOT
    // const: the call retargets the tap slot's anchor pair. That retarget is IDEMPOTENT -- calling
    // readTapAt twice with the same position inside one sample returns the same value and leaves
    // the same state -- because the crossfade itself only ever advances in tick().
    //
    // `tapSlot` selects one of kMaxTapsPerString independent anchor pairs, so a network reading a
    // string at several coil positions gets a crossfade per coil rather than one shared one; the
    // single-argument overload is slot 0. An out-of-range slot returns 0.
    //
    // At position01 == 0 the two rails cancel to the accuracy of the linear rail interpolation
    // against the loop's own interpolator -- near-silent at the rigid nut, but not bit-exactly
    // zero, since the loop reads the rail with the fractional-delay interpolator while the tap
    // reads it linearly.
    SampleT readTapAt(int tapSlot, float position01) noexcept;
    SampleT readTapAt(float position01) noexcept { return readTapAt(0, position01); }

    // 2-port insertion seam for DamperJunction at position p (DamperJunction itself is P2.2).
    // readJunctionInputs reports the two waves arriving at p; writeJunctionOutputs deposits the
    // DIFFERENCE between the junction's outputs and those same arriving waves back into the
    // rails, so a transparent junction (toBridge == fromNut, toNut == fromBridge) is bit-exactly
    // a no-op. Both address the rails through the seam's own dual-anchor crossfade (file header),
    // which is why readJunctionInputs is no longer const.
    //
    // THE PAIR IS ONE OPERATION and must be called with the same position inside one sample, read
    // first. Both retarget the seam's anchor pair and that retarget is idempotent, so the write
    // sees exactly the weights the read used -- which is what makes the difference it deposits the
    // exact transpose of the read, and therefore what makes the moving seam passive (file header)
    // and a transparent junction bit-exactly free even mid-fade.
    void readJunctionInputs(float position01, SampleT& fromNut, SampleT& fromBridge) noexcept;
    void writeJunctionOutputs(float position01, SampleT toBridge, SampleT toNut) noexcept;

    // ---- bridge port coupling (power-normalized wave variables) -------------------------------

    // Wave leaving the string at the bridge, i.e. the output of the termination chain computed
    // by the most recent tick(). With no external port attached, tick() reflects it internally
    // with the rigid -1 termination.
    SampleT railOutgoingAtBridge() const noexcept { return bridgeOutgoing_; }

    // Overrides the internal rigid reflection for the NEXT tick() only. NOTE for P2.4: routing
    // the bridge through an external junction this way inserts one extra sample into the loop,
    // which the loop-length solve must then subtract -- that adjustment lands with
    // BridgeJunction, not here (P1 has no bridge port, so P1 tuning is unaffected).
    void railAcceptFromBridge(SampleT reflected) noexcept;

    // Reference impedance for power normalization across the bridge port. P1 ships a single
    // normalized string (1.0); per-string impedances arrive with BridgeJunction in P2.4.
    float portImpedance() const noexcept { return 1.0f; }

    // ---- per-sample loop ---------------------------------------------------------------------

    // Advance rails and termination filters one sample. Realtime-safe; never allocates.
    void tick() noexcept;

    // Tier-2 energy test hook: bypasses the loop loss filter (the only intentional loss in the
    // string) so the remaining loop is lossless up to the fractional interpolator's own
    // magnitude response. Realtime-safe.
    void setLossBypassed(bool bypass) noexcept;

    // Per-sample smoothed fundamental actually being synthesized right now, in Hz.
    float currentF0Hz() const noexcept { return static_cast<float>(f0Smoothed_); }

    // ---- diagnostics (tests, cnpg_calibrate) --------------------------------------------------

    // Total realized round-trip PHASE delay in samples at currentF0Hz(): rails + fractional
    // delay + dispersion chain + loss filter. Equals fs / currentF0Hz() to solver precision.
    double realizedLoopDelaySamples() const noexcept;

    FractionalDelayKind fractionalDelayKind() const noexcept { return kind_; }

    // Integer part of each rail read, i.e. the LIVE RAIL WINDOW every position-mapped access must
    // stay inside. Slots at delays 1..railBase() are still travelling; Lagrange3's four
    // interpolator taps additionally keep railBase()+1..railBase()+3 live, while Thiran1's rail is
    // consumed at railBase() and nothing past it is ever read again. injectAt / readTapAt /
    // readJunctionInputs / writeJunctionOutputs must address only that window -- addressing past it
    // silently discards signal, which is exactly the defect fixed in P1.4 (see positionSpan_
    // below). "CONTRACT: WaveguideString injections stay inside the live rail window" asserts it.
    int railBase() const noexcept { return railBase_; }

    // Rail contents at delay d >= 1 (d == 1 is the sample written one tick ago); `upRail` selects
    // the nut -> bridge rail. Diagnostics/tests only -- the loop never reads the rails this way.
    SampleT railSampleAtDelay(bool upRail, int delaySamples) const noexcept;

    // Discrete Lyapunov storage function -- see the file header. NOT realtime-safe: the first
    // call after a coefficient change may run a closed-form spectral factorization. Test and
    // diagnostic use only (docs/plan.md section 4.2).
    double energyEstimate() const noexcept;

    // Where a moving read actually sits RIGHT NOW (Task P2.3) -- all of it, which is the point.
    // `anchor01` is the committed position, `pending01` the one being faded in (equal to the anchor
    // when no crossfade is in flight), and `crossfade01` is g2, the amplitude-complementary weight
    // on `pending01`. The effective read position is therefore
    //
    //     (1 - crossfade01) * anchor01 + crossfade01 * pending01
    //
    // and THAT is the quantity a continuity assertion has to be made about: a crossfade commit
    // moves `anchor01` by up to the whole threshold in one sample while the effective position does
    // not move at all, so an assertion written against the anchor alone would report a jump that
    // does not exist -- and, worse, would miss one that does. A smoother nobody can observe is a
    // smoother nobody can gate (the P2.2 lesson), and this is the moving seam's entire state.
    // `armed` is false only before the first read after reset(), where there is no previous
    // position to be continuous with and the first request is therefore snapped rather than faded.
    struct PositionCrossfade {
        float anchor01 = 0.0f;
        float pending01 = 0.0f;
        float crossfade01 = 0.0f;
        bool armed = false;
    };
    PositionCrossfade tapCrossfade(int tapSlot) const noexcept;
    PositionCrossfade junctionCrossfade() const noexcept;

    // Length of one position crossfade in samples at the prepared rate: round(fs *
    // kPositionCrossfadeSeconds), floored at 1. Exposed so a test can assert the ramp it actually
    // runs instead of re-deriving the rounding.
    int positionCrossfadeSamples() const noexcept { return positionCrossfadeSamples_; }

  private:
    struct FirstOrderAllpass {
        SampleT state{};
        SampleT coeff{};
        SampleT process(SampleT x) noexcept {
            const SampleT y = coeff * x + state;
            state = x - coeff * y;
            return y;
        }
    };

    // One moving read's whole state: the committed anchor, the anchor being faded in, the fade's
    // progress, and whether anything has been read through it yet. See the file header.
    struct PositionAnchor {
        double anchor = 0.0;
        double pending = 0.0;
        double fade = 0.0; // g2 in [0, 1]
        bool fading = false;
        bool armed = false; // false until the first read after reset(): nothing to be continuous with
    };

    // Idempotent within a sample -- see readTapAt / readJunctionInputs for why that matters.
    void retargetAnchor(PositionAnchor& anchor, double position01) noexcept;
    void advanceAnchor(PositionAnchor& anchor) noexcept; // one crossfade step; called from tick()
    SampleT tapAtPosition(double position01) const noexcept;
    static PositionCrossfade describeAnchor(const PositionAnchor& anchor) noexcept;

    void retargetSmoothers() noexcept;
    void advanceSmoothers() noexcept;
    void updateCoefficients() noexcept; // re-solves the loop length; called from tick()
    double interpolatorPhaseDelay(double d, double w, double sinw, double cosw) const noexcept;
    double solveFractionalDelay(double target, double w, double sinw, double cosw, double lo, double hi) const noexcept;
    SampleT runLoopChain(SampleT x) noexcept;

    // Rail addressing. Delay index d >= 1 reads the value written d ticks ago (the frame between
    // ticks, i.e. after the most recent write).
    void railDeposit(std::vector<SampleT>& rail, int writeIndex, double d, SampleT value) noexcept;
    SampleT railInterpolate(const std::vector<SampleT>& rail, int writeIndex, double d) const noexcept;
    SampleT railFractionalRead(const std::vector<SampleT>& rail, int writeIndex) const noexcept;

    // Position mapping. p = 0 (nut) is the freshest write of the up rail and the far end of the
    // dn rail; p = 1 (bridge) is the far end of the up rail and the freshest dn write. The span is
    // whatever part of the rail is still ADDRESSABLE -- see positionSpan_.
    double upDelayAt(double position01) const noexcept { return 1.0 + position01 * positionSpan_; }
    double dnDelayAt(double position01) const noexcept { return 1.0 + (1.0 - position01) * positionSpan_; }

    void refreshEnergyCache() const noexcept;

    static constexpr int kDispersionOrder = 4;  // locked: 4 cascaded first-order allpasses
    static constexpr int kLagrangeTaps = 4;     // cubic (order-3) Lagrange interpolator
    static constexpr double kMinRailSpan = 2.0; // keeps the fractional read inside the rail

    // ---- configuration ------------------------------------------------------------------------
    double sampleRate_ = 44100.0;
    double f0Min_ = 24.5;   // MIDI 21 detuned two semitones down; rails are sized for this
    double f0Max_ = 4410.0; // sampleRate / kMinLoopPeriodSamples, recomputed in prepare()
    FractionalDelayKind kind_ = FractionalDelayKind::Lagrange3;

    std::vector<SampleT> up_;
    std::vector<SampleT> dn_;
    int mask_ = 0;
    int upWrite_ = 0;
    int dnWrite_ = 0;

    // ---- parameters and per-sample smoothers --------------------------------------------------
    WaveguideStringParams params_{};
    double smoothingCoeff_ = 0.0; // one-pole per-sample coefficient
    double f0Target_ = 440.0;     // Hz, includes bend
    double f0Smoothed_ = 440.0;
    double lossLowTarget_ = 0.0; // physical round-trip gains, already mapped from the knobs
    double lossLowSmoothed_ = 0.0;
    double lossHighTarget_ = 0.0;
    double lossHighSmoothed_ = 0.0;
    double dispersionTarget_ = 0.0; // allpass coefficient (<= 0), already mapped from the knob
    double dispersionSmoothed_ = 0.0;
    bool smoothersSettled_ = false;

    double analyticCorrectionSamples_ = 0.0;
    std::vector<float> calibrationCents_; // stored by loadCalibrationTable; P2.7 adds the selector
    int calibrationFirstMidiNote_ = 0;

    // ---- rail reads and termination chain ------------------------------------------------------
    int railBase_ = 1;              // integer part of each rail read
    double realizedRailSpan_ = 2.0; // solved per-rail phase delay (railBase_ + interpolator)
    double fractionalDelay_ = 1.5;  // interpolator design delay

    // Span used to map position01 onto a rail delay. This is NOT always realizedRailSpan_,
    // because it must stay inside the part of the rail that is still addressable:
    //   Lagrange3 -- the interpolator IS the rail read, so the rail is live out to railBase_ + 3
    //     (its four taps). realizedRailSpan_ + 1 <= railBase_ + 3 for every D in [1, 2], so the
    //     realized (continuous, fractional) span is usable directly. That is what keeps taps and
    //     injections from stepping when the integer part of the rail read moves during a bend.
    //   Thiran1 -- the allpass sits OUTSIDE the rail, so the rail is consumed at delay railBase_
    //     and nothing past it is ever read again. Mapping over realizedRailSpan_ (= railBase_ plus
    //     up to 1.8 samples of allpass phase delay) would put taps and injections near the bridge
    //     into already-consumed slots: at MIDI 108 / 44.1 kHz railBase_ is 2, so the whole dn half
    //     of a p = 0.28 pluck would land at delays 3-4 and be silently discarded. The span is
    //     therefore railBase_ - 1, so p = 1 lands exactly on the last live slot. The cost is that
    //     the mapping steps with railBase_ -- which is the same integer-stepping this class avoids
    //     for Lagrange3, and one more reason Lagrange3 is the shipping default
    //     (docs/decisions/0002-fractional-delay.md).
    double positionSpan_ = 2.0;
    SampleT lagrangeCoeff_[kLagrangeTaps]{};
    FirstOrderAllpass fracState_[2]{}; // Thiran-1 allpass per rail (unused for Lagrange3)
    FirstOrderAllpass dispersion_[kDispersionOrder]{};

    SampleT lossB0_ = SampleT(1);
    SampleT lossB1_ = SampleT(0);
    SampleT lossA1_ = SampleT(0);
    SampleT lossState_ = SampleT(0);
    bool lossBypassed_ = false;

    SampleT bridgeOutgoing_ = SampleT(0);
    SampleT bridgeAccepted_ = SampleT(0);
    bool bridgeAcceptPending_ = false;

    double realizedLoopDelay_ = 0.0;

    // ---- moving-position anchors (see the file header) -----------------------------------------
    // One anchor pair per tap slot plus one for the junction seam. They are ORDINARY state, not a
    // cache: the fade advances once per tick() and nothing else may move it.
    std::array<PositionAnchor, kMaxTapsPerString> tapAnchor_{};
    PositionAnchor junctionAnchor_{};
    int positionCrossfadeSamples_ = 1;
    double positionFadeStep_ = 1.0;

    // ---- energy-storage cache (see energyEstimate()) -------------------------------------------
    mutable bool energyCacheValid_ = false;
    mutable double lagrangeStorage_[6]{}; // upper triangle of the 3x3 storage matrix P
    mutable double lossStorage_ = 0.0;    // scalar p in V = p * s^2
};

extern template class WaveguideString<float>;  // realtime path
extern template class WaveguideString<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

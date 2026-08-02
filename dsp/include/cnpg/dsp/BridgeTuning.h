#pragma once

#include <cmath>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/IBridgePort.h"

// BridgeTuning -- the fixed-point solve that turns an IBridgePort's phase delay into the loop-length
// correction a string is tuned with. Task P2.7; docs/decisions/0007-bridge-tuning-compensation.md
// D6 ("the solver is specified, not merely required") is the governing text and this file is what
// answers it. Zero JUCE includes; header-only, because it is arithmetic over an interface and there
// is nothing here worth a translation unit.
//
// ---------------------------------------------------------------------------------------------
// WHAT IS ACTUALLY BEING SOLVED, AND WHERE THE SELF-REFERENCE REALLY IS
// ---------------------------------------------------------------------------------------------
// A digital waveguide loop resonates where its round-trip PHASE delay equals one period, i.e.
// where D(f) = fs / f. WaveguideString's loop-length solve chooses the rail span so that this
// holds AT THE TARGET, evaluating every filter's phase delay at the target f0. Adding the bridge
// is one more term in the same sum:
//
//     2*railSpan + tau_dispersion(f) + tau_loss(f) + tau_seam + tau_port(f)  =  fs / f
//
// Choose the rail span from that equation at f = f_target with tau_port evaluated at f_target, and
// f_target is EXACTLY a root. So the compensation itself is a single closed-form evaluation, not an
// iteration -- and stating that plainly matters, because the ADR anticipated an iteration and the
// derivation says otherwise for the ordinary case.
//
// The self-reference is real, and it is about UNIQUENESS rather than about the value. Read the same
// equation the other way: GIVEN a committed compensation tau_c, the frequency the string actually
// sings at is the fixed point of
//
//     Phi(f) = fs / ( fs/f_target - tau_c + tau_port(f) )                                  ... (*)
//
// which is implicit in f, because tau_port is evaluated at the frequency being solved for. f_target
// is a fixed point of (*) whenever tau_c = tau_port(f_target). Whether it is the ONLY one nearby --
// and whether the loop settles onto it rather than being pulled off it -- is decided by
//
//     |Phi'(f)| = (f^2 / fs) * |d tau_port / df|
//
// and that number is not small everywhere. Near a sharp bridge resonance the port's phase slope is
// steep, and once |Phi'| >= 1 the root stops being isolated: the string's fundamental and the bridge
// mode enter an avoided crossing and the loop has three phase-zero crossings instead of one. That is
// physics, it is audible, and it is exactly the "steep phase-slope region around bridge resonance"
// ADR 0007 D6 names as this task's risk item.
//
// So the solver's job is BOTH halves: produce the compensation, and find out whether the root it
// was computed for is a root the instrument can hold. It does the second by iterating (*) from a
// deliberate displacement and requiring the iterate to come back.
//
// ---------------------------------------------------------------------------------------------
// THE CONTRACT (declared here so it is one place, and tested rather than asserted)
// ---------------------------------------------------------------------------------------------
//   tolerance          kBridgeTuningToleranceCents = 0.25 cents -- 8x tighter than the +/-2 cent
//                      [tuning] gate. The margin is stated rather than implied: the solver may
//                      contribute at most an eighth of the budget the gate allows.
//   probe              kBridgeTuningProbeCents = 2.0 cents -- the gate's own width. "A string one
//                      whole gate-width off target returns to inside an eighth of one" is a
//                      statement about the thing being gated, not an arbitrary epsilon.
//   iteration cap      kBridgeTuningMaxIterations = 8 per probe direction.
//   fallback           the one-shot compensation tau_port(f_target), clamped, committed anyway,
//                      with `converged` false. See below for why it is not "revert to zero".
//
// The cap, tolerance and probe together put the convergence boundary at a contraction ratio of
// 0.125^(1/8) = 0.7715 (eight geometric steps must take 2 cents under 0.25). For BridgeJunction's
// load that ratio is |Phi'| ~ mu / (2 pi zeta) with mu = couplingStrength * kBridgeMaxMobilityRatio,
// which puts the boundary near zeta ~ 0.0103 * couplingStrength. MEASURED, at couplingStrength 1.0:
// zeta = 0.0197 (tests/dsp/TuningAccuracyTests.cpp prints it) -- the derivation has the scaling and
// the order right and is low by a factor of 1.9, because the iterate is not exactly geometric near
// the boundary. Either number is ABOVE kBridgeMinDamping = 0.01, so THE FALLBACK IS REACHABLE BY
// DRAGGING TWO SHIPPED SLIDERS TO THEIR STOPS. It is not a corner nobody can get to, and P2.4's
// review is why that sentence is written down instead of assumed: a branch that file called
// unreachable was a shipped slider's minimum. TuningAccuracyTests drives it and asserts the
// fallback, including that the fallback is CONTINUOUS across its own boundary (worst step in
// committed compensation across a sweep that crosses it: 0.0095 samples, i.e. 0.015 cents).
//
// WHY THE FALLBACK COMMITS THE COMPENSATION RATHER THAN DISCARDING IT. Reverting to tau = 0 on
// non-convergence would make the compensation DISCONTINUOUS in the parameters: dragging Bridge
// Damping across the convergence boundary would step every ringing string's loop length by up to
// ~14 cents in one block, which is the click the whole P2.3 crossfade apparatus exists to prevent,
// installed at a boundary the user cannot see. Committing the one-shot value keeps the correction
// continuous everywhere and confines the failure to what it actually is: a region where the
// +/-2 cent guarantee does not hold. That region is the Extended (Effect) range of ADR 0007 D5, and
// it is declared rather than discovered.

namespace cnpg::dsp {

// Convergence tolerance in cents, and the margin it is stated against. Both here so a test can
// assert the RELATIONSHIP rather than restate the numbers.
inline constexpr double kBridgeTuningToleranceCents = 0.25;
inline constexpr double kBridgeTuningGateCents = 2.0; // docs/plan.md section 4.5's criterion
inline constexpr double kBridgeTuningToleranceMargin = kBridgeTuningGateCents / kBridgeTuningToleranceCents; // 8x

// The displacement the fixed point is probed from, both directions. One gate width.
inline constexpr double kBridgeTuningProbeCents = kBridgeTuningGateCents;

// Iteration cap PER PROBE DIRECTION.
inline constexpr int kBridgeTuningMaxIterations = 8;

// Hard bound on the committed compensation, as a fraction of the loop period. Nothing physical
// comes anywhere near it -- BridgeJunction's own maximum is bounded by the peak mobility ratio at
// roughly 0.008 of a period -- so this exists solely so that a pathological or hostile IBridgePort
// implementation cannot drive the rail-span solve out of range. A fraction of the period rather
// than an absolute sample count, because it has to mean the same thing at 27.5 Hz and at 4186 Hz.
inline constexpr double kBridgeTuningMaxPeriodFraction = 0.25;

// Everything the solve produced, including how it got there. `iterations` and `residualCents` are
// not decoration: ADR 0007 D5's criterion (2) is "the fixed-point solver converges reliably", and a
// criterion nobody can observe is a criterion nobody can gate -- the P2.2 lesson, applied to a
// solver instead of to a smoother.
struct BridgeTuningSolution {
    double phaseDelaySamples = 0.0; // the correction to subtract from the loop length
    double residualCents = 0.0;     // worst |cents| the probe iterate still had at its last step
    int iterations = 0;             // worst iteration count over the two probe directions
    bool converged = true;          // false => the +/-2 cent guarantee is void at this setting
    // The port's own answer had to be corrected: it was non-finite (resolved to 0) or outside the
    // period-fraction bound (clamped to it). Nothing physical reaches this -- BridgeJunction's own
    // maximum is ~0.008 of a period against a bound of 0.25 -- so a true here means the attached
    // IBridgePort is broken, not that the setting is extreme.
    bool clamped = false;
};

namespace detail {

inline double centsBetweenHz(double measured, double target) noexcept { return 1200.0 * std::log2(measured / target); }

} // namespace detail

// Solve the bridge tuning compensation for one string. Realtime-SAFE in the sense that matters --
// it allocates nothing, locks nothing and performs no I/O -- but it is NOT per-sample work: callers
// invoke it on parameter change and on note change only (ADR 0007 D6, "offline at parameter-change
// time, never on the audio path"). Cost is at most 2 + 2*kBridgeTuningMaxIterations port
// evaluations, each a handful of trig calls.
template <typename SampleT>
BridgeTuningSolution solveBridgeTuning(const IBridgePort<SampleT>& port, int portIndex, double targetHz,
                                       double sampleRate, int numPorts) noexcept {
    BridgeTuningSolution solution;
    if (!(targetHz > 0.0) || !(sampleRate > 0.0))
        return solution; // nothing to tune; the uncompensated loop is the defined answer

    const double period = sampleRate / targetHz;
    const double bound = kBridgeTuningMaxPeriodFraction * period;

    // One evaluation, clamped and NaN-resolved. A port that answers with a non-finite number is a
    // broken port, and the defined behaviour is the uncompensated loop -- which is exactly the
    // instrument P2.4 shipped, so the failure degrades to a known state rather than to an unknown
    // one. Written as a rejection of "is it inside the range", so NaN falls out of the first test
    // rather than needing its own branch (the same shape BridgeJunction::sanitize uses).
    const auto evaluate = [&](double hz, bool& outOfBound) noexcept {
        const double raw = port.reflectionPhaseDelaySamples(portIndex, hz, numPorts);
        if (!(raw > -bound))
            outOfBound = true;
        else if (!(raw < bound))
            outOfBound = true;
        else
            return raw;
        if (!(raw == raw)) // NaN
            return 0.0;
        return (raw > 0.0) ? bound : -bound;
    };

    bool clamped = false;
    solution.phaseDelaySamples = evaluate(targetHz, clamped);
    solution.clamped = clamped;

    // ---- the fixed point (*), probed from one gate width either side -------------------------
    // With the compensation committed, the frequency the string settles at is the fixed point of
    //     Phi(f) = fs / (period - tau_c + tau_port(f))
    // whose fixed point is targetHz by construction. Displacing and requiring a return is what
    // turns "the correction is exact" from a theorem about the ordinary case into a measurement
    // that also covers the case where it is not.
    bool converged = true;
    for (int direction = 0; direction < 2; ++direction) {
        const double sign = (direction == 0) ? 1.0 : -1.0;
        double hz = targetHz * std::exp2(sign * kBridgeTuningProbeCents / 1200.0);
        bool landed = false;
        int steps = 0;
        double residual = std::fabs(kBridgeTuningProbeCents);

        for (int i = 0; i < kBridgeTuningMaxIterations; ++i) {
            ++steps;
            bool ignored = false;
            const double tau = evaluate(hz, ignored);
            const double denominator = period - solution.phaseDelaySamples + tau;
            if (!(denominator > 0.0))
                break; // the loop equation has no positive root here at all
            hz = sampleRate / denominator;
            if (!(hz > 0.0))
                break;
            residual = std::fabs(detail::centsBetweenHz(hz, targetHz));
            if (!(residual >= 0.0))
                break; // NaN
            if (residual <= kBridgeTuningToleranceCents) {
                landed = true;
                break;
            }
        }

        solution.iterations = (steps > solution.iterations) ? steps : solution.iterations;
        solution.residualCents = (residual > solution.residualCents) ? residual : solution.residualCents;
        if (!landed)
            converged = false;
    }
    solution.converged = converged;
    return solution;
}

} // namespace cnpg::dsp

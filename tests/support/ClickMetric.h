#pragma once

#include <cstddef>
#include <vector>

// ClickMetric -- THE click metric for all of phase P2, defined once here (docs/plan.md Task P2.1
// steps: "Define the click metric here, once, for all of P2 (later tasks reference this
// definition)"). P2.2 (damper), P2.3 (pickup morph/position), P2.4 (bridge coupling) and P2.6
// (retrigger) all gate a state change against it; every one of them uses this file rather than
// re-deriving a ratio of its own. P2.4's gate is "CONTRACT: dragging Bridge Coupling to zero is
// click-free through the network" (tests/dsp/BridgePortContractTests.cpp) -- named here because
// this sentence claimed that gate for a round before it existed.
//
// -----------------------------------------------------------------------------------------------
// THE DEFINITION (verbatim from the task, then how each term is realised)
// -----------------------------------------------------------------------------------------------
//
//   click metric = max over 10 ms windows of (peak |first difference| / whole-render median
//   |first difference|), compared against a reference render without the state change; the
//   criterion is that the metric stays within 3 dB of the reference, with no NaN and no
//   denormal-guard trips.
//
//   - "first difference"          x[n] - x[n-1], in float64, over the analysed span.
//   - "peak |first difference|"   the largest |dx| inside one 10 ms window.
//   - "median |first difference|" the signal's ordinary per-sample motion, which is what makes the
//                                 metric a RATIO: a discontinuity is only a click relative to how
//                                 fast the waveform was already moving.
//   - "within 3 dB"               metric(test) <= metric(reference) * 10^(3/20).
//
// The 10 ms windowing, recorded rather than glossed over: because the denominator is a span
// constant, "max over windows of (window peak / median)" is algebraically equal to "(span peak) /
// (median)". The window size therefore does not change the NUMBER. It is implemented as written
// anyway, because the winning window's start index localises where the suspected click is -- which
// is what a failing gate actually needs to report.
//
// -----------------------------------------------------------------------------------------------
// WHICH RENDER'S MEDIAN (the one real ambiguity in the definition, resolved here for all of P2)
// -----------------------------------------------------------------------------------------------
//
// "whole-render median" does not say WHICH render, and the two readings are not equivalent.
//
//   (a) Each render normalised by its OWN median. Rejected. It fails renders that contain no
//       discontinuity at all, for purely arithmetic reasons. Measured, on the P2.1 enable ramp: a
//       10 ms linear fade to silence produced an excess of +3.21 dB -- over the 3 dB criterion --
//       with a peak |dx| no larger than the reference's. The whole excess came from the
//       denominator: the muted render's span ends in silence, silence contributes |dx| = 0, and
//       those zeros drag the median down. The metric then rises because the signal got QUIETER,
//       which is the opposite of a click. The same effect runs the other way too: a state change
//       that legitimately increases the signal's ordinary motion inflates its own denominator and
//       hides a real discontinuity underneath it.
//
//   (b) Both renders normalised by the REFERENCE's median over the same span. Adopted. The
//       denominator is then what it was always meant to be -- "how fast was this waveform moving
//       before we did anything to it" -- and it is a constant of the comparison rather than a
//       function of the thing being judged. The criterion reduces to: the largest per-sample jump
//       must not grow by more than 3 dB. That is exactly what a click is.
//
// Reading (b) keeps every tooth: on that same P2.1 case a HARD CUT (the pre-ramp behaviour) still
// fails by ~32 dB, and the test that gates the ramp carries that hard cut as a standing negative
// control so the gate can never quietly go vacuous.
//
// -----------------------------------------------------------------------------------------------
// WHAT READING (b) COSTS: the metric is NOT scale-invariant, and here is its blind spot
// -----------------------------------------------------------------------------------------------
//
// Both renders sharing one denominator means the denominator CANCELS out of the criterion:
//
//     clickExcessDb(test, reference) == 20*log10(test.peak|dx| / reference.peak|dx|)
//
// The criterion is therefore a comparison of ABSOLUTE peak first differences. That is the right
// question for a state change that leaves the level roughly alone, and it is why the mute case
// stopped producing a false failure. It also has a specific, nameable blind spot, which every
// caller must weigh before leaning on this number alone:
//
//   A state change that BOTH reduces the level AND inserts a discontinuity can pass.
//
// A step worth half of a signal that the same change made 10 dB quieter is small in absolute |dx|
// -- it can sit comfortably under the reference's peak -- while being plainly audible as a click,
// because what a listener hears it against is the quiet signal that remains, not the loud one that
// used to be there. Note that most of what is left of P2 is a level-reducing change: P2.2's damper
// engagement, P2.6's <= 5 ms Synth fade, P2.4's coupling. This is not a hypothetical.
//
// Its sensitivity floor is worth stating in the same breath, since it is measurable rather than
// notional. On the P2.1 enable case (tests/dsp/StringNetworkScaleTests.cpp, "ramps a disabled string
// silent without a click") the hard-cut negative control fails by 32.50 dB against a 3 dB limit,
// i.e. it clears the bar by 29.50 dB -- so a discontinuity worth about 3.4% of that control's jump
// (20*log10(0.0335) = -29.5 dB) is where the criterion breaks even. The gate catches a
// full-amplitude discontinuity by a very wide margin, and a discontinuity worth a few percent of the
// signal's loudest sample marginally.
//
// THAT FLOOR WAS UNDERSTATED BY 24 dB UNTIL THE CONTROL WAS PLACED BY LEVEL, and the correction is
// the point rather than a footnote. This paragraph previously read 8.29 dB, from a control cut at
// the toggle sample -- an arbitrary phase of a ringing string. A hard cut's peak |dx| IS the sample
// value it lands on, so a blindly placed cut measures the waveform's phase at one index and not the
// metric's sensitivity at all; the same cut placed at the loudest sample in the same window reads
// 32.50 dB.
//
// -----------------------------------------------------------------------------------------------
// *** THE PLACEMENT RULE. IT BINDS CONTROLS AND PERTURBATIONS ALIKE. ***
// -----------------------------------------------------------------------------------------------
//
//   EVERY SAMPLE INDEX AT WHICH THIS SUITE INSERTS, REMOVES OR TRIGGERS A STATE CHANGE MUST BE
//   CHOSEN FROM THE WAVEFORM, NEVER FROM THE CLOCK.
//
// "From the clock" means a round number of samples, blocks, milliseconds or event offsets. Any such
// index is an ARBITRARY PHASE of whatever the instrument is doing there, and every statistic in this
// file is a function of that phase. The rule is stated for BOTH roles because the defect is one
// defect and it has now appeared in both:
//
//   THE NEGATIVE CONTROL -- the artificial discontinuity a gate carries to prove it still
//   discriminates. Placed blind it can land near a zero crossing, read as a cut of almost nothing,
//   and PASS the gate it exists to fail, leaving the gate provably toothless while looking healthy.
//
//   THE PERTURBATION -- the real state change the gate exists to judge. Placed blind, the gate's own
//   reading is one sample of a distribution the test never characterised, and the test cannot then
//   tell a regression from a shift in phase. This is the sharper half and it took longer to see.
//
// EVERY OCCURRENCE SO FAR, all in P2, every one found by review rather than by the suite:
//
//   - P2.2, where the lesson originated: a control placed at the state-change sample rather than by
//     level (tests/dsp/SustainPedalTests.cpp still cites it as "the P2.2 lesson").
//   - P2.6's six-damper pedal-release control, the same defect in a new file: 0.27 dB placed blind
//     at the pedal sample -- i.e. the control PASSED the gate it exists to fail -- against 32.04 dB
//     placed by level. Its sympathetic-truncation control was fixed in the same round (30.70 dB).
//   - P2.6 fixes wave 1, FOUR negative controls at once: +5.90 / +6.61 / +18.89 / +24.07 dB. This
//     is where the rule was first written down -- for controls only.
//   - P2.6 fixes wave 2, the note-off-age PERTURBATION, which is where the rule was found to bind
//     perturbations too: a round 20 ms landed on a zero crossing and the row reported exactly
//     0.00 dB while the discarded peak was -28.8 dBFS. Fixed there and not carried back.
//   - P2.6 fixes wave 3, the cross-pitch re-strike -- a perturbation at a block boundary INSIDE A
//     GATE. Recomputing that gate at ten re-strike phases across one period of the old note swung
//     its median from 0.111 to 3.199 dB, i.e. across the 3 dB criterion, on identical code.
//
// THE PROCEDURE, so the next occurrence has something specific to violate:
//
//   1. Search a window of the relevant waveform for the extremum of |x|, and place there (for a cut,
//      whose peak |dx| IS the sample it lands on) or on the sample AFTER it (for a perturbation
//      whose step replaces that sample).
//   2. THE WINDOW IS ONE FULL PERIOD of the component whose phase is arbitrary. A full period is the
//      shortest window guaranteed to contain the cycle's GLOBAL extremum, so every anchor inside the
//      period resolves to the same phase. A shorter window finds whatever local extremum happens to
//      be inside it, which is a different phase depending on where the window fell: measured on the
//      wave-3 site, a half-period window left the gate's median ranging 0.141 to 3.147 dB across
//      anchors -- barely better than no placement at all -- while a full-period window collapsed all
//      ten anchors onto one phase and a range of 0.017 dB. Use a shorter window ONLY when a SWEEP
//      needs adjacent placements to stay distinct (wave 2's age sweep did, and says so), and then
//      assert the distinctness.
//   3. ASSERT THE PLACEMENT WAS READ OFF THE RENDER BEING MEASURED -- bit-identity over the searched
//      window between the render the search ran on and the arm under test. Without it the placement
//      is a claim about a different waveform.
//   4. WHERE THE PLACEMENT DECIDES A GATE, assert the phase axis is closed: re-measure the gate at a
//      second placement one period away and require the two to agree. That is the only form in which
//      "the phase no longer matters" is evidence rather than an assertion.
//
// THE COMPANION MEASUREMENT. Where a change materially reduces level without silencing the signal,
// measure clickExcessAgainstResidualDb() as well (below). It normalises the test render's peak by
// the test render's OWN median over a span AFTER the change -- the ordinary motion of the signal
// that actually remains -- which is exactly the quantity reading (b) gives up. Use both: reading
// (b) is the criterion, and the residual reading is what stops it being the only thing anyone
// looks at. Where the change ends in silence the residual reading is degenerate (its denominator
// is zero, and it returns infinity), which is the same degeneracy reading (a) has and the reason
// it is a companion rather than a replacement.
//
// -----------------------------------------------------------------------------------------------
// CHOOSING THE ANALYSED SPAN (the caller's job, and it is load-bearing)
// -----------------------------------------------------------------------------------------------
//
// Both renders must be measured over the SAME span, and the reference must be the same render
// without the state change. The span needs enough signal before the change to establish the
// denominator, and enough after it to contain whatever the change did -- a click is impulsive, so
// "the transition plus a short margin" is sufficient after. "Whole render" is the right span for a
// change that leaves the instrument sounding either side of it; a change that ends in silence wants
// a bounded one, so that the reference's median still describes the signal at the moment of the
// change rather than an average over a decay that has since run away from it.
//
// -----------------------------------------------------------------------------------------------
// NaN / denormal
// -----------------------------------------------------------------------------------------------
//
// The criterion's second half ("no NaN and no denormal-guard trips") is measured on the same pass:
// nonFiniteSamples counts anything that is not finite, subnormalSamples counts anything the
// FTZ/DAZ guard should have flushed and did not. A caller asserts both are zero.

namespace cnpg::test {

// The criterion's tolerance, in dB. Locked for all of P2 by the P2.1 task definition.
inline constexpr double kClickMetricToleranceDb = 3.0;

// The window length the definition names. Locked for all of P2.
inline constexpr double kClickMetricWindowSeconds = 0.010;

struct ClickMeasurement {
    double peakWindowAbsDiff = 0.0;  // max over 10 ms windows of the peak |first difference|
    std::size_t peakWindowStart = 0; // sample index (relative to the span) of the winning window
    double medianAbsDiff = 0.0;      // this render's own median |first difference| over the span
    double peakAbsSample = 0.0;      // this render's own peak |x| over the span; reporting only
    double peakStepToLevel = 0.0;    // max over the same 10 ms windows of (window peak |dx|) /
                                     // (window peak |x|) -- the second companion below; not part
                                     // of reading (b)
    long long nonFiniteSamples = 0;
    long long subnormalSamples = 0;

    // The click metric, normalised by `reference`'s median (reading (b) above). Call it on the
    // reference itself to get the reference's own metric -- that is the number the criterion
    // compares against, and passing the reference as its own reference is what makes the two
    // numbers commensurable. Infinity if the reference's median is zero (a degenerate reference the
    // caller should reject before comparing anything).
    double metric(const ClickMeasurement& reference) const noexcept;
};

// Measures over samples[begin, end). `end` clamps to count; a span shorter than two samples yields
// a zeroed measurement, which no criterion can pass or fail meaningfully -- callers assert a
// non-degenerate reference first, exactly as the P1 pitch-bend gate already does.
ClickMeasurement measureClick(const float* samples, std::size_t count, double sampleRate, std::size_t begin,
                              std::size_t end);

ClickMeasurement measureClick(const std::vector<float>& samples, double sampleRate, std::size_t begin, std::size_t end);

// THE CRITERION. 20*log10(metric(test) / metric(reference)): how much WORSE the test render is.
// Positive means worse. The shared denominator cancels, so this is equivalently the growth of the
// peak |first difference| -- see "WHAT READING (b) COSTS" above for what that does and does not
// catch, before gating on it alone.
double clickExcessDb(const ClickMeasurement& test, const ClickMeasurement& reference);

// THE COMPANION, for a state change that materially reduces level without silencing the signal
// (P2.2 damper engagement, P2.6's Synth fade, P2.4 coupling). Each render's peak is normalised by
// the ordinary motion of ITS OWN settled signal after the change -- `testResidual` and
// `referenceResidual` are measurements of the same two renders over a span that starts once the
// change has finished. That is the scale reading (b) deliberately gives up, and it is what a
// listener judges a click against once the change has taken the level down.
//
// Symmetric on purpose. Normalising only the test render's peak by its residual, and leaving the
// reference on its full-span median, was tried first and is wrong: any level change that both
// renders share -- a pickup sweep, a decaying note -- then lands entirely in the numerator and
// reads as a click. Each render is judged against itself, and only the two judgements are compared.
//
// Returns +infinity when the test residual is silence, which is the degeneracy that makes this a
// companion to clickExcessDb() and not a replacement for it: a change that ends in silence has no
// residual to be loud against, and clickExcessDb() is then the only meaningful reading.
double clickExcessAgainstResidualDb(const ClickMeasurement& test, const ClickMeasurement& testResidual,
                                    const ClickMeasurement& reference, const ClickMeasurement& referenceResidual);

// -----------------------------------------------------------------------------------------------
// THE SECOND COMPANION (added at Task P2.2): peak |dx| against the SIGNAL'S OWN LEVEL, for a change
// that alters the signal's DECAY RATE rather than stepping it between two settled levels
// -----------------------------------------------------------------------------------------------
//
// clickExcessAgainstResidualDb above presumes there IS a settled signal after the change to
// normalise by. Damper engagement has no such thing: it does not move a note from one level to
// another, it changes how fast the note is FALLING, so the signal decays continuously from the
// moment of the change and every window after it is quieter than the one before. Measured on this
// project's own note-off renders that reading returns the accumulated level drop and nothing else
// -- 19 dB at maxLoss 0.05, 52 dB at 1.0, both of them the damper working exactly as designed.
//
// clickExcessAgainstLevelDb normalises each render's peak |dx| by THAT RENDER'S OWN PEAK |x|, and
// does it PER 10 ms WINDOW: the reading is max over windows of (window peak |dx| / window peak |x|).
// That is the one denominator a level change of any size, shape or speed divides out of exactly,
// because it is re-measured every 10 ms alongside the numerator. What is left is a pure shape
// number: for a band-limited waveform it sits near 2*pi*f_max/fs, while a step worth a fraction q
// of the signal puts the window containing it at about q. A discontinuity that the change itself
// made quiet -- the exact blind spot reading (b) is documented to have -- still reads large,
// because the yardstick got quiet with it.
//
// Per-window is not a refinement, it is the whole thing working. A single peak |x| taken over a
// span that decays 40 dB across itself comes from the span's loud beginning, and a step late in
// that span then hides behind it exactly as it hides in reading (b). Measured, on this project's
// own full note-off: a 6 dB step 50 ms after the note-off reads 1.3 dB with a span-wide denominator
// and 12 dB with the per-window one.
//
// Span choice still matters: measure over the POST-CHANGE window only. A span that also contains
// the loud pre-change signal will simply take its maximum from a pre-change window, where both
// renders are identical, and report nothing about the change at all.
//
// The criterion is the same 3 dB. Use it ALONGSIDE clickExcessDb, never instead of it: this reading
// is blind to a change that is smooth but wrong, and reading (b) is blind to a jump the change made
// quiet. Between them the level-reducing case is covered.
double clickExcessAgainstLevelDb(const ClickMeasurement& test, const ClickMeasurement& reference);

} // namespace cnpg::test

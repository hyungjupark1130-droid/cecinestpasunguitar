#include "cnpg/dsp/BridgeTuning.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/ClickMetric.h"
#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

// BridgeTuningClickTests -- ADR 0007 D6's second half, and Task P2.7's `[contract]` gate:
//
//   "P2.7 carries a dedicated gate proving that couplingStrength, bridgeResonanceHz and
//    bridgeDamping can each be changed WHILE STRINGS ARE SOUNDING without clicks, discontinuities,
//    or unstable pitch transitions. Note the third of those is new: click-free is not sufficient
//    here, because a converging solver can be smooth and still audibly hunt."
//
// ---------------------------------------------------------------------------------------------
// WHY THIS NEEDS TWO MEASUREMENTS AND NOT ONE
// ---------------------------------------------------------------------------------------------
// Changing a bridge parameter re-solves every string's tuning compensation, and committing a new
// compensation IS changing a ringing string's loop length. That is a PITCH change on a sounding
// note, so it has two independent failure modes and the click metric only sees one of them:
//
//   - a DISCONTINUITY, which the P2.1 click metric measures. Prevented structurally: the
//     compensation reaches the loop through the same loop-length solve f0 and the pitch wheel reach
//     it through, so the rail read moves continuously and every position derived from it moves
//     through P2.3's dual-anchor crossfade. A parameter tweak is therefore the same motion through
//     the rails as a pitch bend, and inherits the same click-freedom.
//
//     What it does NOT share with the pitch wheel is the smoother itself. The compensation glides on
//     an 8 ms LINEAR RAMP THAT LANDS (WaveguideString::setBridgePhaseDelaySamples), not on the 8 ms
//     one-pole f0 uses, and the difference is a measured CPU fact rather than a stylistic one -- a
//     one-pole never arrives, so it held every string in per-sample loop re-solve for 0.22 s after
//     every note change, at 2.55x the CPU. The same duration, a different shape. This gate is what
//     says the swap cost the audible transition nothing: -0.055 / -0.033 / +0.123 dB after it,
//     against -0.057 / -0.033 / +0.115 dB before, overshoot 0 and settled spread 0 in both.
//
//   - HUNTING, which it cannot. A pitch that glides smoothly to the wrong place and then smoothly
//     back is perfectly continuous and perfectly audible. So the second measurement is the
//     INSTANTANEOUS FREQUENCY of the fundamental across the transition, and what it asserts is that
//     the track does not overshoot its own landing point and does not reverse direction.
//
// ---------------------------------------------------------------------------------------------
// PLACEMENT (tests/support/ClickMetric.h's rule, which binds controls AND perturbations)
// ---------------------------------------------------------------------------------------------
// Every sample index at which this file inserts, removes or triggers a state change is chosen from
// the waveform, never from the clock: the parameter change and the negative control alike are placed
// on the extremum of |x| inside a FULL PERIOD of the note being played, the placement is asserted to
// have been read off the render actually under test, and the gate is re-measured one period later
// and required to agree. Four separate defects in P2.2-P2.6 came from skipping one of those.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 64; // small, so a parameter change can land on a chosen sample
// String 0's own open pitch, so the render is the instrument in its ordinary state. NOT a unison
// pair: two identical strings on one bridge are DEGENERATE, their symmetric and antisymmetric normal
// modes split, and "the pitch of the pair" then has no single well-defined value for a settled-pitch
// assertion to be about (measured: the sum of a unison pair reads 5.0 cents off nominal with the
// compensation working perfectly, because the split modes sit either side of it). String 1 rests at
// its own open A2 -- coupled, contributing to the click metric, and not degenerate with string 0.
constexpr int kMidiNote = 40;

// A whole note's period at MIDI 45 / 48 kHz is 436.4 samples; the search window is one full period,
// which is the shortest window guaranteed to contain the cycle's GLOBAL extremum.
int fullPeriodSamples() { return static_cast<int>(std::ceil(kRate / cnpg::test::midiNoteToHz(kMidiNote))); }

enum class Knob { Coupling, Resonance, Damping };

const char* knobName(Knob knob) {
    switch (knob) {
    case Knob::Coupling:
        return "couplingStrength";
    case Knob::Resonance:
        return "bridgeResonanceHz";
    case Knob::Damping:
        return "bridgeDamping";
    }
    return "?";
}

StringNetworkParams baseParams() {
    StringNetworkParams params;
    params.pickupPosition01 = 0.87f;
    params.exciter.noiseAmount = 0.0f;
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;
    // Two strings at their DEFAULT open tuning (E2 and A2): the change has to be click-free on a
    // string that is merely coupled to the plucked one as well as on the plucked one itself, and the
    // default tuning is the configuration a listener actually has.
    return params;
}

void applyKnob(StringNetworkParams& params, Knob knob, float value) {
    switch (knob) {
    case Knob::Coupling:
        params.bridge.couplingStrength = value;
        break;
    case Knob::Resonance:
        params.bridge.resonanceHz = value;
        break;
    case Knob::Damping:
        params.bridge.damping = value;
        break;
    }
}

struct Render {
    std::vector<float> tap;
    double compensationAtEnd = 0.0;
    bool converged = true;
    // The compensation in force, sampled once per block from the change onward. THIS IS THE PITCH:
    // the compensation is a loop-length term, and a loop-length term is a frequency. Observing it
    // directly rather than estimating the audio's instantaneous frequency is the P2.1 ruling applied
    // to this transition -- a heterodyne instantaneous-frequency track of a two-string tap is
    // contaminated by the neighbouring partials at exactly the level the claim needs resolved, and a
    // measurement whose noise floor exceeds its criterion measures nothing.
    std::vector<double> compensationTrack;
    // String 0's own channel, kept separate from the summed tap: the click metric wants everything
    // the instrument emits, and the pitch measurement wants the string whose pitch is being claimed.
    std::vector<float> string0;
    // The solved TARGET, re-sampled on every one of those blocks. This is the one that could hunt:
    // a host reissues setParams every block, so a solver whose answer depended on any smoothed state
    // would return a different target each time and drive the string around under a converging,
    // perfectly smooth-looking glide.
    std::vector<double> targetTrack;
};

// Renders `seconds` of the summed tap channel, moving `knob` from `from` to `to` at sample
// `changeAt` (a negative value renders the reference: the same instrument, never changed).
// `hardCutAt >= 0` additionally zeroes one sample, which is the negative control.
Render render(Knob knob, float from, float to, long long changeAt, double seconds, long long hardCutAt = -1) {
    StringNetworkParams params = baseParams();
    applyKnob(params, knob, from);

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(2);
    network.setParams(params);
    network.reset();

    NoteEvent on{};
    on.type = NoteEventType::NoteOn;
    on.sampleOffset = 0;
    on.stringIndex = 0;
    on.channel = 0;
    on.midiNote = static_cast<std::uint8_t>(kMidiNote);
    on.velocity = 0.8f;
    on.pluckPosition = 0.28f;
    on.hardness = 0.5f;
    BlockEventQueue events;
    events.push(on);

    Render out;
    const auto total = static_cast<std::size_t>(seconds * kRate);
    out.tap.reserve(total);
    bool changed = false;
    while (out.tap.size() < total) {
        const auto rendered = static_cast<long long>(out.tap.size());
        // THE BLOCK GRID IS ALIGNED TO THE PLACEMENT, not the placement to the block grid. A
        // parameter change is block-granular in a host, and the block grid's phase against the note
        // is arbitrary -- which is exactly what the placement rule forbids the measurement to depend
        // on (tests/support/ClickMetric.h; P2.6 fixes wave 3 measured a 3.1 dB swing across one
        // period from that alone). Rendering a short block up to the chosen sample makes the change
        // land on a phase chosen from the WAVEFORM while still arriving at a block boundary, which is
        // a host whose grid happens to line up there. Both arms chunk identically, so the block
        // structure is not a difference between them.
        int wanted = kBlock;
        if (!changed && changeAt > rendered)
            wanted = static_cast<int>(std::min<long long>(kBlock, changeAt - rendered));
        if (!changed && changeAt >= 0 && rendered == changeAt) {
            StringNetworkParams moved = baseParams();
            applyKnob(moved, knob, to);
            network.setParams(moved);
            changed = true;
            wanted = kBlock;
        } else if (changed) {
            // A HOST REISSUES THE SAME AUTOMATION VALUE EVERY BLOCK, so the gate does too. This is
            // the path a hunting solver would take: re-solving from a moving smoothed state would
            // return a new target on every one of these calls.
            StringNetworkParams held = baseParams();
            applyKnob(held, knob, to);
            network.setParams(held);
        }
        if (changed) {
            out.compensationTrack.push_back(network.bridgeCompensationSamples(0));
            // The target the solve just produced, recovered from the string it was pushed to: after
            // setParams the string's smoother TARGET is the solution, and the value in force is what
            // is gliding onto it. Sampling both is what separates "the glide is smooth" from "the
            // destination stopped moving".
            out.targetTrack.push_back(cnpg::dsp::solveBridgeTuning(network.internalBridgeJunction(), 0,
                                                                   cnpg::test::midiNoteToHz(kMidiNote), kRate, 2)
                                          .phaseDelaySamples);
        }
        network.process(events, wanted);
        const float* a = network.tapBuffers().channel(0, 0);
        const float* b = network.tapBuffers().channel(1, 0);
        for (int n = 0; n < wanted && out.tap.size() < total; ++n) {
            out.tap.push_back((a != nullptr ? a[n] : 0.0f) + (b != nullptr ? b[n] : 0.0f));
            out.string0.push_back(a != nullptr ? a[n] : 0.0f);
        }
    }
    if (hardCutAt >= 0 && hardCutAt < static_cast<long long>(out.tap.size()))
        out.tap[static_cast<std::size_t>(hardCutAt)] = 0.0f;
    out.compensationAtEnd = network.bridgeCompensationSamples(0);
    out.converged = network.bridgeTuningConverged(0);
    return out;
}

// Index of the largest |x| inside [begin, begin + window). See the placement rule.
std::size_t loudestSample(const std::vector<float>& signal, std::size_t begin, int window) {
    std::size_t best = begin;
    double peak = -1.0;
    const std::size_t end = std::min(signal.size(), begin + static_cast<std::size_t>(window));
    for (std::size_t k = begin; k < end; ++k) {
        const double magnitude = std::fabs(static_cast<double>(signal[k]));
        if (magnitude > peak) {
            peak = magnitude;
            best = k;
        }
    }
    return best;
}

} // namespace

TEST_CASE("CONTRACT: the three bridge parameters change under a ringing string without clicks or a hunting pitch",
          "[contract]") {
    // Each knob is moved BETWEEN TWO POINTS OF THE PROVISIONAL NORMAL RANGE, because that is the
    // region the tuning guarantee is declared over and therefore the region a click gate has to
    // cover. Each move is large enough to be a real gesture: the coupling move alone changes the
    // committed compensation by ~1.2 samples of loop length, which is ~4.8 cents of pitch on a
    // ringing note. A gate over a move that changed nothing would be a gate over nothing.
    struct Move {
        Knob knob;
        float from;
        float to;
    };
    const Move moves[] = {
        {Knob::Coupling, 0.0f, 0.35f},
        {Knob::Resonance, 80.0f, 330.0f},
        {Knob::Damping, 1.0f, 0.15f},
    };

    // Long enough that a 2^15-sample analysis window fits entirely clear of the transition, which is
    // what the settled-pitch measurement below needs.
    constexpr double kSeconds = 2.0;
    const int period = fullPeriodSamples();
    // Far enough in that the attack transient is over and the note is a settled decaying tone.
    const auto searchFrom = static_cast<std::size_t>(0.30 * kRate);

    for (const Move& move : moves) {
        // ---- placement, read off the render actually under test ---------------------------------
        // The reference render (no change at all) is the waveform the placement is chosen from, and
        // it is bit-identical to the test render up to the change sample by construction, which is
        // asserted rather than assumed below.
        // Two passes, because the placement has to be read off the render that is actually measured
        // and the render's block grid depends on the placement. Pass one finds the phase on an
        // unaligned render; pass two re-renders the reference with the SAME chunking the test arm
        // will use (a no-op setParams at the same sample), and the placement is confirmed against
        // that. Both arms are then bit-identical up to the change, which is asserted below.
        const Render scout = render(move.knob, move.from, move.from, -1, kSeconds);
        const std::size_t placement = loudestSample(scout.tap, searchFrom, period);
        const std::size_t placementNextPeriod =
            loudestSample(scout.tap, searchFrom + static_cast<std::size_t>(period), period);

        const Render reference = render(move.knob, move.from, move.from, static_cast<long long>(placement), kSeconds);
        const Render test = render(move.knob, move.from, move.to, static_cast<long long>(placement), kSeconds);

        // The placement was read off the same waveform the gate measures: bit-identity over the
        // searched window between the render the search ran on and the arm under test.
        for (std::size_t k = searchFrom; k < placement; ++k)
            REQUIRE(test.tap[k] == reference.tap[k]);

        // ---- the negative control, placed by level on the same waveform -------------------------
        const Render control = render(move.knob, move.from, move.from, static_cast<long long>(placement), kSeconds,
                                      static_cast<long long>(placement));

        const std::size_t spanBegin = placement - 1; // measureClick differences from begin + 1
        const auto spanEnd = static_cast<std::size_t>(kSeconds * kRate);
        const cnpg::test::ClickMeasurement referenceMetric =
            cnpg::test::measureClick(reference.tap, kRate, spanBegin, spanEnd);
        const cnpg::test::ClickMeasurement testMetric = cnpg::test::measureClick(test.tap, kRate, spanBegin, spanEnd);
        const cnpg::test::ClickMeasurement controlMetric =
            cnpg::test::measureClick(control.tap, kRate, spanBegin, spanEnd);
        REQUIRE(referenceMetric.medianAbsDiff > 0.0); // a degenerate reference can gate nothing

        const double excessDb = cnpg::test::clickExcessDb(testMetric, referenceMetric);
        const double controlDb = cnpg::test::clickExcessDb(controlMetric, referenceMetric);

        // ---- CLOSURE ON THE PHASE AXIS: the same gate one period later must agree ----------------
        const Render referenceNextRender =
            render(move.knob, move.from, move.from, static_cast<long long>(placementNextPeriod), kSeconds);
        const Render testNext =
            render(move.knob, move.from, move.to, static_cast<long long>(placementNextPeriod), kSeconds);
        const std::size_t nextBegin = placementNextPeriod - 1;
        const cnpg::test::ClickMeasurement referenceNext =
            cnpg::test::measureClick(referenceNextRender.tap, kRate, nextBegin, spanEnd);
        const cnpg::test::ClickMeasurement testNextMetric =
            cnpg::test::measureClick(testNext.tap, kRate, nextBegin, spanEnd);
        const double excessNextDb = cnpg::test::clickExcessDb(testNextMetric, referenceNext);

        // ---- the pitch transition, from the state that IS the pitch -----------------------------
        const double f0 = cnpg::test::midiNoteToHz(kMidiNote);
        const double periodSamples = kRate / f0;
        const std::vector<double>& track = test.compensationTrack;
        const std::vector<double>& targets = test.targetTrack;
        REQUIRE(track.size() > 40);
        REQUIRE(targets.size() == track.size());

        // Samples of loop length converted to cents of pitch: a loop one sample longer is flatter by
        // 1200*log2(1 + 1/period). Reported in cents so the size of the gesture is legible.
        const auto toCents = [periodSamples](double samples) {
            return -1200.0 * std::log2(1.0 + samples / periodSamples);
        };
        const double startCents = toCents(track.front());
        const double landedCents = toCents(track.back());

        // OVERSHOOT: how far the glide goes BEYOND the interval it travels between. A monotone glide
        // has none by construction; a hunting one does not.
        double overshootCents = 0.0;
        for (double samples : track) {
            const double value = toCents(samples);
            const double lo = std::min(startCents, landedCents);
            const double hi = std::max(startCents, landedCents);
            if (value > hi)
                overshootCents = std::max(overshootCents, value - hi);
            else if (value < lo)
                overshootCents = std::max(overshootCents, lo - value);
        }

        // ...and it must have STOPPED. The last quarter of the track is ~250 ms, i.e. 30 time
        // constants of the 8 ms smoother, and its spread is what says the string is not still being
        // moved around.
        double settledSpread = 0.0;
        {
            const std::size_t tailFrom = track.size() - track.size() / 4;
            double lo = 1.0e300;
            double hi = -1.0e300;
            for (std::size_t k = tailFrom; k < track.size(); ++k) {
                lo = std::min(lo, toCents(track[k]));
                hi = std::max(hi, toCents(track[k]));
            }
            settledSpread = hi - lo;
        }

        // THE DESTINATION NEVER MOVED, which is the mechanism by which a converging solver would
        // hunt: every one of these blocks re-issued the same automation value and re-solved, and the
        // answer has to be the same answer every time. Measured as the spread of the solved target
        // across the whole transition -- exactly 0 is what a solve that reads only the smoother
        // TARGETS produces, and anything else means it is reading a moving state.
        double targetSpreadSamples = 0.0;
        {
            double lo = 1.0e300;
            double hi = -1.0e300;
            for (double value : targets) {
                lo = std::min(lo, value);
                hi = std::max(hi, value);
            }
            targetSpreadSamples = hi - lo;
        }
        INFO(knobName(move.knob) << ": the solved target must not move while the glide runs; spread "
                                 << targetSpreadSamples << " samples over " << targets.size() << " re-solves");
        REQUIRE(targetSpreadSamples == 0.0);

        // ...and the audio really did land in tune afterwards, measured on the render rather than
        // inferred from the loop length. A long window well clear of the transition.
        {
            const auto from = placement + static_cast<std::size_t>(0.40 * kRate);
            REQUIRE(test.string0.size() > from + (std::size_t{1} << 15));
            std::vector<double> settled(test.string0.begin() + static_cast<std::ptrdiff_t>(from), test.string0.end());
            const double measured =
                cnpg::test::findPeakHz(cnpg::test::computeSpectrum(settled, kRate, std::size_t{1} << 15), f0, 80.0);
            REQUIRE(measured > 0.0);
            const double settledCents = cnpg::test::centsBetween(measured, f0);
            INFO(knobName(move.knob) << ": settled pitch " << settledCents << " cents off nominal after the change");
            REQUIRE(std::fabs(settledCents) <= cnpg::dsp::kBridgeTuningGateCents);
        }

        std::cout << "[contract] " << knobName(move.knob) << " " << move.from << " -> " << move.to
                  << " under a ringing string (level-placed at sample " << placement << "): click excess " << excessDb
                  << " dB (limit " << cnpg::test::kClickMetricToleranceDb << "), one period later " << excessNextDb
                  << " dB, level-placed hard-cut control " << controlDb << " dB; pitch track " << startCents << " -> "
                  << landedCents << " cents, overshoot " << overshootCents << " cents, settled spread " << settledSpread
                  << " cents; compensation " << test.compensationAtEnd << " samples\n";

        INFO(knobName(move.knob) << ": click excess " << excessDb << " dB");
        REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
        REQUIRE(testMetric.nonFiniteSamples == 0);
        REQUIRE(testMetric.subnormalSamples == 0);

        // The gate is closed on the phase axis: measured one period later, the two must agree within
        // a margin far smaller than the criterion they are being judged against.
        INFO(knobName(move.knob) << ": closure -- " << excessDb << " dB vs " << excessNextDb << " dB one period later");
        REQUIRE(std::fabs(excessDb - excessNextDb) < 1.0);
        REQUIRE(excessNextDb <= cnpg::test::kClickMetricToleranceDb);

        // The control must FAIL by a wide margin, or the gate above is measuring nothing.
        INFO(knobName(move.knob) << ": hard-cut control " << controlDb << " dB");
        REQUIRE(controlDb > 3.0 * cnpg::test::kClickMetricToleranceDb);

        // NO UNSTABLE PITCH TRANSITION -- the half the click metric cannot see. 0.5 cents is a
        // quarter of the +/-2 cent tuning gate and well under the ~5 cent JND for a slow pitch move.
        INFO(knobName(move.knob) << ": pitch overshoot " << overshootCents << " cents, settled spread " << settledSpread
                                 << " cents");
        REQUIRE(overshootCents < 0.5);
        REQUIRE(settledSpread < 0.5);
        REQUIRE(test.converged);
    }
}

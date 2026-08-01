#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/ClickMetric.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// DamperFeltTimeTests -- Task P2.2's note-off: how long it takes, and what it costs.
//
// Everything here is about the ONE state change a player actually triggers -- the felt coming down
// on a ringing string -- measured three ways, because the P2.1 review's ruling is that a state
// change covered only by a click test is not covered:
//
//   1. The felt time constant is audible: the time a note-off takes to reach silence tracks
//      feltTimeConstantMs monotonically across its whole validated 20..100 ms range.
//   2. DIRECT continuity: the note-off sample is bit-identical to the same render without the
//      note-off, the engagement is asserted to be genuinely mid-ramp where the test claims it is,
//      and the render's own per-sample motion never jumps.
//   3. The click metric, with a companion (tests/support/ClickMetric.h): damper engagement is by
//      construction a level-reducing change and the plain criterion is documented as blind to
//      exactly those, so it is gated alongside the level-normalised reading, at three damper
//      depths, with a negative control per reading -- including a control built to BE the
//      documented blind spot.
//
// The retrigger path's setEngagementImmediate() -- the one place the junction's coefficients are
// allowed to jump -- gets its own direct assertion at the bottom.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RetriggerMode;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr float kPickup = 0.87f;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex = 0) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = 0.8f;
    event.pluckPosition = 0.28f;
    event.hardness = 0.5f;
    return event;
}

NoteEvent noteOff(int sampleOffset, int midiNote, int stringIndex = 0) {
    NoteEvent event = noteOn(sampleOffset, midiNote, stringIndex);
    event.type = NoteEventType::NoteOff;
    event.pluckPosition = cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

StringNetworkParams paramsWith(float feltTimeConstantMs, float damperPosition01 = 0.15f) {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    params.damperPosition01 = damperPosition01;
    params.damper.maxLoss = 1.0f;
    params.damper.feltTimeConstantMs = feltTimeConstantMs;
    params.exciter.noiseAmount = 0.0f;
    // DECOUPLED BRIDGE (Task P2.4), and it is a scoping decision rather than a convenience. Every
    // case in this file is about ONE STRING'S LIFECYCLE -- how fast the felt silences it, when the
    // watchdog clears it, whether a clear opens the damper -- and each states its claim as "the
    // network is now bit-identical to a fresh instance" or "energyEstimate() is exactly 0". Those
    // are claims about the STRING, and P2.4 made them false of the INSTRUMENT: the bridge junction
    // is state too, clearing a string does not clear it, and it goes on re-driving that string out
    // of its own residual. So a re-init on a used network is genuinely no longer bit-identical to a
    // fresh one, and "the string is silent" no longer implies "the network stores nothing".
    //
    // That is a real behavioural change and it is NOT hidden here -- it is asserted on its own
    // terms in "CONTRACT: DamperFeltTime -- a cleared string does not clear the coupled
    // instrument" at the end of this file, and recorded in the P2.4 report as a P2.6 entry
    // condition (a NoteOn now lands on a string that may be ringing sympathetically). What this
    // file keeps measuring, unchanged and at the same tolerances, is the damper.
    params.bridge.couplingStrength = 0.0f;
    return params;
}

void configure(StringNetwork<float>& network, const StringNetworkParams& params) {
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset();
}

// Windowed peak of `samples` over non-overlapping windows of `windowSeconds`: the first window
// whose peak sits below `threshold` is where the note is called silent, and its START is the time
// returned (relative to `beginSample`). A windowed peak rather than an instantaneous level for the
// same reason StringNetwork's own silence watchdog uses one -- a decaying sinusoid crosses zero
// every half period, and an instantaneous test would call every one of those crossings silence.
double secondsUntilBelow(const std::vector<float>& samples, std::size_t beginSample, double threshold,
                         double windowSeconds = 0.010) {
    const auto window = std::max<std::size_t>(1, static_cast<std::size_t>(windowSeconds * kRate));
    // Slid by a tenth of a window rather than stepped by a whole one: the criterion compares two
    // felt settings against each other, and a whole-window hop quantises the answer coarsely
    // enough that adjacent settings tie (measured: 40 ms and 60 ms both landing on exactly 0.030 s
    // with a 10 ms hop). The window still spans several periods; only the reported instant is
    // finer.
    const auto hop = std::max<std::size_t>(1, window / 10);
    for (std::size_t start = beginSample; start + window <= samples.size(); start += hop) {
        float peak = 0.0f;
        for (std::size_t i = start; i < start + window; ++i)
            peak = std::max(peak, std::fabs(samples[i]));
        if (static_cast<double>(peak) < threshold)
            return static_cast<double>(start - beginSample) / kRate;
    }
    return -1.0; // never got there inside the render
}

struct NoteOffRender {
    std::vector<float> tap;
    std::size_t noteOffSample = 0;
    float engagementBefore = -1.0f;
    float engagementOneBlockAfter = -1.0f;
    // Peak over the 50 ms IMMEDIATELY BEFORE the note-off, i.e. how loud the note was at the
    // moment the felt came down -- not the attack peak, which a decaying note has left far behind
    // by then and which would make every level-relative reading a statement about the pluck.
    float levelAtNoteOff = 0.0f;
};

NoteOffRender renderNoteOff(const StringNetworkParams& params, double noteOffSeconds, double totalSeconds,
                            bool releaseIt = true, int midiNote = 45) {
    StringNetwork<float> network;
    configure(network, params);

    NoteOffRender out;
    const auto totalBlocks = static_cast<int>(totalSeconds * kRate / kBlock);
    const auto noteOffBlock = static_cast<int>(noteOffSeconds * kRate / kBlock);
    out.noteOffSample = static_cast<std::size_t>(noteOffBlock) * static_cast<std::size_t>(kBlock);
    out.tap.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));

    BlockEventQueue events;
    events.push(noteOn(0, midiNote));
    for (int b = 0; b < totalBlocks; ++b) {
        if (b == noteOffBlock && releaseIt) {
            out.engagementBefore = network.damperEngagement(0);
            events.push(noteOff(0, midiNote));
        }
        network.process(events, kBlock);
        if (b == noteOffBlock && releaseIt)
            out.engagementOneBlockAfter = network.damperEngagement(0);
        const float* channel = network.tapBuffers().channel(0, 0);
        REQUIRE(channel != nullptr);
        out.tap.insert(out.tap.end(), channel, channel + kBlock);
    }
    const auto levelWindow = std::min(out.noteOffSample, static_cast<std::size_t>(0.050 * kRate));
    for (std::size_t i = out.noteOffSample - levelWindow; i < out.noteOffSample; ++i)
        out.levelAtNoteOff = std::max(out.levelAtNoteOff, std::fabs(out.tap[i]));
    return out;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// the felt time constant is audible
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DamperFeltTime -- a note-off reaches silence in a time set by feltTimeConstantMs", "[contract]") {
    // docs/plan.md Task P2.2 acceptance: "NoteOff->silence envelope reaches -60 dBFS within a
    // window consistent with feltTimeConstantMs at 20 ms and at 100 ms (measured decay time
    // monotone in the parameter, factor >= 3 between the two settings)."
    //
    // Measured at MIDI 69 rather than at the lowest note the instrument has, and the reason is
    // physics rather than convenience: the damper's modal decay rate scales with the string's
    // ROUND-TRIP TIME (T60 ~ 1.73 * T_rt / sin^2(n*pi*p)), while the felt ramp is a fixed number of
    // milliseconds. On a very low string the round trip is long enough that the string's own
    // damped decay, not the ramp, dominates the total, and the ratio between two felt settings
    // would be measuring the string. At 440 Hz the damped decay is ~5 ms and the ramp is what is
    // left -- which is the quantity this criterion is about. The same measurement at MIDI 45 is
    // reported below as a second reading rather than hidden.
    // THE CRITERION'S "factor >= 3" IS NOT REACHABLE, and the reason is arithmetic rather than an
    // implementation shortfall. The brief locks the engagement onto a ONE-POLE ramp
    // e(t) = 1 - exp(-t/tau), and the damping rate a resistive junction applies is proportional to
    // that engagement, so the cumulative attenuation is
    //
    //     C(t) = k * [ t - tau * (1 - exp(-t/tau)) ]           (k = the full-engagement rate, dB/s)
    //
    // Writing u = t/tau and alpha = D/(k*tau) for a target drop of D dB, the time to reach D solves
    // u - 1 + exp(-u) = alpha, and T = u*tau. Scaling tau by 5 (20 ms -> 100 ms) scales alpha by
    // 1/5, so the ratio is 5*u(alpha/5)/u(alpha). That function is monotone DECREASING in alpha and
    // its supremum, as alpha -> 0, is 5/sqrt(5) = sqrt(5) = 2.236 -- because in that limit u ~
    // sqrt(2*alpha) and the time grows with the SQUARE ROOT of the time constant, not with it. At
    // the other end (alpha large) the ratio tends to 1. There is no note, no material, no threshold
    // and no damper depth that reaches 3: sqrt(5) is a ceiling over the whole parameter space.
    //
    // So what is gated here is everything the criterion is actually after -- strict monotonicity
    // across the WHOLE validated 20..100 ms range, on three independent readings -- plus a factor
    // bound set as a fraction of the reachable ceiling rather than of a number that cannot exist.
    // The deviation is recorded in this task's report.
    constexpr double kNoteOffSeconds = 0.3; // released while the note is still ringing steadily
    constexpr double kTotalSeconds = 3.0;
    constexpr double kSilenceDbfs = -60.0;
    const double kOnePoleRatioCeiling = std::sqrt(5.0); // 2.236, derived above
    constexpr double kRequiredRatio = 1.50;             // see below for the two effects that compress it

    const double threshold = std::pow(10.0, kSilenceDbfs / 20.0);

    // Sustain material, for the third time in this repo and for the third time for the same
    // reason (the [tuning] sweep and the P2.1 click gate are the other two): the quantity under
    // test is how fast THE DAMPER takes the note down, and at the default material a string's own
    // decay is a large addend that is identical for every felt setting -- it compresses the ratio
    // between two settings toward 1 without saying anything about either. Same filters, same code
    // path, a parameter value inside the shipping range. The default-material reading is taken
    // below as well rather than hidden, and it is monotone there too.
    auto sustaining = [](StringNetworkParams params) {
        params.stringMaterial.lossGainLow = 1.0f;
        params.stringMaterial.lossGainHigh = 1.0f;
        return params;
    };

    // What -60 dBFS is worth here, stated rather than assumed. This is the sample-domain TAP
    // channel, which is upstream of PickupTap's nominal trim (docs/plan.md Task P1.9 calibrates
    // the per-string level to -18 dBFS at the END of the chain), so a plucked note peaks around
    // -36 dBFS on it and the absolute -60 dBFS threshold is ~24 dB of decay rather than ~60. That
    // is a real span and the criterion is measured on it as written -- but it is not a lot, so the
    // same measurement is repeated below against a -60 dB threshold RELATIVE to each render's own
    // pre-note-off level, which is level-independent and where the factor-of-3 criterion has the
    // whole dynamic range to work in. Both readings are gated.
    constexpr double kMinimumHeadroomDb = 18.0;

    auto measure = [&](float feltMs, int midiNote, bool relative, bool sustain = true) {
        const StringNetworkParams params = sustain ? sustaining(paramsWith(feltMs)) : paramsWith(feltMs);
        const NoteOffRender rendered = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, true, midiNote);
        REQUIRE(rendered.engagementBefore == 0.0f);
        REQUIRE(rendered.engagementOneBlockAfter > 0.0f);
        REQUIRE(rendered.engagementOneBlockAfter < 1.0f);

        const double level = static_cast<double>(rendered.levelAtNoteOff);
        const double target = relative ? level * threshold : threshold;
        // Non-vacuous: the note really was well above the threshold it has to fall through. For the
        // ABSOLUTE reading that is a statement about this signal's headroom over -60 dBFS; for the
        // relative one the headroom is 60 dB by construction and what has to be checked instead is
        // that the note was audible at all when it was released.
        REQUIRE(level > (relative ? threshold : target * std::pow(10.0, kMinimumHeadroomDb / 20.0)));
        const double seconds = secondsUntilBelow(rendered.tap, rendered.noteOffSample, target);
        REQUIRE(seconds > 0.0);
        return seconds;
    };

    const float feltSettings[] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
    for (bool relative : {false, true}) {
        std::vector<double> times;
        for (float felt : feltSettings)
            times.push_back(measure(felt, 69, relative));

        std::cout << "[contract] felt time -> note-off silence (MIDI 69, -60 dB "
                  << (relative ? "relative to the note's own level" : "absolute dBFS") << "):";
        for (std::size_t i = 0; i < times.size(); ++i)
            std::cout << " " << feltSettings[i] << " ms -> " << times[i] << " s;";
        std::cout << " ratio 100/20 = " << (times.back() / times.front()) << " (gate " << kRequiredRatio
                  << ", one-pole ceiling " << kOnePoleRatioCeiling << ")\n";

        // Monotone across the WHOLE validated range, not merely at its two ends: a mapping that is
        // right at 20 and 100 and wrong in between would pass the criterion as literally written.
        for (std::size_t i = 1; i < times.size(); ++i) {
            INFO("felt " << feltSettings[i - 1] << " ms -> " << times[i - 1] << " s, " << feltSettings[i] << " ms -> "
                         << times[i] << " s");
            REQUIRE(times[i] > times[i - 1]);
        }
        INFO("20 ms -> " << times.front() << " s, 100 ms -> " << times.back() << " s");
        REQUIRE(times.back() >= kRequiredRatio * times.front());
        // ...and the derivation above is not a story told about the number: the measured ratio must
        // also sit UNDER the ceiling it predicts. A reading above sqrt(5) would mean the model of
        // what this ramp does is wrong, and that is worth failing on.
        REQUIRE(times.back() <= kOnePoleRatioCeiling * times.front() * 1.02);
    }

    // Second reading: MIDI 45 at the DEFAULT material -- the shipping configuration, where the
    // string's own decay is a large addend that both felt settings pay equally. Still monotone; the
    // factor is not required of this reading, and the ratio it prints is smaller for that stated
    // reason rather than because something is wrong.
    const double defaultFast = measure(20.0f, 45, true, false);
    const double defaultSlow = measure(100.0f, 45, true, false);
    std::cout << "[contract] felt time -> note-off silence (MIDI 45, default material, -60 dB relative): 20 ms -> "
              << defaultFast << " s, 100 ms -> " << defaultSlow << " s; ratio " << (defaultSlow / defaultFast) << "\n";
    REQUIRE(defaultSlow > defaultFast);

    // Third reading: a low string, where the round trip is 4x longer and the damper's own modal
    // decay therefore is too. Monotone as well.
    const double lowFast = measure(20.0f, 45, true);
    const double lowSlow = measure(100.0f, 45, true);
    std::cout << "[contract] felt time -> note-off silence (MIDI 45, -60 dB relative): 20 ms -> " << lowFast
              << " s, 100 ms -> " << lowSlow << " s; ratio " << (lowSlow / lowFast) << "\n";
    REQUIRE(lowSlow > lowFast);
}

TEST_CASE("CONTRACT: DamperFeltTime -- a note-off eventually clears the string completely", "[contract]") {
    // The silence watchdog that replaced P1's release envelope. A damped string has to leave the
    // loop, and with a POINT damper that cannot be a timer: partial 20 sits exactly on the node of
    // the default p = 0.15 junction and rides the loop loss down on its own. So the watchdog waits
    // for an observation, and this pins both halves -- it does happen, and it happens only once
    // the string is genuinely inaudible.
    const NoteOffRender rendered = renderNoteOff(paramsWith(40.0f), 0.3, 3.0);

    StringNetwork<float> network;
    configure(network, paramsWith(40.0f));
    BlockEventQueue events;
    events.push(noteOn(0, 45));
    const auto blocksBefore = static_cast<int>(0.3 * kRate / kBlock);
    for (int b = 0; b < blocksBefore; ++b)
        network.process(events, kBlock);
    REQUIRE(network.tapBuffers().isActive(0));
    REQUIRE(network.energyEstimate() > 0.0);

    BlockEventQueue release;
    release.push(noteOff(0, 45));
    const auto blocksAfter = static_cast<int>(2.7 * kRate / kBlock);
    double clearedAtSeconds = -1.0;
    for (int b = 0; b < blocksAfter; ++b) {
        network.process(release, kBlock);
        if (clearedAtSeconds < 0.0 && network.energyEstimate() == 0.0)
            clearedAtSeconds = static_cast<double>(b) * kBlock / kRate;
    }

    std::cout << "[contract] note-off clears the string after " << clearedAtSeconds << " s (silence watchdog)\n";
    INFO("cleared at " << clearedAtSeconds << " s");
    REQUIRE(clearedAtSeconds > 0.0);
    REQUIRE(network.energyEstimate() == 0.0);
    REQUIRE_FALSE(network.tapBuffers().isActive(0));
    // Snapped, so the next note is plucked onto an undamped string rather than into the felt.
    REQUIRE(network.damperEngagement(0) == 0.0f);

    // The clear happened only after the tail was already inaudible: the watchdog's floor is
    // -100 dBFS and the criterion is a WINDOWED PEAK, so nothing above that floor was truncated.
    const auto clearedSample = rendered.noteOffSample + static_cast<std::size_t>(clearedAtSeconds * kRate);
    if (clearedSample < rendered.tap.size()) {
        float peakAtClear = 0.0f;
        const auto from = clearedSample - std::min(clearedSample, static_cast<std::size_t>(0.010 * kRate));
        for (std::size_t i = from; i < clearedSample; ++i)
            peakAtClear = std::max(peakAtClear, std::fabs(rendered.tap[i]));
        INFO("peak in the 10 ms before the clear " << peakAtClear);
        REQUIRE(static_cast<double>(peakAtClear) < 1.0e-4);
    }
}

// ---------------------------------------------------------------------------------------------
// what the note-off costs: DIRECT continuity first, then both click readings
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DamperFeltTime -- the note-off is continuous, sample by sample", "[contract]") {
    // THE direct assertion the P2.1 review's ruling requires, and it is stronger than any metric:
    // engage() only retargets a one-pole ramp sitting at 0, and DamperJunction advances that ramp
    // AFTER the sample it scatters, so the note-off sample and everything before it must be
    // BIT-IDENTICAL to the same render with no note-off at all, and the divergence afterwards must
    // start at zero and grow smoothly rather than stepping.
    constexpr double kNoteOffSeconds = 0.5;
    constexpr double kTotalSeconds = 1.5;

    StringNetworkParams params = paramsWith(20.0f); // the fastest legal felt, i.e. the worst case
    // Sustain material for the same documented reason the P2.1 click gate and the [tuning] sweep
    // use it: every measurement below is about the signal AT the note-off, and at the default
    // material a low note has decayed by an order of magnitude across the span that sets the
    // reference. Same filters, same code path, a parameter value inside the shipping range.
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;

    const NoteOffRender damped = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, true);
    const NoteOffRender reference = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, false);
    REQUIRE(damped.tap.size() == reference.tap.size());

    // In the state this case claims to test: released from rest, and genuinely mid-ramp one block
    // later rather than already settled.
    REQUIRE(damped.engagementBefore == 0.0f);
    REQUIRE(damped.engagementOneBlockAfter > 0.0f);
    REQUIRE(damped.engagementOneBlockAfter < 1.0f);

    // Bit-identical up to AND INCLUDING the sample the note-off is consumed on.
    for (std::size_t i = 0; i <= damped.noteOffSample; ++i) {
        INFO("sample " << i << " of " << damped.noteOffSample);
        REQUIRE(damped.tap[i] == reference.tap[i]);
    }

    // ...and the two renders really do separate afterwards, so the identity above is a property of
    // the ramp's phasing rather than of a damper that never engaged. The separation does not begin
    // on the very next sample and should not be expected to: the junction sits at p = 0.15 and the
    // tap reads at 0.87, so whatever the junction changes has to TRAVEL the 0.72 of a rail span
    // between them -- ~157 samples at MIDI 45 / 48 kHz -- before the tap can see it. That
    // propagation delay is the waveguide working, and asserting against it would be asserting that
    // the string is instantaneous.
    std::size_t firstDifference = 0;
    for (std::size_t i = damped.noteOffSample + 1; i < damped.tap.size(); ++i) {
        if (damped.tap[i] != reference.tap[i]) {
            firstDifference = i;
            break;
        }
    }
    REQUIRE(firstDifference > damped.noteOffSample);
    REQUIRE(firstDifference < damped.noteOffSample + static_cast<std::size_t>(0.020 * kRate));
    // The divergence BEGINS at essentially nothing rather than as a step: the first sample that
    // differs at all differs by less than a millionth of the signal's peak.
    const double firstDelta = std::fabs(static_cast<double>(damped.tap[firstDifference]) -
                                        static_cast<double>(reference.tap[firstDifference]));
    INFO("first difference at +" << (firstDifference - damped.noteOffSample) << " samples, magnitude " << firstDelta);
    REQUIRE(firstDelta < 1.0e-3 * static_cast<double>(damped.levelAtNoteOff));

    // The divergence grows from nothing. Bounded against the reference's own per-sample motion:
    // over the first 20 ms the damped render may not depart from the undamped one faster than the
    // undamped one moves by itself, which is the sharpest continuity statement available without
    // re-deriving the ramp.
    float referenceStep = 0.0f;
    for (std::size_t i = damped.noteOffSample - 480; i < damped.noteOffSample; ++i)
        referenceStep = std::max(referenceStep, std::fabs(reference.tap[i] - reference.tap[i - 1]));
    REQUIRE(referenceStep > 0.0f);

    float worstDivergenceStep = 0.0f;
    float worstDampedStep = 0.0f;
    const auto until = damped.noteOffSample + static_cast<std::size_t>(0.020 * kRate);
    for (std::size_t i = damped.noteOffSample + 1; i < until; ++i) {
        const float divergence = (damped.tap[i] - reference.tap[i]) - (damped.tap[i - 1] - reference.tap[i - 1]);
        worstDivergenceStep = std::max(worstDivergenceStep, std::fabs(divergence));
        worstDampedStep = std::max(worstDampedStep, std::fabs(damped.tap[i] - damped.tap[i - 1]));
    }

    std::cout << "[contract] note-off continuity: worst step of (damped - reference) " << worstDivergenceStep
              << ", worst step of the damped render " << worstDampedStep << ", reference's own worst step "
              << referenceStep << "\n";
    INFO("divergence step " << worstDivergenceStep << " vs reference step " << referenceStep);
    REQUIRE(worstDivergenceStep < referenceStep);
    // The damped render itself never moves faster than the undamped one did either -- a damper can
    // only ever take motion out.
    REQUIRE(worstDampedStep <= referenceStep);
}

namespace {

struct ClickReadings {
    double excessDb = 0.0;
    double residualExcessDb = 0.0;
    double levelExcessDb = 0.0;
    double hardCutExcessDb = 0.0;
    std::size_t hardCutOffsetSamples = 0; // where the negative control's cut landed, past the note-off
    double hardCutLevel = 0.0;            // ...and the sample value it landed on
    double residualLevelDropDb = 0.0;     // how much level the change took out of the residual span
    long long nonFinite = 0;
    long long subnormal = 0;
};

// Both ClickMetric readings for a damper engagement of depth `maxLoss`, plus the hard-mute
// negative control, over the same span.
ClickReadings measureNoteOffClick(float maxLoss, double postSeconds) {
    constexpr double kNoteOffSeconds = 0.5;
    constexpr double kTotalSeconds = 1.5;
    constexpr double kPreSeconds = 0.25; // ringing before the change, which sets the denominator

    StringNetworkParams params = paramsWith(20.0f); // the fastest legal felt: the worst case
    params.damper.maxLoss = maxLoss;
    // Sustain material for the same documented reason the P2.1 click gate and the [tuning] sweep
    // use it: the denominator has to describe the signal AT the change, and at the default
    // material a low note has decayed by an order of magnitude across the span that sets it. Same
    // filters, same code path, a parameter value inside the shipping range.
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;

    const NoteOffRender damped = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, true);
    const NoteOffRender reference = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, false);
    REQUIRE(damped.engagementBefore == 0.0f);
    REQUIRE(damped.engagementOneBlockAfter > 0.0f);
    REQUIRE(damped.engagementOneBlockAfter < 1.0f);

    const auto spanBegin = damped.noteOffSample - static_cast<std::size_t>(kPreSeconds * kRate);
    const auto spanEnd = damped.noteOffSample + static_cast<std::size_t>(postSeconds * kRate);

    const cnpg::test::ClickMeasurement referenceMeasurement =
        cnpg::test::measureClick(reference.tap, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement dampedMeasurement =
        cnpg::test::measureClick(damped.tap, kRate, spanBegin, spanEnd);
    // The residual span is BOUNDED rather than "to the end of the render", and the reason is the
    // same one ClickMetric.h gives for bounding the analysed span: the residual is meant to
    // describe the ordinary motion of the signal that remains once the change has finished, and a
    // span running to the end of a render lets a second of continued damping accumulate into it
    // until it describes silence instead.
    const auto residualEnd = std::min(damped.tap.size(), spanEnd + static_cast<std::size_t>(0.150 * kRate));
    const cnpg::test::ClickMeasurement dampedResidual =
        cnpg::test::measureClick(damped.tap, kRate, spanEnd, residualEnd);
    const cnpg::test::ClickMeasurement referenceResidual =
        cnpg::test::measureClick(reference.tap, kRate, spanEnd, residualEnd);

    // NEGATIVE CONTROL, so the gate provably has teeth on THIS render rather than only on the one
    // it was calibrated against in P2.1: the P1 behaviour taken to its crude limit, an instant
    // mute. It must fail the same criterion the felt ramp passes.
    //
    // LEVEL-PLACED inside the post-note-off window rather than pinned to the note-off sample itself,
    // which is the P2.2 lesson (tests/dsp/SustainPedalTests.cpp states it in full). A hard cut's
    // peak |dx| IS the sample value it lands on, and the note-off sample is an arbitrary phase of a
    // ringing string -- land it near a zero crossing and the control is a cut of almost nothing,
    // i.e. the control passes the gate it exists to fail and the gate is provably toothless. This
    // one site gates three depths, so a weak placement here would have taken all three with it.
    std::size_t cutSample = damped.noteOffSample;
    float cutLevel = 0.0f;
    for (std::size_t i = damped.noteOffSample; i < std::min(spanEnd, reference.tap.size()); ++i) {
        if (std::fabs(reference.tap[i]) > cutLevel) {
            cutLevel = std::fabs(reference.tap[i]);
            cutSample = i;
        }
    }
    REQUIRE(cutLevel > 0.0f);
    std::vector<float> hardCut = reference.tap;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
    const cnpg::test::ClickMeasurement hardCutMeasurement =
        cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd);

    REQUIRE(referenceMeasurement.metric(referenceMeasurement) > 0.0);

    // The LEVEL-NORMALISED reading (tests/support/ClickMetric.h), measured over the POST-CHANGE
    // window only -- which is load-bearing: a span that also contains the loud pre-change signal
    // normalises both renders by the same untouched level and collapses back into clickExcessDb.
    const cnpg::test::ClickMeasurement dampedAfter =
        cnpg::test::measureClick(damped.tap, kRate, damped.noteOffSample, spanEnd);
    const cnpg::test::ClickMeasurement referenceAfter =
        cnpg::test::measureClick(reference.tap, kRate, damped.noteOffSample, spanEnd);
    ClickReadings out;
    out.excessDb = cnpg::test::clickExcessDb(dampedMeasurement, referenceMeasurement);
    out.residualExcessDb = cnpg::test::clickExcessAgainstResidualDb(dampedMeasurement, dampedResidual,
                                                                    referenceMeasurement, referenceResidual);
    out.levelExcessDb = cnpg::test::clickExcessAgainstLevelDb(dampedAfter, referenceAfter);
    out.hardCutExcessDb = cnpg::test::clickExcessDb(hardCutMeasurement, referenceMeasurement);
    out.hardCutOffsetSamples = cutSample - damped.noteOffSample;
    out.hardCutLevel = static_cast<double>(cutLevel);
    out.residualLevelDropDb = (referenceResidual.medianAbsDiff > 0.0 && dampedResidual.medianAbsDiff > 0.0)
                                  ? 20.0 * std::log10(dampedResidual.medianAbsDiff / referenceResidual.medianAbsDiff)
                                  : -300.0;
    out.nonFinite = dampedMeasurement.nonFiniteSamples;
    out.subnormal = dampedMeasurement.subnormalSamples;
    return out;
}

} // namespace

TEST_CASE("CONTRACT: DamperFeltTime -- damper engagement passes the click metric at every depth", "[contract]") {
    // The P2.1 review's binding ruling: a level-reducing state change may not lean on
    // clickExcessDb() alone, because that reading compares ABSOLUTE peak first differences and "a
    // state change that BOTH reduces the level AND inserts a discontinuity can pass". Damper
    // engagement reduces level by construction, so it needs a scale-carrying companion.
    //
    // WHICH companion, and why it is not the one the ruling names. clickExcessAgainstResidualDb()
    // normalises each render by its own SETTLED signal after the change, which presumes the change
    // moves the note from one level to another. Damper engagement does not: it changes the note's
    // DECAY RATE, so the signal falls continuously from the moment of the change and there is no
    // settled level to normalise by -- every window after the change is quieter than the one
    // before it. Measured, that reading returns the accumulated level drop and nothing else: 23 dB
    // at maxLoss 0.05 and 52 dB at 1.0, both of them the damper working exactly as intended. It is
    // computed and printed at every depth below so the number is on the record, and the reading
    // that is GATED alongside it is clickExcessAgainstLevelDb() over the POST-CHANGE window -- peak
    // |dx| against that window's own peak |x|, per 10 ms window, which is the one denominator a
    // progressive level change divides out of exactly. Both are documented in
    // tests/support/ClickMetric.h.
    //
    // WHERE THAT COMPANION STOPS DISCRIMINATING, measured rather than assumed. |dx| is a
    // frequency-weighted quantity -- a band-limited waveform's peak|dx|/peak|x| is about
    // 2*pi*f_max/fs -- and a point damper does not attenuate uniformly: it annihilates the partials
    // with antinodes at p and leaves the ones with nodes there, which at p = 0.15 are partial 20 and
    // up. So a fully engaged damper leaves a residue that is genuinely BRIGHTER than the note was,
    // and the companion reads that brightness exactly as it would read a step. At the palm-mute
    // depths the low partials are still present and it is sharp; at full depth it saturates on the
    // residue, so the full note-off is gated by reading (b), by the structural bound on the
    // coefficient's per-sample step (tests/dsp/DamperEnergyTests.cpp) and by the sample-by-sample
    // continuity case above. The next case is what shows the companion has teeth where it is
    // relied upon.
    struct Depth {
        float maxLoss;
        const char* name;
        bool companionDiscriminates;
    };
    const Depth depths[] = {
        {0.05f, "light palm mute", true}, {0.25f, "palm mute", true}, {1.0f, "full note-off", false}};

    for (const Depth& depth : depths) {
        const ClickReadings readings = measureNoteOffClick(depth.maxLoss, 0.12);

        std::cout << "[contract] damper engagement click metric, " << depth.name << " (maxLoss " << depth.maxLoss
                  << "): absolute excess " << readings.excessDb << " dB, LEVEL-normalised excess "
                  << readings.levelExcessDb << " dB (limit " << cnpg::test::kClickMetricToleranceDb
                  << " dB); the change took " << readings.residualLevelDropDb
                  << " dB out of the residual span, which is what the residual-normalised reading reports ("
                  << readings.residualExcessDb << " dB, not a click); level-placed hard-mute negative control "
                  << readings.hardCutExcessDb << " dB at sample +" << readings.hardCutOffsetSamples << " (level "
                  << readings.hardCutLevel << ")\n";

        INFO(depth.name << ": absolute " << readings.excessDb << " dB, level " << readings.levelExcessDb
                        << " dB; hard-mute control " << readings.hardCutExcessDb << " dB");

        // The scenario really is level-reducing, which is the whole premise of needing a companion.
        REQUIRE(readings.residualLevelDropDb < -3.0);

        // NEGATIVE CONTROL for reading (b): an instant mute at the note-off must fail it.
        REQUIRE(readings.hardCutExcessDb > cnpg::test::kClickMetricToleranceDb);

        // READING (b) ITSELF. Sharper than the criterion, and true because a passive junction can
        // only ever take motion out: the damped render's peak |dx| does not merely fail to grow by
        // 3 dB, it does not grow at all.
        REQUIRE(readings.excessDb <= cnpg::test::kClickMetricToleranceDb);
        REQUIRE(readings.excessDb <= 0.0);
        REQUIRE(readings.nonFinite == 0);
        REQUIRE(readings.subnormal == 0);

        if (depth.companionDiscriminates)
            REQUIRE(readings.levelExcessDb <= cnpg::test::kClickMetricToleranceDb);
        else
            REQUIRE(readings.levelExcessDb > cnpg::test::kClickMetricToleranceDb); // saturated; see above
    }
}

TEST_CASE("CONTRACT: DamperFeltTime -- the level-normalised reading catches a step reading (b) is blind to",
          "[contract]") {
    // THE CASE THAT MAKES THE COMPANION BINDING, and the one the first version of this suite got
    // wrong. tests/support/ClickMetric.h states reading (b)'s failure mode exactly: "A state change
    // that BOTH reduces the level AND inserts a discontinuity can pass." The first attempt at a
    // control put a 6 dB step 50 ms after the note-off -- where the damper had barely started, so
    // the step was worth half of a still-loud signal and reading (b) CAUGHT it at 7-10 dB. A
    // control that the reading under test already catches demonstrates nothing about the reading
    // that is supposed to catch it, and the companion was unbinding as a result.
    //
    // So the step is placed by LEVEL rather than by time: at the first moment the damper has taken
    // the note kBlindSpotDropDb below where it was at the note-off. Halving the signal there is a
    // discontinuity worth 6 dB of everything still audible -- plainly a click to a listener -- and
    // an absolute |dx| far under what the undamped string was doing before the note-off, which is
    // the denominator reading (b) compares against. Both halves are asserted: (b) misses it, the
    // companion catches it, and the un-stepped render passes both so the span itself is fair.
    constexpr float kMaxLoss = 0.25f; // palm-mute depth, where the companion discriminates
    constexpr double kBlindSpotDropDb = -30.0;
    constexpr double kNoteOffSeconds = 0.5;
    constexpr double kTotalSeconds = 1.5;
    constexpr double kPreSeconds = 0.25;

    StringNetworkParams params = paramsWith(20.0f);
    params.damper.maxLoss = kMaxLoss;
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;

    const NoteOffRender damped = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, true);
    const NoteOffRender reference = renderNoteOff(params, kNoteOffSeconds, kTotalSeconds, false);
    REQUIRE(damped.engagementBefore == 0.0f);
    REQUIRE(damped.engagementOneBlockAfter > 0.0f);
    REQUIRE(damped.engagementOneBlockAfter < 1.0f);

    // Where the damper has taken the note kBlindSpotDropDb down, measured on a 10 ms windowed peak
    // so a zero crossing cannot be mistaken for the level.
    const double target = static_cast<double>(damped.levelAtNoteOff) * std::pow(10.0, kBlindSpotDropDb / 20.0);
    const double secondsToTarget = secondsUntilBelow(damped.tap, damped.noteOffSample, target);
    REQUIRE(secondsToTarget > 0.0);
    const auto stepAt = damped.noteOffSample + static_cast<std::size_t>(secondsToTarget * kRate);

    const auto spanBegin = damped.noteOffSample - static_cast<std::size_t>(kPreSeconds * kRate);
    const auto spanEnd = std::min(damped.tap.size(), stepAt + static_cast<std::size_t>(0.050 * kRate));
    REQUIRE(spanEnd > stepAt + 16);

    std::vector<float> stepped = damped.tap;
    for (std::size_t i = stepAt; i < stepped.size(); ++i)
        stepped[i] *= 0.5f;

    const cnpg::test::ClickMeasurement referenceMeasurement =
        cnpg::test::measureClick(reference.tap, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement dampedMeasurement =
        cnpg::test::measureClick(damped.tap, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement steppedMeasurement =
        cnpg::test::measureClick(stepped, kRate, spanBegin, spanEnd);
    REQUIRE(referenceMeasurement.metric(referenceMeasurement) > 0.0);

    // The level-normalised reading is taken over the POST-CHANGE window only, as ClickMetric.h
    // requires -- a span carrying the loud pre-change signal takes its maximum from a window where
    // both renders are identical.
    const cnpg::test::ClickMeasurement referenceAfter =
        cnpg::test::measureClick(reference.tap, kRate, damped.noteOffSample, spanEnd);
    const cnpg::test::ClickMeasurement dampedAfter =
        cnpg::test::measureClick(damped.tap, kRate, damped.noteOffSample, spanEnd);
    const cnpg::test::ClickMeasurement steppedAfter =
        cnpg::test::measureClick(stepped, kRate, damped.noteOffSample, spanEnd);

    const double dampedExcessDb = cnpg::test::clickExcessDb(dampedMeasurement, referenceMeasurement);
    const double steppedExcessDb = cnpg::test::clickExcessDb(steppedMeasurement, referenceMeasurement);
    const double dampedLevelDb = cnpg::test::clickExcessAgainstLevelDb(dampedAfter, referenceAfter);
    const double steppedLevelDb = cnpg::test::clickExcessAgainstLevelDb(steppedAfter, referenceAfter);

    std::cout << "[contract] blind-spot demonstration (maxLoss " << kMaxLoss << ", 6 dB step at " << secondsToTarget
              << " s after the note-off, where the damper has taken " << kBlindSpotDropDb
              << " dB out): reading (b) damped " << dampedExcessDb << " dB -> stepped " << steppedExcessDb
              << " dB (MISSED, limit " << cnpg::test::kClickMetricToleranceDb << "); level-normalised damped "
              << dampedLevelDb << " dB -> stepped " << steppedLevelDb << " dB (CAUGHT)\n";

    INFO("(b): " << dampedExcessDb << " -> " << steppedExcessDb << " dB; level: " << dampedLevelDb << " -> "
                 << steppedLevelDb << " dB");

    // The span is fair: the un-stepped damper passes BOTH readings over exactly this span, so a
    // failure below is the step and not the window.
    REQUIRE(dampedExcessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(dampedLevelDb <= cnpg::test::kClickMetricToleranceDb);

    // THE BLIND SPOT, measured: reading (b) does not merely pass the step, it barely notices it.
    REQUIRE(steppedExcessDb <= cnpg::test::kClickMetricToleranceDb);

    // THE COMPANION EARNING ITS PLACE: the same step, on the same render, over the same span.
    REQUIRE(steppedLevelDb > cnpg::test::kClickMetricToleranceDb);
    REQUIRE(steppedLevelDb > dampedLevelDb + 10.0);
}

// ---------------------------------------------------------------------------------------------
// the one place the coefficients are allowed to jump
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DamperFeltTime -- a retrigger snaps the damper open over cleared state only", "[contract]") {
    // setEngagementImmediate() IS a discontinuity in the junction's scattering coefficients, and
    // it is safe for exactly one reason: StringNetwork calls it only where the string's rails are
    // being zeroed in the same breath. Both halves are asserted directly -- that the snap happens
    // (otherwise the next note would be plucked into a fully engaged felt and arrive dead), and
    // that the state it happens over is cleared (otherwise it is a click).
    //
    // -----------------------------------------------------------------------------------------
    // P2.6 SPLIT THIS IN TWO. FIXES WAVE 2 REJOINED IT, and the invariant never moved.
    // -----------------------------------------------------------------------------------------
    // Through P2.3 every pitch-changing retrigger took the re-init path, so the snap was reached
    // from there. P2.6 made RetriggerMode real and had the two modes take OPPOSITE paths over a
    // string mid-release -- Physical ramping the damper open over kept rails, Synth fading to zero
    // and then snapping -- and this case asserted that disagreement from both sides.
    //
    // Wave 2's ownership ruling removes the disagreement rather than re-tuning it: a released note
    // is over, so "owns a note" is `sounding_` alone and a re-strike on a merely RELEASING string
    // is a fresh note in either mode. Both modes therefore snap here, and both snap over rails
    // clearStringState() zeroes on the same call -- which is the invariant this case has always
    // been about, now with no exception to it.
    //
    // WHERE PHYSICAL GENUINELY KEEPS THE RAILS is a re-strike on a still-SOUNDING string, and the
    // second section asserts what the damper does there: nothing, because the felt was never down.
    // That is what makes "the snap happens only over cleared state" a complete statement instead of
    // half of one -- the other path has no engagement to snap in the first place, and a build that
    // added one (the refused damper choke) would fail that section.

    auto ringThenReleaseHalfway = [](StringNetwork<float>& network) {
        BlockEventQueue events;
        events.push(noteOn(0, 45));
        for (int b = 0; b < 200; ++b)
            network.process(events, kBlock);
        REQUIRE(network.damperEngagement(0) == 0.0f);

        BlockEventQueue release;
        release.push(noteOff(0, 45));
        for (int b = 0; b < 20; ++b) // ~53 ms into a 100 ms ramp
            network.process(release, kBlock);

        // Genuinely MID-RAMP, asserted rather than assumed -- this is the P2.1 trap, where a state
        // change placed outside the window under test passed against the defect bit-identically.
        const float midRamp = network.damperEngagement(0);
        INFO("engagement at the retrigger " << midRamp);
        REQUIRE(midRamp > 0.05f);
        REQUIRE(midRamp < 0.95f);
        REQUIRE(network.energyEstimate() > 0.0); // ...and the string is still ringing under it
        return midRamp;
    };

    SECTION("mid-release, in EITHER mode: the snap happens, and the state it happens over is cleared") {
        for (const RetriggerMode mode : {RetriggerMode::Physical, RetriggerMode::Synth}) {
            StringNetworkParams params = paramsWith(100.0f); // slowest felt, so the ramp is easy to catch
            params.retriggerMode = mode;
            const bool physical = mode == RetriggerMode::Physical;
            INFO("retrigger mode " << (physical ? "Physical" : "Synth"));

            StringNetwork<float> network;
            configure(network, params);
            const float midRamp = ringThenReleaseHalfway(network);

            BlockEventQueue retrigger;
            retrigger.push(noteOn(0, 52));
            std::vector<float> revived;
            for (int b = 0; b < 60; ++b) {
                network.process(retrigger, kBlock);
                if (b == 0) {
                    // SNAPPED, not ramped -- and snapped on the retrigger sample itself, not after
                    // a fade: there is no live note to fade out, so no fade is started in either
                    // mode. One block is 2.67 ms against a 100 ms felt, so a ramped engagement
                    // would still be within a few percent of midRamp here.
                    INFO("engagement one block after the retrigger, was " << midRamp);
                    REQUIRE_FALSE(network.retriggerFadeActive(0));
                    REQUIRE(network.retriggerFadeGain(0) == 1.0f);
                    REQUIRE(network.damperEngagement(0) == 0.0f);
                    REQUIRE(network.damperLossDepth(0) == params.damper.maxLoss); // the depth snapped too
                    REQUIRE(network.retuneRampSamplesRemaining(0) == 0);          // and no retune ramp
                }
                const float* channel = network.tapBuffers().channel(0, 0);
                revived.insert(revived.end(), channel, channel + kBlock);
            }

            // THE STATE THE SNAP HAPPENED OVER REALLY WAS CLEARED, in the strongest form available:
            // from the retrigger sample onward the render is BIT-IDENTICAL to the same note plucked
            // on a string that never rang and was never damped. Nothing of the damped tail and no
            // residual engagement survived into it. Note there is no offset to allow for now -- the
            // 2 ms of Synth-fade latency this comparison used to carry is gone, because the fade
            // does not run over a string that owns no note.
            StringNetwork<float> fresh;
            configure(fresh, params);
            BlockEventQueue freshEvents;
            freshEvents.push(noteOn(0, 52));
            std::vector<float> expected;
            for (int b = 0; b < 60; ++b) {
                fresh.process(freshEvents, kBlock);
                const float* channel = fresh.tapBuffers().channel(0, 0);
                expected.insert(expected.end(), channel, channel + kBlock);
            }

            float peak = 0.0f;
            for (float value : revived)
                peak = std::max(peak, std::fabs(value));
            REQUIRE(peak > 0.001f); // non-vacuous: the retriggered note really sounds
            REQUIRE(revived.size() == expected.size());
            REQUIRE(revived == expected);
        }
    }

    SECTION("over a SOUNDING string, Physical keeps the rails and there is no engagement to snap") {
        // The other path, and the reason the invariant is complete. A re-strike on a string whose
        // note the player is still holding takes the Physical retrigger path: rails kept, f0 glided
        // on the retune ramp, nothing cleared. The damper is 0 before the re-strike and 0 after it,
        // because only a note-off engages it and a note-off is what makes the string stop sounding.
        // So there is no snap here and no ramp-open either -- the release() call on that path is a
        // no-op, and this is what says so.
        //
        // A build that added the plan's refused "damper choke (fast engage())" to this path would
        // fail the second assertion below, which makes this the cheapest standing guard against the
        // refusal being quietly undone.
        StringNetworkParams params = paramsWith(100.0f);
        params.retriggerMode = RetriggerMode::Physical;
        StringNetwork<float> network;
        configure(network, params);

        BlockEventQueue events;
        events.push(noteOn(0, 45));
        for (int b = 0; b < 200; ++b)
            network.process(events, kBlock);
        REQUIRE(network.damperEngagement(0) == 0.0f);
        const double energyBefore = network.energyEstimate();
        REQUIRE(energyBefore > 0.0);

        BlockEventQueue retrigger;
        retrigger.push(noteOn(0, 52));
        network.process(retrigger, kBlock);

        REQUIRE(network.damperEngagement(0) == 0.0f);       // nothing engaged it, nothing snapped it
        REQUIRE(network.energyEstimate() > 0.0);            // the rails were kept
        REQUIRE(network.retuneRampSamplesRemaining(0) > 0); // ...and the pitch is gliding, not jumping
        REQUIRE_FALSE(network.retriggerFadeActive(0));
    }
}

TEST_CASE("CONTRACT: DamperFeltTime -- every path that clears a string also opens its damper", "[contract]") {
    // setEngagementImmediate() has FOUR call sites, not two, and all four are reachable with the
    // felt ramp genuinely mid-flight: the pitch-changing retrigger and the silence watchdog (both
    // covered above), plus these two -- the enable ramp landing on silence, and reset(). Both are
    // reached by ordinary play: disable a string, or turn the string count down, while a note-off
    // tail is still being damped.
    //
    // Each is asserted the same way: the ramp is confirmed mid-flight FIRST (otherwise the case
    // passes against a damper that was never engaged), then the clear is confirmed to have opened
    // it, then a fresh note on that string is confirmed bit-identical to the same note on a string
    // with no history -- which is what says the snap happened over cleared state and not over a
    // live waveform.
    auto ringThenReleaseHalfway = [](StringNetwork<float>& network) {
        BlockEventQueue events;
        events.push(noteOn(0, 45));
        for (int b = 0; b < 200; ++b)
            network.process(events, kBlock);
        REQUIRE(network.damperEngagement(0) == 0.0f);

        BlockEventQueue release;
        release.push(noteOff(0, 45));
        for (int b = 0; b < 20; ++b) // ~53 ms into a 100 ms ramp
            network.process(release, kBlock);

        const float midRamp = network.damperEngagement(0);
        INFO("engagement mid-ramp " << midRamp);
        REQUIRE(midRamp > 0.05f);
        REQUIRE(midRamp < 0.95f);
        REQUIRE(network.energyEstimate() > 0.0); // ...with the string still ringing under it
    };

    SECTION("the enable ramp landing on silence") {
        StringNetworkParams params = paramsWith(100.0f);
        StringNetwork<float> network;
        configure(network, params);
        ringThenReleaseHalfway(network);

        // Disable the string mid-ramp. Its enable ramp is 10 ms, so a few blocks take it to
        // exactly 0, and the loop's clear fires there.
        params.perString[0].enabled = false;
        network.setParams(params);
        BlockEventQueue idle;
        for (int b = 0; b < 10; ++b)
            network.process(idle, kBlock);

        REQUIRE(network.energyEstimate() == 0.0);                     // the clear really happened...
        REQUIRE(network.damperEngagement(0) == 0.0f);                 // ...and it opened the damper
        REQUIRE(network.damperLossDepth(0) == params.damper.maxLoss); // ...and snapped the depth

        // Re-enable and pluck: bit-identical to a string that never rang, so nothing of the damped
        // tail and no residual engagement survived the clear.
        params.perString[0].enabled = true;
        network.setParams(params);
        BlockEventQueue replucked;
        replucked.push(noteOn(0, 52));
        const std::vector<float> revived = [&] {
            std::vector<float> out;
            for (int b = 0; b < 40; ++b) {
                network.process(replucked, kBlock);
                const float* channel = network.tapBuffers().channel(0, 0);
                out.insert(out.end(), channel, channel + kBlock);
            }
            return out;
        }();

        StringNetwork<float> fresh;
        configure(fresh, paramsWith(100.0f));
        BlockEventQueue freshEvents;
        freshEvents.push(noteOn(0, 52));
        std::vector<float> expected;
        for (int b = 0; b < 40; ++b) {
            fresh.process(freshEvents, kBlock);
            const float* channel = fresh.tapBuffers().channel(0, 0);
            expected.insert(expected.end(), channel, channel + kBlock);
        }

        float peak = 0.0f;
        for (float value : revived)
            peak = std::max(peak, std::fabs(value));
        REQUIRE(peak > 0.001f);
        for (std::size_t i = 0; i < revived.size(); ++i) {
            INFO("sample " << i);
            REQUIRE(revived[i] == expected[i]);
        }
    }

    SECTION("a setNumStrings reduction taking the string out") {
        // The same clear, reached the other way the enable ramp can be driven. Two strings so the
        // count can come down to 1 while string 1 is still being damped.
        StringNetworkParams params = paramsWith(100.0f);
        StringNetwork<float> network;
        network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(2);
        network.setParams(params);
        network.reset();

        BlockEventQueue events;
        events.push(noteOn(0, 45, 1));
        for (int b = 0; b < 200; ++b)
            network.process(events, kBlock);
        BlockEventQueue release;
        release.push(noteOff(0, 45, 1));
        for (int b = 0; b < 20; ++b)
            network.process(release, kBlock);

        const float midRamp = network.damperEngagement(1);
        INFO("engagement mid-ramp " << midRamp);
        REQUIRE(midRamp > 0.05f);
        REQUIRE(midRamp < 0.95f);

        network.setNumStrings(1);
        BlockEventQueue idle;
        for (int b = 0; b < 10; ++b)
            network.process(idle, kBlock);
        REQUIRE(network.tapBuffers().numStrings() == 1); // the removed string really has left
        REQUIRE(network.energyEstimate() == 0.0);
        REQUIRE(network.damperEngagement(1) == 0.0f);
    }

    SECTION("reset() while the felt is coming down") {
        StringNetwork<float> network;
        configure(network, paramsWith(100.0f));
        ringThenReleaseHalfway(network);

        network.reset();
        REQUIRE(network.damperEngagement(0) == 0.0f);
        REQUIRE(network.damperLossDepth(0) == paramsWith(100.0f).damper.maxLoss);
        REQUIRE(network.energyEstimate() == 0.0);
    }
}

TEST_CASE("CONTRACT: DamperFeltTime -- a same-pitch pluck over a ringing string leaves the damper alone",
          "[contract]") {
    // The other retrigger branch: the string keeps its state, so the damper must NOT be snapped
    // there. In P2.2 its engagement is always already 0 on this path (only a NoteOff engages it,
    // and a NoteOff clears `sounding_`, which this branch requires) -- which is exactly why the
    // code calls release() rather than setEngagementImmediate(). Pinned so that P2.6, which adds
    // paths that pluck over a damped string, cannot quietly turn it into a snap.
    StringNetwork<float> network;
    configure(network, paramsWith(40.0f));

    BlockEventQueue events;
    events.push(noteOn(0, 45));
    for (int b = 0; b < 200; ++b)
        network.process(events, kBlock);

    BlockEventQueue retrigger;
    retrigger.push(noteOn(0, 45));
    network.process(retrigger, kBlock);
    REQUIRE(network.damperEngagement(0) == 0.0f);
    REQUIRE(network.energyEstimate() > 0.0); // the ringing state was kept, not cleared
}

// ---------------------------------------------------------------------------------------------
// what Task P2.4 changed about "cleared" -- stated here because this file's cases stopped saying it
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DamperFeltTime -- a cleared string does not clear the coupled instrument", "[contract]") {
    // THE BEHAVIOUR CHANGE THE DECOUPLING IN paramsWith() MOVED OUT OF THE WAY, asserted here on
    // its own terms so nothing is lost by that scoping decision.
    //
    // Every lifecycle case above states its claim as "energyEstimate() is exactly 0" or "the render
    // is bit-identical to a fresh instance". Both were true of the whole network while the strings
    // were uncoupled, because clearing the last ringing string cleared everything there was. From
    // Task P2.4 they are true of the STRING and false of the INSTRUMENT: the bridge junction is a
    // mass and a spring, it holds energy that no string clear touches, and it goes on re-driving
    // the strings out of its own residual afterwards.
    //
    // That is not a defect and it is not free. It is a P2.6 ENTRY CONDITION: a NoteOn can now land
    // on a string that is ringing sympathetically with no note of its own, and the re-init path
    // clears it -- which through P2.3 was inaudible by construction (the string really was silent)
    // and now is a small truncation whose level this case measures.
    StringNetworkParams params = paramsWith(40.0f);
    params.bridge.couplingStrength = StringNetworkParams{}.bridge.couplingStrength;
    REQUIRE(params.bridge.couplingStrength > 0.0f);

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(2);
    network.setParams(params);
    network.reset();

    BlockEventQueue events;
    events.push(noteOn(0, 45, 0)); // string 0 only; string 1 rings sympathetically
    for (int b = 0; b < 300; ++b)
        network.process(events, kBlock);

    // IN the state this case claims to test: string 1 really is ringing without ever having been
    // played, which is the situation P2.6's retrigger semantics will have to have an answer for.
    const double sympatheticEnergy = network.stringEnergyEstimate(1);
    REQUIRE(sympatheticEnergy > 0.0);

    // Release string 0 and let the watchdog clear it. The INSTRUMENT is not silent afterwards.
    BlockEventQueue release;
    release.push(noteOff(0, 45, 0));
    for (int b = 0; b < 200; ++b)
        network.process(release, kBlock);

    const double stringZero = network.stringEnergyEstimate(0);
    const double stringOne = network.stringEnergyEstimate(1);
    const double total = network.energyEstimate();
    std::cout << "[contract] a cleared string does not clear the instrument: after string 0's note-off and clear, "
              << "string 0 holds " << stringZero << ", string 1 (never played) holds " << stringOne
              << ", the whole network " << total << "\n";

    // The claim, both halves: the network as a whole still stores energy, and the string nobody
    // played is where it is. A test that only asked "is the total non-zero" would be satisfied by
    // string 0 failing to clear, which is the opposite finding.
    REQUIRE(total > 0.0);
    REQUIRE(stringOne > 0.0);
    REQUIRE(stringOne > stringZero);
}

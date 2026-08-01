#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/AllocationGuard.h"
#include "support/ClickMetric.h"
#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// StringNetworkScaleTests -- Task P2.1: the network at N = 1..8 strings.
//
// tests/dsp/StringNetworkTests.cpp keeps the P1.5 single-string contract (event offsets, retrigger,
// release, bit-identity against an isolated WaveguideString). This file owns everything that only
// becomes meaningful once more than one string runs: the active count, the per-string enable ramp,
// per-string tuning offsets, and the (string, tap) shape of the sample->block domain boundary.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr float kPickup = 0.87f;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex, float velocity = 0.8f) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = velocity;
    event.pluckPosition = 0.28f;
    event.hardness = 0.5f;
    return event;
}

StringNetworkParams defaultParams() {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    // DECOUPLED BRIDGE (Task P2.4) -- same scoping decision as tests/dsp/StringNetworkTests.cpp.
    // This file is the N = 1..8 SCALE-OUT's contracts: the trip count, the deferred reduction, the
    // enable ramp, per-string parameters. "The readmitted string arrives silent" is a statement
    // about the count change, and under bidirectional coupling a silent string is exactly what the
    // bridge rings. Cases that DO want the coupling set it explicitly.
    params.bridge.couplingStrength = 0.0f;
    return params;
}

// prepare -> setNumStrings -> setParams -> reset: the documented order (see StringNetworkTests.cpp)
// so every smoother and every enable gain starts snapped onto its target rather than gliding in.
void configure(StringNetwork<float>& network, const StringNetworkParams& params, int numStrings) {
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(numStrings);
    network.setParams(params);
    network.reset();
}

// The mono sum PickupTap would see: every channel the block reports active, summed. Measuring the
// click metric here rather than through the full P1 chain keeps the number about StringNetwork --
// the oversampler's halfbands and the limiter would both smear a discontinuity before it could be
// measured, which is the opposite of what a click gate wants.
float sumActiveTaps(const StringNetwork<float>& network, int sampleIndex) {
    const auto& taps = network.tapBuffers();
    float sum = 0.0f;
    for (int s = 0; s < taps.numStrings(); ++s) {
        if (!taps.isActive(s))
            continue;
        const float* channel = taps.channel(s, 0);
        if (channel != nullptr)
            sum += channel[sampleIndex];
    }
    return sum;
}

float peakOf(const std::vector<float>& samples, std::size_t begin, std::size_t end) {
    float peak = 0.0f;
    for (std::size_t i = begin; i < std::min(end, samples.size()); ++i)
        peak = std::max(peak, std::fabs(samples[i]));
    return peak;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// the enable ramp
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork ramps a disabled string silent without a click", "[contract]") {
    // docs/plan.md Task P2.1 acceptance criterion 3: "Toggling `enabled` on a ringing string ramps
    // it silent with no click (click metric and criterion as defined in this task's steps)".
    //
    // The reference is the identical render with the toggle never applied, and both are measured
    // over the same span: 0.5 s of ringing before the toggle (which is what sets the denominator)
    // plus 60 ms after it -- long enough to contain the whole ramp and any truncation click at its
    // end, short enough that the trailing silence does not collapse the median. See
    // tests/support/ClickMetric.h for why the span is the caller's job.
    constexpr int kRingBlocks = 400;      // ~1.07 s before the toggle
    constexpr int kTailBlocks = 200;      // ~0.53 s after it
    constexpr double kPreSeconds = 0.25;  // ringing before the change sets the denominator
    constexpr double kPostSeconds = 0.06; // ramp (10 ms) plus margin for a truncation click

    auto render = [](bool disableIt) {
        StringNetworkParams params = defaultParams();
        // Sustain material, for the same reason the [tuning] sweep uses it: the denominator is the
        // signal's ordinary per-sample motion, and it has to describe the signal AT THE TOGGLE. At
        // the default material a low note has decayed by an order of magnitude across a
        // quarter-second span, so the median would describe the loud part and a genuine
        // discontinuity in the quiet part could hide underneath it. Same filters, same code path,
        // a parameter value inside the shipping range.
        params.stringMaterial.lossGainLow = 1.0f;
        params.stringMaterial.lossGainHigh = 1.0f;
        StringNetwork<float> network;
        configure(network, params, 1);

        BlockEventQueue events;
        events.push(noteOn(0, 45, 0));

        std::vector<float> out;
        out.reserve(static_cast<std::size_t>(kRingBlocks + kTailBlocks) * static_cast<std::size_t>(kBlock));
        for (int b = 0; b < kRingBlocks + kTailBlocks; ++b) {
            if (b == kRingBlocks && disableIt) {
                params.perString[0].enabled = false;
                network.setParams(params);
            }
            network.process(events, kBlock);
            for (int n = 0; n < kBlock; ++n)
                out.push_back(sumActiveTaps(network, n));
        }
        return out;
    };

    const std::vector<float> muted = render(true);
    const std::vector<float> reference = render(false);
    REQUIRE(muted.size() == reference.size());

    const auto toggleSample = static_cast<std::size_t>(kRingBlocks) * static_cast<std::size_t>(kBlock);
    const auto spanBegin = toggleSample - static_cast<std::size_t>(kPreSeconds * kRate);
    const auto spanEnd = toggleSample + static_cast<std::size_t>(kPostSeconds * kRate);

    // Non-vacuous on both sides: the string really was ringing, and it really did go silent.
    REQUIRE(peakOf(reference, spanBegin, toggleSample) > 0.001f);
    REQUIRE(peakOf(muted, spanEnd, muted.size()) == 0.0f);
    REQUIRE(peakOf(reference, spanEnd, reference.size()) > 0.0f);

    const cnpg::test::ClickMeasurement referenceMeasurement =
        cnpg::test::measureClick(reference, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement mutedMeasurement = cnpg::test::measureClick(muted, kRate, spanBegin, spanEnd);
    const double excessDb = cnpg::test::clickExcessDb(mutedMeasurement, referenceMeasurement);

    // NEGATIVE CONTROL, and the reason this gate can be trusted. The pre-P2.1 behaviour was a HARD
    // CUT: a disabled string's channel went to exact zeros on the sample the parameter changed.
    // That is modelled exactly by zeroing the reference render from the toggle onward, and it must
    // FAIL the same criterion the ramp passes -- otherwise the criterion is measuring nothing.
    std::vector<float> hardCut = reference;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(toggleSample), hardCut.end(), 0.0f);
    const cnpg::test::ClickMeasurement hardCutMeasurement =
        cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd);
    const double hardCutExcessDb = cnpg::test::clickExcessDb(hardCutMeasurement, referenceMeasurement);

    std::cout << "[contract] enable-toggle click metric: ramp " << mutedMeasurement.metric(referenceMeasurement)
              << " vs reference " << referenceMeasurement.metric(referenceMeasurement) << " -> excess " << excessDb
              << " dB (limit " << cnpg::test::kClickMetricToleranceDb << " dB); hard-cut negative control "
              << hardCutMeasurement.metric(referenceMeasurement) << " -> excess " << hardCutExcessDb << " dB\n";

    REQUIRE(referenceMeasurement.metric(referenceMeasurement) > 0.0);
    INFO("ramp excess " << excessDb << " dB, hard-cut control " << hardCutExcessDb << " dB");
    REQUIRE(hardCutExcessDb > cnpg::test::kClickMetricToleranceDb); // the gate has teeth...
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);       // ...and the ramp passes it
    REQUIRE(mutedMeasurement.nonFiniteSamples == 0);
    REQUIRE(mutedMeasurement.subnormalSamples == 0);
}

TEST_CASE("CONTRACT: StringNetwork setNumStrings reduction ramps the removed string out first", "[contract]") {
    // docs/plan.md Task P2.1 acceptance criterion 4: "`setNumStrings` reduction while ringing: the
    // removed string ramps silent through its enable-ramp before the trip count drops on a later
    // block; the click metric criterion holds across the count change; a count increase starts the
    // new string silent immediately."
    //
    // The trip count is observable as tapBuffers().numStrings(): that is the number of channels the
    // block-domain consumer is handed, and dropping it in the same block that requests the
    // reduction IS the click -- PickupTap would stop summing a still-ringing string mid-decay.
    constexpr int kRingBlocks = 400;
    constexpr int kTailBlocks = 200;
    constexpr double kPreSeconds = 0.5;
    constexpr double kPostSeconds = 0.06;
    constexpr int kStrings = 3;

    int tripCountRightAfter = -1;
    int tripCountLater = -1;

    auto render = [&](bool reduceIt) {
        StringNetworkParams params = defaultParams();
        StringNetwork<float> network;
        configure(network, params, kStrings);

        BlockEventQueue events;
        for (int s = 0; s < kStrings; ++s)
            events.push(noteOn(0, 40 + 5 * s, s));

        std::vector<float> out;
        out.reserve(static_cast<std::size_t>(kRingBlocks + kTailBlocks) * static_cast<std::size_t>(kBlock));
        for (int b = 0; b < kRingBlocks + kTailBlocks; ++b) {
            if (b == kRingBlocks && reduceIt)
                network.setNumStrings(kStrings - 1);
            network.process(events, kBlock);
            if (reduceIt && b == kRingBlocks)
                tripCountRightAfter = network.tapBuffers().numStrings();
            if (reduceIt && b == kRingBlocks + kTailBlocks - 1)
                tripCountLater = network.tapBuffers().numStrings();
            for (int n = 0; n < kBlock; ++n)
                out.push_back(sumActiveTaps(network, n));
        }
        return out;
    };

    const std::vector<float> reduced = render(true);
    const std::vector<float> reference = render(false);

    // The removed string is still in the trip count on the block that requested the reduction...
    REQUIRE(tripCountRightAfter == kStrings);
    // ...and has left it by the end of the tail.
    REQUIRE(tripCountLater == kStrings - 1);

    const auto changeSample = static_cast<std::size_t>(kRingBlocks) * static_cast<std::size_t>(kBlock);
    const auto spanBegin = changeSample - static_cast<std::size_t>(kPreSeconds * kRate);
    const auto spanEnd = changeSample + static_cast<std::size_t>(kPostSeconds * kRate);

    REQUIRE(peakOf(reference, spanBegin, changeSample) > 0.001f);

    const cnpg::test::ClickMeasurement referenceMeasurement =
        cnpg::test::measureClick(reference, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement reducedMeasurement =
        cnpg::test::measureClick(reduced, kRate, spanBegin, spanEnd);
    const double excessDb = cnpg::test::clickExcessDb(reducedMeasurement, referenceMeasurement);

    std::cout << "[contract] setNumStrings-reduction click metric: reduced "
              << reducedMeasurement.metric(referenceMeasurement) << ", reference "
              << referenceMeasurement.metric(referenceMeasurement) << " -> excess " << excessDb << " dB (limit "
              << cnpg::test::kClickMetricToleranceDb << " dB)\n";

    REQUIRE(referenceMeasurement.metric(referenceMeasurement) > 0.0);
    INFO("reduction excess " << excessDb << " dB");
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(reducedMeasurement.nonFiniteSamples == 0);
    REQUIRE(reducedMeasurement.subnormalSamples == 0);
}

TEST_CASE("CONTRACT: StringNetwork setNumStrings increase starts the new string silent", "[contract]") {
    // The other half of acceptance criterion 4. An increase is immediate -- the new string is in the
    // trip count on the very next block -- and it arrives silent, because its state was cleared
    // rather than left holding whatever it was ringing the last time the count included it.
    StringNetworkParams params = defaultParams();
    StringNetwork<float> network;
    configure(network, params, 3);

    // Ring all three, then drop to two and let string 2's ramp complete and clear it.
    BlockEventQueue events;
    for (int s = 0; s < 3; ++s)
        events.push(noteOn(0, 40 + 5 * s, s));
    for (int b = 0; b < 200; ++b)
        network.process(events, kBlock);
    network.setNumStrings(2);
    for (int b = 0; b < 200; ++b)
        network.process(events, kBlock);
    REQUIRE(network.tapBuffers().numStrings() == 2);

    // Back up to three. Immediate...
    network.setNumStrings(3);
    REQUIRE(network.numStrings() == 3);
    BlockEventQueue idle;
    network.process(idle, kBlock);
    REQUIRE(network.tapBuffers().numStrings() == 3);

    // ...and silent: no leftover tail from the string's previous life, and no fade-in either -- the
    // very first sample of a note plucked on it is at full amplitude.
    REQUIRE_FALSE(network.tapBuffers().isActive(2));
    for (int n = 0; n < kBlock; ++n)
        REQUIRE(network.tapBuffers().channel(2, 0)[n] == 0.0f);

    BlockEventQueue replucked;
    replucked.push(noteOn(0, 50, 2));
    std::vector<float> revived;
    for (int b = 0; b < 40; ++b) {
        network.process(replucked, kBlock);
        const float* channel = network.tapBuffers().channel(2, 0);
        revived.insert(revived.end(), channel, channel + kBlock);
    }

    StringNetwork<float> fresh;
    configure(fresh, params, 3);
    BlockEventQueue freshEvents;
    freshEvents.push(noteOn(0, 50, 2));
    std::vector<float> reference;
    for (int b = 0; b < 40; ++b) {
        fresh.process(freshEvents, kBlock);
        const float* channel = fresh.tapBuffers().channel(2, 0);
        reference.insert(reference.end(), channel, channel + kBlock);
    }

    REQUIRE(peakOf(revived, 0, revived.size()) > 0.001f);
    // Bit-identical to the same note on a string that was never removed at all: "starts silent"
    // means the readmitted string carries no history whatsoever, not merely that it is quiet.
    for (std::size_t i = 0; i < revived.size(); ++i)
        REQUIRE(revived[i] == reference[i]);
}

// ---------------------------------------------------------------------------------------------
// the active count, 1..8
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork runs every string count from 1 to 8", "[contract]") {
    // docs/plan.md Task P2.1 acceptance criterion 1: "setNumStrings(n) for every n in 1..8: a
    // contract test triggers a note per string and observes n active tap channels and silence on
    // disabled ones".
    for (int n = 1; n <= cnpg::dsp::kMaxStrings; ++n) {
        StringNetworkParams params = defaultParams();
        // Every OTHER string disabled, so "silence on disabled ones" is exercised at every count
        // rather than only at the one where it happens to be convenient.
        for (int s = 1; s < n; s += 2)
            params.perString[static_cast<std::size_t>(s)].enabled = false;

        StringNetwork<float> network;
        configure(network, params, n);
        REQUIRE(network.numStrings() == n);

        // One note per string, all in the same block, each on its own pitch.
        BlockEventQueue events;
        for (int s = 0; s < n; ++s)
            events.push(noteOn(0, 40 + 3 * s, s));

        std::vector<std::vector<float>> perString(static_cast<std::size_t>(n));
        for (int b = 0; b < 40; ++b) {
            network.process(events, kBlock);
            const auto& taps = network.tapBuffers();
            REQUIRE(taps.numStrings() == n);
            REQUIRE(taps.numTaps() == 1);
            for (int s = 0; s < n; ++s) {
                const float* channel = taps.channel(s, 0);
                REQUIRE(channel != nullptr);
                perString[static_cast<std::size_t>(s)].insert(perString[static_cast<std::size_t>(s)].end(), channel,
                                                              channel + kBlock);
            }
            // Nothing beyond the count is addressable at all.
            REQUIRE(taps.channel(n, 0) == nullptr);
        }

        const auto& taps = network.tapBuffers();
        for (int s = 0; s < n; ++s) {
            const bool enabled = params.perString[static_cast<std::size_t>(s)].enabled;
            INFO("count " << n << ", string " << s << (enabled ? " (enabled)" : " (disabled)"));
            REQUIRE(taps.isActive(s) == enabled);
            const float peak =
                peakOf(perString[static_cast<std::size_t>(s)], 0, perString[static_cast<std::size_t>(s)].size());
            if (enabled)
                REQUIRE(peak > 0.001f); // it really sounds
            else
                REQUIRE(peak == 0.0f); // and a disabled one really is silent
        }
    }
}

TEST_CASE("CONTRACT: StringNetwork scale-out allocates nothing after prepare", "[contract]") {
    // Acceptance criterion 1, second half: "no allocation after prepare()". The run deliberately
    // exercises everything P2.1 added on the realtime path -- count increases and reductions (which
    // must never allocate, since every string is preallocated), enable toggles and their ramps, a
    // tap-arity change, a moving pickup and a moving bend -- on top of the P1.5 event/release path.
    constexpr int kBlocks = 1000;

    StringNetworkParams params = defaultParams();
    StringNetwork<float> network;
    configure(network, params, cnpg::dsp::kMaxStrings);

    BlockEventQueue events;

    cnpg::test::resetAllocationCount();
    for (int b = 0; b < kBlocks; ++b) {
        if ((b % 17) == 0)
            events.push(noteOn(b % kBlock, 40 + (b % 24), b % cnpg::dsp::kMaxStrings));
        if ((b % 61) == 0)
            network.setNumStrings(1 + (b / 61) % cnpg::dsp::kMaxStrings);
        if ((b % 97) == 0)
            network.setNumTapsPerString(1 + (b / 97) % cnpg::dsp::kMaxTapsPerString);

        const auto t = static_cast<float>(b) * static_cast<float>(kBlock) / static_cast<float>(kRate);
        params.pitchBendSemitones = 2.0f * std::sin(6.2831853f * 2.0f * t);
        params.pickupPosition01 = 0.5f + 0.4f * std::sin(6.2831853f * 0.7f * t);
        params.perString[3].enabled = ((b / 40) % 2) == 0;
        network.setParams(params);
        network.process(events, kBlock);
        (void)network.tapBuffers().channel(0, 0);
        (void)network.bridgeOutputBuffer();
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// ---------------------------------------------------------------------------------------------
// per-string tuning offsets
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: a per-string tuning offset detunes exactly one string of a unison pair", "[tuning]") {
    // docs/plan.md Task P2.1 acceptance criterion 2: "Per-string tuningOffsetCents = +25 on one
    // string of a unison pair produces a measured f0 ratio within +/-2 cents of 25 cents (FFT phase
    // method, 48 kHz)".
    //
    // Measured with the section-4.5 estimator (tests/support/SpectralAnalysis.h): 2^18 analysis
    // samples starting 0.5 s after the pluck, Blackman-Harris window, x4 zero pad, parabolic peak
    // refinement, peak search restricted to +/-80 cents around each string's OWN nominal so the
    // detuned string cannot be measured against its neighbour's fundamental.
    //
    // MEASURED WITH THE BRIDGE DECOUPLED, and that is a statement about what the criterion means
    // rather than a convenience (Task P2.4). "String 1's f0" is a well-posed quantity only while
    // the strings are independent oscillators. Once the bridge couples them they are ONE system
    // with two normal modes, both of which appear in BOTH taps, and the estimator -- which returns
    // the strongest peak in its window -- then reports the same frequency for both channels
    // whatever the per-string offset is. Measured at the shipping default that is exactly what
    // happens: both taps read 111.30 Hz for a pair nominally at 110.00 and 111.60. Nothing is
    // broken there; the criterion's question has simply stopped being about one string. The
    // coupled behaviour is measured on its own terms in tests/dsp/CoupledStringsTests.cpp (beating,
    // sympathetic response, two-stage decay) and reported at the end of this case.
    constexpr int kMidiNote = 45;
    constexpr float kOffsetCents = 25.0f;
    constexpr double kToleranceCents = 2.0;
    constexpr double kRenderSeconds = 7.0;
    constexpr double kDiscardSeconds = 0.5;
    const std::size_t kAnalysisLength = std::size_t{1} << 18;

    StringNetworkParams params = defaultParams();
    // Sustain material, exactly as the P1 [tuning] sweep uses and for the same documented reason:
    // the estimator analyses 5.5 s starting half a second after the pluck, and at the default
    // material there would be nothing left in that window to measure. Same filters, same code path.
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;
    params.exciter.noiseAmount = 0.0f;
    params.perString[1].tuningOffsetCents = kOffsetCents;
    params.bridge.couplingStrength = 0.0f; // see the note above

    StringNetwork<float> network;
    configure(network, params, 2);

    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote, 0));
    events.push(noteOn(0, kMidiNote, 1));

    std::vector<double> plain;
    std::vector<double> detuned;
    plain.reserve(kAnalysisLength);
    detuned.reserve(kAnalysisLength);

    const auto totalSamples = static_cast<std::size_t>(kRenderSeconds * kRate);
    const auto discard = static_cast<std::size_t>(kDiscardSeconds * kRate);
    std::size_t rendered = 0;
    while (rendered < totalSamples && plain.size() < kAnalysisLength) {
        network.process(events, kBlock);
        const float* a = network.tapBuffers().channel(0, 0);
        const float* b = network.tapBuffers().channel(1, 0);
        REQUIRE(a != nullptr);
        REQUIRE(b != nullptr);
        for (int n = 0; n < kBlock; ++n, ++rendered) {
            if (rendered < discard || plain.size() >= kAnalysisLength)
                continue;
            plain.push_back(static_cast<double>(a[n]));
            detuned.push_back(static_cast<double>(b[n]));
        }
    }
    REQUIRE(plain.size() == kAnalysisLength);

    const double nominalHz = cnpg::test::midiNoteToHz(kMidiNote);
    const double detunedNominalHz = nominalHz * std::exp2(static_cast<double>(kOffsetCents) / 1200.0);

    const cnpg::test::Spectrum plainSpectrum = cnpg::test::computeSpectrum(plain, kRate, kAnalysisLength);
    const cnpg::test::Spectrum detunedSpectrum = cnpg::test::computeSpectrum(detuned, kRate, kAnalysisLength);
    const double plainHz = cnpg::test::findPeakHz(plainSpectrum, nominalHz, cnpg::test::kTuningSearchCents);
    const double detunedHz = cnpg::test::findPeakHz(detunedSpectrum, detunedNominalHz, cnpg::test::kTuningSearchCents);
    REQUIRE(plainHz > 0.0);
    REQUIRE(detunedHz > 0.0);

    const double measuredOffsetCents = cnpg::test::centsBetween(detunedHz, plainHz);
    std::cout << "[tuning] per-string offset: string 0 " << plainHz << " Hz, string 1 " << detunedHz << " Hz -> "
              << measuredOffsetCents << " cents (target " << kOffsetCents << ", tolerance +/-" << kToleranceCents
              << ")\n";

    INFO("measured " << measuredOffsetCents << " cents against a " << kOffsetCents << " cent offset");
    REQUIRE(std::fabs(measuredOffsetCents - static_cast<double>(kOffsetCents)) <= kToleranceCents);

    // The offset is PER STRING, not a global retune: string 0 must still be exactly in tune. Held
    // to the same +/-2 cents the P1 [tuning] gate holds the whole range to.
    const double plainErrorCents = cnpg::test::centsBetween(plainHz, nominalHz);
    INFO("unoffset string measured " << plainErrorCents << " cents from nominal");
    REQUIRE(std::fabs(plainErrorCents) <= kToleranceCents);

    // REPORTED, not gated: the same render at the shipping coupling, so the note at the top of this
    // case is a measurement rather than an assertion about something nobody looked at.
    StringNetworkParams coupledParams = params;
    coupledParams.bridge.couplingStrength = StringNetworkParams{}.bridge.couplingStrength;
    StringNetwork<float> coupled;
    configure(coupled, coupledParams, 2);
    BlockEventQueue coupledEvents;
    coupledEvents.push(noteOn(0, kMidiNote, 0));
    coupledEvents.push(noteOn(0, kMidiNote, 1));
    std::vector<double> coupledA;
    std::vector<double> coupledB;
    std::size_t coupledRendered = 0;
    while (coupledRendered < totalSamples && coupledA.size() < kAnalysisLength) {
        coupled.process(coupledEvents, kBlock);
        const float* a = coupled.tapBuffers().channel(0, 0);
        const float* b = coupled.tapBuffers().channel(1, 0);
        for (int n = 0; n < kBlock; ++n, ++coupledRendered) {
            if (coupledRendered < discard || coupledA.size() >= kAnalysisLength)
                continue;
            coupledA.push_back(static_cast<double>(a[n]));
            coupledB.push_back(static_cast<double>(b[n]));
        }
    }
    const double coupledAHz = cnpg::test::findPeakHz(cnpg::test::computeSpectrum(coupledA, kRate, kAnalysisLength),
                                                     nominalHz, cnpg::test::kTuningSearchCents);
    const double coupledBHz = cnpg::test::findPeakHz(cnpg::test::computeSpectrum(coupledB, kRate, kAnalysisLength),
                                                     detunedNominalHz, cnpg::test::kTuningSearchCents);
    std::cout << "[tuning] the same pair at the shipping couplingStrength " << coupledParams.bridge.couplingStrength
              << ": string 0 tap peaks at " << coupledAHz << " Hz, string 1 tap at " << coupledBHz << " Hz (nominals "
              << nominalHz << " / " << detunedNominalHz
              << ") -- one coupled system with shared normal modes, which is why the gate above runs decoupled\n";
    REQUIRE(coupledAHz > 0.0);
    REQUIRE(coupledBHz > 0.0);
}

// ---------------------------------------------------------------------------------------------
// the (string, tap) domain boundary (ADR 0004 D1)
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork's tap boundary is addressed by (string, tap)", "[contract]") {
    // ADR 0004 D1: the sample->block boundary carries kMaxTapsPerString preallocated spatial taps
    // per string with exactly ONE active, mirroring the kMaxStrings / setNumStrings(1) pattern.
    constexpr int kStrings = 3;
    StringNetwork<float> network;
    configure(network, defaultParams(), kStrings);

    REQUIRE(cnpg::dsp::kMaxTapsPerString == 4);
    REQUIRE(network.numTapsPerString() == 1); // exactly one active at this task's exit

    BlockEventQueue events;
    for (int s = 0; s < kStrings; ++s)
        events.push(noteOn(0, 40 + 5 * s, s));
    for (int b = 0; b < 20; ++b)
        network.process(events, kBlock);

    {
        const auto& taps = network.tapBuffers();
        REQUIRE(taps.numTaps() == 1);
        REQUIRE(taps.channel(0, -1) == nullptr);
        REQUIRE(taps.channel(0, 1) == nullptr); // not active yet, so not addressable
        REQUIRE(taps.channel(0, cnpg::dsp::kMaxTapsPerString) == nullptr);
    }

    // Turn the second tap on. Storage was preallocated for it, so this never allocates.
    cnpg::test::resetAllocationCount();
    network.setNumTapsPerString(2);
    REQUIRE(cnpg::test::allocationCount() == 0);
    REQUIRE(network.numTapsPerString() == 2);

    BlockEventQueue idle;
    network.process(idle, kBlock);
    const auto& taps = network.tapBuffers();
    REQUIRE(taps.numTaps() == 2);

    for (int s = 0; s < kStrings; ++s) {
        const float* tap0 = taps.channel(s, 0);
        const float* tap1 = taps.channel(s, 1);
        REQUIRE(tap0 != nullptr);
        REQUIRE(tap1 != nullptr);

        // Distinct storage, adjacent, one prepared block apart -- (string, tap, sample), laid out
        // string-major so a string's own taps are neighbours.
        REQUIRE(tap1 != tap0);
        REQUIRE(tap1 - tap0 == kBlock);

        // ...carrying identical samples, because every tap of a string currently reads the same
        // smoothed position. That is exactly the claim P2.1 makes: the arity is real storage and a
        // real per-tap read, and it is not yet a second COIL -- spacing, aperture and polarity are
        // what will make tap 1 differ, and they belong to the task that adds them.
        for (int n = 0; n < kBlock; ++n)
            REQUIRE(tap1[n] == tap0[n]);
    }

    bool sounded = false;
    for (int n = 0; n < kBlock; ++n)
        sounded |= (taps.channel(0, 0)[n] != 0.0f);
    REQUIRE(sounded); // non-vacuous: the strings really were ringing through all of this
}

TEST_CASE("CONTRACT: StringNetwork keeps one position smoother per (string, tap)", "[contract]") {
    // ADR 0004 D1/amendment A2: the single global pickupSmoothed_ is replaced by one smoother per
    // (string, tap). Every slot currently carries the same target, so the split moves no output
    // sample -- which means the only way to SEE that they are genuinely independent is to put two
    // of them in different states. A count increase does exactly that: the string being added is
    // snapped onto the current target (it has no history to glide from) while the strings already
    // running are still mid-glide.
    StringNetworkParams params = defaultParams();
    params.pickupPosition01 = 0.2f;

    StringNetwork<float> network;
    configure(network, params, 2);
    network.setNumTapsPerString(2);
    REQUIRE(network.tapPosition01(0, 0) == 0.2f);
    REQUIRE(network.tapPosition01(0, 1) == 0.2f);

    // Retarget and glide part of the way there -- one block is 128 samples against an 8 ms (384
    // sample) smoother, so this lands well short of the target.
    params.pickupPosition01 = 0.9f;
    network.setParams(params);
    BlockEventQueue idle;
    network.process(idle, kBlock);

    const float gliding = network.tapPosition01(0, 0);
    REQUIRE(gliding > 0.2f);
    REQUIRE(gliding < 0.9f);
    REQUIRE(network.tapPosition01(1, 0) == gliding); // strings started together, so they track
    REQUIRE(network.tapPosition01(0, 1) == gliding); // and so do a string's own taps

    // Now add a string mid-glide. Its smoothers are its own, and they start ON the target.
    network.setNumStrings(3);
    REQUIRE(network.tapPosition01(2, 0) == 0.9f);
    REQUIRE(network.tapPosition01(2, 1) == 0.9f);
    REQUIRE(network.tapPosition01(0, 0) == gliding); // ...and the others were not disturbed
    REQUIRE(network.tapPosition01(1, 0) == gliding);

    REQUIRE(network.tapPosition01(-1, 0) == 0.0f);
    REQUIRE(network.tapPosition01(0, cnpg::dsp::kMaxTapsPerString) == 0.0f);
}

// ---------------------------------------------------------------------------------------------
// reserved seams
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PerString envelopeScale is reserved and inert", "[contract]") {
    // ADR 0004 D2 / amendment A4: the per-string envelope-scaling scalar is declared now because the
    // Envelope module's Physical mode drives LOOP LOSS, which lives in the SHARED stringMaterial
    // set -- so without a per-string scalar one string's note-on would alter every other ringing
    // string's decay. Until that module exists the field must not move one sample, and this pins it.
    REQUIRE(StringNetworkParams{}.perString[0].envelopeScale == 1.0f);

    auto render = [](float scale) {
        StringNetworkParams params = defaultParams();
        for (auto& perString : params.perString)
            perString.envelopeScale = scale;

        StringNetwork<float> network;
        configure(network, params, 2);
        BlockEventQueue events;
        events.push(noteOn(0, 45, 0));
        events.push(noteOn(37, 52, 1));

        std::vector<float> out;
        for (int b = 0; b < 60; ++b) {
            network.process(events, kBlock);
            for (int n = 0; n < kBlock; ++n)
                out.push_back(sumActiveTaps(network, n));
        }
        return out;
    };

    const std::vector<float> unity = render(1.0f);
    REQUIRE(peakOf(unity, 0, unity.size()) > 0.001f);
    for (float scale : {0.0f, 0.25f, 4.0f}) {
        const std::vector<float> scaled = render(scale);
        INFO("envelopeScale " << scale);
        REQUIRE(scaled.size() == unity.size());
        for (std::size_t i = 0; i < scaled.size(); ++i)
            REQUIRE(scaled[i] == unity[i]);
    }
}

TEST_CASE("CONTRACT: StringNetwork drops a SILENT string from the trip count immediately", "[contract]") {
    // The enable ramp exists to protect a ringing tail. A string with no tail has nothing to
    // protect, so it is snapped rather than ramped in either direction -- 0 * silence and 1 *
    // silence are the same silence, and holding a string that has nothing to say in the loop for
    // ten more milliseconds buys nobody anything. This is the counterpart to the ramped reduction
    // above, and the two together are the whole of the reduction contract.
    StringNetworkParams params = defaultParams();
    StringNetwork<float> network;
    configure(network, params, cnpg::dsp::kMaxStrings);

    BlockEventQueue idle;
    network.process(idle, kBlock);
    REQUIRE(network.tapBuffers().numStrings() == cnpg::dsp::kMaxStrings);

    // Nothing has ever been plucked, so the count falls on the very next block, not eight
    // milliseconds later.
    network.setNumStrings(2);
    network.process(idle, kBlock);
    REQUIRE(network.tapBuffers().numStrings() == 2);

    // The same, one string at a time, through `enabled` rather than the count: a silent string
    // switched off is off at once, and switched back on is on at once and at full amplitude.
    params.perString[1].enabled = false;
    network.setParams(params);
    network.process(idle, kBlock);
    REQUIRE_FALSE(network.tapBuffers().isActive(1));

    params.perString[1].enabled = true;
    network.setParams(params);
    BlockEventQueue events;
    events.push(noteOn(0, 45, 1));
    std::vector<float> revived;
    for (int b = 0; b < 20; ++b) {
        network.process(events, kBlock);
        const float* channel = network.tapBuffers().channel(1, 0);
        revived.insert(revived.end(), channel, channel + kBlock);
    }

    StringNetwork<float> fresh;
    configure(fresh, defaultParams(), 2);
    BlockEventQueue freshEvents;
    freshEvents.push(noteOn(0, 45, 1));
    std::vector<float> reference;
    for (int b = 0; b < 20; ++b) {
        fresh.process(freshEvents, kBlock);
        const float* channel = fresh.tapBuffers().channel(1, 0);
        reference.insert(reference.end(), channel, channel + kBlock);
    }

    REQUIRE(peakOf(revived, 0, revived.size()) > 0.001f);
    // Bit-identical: no residual ramp attenuating the attack of the first note after the toggle.
    for (std::size_t i = 0; i < revived.size(); ++i)
        REQUIRE(revived[i] == reference[i]);
}

TEST_CASE("CONTRACT: StringNetwork re-increasing the count mid-ramp-out does not jump the tap", "[contract]") {
    // "Outside the count" does not imply "carries no state". A reduction deliberately leaves its
    // removed strings RINGING for the length of their enable ramp -- that deferral is the whole
    // reason the trip count lags -- so an increase arriving INSIDE that window readmits strings
    // that are still sounding, and their position smoothers must not be snapped onto the target.
    // Snapping one jumps the tap read by however far the smoother still had to glide, on a string
    // with a live waveform under the tap: a discontinuity in readTapAt(), and the exact defect
    // class the smoother exists to prevent.
    //
    // The timing here is load-bearing, and was got wrong once while writing this: the enable ramp
    // is 10 ms, which at 48 kHz is 480 samples, so a re-increase four 128-sample blocks later (512
    // samples) lands AFTER the ramp has completed and cleared the strings -- at which point
    // snapping is correct, and this test would have passed against the defect it exists to catch.
    // Two blocks (256 samples) is inside the window, and the assertions below pin that rather than
    // assuming it.
    constexpr int kStrings = 5;
    constexpr int kRingBlocks = 400;    // ~1.07 s of ringing before anything moves
    constexpr int kReduceBlock = 402;   // pickup retargeted at 400, count cut at 402
    constexpr int kReIncreaseBlock = 2; // 256 samples later: inside the 480-sample enable ramp
    constexpr int kTailBlocks = 200;
    constexpr double kPreSeconds = 0.25;
    constexpr double kPostSeconds = 0.06;

    // The pickup is swept across the same window on purpose: with a stationary pickup the snap is
    // a no-op and none of this would discriminate.
    auto configureSwept = [](StringNetwork<float>& network) {
        StringNetworkParams params = defaultParams();
        params.stringMaterial.lossGainLow = 1.0f;  // sustain, so the click metric's denominator
        params.stringMaterial.lossGainHigh = 1.0f; // describes the signal at the change
        params.pickupPosition01 = 0.2f;
        configure(network, params, kStrings);
        return params;
    };

    auto pluckAll = [](BlockEventQueue& events) {
        for (int s = 0; s < kStrings; ++s)
            events.push(noteOn(0, 40 + 3 * s, s));
    };

    // ---- the direct observation: does the readmitted string's tap read jump? ------------------
    {
        StringNetwork<float> network;
        StringNetworkParams params = configureSwept(network);
        BlockEventQueue events;
        pluckAll(events);

        for (int b = 0; b < kReduceBlock; ++b) {
            if (b == kRingBlocks) {
                params.pickupPosition01 = 0.9f; // start the glide
                network.setParams(params);
            }
            network.process(events, kBlock);
        }
        network.setNumStrings(2);
        for (int b = 0; b < kReIncreaseBlock; ++b)
            network.process(events, kBlock);

        // The scenario is live, asserted rather than assumed: the removed strings are still in the
        // trip count, still ringing, and the position smoother is genuinely mid-glide.
        REQUIRE(network.tapBuffers().numStrings() == kStrings);
        REQUIRE(network.tapBuffers().isActive(kStrings - 1));
        const float glidingBefore = network.tapPosition01(0, 0);
        REQUIRE(glidingBefore > 0.2f);
        REQUIRE(glidingBefore < 0.9f);
        REQUIRE(network.tapPosition01(kStrings - 1, 0) == glidingBefore);

        network.setNumStrings(kStrings);

        // THE assertion. A readmitted string that is still ringing keeps tracking the smoother it
        // was already tracking; it does not teleport to the target. Removing the !stringHasState
        // guard in StringNetwork::setNumStrings() makes this read 0.9 against a 0.715 neighbour.
        REQUIRE(network.tapPosition01(0, 0) == glidingBefore);
        for (int s = 2; s < kStrings; ++s) {
            INFO("readmitted string " << s);
            REQUIRE(network.tapPosition01(s, 0) == glidingBefore);
        }
    }

    // ---- and the same thing heard: the click metric across the whole churn -------------------
    auto render = [&](bool changeTheCount) {
        StringNetwork<float> network;
        StringNetworkParams params = configureSwept(network);
        BlockEventQueue events;
        pluckAll(events);

        std::vector<float> out;
        const int totalBlocks = kRingBlocks + kTailBlocks;
        out.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));
        for (int b = 0; b < totalBlocks; ++b) {
            if (b == kRingBlocks) {
                params.pickupPosition01 = 0.9f;
                network.setParams(params);
            }
            if (changeTheCount && b == kReduceBlock)
                network.setNumStrings(2);
            if (changeTheCount && b == kReduceBlock + kReIncreaseBlock)
                network.setNumStrings(kStrings);

            network.process(events, kBlock);
            for (int n = 0; n < kBlock; ++n)
                out.push_back(sumActiveTaps(network, n));
        }
        return out;
    };

    const std::vector<float> churned = render(true);
    const std::vector<float> reference = render(false);
    REQUIRE(churned.size() == reference.size());

    const auto changeSample =
        static_cast<std::size_t>(kReduceBlock + kReIncreaseBlock) * static_cast<std::size_t>(kBlock);
    const auto spanBegin = changeSample - static_cast<std::size_t>(kPreSeconds * kRate);
    const auto spanEnd = changeSample + static_cast<std::size_t>(kPostSeconds * kRate);

    REQUIRE(peakOf(reference, spanBegin, changeSample) > 0.001f);
    REQUIRE(peakOf(churned, spanEnd, churned.size()) > 0.001f); // still sounding afterwards

    const cnpg::test::ClickMeasurement referenceMeasurement =
        cnpg::test::measureClick(reference, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement churnedMeasurement =
        cnpg::test::measureClick(churned, kRate, spanBegin, spanEnd);
    const double excessDb = cnpg::test::clickExcessDb(churnedMeasurement, referenceMeasurement);

    // The churn takes real level out (three of five strings ramp down and part-way back), so this
    // is one of the level-reducing cases the plain criterion is documented as blind to. Both
    // readings are taken, each render's peak normalised by the motion of its OWN settled signal
    // after the change -- see tests/support/ClickMetric.h.
    const cnpg::test::ClickMeasurement churnedResidual =
        cnpg::test::measureClick(churned, kRate, spanEnd, churned.size());
    const cnpg::test::ClickMeasurement referenceResidual =
        cnpg::test::measureClick(reference, kRate, spanEnd, reference.size());
    const double residualExcessDb = cnpg::test::clickExcessAgainstResidualDb(churnedMeasurement, churnedResidual,
                                                                             referenceMeasurement, referenceResidual);

    std::cout << "[contract] count-churn-under-moving-pickup click metric: excess " << excessDb
              << " dB, residual-normalised " << residualExcessDb << " dB (limit " << cnpg::test::kClickMetricToleranceDb
              << " dB)\n";

    REQUIRE(referenceMeasurement.metric(referenceMeasurement) > 0.0);
    INFO("excess " << excessDb << " dB, residual-normalised " << residualExcessDb << " dB");
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(residualExcessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(churnedMeasurement.nonFiniteSamples == 0);
    REQUIRE(churnedMeasurement.subnormalSamples == 0);
}

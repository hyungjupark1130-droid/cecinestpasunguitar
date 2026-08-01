#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/ClickMetric.h"
#include "support/SpectralAnalysis.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include <catch2/catch_test_macros.hpp>

// RetriggerModeTests -- docs/plan.md Task P2.6, RetriggerMode at event consumption.
//
// Two modes, and the whole of the difference is what happens to the rails when a note lands on a
// string that is already playing one:
//
//   PHYSICAL keeps them. A same-pitch restrike plucks over the ringing state (exact superposition,
//   gated in StringNetworkTests). A pitch change retargets f0 and glides it over a ramp that LANDS,
//   with nothing cleared -- so the old note continues into the new one, which is what "emergent
//   legato" means here.
//   SYNTH throws them away, after a 2 ms fade that makes the throw click-free, so no partial of the
//   old note survives into the new one at all.
//
// The file also carries two measurements that are not acceptance criteria but are owed to the
// record: what a pitch-changing retrigger costs a string that was ringing SYMPATHETICALLY (Task
// P2.4's entry condition for this task), and the evidence behind refusing the plan's "damper choke"
// clause.

namespace {

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RetriggerMode;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr float kPickup = 0.87f;

// The two pitches every cross-pitch case uses: MIDI 45 (110.000 Hz) and MIDI 51 (155.563 Hz), a
// tritone apart. The ratio is 2^(1/2), an irrational number, so NO partial of the old note ever
// coincides with a partial of the new one -- which is what makes "no old-pitch partial exceeds
// -60 dBc" a measurable claim instead of a claim about whichever partial they happen to share.
constexpr int kOldNote = 45;
constexpr int kNewNote = 51;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex = 0, float velocity = 0.8f) {
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

NoteEvent noteOff(int sampleOffset, int midiNote, int stringIndex = 0) {
    NoteEvent event = noteOn(sampleOffset, midiNote, stringIndex);
    event.type = NoteEventType::NoteOff;
    event.pluckPosition = cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

StringNetworkParams paramsFor(RetriggerMode mode, float coupling = 0.0f) {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    params.retriggerMode = mode;
    // Decoupled unless a case is about coupling: these cases state their claims about ONE string's
    // tap, and a loaded bridge would put the other strings' answer into it. The two cases that ARE
    // about coupling set it explicitly to the shipping value.
    params.bridge.couplingStrength = coupling;
    return params;
}

void configure(StringNetwork<float>& network, const StringNetworkParams& params, int numStrings = 1) {
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(numStrings);
    network.setParams(params);
    network.reset();
}

void renderInto(StringNetwork<float>& network, BlockEventQueue& events, int blocks, std::vector<float>& out,
                int stringIndex = 0) {
    for (int b = 0; b < blocks; ++b) {
        network.process(events, kBlock);
        const float* channel = network.tapBuffers().channel(stringIndex, 0);
        REQUIRE(channel != nullptr);
        out.insert(out.end(), channel, channel + kBlock);
    }
}

float peakOf(const std::vector<float>& samples, std::size_t begin = 0, std::size_t end = 0) {
    if (end == 0 || end > samples.size())
        end = samples.size();
    float peak = 0.0f;
    for (std::size_t i = begin; i < end; ++i)
        peak = std::max(peak, std::fabs(samples[i]));
    return peak;
}

double rmsOver(const std::vector<float>& samples, std::size_t begin, std::size_t end) {
    end = std::min(end, samples.size());
    if (begin >= end)
        return 0.0;
    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i)
        sum += static_cast<double>(samples[i]) * static_cast<double>(samples[i]);
    return std::sqrt(sum / static_cast<double>(end - begin));
}

std::vector<double> toDouble(const std::vector<float>& samples, std::size_t begin, std::size_t count) {
    std::vector<double> out;
    out.reserve(count);
    for (std::size_t i = begin; i < begin + count && i < samples.size(); ++i)
        out.push_back(static_cast<double>(samples[i]));
    return out;
}

double dbOf(double linear) { return 20.0 * std::log10(std::max(linear, 1e-30)); }

// The index of the loudest sample in [begin, end). Pure mechanics; WHY a hard-cut negative control
// has to be placed there rather than at the event is stated at each call site, because the reason is
// a claim about that call site's own signal.
std::size_t loudestSample(const std::vector<float>& samples, std::size_t begin, std::size_t end) {
    end = std::min(end, samples.size());
    std::size_t best = begin;
    float bestLevel = -1.0f;
    for (std::size_t i = begin; i < end; ++i) {
        if (std::fabs(samples[i]) > bestLevel) {
            bestLevel = std::fabs(samples[i]);
            best = i;
        }
    }
    return best;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Physical: same pitch
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: Physical same-pitch restrike is click-free and never lets the string fall silent", "[contract]") {
    constexpr int kRingBlocks = 200; // ~0.53 s between the two attacks
    constexpr int kTailBlocks = 60;

    // TEST: attack, ring, restrike at the same pitch over the ringing state.
    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Physical));
    std::vector<float> test;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderInto(network, first, kRingBlocks, test);

    // IN THE STATE THIS CASE CLAIMS TO EXERCISE: the string is genuinely ringing, owns the note,
    // and its damper is open. Asserted rather than assumed -- the P2.1 trap.
    REQUIRE(network.energyEstimate() > 0.0);
    REQUIRE(network.damperEngagement(0) == 0.0f);
    const double energyBefore = network.energyEstimate();

    BlockEventQueue restrike;
    restrike.push(noteOn(0, kOldNote));
    renderInto(network, restrike, kTailBlocks, test);

    // NOTHING WAS CLEARED. The direct state assertion the P2.1 ruling requires beside the click
    // reading: after the restrike the string holds MORE than it did before, because the pluck was
    // added to what was already circulating rather than replacing it.
    REQUIRE(network.energyEstimate() > energyBefore);
    REQUIRE(network.retuneRampSamplesRemaining(0) == 0); // same pitch: no ramp at all
    REQUIRE_FALSE(network.retriggerFadeActive(0));       // Physical: no fade at all

    // REFERENCE: the identical restrike on a string that was NOT already ringing -- the same render
    // differing only in the thing under test, which is the presence of the prior state.
    StringNetwork<float> reference;
    configure(reference, paramsFor(RetriggerMode::Physical));
    std::vector<float> control;
    BlockEventQueue idle;
    renderInto(reference, idle, kRingBlocks, control);
    BlockEventQueue freshPluck;
    freshPluck.push(noteOn(0, kOldNote));
    renderInto(reference, freshPluck, kTailBlocks, control);

    // THE SPAN STARTS AT THE RESTRIKE, and that is forced rather than chosen. The reference render
    // is silent before its pluck, so a span reaching back 20 blocks would give it a median |dx| of
    // exactly zero -- half the samples are bit-identical silence -- and the metric would report
    // infinity for every render compared against it. That is ClickMetric.h's documented degeneracy
    // (and the same shared-pre-roll trap the P2.4 review recorded), so the analysed span is the
    // restrike plus 40 blocks, over which BOTH renders are sounding throughout.
    const std::size_t restrikeSample = static_cast<std::size_t>(kRingBlocks * kBlock);
    const std::size_t spanBegin = restrikeSample;
    const std::size_t spanEnd = restrikeSample + static_cast<std::size_t>(40 * kBlock);

    // The A/B pair: the same pluck at the same moment, differing only in whether the string was
    // already ringing. That is the thing under test -- "does plucking over a ringing string
    // introduce a discontinuity the same pluck on a silent string does not".
    const cnpg::test::ClickMeasurement freshMeasurement = cnpg::test::measureClick(control, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement measured = cnpg::test::measureClick(test, kRate, spanBegin, spanEnd);
    REQUIRE(freshMeasurement.medianAbsDiff > 0.0); // non-degenerate reference
    REQUIRE(measured.nonFiniteSamples == 0);
    REQUIRE(measured.subnormalSamples == 0);
    REQUIRE(freshMeasurement.nonFiniteSamples == 0);
    const double excessDb = cnpg::test::clickExcessDb(measured, freshMeasurement);

    // NEGATIVE CONTROL, carried inside the gate: the test render hard-cut inside the span, i.e. a
    // real discontinuity in the same signal over the same window. If the metric ever stops
    // discriminating, this fails first.
    //
    // LEVEL-PLACED, not placed at a fixed offset, and that is the P2.2 lesson rather than a
    // convenience (tests/dsp/SustainPedalTests.cpp states it in full). A hard cut's peak |dx| IS the
    // sample value it lands on, so a cut taken at an arbitrary offset can land near a zero crossing
    // and be a cut of almost nothing -- the control then PASSES the gate it exists to fail and the
    // gate is left provably toothless. Placing it at the loudest sample in the window it is allowed
    // to use is the largest discontinuity this signal can be given there, so the control is as
    // strong as the signal permits rather than as strong as an arbitrary index happened to make it.
    const std::size_t cutSample = loudestSample(test, spanBegin, spanEnd);
    const float cutLevel = std::fabs(test[cutSample]);
    REQUIRE(cutLevel > 0.0f);
    std::vector<float> hardCut = test;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
    const cnpg::test::ClickMeasurement hardCutMeasurement =
        cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd);
    const double hardCutExcessDb = cnpg::test::clickExcessDb(hardCutMeasurement, freshMeasurement);

    // RMS FLOOR. "String RMS never drops below -60 dBFS between the two attacks", measured in 10 ms
    // windows across the whole gap so a single quiet moment cannot hide inside a long average.
    const std::size_t windowSamples = static_cast<std::size_t>(0.010 * kRate);
    double worstWindowDb = 0.0;
    bool firstWindow = true;
    for (std::size_t begin = windowSamples; begin + windowSamples <= restrikeSample; begin += windowSamples) {
        const double windowDb = dbOf(rmsOver(test, begin, begin + windowSamples));
        if (firstWindow || windowDb < worstWindowDb) {
            worstWindowDb = windowDb;
            firstWindow = false;
        }
    }

    std::cout << "[contract] Physical same-pitch restrike: click excess " << excessDb << " dB (limit "
              << cnpg::test::kClickMetricToleranceDb << "), level-placed hard-cut control " << hardCutExcessDb
              << " dB at sample +" << (cutSample - restrikeSample) << " (level " << cutLevel
              << "); worst 10 ms RMS between the attacks " << worstWindowDb << " dBFS (floor -60)\n";

    INFO("excess " << excessDb << " dB, hard-cut control " << hardCutExcessDb << " dB");
    REQUIRE(hardCutExcessDb > cnpg::test::kClickMetricToleranceDb);
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(worstWindowDb > -60.0);
}

// ---------------------------------------------------------------------------------------------
// Physical: pitch change
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: Physical cross-pitch restrike lands the new pitch inside 30 ms with no click", "[contract]") {
    constexpr int kRingBlocks = 200;
    constexpr int kTailBlocks = 320;            // enough for the 30 ms offset plus a 2^15 analysis window
    constexpr double kCriterionSeconds = 0.030; // docs/plan.md P2.6: "within 30 ms"

    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Physical));
    std::vector<float> test;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderInto(network, first, kRingBlocks, test);

    const double energyBefore = network.energyEstimate();
    REQUIRE(energyBefore > 0.0);
    const float oldHz = network.stringF0Hz(0);
    REQUIRE(oldHz > 0.0f);

    // The ramp fires, and it is asserted MID-FLIGHT before its landing is asserted: a ramp measured
    // after it has cleared is a ramp nobody measured.
    BlockEventQueue restrike;
    restrike.push(noteOn(0, kNewNote));
    network.process(restrike, kBlock);
    {
        const float* channel = network.tapBuffers().channel(0, 0);
        test.insert(test.end(), channel, channel + kBlock);
    }
    const int rampSamples = static_cast<int>(std::lround(network.retuneRampSeconds() * kRate));
    REQUIRE(rampSamples > kBlock);
    REQUIRE(network.retuneRampSamplesRemaining(0) == rampSamples - kBlock);
    const float midRampHz = network.stringF0Hz(0);
    INFO("f0 mid-ramp " << midRampHz << " Hz between " << oldHz << " and the target");
    REQUIRE(midRampHz > oldHz); // genuinely gliding upward, not snapped and not stuck

    // RAIL STATE PRESERVED. Physical mode never clears here, so the string is not a fresh instance:
    // its stored energy did not pass through zero on the way.
    REQUIRE(network.energyEstimate() > 0.0);
    REQUIRE_FALSE(network.retriggerFadeActive(0));

    renderInto(network, restrike, kTailBlocks, test);

    // THE CRITERION, on the direct state: f0 IS the new note -- exactly, to the last bit -- and it
    // got there well inside 30 ms.
    const double landedSeconds = static_cast<double>(rampSamples) / kRate;
    const float targetHz = static_cast<float>(cnpg::test::midiNoteToHz(kNewNote));
    REQUIRE(network.retuneRampSamplesRemaining(0) == 0);
    REQUIRE(network.stringF0Hz(0) == targetHz);
    REQUIRE(landedSeconds <= kCriterionSeconds + 1.0e-12);

    // ...and on the AUDIO, which is the claim a listener could check: the tap's measured
    // fundamental, over a window that starts once the ramp is spent, is the new note.
    const std::size_t restrikeSample = static_cast<std::size_t>(kRingBlocks * kBlock);
    const std::size_t analysisBegin = restrikeSample + static_cast<std::size_t>(kCriterionSeconds * kRate);
    const std::size_t analysisLength = 1u << 15;
    REQUIRE(test.size() >= analysisBegin + analysisLength);
    const std::vector<double> analysed = toDouble(test, analysisBegin, analysisLength);
    const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(analysed, kRate, analysisLength);
    const double measuredHz = cnpg::test::findPeakHz(spectrum, targetHz, cnpg::test::kTuningSearchCents);
    REQUIRE(measuredHz > 0.0);
    const double centsOff = cnpg::test::centsBetween(measuredHz, targetHz);

    // CLICK. Reference: the same note plucked on a string that was not already ringing, i.e. the
    // same render differing only in whether there was an old note to retune.
    StringNetwork<float> reference;
    configure(reference, paramsFor(RetriggerMode::Physical));
    std::vector<float> control;
    BlockEventQueue idle;
    renderInto(reference, idle, kRingBlocks, control);
    BlockEventQueue freshPluck;
    freshPluck.push(noteOn(0, kNewNote));
    renderInto(reference, freshPluck, kTailBlocks + 1, control);
    REQUIRE(control.size() == test.size());

    // The span starts AT the restrike: the reference render is silent before its own pluck, so a
    // span reaching backwards would hand it a median |dx| of exactly zero and every comparison
    // against it would read infinity (ClickMetric.h's documented degeneracy).
    const std::size_t spanBegin = restrikeSample;
    const std::size_t spanEnd = restrikeSample + static_cast<std::size_t>(40 * kBlock);
    const cnpg::test::ClickMeasurement freshMeasurement = cnpg::test::measureClick(control, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement measured = cnpg::test::measureClick(test, kRate, spanBegin, spanEnd);
    REQUIRE(freshMeasurement.medianAbsDiff > 0.0);
    REQUIRE(measured.nonFiniteSamples == 0);
    REQUIRE(measured.subnormalSamples == 0);
    const double excessDb = cnpg::test::clickExcessDb(measured, freshMeasurement);

    // NEGATIVE CONTROL ONE: a hard cut inside the span, the standing check that the metric still
    // discriminates a real discontinuity in this signal over this window. LEVEL-PLACED at the
    // loudest sample in the span for the reason the same-pitch case above states in full: a cut's
    // peak |dx| is the sample value it lands on, so a cut at an arbitrary offset can land near a
    // zero crossing and pass the gate it exists to fail.
    const std::size_t cutSample = loudestSample(test, spanBegin, spanEnd);
    const float cutLevel = std::fabs(test[cutSample]);
    REQUIRE(cutLevel > 0.0f);
    std::vector<float> hardCut = test;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
    const double hardCutExcessDb =
        cnpg::test::clickExcessDb(cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd), freshMeasurement);

    // THE RAMP-LENGTH SWEEP. A retune ramp is not a fade: it does not remove motion, it TURNS A
    // STEP INTO A GLIDE, and a glide of a full rail has motion of its own -- while the rails
    // shorten, the read position sweeps through the buffer faster than one sample per sample, which
    // is a real and intended pitch change and reads on a peak-|dx| metric as extra motion that a
    // fresh pluck does not have. The excess against a fresh pluck therefore falls with the ramp
    // length rather than vanishing at any length. Measured here rather than assumed.
    //
    // WHAT THIS STATISTIC CANNOT DO, stated before anything is concluded from it: it cannot choose
    // the constant. The reference is a FRESH PLUCK, which contains no glide at all, so every
    // millisecond of glide is excess by construction and a longer ramp always reads better -- the
    // statistic's minimum is at "no legato", which is not an available answer. It also oscillates by
    // up to 2.4 dB between adjacent millisecond values (below), because it is a peak taken over a
    // 40-block window and a glide moves the phase at which the loudest transition lands. It is a
    // check that the transition is not a STEP; it is not a preference ordering over ramp lengths.
    // The real decider is whether 30 ms of glide reads as a hammer-on or as a slide, which is
    // docs/listening/physical-plausibility-checklist.md item 17 and has no headless answer.
    auto excessForRamp = [&](double rampSeconds) {
        StringNetwork<float> variant;
        configure(variant, paramsFor(RetriggerMode::Physical));
        variant.setRetuneRampSeconds(rampSeconds);
        std::vector<float> render;
        BlockEventQueue firstNote;
        firstNote.push(noteOn(0, kOldNote));
        renderInto(variant, firstNote, kRingBlocks, render);
        BlockEventQueue secondNote;
        secondNote.push(noteOn(0, kNewNote));
        renderInto(variant, secondNote, kTailBlocks + 1, render);
        REQUIRE(variant.retuneRampSamplesRemaining(0) == 0);
        return cnpg::test::clickExcessDb(cnpg::test::measureClick(render, kRate, spanBegin, spanEnd), freshMeasurement);
    };

    // Three coarse points, a 1 ms-resolution fan from 16 ms to 26 ms, and the 28..32 ms neighbourhood
    // the gate below is taken over. The fan exists because the five-point sweep this file shipped
    // with was too coarse to see what the statistic actually does between its samples, and a claim
    // was drawn from it that a finer sweep falsifies.
    static const double kSweptRamps[] = {1.0 / kRate, 0.002, 0.008, 0.016, 0.017, 0.018, 0.019, 0.020, 0.021, 0.022,
                                         0.023,       0.024, 0.025, 0.026, 0.028, 0.029, 0.030, 0.031, 0.032};
    constexpr std::size_t kSweptCount = sizeof(kSweptRamps) / sizeof(kSweptRamps[0]);
    constexpr std::size_t kFastPoints = 3;     // <= 8 ms
    constexpr std::size_t kShippedIndex = 16;  // 30 ms, the shipped kRetuneRampSeconds
    constexpr std::size_t kNeighbourhood = 14; // first index of the 28..32 ms neighbourhood
    double sweptExcess[kSweptCount] = {};
    std::cout << "[contract] Physical retune ramp length vs click excess against a fresh pluck:";
    for (std::size_t i = 0; i < kSweptCount; ++i) {
        sweptExcess[i] = excessForRamp(kSweptRamps[i]);
        std::cout << "  " << (kSweptRamps[i] * 1000.0) << " ms -> " << sweptExcess[i] << " dB";
    }
    std::cout << "\n";
    REQUIRE(kSweptRamps[kShippedIndex] == cnpg::dsp::kRetuneRampSeconds);
    // The sweep really re-measures the SHIPPED configuration at its shipped index, rather than
    // running beside the gate and agreeing with it by coincidence.
    REQUIRE(std::fabs(sweptExcess[kShippedIndex] - excessDb) < 1.0e-9);

    // THE GATE, TAKEN OVER A NEIGHBOURHOOD OF RAMP LENGTHS RATHER THAN AT ONE POINT, and the reason
    // is in the sweep above. The statistic swings by up to 2.4 dB between adjacent millisecond
    // values -- 26 ms reads 1.90, 28 ms reads 4.13, 29 ms reads 1.19 -- so a single-point gate with
    // 1.3 dB of margin is decided by which side of one local oscillation the shipped value lands on,
    // and a different note pair, a change to the fractional-delay solve or a different toolchain
    // could flip it red for no musical reason at all. It would also flip GREEN for none.
    //
    // The physical claim is about a retune ramp of ORDER 30 ms, not about 1440 samples exactly, so
    // the criterion is applied to the MEDIAN of the shipped value plus and minus 2 ms. A median of
    // five is unmoved by up to two outlying points, so no single oscillation can decide the gate;
    // a real regression -- anything that makes the retune transition itself a step -- moves every
    // point in the neighbourhood together and is caught. Three of five points would have to fail
    // before this line does, against one for the point gate it replaces, at the same margin.
    std::vector<double> neighbourhood(std::begin(sweptExcess) + kNeighbourhood, std::end(sweptExcess));
    REQUIRE(neighbourhood.size() == 5);
    std::sort(neighbourhood.begin(), neighbourhood.end());
    const double neighbourhoodMedianDb = neighbourhood[2];

    std::cout << "[contract] Physical cross-pitch restrike: ramp " << rampSamples << " samples ("
              << (landedSeconds * 1000.0) << " ms, criterion " << (kCriterionSeconds * 1000.0)
              << "); measured f0 after the ramp " << measuredHz << " Hz vs target " << targetHz << " (" << centsOff
              << " cents); click excess " << excessDb << " dB (peak |dx| " << measured.peakWindowAbsDiff << " vs "
              << freshMeasurement.peakWindowAbsDiff << " for the fresh pluck), one-sample-ramp control "
              << sweptExcess[0] << " dB, level-placed hard-cut control " << hardCutExcessDb << " dB at sample +"
              << (cutSample - restrikeSample) << " (level " << cutLevel << ")\n";

    INFO("excess " << excessDb << " dB, hard-cut " << hardCutExcessDb << " dB, snapped " << sweptExcess[0] << " dB");
    REQUIRE(hardCutExcessDb > cnpg::test::kClickMetricToleranceDb); // the metric still discriminates
    REQUIRE(std::fabs(centsOff) < 20.0); // the audio really is at the new pitch, not merely the state

    // THE CRITERION AS WRITTEN, at the shipped ramp length, read over its neighbourhood.
    std::cout << "[contract] Physical retune ramp click gate over the 28..32 ms neighbourhood: median "
              << neighbourhoodMedianDb << " dB (limit " << cnpg::test::kClickMetricToleranceDb
              << "), shipped 30 ms point " << excessDb << " dB, neighbourhood spread " << neighbourhood.front() << ".."
              << neighbourhood.back() << " dB\n";
    INFO("neighbourhood median " << neighbourhoodMedianDb << " dB, shipped point " << excessDb << " dB");
    REQUIRE(neighbourhoodMedianDb <= cnpg::test::kClickMetricToleranceDb);

    // ...and the reading really is measuring the glide rather than something incidental: every ramp
    // at or above 16 ms reads strictly better than every ramp at or below 8 ms. That is the trend
    // the constant leans on, and it is the only monotone statement the data supports.
    const double worstSlow = *std::max_element(std::begin(sweptExcess) + kFastPoints, std::end(sweptExcess));
    const double bestFast = *std::min_element(std::begin(sweptExcess), std::begin(sweptExcess) + kFastPoints);
    REQUIRE(worstSlow < bestFast);
    REQUIRE(excessDb < sweptExcess[0]);

    // THE STATISTIC IS NOT MONOTONE, and that is asserted rather than merely printed, because the
    // claim it replaces was. This file previously asserted a three-point descent over the top of a
    // five-point sweep and read it as "30 ms is the shortest ramp that clears the criterion". At
    // 1 ms resolution that is false in both halves: the sequence inverts repeatedly across the WHOLE
    // range (not only at the fast end), and 22 ms clears the criterion at 1.70 dB while 28 ms FAILS
    // it at 4.13 dB sitting between two passing neighbours. A sampled statistic that jumps like that
    // between adjacent values cannot order ramp lengths, and no constant may be chosen from it.
    int inversions = 0;
    int shorterRampsThatClear = 0;
    for (std::size_t i = kFastPoints; i < kSweptCount; ++i) {
        if (i > kFastPoints && sweptExcess[i] > sweptExcess[i - 1])
            ++inversions; // a LONGER ramp reading WORSE than the one before it
        if (kSweptRamps[i] < cnpg::dsp::kRetuneRampSeconds && sweptExcess[i] <= cnpg::test::kClickMetricToleranceDb)
            ++shorterRampsThatClear;
    }
    std::cout << "[contract] Physical retune ramp sweep shape: " << inversions
              << " points where a LONGER ramp read worse, and " << shorterRampsThatClear
              << " ramps shorter than the shipped 30 ms that clear the " << cnpg::test::kClickMetricToleranceDb
              << " dB criterion -- so 30 ms is NOT the shortest ramp that clears it, and this statistic does not "
                 "order ramp lengths\n";
    REQUIRE(inversions > 0);
    REQUIRE(shorterRampsThatClear > 0);
}

// ---------------------------------------------------------------------------------------------
// Synth
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: Synth cross-pitch restrike leaves no old-pitch partial above -60 dBc", "[contract]") {
    constexpr int kRingBlocks = 200;
    constexpr int kTailBlocks = 400; // long enough for the 100 ms offset plus a 2^15 analysis window
    constexpr double kAnalysisOffsetSeconds = 0.100;
    constexpr double kCarrierLimitDbc = -60.0;

    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Synth));
    std::vector<float> render;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderInto(network, first, kRingBlocks, render);

    // IN STATE: the old note is genuinely ringing when the restrike arrives.
    REQUIRE(network.energyEstimate() > 0.0);
    const float oldPeakBefore = peakOf(render, render.size() - static_cast<std::size_t>(kBlock), render.size());
    REQUIRE(oldPeakBefore > 0.001f);

    // The fade is asserted MID-FLIGHT, one sample-block at a time, so its shape is pinned rather
    // than inferred from the result: the gain descends, reaches exactly 0, and the state is cleared
    // on the sample after that.
    const int fadeSamples = static_cast<int>(std::lround(cnpg::dsp::kSynthFadeSeconds * kRate));
    REQUIRE(fadeSamples >= 1);
    REQUIRE(static_cast<double>(fadeSamples) / kRate <= 0.005); // the plan's "<= 5 ms"

    BlockEventQueue restrike;
    restrike.push(noteOn(0, kNewNote));
    // One sub-block short of the fade's length, so the fade is caught in flight.
    const int midFadeSamples = fadeSamples / 2;
    network.process(restrike, midFadeSamples);
    {
        const float* channel = network.tapBuffers().channel(0, 0);
        render.insert(render.end(), channel, channel + midFadeSamples);
    }
    REQUIRE(network.retriggerFadeActive(0));
    const float midFadeGain = network.retriggerFadeGain(0);
    INFO("fade gain " << midFadeGain << " after " << midFadeSamples << " of " << fadeSamples << " samples");
    REQUIRE(midFadeGain > 0.0f);
    REQUIRE(midFadeGain < 1.0f);
    REQUIRE(network.energyEstimate() > 0.0); // not cleared yet: the fade comes first

    renderInto(network, restrike, kTailBlocks, render);
    REQUIRE_FALSE(network.retriggerFadeActive(0));
    REQUIRE(network.retriggerFadeGain(0) == 1.0f);
    // Synth re-inits rather than glides: no retune ramp is ever started.
    REQUIRE(network.retuneRampSamplesRemaining(0) == 0);
    REQUIRE(network.stringF0Hz(0) == static_cast<float>(cnpg::test::midiNoteToHz(kNewNote)));

    // THE CRITERION. Spectrum of the string's tap over a window starting 100 ms after the restrike.
    const std::size_t restrikeSample = static_cast<std::size_t>(kRingBlocks * kBlock);
    const std::size_t analysisBegin = restrikeSample + static_cast<std::size_t>(kAnalysisOffsetSeconds * kRate);
    const std::size_t analysisLength = 1u << 15; // 683 ms at 48 kHz: ~5.9 Hz of main-lobe half-width
    REQUIRE(render.size() >= analysisBegin + analysisLength);

    const std::vector<double> analysed = toDouble(render, analysisBegin, analysisLength);
    const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(analysed, kRate, analysisLength);

    const double newF0 = cnpg::test::midiNoteToHz(kNewNote);
    const double oldF0 = cnpg::test::midiNoteToHz(kOldNote);

    // THE CARRIER, i.e. the "c" in dBc: the new note's own fundamental in the same spectrum.
    auto magnitudeAt = [&spectrum](double hz) {
        const double bin = spectrum.hzToBin(hz);
        const auto index = static_cast<std::size_t>(std::lround(bin));
        double best = 0.0;
        // The peak within one main lobe of the nominal frequency, so a partial pulled a few cents by
        // dispersion is measured rather than missed.
        const std::size_t half = 8;
        for (std::size_t i = (index > half ? index - half : 0);
             i <= index + half && i < spectrum.magnitudeSquared.size(); ++i)
            best = std::max(best, spectrum.magnitudeSquared[i]);
        return std::sqrt(best);
    };

    const double carrier = magnitudeAt(newF0);
    REQUIRE(carrier > 0.0); // non-vacuous: the new note really is there

    double worstOldPartialDbc = -1000.0;
    int checkedPartials = 0;
    std::cout << "[contract] Synth cross-pitch: old-pitch partials relative to the new fundamental";
    for (int k = 1; k <= 6; ++k) {
        const double oldPartial = oldF0 * static_cast<double>(k);
        // Skip a partial that sits inside a new-note partial's main lobe -- there it is not
        // measurable as "old" content at all. With a tritone ratio the nearest approach over these
        // six partials is 18.9 Hz, i.e. three main-lobe half-widths, so nothing is skipped in
        // practice and the count below asserts it.
        bool collides = false;
        for (int m = 1; m <= 8; ++m)
            if (std::fabs(oldPartial - newF0 * static_cast<double>(m)) < 12.0)
                collides = true;
        if (collides)
            continue;
        ++checkedPartials;
        const double dbc = dbOf(magnitudeAt(oldPartial) / carrier);
        worstOldPartialDbc = std::max(worstOldPartialDbc, dbc);
        std::cout << " p" << k << "(" << oldPartial << " Hz)=" << dbc << " dBc";
    }
    std::cout << "\n";
    REQUIRE(checkedPartials >= 5); // the measurement is not vacuous by exclusion
    INFO("worst old-pitch partial " << worstOldPartialDbc << " dBc");
    REQUIRE(worstOldPartialDbc < kCarrierLimitDbc);

    // AND THE CONTRAST, on the same measurement: Physical mode keeps the rails, so its stored energy
    // at the restrike does NOT pass through zero, while Synth's does -- exactly.
    StringNetwork<float> physical;
    configure(physical, paramsFor(RetriggerMode::Physical));
    std::vector<float> ignored;
    BlockEventQueue physicalFirst;
    physicalFirst.push(noteOn(0, kOldNote));
    renderInto(physical, physicalFirst, kRingBlocks, ignored);
    const double physicalBefore = physical.energyEstimate();
    BlockEventQueue physicalRestrike;
    physicalRestrike.push(noteOn(0, kNewNote));
    physical.process(physicalRestrike, 1);
    std::cout << "[contract] at the restrike sample, stored energy: Physical " << physical.energyEstimate() << " (was "
              << physicalBefore << "), Synth cleared to exactly 0 at its fade's landing\n";
    REQUIRE(physical.energyEstimate() > 0.0);
}

TEST_CASE("CONTRACT: the Synth fade reaches exactly zero and clears the string there", "[contract]") {
    // The fade's own contract, sample by sample, on the quantity a click reading cannot show: it
    // descends monotonically, its last faded sample is EXACTLY zero (which is what makes the state
    // clear on the next sample free), and the string is silent for no longer than that.
    const int fadeSamples = static_cast<int>(std::lround(cnpg::dsp::kSynthFadeSeconds * kRate));

    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Synth));
    std::vector<float> render;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderInto(network, first, 200, render);
    const double energyBefore = network.energyEstimate();
    REQUIRE(energyBefore > 0.0);

    BlockEventQueue restrike;
    restrike.push(noteOn(0, kNewNote));

    std::vector<float> gains;
    std::vector<float> samples;
    for (int n = 0; n < fadeSamples + 2; ++n) {
        network.process(restrike, 1);
        gains.push_back(network.retriggerFadeGain(0));
        samples.push_back(network.tapBuffers().channel(0, 0)[0]);
    }

    for (int n = 0; n < fadeSamples; ++n) {
        INFO("fade sample " << n << " gain " << gains[static_cast<std::size_t>(n)]);
        if (n > 0)
            REQUIRE(gains[static_cast<std::size_t>(n)] < gains[static_cast<std::size_t>(n) - 1]);
    }
    REQUIRE(gains[static_cast<std::size_t>(fadeSamples) - 1] == 0.0f);   // lands on exactly zero
    REQUIRE(samples[static_cast<std::size_t>(fadeSamples) - 1] == 0.0f); // ...so the tap does too
    REQUIRE(gains[static_cast<std::size_t>(fadeSamples)] == 1.0f);       // and the next sample is the new note
    REQUIRE_FALSE(network.retriggerFadeActive(0));

    std::cout << "[contract] Synth fade: " << fadeSamples << " samples (" << (1000.0 * fadeSamples / kRate)
              << " ms, criterion <= 5 ms), gain " << gains.front() << " -> 0 -> 1, latency " << (fadeSamples + 1)
              << " samples\n";
}

TEST_CASE("CONTRACT: a note released inside the Synth fade is not a stuck note", "[contract]") {
    // A host may send a note-off inside the 2 ms the fade takes. The allocator has already moved
    // ownership to the note that has not started yet, so this is the ONLY note-off that note will
    // ever receive -- dropping it would leave the string sounding for ever.
    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Synth));
    std::vector<float> render;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderInto(network, first, 200, render);

    BlockEventQueue events;
    events.push(noteOn(0, kNewNote));
    events.push(noteOff(1, kNewNote)); // one sample later: inside the fade by construction
    const int fadeSamples = static_cast<int>(std::lround(cnpg::dsp::kSynthFadeSeconds * kRate));
    REQUIRE(fadeSamples > 1);

    renderInto(network, events, 40, render);

    // The note started AND was released: the damper is engaged and the string is on its way out.
    REQUIRE(network.damperEngagement(0) > 0.0f);
    REQUIRE_FALSE(network.retriggerFadeActive(0));

    BlockEventQueue idle;
    renderInto(network, idle, 900, render); // ~2.4 s
    std::cout << "[contract] zero-length note across a Synth fade: energy after 2.4 s of silence "
              << network.energyEstimate() << "\n";
    REQUIRE(network.energyEstimate() == 0.0); // the watchdog cleared it: not stuck
}

// ---------------------------------------------------------------------------------------------
// Task P2.4's entry condition: what a retrigger costs a SYMPATHETICALLY ringing string
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: a fresh attack on a sympathetically ringing string truncates only what P2.4 measured",
          "[contract]") {
    // THE CARRY-FORWARD, quantified. Since Task P2.4 a string can be ringing with no note of its
    // own, and a NoteOn arriving on such a string takes the FRESH path -- it owns no note, so there
    // is no old note to be legato from -- which clears that motion. Through P2.3 that was inaudible
    // by construction (the string really was silent); it is now a real truncation, and this case
    // measures how big it is instead of asserting that it is small.
    //
    // The yardstick is P2.4's own number: at couplingStrength 0.35 a sympathetically driven string
    // peaks at -53.5 dBFS within 1 s (ADR 0006's measured table). Whatever a truncation discards, it
    // cannot exceed the level of the thing being truncated, so the claim to check is that the
    // discarded content sits at or below that -- and, more usefully, how far below the note that
    // replaces it the discarded content actually was.
    constexpr double kP24SympatheticPeakDbfs = -53.5;

    StringNetworkParams params = paramsFor(RetriggerMode::Physical, StringNetworkParams{}.bridge.couplingStrength);
    REQUIRE(params.bridge.couplingStrength > 0.0f); // the case is about coupling; assert it is on

    StringNetwork<float> network;
    configure(network, params, 2);

    std::vector<float> sympathetic;
    BlockEventQueue drive;
    drive.push(noteOn(0, kOldNote, 0)); // string 0 only; string 1 is never played
    renderInto(network, drive, 300, sympathetic, 1);

    // IN THE STATE THIS CASE CLAIMS TO EXERCISE: string 1 carries real motion and owns no note.
    const double sympatheticEnergy = network.stringEnergyEstimate(1);
    REQUIRE(sympatheticEnergy > 0.0);
    const std::size_t preWindow = static_cast<std::size_t>(10 * kBlock);
    const float truncatedPeak = peakOf(sympathetic, sympathetic.size() - preWindow, sympathetic.size());
    REQUIRE(truncatedPeak > 0.0f);

    // Now play a note on string 1. It owns none, so this is a fresh attack and the sympathetic
    // motion is discarded.
    std::vector<float> afterAttack;
    BlockEventQueue attack;
    attack.push(noteOn(0, kNewNote, 1));
    renderInto(network, attack, 40, afterAttack, 1);

    const double energyAfter = network.stringEnergyEstimate(1);
    const float attackPeak = peakOf(afterAttack);
    REQUIRE(attackPeak > 0.001f);

    const double truncatedDbfs = dbOf(static_cast<double>(truncatedPeak));
    const double belowAttackDb = dbOf(static_cast<double>(truncatedPeak) / static_cast<double>(attackPeak));

    std::cout << "[contract] sympathetic truncation at couplingStrength " << params.bridge.couplingStrength
              << ": discarded content peaked at " << truncatedDbfs << " dBFS in the 27 ms before the attack ("
              << belowAttackDb << " dB below the note that replaced it); string energy " << sympatheticEnergy << " -> "
              << energyAfter << ". P2.4 (ADR 0006) measured " << kP24SympatheticPeakDbfs
              << " dBFS for a unison pair; this reads " << (truncatedDbfs - kP24SympatheticPeakDbfs)
              << " dB HIGHER -- see the note below, it is a property of the DRIVEN string's tuning, not of the "
                 "truncation\n";

    // IT EXCEEDS P2.4'S FIGURE, AND THAT IS REPORTED RATHER THAN ABSORBED. P2.4 measured a unison
    // PAIR -- two strings dialled to nearly the same pitch -- which couples through one shared
    // partial. Here string 1 has never been played, so it still carries the pitch prepare() left it
    // at, kMinMidiNote (A0, 27.5 Hz), and A0's harmonic series contains 110 Hz exactly, as its
    // fourth partial. An untouched string is therefore a BETTER sympathetic resonator for the note
    // being played than a real open string would be, and it is the same for every string and every
    // note, because they are all at A0 until somebody plays them.
    //
    // *** FINDING FOR P2.7 / P2.8, not fixed here: an untouched string should rest at its OPEN
    // TUNING, not at A0. *** NoteAllocatorParams now knows what that tuning is
    // (openStringMidiNote), but StringNetworkParams has nowhere to put it -- midiNote_ is only ever
    // written by a note event -- so giving strings a rest pitch is a StringNetworkParams change that
    // moves the coupled chord goldens, and it belongs with the task that is already judging how much
    // sympathetic resonance the instrument should have (P2.8) or already regenerating for tuning
    // (P2.7). Recorded here with the measurement rather than left to be rediscovered.
    //
    REQUIRE(truncatedDbfs > kP24SympatheticPeakDbfs); // the direction of the excess is asserted, not hidden

    // THE CLAIM THIS CASE GATES: the truncation does not register as a click. It has a genuine A/B
    // control pair -- the SAME attack, on the SAME string, of the SAME instrument, differing only in
    // whether string 0 was played first and therefore whether string 1 had any sympathetic motion to
    // discard. Nothing else about the render changes, which is what makes the comparison a
    // measurement of the truncation rather than of the coupling.
    StringNetwork<float> quiet;
    configure(quiet, params, 2);
    std::vector<float> quietTap;
    BlockEventQueue silence;
    renderInto(quiet, silence, 300, quietTap, 1);  // string 0 never plays: string 1 stays silent
    REQUIRE(quiet.stringEnergyEstimate(1) == 0.0); // in state: NOTHING to truncate in the control
    BlockEventQueue quietAttack;
    quietAttack.push(noteOn(0, kNewNote, 1));
    renderInto(quiet, quietAttack, 40, quietTap, 1);

    std::vector<float> truncatedTap = sympathetic;
    truncatedTap.insert(truncatedTap.end(), afterAttack.begin(), afterAttack.end());
    REQUIRE(truncatedTap.size() == quietTap.size());

    const std::size_t attackSample = sympathetic.size();
    const std::size_t spanBegin = attackSample;
    const std::size_t spanEnd = attackSample + static_cast<std::size_t>(30 * kBlock);
    const cnpg::test::ClickMeasurement quietMeasurement = cnpg::test::measureClick(quietTap, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement truncatedMeasurement =
        cnpg::test::measureClick(truncatedTap, kRate, spanBegin, spanEnd);
    REQUIRE(quietMeasurement.medianAbsDiff > 0.0);
    REQUIRE(truncatedMeasurement.nonFiniteSamples == 0);
    REQUIRE(truncatedMeasurement.subnormalSamples == 0);
    const double truncationExcessDb = cnpg::test::clickExcessDb(truncatedMeasurement, quietMeasurement);

    // The negative control, level-placed for the reason recorded in SustainPedalTests: a blind cut
    // lands wherever it lands, and a cut through a zero crossing is a cut of nothing.
    std::size_t cutSample = attackSample;
    float cutLevel = 0.0f;
    for (std::size_t i = attackSample; i < spanEnd && i < quietTap.size(); ++i)
        if (std::fabs(quietTap[i]) > cutLevel) {
            cutLevel = std::fabs(quietTap[i]);
            cutSample = i;
        }
    REQUIRE(cutLevel > 0.0f);
    std::vector<float> hardCut = quietTap;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
    const double hardCutExcessDb =
        cnpg::test::clickExcessDb(cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd), quietMeasurement);

    std::cout << "[contract] sympathetic truncation click excess " << truncationExcessDb << " dB against the same "
              << "attack with nothing to truncate (limit " << cnpg::test::kClickMetricToleranceDb
              << "); level-placed hard-cut control " << hardCutExcessDb << " dB\n";
    INFO("truncation excess " << truncationExcessDb << " dB, hard-cut control " << hardCutExcessDb << " dB");
    REQUIRE(hardCutExcessDb > cnpg::test::kClickMetricToleranceDb);
    REQUIRE(truncationExcessDb <= cnpg::test::kClickMetricToleranceDb);

    // AND THE CONTRAST that makes the design decision checkable: a string that DOES own a note is
    // not truncated at all in Physical mode. Same instrument, same coupling, same pitch change --
    // the only difference is whether the string had a note of its own, which is exactly the
    // predicate NoteAllocator uses for "idle".
    StringNetwork<float> owning;
    configure(owning, params, 2);
    std::vector<float> ignored;
    BlockEventQueue own;
    own.push(noteOn(0, kOldNote, 1));
    renderInto(owning, own, 300, ignored, 1);
    const double ownedBefore = owning.stringEnergyEstimate(1);
    REQUIRE(ownedBefore > 0.0);
    BlockEventQueue ownRestrike;
    ownRestrike.push(noteOn(0, kNewNote, 1));
    owning.process(ownRestrike, 1);
    REQUIRE(owning.stringEnergyEstimate(1) > 0.0); // kept, not cleared
    REQUIRE(owning.retuneRampSamplesRemaining(1) > 0);
}

// ---------------------------------------------------------------------------------------------
// the refused clause
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: a retrigger damper choke buys suppression only with elapsed time, and charges the attack for it",
          "[contract]") {
    // THE EVIDENCE BEHIND A REFUSAL (see dsp/src/StringNetwork.cpp, the Physical pitch-change
    // branch). docs/plan.md P2.6 asks a pitch-changing retrigger to perform a "damper choke (fast
    // engage())" before re-exciting. It is refused, and this case is the measurement.
    //
    // A damper is a LINEAR two-port, so the output after the note-on is the free response of the
    // circulating state PLUS the response to the pluck -- superposition gives additivity, and that
    // is all it gives. It does not by itself give equal attenuation: a damper attenuates per PASS
    // through the junction, and for the first round trip (9.1 ms at 110 Hz) the fresh injection has
    // made fewer passes than the content already going round, so the balance really does shift for
    // that long. TO FIRST ORDER, ONCE ONE ROUND TRIP HAS ELAPSED, both occupy the same modal series
    // of the same string at the same rate and a damper acting from the re-excitation onward moves
    // the level of the whole result rather than the balance inside it. The transient is why this
    // case measures the balance instead of arguing it.
    //
    // The one thing a damper can do outright is act FIRST, on the old content alone, before the
    // pluck exists -- which is exactly what the plan's own ordering ("choke, ramp, THEN
    // re-excitation") describes, and its entire currency is elapsed time, up to the 30 ms the plan
    // itself budgets. 30 ms of latency on every legato note is not a playable instrument, and Q3
    // defers audible-slide behaviour besides.
    //
    // Measured with the SHIPPED machinery rather than with code kept alive to be measured: a
    // note-off placed 10.7 ms before the restrike engages the felt for real. Two numbers come out
    // of the same pair of renders -- what the choke removed from the old note (a direct reading of
    // the storage functional, not a spectrum), and what it then cost the re-attack that followed.
    constexpr int kRingBlocks = 200;
    constexpr int kChokeBlocks = 4; // 10.7 ms at 48 kHz / 128, well inside the plan's 30 ms budget
    constexpr int kAttackBlocks = 12;

    auto run = [](bool chokeFirst) {
        StringNetwork<float> network;
        configure(network, paramsFor(RetriggerMode::Physical));
        std::vector<float> out;
        BlockEventQueue first;
        first.push(noteOn(0, kOldNote));
        renderInto(network, first, kRingBlocks, out);

        if (chokeFirst) {
            BlockEventQueue release;
            release.push(noteOff(0, kOldNote));
            renderInto(network, release, kChokeBlocks, out);
        } else {
            BlockEventQueue idle;
            renderInto(network, idle, kChokeBlocks, out);
        }

        struct Result {
            double energyAtRestrike;
            float engagementAtRestrike;
            float attackPeak;
        };
        Result result{};
        result.energyAtRestrike = network.energyEstimate();
        result.engagementAtRestrike = network.damperEngagement(0);

        const std::size_t restrikeSample = out.size();
        BlockEventQueue restrike;
        restrike.push(noteOn(0, kNewNote));
        renderInto(network, restrike, kAttackBlocks, out);
        result.attackPeak = peakOf(out, restrikeSample, out.size());
        return result;
    };

    const auto plain = run(false);
    const auto choked = run(true);

    // IN STATE: one arm really has a felt on the string and the other really does not.
    REQUIRE(plain.engagementAtRestrike == 0.0f);
    REQUIRE(choked.engagementAtRestrike > 0.05f);
    REQUIRE(plain.energyAtRestrike > 0.0);
    REQUIRE(choked.energyAtRestrike > 0.0);

    const double suppressionDb = dbOf(choked.energyAtRestrike / plain.energyAtRestrike) * 0.5; // energy -> amplitude
    const double attackCostDb = dbOf(static_cast<double>(choked.attackPeak) / static_cast<double>(plain.attackPeak));

    std::cout << "[contract] refused choke, measured on the shipped felt (" << (1000.0 * kChokeBlocks * kBlock / kRate)
              << " ms before the restrike, engagement " << choked.engagementAtRestrike << "): the old note's stored "
              << "energy fell " << suppressionDb << " dB (amplitude), and the re-attack that followed was "
              << attackCostDb << " dB quieter than without the choke\n";

    std::cout << "[contract] refused choke, NET effect on the balance the choke exists to change: "
              << (suppressionDb - attackCostDb) << " dB\n";

    // (a) IT WORKS, AND IT WORKS BY ACTING FIRST. The suppression is real and it was bought with
    //     10.7 ms of elapsed time in which no new note existed.
    REQUIRE(suppressionDb < -1.0);
    // (b) AND IT CHARGES THE ATTACK. The same felt is still on the string when the pluck arrives --
    //     it decays with the felt time constant, not with the ramp -- so the note the player just
    //     asked for comes out quieter by very nearly the same amount.
    REQUIRE(attackCostDb < -0.5);
    // (c) THE REFUSAL, IN ONE NUMBER. What the choke changed about the BALANCE between the old note
    //     and the new one -- the only thing a choke could be for -- is the difference between those
    //     two, and it is under half a decibel. The damper took from both, because it is linear and
    //     they share a string. Everything it bought, it charged for.
    REQUIRE(std::fabs(suppressionDb - attackCostDb) < 0.5);
}

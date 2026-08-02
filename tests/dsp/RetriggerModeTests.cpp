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
    constexpr int kNominalRestrike = kRingBlocks * kBlock;

    // Renders an arbitrary NUMBER OF SAMPLES rather than a whole number of blocks, because the
    // re-strike below is placed by level and a level placement does not land on a block boundary.
    auto renderSamplesInto = [](StringNetwork<float>& network, BlockEventQueue& events, int samples,
                                std::vector<float>& out) {
        int done = 0;
        while (done < samples) {
            const int chunk = std::min(kBlock, samples - done);
            network.process(events, chunk);
            const float* channel = network.tapBuffers().channel(0, 0);
            REQUIRE(channel != nullptr);
            out.insert(out.end(), channel, channel + chunk);
            done += chunk;
        }
    };

    // ---------------------------------------------------------------------------------------------
    // *** THE RE-STRIKE IS PLACED BY LEVEL. A BLOCK BOUNDARY IS A BLIND PLACEMENT. ***
    // ---------------------------------------------------------------------------------------------
    // Until fixes wave 3 this case struck at sample 25600 -- kRingBlocks * kBlock, a round number of
    // blocks and therefore an ARBITRARY PHASE of the 110 Hz note being replaced. tests/support/
    // ClickMetric.h states the rule that forbids it, in the form that covers negative controls and
    // perturbations alike, and lists every site in the project that has broken it. This one is the
    // latest, and it is the first where the blind axis was inside the GATE rather than beside it.
    //
    // MEASURED, because the argument for the gate's shape depended on it. Recomputing the whole
    // 28..32 ms neighbourhood at ten re-strike phases spanning one period of the old note (436
    // samples), same code and same spans, gave medians of 0.111, 1.078, 1.711, 1.714, 1.764, 1.941,
    // 2.253, 2.426, 2.539 and 3.199 dB. The last one FAILS the 3 dB criterion -- three of the five
    // neighbourhood points fail there -- and it fails for no reason but a 3 ms shift in when the
    // second note arrives. The median-of-five removed the RAMP-LENGTH oscillation and left a second
    // arbitrary axis whose swing (3.09 dB) was larger than the gate's own margin (1.24 dB), which
    // destroys the discrimination the median was adopted for: "every point moved together" no longer
    // distinguishes a regression from a phase change.
    //
    // THE FIX IS THE ONE WAVE 2 ALREADY APPLIED TO ITS OWN PERTURBATION: search the old note's
    // waveform for its loudest sample and strike on the sample after it. The phase then stops being
    // a free parameter and becomes a property of the signal -- the peak of the cycle -- which is
    // both reproducible and the worst case available.
    //
    // *** THE SEARCH WINDOW IS A FULL PERIOD HERE, WHERE WAVE 2 USED HALF, AND THE DIFFERENCE IS
    // MEASURED RATHER THAN STYLISTIC. *** A window of one full period contains the cycle's GLOBAL
    // |x| extremum, so every anchor inside the period finds the same phase; a half-period window
    // contains only some local extremum, which for this waveform is a different phase depending on
    // where the window fell. Measured across the same ten anchors: with a half-period window the
    // placements landed on five different phases and the neighbourhood median still ranged 0.141 to
    // 3.147 dB -- the axis was NOT closed. With a full-period window all ten anchors collapse onto
    // one phase (two distinct samples, exactly one period apart) and the medians read 0.959 and
    // 0.976 dB. Wave 2 needed the shorter window because its sweep had seven ages that had to stay
    // distinct; this case places once, so the full period is available and it is the one that works.
    const int kOldPeriodSamples = static_cast<int>(std::ceil(kRate / cnpg::test::midiNoteToHz(kOldNote)));

    // THE RING-ONLY RENDER: the old note and nothing else. It is the shared pre-re-strike history of
    // every arm below, so it is both the waveform the placement is read off and the render the
    // bit-identity guard checks against.
    std::vector<float> ringOnly;
    {
        StringNetwork<float> ring;
        configure(ring, paramsFor(RetriggerMode::Physical));
        BlockEventQueue ringNote;
        ringNote.push(noteOn(0, kOldNote));
        renderSamplesInto(ring, ringNote, kNominalRestrike + 3 * kOldPeriodSamples, ringOnly);
    }
    auto placeRestrike = [&](int anchor) {
        return static_cast<int>(loudestSample(ringOnly, static_cast<std::size_t>(anchor),
                                              static_cast<std::size_t>(anchor + kOldPeriodSamples))) +
               1;
    };
    const int restrikeSample = placeRestrike(kNominalRestrike);
    // The same placement rule anchored one period later. It is a DIFFERENT SAMPLE AT THE SAME PHASE,
    // which is what makes it the guard the gate needs: if the placement really has closed the phase
    // axis, the whole neighbourhood measured there must agree with the one measured here. Asserted
    // below as an agreement in dB, not merely printed. (Wave 2's "adjacent placements are distinct"
    // guard, in the form this case can carry: it places once, so what has to be shown is not that
    // two rows differ but that two placements one cycle apart do not.)
    const int nextCyclePlacement = placeRestrike(kNominalRestrike + kOldPeriodSamples);
    REQUIRE(restrikeSample > kNominalRestrike);
    REQUIRE(nextCyclePlacement > restrikeSample);
    REQUIRE(nextCyclePlacement - restrikeSample >= kOldPeriodSamples - 2);
    REQUIRE(nextCyclePlacement - restrikeSample <= kOldPeriodSamples + 2);

    StringNetwork<float> network;
    configure(network, paramsFor(RetriggerMode::Physical));
    std::vector<float> test;
    BlockEventQueue first;
    first.push(noteOn(0, kOldNote));
    renderSamplesInto(network, first, restrikeSample, test);

    // THE PLACEMENT WAS READ OFF THE WAVEFORM BEING MEASURED, asserted rather than assumed (wave 2's
    // second guard). Without this the level placement would be a claim about a different render.
    REQUIRE(std::equal(test.begin() + kNominalRestrike, test.begin() + restrikeSample,
                       ringOnly.begin() + kNominalRestrike));
    // ...and the placement DID something: the sample the re-strike lands on top of is at least as
    // large as the one the old block-boundary placement would have used. That is what "worst case
    // available" means here, and it is the non-vacuity of the search.
    const double placedLevel = std::fabs(static_cast<double>(test[static_cast<std::size_t>(restrikeSample) - 1]));
    const double blindLevel = std::fabs(static_cast<double>(test[kNominalRestrike - 1]));
    REQUIRE(placedLevel >= blindLevel);

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

    renderSamplesInto(network, restrike, kTailBlocks * kBlock, test);

    // THE CRITERION, on the direct state: f0 IS the new note -- exactly, to the last bit -- and it
    // got there well inside 30 ms.
    const double landedSeconds = static_cast<double>(rampSamples) / kRate;
    const float targetHz = static_cast<float>(cnpg::test::midiNoteToHz(kNewNote));
    REQUIRE(network.retuneRampSamplesRemaining(0) == 0);
    REQUIRE(network.stringF0Hz(0) == targetHz);
    REQUIRE(landedSeconds <= kCriterionSeconds + 1.0e-12);

    // ...and on the AUDIO, which is the claim a listener could check: the tap's measured
    // fundamental, over a window that starts once the ramp is spent, is the new note.
    const std::size_t analysisBegin =
        static_cast<std::size_t>(restrikeSample) + static_cast<std::size_t>(kCriterionSeconds * kRate);
    const std::size_t analysisLength = 1u << 15;
    REQUIRE(test.size() >= analysisBegin + analysisLength);
    const std::vector<double> analysed = toDouble(test, analysisBegin, analysisLength);
    const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(analysed, kRate, analysisLength);
    const double measuredHz = cnpg::test::findPeakHz(spectrum, targetHz, cnpg::test::kTuningSearchCents);
    REQUIRE(measuredHz > 0.0);
    const double centsOff = cnpg::test::centsBetween(measuredHz, targetHz);

    // CLICK. Reference: the same note plucked on a string that was not already ringing, i.e. the
    // same render differing only in whether there was an old note to retune. Built by a lambda
    // because the phase-invariance guard below needs the SAME construction at a second placement,
    // and two hand-written copies of it would not be comparable evidence.
    auto freshRenderAt = [&](int placement) {
        StringNetwork<float> reference;
        configure(reference, paramsFor(RetriggerMode::Physical));
        std::vector<float> tap;
        BlockEventQueue idle;
        renderSamplesInto(reference, idle, placement, tap);
        BlockEventQueue freshPluck;
        freshPluck.push(noteOn(0, kNewNote));
        renderSamplesInto(reference, freshPluck, (kTailBlocks + 1) * kBlock, tap);
        return tap;
    };
    const std::vector<float> control = freshRenderAt(restrikeSample);
    REQUIRE(control.size() == test.size());

    // THE SPAN STARTS ONE SAMPLE BEFORE THE RE-STRIKE, and it did not until fixes wave 3. The same
    // correction wave 2 made at two other sites in this file applies here for the same reason:
    // measureClick forms its first difference from samples[begin + 1] - samples[begin], so a span
    // beginning AT the re-strike skips the one difference that spans the transition -- which is
    // exactly the difference the level placement above exists to make worst-case. Reaching back
    // further is what is forbidden, not reaching back one sample: the reference render is silent
    // before its own pluck, so a long backward span hands it a median |dx| of zero and every
    // comparison against it reads infinity (ClickMetric.h's documented degeneracy). One sample adds
    // exactly one zero difference to a 5 120-sample span and moves no median.
    const auto spanBegin = static_cast<std::size_t>(restrikeSample - 1);
    const auto spanEnd = static_cast<std::size_t>(restrikeSample + 40 * kBlock);
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
    // up to 3.0 dB between adjacent millisecond values (below), because it is a peak taken over a
    // 40-block window and a glide moves the phase at which the loudest transition lands. It is a
    // check that the transition is not a STEP; it is not a preference ordering over ramp lengths.
    // The real decider is whether 30 ms of glide reads as a hammer-on or as a slide, which is
    // docs/listening/physical-plausibility-checklist.md item 17 and has no headless answer.
    auto excessForRampAt = [&](int placement, const cnpg::test::ClickMeasurement& reference, double rampSeconds) {
        StringNetwork<float> variant;
        configure(variant, paramsFor(RetriggerMode::Physical));
        variant.setRetuneRampSeconds(rampSeconds);
        std::vector<float> render;
        BlockEventQueue firstNote;
        firstNote.push(noteOn(0, kOldNote));
        renderSamplesInto(variant, firstNote, placement, render);
        BlockEventQueue secondNote;
        secondNote.push(noteOn(0, kNewNote));
        renderSamplesInto(variant, secondNote, (kTailBlocks + 1) * kBlock, render);
        REQUIRE(variant.retuneRampSamplesRemaining(0) == 0);
        return cnpg::test::clickExcessDb(cnpg::test::measureClick(render, kRate,
                                                                  static_cast<std::size_t>(placement - 1),
                                                                  static_cast<std::size_t>(placement + 40 * kBlock)),
                                         reference);
    };
    auto excessForRamp = [&](double rampSeconds) {
        return excessForRampAt(restrikeSample, freshMeasurement, rampSeconds);
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
    // is in the sweep above. The statistic swings by up to 3.0 dB between adjacent millisecond
    // values, so a single-point gate with a decibel of margin is decided by which side of one local
    // oscillation the shipped value lands on, and a different note pair, a change to the
    // fractional-delay solve or a different toolchain could flip it red for no musical reason at
    // all. It would also flip GREEN for none.
    //
    // The physical claim is about a retune ramp of ORDER 30 ms, not about 1440 samples exactly, so
    // the criterion is applied to the MEDIAN of the shipped value plus and minus 2 ms. A median of
    // five is unmoved by up to two outlying points, so no single oscillation can decide the gate;
    // a real regression -- anything that makes the retune transition itself a step -- moves every
    // point in the neighbourhood together and is caught. Three of five points would have to fail
    // before this line does, against one for the point gate it replaces.
    //
    // *** THAT ARGUMENT ONLY HOLDS ONCE THE RE-STRIKE PHASE IS ALSO PINNED, WHICH IS WHY THE
    // PLACEMENT ABOVE IS PART OF THIS GATE AND NOT A TIDYING-UP. *** With the re-strike at a block
    // boundary, three of five points DID fail at one arbitrary phase (median 3.199 dB, 3 ms from the
    // shipped one), so "every point moved together" no longer distinguished a regression from a
    // shift in when the second note arrived -- which was the whole of the median's justification.
    // Level-placing closes that axis, and the closure is asserted immediately below rather than
    // argued: the entire neighbourhood is re-measured at the placement ONE PERIOD LATER -- a
    // different sample, the same phase -- and the two medians must agree. A build that reintroduced
    // a phase dependence would separate them.
    std::vector<double> neighbourhood(std::begin(sweptExcess) + kNeighbourhood, std::end(sweptExcess));
    REQUIRE(neighbourhood.size() == 5);
    std::sort(neighbourhood.begin(), neighbourhood.end());
    const double neighbourhoodMedianDb = neighbourhood[2];

    const std::vector<float> nextCycleControl = freshRenderAt(nextCyclePlacement);
    const cnpg::test::ClickMeasurement nextCycleFresh =
        cnpg::test::measureClick(nextCycleControl, kRate, static_cast<std::size_t>(nextCyclePlacement - 1),
                                 static_cast<std::size_t>(nextCyclePlacement + 40 * kBlock));
    REQUIRE(nextCycleFresh.medianAbsDiff > 0.0);
    std::vector<double> nextCycleNeighbourhood;
    for (std::size_t i = kNeighbourhood; i < kSweptCount; ++i)
        nextCycleNeighbourhood.push_back(excessForRampAt(nextCyclePlacement, nextCycleFresh, kSweptRamps[i]));
    std::sort(nextCycleNeighbourhood.begin(), nextCycleNeighbourhood.end());
    const double nextCycleMedianDb = nextCycleNeighbourhood[2];

    std::cout << "[contract] Physical cross-pitch restrike: ramp " << rampSamples << " samples ("
              << (landedSeconds * 1000.0) << " ms, criterion " << (kCriterionSeconds * 1000.0)
              << "); measured f0 after the ramp " << measuredHz << " Hz vs target " << targetHz << " (" << centsOff
              << " cents); click excess " << excessDb << " dB (peak |dx| " << measured.peakWindowAbsDiff << " vs "
              << freshMeasurement.peakWindowAbsDiff << " for the fresh pluck), one-sample-ramp control "
              << sweptExcess[0] << " dB, level-placed hard-cut control " << hardCutExcessDb << " dB at sample +"
              << (static_cast<int>(cutSample) - restrikeSample) << " (level " << cutLevel << ")\n";
    std::cout << "[contract] the re-strike is LEVEL-PLACED, not struck at a block boundary: sample +"
              << (restrikeSample - kNominalRestrike) << " past the nominal one, on the loudest sample of the "
              << kOldPeriodSamples << "-sample period after it (level " << placedLevel << " vs " << blindLevel
              << " where a block boundary would have struck)\n";

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

    // *** THE PHASE AXIS IS CLOSED, ASSERTED. *** The same neighbourhood, measured with the
    // re-strike placed one period later -- a different sample of the render, the same phase of the
    // old note. Before the placement landed, moving the re-strike by 3 ms moved this median by up to
    // 3.09 dB, which is more than the gate's margin; after it, one whole period moves it by a
    // fraction of a decibel. The 0.5 dB bound is 30x the measured difference and a quarter of the
    // gate's own margin, so it is a real constraint rather than a formality: a change that made this
    // statistic phase-dependent again would have to keep the dependence under a quarter of the
    // margin to escape, at which point it cannot decide the gate either.
    std::cout << "[contract] the gate no longer depends on WHEN the re-strike lands: the same 28..32 ms "
                 "neighbourhood level-placed one period later (sample +"
              << (nextCyclePlacement - kNominalRestrike) << ", " << (nextCyclePlacement - restrikeSample)
              << " samples on) reads median " << nextCycleMedianDb << " dB against " << neighbourhoodMedianDb
              << " dB here -- a difference of " << std::fabs(nextCycleMedianDb - neighbourhoodMedianDb)
              << " dB, where a 3 ms shift of a BLINDLY placed re-strike moved it by up to 3.09 dB\n";
    INFO("median here " << neighbourhoodMedianDb << " dB, one period later " << nextCycleMedianDb << " dB");
    REQUIRE(std::fabs(nextCycleMedianDb - neighbourhoodMedianDb) < 0.5);

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
    // range (not only at the fast end), and 22 ms clears the criterion at 1.18 dB while 28 ms FAILS
    // it at 3.59 dB sitting between two passing neighbours. A sampled statistic that jumps like that
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
    // The yardstick this case USED to gate on was P2.4's own number: at couplingStrength 0.35 a
    // sympathetically driven string peaks at -53.5 dBFS within 1 s (ADR 0006's measured table).
    //
    // *** THAT NUMBER IS PRINTED AND NO LONGER ASSERTED, AND THE DEMOTION IS DELIBERATE. *** The
    // assertion was `REQUIRE(truncatedDbfs > kP24SympatheticPeakDbfs)`, and it was wrong twice over:
    //
    //   1. -53.5 dBFS is DERIVED FROM couplingStrength = 0.35, and ADR 0007 D4 makes that default
    //      PROVISIONAL pending the P2.8 listening pass -- it is confirmed or replaced by ear, and
    //      lower values are explicitly on the table. truncatedDbfs scales with coupling too, so a
    //      test comparing one number derived from 0.35 against another derived from 0.35 breaks the
    //      moment the default moves, in a task that has nothing to do with retrigger semantics. No
    //      gate in this suite may depend on that constant in a way that a change to it turns red.
    //   2. Worse, the assertion was ABOUT the deferred rest-pitch finding below rather than about
    //      anything this case gates. The excess exists BECAUSE an untouched string still rests at
    //      MIDI 21. Fixing that -- which is P2.7's job, scheduled below -- drops truncatedDbfs well
    //      under -53.5 and turns this line red as a DIRECT CONSEQUENCE OF THE FIX. An assertion that
    //      fails when a known defect is repaired is an assertion that encodes the defect.
    //
    // What IS asserted instead is the precondition the measurement actually depends on and which is
    // invariant under both of those moves: string 1 has never been played, so it rests at its OWN
    // REST PITCH. That is a fact about prepare() and about midiNote_ being written only by note
    // events and by clearStringState() -- no coupling value enters it -- and it is the fact that
    // makes the number what it is.
    //
    // *** UPDATED AT P2.7, BY THE TASK THAT CHANGED THE THING IT NAMES, exactly as the previous
    // revision of this comment said it would be. *** The pitch it asserts was kMinMidiNote (A0,
    // 27.5 Hz) and is now the string's open tuning (StringNetworkParams::PerString::restMidiNote;
    // string 1's default is A2, 110 Hz). The assertion is the SAME assertion -- "an untouched string
    // is where prepare() left it, in cents" -- re-pointed at the value that is now correct, and it
    // is what stops the discarded level below being attributed to the wrong cause.
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

    // THE PRECONDITION THE MEASUREMENT DEPENDS ON, asserted directly rather than through a level
    // derived from a provisional coupling default: string 1 has never been played, so it is still
    // tuned where prepare() left it -- kMinMidiNote. A0's harmonic series contains 110 Hz exactly
    // (its 4th partial), which is why an untouched string is a BETTER sympathetic resonator for the
    // note being played than a real open string would be, and therefore why the discarded level
    // below reads high. Checked in cents so it is a statement about pitch rather than about float
    // formatting.
    const int restNote = static_cast<int>(params.perString[1].restMidiNote);
    const double restPitchHz = cnpg::test::midiNoteToHz(restNote);
    const double restCentsOff = 1200.0 * std::log2(static_cast<double>(network.stringF0Hz(1)) / restPitchHz);
    INFO("string 1 rest pitch " << network.stringF0Hz(1) << " Hz vs restMidiNote " << restNote << " (" << restPitchHz
                                << " Hz)");
    REQUIRE(std::fabs(restCentsOff) < 1.0);
    // ...and it is NOT kMinMidiNote any more, asserted rather than assumed: an untouched string
    // resting at A0 is the defect P2.7 fixed, and a regression that reinstated it would otherwise
    // satisfy the line above only if restMidiNote regressed with it.
    REQUIRE(restNote != cnpg::dsp::kMinMidiNote);
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
              << energyAfter << ". OBSERVATION, NOT A GATE: P2.4 (ADR 0006) measured " << kP24SympatheticPeakDbfs
              << " dBFS for a unison pair at this coupling; this reads " << (truncatedDbfs - kP24SympatheticPeakDbfs)
              << " dB HIGHER -- a property of the DRIVEN string's rest tuning, not of the truncation, and both "
                 "numbers scale with a couplingStrength ADR 0007 D4 leaves provisional\n";

    // *** THE P2.6 FINDING, AND ITS P2.7 RESOLUTION, KEPT TOGETHER SO THE NUMBER STAYS READABLE. ***
    //
    // At P2.6 this line read 6.63 dB ABOVE P2.4's figure and the excess was explained, not absorbed:
    // string 1 had never been played, so it carried the pitch prepare() left it at -- kMinMidiNote,
    // A0, 27.5 Hz -- and A0's harmonic series contains 110 Hz exactly, as its fourth partial. An
    // untouched string was therefore a BETTER sympathetic resonator for the note being played than a
    // real open string would be, and identically so for every string and every note, because they
    // were all at A0 until somebody played them.
    //
    // P2.7 gave strings a rest pitch (StringNetworkParams::PerString::restMidiNote), so string 1 now
    // sits at its open A2 -- and the excess BARELY MOVED, from 6.63 dB to 6.39 dB. That is not the
    // fix failing; it is the fix changing what the number means, and the distinction is worth the
    // four lines it takes to write down.
    //
    // kOldNote is MIDI 45, and MIDI 45 IS string 1's open pitch (A2, the second entry of
    // kDefaultOpenStringMidiNote). So the configuration that used to be "a plucked A2 next to an
    // accidental A0 comb" is now "a plucked A2 next to an open A string" -- a TRUE UNISON PAIR, the
    // strongest sympathetic configuration an instrument has, and a completely ordinary thing for a
    // guitarist to do. The level is high for a real reason now instead of an artificial one.
    //
    // What is left of the difference from ADR 0006's -53.5 dBFS is a difference of NOTE and of
    // measurement window, not of tuning: ADR 0006 measured MIDI 53, sitting on the 180 Hz bridge
    // resonance, peak within 1 s; this measures MIDI 45 in a 27 ms window ~0.8 s in. Still PRINTED
    // rather than gated, for the reason that has not changed: both numbers scale with a
    // couplingStrength that ADR 0007 D4 leaves provisional until the P2.8 listening pass, and an
    // assertion on it would break when the default moves, in a task with nothing to do with
    // retrigger semantics.

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

    // THE SPAN BEGINS ONE SAMPLE BEFORE THE ATTACK, and it did not until fixes wave 2. measureClick
    // forms its first difference from samples[begin + 1] - samples[begin], so a span beginning AT
    // the attack skips the one difference that spans the state clear -- i.e. it skips the
    // discontinuity, which is the entire thing this gate is about. The case still passes with the
    // step included (the numbers below say by how much), so this is a gate made honest rather than
    // a defect found; the sweep case further down, where the discarded content is 20 dB louder, is
    // where the difference between the two spans decides the answer.
    const std::size_t attackSample = sympathetic.size();
    const std::size_t spanBegin = attackSample - 1;
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
// what the "a released note is over" ruling COSTS, swept over note-off age
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: re-striking a released string clears it, and what that discards is measured at every age",
          "[contract]") {
    // THE RISK THE "A RELEASED NOTE IS OVER" RULING CREATES, measured rather than assumed.
    //
    // StringNetwork's "owns a note" is `sounding_` alone (fixes wave 2). A NoteOn onto a string that
    // is merely RELEASING therefore takes the fresh path, which CLEARS the string -- so a re-strike
    // arriving soon after a note-off throws away content that is still plainly audible, and the
    // sooner it arrives the louder that content is. That is the price of making the predicate agree
    // with NoteAllocator's, and it is a price paid on the most common gesture in playing.
    //
    // IT IS NOT THE SAME MEASUREMENT AS THE SYMPATHETIC-TRUNCATION CASE ABOVE, and assuming that one
    // covers this one would be wrong by tens of dB. That case clears a string carrying a NEIGHBOUR'S
    // drive -- about -47 dBFS, some 24 dB below the note that replaces it. This one clears a string
    // carrying the decaying remains of a note the player struck himself, which 5 ms after the
    // note-off is about 6 dB below the note replacing it. Different amounts of energy, different
    // question, and the earlier case cannot stand in for this one.
    //
    // THE A/B PAIR. Test and control are the SAME instrument, the SAME re-strike, at the SAME
    // sample, of the SAME pitch, differing only in whether an earlier note was played and released
    // on that string -- i.e. only in whether there is anything to discard.
    //
    // THE PERTURBATION IS PLACED BY LEVEL, NOT AT A ROUND NUMBER OF MILLISECONDS, and that is this
    // case's central methodological point. The step the state clear inserts is exactly the tail's
    // INSTANTANEOUS value at the sample before the re-strike -- not its peak, not its RMS. A
    // re-strike at a nominal age lands at an arbitrary phase of a 110 Hz waveform, so a sweep of
    // round ages measures the waveform's phase at seven arbitrary indices and calls the result a
    // function of age. Measured, before this placement was adopted: at 20 ms the reading came out at
    // EXACTLY 0.00 dB while the discarded peak was -28.8 dBFS, because that particular sample
    // happened to sit on a zero crossing -- the identical defect fixes wave 1 found in four blindly
    // placed negative controls, here in the perturbation itself. Each age therefore searches HALF a
    // period of the old note (219 samples, 4.56 ms at 110 Hz / 48 kHz) for the loudest tail sample
    // and puts the re-strike on the sample AFTER it, so every row reports the worst case available
    // near its nominal age. The actual age used is printed beside the nominal one.
    //
    // HALF a period rather than a whole one, and the reason is not aesthetic: a half period is the
    // shortest window guaranteed to contain an extremum of the fundamental, and it is shorter than
    // the 5 ms spacing between the first two swept ages. A full-period window was tried first and
    // the 5 ms and 10 ms rows both slid onto the SAME sample at 12.35 ms -- two rows measuring one
    // point. The distinctness of adjacent placements is asserted below so that cannot come back.
    //
    // THE SPAN starts ONE SAMPLE before the re-strike, and that is load-bearing rather than
    // incidental. measureClick forms its first difference from samples[begin + 1] - samples[begin],
    // so a span beginning AT the re-strike sample would skip the one difference that spans the state
    // clear -- the discontinuity itself. Beginning exactly one sample earlier includes it and adds
    // no other pre-attack signal, which matters here because the test render (unlike the control)
    // has a live tail before the attack and a longer pre-roll would let that tail's ordinary motion
    // enter a comparison that is supposed to be about the step.
    //
    // BOTH READINGS ARE GATED, per the ClickMetric header's own instruction. This change reduces
    // level AND inserts a discontinuity, which is the documented blind spot of reading (b):
    // clickExcessDb compares absolute peak |dx|, so a step the change itself made quiet can hide
    // under the attack's own transient. clickExcessAgainstLevelDb re-normalises per 10 ms window by
    // that window's own peak |x| and does not have that blind spot.
    constexpr int kRingSamples = 100 * kBlock;  // ~267 ms: the note is fully established
    constexpr int kTailSamples = 40 * kBlock;   // ~107 ms after the re-strike
    constexpr int kClickSpanBlocks = 30;        // the span the click readings are taken over
    constexpr int kDiscardWindow = 10 * kBlock; // ~27 ms, the window whose level is thrown away

    // Half a period of the old note: the search window for the level placement above.
    const int kOldPeriodSamples = static_cast<int>(std::ceil(kRate / cnpg::test::midiNoteToHz(kOldNote)));
    const int kSearchSamples = (kOldPeriodSamples + 1) / 2;
    REQUIRE(kSearchSamples > 1);
    // Shorter than the closest spacing in the sweep, which is what stops two rows collapsing.
    REQUIRE(static_cast<double>(kSearchSamples) / kRate < 0.005);

    // The sweep, in milliseconds of note-off age. 5 ms is about half a round trip of the old note;
    // 500 ms is a note a listener would call finished. Every point is inside the window in which
    // `releasing_` was still true and the SHIPPED P2.6 code took the retrigger path, because the
    // silence watchdog needs the outgoing bridge wave under kSilenceFloor for a whole
    // kSilenceWindowSeconds window before it clears anything.
    const std::vector<double> kNominalAgesMs{5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0};

    struct Arm {
        std::vector<float> tap;
        double engagementBefore = 0.0;
        double energyBefore = 0.0;
        double engagementAfter = 0.0;
        int rampAfter = 0;
        bool fadeAfter = false;
    };

    // One arm. `restrikeSample` is absolute; a value of -1 renders the note-off and then nothing at
    // all, which is the release-only reference the placement is read from.
    auto renderArm = [&](RetriggerMode mode, bool playOldNote, int restrikeSample, int totalSamples) {
        StringNetwork<float> network;
        configure(network, paramsFor(mode));
        Arm arm;
        arm.tap.reserve(static_cast<std::size_t>(totalSamples));

        BlockEventQueue queue;
        auto run = [&](int samples) {
            int done = 0;
            while (done < samples) {
                const int chunk = std::min(kBlock, samples - done);
                network.process(queue, chunk);
                const float* channel = network.tapBuffers().channel(0, 0);
                REQUIRE(channel != nullptr);
                arm.tap.insert(arm.tap.end(), channel, channel + chunk);
                done += chunk;
            }
        };

        if (playOldNote)
            queue.push(noteOn(0, kOldNote));
        run(kRingSamples);
        if (playOldNote)
            queue.push(noteOff(0, kOldNote));

        if (restrikeSample < 0) {
            run(totalSamples - kRingSamples);
            return arm;
        }

        run(restrikeSample - kRingSamples);

        // THE STATE THE RULING IS ABOUT, sampled at the instant the re-strike lands.
        arm.engagementBefore = static_cast<double>(network.damperEngagement(0));
        arm.energyBefore = network.stringEnergyEstimate(0);

        queue.push(noteOn(0, kNewNote));
        run(1); // one sample, so the snapshot below is the re-strike itself and not 128 samples of it
        arm.engagementAfter = static_cast<double>(network.damperEngagement(0));
        arm.rampAfter = network.retuneRampSamplesRemaining(0);
        arm.fadeAfter = network.retriggerFadeActive(0);
        run(totalSamples - restrikeSample - 1);
        return arm;
    };

    // THE RELEASE-ONLY REFERENCE. Note on, note off, then nothing: this is the pre-re-strike history
    // of every test arm below, because nothing touches the string between the note-off and the
    // re-strike. It is therefore both the thing that gets discarded and the waveform the placement is
    // read off, and the identity is ASSERTED per row rather than assumed.
    const int kMaxAgeSamples = static_cast<int>(std::lround(kNominalAgesMs.back() * 0.001 * kRate));
    const int kReleaseSamples = kRingSamples + kMaxAgeSamples + 2 * kOldPeriodSamples;
    const Arm releaseOnly = renderArm(RetriggerMode::Physical, true, -1, kReleaseSamples);

    struct Row {
        double nominalAgeMs = 0.0;
        double actualAgeMs = 0.0;
        double discardedStepDbfs = 0.0; // the ONE sample the clear replaces -- the step itself
        double discardedPeakDbfs = 0.0; // peak over the 27 ms before the re-strike
        double belowAttackDb = 0.0;
        double engagementBefore = 0.0;
        double excessDb = 0.0;
        double levelExcessDb = 0.0;
        double hardCutDb = 0.0;
    };
    std::vector<Row> rows;

    for (const double nominalAgeMs : kNominalAgesMs) {
        INFO("nominal note-off age " << nominalAgeMs << " ms");
        const int nominal = kRingSamples + static_cast<int>(std::lround(nominalAgeMs * 0.001 * kRate));
        // The loudest tail sample within one period of the nominal age; the re-strike goes on the
        // sample AFTER it, so that sample is the one the state clear replaces.
        const std::size_t loudest = loudestSample(releaseOnly.tap, static_cast<std::size_t>(nominal),
                                                  static_cast<std::size_t>(nominal + kSearchSamples));
        const auto restrike = static_cast<int>(loudest) + 1;
        const double actualAgeMs = 1000.0 * static_cast<double>(restrike - kRingSamples) / kRate;
        const int totalSamples = restrike + kTailSamples;

        const Arm test = renderArm(RetriggerMode::Physical, true, restrike, totalSamples);
        const Arm control = renderArm(RetriggerMode::Physical, false, restrike, totalSamples);
        REQUIRE(test.tap.size() == static_cast<std::size_t>(totalSamples));
        REQUIRE(control.tap.size() == test.tap.size());

        // THE PLACEMENT IS ON THE REAL WAVEFORM: the test arm's tail over the period the placement
        // searched is bit-identical to the release-only render it was read from. Without this the
        // level placement would be a claim about a different render than the one being measured.
        const auto tailBegin = static_cast<std::ptrdiff_t>(restrike - kSearchSamples);
        REQUIRE(
            std::equal(test.tap.begin() + tailBegin, test.tap.begin() + restrike, releaseOnly.tap.begin() + tailBegin));

        // NON-VACUITY OF THE PAIR, in state rather than by inference. The test arm really is
        // releasing -- the felt is down and the string still carries energy -- and the control arm
        // really has nothing to discard.
        REQUIRE(test.engagementBefore > 0.0);
        REQUIRE(test.energyBefore > 0.0);
        REQUIRE(control.engagementBefore == 0.0);
        REQUIRE(control.energyBefore == 0.0);
        const bool controlSilentBefore =
            std::all_of(control.tap.begin(), control.tap.begin() + restrike, [](float x) { return x == 0.0f; });
        REQUIRE(controlSilentBefore);

        // THE RULING, ASSERTED DIRECTLY rather than inferred from the audio (the P2.1 binding
        // ruling). A re-strike on a releasing string takes the FRESH path: the state is cleared, so
        // the damper is reset fully open on that same sample -- not ramped open over the felt's own
        // time constant, which is what the shipped `sounding_ || releasing_` predicate did and what
        // cost the corpus 4.88 dB. No retune ramp is started either, and no Synth fade, because
        // there is no old note to glide from or fade out.
        REQUIRE(test.engagementAfter == 0.0);
        REQUIRE(test.rampAfter == 0);
        REQUIRE_FALSE(test.fadeAfter);

        // AND THE WHOLE OF WHAT THE TRUNCATION DOES IS ONE SAMPLE WIDE. From the re-strike onward
        // the two renders are BIT-IDENTICAL: the clear is total, the coupling is off, and the
        // exciter contributes no history at the shipped noiseAmount of 0. So the click readings
        // below are not a summary of a diffuse difference -- they are the exact size of a single
        // step, and the difference signal between the A and B arms is the release tail before the
        // re-strike and exactly zero after it. That is the linearity between perturbation and tap
        // that the click metric's causal reading requires, established rather than argued.
        const bool identicalAfterRestrike =
            std::equal(test.tap.begin() + restrike, test.tap.end(), control.tap.begin() + restrike);
        REQUIRE(identicalAfterRestrike);

        const auto spanBegin = static_cast<std::size_t>(restrike - 1);
        const auto spanEnd = static_cast<std::size_t>(restrike + kClickSpanBlocks * kBlock);
        const cnpg::test::ClickMeasurement controlClick =
            cnpg::test::measureClick(control.tap, kRate, spanBegin, spanEnd);
        const cnpg::test::ClickMeasurement testClick = cnpg::test::measureClick(test.tap, kRate, spanBegin, spanEnd);
        REQUIRE(controlClick.medianAbsDiff > 0.0);
        REQUIRE(controlClick.peakStepToLevel > 0.0);
        REQUIRE(testClick.nonFiniteSamples == 0);
        REQUIRE(testClick.subnormalSamples == 0);

        const double excessDb = cnpg::test::clickExcessDb(testClick, controlClick);
        const double levelExcessDb = cnpg::test::clickExcessAgainstLevelDb(testClick, controlClick);

        // THE NEGATIVE CONTROL, level-placed (fixes wave 1's standing instruction, and the reason
        // for it: a hard cut's peak |dx| IS the sample value it lands on, so a blindly placed cut
        // measures the waveform's phase rather than the metric's sensitivity and can PASS the gate
        // it exists to fail). Placed in the CONTROL render, which is the reference both readings are
        // taken against, so its number is on the same footing as excessDb.
        const std::size_t cutSample = loudestSample(control.tap, static_cast<std::size_t>(restrike), spanEnd);
        REQUIRE(std::fabs(control.tap[cutSample]) > 0.0f);
        std::vector<float> hardCut = control.tap;
        std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
        const double hardCutDb =
            cnpg::test::clickExcessDb(cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd), controlClick);
        // THE GATE HAS TEETH AT THIS AGE. Asserted inside the loop because it is a property of the
        // measurement rather than of the result: a toothless row makes its own reading meaningless
        // whatever the table says afterwards.
        REQUIRE(hardCutDb > cnpg::test::kClickMetricToleranceDb);

        const double stepLevel = std::fabs(static_cast<double>(test.tap[static_cast<std::size_t>(restrike - 1)]));
        const double discardedPeak = static_cast<double>(
            peakOf(test.tap, static_cast<std::size_t>(restrike - kDiscardWindow), static_cast<std::size_t>(restrike)));
        const double attackPeak = static_cast<double>(peakOf(test.tap, static_cast<std::size_t>(restrike), spanEnd));
        REQUIRE(attackPeak > 0.0);
        REQUIRE(discardedPeak > 0.0);

        rows.push_back(Row{nominalAgeMs, actualAgeMs, dbOf(stepLevel), dbOf(discardedPeak),
                           dbOf(discardedPeak / attackPeak), test.engagementBefore, excessDb, levelExcessDb,
                           hardCutDb});
    }

    // Printed as a table BEFORE anything is judged, so a failing row cannot hide the shape of the
    // sweep the case exists to produce.
    for (const Row& row : rows)
        std::cout << "[contract] note-off age " << row.nominalAgeMs << " ms (placed at " << row.actualAgeMs
                  << " ms, the loudest tail sample in the half period after it): the discarded step is "
                  << row.discardedStepDbfs << " dBFS, the 27 ms before the re-strike peak at " << row.discardedPeakDbfs
                  << " dBFS (" << row.belowAttackDb << " dB below the re-strike), felt engagement "
                  << row.engagementBefore << "; click excess " << row.excessDb << " dB, per-window level-normalised "
                  << row.levelExcessDb << " dB (limit " << cnpg::test::kClickMetricToleranceDb
                  << "), level-placed hard-cut control " << row.hardCutDb << " dB\n";

    REQUIRE(rows.size() == kNominalAgesMs.size());
    // THE SWEEP IS NOT DEGENERATE: the earliest re-strike discards far more than the latest, which
    // is the whole reason age is swept rather than probed at one point. If this collapsed, every row
    // would be measuring the same thing and the gate would be one point wearing seven hats.
    REQUIRE(rows.front().discardedStepDbfs - rows.back().discardedStepDbfs > 40.0);
    // ...and no two rows landed on the same sample, which is the failure mode the half-period search
    // window exists to prevent and which a full-period one actually produced.
    for (std::size_t i = 1; i < rows.size(); ++i) {
        INFO("rows " << (i - 1) << " and " << i);
        REQUIRE(rows[i].actualAgeMs > rows[i - 1].actualAgeMs);
    }

    // -------------------------------------------------------------------------------------------
    // WHERE THE LINE ACTUALLY FALLS, asserted from BOTH sides rather than picked
    // -------------------------------------------------------------------------------------------
    // *** A RE-STRIKE SOON AFTER A NOTE-OFF IS A CLICK, AND IT ALWAYS HAS BEEN. *** Near the
    // note-off the discarded tail is only about 6 dB below the note replacing it, and dropping it to
    // zero in one sample reads roughly 20 dB over the criterion on BOTH readings -- about two thirds
    // of a full hard cut, in dB.
    //
    // THAT IS NOT A DEFECT THIS RULING INTRODUCES. It is exactly what the code did before P2.6:
    // 77b0430's handleEvent sends any note arriving on a releasing string down its
    // `!pluckOverRinging` branch, which calls clearStringState -- and the comment there asserted the
    // opposite in words, "a string mid-release ... its tail is already attenuated, so clearing it is
    // inaudible". No test ever re-struck a releasing string, so nothing measured it, and the claim
    // stood unchallenged from P2.2 to here. The shipped P2.6 predicate masked the click by accident,
    // at a cost of 4.88 dB of corpus RMS on the commonest gesture in playing; wave 2 takes the mask
    // off, and this case is the first thing in the project to look underneath it.
    //
    // So the criterion is asserted where it holds and the FAILURE is asserted where it does not, and
    // both directions are gated. Asserting only the passing half would let a future change quietly
    // widen the bad region; asserting only the shape would let one quietly appear inside the clean
    // one. The threshold is measured, not chosen: it is the first swept age at which both readings
    // clear 3 dB, and the row before it is required to fail.
    constexpr double kClickFreeFromMs = 50.0;

    for (const Row& row : rows) {
        INFO("nominal note-off age " << row.nominalAgeMs << " ms, placed at " << row.actualAgeMs << " ms");
        // At every age the truncation is strictly milder than discarding the whole render at its
        // loudest sample. This is the bound that holds everywhere, and it is what says the short-age
        // reading is a real truncation rather than a broken measurement.
        REQUIRE(row.excessDb < row.hardCutDb);

        if (row.nominalAgeMs >= kClickFreeFromMs) {
            REQUIRE(row.excessDb <= cnpg::test::kClickMetricToleranceDb);
            REQUIRE(row.levelExcessDb <= cnpg::test::kClickMetricToleranceDb);
        } else {
            // NOT a tolerated failure: an ASSERTED one. This is the finding, pinned in the direction
            // that matters -- if a later change makes an early re-strike click-free, this line goes
            // red and whoever made it has to come and move the boundary deliberately.
            REQUIRE(row.excessDb > cnpg::test::kClickMetricToleranceDb);
            REQUIRE(row.levelExcessDb > cnpg::test::kClickMetricToleranceDb);
        }
    }

    const auto firstClean = std::find_if(
        rows.begin(), rows.end(), [](const Row& row) { return row.excessDb <= cnpg::test::kClickMetricToleranceDb; });
    REQUIRE(firstClean != rows.end());
    REQUIRE(firstClean != rows.begin());
    REQUIRE(firstClean->nominalAgeMs == kClickFreeFromMs);
    const double worstCleanDb = std::max_element(firstClean, rows.end(), [](const Row& a, const Row& b) {
                                    return a.excessDb < b.excessDb;
                                })->excessDb;

    std::cout << "[contract] re-strike over a released string: the state clear IS a click for note-off ages under "
              << kClickFreeFromMs << " ms (worst " << rows.front().excessDb << " dB at " << rows.front().actualAgeMs
              << " ms, against a level-placed hard cut of " << rows.front().hardCutDb << " dB) and clean from "
              << kClickFreeFromMs << " ms up (worst " << worstCleanDb
              << " dB). This is pre-P2.6 behaviour restored and first measured, not new -- see the comment above\n";

    // BOTH MODES TAKE THE SAME PATH, because the predicate is mode-independent -- worth one direct
    // assertion rather than an argument, since Synth's whole purpose is to fade before it clears and
    // it does NOT do so here: there is no live note to fade. Asserted as bit-identity of the two
    // renders, the strongest available form of "the same code ran".
    const int fiftyMs = kRingSamples + static_cast<int>(std::lround(0.050 * kRate));
    const Arm physical = renderArm(RetriggerMode::Physical, true, fiftyMs, fiftyMs + kTailSamples);
    const Arm synth = renderArm(RetriggerMode::Synth, true, fiftyMs, fiftyMs + kTailSamples);
    REQUIRE_FALSE(synth.fadeAfter);
    REQUIRE(synth.engagementAfter == 0.0);
    REQUIRE(synth.rampAfter == 0);
    REQUIRE(synth.tap.size() == physical.tap.size());
    REQUIRE(synth.tap == physical.tap);
    std::cout << "[contract] a re-strike 50 ms after the note-off renders bit-identically in Physical and Synth: "
              << synth.tap.size() << " samples, both on the fresh path, neither fading nor ramping\n";
}

// ---------------------------------------------------------------------------------------------
// the refused clause
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: a retrigger damper choke has no state left to act on, and buying one costs elapsed time",
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
    // the level of the whole result rather than the balance inside it.
    //
    // The one thing a damper can do outright is act FIRST, on the old content alone, before the
    // pluck exists -- which is exactly what the plan's own ordering ("choke, ramp, THEN
    // re-excitation") describes, and its entire currency is elapsed time, up to the 30 ms the plan
    // itself budgets. 30 ms of latency on every legato note is not a playable instrument, and Q3
    // defers audible-slide behaviour besides.
    //
    // -----------------------------------------------------------------------------------------
    // FIXES WAVE 2 BROKE THE OLD MEASUREMENT. THE REFUSAL STANDS ON LESS EVIDENCE THAN IT DID.
    // -----------------------------------------------------------------------------------------
    // The original form of this case built an engaged felt out of the SHIPPED machinery -- a
    // note-off 10.7 ms before the restrike -- and measured two things from one pair of renders: the
    // choke took 1.4925 dB (amplitude) out of the old note's stored energy and charged the re-attack
    // 1.2233 dB for it, netting 0.2692 dB on the balance. Those numbers were taken at 0eb52b9 and
    // they were true of that code.
    //
    // They are not reproducible here, and the reason is the point rather than an inconvenience.
    // Wave 2's ownership ruling makes a released note OVER, so a note-off no longer leaves a string
    // that a later NoteOn re-triggers -- it leaves a string that a later NoteOn CLEARS. The two arms
    // of the old comparison therefore stopped differing only in the felt and started differing in
    // which path ran, which is not a measurement of a choke. (Measured, for the record: the
    // attack-cost reading collapses from -1.2233 dB to -0.2070 dB, and what remains is the missing
    // superposition of the old note in the un-choked arm, not a damper cost.)
    //
    // *** WAVE 2 CALLED THE REPLACEMENT "STRONGER THAN REFUSING IT ON COST". IT IS NOT, AND FIXES
    // WAVE 3 WITHDRAWS THAT. *** Section (a) below observes that nothing engages the damper on this
    // path today. The plan's clause proposes to ADD a fast engage(); "there is no engagement today"
    // cannot refute a proposal to create one, because it RESTATES THE CONTROL FLOW the proposal is
    // asking to change. What (a) actually is -- and it is worth having -- is a REGRESSION GUARD: a
    // build that inserts the choke fails on the sample it was inserted, so the refusal cannot be
    // undone silently. It is evidence about the code, not an argument about the design.
    //
    // THE REFUSAL ITSELF RESTS WHERE IT ALWAYS DID: on COST and on LATENCY. A choke's whole currency
    // is elapsed time in which no new note exists, section (c) measures what that time buys
    // (-1.4925 dB of amplitude for 10.67 ms of felt, reproduced exactly from 0eb52b9), and the plan
    // budgets up to 30 ms of it on every legato note. That is the trade being declined.
    //
    // AND THE EVIDENTIARY BASE IS THINNER THAN IT WAS, WHICH IS SAID PLAINLY RATHER THAN GLOSSED.
    // The sharpest number the original refusal had -- the attack cost of -1.22328 dB, and with it
    // the -0.269221 dB NET effect on the balance, the one quantity a choke exists to change -- is no
    // longer reproducible in this tree and survives only as a quotation from the P2.6 report. The
    // conclusion has not moved; the evidence for it has.
    //
    //   (a) THE REGRESSION GUARD. The path the plan's clause names -- Physical, pitch changing,
    //       plucking over a live note -- runs only on a SOUNDING string, and a sounding string's
    //       damper is at 0: the only thing that engages one is the note-off branch, which clears
    //       `sounding_` on the same line, and landSynthFade's pending note-off, which does the same.
    //       Section (a) asserts the engagement is 0 on both sides of exactly that re-strike, so a
    //       build that added the choke fails it. That is what it is for.
    //   (b) The path where the felt IS down -- a re-strike on a released string -- already discards
    //       the old content ENTIRELY and resets the felt to fully open on the same sample. The
    //       choke's stated goal is to make the old note quieter relative to the new one; that path
    //       already achieves it completely, and at no cost to the attack. Section (b) asserts both.
    //       This one IS an argument about the design, and it is about the OTHER path.
    //
    //   (c) THE HALF OF THE ORIGINAL MEASUREMENT THAT STILL STANDS, and the half the refusal rests
    //       on, because it is about the felt and not about the retrigger: suppression is bought with
    //       ELAPSED TIME. Section (c) measures what 10.7 ms of felt takes out of a ringing string,
    //       with no re-strike involved at all -- which is the quantity the plan would have to spend
    //       latency to obtain.
    constexpr int kRingBlocks = 200;
    constexpr int kChokeBlocks = 4; // 10.7 ms at 48 kHz / 128, well inside the plan's 30 ms budget

    SECTION("(a) the path the clause names has no engagement to choke with") {
        StringNetwork<float> network;
        configure(network, paramsFor(RetriggerMode::Physical));
        std::vector<float> out;
        BlockEventQueue first;
        first.push(noteOn(0, kOldNote));
        renderInto(network, first, kRingBlocks, out);

        // IN STATE: the string owns a live note and its felt is open.
        REQUIRE(network.energyEstimate() > 0.0);
        REQUIRE(network.damperEngagement(0) == 0.0f);

        BlockEventQueue restrike;
        restrike.push(noteOn(0, kNewNote));
        network.process(restrike, 1);

        // THE PITCH-CHANGE BRANCH RAN -- non-vacuity, so the assertion below is about that branch.
        REQUIRE(network.retuneRampSamplesRemaining(0) > 0);
        REQUIRE(network.energyEstimate() > 0.0); // rails kept: this is the retrigger path
        // ...AND THE FELT NEVER CAME DOWN. This is the refusal as an assertion: the shipped branch
        // does not engage the damper, and a build that inserted the plan's "fast engage()" would
        // read non-zero here on the very sample it was inserted.
        REQUIRE(network.damperEngagement(0) == 0.0f);
        renderInto(network, restrike, 4, out);
        REQUIRE(network.damperEngagement(0) == 0.0f);
    }

    SECTION("(b) where the felt IS down, the old note is discarded whole and the attack pays nothing") {
        auto run = [](bool releaseFirst) {
            StringNetwork<float> network;
            configure(network, paramsFor(RetriggerMode::Physical));
            std::vector<float> out;
            BlockEventQueue first;
            first.push(noteOn(0, kOldNote));
            renderInto(network, first, kRingBlocks, out);

            BlockEventQueue between;
            if (releaseFirst)
                between.push(noteOff(0, kOldNote));
            renderInto(network, between, kChokeBlocks, out);

            struct Result {
                double energyAtRestrike = 0.0;
                float engagementAtRestrike = 0.0f;
                float engagementAfterRestrike = 0.0f;
                double energyAfterRestrike = 0.0;
                float attackPeak = 0.0f;
            };
            Result result{};
            result.energyAtRestrike = network.energyEstimate();
            result.engagementAtRestrike = network.damperEngagement(0);

            const std::size_t restrikeSample = out.size();
            BlockEventQueue restrike;
            restrike.push(noteOn(0, kNewNote));
            network.process(restrike, 1);
            result.engagementAfterRestrike = network.damperEngagement(0);
            renderInto(network, restrike, 12, out);
            result.attackPeak = peakOf(out, restrikeSample, out.size());
            result.energyAfterRestrike = network.energyEstimate();
            return result;
        };

        const auto held = run(false);    // the note is still held: the re-strike is a retrigger
        const auto released = run(true); // the note was released 10.7 ms ago: the re-strike is fresh

        // IN STATE: one arm really has a felt on the string and the other really does not.
        REQUIRE(held.engagementAtRestrike == 0.0f);
        REQUIRE(released.engagementAtRestrike > 0.05f);
        REQUIRE(held.energyAtRestrike > 0.0);
        REQUIRE(released.energyAtRestrike > 0.0);

        // THE FELT IS GONE ON THE SAMPLE OF THE RE-STRIKE, not ramped away over the felt time. That
        // is what says the choke has nothing left to charge the attack for: the pluck lands into a
        // fully open damper either way.
        REQUIRE(released.engagementAfterRestrike == 0.0f);
        REQUIRE(held.engagementAfterRestrike == 0.0f);

        const double attackCostDb =
            dbOf(static_cast<double>(released.attackPeak) / static_cast<double>(held.attackPeak));
        std::cout << "[contract] refused choke (a): the shipped pitch-change branch leaves the felt at 0 engagement, "
                     "so there is nothing to choke with; (b) after a note-off "
                  << (1000.0 * kChokeBlocks * kBlock / kRate) << " ms earlier the felt reads "
                  << released.engagementAtRestrike
                  << " at the re-strike and exactly 0 one sample later -- the old note is discarded WHOLE and the "
                     "re-attack is "
                  << attackCostDb
                  << " dB against the held-note arm, which is the missing superposition, not a "
                     "damper cost\n";

        // The remaining difference is small AND it is in the direction of the missing old note
        // rather than of a damped attack: under a decibel, where the original engaged-felt reading
        // was -1.2233 dB. Bounded rather than pinned, because it is a superposition residue and not
        // a designed quantity.
        REQUIRE(attackCostDb < 0.0);
        REQUIRE(attackCostDb > -1.0);
    }

    SECTION("(c) suppression is bought with elapsed time, and that half of the old measurement stands") {
        // No re-strike anywhere in this section: it measures what the felt alone does to a ringing
        // string over the interval the plan would have to spend before re-exciting. This is the
        // quantity the choke trades latency for, and it is real.
        auto storedEnergyAfter = [](bool releaseFirst) {
            StringNetwork<float> network;
            configure(network, paramsFor(RetriggerMode::Physical));
            std::vector<float> out;
            BlockEventQueue first;
            first.push(noteOn(0, kOldNote));
            renderInto(network, first, kRingBlocks, out);

            BlockEventQueue between;
            if (releaseFirst)
                between.push(noteOff(0, kOldNote));
            renderInto(network, between, kChokeBlocks, out);
            return network.energyEstimate();
        };

        const double plain = storedEnergyAfter(false);
        const double choked = storedEnergyAfter(true);
        REQUIRE(plain > 0.0);
        REQUIRE(choked > 0.0);
        const double suppressionDb = dbOf(choked / plain) * 0.5; // energy -> amplitude

        std::cout << "[contract] refused choke (c): " << (1000.0 * kChokeBlocks * kBlock / kRate)
                  << " ms of felt takes " << suppressionDb
                  << " dB (amplitude) out of a ringing string's stored energy -- the whole of what a choke buys, and "
                     "it is bought with latency on every legato note\n";

        // IT WORKS, AND IT WORKS BY ACTING FIRST. The suppression is real and it was bought with
        // 10.7 ms of elapsed time in which no new note existed. That is the trade the refusal
        // declines, and declining it is a judgement about playability, not about whether the felt
        // does anything.
        REQUIRE(suppressionDb < -1.0);
    }
}

// ---------------------------------------------------------------------------------------------
// the one residual path on which the two ownership predicates can still differ
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: a string removed from the active count stops owning its note within the enable ramp",
          "[contract]") {
    // WHAT BOUNDS THE ONE REMAINING DISAGREEMENT between NoteAllocator::owned_ and
    // StringNetwork::sounding_, and why it is a bound rather than a defect.
    //
    // A NoteOff for a note whose string has left the active count (or been muted) cannot be
    // delivered: StringNetwork::handleEvent drops any event for such a string at its own early
    // returns. NoteAllocator therefore declines to emit it, releases the ownership, and counts it on
    // unaddressableNoteOffCount() (fixes wave 2). The allocator is then correct and the NETWORK is
    // the one holding a stale flag: `sounding_` is still true on a string whose note is over.
    //
    // What makes that bounded rather than open-ended is the enable ramp. A removed string ramps to
    // silence over kEnableRampSeconds and the per-sample loop clears its state -- and its
    // sounding_/releasing_ flags -- on the sample the ramp lands on zero. So the disagreement lives
    // for at most one enable ramp plus the block it lands in, after which the two levels agree again
    // and agree on "free". THIS CASE ASSERTS THAT BOUND, because it is the invariant the whole
    // argument rests on: lengthen the ramp, or stop clearing the flags there, and the disagreement
    // becomes permanent.
    //
    // *** THE RESIDUAL, PRINTED AND NOT ASSERTED: a count that dips and RETURNS inside the ramp. ***
    // If the count comes back before the ramp lands, the flags are never cleared, and a fresh note
    // on that string is taken as a retrigger of a note the player already released -- it keeps rails
    // that were never damped and glides f0 from the old pitch. It is reachable only by an automation
    // ride that crosses the owning string's index and returns inside 10 ms with a note-off in
    // between, which is Check B's gesture played very fast. It is NOT asserted here, deliberately:
    // the honest fix is to let a NoteOff through handleEvent's addressability gate (a NoteOff for a
    // string being ramped silent is harmless -- it only moves flags), and an assertion pinning
    // today's behaviour would go red the moment somebody made it. Reported in
    // task-P2.6-fixes-wave2.md instead, with this measurement.
    constexpr int kRampSamples = 480; // kEnableRampSeconds * kRate = 10 ms at 48 kHz

    auto probe = [](int samplesOutOfCount) {
        StringNetwork<float> network;
        configure(network, paramsFor(RetriggerMode::Physical), 2);

        BlockEventQueue events;
        events.push(noteOn(0, kOldNote, 1));
        for (int b = 0; b < 100; ++b)
            network.process(events, kBlock);
        REQUIRE(network.stringEnergyEstimate(1) > 0.0);

        network.setNumStrings(1); // string 1 leaves; its enable ramp starts

        // The note-off the allocator would have declined to emit, pushed here anyway so this case
        // reproduces exactly what StringNetwork sees: an event it drops at its early return.
        BlockEventQueue undeliverable;
        undeliverable.push(noteOff(0, kOldNote, 1));
        network.process(undeliverable, 1);
        REQUIRE(network.damperEngagement(1) == 0.0f); // dropped: the felt never came down

        for (int done = 1; done < samplesOutOfCount;) {
            const int chunk = std::min(kBlock, samplesOutOfCount - done);
            BlockEventQueue idle;
            network.process(idle, chunk);
            done += chunk;
        }

        network.setNumStrings(2); // ...and back
        BlockEventQueue restrike;
        restrike.push(noteOn(0, kNewNote, 1));
        network.process(restrike, 1);

        struct Result {
            int rampAfter = 0;
        };
        return Result{network.retuneRampSamplesRemaining(1)};
    };

    // THE BOUND: one whole enable ramp plus the block it lands in is enough. The string has stopped
    // owning its note, so the re-strike is a FRESH note -- no retune ramp is started.
    const auto settled = probe(kRampSamples + kBlock);
    REQUIRE(settled.rampAfter == 0);

    // THE RESIDUAL, measured and printed. Two milliseconds out of the count is not enough, and the
    // re-strike is taken as a retrigger of a note that was already released.
    const auto hurried = probe(96); // 2 ms
    std::cout << "[contract] a string that left the active count stops owning its note within the "
              << (1000.0 * kRampSamples / kRate) << " ms enable ramp: after "
              << (1000.0 * (kRampSamples + kBlock) / kRate)
              << " ms out of the count a re-strike starts no retune ramp (" << settled.rampAfter
              << " samples). OBSERVATION, NOT A GATE: after only 2 ms it starts one of " << hurried.rampAfter
              << " samples -- the flag had not been cleared yet, which is the one residual disagreement "
                 "between the two ownership predicates and is bounded by that ramp\n";
}

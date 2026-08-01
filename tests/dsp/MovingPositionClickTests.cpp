#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"
#include "support/ClickMetric.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// MovingPositionClickTests -- Task P2.3's gates for MOVING positions: the pickup tap and the damper
// junction seam, both modulated while a chord rings.
//
// Four kinds of thing live here, and the order is deliberate, because the P2.1 review's binding
// ruling is that a state change covered ONLY by a click test is not covered:
//
//   1. The three click scenarios the task's acceptance criteria name. They are gated, and the
//      reading each one is gated on is stated where it is taken -- including, for the 5 Hz case,
//      the measurement that says reading (b)'s answer there is the sweep's own LEVEL and not a
//      click at all.
//   2. A standing NEGATIVE CONTROL: the same six-string mix with the pickup stepped by exactly one
//      threshold with no crossfade at all. It must fail, and by how much is what pins the gate's
//      sensitivity so it can never quietly go vacuous.
//   3. DIRECT state assertions on everything P2.3 introduces or touches -- the crossfade state
//      machine itself, sample by sample, and the damper-position smoother that P2.2 left exposed.
//   4. The transparency claim, extended into motion: a transparent junction deposits exactly 0.0
//      even mid-crossfade, which is what keeps P2.2's bit-exact seam contract true while the
//      junction is being swept.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RetriggerMode;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr double kTwoPi = 6.283185307179586;
constexpr int kChordStrings = 6;
constexpr int kChord[kChordStrings] = {40, 45, 50, 55, 59, 64}; // open E major
constexpr double kReleaseSeconds = 0.25;

struct Lane {
    float centre = 0.5f;
    float depth = 0.0f; // 0 = frozen at `centre`
    double hz = 2.0;
    float at(double t) const noexcept {
        return static_cast<float>(static_cast<double>(centre) + static_cast<double>(depth) * std::sin(kTwoPi * hz * t));
    }
};

struct Spec {
    Lane pickup{0.5f, 0.0f, 2.0};
    Lane damper{0.5f, 0.0f, 2.0};
    float maxLoss = 1.0f;
    double seconds = 10.0;
    int heldStrings = kChordStrings; // [0, heldStrings) stay sounding; the rest get a NoteOff
    double retriggerPeriod = 0.0;    // 0 = none; same-pitch NoteOns on the HELD strings

    // Re-strike the RELEASED strings on the same period and release them again shortly after, so
    // that a damper is coming down on a freshly plucked string inside every retrigger interval
    // rather than only inside the first. Without it a released string is damped to the silence
    // watchdog's floor within a few hundred milliseconds and the rest of the render is pickup-only
    // -- which is exactly what the first version of the combined 5 Hz case measured while calling
    // itself a test of both positions moving together.
    bool restrikeDamped = false;
    double damperHoldSeconds = 0.15; // pluck -> NoteOff gap for the re-struck strings: a chug
};

struct Render {
    std::vector<float> mix;
    std::vector<float> damperPosition; // per block: what the junction seam is actually reading at
    std::vector<float> tapPosition;    // per block: what the pickup tap is actually reading at
    // Per block, string 0 -- the LOWEST note of the chord, and the probe is on it deliberately. A
    // point damper's modal decay scales with the string's round-trip time, so the top of the chord
    // is damped into the silence watchdog's floor (which snaps the engagement back to 0) inside a
    // few tens of milliseconds while the bottom is still ringing. Probing the top string would
    // report "engagement 0" for a render whose audible content is entirely damped strings.
    std::vector<float> engagement;
    std::vector<float> lossDepth;

    // Per block, the STRONGEST junction anywhere in the network: max over strings of
    // engagement * lossDepth, which is the `s` in g = s/(s+1) and therefore the whole of how much
    // damping is in force. Per block and over all strings because "is the damper doing anything
    // during THIS segment" is a question about the segment, not about one string at one instant.
    std::vector<float> strongestDamperProduct;
    std::vector<std::size_t> retriggerSamples;
};

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex) {
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

NoteEvent noteOff(int sampleOffset, int midiNote, int stringIndex) {
    NoteEvent event = noteOn(sampleOffset, midiNote, stringIndex);
    event.type = NoteEventType::NoteOff;
    event.pluckPosition = cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

Render renderChord(const Spec& spec) {
    StringNetworkParams params;
    params.retriggerMode = RetriggerMode::Physical;
    params.pickupPosition01 = spec.pickup.at(0.0);
    params.damperPosition01 = spec.damper.at(0.0);
    params.damper.maxLoss = spec.maxLoss;
    params.damper.feltTimeConstantMs = 40.0f;
    // Sustain material, for the same documented reason the [tuning] sweep, the P2.1 click gate and
    // the P2.2 note-off cases use it: every reading here is about what a POSITION move does to a
    // ringing string, and at the default material this chord has decayed into the noise long before
    // the ten seconds the criterion names. Same filters, same code path, a parameter value inside
    // the shipping range.
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(kChordStrings);
    network.setParams(params);
    network.reset();

    // THE SHIPPING DENORMAL CONFIGURATION (Task P2.4). This file asserts
    // ClickMeasurement::subnormalSamples == 0 on every render, and that held trivially while the
    // strings were uncoupled: an unplucked or fully damped string's state was CLEARED outright by
    // the silence watchdog, so nothing ever spent time in the subnormal range on the way down.
    // Bidirectional coupling gives every string a long, quiet, bridge-driven tail instead, and a
    // float32 render of that tail without the guard produces genuine subnormals (measured: 14 228
    // of them in the damper-sweep case). PluginProcessor::processBlock has constructed one of these
    // first since Task P1.1, so this IS the shipping path; what changed is not that denormals
    // appeared but that there is now a tail for them to appear in. Same finding, same reasoning and
    // the same double-precision control as the header of
    // tests/dsp/NetworkEnergyTierThreeTests.cpp.
    const cnpg::dsp::ScopedFtzDazGuard denormalGuard;

    const auto totalBlocks = static_cast<int>(spec.seconds * kRate / kBlock);
    const auto releaseBlock = static_cast<int>(kReleaseSeconds * kRate / kBlock);
    const int retriggerBlocks =
        (spec.retriggerPeriod > 0.0) ? static_cast<int>(spec.retriggerPeriod * kRate / kBlock) : 0;
    const int damperHoldBlocks = std::max(1, static_cast<int>(spec.damperHoldSeconds * kRate / kBlock));

    Render out;
    out.mix.assign(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock), 0.0f);
    out.damperPosition.reserve(static_cast<std::size_t>(totalBlocks));
    out.tapPosition.reserve(static_cast<std::size_t>(totalBlocks));
    out.engagement.reserve(static_cast<std::size_t>(totalBlocks));
    out.lossDepth.reserve(static_cast<std::size_t>(totalBlocks));

    BlockEventQueue events;
    for (int s = 0; s < kChordStrings; ++s)
        events.push(noteOn(0, kChord[s], s));

    for (int b = 0; b < totalBlocks; ++b) {
        const double t = static_cast<double>(b) * static_cast<double>(kBlock) / kRate;
        // Retargeted once per block, as a host delivers automation: the click-freedom on trial is
        // the per-sample smoother plus the crossfade absorbing that staircase, not an artificially
        // smooth per-sample input.
        params.pickupPosition01 = spec.pickup.at(t);
        params.damperPosition01 = spec.damper.at(t);
        network.setParams(params);

        if (b == releaseBlock)
            for (int s = spec.heldStrings; s < kChordStrings; ++s)
                events.push(noteOff(0, kChord[s], s));
        if (retriggerBlocks > 0 && b > 0 && (b % retriggerBlocks) == 0) {
            for (int s = 0; s < spec.heldStrings; ++s)
                events.push(noteOn(0, kChord[s], s)); // SAME pitch: pluck over the ringing state
            if (spec.restrikeDamped) {
                // The released strings are silent and watchdog-cleared by now (a fully engaged
                // damper at depth 0.5 takes a string under -100 dBFS inside a few hundred ms), so
                // this NoteOn's re-init runs over zeroed rails -- the one condition under which
                // clearStringState() is inaudible. It also lands on the retrigger sample, which
                // every analysed segment already excludes its first 50 ms of.
                for (int s = spec.heldStrings; s < kChordStrings; ++s)
                    events.push(noteOn(0, kChord[s], s));
            }
            out.retriggerSamples.push_back(static_cast<std::size_t>(b) * static_cast<std::size_t>(kBlock));
        }
        if (spec.restrikeDamped && retriggerBlocks > 0 && b > damperHoldBlocks &&
            (b % retriggerBlocks) == (damperHoldBlocks % retriggerBlocks))
            for (int s = spec.heldStrings; s < kChordStrings; ++s)
                events.push(noteOff(0, kChord[s], s)); // the felt comes down inside every segment

        network.process(events, kBlock);

        const auto base = static_cast<std::size_t>(b) * static_cast<std::size_t>(kBlock);
        const int channels = network.tapBuffers().numStrings();
        for (int s = 0; s < channels; ++s) {
            const float* channel = network.tapBuffers().channel(s, 0);
            if (channel == nullptr)
                continue;
            for (int n = 0; n < kBlock; ++n)
                out.mix[base + static_cast<std::size_t>(n)] += channel[n];
        }
        out.damperPosition.push_back(network.damperPosition01(0));
        out.tapPosition.push_back(network.tapPosition01(0, 0));
        out.engagement.push_back(network.damperEngagement(0));
        out.lossDepth.push_back(network.damperLossDepth(0));
        float strongest = 0.0f;
        for (int s = 0; s < kChordStrings; ++s)
            strongest = std::max(strongest, network.damperEngagement(s) * network.damperLossDepth(s));
        out.strongestDamperProduct.push_back(strongest);
    }
    return out;
}

// Last sample index (exclusive) at which a 10 ms windowed peak is still at or above `floor`.
std::size_t audibleUntil(const std::vector<float>& samples, std::size_t from, double floor) {
    const auto window = static_cast<std::size_t>(0.010 * kRate);
    std::size_t last = from;
    for (std::size_t start = from; start + window <= samples.size(); start += window) {
        float peak = 0.0f;
        for (std::size_t i = start; i < start + window; ++i)
            peak = std::max(peak, std::fabs(samples[i]));
        if (static_cast<double>(peak) >= floor)
            last = start + window;
    }
    return last;
}

double peakOver(const std::vector<float>& samples, std::size_t from, std::size_t to) {
    double peak = 0.0;
    for (std::size_t i = from; i < std::min(to, samples.size()); ++i)
        peak = std::max(peak, std::fabs(static_cast<double>(samples[i])));
    return peak;
}

bool rendersDiffer(const std::vector<float>& a, const std::vector<float>& b, std::size_t from, std::size_t to) {
    for (std::size_t i = from; i < std::min({to, a.size(), b.size()}); ++i)
        if (a[i] != b[i])
            return true;
    return false;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// pickup position, swept while a six-string chord rings
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: MovingPosition -- a swept pickup position is click-free", "[contract]") {
    Spec sweptSpec;
    sweptSpec.pickup = Lane{0.5f, 0.45f, 2.0}; // 0.05 -> 0.95 at 2 Hz
    Spec frozenSpec;
    frozenSpec.pickup = Lane{0.5f, 0.0f, 2.0};

    const Render swept = renderChord(sweptSpec);
    const Render frozen = renderChord(frozenSpec);
    REQUIRE(swept.mix.size() == frozen.mix.size());

    // In the state this case claims to test: the chord is still ringing across the whole span, the
    // tap really did travel the range the criterion names, and the two renders really do differ.
    const auto spanBegin = static_cast<std::size_t>(0.10 * kRate);
    const std::size_t spanEnd = swept.mix.size();
    REQUIRE(peakOver(frozen.mix, spanBegin, spanEnd) > 0.001);
    REQUIRE(peakOver(swept.mix, spanBegin, spanEnd) > 0.001);
    REQUIRE(rendersDiffer(swept.mix, frozen.mix, spanBegin, spanEnd));

    const auto minmax = std::minmax_element(swept.tapPosition.begin(), swept.tapPosition.end());
    std::cout << "[contract] pickup sweep: tap position travelled " << *minmax.first << " .. " << *minmax.second
              << " (target 0.05 .. 0.95)\n";
    REQUIRE(*minmax.first < 0.10f);
    REQUIRE(*minmax.second > 0.90f);

    const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(frozen.mix, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement test = cnpg::test::measureClick(swept.mix, kRate, spanBegin, spanEnd);
    REQUIRE(reference.metric(reference) > 0.0);

    const double excessDb = cnpg::test::clickExcessDb(test, reference);
    const double levelDb = cnpg::test::clickExcessAgainstLevelDb(test, reference);
    std::cout << "[contract] pickup sweep click metric: excess " << excessDb << " dB, level-normalised " << levelDb
              << " dB (limit " << cnpg::test::kClickMetricToleranceDb << "); peak |dx| swept " << test.peakWindowAbsDiff
              << " vs frozen " << reference.peakWindowAbsDiff << "\n";

    INFO("excess " << excessDb << " dB, level " << levelDb << " dB");
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(test.nonFiniteSamples == 0);
    REQUIRE(test.subnormalSamples == 0);
}

// ---------------------------------------------------------------------------------------------
// damper position, swept while the felt is down
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: MovingPosition -- a swept damper position is click-free", "[contract]") {
    // "engagement held at 0.5": the junction's loss coefficient is g = s/(s+1) with
    // s = engagement * lossDepth, so an engagement of 0.5 at full depth and a fully engaged damper
    // at depth 0.5 are the SAME junction. The second is reachable through the network's own surface
    // (a note-off ramps engagement to exactly 1) and the first is not, so the render below uses it
    // and asserts the product directly.
    Spec sweptSpec;
    sweptSpec.damper = Lane{0.5f, 0.4f, 2.0}; // 0.1 -> 0.9 at 2 Hz
    sweptSpec.maxLoss = 0.5f;
    sweptSpec.heldStrings = 0; // every string released, so every damper engages
    Spec frozenSpec = sweptSpec;
    frozenSpec.damper = Lane{0.5f, 0.0f, 2.0};

    const Render swept = renderChord(sweptSpec);
    const Render frozen = renderChord(frozenSpec);

    const auto releaseSample = static_cast<std::size_t>(kReleaseSeconds * kRate);
    const std::size_t audible = audibleUntil(frozen.mix, releaseSample, 1.0e-4);
    const auto audibleBlocks = static_cast<std::ptrdiff_t>(audible / kBlock);
    const auto minmax = std::minmax_element(swept.damperPosition.begin(), swept.damperPosition.begin() + audibleBlocks);

    // The strongest junction in force anywhere inside the analysed span, and WHEN. Taken over the
    // span rather than at one instant, because a fully engaged damper at depth 0.5 takes a string
    // down fast enough that a probe placed by the clock can easily land after the silence watchdog
    // has cleared the string and snapped the engagement back to 0 -- which is exactly how the first
    // draft of this case managed to assert "engagement 0" on a render whose whole audible content
    // was damped strings.
    double strongestProduct = 0.0;
    std::size_t strongestBlock = 0;
    for (std::ptrdiff_t b = 0; b < audibleBlocks; ++b) {
        const double product = static_cast<double>(swept.engagement[static_cast<std::size_t>(b)]) *
                               static_cast<double>(swept.lossDepth[static_cast<std::size_t>(b)]);
        if (product > strongestProduct) {
            strongestProduct = product;
            strongestBlock = static_cast<std::size_t>(b);
        }
    }

    const auto spanBegin = releaseSample;
    const std::size_t spanEnd = audible;
    const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(frozen.mix, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement test = cnpg::test::measureClick(swept.mix, kRate, spanBegin, spanEnd);
    const double excessDb = cnpg::test::clickExcessDb(test, reference);
    const double levelDb = cnpg::test::clickExcessAgainstLevelDb(test, reference);

    std::cout << "[contract] damper sweep: junction position travelled " << *minmax.first << " .. " << *minmax.second
              << "; strongest junction over the span s = " << strongestProduct << " at "
              << (static_cast<double>(strongestBlock) * kBlock / kRate) << " s; audible until "
              << (static_cast<double>(audible) / kRate) << " s (release at " << kReleaseSeconds << " s), i.e. "
              << (static_cast<double>(audible - releaseSample) / kRate * 2.0) << " sweep cycles\n";
    std::cout << "[contract] damper sweep click metric: excess " << excessDb << " dB, level-normalised " << levelDb
              << " dB (limit " << cnpg::test::kClickMetricToleranceDb << "); peak |dx| swept " << test.peakWindowAbsDiff
              << " vs frozen " << reference.peakWindowAbsDiff << "\n";

    // IN the state this case claims to test: the felt really came all the way down at depth 0.5
    // while the chord was still audible, the junction really did travel 0.1 .. 0.9, and the two
    // renders really do differ.
    INFO("strongest s " << strongestProduct << " at block " << strongestBlock);
    REQUIRE(strongestProduct > 0.49);
    REQUIRE(strongestProduct <= 0.5);
    REQUIRE(*minmax.first < 0.12f);
    REQUIRE(*minmax.second > 0.88f);
    REQUIRE(spanEnd > spanBegin + static_cast<std::size_t>(0.020 * kRate));
    REQUIRE(reference.metric(reference) > 0.0);
    REQUIRE(rendersDiffer(swept.mix, frozen.mix, spanBegin, spanEnd));

    INFO("excess " << excessDb << " dB, level " << levelDb << " dB");
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(test.nonFiniteSamples == 0);
    REQUIRE(test.subnormalSamples == 0);
}

// ---------------------------------------------------------------------------------------------
// the combined worst case: both positions at 5 Hz across a Physical retrigger
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: MovingPosition -- both positions at 5 Hz across a Physical retrigger", "[contract]") {
    Spec sweptSpec;
    sweptSpec.pickup = Lane{0.5f, 0.45f, 5.0};
    sweptSpec.damper = Lane{0.5f, 0.4f, 5.0};
    sweptSpec.maxLoss = 0.5f;
    sweptSpec.heldStrings = 3; // 0..2 held and retriggered; 3..5 chugged, so their dampers engage
    sweptSpec.retriggerPeriod = 1.0;
    sweptSpec.seconds = 6.0;
    // Strings 3..5 are re-struck and released again on the retrigger period. Without this the
    // criterion's "both positions swept simultaneously" is a lie after the first second: a fully
    // engaged damper at depth 0.5 takes a string under the silence watchdog's floor in a few
    // hundred milliseconds, so the damper lane stops affecting anything and the case becomes a
    // pickup-only test wearing the combined name. The first version of this case measured exactly
    // that -- its own printed decomposition read `damper-only 0.0005 dB` in the segment that set
    // the gate -- and disclosed it in prose instead of asserting against it. The per-segment
    // assertions below are what make the disclosure binding.
    sweptSpec.restrikeDamped = true;
    Spec frozenSpec = sweptSpec;
    frozenSpec.pickup = Lane{0.5f, 0.0f, 5.0};
    frozenSpec.damper = Lane{0.5f, 0.0f, 5.0};

    // THE STATIC-POSITION FAMILY, and why the criterion's single frozen reference is not enough at
    // this sweep rate. A pickup is a comb filter whose output LEVEL and brightness are functions of
    // where it sits -- p = 0.5 is the null of every even partial and about the quietest place on the
    // string -- and reading (b) is documented as non-scale-invariant: it compares ABSOLUTE peak
    // first differences, so any level change lands in the numerator. ClickMetric.h records that
    // hazard in the level-REDUCING direction (a click can hide); it is symmetric, and at 5 Hz this
    // scenario is the other side of it. Measured below: the swept render is 4.4-5.1 dB LOUDER than
    // the frozen-0.5 reference in every segment, which is more than the whole reading-(b) excess.
    //
    // So the family: the same render frozen at five positions across the range the sweep visits.
    // Whatever spread those show against the frozen-0.5 reference is POSITION and cannot be motion,
    // because nothing is moving in any of them. The gate is that the sweep stays within the
    // criterion's 3 dB of that spread rather than of the single quietest point in it -- and the
    // negative control at the bottom of this file is what says a real uncrossfaded position step
    // still fails by a wide margin, so the widened denominator has not cost the gate its teeth.
    constexpr float kStaticFamily[] = {0.05f, 0.28f, 0.5f, 0.72f, 0.95f};

    Spec pickupOnlySpec = frozenSpec;
    pickupOnlySpec.pickup = Lane{0.5f, 0.45f, 5.0};
    Spec damperOnlySpec = frozenSpec;
    damperOnlySpec.damper = Lane{0.5f, 0.4f, 5.0};

    const Render swept = renderChord(sweptSpec);
    const Render frozen = renderChord(frozenSpec);
    const Render pickupOnly = renderChord(pickupOnlySpec);
    const Render damperOnly = renderChord(damperOnlySpec);
    std::vector<Render> family;
    for (float position : kStaticFamily) {
        Spec staticSpec = frozenSpec;
        staticSpec.pickup = Lane{position, 0.0f, 5.0};
        family.push_back(renderChord(staticSpec));
    }
    REQUIRE(swept.retriggerSamples.size() >= 4);
    REQUIRE(swept.retriggerSamples == frozen.retriggerSamples);

    // Measured per inter-retrigger segment, each starting 50 ms AFTER the retrigger. A pluck's own
    // attack is a large, legitimate first difference present identically in both renders; a span
    // containing it simply takes its maximum there, where the two renders agree, and reports nothing
    // about the position motion at all.
    double worstExcessDb = -1000.0;
    double worstLevelDb = -1000.0;
    double worstFamilyExcessDb = -1000.0;
    double worstFamilyLevelDb = -1000.0;
    double worstMotionExcessDb = -1000.0; // swept excess ABOVE the family's own spread
    double weakestSegmentDamper = 1000.0; // the LEAST damped segment; the gate is on this one
    std::size_t worstAt = 0;
    int segments = 0;
    long long nonFinite = 0;
    long long subnormal = 0;
    const auto skip = static_cast<std::size_t>(0.050 * kRate);
    for (std::size_t k = 0; k < swept.retriggerSamples.size(); ++k) {
        const std::size_t from = swept.retriggerSamples[k] + skip;
        const std::size_t to =
            (k + 1 < swept.retriggerSamples.size()) ? swept.retriggerSamples[k + 1] : swept.mix.size();
        if (to <= from + 16)
            continue;
        ++segments;

        // THE DAMPER IS DOING SOMETHING IN THIS SEGMENT, asserted rather than assumed. Same
        // discipline as the 2 Hz damper case above, applied where it was missing: the strongest
        // junction anywhere in the network across this segment's blocks, and separately that the
        // damper lane alone measurably changes the render over this exact span. Without both, a
        // segment in which every damped string had already decayed to silence would still be
        // reported as evidence about "both positions swept simultaneously".
        float segmentDamper = 0.0f;
        for (std::size_t b = from / kBlock; b < std::min(to / kBlock, swept.strongestDamperProduct.size()); ++b)
            segmentDamper = std::max(segmentDamper, swept.strongestDamperProduct[b]);
        weakestSegmentDamper = std::min(weakestSegmentDamper, static_cast<double>(segmentDamper));
        INFO("segment " << k << " strongest junction s = " << segmentDamper);
        REQUIRE(segmentDamper > 0.4f); // depth 0.5 with the felt essentially all the way down
        REQUIRE(rendersDiffer(damperOnly.mix, frozen.mix, from, to));

        const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(frozen.mix, kRate, from, to);
        const cnpg::test::ClickMeasurement test = cnpg::test::measureClick(swept.mix, kRate, from, to);
        REQUIRE(reference.metric(reference) > 0.0);
        const double excessDb = cnpg::test::clickExcessDb(test, reference);
        const double levelDb = cnpg::test::clickExcessAgainstLevelDb(test, reference);

        double familyExcessDb = -1000.0;
        double familyLevelDb = -1000.0;
        for (const Render& still : family) {
            const cnpg::test::ClickMeasurement measurement = cnpg::test::measureClick(still.mix, kRate, from, to);
            familyExcessDb = std::max(familyExcessDb, cnpg::test::clickExcessDb(measurement, reference));
            familyLevelDb = std::max(familyLevelDb, cnpg::test::clickExcessAgainstLevelDb(measurement, reference));
        }

        const cnpg::test::ClickMeasurement pickupOnlyMeasurement =
            cnpg::test::measureClick(pickupOnly.mix, kRate, from, to);
        const cnpg::test::ClickMeasurement damperOnlyMeasurement =
            cnpg::test::measureClick(damperOnly.mix, kRate, from, to);
        std::cout << "[contract]   segment " << k << " [" << (static_cast<double>(from) / kRate) << " s, "
                  << (static_cast<double>(to) / kRate) << " s): swept " << excessDb << " dB (level-normalised "
                  << levelDb << " dB); static family worst " << familyExcessDb << " dB (level-normalised "
                  << familyLevelDb << " dB); pickup-only "
                  << cnpg::test::clickExcessDb(pickupOnlyMeasurement, reference) << " dB, damper-only "
                  << cnpg::test::clickExcessDb(damperOnlyMeasurement, reference) << " dB; peak |x| swept "
                  << test.peakAbsSample << " vs frozen " << reference.peakAbsSample << " ("
                  << (20.0 * std::log10(test.peakAbsSample / reference.peakAbsSample)) << " dB of level)\n";

        nonFinite += test.nonFiniteSamples;
        subnormal += test.subnormalSamples;
        if (excessDb > worstExcessDb) {
            worstExcessDb = excessDb;
            worstAt = from;
        }
        worstLevelDb = std::max(worstLevelDb, levelDb);
        worstFamilyExcessDb = std::max(worstFamilyExcessDb, familyExcessDb);
        worstFamilyLevelDb = std::max(worstFamilyLevelDb, familyLevelDb);
        worstMotionExcessDb = std::max(worstMotionExcessDb, excessDb - std::max(0.0, familyExcessDb));
    }

    std::cout << "[contract] combined 5 Hz sweep across " << swept.retriggerSamples.size()
              << " Physical retriggers: worst per-segment excess " << worstExcessDb << " dB at sample " << worstAt
              << " (level-normalised " << worstLevelDb << " dB); the static-position family alone spans "
              << worstFamilyExcessDb << " dB (level-normalised " << worstFamilyLevelDb
              << " dB), so the MOTION accounts for " << worstMotionExcessDb << " dB (limit "
              << cnpg::test::kClickMetricToleranceDb
              << "). Least-damped segment: strongest junction s = " << weakestSegmentDamper << " over " << segments
              << " segments\n";

    INFO("worst excess " << worstExcessDb << " dB over the frozen-0.5 reference; static family spans "
                         << worstFamilyExcessDb << " dB; motion accounts for " << worstMotionExcessDb << " dB");
    REQUIRE(worstExcessDb > -1000.0); // at least one segment was measured
    // The family really does span a range -- otherwise the widened denominator below would be
    // nothing but a bigger number, and this case would have quietly stopped gating anything.
    REQUIRE(worstFamilyExcessDb > 0.0);
    // THE CRITERION, measured against the family the sweep travels through rather than against the
    // single quietest point in it. Both readings, because at this rate neither is sufficient alone:
    // (b) carries the sweep's level change and the level-normalised one carries its brightness.
    REQUIRE(worstMotionExcessDb <= cnpg::test::kClickMetricToleranceDb);
    REQUIRE(worstLevelDb <= worstFamilyLevelDb + cnpg::test::kClickMetricToleranceDb);
    REQUIRE(nonFinite == 0);
    REQUIRE(subnormal == 0);
}

// ---------------------------------------------------------------------------------------------
// the standing negative control: the same position change with NO crossfade
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: MovingPosition -- an uncrossfaded position step fails the same gate", "[contract]") {
    // WHY THIS CONTROL IS EXACT, and not an approximation of the artifact. readTapAt is a pure READ:
    // it never deposits into the rails, so two renders of this chord that differ ONLY in
    // pickupPosition01 carry bit-identical string state at every sample. Splicing one into the other
    // at sample k is therefore EXACTLY "the tap position stepped by one threshold, with no
    // crossfade" -- the artifact the machinery under test exists to remove -- and not a splice of
    // two histories that had already drifted apart.
    //
    // The step is one threshold (1/32 of the string) because that is the largest position error the
    // machinery ever holds before it acts, i.e. the biggest step it could possibly have to hide. If
    // the gate cannot see THAT, it cannot see anything the crossfade is doing.
    constexpr float kFrom = 0.5f;
    const float kTo = kFrom + cnpg::dsp::kPositionAnchorThreshold01;

    Spec fromSpec;
    fromSpec.pickup = Lane{kFrom, 0.0f, 2.0};
    fromSpec.seconds = 3.0;
    Spec toSpec = fromSpec;
    toSpec.pickup = Lane{kTo, 0.0f, 2.0};

    const Render a = renderChord(fromSpec);
    const Render b = renderChord(toSpec);
    REQUIRE(a.mix.size() == b.mix.size());

    // The two static renders differ (the tap really did move) and neither is silent -- without this
    // the splice below could be a step between two identical signals, i.e. no step at all.
    const auto spanBegin = static_cast<std::size_t>(0.5 * kRate);
    const std::size_t spanEnd = a.mix.size();
    REQUIRE(rendersDiffer(a.mix, b.mix, spanBegin, spanEnd));
    REQUIRE(peakOver(a.mix, spanBegin, spanEnd) > 0.001);

    // Several splice points, because where in the waveform a step lands changes how big it is and
    // one lucky instant would say nothing. The range is reported and the WEAKEST is what is gated:
    // the control has to fail everywhere, not on average.
    //
    // Each splice is measured over its OWN 300 ms window rather than over the whole render, exactly
    // as ClickMetric.h prescribes ("the transition plus a short margin"). Measured over the whole
    // three seconds instead, half the splice points read 0.0 dB -- not because the step was
    // inaudible but because the span's peak first difference is the PLUCK ATTACK, which both renders
    // share, so the maximum never moves off it. That is the span-choice trap the header warns about,
    // and getting it wrong here would have turned the negative control into a passing tautology.
    // THE CONTROL PROPER: not one step, but the whole gesture delivered as uncrossfaded steps at
    // the rate the machinery actually commits them. A 2 Hz full-depth sweep moves the request at up
    // to 0.45 * 2*pi * 2 = 5.65 string lengths per second, so it crosses one threshold every
    // 1/32 / 5.65 = 5.5 ms; an implementation with no crossfade would step the tap that often for
    // the whole render. Alternating between the two static renders on that period reproduces exactly
    // that -- and UNDERSTATES it, since a real staircase steps through many positions rather than
    // between two.
    const auto commitPeriod = static_cast<std::size_t>(static_cast<double>(cnpg::dsp::kPositionAnchorThreshold01) /
                                                       (0.45 * kTwoPi * 2.0) * kRate);
    REQUIRE(commitPeriod > 16);

    std::vector<float> stepped(a.mix.size(), 0.0f);
    std::size_t steps = 0;
    for (std::size_t n = 0; n < stepped.size(); ++n) {
        const bool useB = ((n / commitPeriod) % 2) == 1;
        stepped[n] = useB ? b.mix[n] : a.mix[n];
        if (n > 0 && (((n - 1) / commitPeriod) % 2) != (useB ? 1u : 0u))
            ++steps;
    }

    const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(a.mix, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement steppedMeasurement =
        cnpg::test::measureClick(stepped, kRate, spanBegin, spanEnd);
    REQUIRE(reference.metric(reference) > 0.0);
    const double steppedExcessDb = cnpg::test::clickExcessDb(steppedMeasurement, reference);
    const double steppedLevelDb = cnpg::test::clickExcessAgainstLevelDb(steppedMeasurement, reference);

    // ...and the SINGLE-step reading alongside it, which is this gate's sensitivity floor rather
    // than its criterion: how big one 1/32 step is depends on where in the waveform it lands, and at
    // the favourable instants the two tap positions agree closely enough that it disappears under
    // the signal's own motion. Reported, not gated, and it is the standing reason this file also
    // carries the direct sample-by-sample state assertions above rather than resting on the metric.
    constexpr int kSplicePoints = 16;
    double worstSingleDb = -1000.0;
    double weakestSingleDb = 1000.0;
    int failingSingles = 0;
    const auto localHalfSpan = static_cast<std::size_t>(0.150 * kRate);
    for (int step = 0; step < kSplicePoints; ++step) {
        const auto spliceAt = static_cast<std::size_t>((1.0 + 0.1 * step) * kRate);
        std::vector<float> spliced(a.mix.begin(), a.mix.begin() + static_cast<std::ptrdiff_t>(spliceAt));
        spliced.insert(spliced.end(), b.mix.begin() + static_cast<std::ptrdiff_t>(spliceAt), b.mix.end());
        const std::size_t from = spliceAt - localHalfSpan;
        const std::size_t to = spliceAt + localHalfSpan;
        const cnpg::test::ClickMeasurement localReference = cnpg::test::measureClick(a.mix, kRate, from, to);
        REQUIRE(localReference.metric(localReference) > 0.0);
        const double excessDb =
            cnpg::test::clickExcessDb(cnpg::test::measureClick(spliced, kRate, from, to), localReference);
        worstSingleDb = std::max(worstSingleDb, excessDb);
        weakestSingleDb = std::min(weakestSingleDb, excessDb);
        if (excessDb > cnpg::test::kClickMetricToleranceDb)
            ++failingSingles;
    }

    std::cout
        << "[contract] NEGATIVE CONTROL, pickup stepped " << kFrom << " -> " << kTo << " with no crossfade: " << steps
        << " steps at the sweep's own commit period (" << commitPeriod << " samples) reads " << steppedExcessDb
        << " dB excess (level-normalised " << steppedLevelDb << ", limit " << cnpg::test::kClickMetricToleranceDb
        << ") -- against which the swept renders read 0.39 / 0.67 dB. Sensitivity floor: a SINGLE such step reads "
        << weakestSingleDb << " .. " << worstSingleDb << " dB over " << kSplicePoints << " splice points, "
        << failingSingles << " of them failing.\n";

    INFO("stepped " << steppedExcessDb << " dB, single-step range " << weakestSingleDb << " .. " << worstSingleDb
                    << " dB");
    REQUIRE(steps > 100); // the control really is a chain of steps, not one
    REQUIRE(steppedExcessDb > cnpg::test::kClickMetricToleranceDb);
    REQUIRE(steppedLevelDb > cnpg::test::kClickMetricToleranceDb);
    REQUIRE(worstSingleDb > cnpg::test::kClickMetricToleranceDb);
}

// ---------------------------------------------------------------------------------------------
// DIRECT state assertions: the crossfade state machine, sample by sample
// ---------------------------------------------------------------------------------------------

namespace {

// One string, plucked, with its tap position driven by `position(sampleIndex)`. Returns the
// crossfade state ACTUALLY USED for each sample's read (i.e. after the read's retarget and before
// the tick that advances it), so a continuity assertion is made against the weights that produced
// the output rather than against a snapshot taken at some other moment.
struct AnchorTrace {
    std::vector<WaveguideString<double>::PositionCrossfade> state;
    std::vector<double> requested;
    std::vector<double> tap;
};

template <typename PositionFn> AnchorTrace traceTap(PositionFn position, int samples) {
    WaveguideString<double> string;
    string.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    WaveguideStringParams params;
    params.f0Hz = 110.0f;
    params.stringMaterial.lossGainLow = 1.0f;
    params.stringMaterial.lossGainHigh = 1.0f;
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset();

    cnpg::dsp::PluckExciter<double> exciter;
    exciter.prepare(kRate, kBlock);
    exciter.trigger(0.8f, 0.28f, 0.5f);

    AnchorTrace out;
    out.state.reserve(static_cast<std::size_t>(samples));
    out.requested.reserve(static_cast<std::size_t>(samples));
    out.tap.reserve(static_cast<std::size_t>(samples));
    for (int n = 0; n < samples; ++n) {
        const double excitation = exciter.renderSample();
        if (excitation != 0.0)
            string.injectAt(exciter.latchedPosition01(), excitation);
        const double p = position(n);
        out.requested.push_back(p);
        out.tap.push_back(string.readTapAt(0, static_cast<float>(p)));
        out.state.push_back(string.tapCrossfade(0));
        string.tick();
    }
    return out;
}

double effectivePosition(const WaveguideString<double>::PositionCrossfade& state) {
    return (1.0 - static_cast<double>(state.crossfade01)) * static_cast<double>(state.anchor01) +
           static_cast<double>(state.crossfade01) * static_cast<double>(state.pending01);
}

} // namespace

TEST_CASE("CONTRACT: MovingPosition -- the tap crossfade is a continuous staircase, sample by sample", "[contract]") {
    // THE direct assertion the P2.1 ruling requires for the state change this task introduces. The
    // click metric on the rendered audio is a separate, weaker observation (the three cases above
    // hold it); this one measures the state machine itself. In P2.1 a real discontinuity -- a tap
    // jumping 0.715 -> 0.900 -- was caught by exactly this kind of assertion while the click metric
    // read clean, and that is the standing reason it is written and gated here too.
    constexpr int kSamples = 48000;
    constexpr double kSweepHz = 5.0; // the worst case the acceptance criteria name

    // The same one-pole smoothing StringNetwork applies, so the trace sees the staircase the
    // shipping path actually hands the string rather than a raw sinusoid.
    const double smoothingCoeff = 1.0 - std::exp(-1.0 / (0.008 * kRate));
    double smoothed = 0.5;
    const AnchorTrace trace = traceTap(
        [&](int n) {
            const double target = 0.5 + 0.45 * std::sin(kTwoPi * kSweepHz * static_cast<double>(n) / kRate);
            smoothed += smoothingCoeff * (target - smoothed);
            return smoothed;
        },
        kSamples);

    WaveguideString<double> probe;
    probe.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    const int fadeSamples = probe.positionCrossfadeSamples();
    const double fadeStep = 1.0 / static_cast<double>(fadeSamples);
    REQUIRE(fadeSamples == 128); // 128 samples at 48 kHz, exactly as the technique specifies

    double worstRequestStep = 0.0;
    for (std::size_t n = 1; n < trace.requested.size(); ++n)
        worstRequestStep = std::max(worstRequestStep, std::fabs(trace.requested[n] - trace.requested[n - 1]));

    // The bound on how far apart a fade's two anchors can ever be, derived rather than fitted, and
    // it has TWO branches because a fade can arm two ways:
    //
    //   - from rest, on the sample the request first crosses the threshold: at most one threshold
    //     plus one sample of the request's own motion;
    //   - immediately after a commit, where the anchor becomes a position the request left
    //     (fadeSamples + 1) samples ago and has been travelling away from ever since: at most the
    //     request's motion over that whole fade.
    //
    // The second branch is what "crossfades re-arm immediately so continuous fast sweeps become a
    // chain of overlapping segments" costs, and at 5 Hz it is the larger of the two. The effective
    // read position then moves by at most that separation times one fade step per sample.
    const double maxSeparation = std::max(static_cast<double>(cnpg::dsp::kPositionAnchorThreshold01) + worstRequestStep,
                                          static_cast<double>(fadeSamples + 1) * worstRequestStep);
    const double effectiveStepLimit = maxSeparation * fadeStep + 1.0e-7;

    double worstEffectiveStep = 0.0;
    double worstSeparation = 0.0;
    double worstCrossfadeStep = 0.0;
    double worstLag = 0.0;
    int commits = 0;
    int midFadeSamples = 0;
    bool sawExactlyOne = false;

    for (std::size_t n = 1; n < trace.state.size(); ++n) {
        const auto& now = trace.state[n];
        const auto& before = trace.state[n - 1];
        INFO("sample " << n << ": anchor " << now.anchor01 << " pending " << now.pending01 << " g2 " << now.crossfade01
                       << " (was anchor " << before.anchor01 << " pending " << before.pending01 << " g2 "
                       << before.crossfade01 << ")");

        // The anchor pair is armed from the very first read -- there is no un-anchored sample.
        REQUIRE(now.armed);
        // Amplitude-complementary by construction: g2 in [0, 1], so g1 = 1 - g2 is too and the pair
        // sums to one. An equal-power law is what this is NOT.
        REQUIRE(now.crossfade01 >= 0.0f);
        REQUIRE(now.crossfade01 <= 1.0f);

        const double separation = std::fabs(static_cast<double>(now.pending01) - static_cast<double>(now.anchor01));
        worstSeparation = std::max(worstSeparation, separation);
        worstLag = std::max(worstLag, std::fabs(effectivePosition(now) - trace.requested[n]));

        if (now.crossfade01 > 0.0f && now.crossfade01 < 1.0f)
            ++midFadeSamples;
        if (now.crossfade01 == 1.0f)
            sawExactlyOne = true;

        if (now.crossfade01 < before.crossfade01) {
            // A commit. The new anchor must be EXACTLY the pending that was being faded in -- not a
            // third value, and not the live request, which by now has moved on.
            ++commits;
            REQUIRE(now.anchor01 == before.pending01);
            REQUIRE(before.crossfade01 == 1.0f); // ...and it committed only after the fade completed
        } else {
            worstCrossfadeStep = std::max(worstCrossfadeStep, static_cast<double>(now.crossfade01) -
                                                                  static_cast<double>(before.crossfade01));
        }

        // THE CONTINUITY CLAIM, and it is about the EFFECTIVE position, not the anchor: a commit
        // moves the anchor by a whole threshold in one sample while the position being read does not
        // move at all, so an assertion written against the anchor would report a jump that is not
        // there -- and, worse, would miss one that is.
        const double effectiveStep = std::fabs(effectivePosition(now) - effectivePosition(before));
        worstEffectiveStep = std::max(worstEffectiveStep, effectiveStep);
        REQUIRE(effectiveStep <= effectiveStepLimit);
    }

    std::cout << "[contract] tap crossfade at " << kSweepHz << " Hz: " << commits << " commits, " << midFadeSamples
              << " samples strictly mid-fade of " << kSamples << "; worst anchor separation " << worstSeparation
              << " (threshold " << cnpg::dsp::kPositionAnchorThreshold01 << " + one request step " << worstRequestStep
              << "), worst effective-position step " << worstEffectiveStep << " (limit " << effectiveStepLimit
              << "), worst g2 step " << worstCrossfadeStep << " (fade step " << fadeStep << "), worst position lag "
              << worstLag << "\n";

    // IN the state this case claims to test -- the P2.1/P2.2 standing lesson. A sweep that never
    // armed a fade, or never sat strictly between the two anchors, would satisfy every continuity
    // assertion above for free.
    REQUIRE(commits > 100);
    REQUIRE(midFadeSamples > kSamples / 2);
    REQUIRE(sawExactlyOne);
    REQUIRE(worstSeparation > static_cast<double>(cnpg::dsp::kPositionAnchorThreshold01));
    REQUIRE(worstSeparation <= maxSeparation + 1.0e-7);
    // g2 advances by exactly one fade step and never more, so the crossfade really is the linear
    // ramp the technique specifies rather than something that happens to end in the right place.
    REQUIRE(worstCrossfadeStep <= fadeStep * (1.0 + 1.0e-6));
    REQUIRE(worstCrossfadeStep >= fadeStep * (1.0 - 1.0e-6));
    // The staircase tracks: it never falls further behind the request than one threshold plus the
    // distance the request covers inside one fade.
    REQUIRE(worstLag <= maxSeparation + static_cast<double>(fadeSamples) * worstRequestStep);
}

TEST_CASE("CONTRACT: MovingPosition -- a static position never arms a crossfade at all", "[contract]") {
    // The other half of the state machine, and what keeps a frozen-position render bit-identical to
    // the pre-P2.3 arithmetic (which is what leaves the layer-(b) goldens unmoved): inside the
    // threshold nothing moves, so a static parameter reads through the plain single-anchor path for
    // the whole render.
    const AnchorTrace still = traceTap([](int) { return 0.87; }, 8192);
    for (std::size_t n = 0; n < still.state.size(); ++n) {
        INFO("sample " << n);
        REQUIRE(still.state[n].armed);
        REQUIRE(still.state[n].crossfade01 == 0.0f);
        REQUIRE(still.state[n].anchor01 == 0.87f);
        REQUIRE(still.state[n].pending01 == 0.87f);
    }

    // ...and a drift that stays inside the threshold does not arm one either, which is the whole
    // point of having a threshold: the read position is HELD rather than resampled.
    const AnchorTrace drifting = traceTap(
        [](int n) {
            return 0.5 +
                   0.9 * static_cast<double>(cnpg::dsp::kPositionAnchorThreshold01) * static_cast<double>(n) / 8192.0;
        },
        8192);
    for (const auto& state : drifting.state)
        REQUIRE(state.crossfade01 == 0.0f);
    REQUIRE(drifting.state.back().anchor01 == 0.5f);
}

TEST_CASE("CONTRACT: MovingPosition -- reading a moving seam twice in one sample changes nothing", "[contract]") {
    // The property that lets readJunctionInputs and writeJunctionOutputs agree on the crossfade's
    // weights without passing them between the two, and that makes a diagnostic probe harmless: the
    // retarget is idempotent, and the fade advances ONLY in tick(). Without it, the junction seam's
    // write would deposit through a different functional than its read used -- which is precisely
    // how a transparent junction would stop being free.
    WaveguideString<double> string;
    string.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    WaveguideStringParams params;
    params.f0Hz = 110.0f;
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset();

    cnpg::dsp::PluckExciter<double> exciter;
    exciter.prepare(kRate, kBlock);
    exciter.trigger(0.8f, 0.28f, 0.5f);

    bool sawFade = false;
    for (int n = 0; n < 24000; ++n) {
        const double excitation = exciter.renderSample();
        if (excitation != 0.0)
            string.injectAt(exciter.latchedPosition01(), excitation);
        const double p = 0.5 + 0.45 * std::sin(kTwoPi * 5.0 * static_cast<double>(n) / kRate);

        const double first = string.readTapAt(0, static_cast<float>(p));
        const auto afterFirst = string.tapCrossfade(0);
        const double second = string.readTapAt(0, static_cast<float>(p));
        const auto afterSecond = string.tapCrossfade(0);

        INFO("sample " << n);
        REQUIRE(first == second);
        REQUIRE(afterFirst.anchor01 == afterSecond.anchor01);
        REQUIRE(afterFirst.pending01 == afterSecond.pending01);
        REQUIRE(afterFirst.crossfade01 == afterSecond.crossfade01);
        sawFade |= (afterFirst.crossfade01 > 0.0f);

        double fromNut = 0.0;
        double fromBridge = 0.0;
        string.readJunctionInputs(static_cast<float>(p), fromNut, fromBridge);
        double againNut = 0.0;
        double againBridge = 0.0;
        string.readJunctionInputs(static_cast<float>(p), againNut, againBridge);
        REQUIRE(fromNut == againNut);
        REQUIRE(fromBridge == againBridge);

        string.tick();
    }
    REQUIRE(sawFade); // non-vacuous: the idempotence was exercised WHILE a crossfade was in flight
}

TEST_CASE("CONTRACT: MovingPosition -- the applied crossfade weights sum to exactly one", "[contract]") {
    // THE LOCKED LAW, asserted on the weights the read ACTUALLY APPLIES rather than on the ramp
    // that feeds them. Everything else in this file pins `crossfade01` -- that it is linear, that
    // it steps by exactly 1/128, that it lands on 1 -- and none of it would notice an
    // implementation that kept that ramp and applied sin(pi/2 g2) / cos(pi/2 g2) INSIDE the read.
    // That is precisely the equal-power law the ADR forbids, it would put up to +3 dB into
    // correlated mid-fade content, and the only current check that would catch it is the energy
    // gate -- indirectly, and in a different file.
    //
    // The construction is exact. Driving railAcceptFromBridge(c) every tick fills the dn rail with
    // c; the nut's rigid inversion then writes up_ = -toNut, so the up rail fills with -c. A
    // CONSTANT rail reads as that constant through any interpolation whose weights sum to one, at
    // every position -- so mid-crossfade,
    //
    //     fromNut = (g1 + g2) * (-c)     and     fromBridge = (g1 + g2) * (+c)
    //
    // and the two arriving waves are exactly -c and +c if and only if the applied weights sum to
    // exactly one. An equal-power law reads (sin + cos) * c, i.e. up to 41% high.
    //
    // Deliberately measured on the junction seam and not on readTapAt: the tap SUMS the two rails,
    // which under this fill are -c and +c, so the tap is identically 0 whatever the weights sum to.
    // The tap's own weights are pinned by the decomposition case below instead.
    constexpr double kFill = 0.375; // exactly representable, so "equals the constant" can be exact
    for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
        WaveguideString<double> string;
        string.prepare(kRate, kBlock, kind);
        WaveguideStringParams params;
        params.f0Hz = 110.0f;
        string.setParams(params);
        string.setAnalyticTuningCompensation(0.0f);
        string.reset();
        string.setLossBypassed(true); // so the loop chain cannot colour the constant on its way round

        // Fill both rails. The dn rail is driven directly; the up rail fills from the nut inversion,
        // so it needs a full rail length to converge -- 2000 ticks is far more than the longest rail
        // at 110 Hz / 48 kHz.
        for (int n = 0; n < 2000; ++n) {
            string.railAcceptFromBridge(kFill);
            string.tick();
        }

        int midFadeSamples = 0;
        double worstNutError = 0.0;
        double worstBridgeError = 0.0;
        double worstWeightSumError = 0.0;
        for (int n = 0; n < 24000; ++n) {
            string.railAcceptFromBridge(kFill); // keep the rails constant while the seam moves
            const auto p = static_cast<float>(0.5 + 0.4 * std::sin(kTwoPi * 5.0 * static_cast<double>(n) / kRate));

            double fromNut = 0.0;
            double fromBridge = 0.0;
            string.readJunctionInputs(p, fromNut, fromBridge);
            const auto crossfade = string.junctionCrossfade();
            if (crossfade.crossfade01 > 0.0f && crossfade.crossfade01 < 1.0f)
                ++midFadeSamples;

            INFO("sample " << n << ", g2 " << crossfade.crossfade01 << ", fromNut " << fromNut << ", fromBridge "
                           << fromBridge);
            worstNutError = std::max(worstNutError, std::fabs(fromNut - (-kFill)));
            worstBridgeError = std::max(worstBridgeError, std::fabs(fromBridge - kFill));
            // The implied weight sum, reported as the number the law is actually about.
            worstWeightSumError = std::max(worstWeightSumError, std::fabs(fromBridge / kFill - 1.0));

            // A transparent junction, so the rails stay constant for the next sample.
            string.writeJunctionOutputs(p, fromNut, fromBridge);
            string.tick();
        }

        std::cout << "[contract] crossfade weight sum ("
                  << (kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1") << "): " << midFadeSamples
                  << " of 24000 samples strictly mid-fade; worst |g1 + g2 - 1| " << worstWeightSumError
                  << ", worst |fromNut + " << kFill << "| " << worstNutError << ", worst |fromBridge - " << kFill
                  << "| " << worstBridgeError << " (an equal-power law would read up to 0.414)\n";

        // IN the state this case claims to test: it really was crossfading for most of the render.
        REQUIRE(midFadeSamples > 12000);
        // The weights sum to one to the float64 floor -- three orders of magnitude tighter than the
        // 1e-9 the rest of this project uses for "exactly", and eleven orders under the 0.414 an
        // equal-power law would show.
        REQUIRE(worstWeightSumError < 1.0e-12);
        REQUIRE(worstNutError < 1.0e-12);
        REQUIRE(worstBridgeError < 1.0e-12);
    }
}

TEST_CASE("CONTRACT: MovingPosition -- the tap applies exactly (1 - g2, g2) to its two anchors", "[contract]") {
    // The same law on the tap, where the constant-rail construction above is degenerate (the tap
    // sums the two rails, which that fill makes -c and +c). Instead the mix is DECOMPOSED: two
    // further tap slots are parked at the two anchor positions the crossfade is between, and the
    // fading slot's output must equal (1 - g2) * one + g2 * the other.
    //
    // Two passes, because the anchor positions are not known until the sweep has been run once and
    // parking a slot on a moving target would make it fade too. Pass 1 finds the first strictly
    // mid-fade sample and records the state there; pass 2 re-runs the identical string -- the tap is
    // a pure read, so the rails evolve identically -- with slots 1 and 2 held at those two constant
    // positions from sample 0, where they snap on first read and never move.
    constexpr int kSamples = 24000;
    auto positionAt = [](int n) { return 0.5 + 0.45 * std::sin(kTwoPi * 5.0 * static_cast<double>(n) / kRate); };

    auto makeString = [](WaveguideString<double>& string) {
        string.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        WaveguideStringParams params;
        params.f0Hz = 110.0f;
        params.stringMaterial.lossGainLow = 1.0f;
        params.stringMaterial.lossGainHigh = 1.0f;
        string.setParams(params);
        string.setAnalyticTuningCompensation(0.0f);
        string.reset();
    };

    // ---- pass 1: find the LOUDEST genuinely mid-fade sample, and record the state there --------
    // The loudest rather than the first, and that is not cosmetic. Both candidate laws agree
    // exactly when the two anchor reads are near zero, so the FIRST mid-fade sample can easily be
    // one where equal-power and amplitude-complementary differ by 3e-6 -- a difference that would
    // still satisfy a relative discrimination check while demonstrating nothing. Picking the sample
    // where the tap is largest is what gives the two laws room to disagree.
    int probeSample = -1;
    double probeMagnitude = 0.0;
    WaveguideString<double>::PositionCrossfade probeState{};
    {
        WaveguideString<double> string;
        makeString(string);
        cnpg::dsp::PluckExciter<double> exciter;
        exciter.prepare(kRate, kBlock);
        exciter.trigger(0.8f, 0.28f, 0.5f);
        for (int n = 0; n < kSamples; ++n) {
            const double excitation = exciter.renderSample();
            if (excitation != 0.0)
                string.injectAt(exciter.latchedPosition01(), excitation);
            const double tap = string.readTapAt(0, static_cast<float>(positionAt(n)));
            const auto state = string.tapCrossfade(0);
            // Far enough in that the pluck has filled the rails, and strictly between the anchors so
            // the decomposition is not a one-term identity.
            if (n > 4000 && state.crossfade01 > 0.2f && state.crossfade01 < 0.8f && state.anchor01 != state.pending01 &&
                std::fabs(tap) > probeMagnitude) {
                probeSample = n;
                probeMagnitude = std::fabs(tap);
                probeState = state;
            }
            string.tick();
        }
    }
    REQUIRE(probeSample > 0);
    REQUIRE(probeMagnitude > 0.001); // the probe really is on a live part of the waveform

    // ---- pass 2: decompose the fading read against two parked ones ----------------------------
    WaveguideString<double> string;
    makeString(string);
    cnpg::dsp::PluckExciter<double> exciter;
    exciter.prepare(kRate, kBlock);
    exciter.trigger(0.8f, 0.28f, 0.5f);

    double faded = 0.0;
    double atAnchor = 0.0;
    double atPending = 0.0;
    for (int n = 0; n <= probeSample; ++n) {
        const double excitation = exciter.renderSample();
        if (excitation != 0.0)
            string.injectAt(exciter.latchedPosition01(), excitation);
        faded = string.readTapAt(0, static_cast<float>(positionAt(n)));
        // Constant from sample 0, so these two never arm a fade of their own.
        atAnchor = string.readTapAt(1, probeState.anchor01);
        atPending = string.readTapAt(2, probeState.pending01);
        if (n < probeSample)
            string.tick();
    }

    // Pass 2 really did reproduce pass 1's state, and the two reference slots really are parked
    // exactly where the fading slot's anchors are -- both anchors came in as floats, so the stored
    // doubles are exactly those floats and there is no conversion slack here.
    const auto state = string.tapCrossfade(0);
    REQUIRE(state.anchor01 == probeState.anchor01);
    REQUIRE(state.pending01 == probeState.pending01);
    REQUIRE(state.crossfade01 == probeState.crossfade01);
    REQUIRE(string.tapCrossfade(1).crossfade01 == 0.0f);
    REQUIRE(string.tapCrossfade(1).anchor01 == probeState.anchor01);
    REQUIRE(string.tapCrossfade(2).crossfade01 == 0.0f);
    REQUIRE(string.tapCrossfade(2).anchor01 == probeState.pending01);
    // Non-vacuous: the two anchors really do read different values, so the weights are load-bearing.
    REQUIRE(atAnchor != atPending);

    const double g2 = static_cast<double>(state.crossfade01);
    const double linear = (1.0 - g2) * atAnchor + g2 * atPending;
    const double equalPower =
        std::cos(0.5 * kTwoPi * 0.5 * g2) * atAnchor + std::sin(0.5 * kTwoPi * 0.5 * g2) * atPending;
    const double linearError = std::fabs(faded - linear);
    const double equalPowerError = std::fabs(faded - equalPower);

    std::cout << "[contract] tap crossfade decomposition at sample " << probeSample << " (g2 " << g2 << ", tap "
              << faded << ", anchors " << atAnchor << " / " << atPending << "): |applied - amplitude-complementary| "
              << linearError << ", |applied - equal-power| " << equalPowerError << "\n";

    INFO("linear error " << linearError << ", equal-power error " << equalPowerError << ", tap " << faded);
    // The applied weights ARE (1 - g2, g2). Measured at exactly 0: the fade accumulates in steps of
    // 1/128, a power of two, so the reported float crossfade01 is the double the read applied, bit
    // for bit. The tolerance rather than == 0 is for the rates where the fade length is not a power
    // of two, and for FMA contraction differing between this expression and the one in the read.
    REQUIRE(linearError < 1.0e-9 * std::max(1.0, std::fabs(atAnchor) + std::fabs(atPending)));
    // ...and the equal-power law the ADR forbids is nowhere near. Stated ABSOLUTELY as well as
    // relatively: a purely relative bound is satisfied for free when linearError is 0, which says
    // nothing about whether the two laws were distinguishable at this sample at all.
    REQUIRE(equalPowerError > 0.01 * std::fabs(faded));
    REQUIRE(equalPowerError > 1.0e-4);
}

TEST_CASE("CONTRACT: MovingPosition -- a transparent junction is free even mid-crossfade", "[contract]") {
    // P2.2's bit-exact seam contract, carried into motion. writeJunctionOutputs re-reads through the
    // SAME crossfaded functional the read used, so a junction that returns its inputs unchanged
    // deposits exactly 0.0 into both rails at BOTH anchors. Asserted against a string carrying no
    // seam at all, sample for sample, while the junction position is swept across the string.
    //
    // This is also the measurement behind the P2.3 report's G4 finding: setLosslessTestMode(true)
    // makes DamperJunction transparent, and a transparent junction is free AT EVERY POSITION, so a
    // damper-position sweep under lossless mode cannot perturb one sample. That is a property of the
    // transparent junction, not a gap in the crossfade -- and it is why the moving-junction [energy]
    // gate (tests/dsp/MovingJunctionEnergyTests.cpp) runs a DISSIPATIVE junction on a lossless
    // string rather than inheriting the plan's lossless-everything scenario unexamined.
    for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
        WaveguideString<double> plain;
        WaveguideString<double> seamed;
        for (WaveguideString<double>* string : {&plain, &seamed}) {
            string->prepare(kRate, kBlock, kind);
            WaveguideStringParams params;
            params.f0Hz = 110.0f;
            string->setParams(params);
            string->setAnalyticTuningCompensation(0.0f);
            string->reset();
        }

        cnpg::dsp::PluckExciter<double> exciterA;
        cnpg::dsp::PluckExciter<double> exciterB;
        exciterA.prepare(kRate, kBlock);
        exciterB.prepare(kRate, kBlock);
        exciterA.trigger(0.8f, 0.28f, 0.5f);
        exciterB.trigger(0.8f, 0.28f, 0.5f);

        int midFadeSamples = 0;
        double worstDifference = 0.0;
        for (int n = 0; n < 48000; ++n) {
            const double ea = exciterA.renderSample();
            if (ea != 0.0)
                plain.injectAt(exciterA.latchedPosition01(), ea);
            const double eb = exciterB.renderSample();
            if (eb != 0.0)
                seamed.injectAt(exciterB.latchedPosition01(), eb);

            const auto p = static_cast<float>(0.5 + 0.4 * std::sin(kTwoPi * 5.0 * static_cast<double>(n) / kRate));
            double fromNut = 0.0;
            double fromBridge = 0.0;
            seamed.readJunctionInputs(p, fromNut, fromBridge);
            seamed.writeJunctionOutputs(p, fromNut, fromBridge); // a transparent junction, exactly
            if (seamed.junctionCrossfade().crossfade01 > 0.0f)
                ++midFadeSamples;

            const double a = plain.readTapAt(0.87f);
            const double b = seamed.readTapAt(0.87f);
            worstDifference = std::max(worstDifference, std::fabs(a - b));
            INFO("sample " << n);
            REQUIRE(a == b);
            plain.tick();
            seamed.tick();
        }
        std::cout << "[contract] transparent junction swept 0.1 .. 0.9 at 5 Hz ("
                  << (kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1") << "): " << midFadeSamples
                  << " of 48000 samples mid-crossfade, worst |waveform difference| " << worstDifference << "\n";
        REQUIRE(midFadeSamples > 24000); // non-vacuous: mostly IN the state this case claims to test
        REQUIRE(worstDifference == 0.0);
    }
}

// ---------------------------------------------------------------------------------------------
// DIRECT state assertion for the smoother P2.2 left exposed
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: MovingPosition -- StringNetwork glides damperPosition01 instead of stepping it", "[contract]") {
    // P2.2 shipped damperPosition01 as an automatable APVTS parameter applied RAW: it stepped at
    // every block boundary, and with the felt down that measured 5.50 dB of click-metric excess
    // against a 3 dB criterion. This is the direct state assertion for the smoother that fixes it --
    // the same shape as P2.2's own maxLoss-glide case, and for the same reason: a smoother that is
    // only ever observed at its endpoints would be satisfied by an implementation that steps.
    constexpr float kFrom = 0.15f;
    constexpr float kTo = 0.85f;

    StringNetworkParams params;
    params.damperPosition01 = kFrom;
    params.damper.maxLoss = 1.0f;

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset();
    REQUIRE(network.damperPosition01(0) == kFrom); // reset() snaps, as the lifecycle documents

    BlockEventQueue events;
    events.push(noteOn(0, 45, 0));
    for (int b = 0; b < 100; ++b)
        network.process(events, kBlock);
    BlockEventQueue release;
    release.push(noteOff(0, 45, 0));
    for (int b = 0; b < 40; ++b)
        network.process(release, kBlock);
    // In the state this case claims to test: the felt is down and the string is still ringing, so
    // the junction being moved is one a waveform is passing through right now.
    REQUIRE(network.damperEngagement(0) > 0.9f);
    REQUIRE(network.energyEstimate() > 0.0);
    REQUIRE(network.damperPosition01(0) == kFrom);

    params.damperPosition01 = kTo;
    network.setParams(params);
    // setParams only RETARGETS: the value in force must not have moved yet.
    REQUIRE(network.damperPosition01(0) == kFrom);

    // The value is observable per block, so the per-BLOCK bound is what is checked: kBlock steps of
    // the same 8 ms one-pole the pickup tap uses, over the whole distance.
    const double coeff = 1.0 - std::exp(-1.0 / (0.008 * kRate));
    const double blockLimit =
        (1.0 - std::pow(1.0 - coeff, static_cast<double>(kBlock))) * static_cast<double>(kTo - kFrom) + 1.0e-6;

    double worstBlockStep = 0.0;
    float previous = network.damperPosition01(0);
    bool sawIntermediate = false;
    BlockEventQueue idle;
    for (int b = 0; b < 200; ++b) {
        network.process(idle, kBlock);
        const float now = network.damperPosition01(0);
        worstBlockStep = std::max(worstBlockStep, std::fabs(static_cast<double>(now - previous)));
        // Monotone, and strictly between the endpoints while it travels -- a GLIDE, not a step that
        // happened to land on the target.
        REQUIRE(now >= previous);
        if (now > kFrom && now < kTo)
            sawIntermediate = true;
        previous = now;
    }

    std::cout << "[contract] damperPosition01 glide " << kFrom << " -> " << kTo << ": worst per-block step "
              << worstBlockStep << " (limit " << blockLimit << "), settled at " << network.damperPosition01(0) << "\n";

    INFO("worst block step " << worstBlockStep << " against limit " << blockLimit);
    REQUIRE(sawIntermediate);
    REQUIRE(worstBlockStep <= blockLimit);
    // "Arrived" is a reachable state, exactly, like every other smoother in this codebase.
    REQUIRE(network.damperPosition01(0) == kTo);
}

TEST_CASE("CONTRACT: MovingPosition -- the moving-position path allocates nothing", "[contract]") {
    StringNetworkParams params;
    params.damper.maxLoss = 0.5f;
    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(kChordStrings);
    network.setParams(params);
    network.reset();

    BlockEventQueue events;
    for (int s = 0; s < kChordStrings; ++s)
        events.push(noteOn(0, kChord[s], s));

    cnpg::test::resetAllocationCount();
    for (int b = 0; b < 400; ++b) {
        const double t = static_cast<double>(b) * static_cast<double>(kBlock) / kRate;
        params.pickupPosition01 = static_cast<float>(0.5 + 0.45 * std::sin(kTwoPi * 5.0 * t));
        params.damperPosition01 = static_cast<float>(0.5 + 0.4 * std::sin(kTwoPi * 7.0 * t));
        network.setParams(params);
        if (b == 100)
            for (int s = 0; s < kChordStrings; ++s)
                events.push(noteOff(0, kChord[s], s));
        network.process(events, kBlock);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

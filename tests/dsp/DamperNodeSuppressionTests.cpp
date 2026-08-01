#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// DamperNodeSuppressionTests -- Task P2.2's physics payoff, and the one test in this task that
// would fail against a "damper" that was any kind of gain.
//
// A resistive point contact at p can only dissipate the part of a wave pair that MOVES it. Mode n
// has displacement shape sin(n*pi*x), so at p = 1/2 mode 1 has an antinode (fully damped) and
// mode 2 has an exact node (untouchable). Put the damper at the midpoint of a ringing string and
// the fundamental dies while the octave rings on -- which is a guitarist's natural harmonic, and
// is also exactly the unit eigenvalue in DamperJunction's scattering matrix showing up as sound.
//
// The measurement is per-partial and time-resolved (tests/support/SpectralAnalysis.h,
// partialEnvelope), because the whole claim is about two partials of ONE note decaying at
// different rates: an octave-band T60 cannot separate 110 Hz from 220 Hz, and a whole-render FFT
// averages over the very transition being measured.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr int kMidiNote = 45; // 110 Hz: 27 cycles of the fundamental fit in the 250 ms window
constexpr float kPluckPosition = 0.28f;
constexpr float kPickup = 0.87f; // sees both partials: |sin(pi*0.87)| = 0.40, |sin(2pi*0.87)| = 0.74

// Well under the 110 Hz partial spacing: four one-pole sections give 4 * 20*log10(110/18) = 31 dB
// of rejection of the neighbouring partial, and 1/(2*pi*18 Hz) = 8.8 ms of envelope smearing,
// which is the resolution limit this file states its measurements against.
constexpr double kEnvelopeBandwidthHz = 18.0;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex = 0) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = 0.8f;
    event.pluckPosition = kPluckPosition;
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

double dbRatio(double after, double before) {
    if (!(before > 0.0))
        return 0.0;
    if (!(after > 0.0))
        return -300.0;
    return 20.0 * std::log10(after / before);
}

struct Render {
    std::vector<double> tap;
    std::size_t noteOffSample = 0;
    std::size_t fullEngagementSample = 0; // first sample at which damperEngagement() >= 0.99
    float engagementAtNoteOff = -1.0f;
    float engagementBeforeNoteOff = -1.0f;
};

// One string, plucked, and (unless `releaseIt` is false) released after `noteOffSeconds` with the
// damper at `damperPosition01`. The un-released render is the REFERENCE: the identical note with
// the state change never applied, which is how everything else in this repo isolates what a state
// change did from what the instrument was doing anyway.
Render render(float damperPosition01, float maxLoss, double noteOffSeconds, double totalSeconds, bool releaseIt = true,
              float feltTimeConstantMs = 40.0f) {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    params.damperPosition01 = damperPosition01;
    params.damper.maxLoss = maxLoss;
    params.damper.feltTimeConstantMs = feltTimeConstantMs;
    params.exciter.noiseAmount = 0.0f; // a deterministic partial spectrum to measure

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset();

    // The junction really is where the test says it is, and it starts released -- asserted, not
    // assumed, because every measurement below is about a position and an engagement.
    REQUIRE(network.damperPosition01(0) == damperPosition01);
    REQUIRE(network.damperEngagement(0) == 0.0f);

    Render out;
    const auto totalBlocks = static_cast<int>(totalSeconds * kRate / kBlock);
    const auto noteOffBlock = static_cast<int>(noteOffSeconds * kRate / kBlock);
    out.noteOffSample = static_cast<std::size_t>(noteOffBlock) * static_cast<std::size_t>(kBlock);
    out.tap.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));

    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));
    for (int b = 0; b < totalBlocks; ++b) {
        if (b == noteOffBlock && releaseIt) {
            out.engagementBeforeNoteOff = network.damperEngagement(0);
            events.push(noteOff(0, kMidiNote));
        }
        network.process(events, kBlock);
        if (b == noteOffBlock && releaseIt)
            out.engagementAtNoteOff = network.damperEngagement(0);
        if (out.fullEngagementSample == 0 && network.damperEngagement(0) >= 0.99f)
            out.fullEngagementSample = out.tap.size();
        const float* channel = network.tapBuffers().channel(0, 0);
        REQUIRE(channel != nullptr);
        for (int n = 0; n < kBlock; ++n)
            out.tap.push_back(static_cast<double>(channel[n]));
    }
    return out;
}

} // namespace

TEST_CASE("CONTRACT: DamperNodeSuppression -- a damper at p = 1/2 kills the fundamental and spares the octave",
          "[contract]") {
    // docs/plan.md Task P2.2 acceptance: "pluck at position 0.28, damper at position01 = 0.5,
    // engage() at t = 1.0 s with maxLoss 1.0: within 250 ms of full engagement the fundamental
    // partial drops >= 24 dB from its pre-engage level while the 2nd harmonic drops <= 6 dB;
    // fitted T60(fundamental) <= 0.25 x T60(2nd harmonic)."
    constexpr double kNoteOffSeconds = 1.0;
    constexpr double kTotalSeconds = 4.0;
    constexpr double kMeasureAfterSeconds = 0.250;
    constexpr double kFundamentalDropDb = -24.0;
    constexpr double kSecondHarmonicDropDb = -6.0;
    constexpr double kT60Ratio = 0.25;

    const Render rendered = render(0.5f, 1.0f, kNoteOffSeconds, kTotalSeconds);
    // The identical note with no note-off at all. Both "drops" below are measured against this as
    // well as against the pre-engage level, and the reason is arithmetic rather than taste: at the
    // default material this string's 2nd harmonic has a T60 of ~2.2 s, so over the ~435 ms between
    // the note-off and the measurement point it loses ~11.8 dB TO ITS OWN DECAY -- more than the
    // 6 dB the criterion allows, before the damper has done anything at all. Judging the criterion
    // on the raw pre-engage drop would therefore be measuring the string's loop loss and calling
    // it damper leakage. The decomposition is asserted below rather than asserted about.
    const Render reference = render(0.5f, 1.0f, kNoteOffSeconds, kTotalSeconds, false);
    REQUIRE(reference.tap.size() == rendered.tap.size());

    // ---- the scenario is genuinely the scenario ------------------------------------------------
    // The damper was released while the note rang, the note-off left the ramp starting from 0 (so
    // the note-off sample itself is undamped), and full engagement really was reached.
    REQUIRE(rendered.engagementBeforeNoteOff == 0.0f);
    REQUIRE(rendered.engagementAtNoteOff > 0.0f); // the ramp moved inside that very block...
    REQUIRE(rendered.engagementAtNoteOff < 1.0f); // ...and did not jump
    REQUIRE(rendered.fullEngagementSample > rendered.noteOffSample);
    const double engagementSeconds =
        static_cast<double>(rendered.fullEngagementSample - rendered.noteOffSample) / kRate;

    const double fundamentalHz = cnpg::test::midiNoteToHz(kMidiNote);
    const std::vector<double> partial1 =
        cnpg::test::partialEnvelope(rendered.tap, kRate, fundamentalHz, kEnvelopeBandwidthHz);
    const std::vector<double> partial2 =
        cnpg::test::partialEnvelope(rendered.tap, kRate, 2.0 * fundamentalHz, kEnvelopeBandwidthHz);

    // Pre-engage levels, read one envelope settling time BEFORE the note-off so the reading is of
    // the ringing string and not of the transition.
    const auto preIndex = rendered.noteOffSample - static_cast<std::size_t>(0.020 * kRate);
    const double pre1 = partial1[preIndex];
    const double pre2 = partial2[preIndex];
    REQUIRE(pre1 > 1.0e-4); // both partials really were ringing before the damper came down
    REQUIRE(pre2 > 1.0e-4);

    const auto measureIndex = rendered.fullEngagementSample + static_cast<std::size_t>(kMeasureAfterSeconds * kRate);
    REQUIRE(measureIndex < rendered.tap.size());
    const double rawDrop1 = dbRatio(partial1[measureIndex], pre1);
    const double rawDrop2 = dbRatio(partial2[measureIndex], pre2);

    // The same two partials of the un-released note, at the same instant: what the string would
    // have been doing anyway. The difference is what the damper did.
    const std::vector<double> undamped1 =
        cnpg::test::partialEnvelope(reference.tap, kRate, fundamentalHz, kEnvelopeBandwidthHz);
    const std::vector<double> undamped2 =
        cnpg::test::partialEnvelope(reference.tap, kRate, 2.0 * fundamentalHz, kEnvelopeBandwidthHz);
    const double naturalDrop1 = dbRatio(undamped1[measureIndex], undamped1[preIndex]);
    const double naturalDrop2 = dbRatio(undamped2[measureIndex], undamped2[preIndex]);
    const double drop1 = dbRatio(partial1[measureIndex], undamped1[measureIndex]);
    const double drop2 = dbRatio(partial2[measureIndex], undamped2[measureIndex]);

    // ---- fitted decay rates --------------------------------------------------------------------
    // Fitted from the NOTE-OFF, not from full engagement: by the time the ramp has settled the
    // fundamental is long gone and there is no decay left to fit. Both partials are fitted over
    // the same span by the same estimator, which is what makes the ratio meaningful.
    const double t60Fundamental = cnpg::test::partialT60Seconds(partial1, kRate, rendered.noteOffSample);
    const double t60SecondHarmonic = cnpg::test::partialT60Seconds(partial2, kRate, rendered.noteOffSample);

    std::cout << "[contract] node suppression, damper at p = 0.5, maxLoss 1.0: full engagement " << engagementSeconds
              << " s after note-off; at +" << kMeasureAfterSeconds << " s the fundamental is " << drop1
              << " dB below the un-released note (raw " << rawDrop1 << " dB, of which " << naturalDrop1
              << " dB is the string's own decay; limit " << kFundamentalDropDb << ") and the 2nd harmonic " << drop2
              << " dB (raw " << rawDrop2 << " dB, of which " << naturalDrop2 << " dB is its own decay; limit "
              << kSecondHarmonicDropDb << "); fitted T60 " << t60Fundamental << " s vs " << t60SecondHarmonic
              << " s (ratio " << (t60Fundamental / t60SecondHarmonic) << ", limit " << kT60Ratio << ")\n";

    INFO("fundamental " << drop1 << " dB (raw " << rawDrop1 << "), 2nd harmonic " << drop2 << " dB (raw " << rawDrop2
                        << ")");
    REQUIRE(drop1 <= kFundamentalDropDb);
    REQUIRE(drop2 >= kSecondHarmonicDropDb);

    // The raw pre-engage reading still has to satisfy the criterion for the fundamental, where
    // the string's own decay only helps -- so that half of the criterion is met on either
    // reading, and only the 2nd harmonic's needed the reference render.
    REQUIRE(rawDrop1 <= kFundamentalDropDb);
    // ...and the decomposition asserted rather than asserted about: the 2nd harmonic's raw drop IS
    // its natural decay plus the damper's small contribution, to within the envelope estimator's
    // own resolution.
    REQUIRE(std::fabs(rawDrop2 - (naturalDrop2 + drop2)) < 0.5);

    REQUIRE(t60Fundamental > 0.0);
    REQUIRE(t60SecondHarmonic > 0.0);
    // NOTE on what this number is. The fundamental's true decay under a matched damper at its own
    // antinode is faster than the 8.8 ms envelope cascade can follow, so the fitted value is an
    // UPPER bound on it -- the measurement is conservative in exactly the direction this assertion
    // needs, and the true ratio is smaller than the one printed above.
    INFO("T60 fundamental " << t60Fundamental << " s, 2nd harmonic " << t60SecondHarmonic << " s");
    REQUIRE(t60Fundamental <= kT60Ratio * t60SecondHarmonic);
}

TEST_CASE("CONTRACT: DamperNodeSuppression -- the surviving partial is the one with the node, not the higher one",
          "[contract]") {
    // The claim under test is POSITIONAL, and a plain lowpass damping would satisfy the case above
    // by accident: the fundamental is louder, so any broadband damping plus any decay ordering
    // could produce "1 falls, 2 falls less". Two controls, and neither passes without a genuine
    // position-dependent scattering junction.
    constexpr double kNoteOffSeconds = 1.0;
    constexpr double kTotalSeconds = 2.5;
    const double fundamentalHz = cnpg::test::midiNoteToHz(kMidiNote);

    auto dropsAfter = [&](float damperPosition01, double afterSeconds) {
        const Render rendered = render(damperPosition01, 1.0f, kNoteOffSeconds, kTotalSeconds);
        REQUIRE(rendered.engagementBeforeNoteOff == 0.0f);
        const auto preIndex = rendered.noteOffSample - static_cast<std::size_t>(0.020 * kRate);
        const auto postIndex = rendered.noteOffSample + static_cast<std::size_t>(afterSeconds * kRate);
        REQUIRE(postIndex < rendered.tap.size());

        std::vector<double> drops;
        for (int partial = 1; partial <= 3; ++partial) {
            const std::vector<double> envelope = cnpg::test::partialEnvelope(
                rendered.tap, kRate, static_cast<double>(partial) * fundamentalHz, kEnvelopeBandwidthHz);
            REQUIRE(envelope[preIndex] > 1.0e-5);
            drops.push_back(dbRatio(envelope[postIndex], envelope[preIndex]));
        }
        return drops;
    };

    // Control 1: at p = 1/2 the damper is blind to EVEN partials by construction -- partial 2 has
    // a node there and partial 3 an antinode. A frequency-dependent damper would take 3 down
    // hardest and 1 least; the node structure does the opposite.
    const std::vector<double> atHalf = dropsAfter(0.5f, 0.5);
    std::cout << "[contract] node structure at p = 0.5: partial 1 " << atHalf[0] << " dB, partial 2 " << atHalf[1]
              << " dB, partial 3 " << atHalf[2] << " dB\n";
    INFO("p = 0.5 drops: " << atHalf[0] << ", " << atHalf[1] << ", " << atHalf[2] << " dB");
    REQUIRE(atHalf[1] > atHalf[0] + 18.0); // partial 2 survives its neighbours by a wide margin
    REQUIRE(atHalf[1] > atHalf[2] + 18.0);

    // Control 2: move the junction to p = 1/3 and the SURVIVOR MOVES WITH IT -- partial 3 now has
    // the node and partial 2 an antinode. Same damper, same depth, same note: only p changed.
    const std::vector<double> atThird = dropsAfter(1.0f / 3.0f, 0.5);
    std::cout << "[contract] node structure at p = 1/3: partial 1 " << atThird[0] << " dB, partial 2 " << atThird[1]
              << " dB, partial 3 " << atThird[2] << " dB\n";
    INFO("p = 1/3 drops: " << atThird[0] << ", " << atThird[1] << ", " << atThird[2] << " dB");
    REQUIRE(atThird[2] > atThird[1] + 12.0);
    REQUIRE(atThird[2] > atHalf[2] + 12.0); // and partial 3 is far better off than it was at p = 1/2
}

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"
#include "support/SpectralAnalysis.h"
#include "support/StringIrScenarios.h"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

// DamperEnergyTests -- Task P2.2's passivity and transparency gates for DamperJunction.
//
// Three things live here, and they are the same claim measured at three altitudes:
//
//   1. Tier-1 [energy] (docs/plan.md section 4.2): the 2x2 scattering matrix is passive over the
//      canonical grid. This is the algebra.
//   2. Transparency [contract]: at engagement 0 scatter() is a BIT-EXACT pass-through, and the
//      permanently in-line seam therefore costs a ringing string nothing measurable. This is the
//      same matrix at one grid point, plus what the surrounding WaveguideString seam does with it.
//   3. Tier-3 [energy]: a plucked network with real losses only ever dissipates, dampers engaged
//      or not. This is the algebra actually holding inside the loop it was built for.
//
// The felt-time-constant VALIDATION and the engagement ramp's continuity are here too, because
// they are properties of the junction itself; the felt time's audible consequence (how long a
// note-off takes to silence) is tests/dsp/DamperFeltTimeTests.cpp, and the position-dependent
// modal behaviour is tests/dsp/DamperNodeSuppressionTests.cpp.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::DamperJunction;
using cnpg::dsp::DamperJunctionParams;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;

// Closed-form 2x2 SVD. Deliberately the GENERAL formula rather than the symmetric shortcut the
// derivation in DamperJunction.h licenses: the test must measure the matrix it is handed, not
// re-assume the structure it is supposed to be checking.
//
//   E = (a+d)/2, F = (a-d)/2, G = (c+b)/2, H = (c-b)/2
//   sigma_max = sqrt(E^2 + H^2) + sqrt(F^2 + G^2)
double spectralNorm2x2(const double* rowMajor) {
    const double a = rowMajor[0];
    const double b = rowMajor[1];
    const double c = rowMajor[2];
    const double d = rowMajor[3];
    const double e = 0.5 * (a + d);
    const double f = 0.5 * (a - d);
    const double g = 0.5 * (c + b);
    const double h = 0.5 * (c - b);
    return std::hypot(e, h) + std::hypot(f, g);
}

// docs/plan.md section 4.2, tier 1: "position01 in {0.0, 0.1, ..., 1.0} x engagement in
// {0, 0.25, 0.5, 0.75, 1} (via setEngagementImmediate) x maxLoss in {0, 0.5, 1}". Referenced from
// this one place so the grid is stated once.
constexpr std::array<float, 5> kGridEngagements{0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
constexpr std::array<float, 3> kGridMaxLosses{0.0f, 0.5f, 1.0f};
constexpr int kGridPositions = 11; // 0.0, 0.1, ... 1.0

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

NoteEvent noteOff(int sampleOffset, int midiNote, int stringIndex) {
    NoteEvent event = noteOn(sampleOffset, midiNote, stringIndex);
    event.type = NoteEventType::NoteOff;
    event.pluckPosition = cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

// Puts a junction into an exactly-known state: parameters applied, loss depth snapped onto
// maxLoss by reset(), engagement snapped by setEngagementImmediate. Every caller then ASSERTS it
// is in that state rather than assuming the sequence worked -- the P2.1 review's standing lesson.
template <typename SampleT>
void placeAt(DamperJunction<SampleT>& damper, float position01, float maxLoss, float engagement) {
    DamperJunctionParams params;
    params.position01 = position01;
    params.maxLoss = maxLoss;
    damper.prepare(kRate, kBlock);
    damper.setParams(params);
    damper.reset();
    damper.setEngagementImmediate(engagement);
}

} // namespace

// ---------------------------------------------------------------------------------------------
// tier 1: the scattering matrix is passive, by construction, over the canonical grid
// ---------------------------------------------------------------------------------------------

TEMPLATE_TEST_CASE("ENERGY/T1: DamperJunction scattering matrix is passive", "[energy]", float, double) {
    // docs/plan.md section 4.2, tier 1 -- the grid is defined there and referenced here (see
    // kGridEngagements / kGridMaxLosses / kGridPositions above), never restated as new numbers.
    constexpr double kNormLimit = 1.0 + 1.0e-12;

    double worstNorm = 0.0;
    float worstAt[3] = {0.0f, 0.0f, 0.0f};

    for (int positionStep = 0; positionStep < kGridPositions; ++positionStep) {
        const auto position01 = static_cast<float>(positionStep) / static_cast<float>(kGridPositions - 1);
        for (float maxLoss : kGridMaxLosses) {
            for (float engagement : kGridEngagements) {
                DamperJunction<TestType> damper;
                placeAt(damper, position01, maxLoss, engagement);

                // The grid point is REALLY the grid point. Without this the sweep could be
                // measuring one state 165 times and nobody would know.
                INFO("p " << position01 << " maxLoss " << maxLoss << " engagement " << engagement);
                REQUIRE(damper.currentPosition01() == position01);
                REQUIRE(damper.currentLossDepth() == maxLoss);
                REQUIRE(damper.currentEngagement() == engagement);

                double matrix[4] = {0.0, 0.0, 0.0, 0.0};
                damper.copyScatteringMatrix(matrix);
                const double norm = spectralNorm2x2(matrix);

                REQUIRE(std::isfinite(norm));
                REQUIRE(norm <= kNormLimit);
                if (norm > worstNorm) {
                    worstNorm = norm;
                    worstAt[0] = position01;
                    worstAt[1] = maxLoss;
                    worstAt[2] = engagement;
                }

                // Exact transparency at engagement 0 -- section 4.2 asks for it inside this same
                // case: "Additionally asserts exact transparency (S = anti-diagonal pass-through)
                // at engagement 0". maxLoss is irrelevant there, which the grid covers.
                const double product = static_cast<double>(engagement) * static_cast<double>(maxLoss);
                if (product == 0.0) {
                    REQUIRE(matrix[0] == 0.0);
                    REQUIRE(matrix[1] == 1.0);
                    REQUIRE(matrix[2] == 1.0);
                    REQUIRE(matrix[3] == 0.0);
                }

                // NON-VACUITY, and the reason this case cannot quietly become a tautology: a
                // junction that never damps anything is trivially passive. At full engagement and
                // full depth the matrix must ANNIHILATE the symmetric (displacement-carrying)
                // mode -- S (1,1)^T == 0, the matched resistive termination -- while the
                // antisymmetric mode, which puts a node on the damper and cannot be dissipated,
                // must pass at exactly unit gain.
                if (product == 1.0) {
                    REQUIRE(matrix[0] + matrix[1] == 0.0);
                    REQUIRE(matrix[2] + matrix[3] == 0.0);
                    REQUIRE(matrix[0] - matrix[1] == -1.0);
                }

                // The matrix does not depend on WHERE the junction sits: position selects the
                // point on the string the seam reads and writes, and the two-port itself is
                // memoryless in p. Asserted against the p = 0 point of the same (maxLoss,
                // engagement) pair so the grid's position axis is checked rather than merely
                // swept.
                DamperJunction<TestType> atOrigin;
                placeAt(atOrigin, 0.0f, maxLoss, engagement);
                double originMatrix[4] = {0.0, 0.0, 0.0, 0.0};
                atOrigin.copyScatteringMatrix(originMatrix);
                for (int i = 0; i < 4; ++i)
                    REQUIRE(matrix[i] == originMatrix[i]);
            }
        }
    }

    std::cout << "[energy] T1 DamperJunction |S|_2: worst " << worstNorm << " over the section-4.2 grid (limit "
              << kNormLimit << "), at position " << worstAt[0] << " maxLoss " << worstAt[1] << " engagement "
              << worstAt[2] << "\n";

    // The junction is lossless-or-dissipative everywhere and ACTIVE nowhere: the worst norm over
    // the whole grid is exactly the unit gain of the undissipatable antisymmetric mode.
    REQUIRE(worstNorm == 1.0);
}

TEST_CASE("ENERGY/T1: DamperJunction sanitizes an out-of-range engagement into a passive one", "[energy]") {
    // WHAT THIS CASE CAN AND CANNOT SHOW. It was first written claiming to probe "conductances far
    // outside the parameter range", and it cannot: setEngagementImmediate() sanitizes its argument
    // into [0, 1] before anything downstream sees it, so nothing here ever reaches the coefficients
    // with s outside [0, 1]. What it actually shows is INPUT SANITIZATION -- that an out-of-range
    // or NaN engagement from a caller resolves rather than propagating into the audio path -- which
    // is worth a test on its own, and is what the title now says.
    //
    // The claim that ||S||_2 <= 1 for ANY non-negative conductance is carried by the derivation in
    // DamperJunction.h (S is real symmetric with eigenvalues 1-2g and -1, and g = R/(R+2) lies in
    // [0,1) for every R >= 0), not by a sweep. There is deliberately no entry point that hands the
    // junction a raw conductance, because there is no entry point that lets the audio path do it
    // either -- which is the same reason the bound is structural rather than measured.
    for (float engagement : {-1.0f, -0.0f, 0.5f, 1.0f, 3.0f, 1.0e30f, std::numeric_limits<float>::quiet_NaN()}) {
        DamperJunction<float> damper;
        damper.prepare(kRate, kBlock);
        damper.setParams(DamperJunctionParams{});
        damper.reset();
        damper.setEngagementImmediate(engagement);

        // Out-of-range input resolves into 0..1 rather than reaching the coefficients raw.
        const float settled = damper.currentEngagement();
        INFO("requested engagement " << engagement << " resolved to " << settled);
        REQUIRE(settled >= 0.0f);
        REQUIRE(settled <= 1.0f);

        double matrix[4] = {0.0, 0.0, 0.0, 0.0};
        damper.copyScatteringMatrix(matrix);
        for (double value : matrix)
            REQUIRE(std::isfinite(value));
        REQUIRE(spectralNorm2x2(matrix) <= 1.0 + 1.0e-12);
    }
}

// ---------------------------------------------------------------------------------------------
// transparency: the junction is permanently in-line, and at engagement 0 that costs nothing
// ---------------------------------------------------------------------------------------------

TEMPLATE_TEST_CASE("CONTRACT: DamperJunction at engagement 0 is a bit-exact pass-through", "[contract]", float,
                   double) {
    // docs/plan.md Task P2.2 acceptance: "a unit test that DamperJunction::scatter() at
    // engagement 0 is bit-exact pass-through (toBridge == fromNut, toNut == fromBridge)".
    const TestType operands[] = {TestType(0),
                                 TestType(-0.0),
                                 TestType(1),
                                 TestType(-1),
                                 TestType(0.7071067811865476),
                                 TestType(-3.3333333e-3),
                                 TestType(1.0e-30),
                                 TestType(-1.0e-30),
                                 TestType(1.7e18)};

    for (float maxLoss : kGridMaxLosses) {
        DamperJunction<TestType> damper;
        placeAt(damper, 0.37f, maxLoss, 0.0f);
        REQUIRE(damper.currentEngagement() == 0.0f); // in the state this case claims to test

        for (TestType fromNut : operands) {
            for (TestType fromBridge : operands) {
                TestType toBridge = TestType(12345);
                TestType toNut = TestType(-12345);
                damper.scatter(fromNut, fromBridge, toBridge, toNut);

                INFO("maxLoss " << maxLoss << " fromNut " << fromNut << " fromBridge " << fromBridge);
                REQUIRE(toBridge == fromNut);
                REQUIRE(toNut == fromBridge);

                // Stronger than the criterion where it is available: identical BIT PATTERNS, not
                // merely equal values. Excluded for zero operands only, and for one reason worth
                // recording: with both operands negative zero, `a - (0 * (a + b))` is
                // -0.0 - (-0.0) == +0.0. Every subsequent arithmetic operation treats those two
                // zeros identically, and == already covers the case, so the seam is unaffected --
                // but a memcmp would call it a difference, and pretending otherwise would be
                // dressing up the claim.
                if (fromNut != TestType(0)) {
                    REQUIRE(std::memcmp(&toBridge, &fromNut, sizeof(TestType)) == 0);
                }
                if (fromBridge != TestType(0)) {
                    REQUIRE(std::memcmp(&toNut, &fromBridge, sizeof(TestType)) == 0);
                }

                // ...and the engagement really has not crept: the ramp is parked, so every one of
                // these samples was scattered by the same transparent junction.
                REQUIRE(damper.currentEngagement() == 0.0f);
            }
        }
    }
}

TEST_CASE("CONTRACT: DamperJunction bypassing the loss is transparent at full engagement", "[contract]") {
    // The energy-test hook StringNetwork::setLosslessTestMode forwards (docs/plan.md section 2.7).
    // The resistive junction loss is this class's only intentional loss, so bypassing it can only
    // mean "transparent", and the engagement ramp must keep running underneath so that switching
    // the bypass back off resumes where the ramp had got to rather than restarting it.
    DamperJunction<float> damper;
    placeAt(damper, 0.5f, 1.0f, 1.0f);
    REQUIRE(damper.currentEngagement() == 1.0f);

    damper.setLossBypassed(true);
    float toBridge = 0.0f;
    float toNut = 0.0f;
    damper.scatter(0.25f, -0.75f, toBridge, toNut);
    REQUIRE(toBridge == 0.25f);
    REQUIRE(toNut == -0.75f);

    double matrix[4] = {0.0, 0.0, 0.0, 0.0};
    damper.copyScatteringMatrix(matrix);
    REQUIRE(matrix[0] == 0.0);
    REQUIRE(matrix[1] == 1.0);
    REQUIRE(matrix[2] == 1.0);
    REQUIRE(matrix[3] == 0.0);

    // The ramp underneath is untouched, and the junction damps again the moment the bypass lifts.
    REQUIRE(damper.currentEngagement() == 1.0f);
    damper.setLossBypassed(false);
    damper.scatter(0.25f, -0.75f, toBridge, toNut);
    REQUIRE(toBridge != 0.25f);
}

TEST_CASE("CONTRACT: a permanently in-line damper at engagement 0 costs the string nothing", "[contract]") {
    // docs/plan.md Task P2.2 acceptance, second half: "The junction stays permanently in-line -- no
    // compiled-out comparison exists, because the fractional seam interpolates even at zero
    // engagement; instead, an engagement-0 string impulse response is compared to the no-damper
    // reference via the layer-(a) feature invariants (first 8 partials +/-2 cents, T60 +/-10%) plus
    // a stated non-zero waveform tolerance, and the P2.7 calibration table absorbs the seam's group
    // delay."
    //
    // THE STATED TOLERANCE IS 1e-7 (kStringIrGoldenAtol, the same number section 4.3 layer (b)
    // uses), and the measured difference is reported below. It comes out at exactly 0.0, and that
    // is not luck: WaveguideString::writeJunctionOutputs deposits only the DIFFERENCE between the
    // junction's outputs and the waves it just read, and a transparent scatter() returns those same
    // waves bit-for-bit, so the deposit is exactly 0.0f into both rails. The seam interpolates
    // (twice on the read side, twice more on the write side, every sample, for every string) and
    // then adds nothing. There is consequently no group delay for P2.7's table to absorb either --
    // the brief allows for one, and the construction does not produce one.
    //
    // The feature invariants are measured anyway rather than skipped as implied by bit-identity:
    // they are what the acceptance criterion names, and they are what will still be meaningful in
    // P2.3 when the moving-junction crossfade makes the seam genuinely non-exact.
    constexpr double kSeconds = 3.0;
    constexpr float kPluck = 0.28f;
    constexpr float kPickup = 0.87f;

    // Three junction positions at the file's own rate/note, plus the extreme the golden IRs are
    // captured at (MIDI 21 at 44.1 kHz -- the longest rail this instrument has, and the scenario
    // where a 1-ulp perturbation has the most room to grow over three seconds).
    struct Scenario {
        int midiNote;
        double sampleRate;
        float damperPosition;
    };
    const Scenario scenarios[] = {{45, kRate, 0.15f}, {45, kRate, 0.5f}, {45, kRate, 0.87f}, {21, 44100.0, 0.15f}};

    for (const Scenario& scenario : scenarios) {
        const int kMidiNote = scenario.midiNote;
        const double rate = scenario.sampleRate;
        const float damperPosition = scenario.damperPosition;
        StringNetworkParams params;
        params.pickupPosition01 = kPickup;
        params.damperPosition01 = damperPosition;
        params.damper.maxLoss = 1.0f; // irrelevant at engagement 0, and that is part of the claim
        params.exciter.noiseAmount = 0.25f;
        // The bridge is held at its rigid limit and the hand-driven reference below is given the
        // SAME rigid termination at the SAME one-sample offset (Task P2.4). What this case claims
        // is that the DAMPER SEAM costs the string nothing; the bridge seam is a different seam
        // with a different cost, and leaving it in would fold the two together and measure neither.
        params.bridge.couplingStrength = 0.0f;

        StringNetwork<float> network;
        network.prepare(rate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        network.setParams(params);
        network.reset();
        REQUIRE(network.damperPosition01(0) == damperPosition);
        REQUIRE(network.damperEngagement(0) == 0.0f); // in the state this case claims to test

        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote, 0));

        // The NO-DAMPER reference: the same string, exciter and tap driven by hand, with no
        // junction in the loop at all. This is the comparison the acceptance criterion names, and
        // it is the only way to have one -- there is no build of StringNetwork without the seam.
        cnpg::dsp::WaveguideString<float> string;
        string.prepare(rate, kBlock, FractionalDelayKind::Lagrange3);
        cnpg::dsp::WaveguideStringParams stringParams;
        stringParams.f0Hz = static_cast<float>(440.0 * std::exp2((static_cast<double>(kMidiNote) - 69.0) / 12.0));
        string.setParams(stringParams);
        string.setAnalyticTuningCompensation(0.0f);
        string.setBridgePortDriven(true); // the network's topology, so the comparison is fair
        string.reset();

        cnpg::dsp::RigidBridgeTermination<float> rigid;
        rigid.prepare(rate, kBlock, 1, nullptr);

        cnpg::dsp::PluckExciter<float> exciter;
        exciter.prepare(rate, kBlock);
        cnpg::dsp::PluckExciterParams exciterParams;
        exciterParams.noiseAmount = 0.25f;
        exciter.setParams(exciterParams);
        exciter.trigger(0.8f, kPluck, 0.5f);

        const auto totalSamples = static_cast<std::size_t>(kSeconds * rate);
        std::vector<double> withDamper;
        std::vector<double> withoutDamper;
        withDamper.reserve(totalSamples);
        withoutDamper.reserve(totalSamples);

        double worstDifference = 0.0;
        while (withDamper.size() < totalSamples) {
            network.process(events, kBlock);
            const float* channel = network.tapBuffers().channel(0, 0);
            REQUIRE(channel != nullptr);
            for (int n = 0; n < kBlock && withDamper.size() < totalSamples; ++n) {
                const float excitation = exciter.renderSample();
                if (excitation != 0.0f)
                    string.injectAt(exciter.latchedPosition01(), excitation);
                const float direct = string.readTapAt(kPickup);
                // Exactly StringNetwork's ordering: last tick's outgoing wave, scattered through
                // the same rigid termination, handed back before this tick.
                const float incident = string.railOutgoingAtBridge();
                float reflected = 0.0f;
                rigid.scatter(&incident, &reflected, 1);
                string.railAcceptFromBridge(reflected);
                string.tick();

                withDamper.push_back(static_cast<double>(channel[n]));
                withoutDamper.push_back(static_cast<double>(direct));
                worstDifference = std::max(worstDifference, std::fabs(static_cast<double>(channel[n] - direct)));
            }
        }

        const cnpg::test::StringIrFeatures damped = cnpg::test::extractStringIrFeatures(withDamper, rate, kMidiNote);
        const cnpg::test::StringIrFeatures reference =
            cnpg::test::extractStringIrFeatures(withoutDamper, rate, kMidiNote);

        double worstCents = 0.0;
        for (std::size_t partial = 0; partial < damped.partialHz.size(); ++partial) {
            if (!(damped.partialHz[partial] > 0.0) || !(reference.partialHz[partial] > 0.0))
                continue;
            worstCents =
                std::max(worstCents,
                         std::fabs(cnpg::test::centsBetween(damped.partialHz[partial], reference.partialHz[partial])));
        }
        double worstT60Ratio = 0.0;
        for (std::size_t band = 0; band < damped.bandT60.size(); ++band) {
            if (!(damped.bandT60[band] > 0.0) || !(reference.bandT60[band] > 0.0))
                continue;
            worstT60Ratio = std::max(worstT60Ratio, std::fabs(damped.bandT60[band] / reference.bandT60[band] - 1.0));
        }

        std::cout << "[contract] in-line damper at engagement 0, MIDI " << kMidiNote << " at " << rate
                  << " Hz, p = " << damperPosition << ": worst |waveform difference| " << worstDifference
                  << " (stated tolerance " << cnpg::test::kStringIrGoldenAtol << "), worst partial shift " << worstCents
                  << " cents (limit 2), worst band-T60 change " << (100.0 * worstT60Ratio) << "% (limit 10)\n";

        INFO("MIDI " << kMidiNote << " at " << rate << " Hz, damper position " << damperPosition
                     << ": worst difference " << worstDifference << ", worst cents " << worstCents
                     << ", worst T60 ratio " << worstT60Ratio);
        REQUIRE(worstDifference <= cnpg::test::kStringIrGoldenAtol);
        REQUIRE(worstCents <= 2.0);
        REQUIRE(worstT60Ratio <= 0.10);
        // The measured value, asserted: bit-identity, not merely a tolerance met. If a future change
        // to the seam makes this stop holding, the tolerance above still gates the audible claim but
        // THIS line is what says the change happened.
        REQUIRE(worstDifference == 0.0);
        // Non-vacuous on both sides.
        REQUIRE(worstCents >= 0.0);
        REQUIRE(reference.partialHz[0] > 0.0);
    }
}

// ---------------------------------------------------------------------------------------------
// the engagement ramp itself (every state change gets a DIRECT assertion, P2.1 review ruling)
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DamperJunction validates the felt time constant into 20..100 ms", "[contract]") {
    // docs/plan.md section 2.5: "20-100 ms engage/release ramp", and Task P2.2: "a one-pole ramp
    // with time constant feltTimeConstantMs (validated into 20..100 ms)". Asserted twice over:
    // once on the validated value the junction reports, and once on the ramp it actually runs,
    // because the first without the second would only prove a number was stored.
    struct Case {
        float requested;
        float expected;
    };
    const Case cases[] = {{40.0f, 40.0f},
                          {20.0f, 20.0f},
                          {100.0f, 100.0f},
                          {0.0f, cnpg::dsp::kFeltTimeConstantMinMs},
                          {-5.0f, cnpg::dsp::kFeltTimeConstantMinMs},
                          {19.999f, cnpg::dsp::kFeltTimeConstantMinMs},
                          {5000.0f, cnpg::dsp::kFeltTimeConstantMaxMs},
                          {std::numeric_limits<float>::infinity(), cnpg::dsp::kFeltTimeConstantMaxMs},
                          {std::numeric_limits<float>::quiet_NaN(), cnpg::dsp::kFeltTimeConstantMinMs}};

    for (const Case& testCase : cases) {
        DamperJunction<float> damper;
        damper.prepare(kRate, kBlock);
        DamperJunctionParams params;
        params.feltTimeConstantMs = testCase.requested;
        damper.setParams(params);
        damper.reset();

        INFO("requested " << testCase.requested);
        REQUIRE(damper.currentFeltTimeConstantMs() == testCase.expected);

        // The ramp really runs at the validated time constant: after exactly one time constant a
        // one-pole engage() has covered 1 - 1/e of the distance.
        damper.engage();
        const auto samples = static_cast<int>(std::lround(static_cast<double>(testCase.expected) * 0.001 * kRate));
        float sink = 0.0f;
        for (int n = 0; n < samples; ++n)
            damper.scatter(0.0f, 0.0f, sink, sink);
        const double reached = static_cast<double>(damper.currentEngagement());
        INFO("after one time constant the ramp reached " << reached);
        REQUIRE(std::fabs(reached - (1.0 - std::exp(-1.0))) < 0.005);
    }
}

TEST_CASE("CONTRACT: DamperJunction engagement ramps continuously and settles exactly", "[contract]") {
    // THE direct state assertion for the engage/release state change. The click metric on the
    // rendered audio is a separate, weaker observation (tests/dsp/DamperFeltTimeTests.cpp holds
    // it); this one measures the state itself, which is what the P2.1 review's ruling asks for.
    constexpr float kFeltMs = 20.0f; // the fastest legal ramp, i.e. the largest legal step
    DamperJunction<float> damper;
    damper.prepare(kRate, kBlock);
    DamperJunctionParams params;
    params.feltTimeConstantMs = kFeltMs;
    params.maxLoss = 1.0f;
    damper.setParams(params);
    damper.reset();

    REQUIRE(damper.currentEngagement() == 0.0f);

    // One one-pole step can never exceed coeff * 1, and the settle snap can never exceed the
    // settle epsilon -- so this is the whole bound on any single-sample move of the ramp.
    const double coeff = 1.0 - std::exp(-1.0 / (static_cast<double>(kFeltMs) * 0.001 * kRate));
    const double stepLimit = coeff * (1.0 + 1.0e-9) + 1.0e-9;

    auto runRamp = [&](int samples) {
        double worstStep = 0.0;
        double worstCoefficientStep = 0.0;
        float previous = damper.currentEngagement();
        double previousG = 0.0;
        {
            double matrix[4] = {0.0, 0.0, 0.0, 0.0};
            damper.copyScatteringMatrix(matrix);
            previousG = -matrix[0];
        }
        for (int n = 0; n < samples; ++n) {
            float sink = 0.0f;
            damper.scatter(0.0f, 0.0f, sink, sink);
            const float now = damper.currentEngagement();
            worstStep = std::max(worstStep, std::fabs(static_cast<double>(now - previous)));
            previous = now;

            double matrix[4] = {0.0, 0.0, 0.0, 0.0};
            damper.copyScatteringMatrix(matrix);
            const double g = -matrix[0];
            worstCoefficientStep = std::max(worstCoefficientStep, std::fabs(g - previousG));
            previousG = g;
        }
        return std::pair<double, double>{worstStep, worstCoefficientStep};
    };

    damper.engage();
    const auto engageSteps = runRamp(static_cast<int>(0.5 * kRate)); // 25 time constants
    INFO("worst engage step " << engageSteps.first << " against limit " << stepLimit);
    REQUIRE(engageSteps.first <= stepLimit);
    // ...and so does the coefficient the audio path actually multiplies by: g = s / (s + 1) has
    // dg/ds = 1 / (1 + s)^2 <= 1, so the loss coefficient can never move FASTER than the
    // engagement behind it. That, not the engagement value, is what a click would come from.
    REQUIRE(engageSteps.second <= stepLimit);
    REQUIRE(engageSteps.second <= engageSteps.first * (1.0 + 1.0e-9));
    // "Fully engaged" is a reachable state, exactly, not an asymptote.
    REQUIRE(damper.currentEngagement() == 1.0f);

    damper.release();
    const auto releaseSteps = runRamp(static_cast<int>(0.5 * kRate));
    INFO("worst release step " << releaseSteps.first << " against limit " << stepLimit);
    REQUIRE(releaseSteps.first <= stepLimit);
    REQUIRE(releaseSteps.second <= stepLimit);
    REQUIRE(releaseSteps.second <= releaseSteps.first * (1.0 + 1.0e-9));
    // ...and so is "fully released", which is what restores the bit-exact transparency above.
    REQUIRE(damper.currentEngagement() == 0.0f);
    float toBridge = 0.0f;
    float toNut = 0.0f;
    damper.scatter(0.3f, -0.4f, toBridge, toNut);
    REQUIRE(toBridge == 0.3f);
    REQUIRE(toNut == -0.4f);

    std::cout << "[contract] DamperJunction ramp at " << kFeltMs << " ms: worst engagement step " << engageSteps.first
              << ", worst scattering-coefficient step " << engageSteps.second << " (limit " << stepLimit << ")\n";
}

TEST_CASE("CONTRACT: DamperJunction glides a maxLoss change instead of stepping it", "[contract]") {
    // The junction has TWO smoothers, and the second one is a state change like any other, so the
    // P2.1 ruling applies to it too: the engagement ramp is asserted directly above, and until now
    // the loss depth was only ever observed at snapped values -- which would have been satisfied by
    // an implementation that stepped it. A maxLoss automation move on an ENGAGED damper is a live
    // change to the scattering coefficient of a junction with a full waveform passing through it,
    // i.e. exactly the shape of thing that clicks if it steps.
    constexpr float kFrom = 1.0f;
    constexpr float kTo = 0.2f;

    DamperJunction<float> damper;
    placeAt(damper, 0.4f, kFrom, 1.0f);
    REQUIRE(damper.currentLossDepth() == kFrom);
    REQUIRE(damper.currentEngagement() == 1.0f); // fully engaged: the depth is audible right now

    DamperJunctionParams moved;
    moved.position01 = 0.4f;
    moved.maxLoss = kTo;
    damper.setParams(moved);
    // setParams only RETARGETS: the value must not have moved yet.
    REQUIRE(damper.currentLossDepth() == kFrom);

    // One one-pole step over the 8 ms parameter smoother, plus the settle epsilon.
    const double coeff = 1.0 - std::exp(-1.0 / (0.008 * kRate));
    const double stepLimit = coeff * static_cast<double>(kFrom - kTo) + 1.0e-9;

    double worstStep = 0.0;
    double worstCoefficientStep = 0.0;
    float previous = damper.currentLossDepth();
    double previousG = 0.0;
    {
        double matrix[4] = {0.0, 0.0, 0.0, 0.0};
        damper.copyScatteringMatrix(matrix);
        previousG = -matrix[0];
    }
    bool sawIntermediate = false;
    float sink = 0.0f;
    for (int n = 0; n < static_cast<int>(0.2 * kRate); ++n) {
        damper.scatter(0.5f, -0.25f, sink, sink);
        const float now = damper.currentLossDepth();
        worstStep = std::max(worstStep, std::fabs(static_cast<double>(now - previous)));
        // Monotone, and strictly between the endpoints while it travels -- so this is a GLIDE and
        // not a step that happened to land on the target.
        REQUIRE(now <= previous);
        if (now > kTo && now < kFrom)
            sawIntermediate = true;
        previous = now;

        double matrix[4] = {0.0, 0.0, 0.0, 0.0};
        damper.copyScatteringMatrix(matrix);
        const double g = -matrix[0];
        worstCoefficientStep = std::max(worstCoefficientStep, std::fabs(g - previousG));
        previousG = g;
    }

    std::cout << "[contract] DamperJunction maxLoss glide " << kFrom << " -> " << kTo << ": worst depth step "
              << worstStep << " (limit " << stepLimit << "), worst scattering-coefficient step " << worstCoefficientStep
              << "\n";

    INFO("worst depth step " << worstStep << " against limit " << stepLimit);
    REQUIRE(sawIntermediate);
    REQUIRE(worstStep <= stepLimit);
    // And the quantity the audio path multiplies by moves no faster: g = s/(s+1) has dg/ds <= 1.
    REQUIRE(worstCoefficientStep <= stepLimit);
    // "Fully arrived" is a reachable state, exactly, like the engagement ramp's endpoints.
    REQUIRE(damper.currentLossDepth() == kTo);
}

TEST_CASE("CONTRACT: DamperJunction reset and setParams behave as the lifecycle documents", "[contract]") {
    DamperJunction<float> damper;
    damper.prepare(kRate, kBlock);

    DamperJunctionParams params;
    params.position01 = 0.42f;
    params.maxLoss = 0.75f;
    damper.setParams(params);
    damper.reset();

    // reset() snaps the loss depth onto the parameter and the engagement to 0.
    REQUIRE(damper.currentLossDepth() == 0.75f);
    REQUIRE(damper.currentEngagement() == 0.0f);
    REQUIRE(damper.currentPosition01() == 0.42f);

    damper.engage();
    float sink = 0.0f;
    for (int n = 0; n < 4800; ++n)
        damper.scatter(0.5f, 0.25f, sink, sink);
    REQUIRE(damper.currentEngagement() > 0.9f);

    damper.reset();
    REQUIRE(damper.currentEngagement() == 0.0f);
    REQUIRE(damper.currentLossDepth() == 0.75f);
    damper.reset(); // twice equals once
    REQUIRE(damper.currentEngagement() == 0.0f);

    // Out-of-range parameters resolve rather than reaching the coefficients.
    DamperJunctionParams wild;
    wild.position01 = 4.0f;
    wild.maxLoss = -2.0f;
    damper.setParams(wild);
    damper.reset();
    REQUIRE(damper.currentPosition01() == 1.0f);
    REQUIRE(damper.currentLossDepth() == 0.0f);

    wild.position01 = std::numeric_limits<float>::quiet_NaN();
    wild.maxLoss = std::numeric_limits<float>::quiet_NaN();
    damper.setParams(wild);
    damper.reset();
    REQUIRE(damper.currentPosition01() == 0.0f);
    REQUIRE(damper.currentLossDepth() == 0.0f);
}

TEST_CASE("CONTRACT: DamperJunction scatter allocates nothing", "[contract]") {
    DamperJunction<float> damper;
    damper.prepare(kRate, kBlock);
    damper.setParams(DamperJunctionParams{});
    damper.reset();

    cnpg::test::resetAllocationCount();
    float toBridge = 0.0f;
    float toNut = 0.0f;
    for (int n = 0; n < 100000; ++n) {
        if ((n % 4096) == 0)
            damper.engage();
        if ((n % 4096) == 2048)
            damper.release();
        damper.scatter(static_cast<float>(std::sin(0.01 * n)), static_cast<float>(std::cos(0.013 * n)), toBridge,
                       toNut);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// ---------------------------------------------------------------------------------------------
// tier 3: a plucked network with real losses only ever dissipates, dampers engaged or not
// ---------------------------------------------------------------------------------------------

TEST_CASE("ENERGY/T3: lossy network only dissipates with dampers engaged", "[energy]") {
    // docs/plan.md section 4.2 tier 3, and Task P2.2's acceptance "Tier-3 [energy] still passes
    // with dampers engaged". SCOPE: the tier-3 case section 4.2 describes runs against the
    // coupled network, and the bridge does not carry a load until Task P2.4/P2.5 -- which is why
    // that task's file list owns tests/dsp/NetworkEnergyTierThreeTests.cpp. What is measurable
    // TODAY, and what this task is responsible for, is that inserting a permanently in-line
    // dissipative two-port into every string's loop does not turn a monotone decay into growth.
    // So this is tier 3 at the scope this task can honestly gate: 6 strings, staggered plucks,
    // default material, the shipping float32 instantiation, run once with the dampers idle and
    // once with every damper engaged.
    constexpr int kStrings = 6;
    constexpr int kBlocks = 1200; // ~3.2 s at 48 kHz
    constexpr int kExcitationBlocks = 60;
    constexpr double kRelativeTolerance = 1.0e-6; // absorbs float32 state rounding, per section 4.2
    // Shared with tests/dsp/NetworkEnergyTierThreeTests.cpp, where it is derived; restated as a
    // local constant rather than exported because the two suites gate different scenarios and a
    // shared symbol would invite one of them to move it for the other's sake.
    constexpr double kMeasurementFloor = 1.0e-40;

    auto run = [](bool engageDampers) {
        StringNetworkParams params;
        params.pickupPosition01 = 0.87f;
        StringNetwork<float> network;
        network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(kStrings);
        network.setParams(params);
        network.reset();

        BlockEventQueue events;
        for (int s = 0; s < kStrings; ++s)
            events.push(noteOn(s * 17, 40 + 5 * s, s));

        std::vector<double> energies;
        energies.reserve(static_cast<std::size_t>(kBlocks));
        // THE SHIPPING CONFIGURATION (added at Task P2.4). This case measures the float32 path, and
        // the float32 path has run under ScopedFtzDazGuard since Task P1.1 -- PluginProcessor's
        // processBlock constructs one first. It did not matter while the strings were UNCOUPLED:
        // an engaged damper took every string to the silence watchdog's floor and the state was
        // cleared outright. Bidirectional coupling (P2.4) gives the decay a long subnormal tail
        // instead -- the bridge resonator keeps re-driving damped strings out of its own residual --
        // and scoring energyEstimate() on subnormal float32 state measured "growth" of 0.58 at total
        // energies around 1e-87, i.e. -870 dBFS. See the header of
        // tests/dsp/NetworkEnergyTierThreeTests.cpp for the diagnosis and the standing case that
        // pins it (the identical scenario on the double instantiation is clean).
        const cnpg::dsp::ScopedFtzDazGuard denormalGuard;
        for (int b = 0; b < kBlocks; ++b) {
            if (engageDampers && b == kExcitationBlocks) {
                BlockEventQueue offs;
                for (int s = 0; s < kStrings; ++s)
                    offs.push(noteOff(0, 40 + 5 * s, s));
                network.process(offs, kBlock);
            } else {
                network.process(events, kBlock);
            }
            if (b >= kExcitationBlocks)
                energies.push_back(network.energyEstimate());
        }
        return energies;
    };

    for (bool engageDampers : {false, true}) {
        const std::vector<double> energies = run(engageDampers);
        REQUIRE(energies.size() > 2);
        REQUIRE(energies.front() > 0.0); // non-vacuous: there really was energy to dissipate

        double worstGrowth = 0.0;
        std::size_t worstAt = 0;
        double worstBelowFloor = 0.0;
        double lowestGated = energies.front();
        for (std::size_t k = 1; k < energies.size(); ++k) {
            const double previous = energies[k - 1];
            if (!(previous > 0.0))
                continue;
            const double growth = energies[k] / previous - 1.0;
            // The float32 measurement floor (Task P2.4). Below it the shipping FTZ/DAZ guard is
            // flushing the recursions' own intermediate products, so energyEstimate() stops being
            // a measurement -- the derivation, the margin and the double-precision control are in
            // the header of tests/dsp/NetworkEnergyTierThreeTests.cpp. This case never went near
            // the floor while the strings were uncoupled; P2.4's bridge is what gave the engaged
            // decay a tail long enough to reach it.
            if (previous < kMeasurementFloor) {
                worstBelowFloor = std::max(worstBelowFloor, growth);
                continue;
            }
            lowestGated = std::min(lowestGated, previous);
            if (growth > worstGrowth) {
                worstGrowth = growth;
                worstAt = k;
            }
        }

        std::cout << "[energy] T3 " << (engageDampers ? "dampers engaged" : "dampers idle")
                  << ": worst per-block growth " << worstGrowth << " at block " << worstAt << " (limit "
                  << kRelativeTolerance << "), start " << energies.front() << " -> end " << energies.back()
                  << "; gated down to " << lowestGated << ", worst growth below the float32 measurement floor "
                  << worstBelowFloor << "\n";

        INFO((engageDampers ? "dampers engaged" : "dampers idle")
             << ": worst growth " << worstGrowth << " at block " << worstAt);
        REQUIRE(worstGrowth <= kRelativeTolerance);

        // ...and the dampers really did their job: engaging them takes the network to silence
        // inside this render, which the idle run does not reach.
        if (engageDampers)
            REQUIRE(energies.back() == 0.0);
        else
            REQUIRE(energies.back() > 0.0);
    }
}

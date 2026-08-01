#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// MovingJunctionEnergyTests -- Task P2.3's tier-2 [energy] gate for the MOVING seam: does the
// dual-anchor amplitude-complementary crossfade create energy?
//
// ---------------------------------------------------------------------------------------------
// WHY THE PLAN'S OWN SCENARIO CANNOT ANSWER THAT, AND WHAT IS RUN INSTEAD
// ---------------------------------------------------------------------------------------------
// docs/plan.md section 4.2 tier 2 names three motion scenarios on a lossless network: static
// positions, `damperPosition01` swept 0.1->0.9, and `pickupPosition01` swept likewise. Two of the
// three are VACUOUS by construction, and the P2.3 brief asked for that to be confirmed by
// measurement rather than inherited:
//
//   - `setLosslessTestMode(true)` forwards `setLossBypassed(true)` to every DamperJunction, and a
//     bypassed junction is a BIT-EXACT pass-through. WaveguideString's seam deposits only the
//     DIFFERENCE between the junction's outputs and the waves it read through the same crossfaded
//     functional, so a pass-through deposits exactly 0.0 at both anchors -- at every position, and
//     mid-crossfade too (tests/dsp/MovingPositionClickTests.cpp asserts that directly). Sweeping the
//     damper position under lossless mode therefore cannot perturb one sample. Measured below as
//     bit-identity of the whole energy trace against a frozen-position run, which is the strongest
//     possible statement of the vacuity -- and the reason it is stated rather than deleted.
//   - `readTapAt` is a pure READ: it never deposits into the rails. A pickup sweep cannot inject
//     energy for a structural reason, not a numerical one, and that too is measured below as
//     bit-identity of the energy trace while the AUDIO differs.
//
// So the scenario that actually exercises the moving seam is the third case here: a LOSSLESS string
// carrying a DISSIPATIVE, fully engaged damper whose position is swept. That is the only
// configuration in which the crossfade's write side deposits anything at all, and it is where a
// crossfade that leaked energy would show. It is run on the `double` instantiation, where the tier-2
// bound is justified, at all three rates and for both fractional-delay kinds.
//
// ---------------------------------------------------------------------------------------------
// WHAT IS ASSERTED, AND WHY IT IS NOT THE STRICT NON-INCREASE
// ---------------------------------------------------------------------------------------------
// Static-position runs get the strict 1e-9 per-block non-increase. Motion runs get BOUNDED GROWTH:
// cumulative energyEstimate() never exceeds its post-excitation maximum, and per-block growth stays
// within a stated tolerance. The distinction is the plan's (section 4.2) and it is not a hedge -- a
// unit gain norm on the junction does NOT prevent energy pumping, because the energy after
// superposition depends on the correlation between what is deposited and what the rails already
// carry. What DOES carry the bound here is the seam's adjointness: railDeposit is the exact
// transpose of railInterpolate, so the crossfaded read functional r = g1 r_A + g2 r_B has
// ||r|| <= g1 + g2 = 1 and the seam update x' = x + r^T(S(r x) - r x) is a contraction for every
// fade position (the identity is written out in WaveguideString.h). The measured per-block growth
// below is consequently at the float64 arithmetic floor rather than at any crossfade-derived
// tolerance, which is what the printed numbers say.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::DamperJunction;
using cnpg::dsp::DamperJunctionParams;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr int kBlock = 128;
constexpr double kTwoPi = 6.283185307179586;
constexpr int kStrings = 6;
constexpr int kChord[kStrings] = {40, 45, 50, 55, 59, 64};

// The tier-2 tolerance for a MOTION run. Derived from what the crossfade can do rather than fitted:
// the seam is a contraction at every fade position (see the header), so nothing but float64
// round-off is left, and 1e-9 -- the same number the static bound uses -- is already three orders of
// magnitude above the measured worst. Stated as its own constant so a future change that genuinely
// needs slack has to widen it deliberately.
constexpr double kMotionGrowthTolerance = 1.0e-9;
constexpr double kStaticGrowthTolerance = 1.0e-9;

const double kRates[3] = {44100.0, 48000.0, 96000.0};
const FractionalDelayKind kKinds[2] = {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1};

const char* kindName(FractionalDelayKind kind) {
    return kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1";
}

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = 0.8f;
    event.pluckPosition = 0.28f;
    event.hardness = 1.0f;
    return event;
}

struct NetworkRun {
    std::vector<double> energy; // one sample per block, after the excitation has finished
    std::vector<double> audio;  // the summed tap channels, so a "nothing changed" claim is checkable
};

// A six-string lossless network impulse, with `pickupSweep` / `damperSweep` driving the two
// positions 0.1 -> 0.9 over the whole render (0 = frozen at 0.5).
NetworkRun runNetwork(double sampleRate, FractionalDelayKind kind, double seconds, bool pickupSweep, bool damperSweep) {
    StringNetworkParams params;
    params.pickupPosition01 = 0.5f;
    params.damperPosition01 = 0.5f;
    params.damper.maxLoss = 1.0f;
    params.exciter.noiseAmount = 0.0f;

    StringNetwork<double> network;
    network.prepare(sampleRate, kBlock, kind);
    network.setNumStrings(kStrings);
    network.setParams(params);
    network.setLosslessTestMode(true);
    network.reset();

    BlockEventQueue events;
    for (int s = 0; s < kStrings; ++s)
        events.push(noteOn(s * 13, kChord[s], s));

    const auto totalBlocks = static_cast<int>(seconds * sampleRate / kBlock);
    const int excitationBlocks = std::max(4, static_cast<int>(0.05 * sampleRate / kBlock));

    NetworkRun out;
    out.energy.reserve(static_cast<std::size_t>(totalBlocks));
    out.audio.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));

    for (int b = 0; b < totalBlocks; ++b) {
        // A full traverse 0.1 -> 0.9 across the render, retargeted per block as a host would.
        const double phase = static_cast<double>(b) / static_cast<double>(std::max(1, totalBlocks - 1));
        const auto position = static_cast<float>(0.1 + 0.8 * phase);
        params.pickupPosition01 = pickupSweep ? position : 0.5f;
        params.damperPosition01 = damperSweep ? position : 0.5f;
        network.setParams(params);
        network.process(events, kBlock);

        const int channels = network.tapBuffers().numStrings();
        for (int n = 0; n < kBlock; ++n) {
            double sum = 0.0;
            for (int s = 0; s < channels; ++s) {
                const double* channel = network.tapBuffers().channel(s, 0);
                if (channel != nullptr)
                    sum += channel[n];
            }
            out.audio.push_back(sum);
        }
        if (b >= excitationBlocks)
            out.energy.push_back(network.energyEstimate());
    }
    return out;
}

struct GrowthReport {
    double worstGrowth = 0.0;
    std::size_t worstAt = 0;
    double maximum = 0.0;
    std::size_t maximumAt = 0;
    double overshootAboveMaximum = 0.0; // (max after the first block) / (running max before it) - 1
};

GrowthReport measureGrowth(const std::vector<double>& energy) {
    GrowthReport out;
    if (energy.empty())
        return out;
    out.maximum = energy.front();
    double runningMaximum = energy.front();
    for (std::size_t k = 1; k < energy.size(); ++k) {
        const double previous = energy[k - 1];
        const double growth = (previous > 0.0) ? (energy[k] / previous - 1.0) : 0.0;
        if (growth > out.worstGrowth) {
            out.worstGrowth = growth;
            out.worstAt = k;
        }
        if (energy[k] > out.maximum) {
            out.maximum = energy[k];
            out.maximumAt = k;
        }
        // "Never exceeds the POST-EXCITATION maximum" is a statement about the running maximum: the
        // first sampled block is the post-excitation peak of a passive system, so anything later that
        // rises above what has been seen so far is cumulative growth however slowly it accrued.
        if (energy[k] > runningMaximum) {
            out.overshootAboveMaximum = std::max(out.overshootAboveMaximum, energy[k] / runningMaximum - 1.0);
            runningMaximum = energy[k];
        }
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// tier 2, the plan's two motion scenarios -- and the measurement of what they can gate
// ---------------------------------------------------------------------------------------------

TEST_CASE("ENERGY/T2: a lossless network is inert to a damper-position sweep", "[energy]") {
    // THE G4 MEASUREMENT (see this file's header). Not "close enough" -- BIT-IDENTICAL, energy trace
    // and audio alike, because a bypassed DamperJunction is a bit-exact pass-through and the seam
    // deposits only the difference. Asserting the identity rather than a tolerance is what makes the
    // vacuity a recorded fact instead of a suspicion, and what would make any future change that
    // gives lossless mode a non-transparent junction fail loudly here rather than quietly turn this
    // scenario into a real (and then differently-gated) test.
    constexpr double kSeconds = 2.0;
    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : kKinds) {
            const NetworkRun swept = runNetwork(sampleRate, kind, kSeconds, false, true);
            const NetworkRun frozen = runNetwork(sampleRate, kind, kSeconds, false, false);
            REQUIRE(swept.energy.size() == frozen.energy.size());
            REQUIRE(swept.energy.size() > 2);
            REQUIRE(swept.energy.front() > 0.0); // non-vacuous: there really is energy in the network

            for (std::size_t k = 0; k < swept.energy.size(); ++k) {
                INFO(kindName(kind) << " @ " << sampleRate << " Hz, block " << k);
                REQUIRE(swept.energy[k] == frozen.energy[k]);
            }
            for (std::size_t n = 0; n < swept.audio.size(); ++n) {
                INFO(kindName(kind) << " @ " << sampleRate << " Hz, sample " << n);
                REQUIRE(swept.audio[n] == frozen.audio[n]);
            }

            // The scenario is still gated on the bound the plan names, so the case does not become
            // merely a documentation of its own vacuity.
            const GrowthReport report = measureGrowth(swept.energy);
            INFO(kindName(kind) << " @ " << sampleRate << ": worst growth " << report.worstGrowth);
            REQUIRE(report.worstGrowth <= kMotionGrowthTolerance);
            REQUIRE(report.overshootAboveMaximum <= kMotionGrowthTolerance);
        }
    }
    std::cout << "[energy] T2 lossless network, damperPosition01 swept 0.1 -> 0.9: BIT-IDENTICAL to the frozen run at "
                 "every rate and both interpolators. setLosslessTestMode(true) bypasses the junction's only loss, and "
                 "a transparent junction deposits exactly 0.0 through the seam at every position -- so this scenario "
                 "gates the crossfade's energy behaviour not at all. See ENERGY/T2 moving damper below.\n";
}

TEST_CASE("ENERGY/T2: a lossless network gains nothing from a pickup-position sweep", "[energy]") {
    // "tap reads must never inject energy" (docs/plan.md section 4.2). readTapAt does not write to
    // the rails at all, so the claim is structural and the measurement is again an identity rather
    // than a bound -- while the AUDIO differs, which is what says the sweep really happened.
    constexpr double kSeconds = 2.0;
    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : kKinds) {
            const NetworkRun swept = runNetwork(sampleRate, kind, kSeconds, true, false);
            const NetworkRun frozen = runNetwork(sampleRate, kind, kSeconds, false, false);
            REQUIRE(swept.energy.size() == frozen.energy.size());
            REQUIRE(swept.energy.front() > 0.0);

            for (std::size_t k = 0; k < swept.energy.size(); ++k) {
                INFO(kindName(kind) << " @ " << sampleRate << " Hz, block " << k);
                REQUIRE(swept.energy[k] == frozen.energy[k]);
            }

            bool audioDiffers = false;
            for (std::size_t n = 0; n < swept.audio.size() && !audioDiffers; ++n)
                audioDiffers = (swept.audio[n] != frozen.audio[n]);
            REQUIRE(audioDiffers); // non-vacuous: the tap really moved

            const GrowthReport report = measureGrowth(swept.energy);
            REQUIRE(report.worstGrowth <= kMotionGrowthTolerance);
            REQUIRE(report.overshootAboveMaximum <= kMotionGrowthTolerance);
        }
    }
    std::cout << "[energy] T2 lossless network, pickupPosition01 swept 0.1 -> 0.9: energy trace BIT-IDENTICAL to the "
                 "frozen run while the audio differs -- readTapAt never deposits into the rails.\n";
}

TEST_CASE("ENERGY/T2: a lossless network only dissipates with both positions static", "[energy]") {
    // The strict per-block non-increase, which per docs/plan.md section 4.2 applies to
    // static-position runs only. This is the baseline the motion runs above are measured against.
    constexpr double kSeconds = 2.0;
    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : kKinds) {
            const NetworkRun still = runNetwork(sampleRate, kind, kSeconds, false, false);
            REQUIRE(still.energy.size() > 2);
            REQUIRE(still.energy.front() > 0.0);
            const GrowthReport report = measureGrowth(still.energy);
            std::cout << "[energy] T2 static " << kindName(kind) << " @ " << sampleRate
                      << " Hz: worst per-block growth " << report.worstGrowth << " at block " << report.worstAt
                      << " (limit " << kStaticGrowthTolerance << "), energy " << still.energy.front() << " -> "
                      << still.energy.back() << "\n";
            INFO(kindName(kind) << " @ " << sampleRate << ": worst growth " << report.worstGrowth << " at block "
                                << report.worstAt);
            REQUIRE(report.worstGrowth <= kStaticGrowthTolerance);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// tier 2, the scenario that actually exercises the moving seam
// ---------------------------------------------------------------------------------------------

namespace {

struct SeamRun {
    std::vector<double> energy;
    double worstDeposit = 0.0; // largest |difference| the seam ever wrote into a rail
    int midFadeSamples = 0;
    double positionLow = 1.0;
    double positionHigh = 0.0;
};

// A lossless string with a fully engaged, DISSIPATIVE damper on it, the junction position driven
// 0.1 -> 0.9 over the render (`sweepHz` 0), sinusoidally between the same endpoints at `sweepHz`,
// or held at 0.5 (`sweep` false). This is the tier-2 configuration the plan's lossless-everything
// scenario cannot reach: the string's own loop loss is bypassed, so the ONLY element that can move
// energy is the moving junction itself.
SeamRun runMovingSeam(double sampleRate, FractionalDelayKind kind, double seconds, float maxLoss, bool sweep,
                      double sweepHz = 0.0) {
    WaveguideString<double> string;
    string.prepare(sampleRate, kBlock, kind);
    WaveguideStringParams params;
    params.f0Hz = 110.0f;
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset();
    string.setLossBypassed(true); // the string is lossless; the junction is not

    DamperJunction<double> damper;
    damper.prepare(sampleRate, kBlock);
    DamperJunctionParams damperParams;
    damperParams.maxLoss = maxLoss;
    damper.setParams(damperParams);
    damper.reset();
    damper.setEngagementImmediate(1.0f);
    REQUIRE(damper.currentEngagement() == 1.0f); // IN the state this run claims to be in
    REQUIRE(damper.currentLossDepth() == maxLoss);

    cnpg::dsp::PluckExciter<double> exciter;
    exciter.prepare(sampleRate, kBlock);
    cnpg::dsp::PluckExciterParams exciterParams;
    exciterParams.noiseAmount = 0.0f;
    exciter.setParams(exciterParams);
    exciter.trigger(0.8f, 0.28f, 1.0f);

    const auto totalSamples = static_cast<int>(seconds * sampleRate);
    const int excitationSamples = std::max(kBlock, static_cast<int>(0.05 * sampleRate));

    // The same 8 ms one-pole StringNetwork applies, so the seam sees the staircase the shipping path
    // hands it rather than a raw ramp.
    const double smoothingCoeff = 1.0 - std::exp(-1.0 / (0.008 * sampleRate));
    double smoothed = (sweep && sweepHz <= 0.0) ? 0.1 : 0.5;

    SeamRun out;
    out.energy.reserve(static_cast<std::size_t>(totalSamples / kBlock));
    for (int n = 0; n < totalSamples; ++n) {
        const double excitation = exciter.renderSample();
        if (excitation != 0.0)
            string.injectAt(exciter.latchedPosition01(), excitation);

        double target = 0.5;
        if (sweep) {
            target = (sweepHz > 0.0)
                         ? 0.5 + 0.4 * std::sin(kTwoPi * sweepHz * static_cast<double>(n) / sampleRate)
                         : 0.1 + 0.8 * static_cast<double>(n) / static_cast<double>(std::max(1, totalSamples - 1));
        }
        smoothed += smoothingCoeff * (target - smoothed);
        const auto position = static_cast<float>(smoothed);

        double fromNut = 0.0;
        double fromBridge = 0.0;
        string.readJunctionInputs(position, fromNut, fromBridge);
        double toBridge = 0.0;
        double toNut = 0.0;
        damper.scatter(fromNut, fromBridge, toBridge, toNut);
        string.writeJunctionOutputs(position, toBridge, toNut);

        const auto crossfade = string.junctionCrossfade();
        if (crossfade.crossfade01 > 0.0f)
            ++out.midFadeSamples;
        const double effective =
            (1.0 - static_cast<double>(crossfade.crossfade01)) * static_cast<double>(crossfade.anchor01) +
            static_cast<double>(crossfade.crossfade01) * static_cast<double>(crossfade.pending01);
        out.positionLow = std::min(out.positionLow, effective);
        out.positionHigh = std::max(out.positionHigh, effective);
        // What the seam actually deposited: a transparent junction would make both of these exactly
        // 0, which is how the plan's lossless scenario goes vacuous. Here they must not be.
        out.worstDeposit = std::max({out.worstDeposit, std::fabs(toBridge - fromNut), std::fabs(toNut - fromBridge)});

        string.tick();
        if (n >= excitationSamples && ((n + 1) % kBlock) == 0)
            out.energy.push_back(string.energyEstimate());
    }
    return out;
}

} // namespace

TEST_CASE("ENERGY/T2: a moving dissipative junction on a lossless string never creates energy", "[energy]") {
    // THE tier-2 case for the moving seam. The plan's lossless-everything damper sweep cannot gate
    // this (see the file header and the ENERGY/T2 case above, which measures the vacuity), so the
    // configuration is inverted: the STRING's loss is bypassed and the JUNCTION's is not. Everything
    // that can move energy is then the moving seam plus a passive dashpot, and a crossfade that
    // pumped would have nothing to hide behind.
    constexpr double kSeconds = 2.0;
    constexpr float kMaxLoss = 0.02f; // deep enough to matter, shallow enough to leave a tail to watch

    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : kKinds) {
            const SeamRun swept = runMovingSeam(sampleRate, kind, kSeconds, kMaxLoss, true);
            const SeamRun still = runMovingSeam(sampleRate, kind, kSeconds, kMaxLoss, false);
            REQUIRE(swept.energy.size() > 2);
            REQUIRE(swept.energy.front() > 0.0);

            // IN the state this case claims to test, all four halves of it: the junction really
            // scatters (so the seam deposits something), it really moved across the string, it
            // really spent time mid-crossfade, and the sweep really changed the outcome.
            INFO(kindName(kind) << " @ " << sampleRate << " Hz");
            REQUIRE(swept.worstDeposit > 0.0);
            REQUIRE(swept.positionLow < 0.15);
            REQUIRE(swept.positionHigh > 0.85);
            REQUIRE(swept.midFadeSamples > 1000);
            REQUIRE(swept.energy.back() != still.energy.back());
            // The static counterpart must never crossfade at all, so the two runs really are the
            // two sides of the comparison rather than two flavours of the same thing.
            REQUIRE(still.midFadeSamples == 0);
            // ...and the damper really is dissipating, so "never grows" is not a statement about a
            // system with nothing happening in it.
            REQUIRE(swept.energy.back() < swept.energy.front());

            const GrowthReport moving = measureGrowth(swept.energy);
            const GrowthReport stationary = measureGrowth(still.energy);

            std::cout << "[energy] T2 moving damper " << kindName(kind) << " @ " << sampleRate
                      << " Hz: worst per-block growth " << moving.worstGrowth << " at block " << moving.worstAt
                      << ", cumulative overshoot above the post-excitation maximum " << moving.overshootAboveMaximum
                      << " (limit " << kMotionGrowthTolerance << "); " << swept.midFadeSamples
                      << " samples mid-crossfade, worst seam deposit " << swept.worstDeposit << ", energy "
                      << swept.energy.front() << " -> " << swept.energy.back() << ". Static counterpart: worst growth "
                      << stationary.worstGrowth << " (limit " << kStaticGrowthTolerance << ")\n";

            // BOUNDED GROWTH for the motion run (docs/plan.md section 4.2), both halves of it.
            INFO("moving worst growth " << moving.worstGrowth << ", overshoot " << moving.overshootAboveMaximum);
            REQUIRE(moving.worstGrowth <= kMotionGrowthTolerance);
            REQUIRE(moving.overshootAboveMaximum <= kMotionGrowthTolerance);
            // ...and the STRICT non-increase for the static counterpart, which is what says the
            // motion tolerance above is not quietly absorbing a defect the static case would catch.
            REQUIRE(stationary.worstGrowth <= kStaticGrowthTolerance);
        }
    }
}

TEST_CASE("ENERGY/T2: a moving junction stays passive at full depth and full crossfade duty", "[energy]") {
    // The two corners the plan's single 0.1 -> 0.9 traverse does not reach, taken together:
    //
    //   - FULL DEPTH (maxLoss 1.0, the matched resistive termination), where the junction
    //     annihilates the whole symmetric mode on every pass and the seam's deposits are therefore
    //     as large as they can ever get -- which is where a crossfade that failed to be the exact
    //     transpose of its own read would show first.
    //   - FULL CROSSFADE DUTY: a 5 Hz sinusoidal sweep crosses the threshold faster than one fade
    //     completes over most of its cycle, so fades chain back to back and the seam reads and
    //     writes through two anchors for about 63% of the render (the remainder is the turning
    //     points, where the sinusoid's own rate falls to zero and no fade is due). The single
    //     traverse above spends only a few per cent of its samples there, which is enough to be
    //     non-vacuous and not enough to be the worst case.
    constexpr double kSeconds = 1.0;
    const SeamRun swept = runMovingSeam(48000.0, FractionalDelayKind::Lagrange3, kSeconds, 1.0f, true, 5.0);
    REQUIRE(swept.energy.size() > 2);
    REQUIRE(swept.energy.front() > 0.0);
    REQUIRE(swept.worstDeposit > 0.0);
    // IN the corner this case claims to test: the seam really is crossfading most of the time.
    REQUIRE(swept.midFadeSamples > static_cast<int>(0.5 * kSeconds * 48000.0));

    const GrowthReport report = measureGrowth(swept.energy);
    std::cout << "[energy] T2 moving damper at full depth, 5 Hz sweep: " << swept.midFadeSamples << " of "
              << static_cast<int>(kSeconds * 48000.0) << " samples mid-crossfade; worst per-block growth "
              << report.worstGrowth << ", overshoot " << report.overshootAboveMaximum << " (limit "
              << kMotionGrowthTolerance << "), worst seam deposit " << swept.worstDeposit << ", energy "
              << swept.energy.front() << " -> " << swept.energy.back() << "\n";
    INFO("worst growth " << report.worstGrowth << ", overshoot " << report.overshootAboveMaximum);
    REQUIRE(report.worstGrowth <= kMotionGrowthTolerance);
    REQUIRE(report.overshootAboveMaximum <= kMotionGrowthTolerance);
}

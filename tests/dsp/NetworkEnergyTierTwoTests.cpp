#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// NetworkEnergyTierTwoTests -- Task P2.4's tier-2 [energy] gate: with every intentional loss
// disabled, does the ASSEMBLED coupled network create energy?
//
// docs/plan.md section 4.2 defines the tier and the bound; this file references them and restates
// neither. Tier 1 (tests/dsp/BridgeEnergyTierOneTests.cpp) proved the junction's algebra is
// passive pointwise; tier 2 asks whether the assembly around it -- six delay rails, fractional
// interpolation, the moving damper seam, the enable gains, the one-sample bridge seam and the
// power normalization across it -- can still pump. A tier-2 failure with tier 1 green points at
// the assembly, never at the junction, which is the whole reason they are separate cases.
//
// ---------------------------------------------------------------------------------------------
// WHY THIS SCENARIO IS NOT VACUOUS, unlike the damper one it is modelled on
// ---------------------------------------------------------------------------------------------
// The P2.3 amendment to section 4.2 records that the plan's lossless damper-motion scenario tested
// nothing: setLosslessTestMode(true) makes DamperJunction TRANSPARENT, so the element under test
// disappeared and the case passed identically against an implementation with no crossfade at all.
// The bridge is structurally different and the difference is worth stating plainly:
// setLossBypassed(true) removes only the junction's DASHPOT, leaving a lossless mass-spring
// resonator that still couples every string to every other one. The coupling -- the thing this
// tier exists to gate -- is fully present in lossless mode, and the case below MEASURES that it is
// (against a couplingStrength-0 run, which must differ) rather than asserting it.
//
// ---------------------------------------------------------------------------------------------
// AND WHY THE FUNCTIONAL HAS TO INCLUDE THE BRIDGE
// ---------------------------------------------------------------------------------------------
// energyEstimate() is a Lyapunov storage functional, not rail energy: rail-only accounting was a
// reviewer-caught blocker at P1.4 because energy migrates between the rails and the filter states
// within a block. The bridge adds two more places for it to migrate to -- the mass and the spring
// -- plus one per string at the seam register. The last case in this file measures what leaving
// them out would have cost, so "the functional includes them" is a number rather than a claim.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr int kBlock = 128;
constexpr int kStrings = 6;
constexpr int kChord[kStrings] = {40, 45, 50, 55, 59, 64};

// docs/plan.md section 4.2 tier 2: "non-increasing per block within 1e-9 relative tolerance",
// justified on the double instantiation and nowhere else.
constexpr double kTolerance = 1.0e-9;

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

struct Tier2Run {
    std::vector<double> energy;      // the full storage functional, one sample per block
    std::vector<double> stringsOnly; // ...with the junction's own store left OUT
    std::vector<double> bridgeStore; // the junction's store alone
    std::vector<double> audio;       // summed taps, so "the runs differ" is checkable
    double peakBridgeStore = 0.0;
    unsigned long long unbridgedTicks = 0;
};

struct Tier2Spec {
    double sampleRate = 48000.0;
    FractionalDelayKind kind = FractionalDelayKind::Lagrange3;
    double seconds = 6.0;
    float coupling = 0.35f;
    float resonanceHz = 180.0f;
    float damping = 0.5f;
};

Tier2Run runLosslessNetwork(const Tier2Spec& spec) {
    StringNetworkParams params;
    params.pickupPosition01 = 0.5f;
    params.damperPosition01 = 0.5f;
    params.damper.maxLoss = 1.0f;
    params.exciter.noiseAmount = 0.0f;
    params.bridge.couplingStrength = spec.coupling;
    params.bridge.resonanceHz = spec.resonanceHz;
    params.bridge.damping = spec.damping;

    StringNetwork<double> network;
    network.prepare(spec.sampleRate, kBlock, spec.kind);
    network.setNumStrings(kStrings);
    network.setParams(params);
    network.setLosslessTestMode(true);
    network.reset();

    BlockEventQueue events;
    for (int s = 0; s < kStrings; ++s)
        events.push(noteOn(s * 13, kChord[s], s));

    const auto totalBlocks = static_cast<int>(spec.seconds * spec.sampleRate / kBlock);
    // Sampling starts after the excitation bursts have finished: the exciter is an INPUT, so a
    // functional that included the burst would legitimately rise while it is still injecting.
    const int excitationBlocks = std::max(4, static_cast<int>(0.05 * spec.sampleRate / kBlock));

    Tier2Run out;
    out.energy.reserve(static_cast<std::size_t>(totalBlocks));
    for (int b = 0; b < totalBlocks; ++b) {
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
        if (b >= excitationBlocks) {
            const double store = network.internalBridgeJunction().storageEnergy();
            out.energy.push_back(network.energyEstimate());
            out.bridgeStore.push_back(store);
            out.stringsOnly.push_back(network.energyEstimate() - store);
            out.peakBridgeStore = std::max(out.peakBridgeStore, store);
        }
    }
    out.unbridgedTicks = network.unbridgedTicks();
    return out;
}

struct Growth {
    double worst = 0.0;
    std::size_t worstAt = 0;
    double overshoot = 0.0; // above the post-excitation maximum, denominator FROZEN at energy[0]
};

Growth measureGrowth(const std::vector<double>& energy) {
    Growth out;
    if (energy.size() < 2)
        return out;
    const double reference = energy.front();
    for (std::size_t k = 1; k < energy.size(); ++k) {
        const double previous = energy[k - 1];
        const double growth = (previous > 0.0) ? (energy[k] / previous - 1.0) : 0.0;
        if (growth > out.worst) {
            out.worst = growth;
            out.worstAt = k;
        }
        if (reference > 0.0)
            out.overshoot = std::max(out.overshoot, energy[k] / reference - 1.0);
    }
    return out;
}

} // namespace

TEST_CASE("ENERGY/T2: a lossless coupled network only dissipates", "[energy]") {
    // THE tier-2 case docs/plan.md section 4.2 and Task P2.4's acceptance name: 6 strings, all
    // intentional losses off, the DOUBLE instantiation, static positions, strict per-block
    // non-increase within 1e-9.
    double worstOverall = 0.0;
    const char* worstWhere = "";

    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : kKinds) {
            Tier2Spec spec;
            spec.sampleRate = sampleRate;
            spec.kind = kind;
            const Tier2Run run = runLosslessNetwork(spec);

            REQUIRE(run.energy.size() > 2);
            REQUIRE(run.energy.front() > 0.0);  // non-vacuous: there is energy to conserve
            REQUIRE(run.unbridgedTicks == 0);   // ...and it really went through the junction
            REQUIRE(run.peakBridgeStore > 0.0); // ...and the junction really stored some of it

            const Growth growth = measureGrowth(run.energy);
            std::cout << "[energy] T2 coupled lossless " << kindName(kind) << " @ " << sampleRate
                      << " Hz: worst per-block growth " << growth.worst << " at block " << growth.worstAt << " (limit "
                      << kTolerance << "), cumulative overshoot " << growth.overshoot << ", energy "
                      << run.energy.front() << " -> " << run.energy.back() << ", peak bridge store "
                      << run.peakBridgeStore << " (" << (100.0 * run.peakBridgeStore / run.energy.front())
                      << "% of the total)\n";

            INFO(kindName(kind) << " @ " << sampleRate << ": worst growth " << growth.worst << " at block "
                                << growth.worstAt);
            REQUIRE(growth.worst <= kTolerance);
            REQUIRE(growth.overshoot <= kTolerance);
            if (growth.worst > worstOverall) {
                worstOverall = growth.worst;
                worstWhere = kindName(kind);
            }
        }
    }
    std::cout << "[energy] T2 worst per-block growth over all rates and interpolators: " << worstOverall << " ("
              << worstWhere << ")\n";
}

TEST_CASE("ENERGY/T2: a lossless coupled network only dissipates across the admittance range", "[energy]") {
    // The parameter axis the single scenario above does not sweep. A junction that was passive at
    // one operating point and active at another would be exactly the kind of defect the P2.5
    // fallback protocol exists for, and it would hide behind a single default.
    struct Point {
        float coupling;
        float resonanceHz;
        float damping;
    };
    const Point points[] = {
        {1.0f, 180.0f, 0.5f},  {1.0f, 80.0f, 0.01f}, {1.0f, 8000.0f, 0.01f}, {1.0f, 400.0f, 10.0f},
        {0.5f, 2000.0f, 0.1f}, {0.1f, 180.0f, 1.0f}, {0.35f, 60.0f, 0.05f},  {1.0f, 20000.0f, 0.5f},
    };

    double worst = 0.0;
    for (const Point& point : points) {
        for (double sampleRate : {44100.0, 96000.0}) {
            Tier2Spec spec;
            spec.sampleRate = sampleRate;
            spec.seconds = 3.0;
            spec.coupling = point.coupling;
            spec.resonanceHz = point.resonanceHz;
            spec.damping = point.damping;
            const Tier2Run run = runLosslessNetwork(spec);
            REQUIRE(run.energy.size() > 2);
            REQUIRE(run.energy.front() > 0.0);
            REQUIRE(run.peakBridgeStore > 0.0);
            const Growth growth = measureGrowth(run.energy);
            INFO("coupling " << point.coupling << " resonance " << point.resonanceHz << " damping " << point.damping
                             << " @ " << sampleRate << ": worst growth " << growth.worst);
            REQUIRE(growth.worst <= kTolerance);
            REQUIRE(growth.overshoot <= kTolerance);
            worst = std::max(worst, growth.worst);
        }
    }
    std::cout << "[energy] T2 admittance sweep (8 points x 2 rates): worst per-block growth " << worst << " (limit "
              << kTolerance << ")\n";
}

TEST_CASE("ENERGY/T2: the coupled lossless network is not the decoupled one", "[energy]") {
    // NON-VACUITY, and the difference from the damper's tier-2 scenario. Under lossless mode a
    // DamperJunction becomes a bit-exact pass-through, so the P2.3 damper-sweep case was measured
    // to be bit-identical to a frozen run -- it gated the crossfade not at all. The bridge's
    // lossless mode removes only the dashpot, so the coupling survives it. Measured, not asserted:
    // the audio must differ from a couplingStrength-0 run, and the junction must hold energy.
    Tier2Spec coupled;
    coupled.seconds = 2.0;
    Tier2Spec decoupled = coupled;
    decoupled.coupling = 0.0f;

    const Tier2Run withBridge = runLosslessNetwork(coupled);
    const Tier2Run without = runLosslessNetwork(decoupled);
    REQUIRE(withBridge.audio.size() == without.audio.size());

    std::size_t firstDifference = without.audio.size();
    double worstDifference = 0.0;
    for (std::size_t n = 0; n < without.audio.size(); ++n) {
        const double diff = std::fabs(withBridge.audio[n] - without.audio[n]);
        if (diff != 0.0 && firstDifference == without.audio.size())
            firstDifference = n;
        worstDifference = std::max(worstDifference, diff);
    }

    std::cout << "[energy] T2 non-vacuity: lossless coupled vs lossless decoupled -- first differing sample "
              << firstDifference << ", worst |difference| " << worstDifference << "; peak bridge store "
              << withBridge.peakBridgeStore << " coupled vs " << without.peakBridgeStore << " decoupled\n";

    REQUIRE(worstDifference > 0.0);
    REQUIRE(withBridge.peakBridgeStore > 0.0);
    // The decoupled run's junction is the Y == 0 limit: it cannot store anything, ever.
    REQUIRE(without.peakBridgeStore == 0.0);
}

TEST_CASE("ENERGY/T2: leaving the junction's store out of the functional would fail the bound", "[energy]") {
    // WHAT THE STORAGE TERM IS WORTH, as a number. carry-forward E: "energyEstimate() is a Lyapunov
    // storage functional, not rail energy. It must include the closed-form quadratic storage of
    // EVERY state-bearing element ... and the new bridge admittance biquad states. Rail-only energy
    // was a reviewer-caught blocker at P1.4 -- do not regress it."
    //
    // This case measures the regression rather than trusting the comment: the same run is scored
    // twice, once with the junction's store in the functional and once with it left out, and the
    // second must VIOLATE the tier-2 bound. If it ever stops violating it, the term has become
    // decorative and this case says so.
    Tier2Spec spec;
    spec.seconds = 3.0;
    spec.coupling = 1.0f; // the strongest coupling, i.e. the most energy in the junction at once
    const Tier2Run run = runLosslessNetwork(spec);
    REQUIRE(run.energy.size() > 2);

    const Growth withStore = measureGrowth(run.energy);
    const Growth withoutStore = measureGrowth(run.stringsOnly);

    std::cout << "[energy] T2 storage-term worth: WITH the junction's store, worst per-block growth " << withStore.worst
              << "; WITHOUT it, " << withoutStore.worst << " (limit " << kTolerance << ") -- "
              << (withoutStore.worst / std::max(kTolerance, 1.0e-300)) << "x the bound\n";

    REQUIRE(withStore.worst <= kTolerance);
    REQUIRE(withoutStore.worst > kTolerance);
}

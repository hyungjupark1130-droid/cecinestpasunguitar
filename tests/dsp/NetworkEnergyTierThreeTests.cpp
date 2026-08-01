#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// NetworkEnergyTierThreeTests -- Task P2.4's tier-3 [energy] gate: with the intentional losses back
// ON and on the shipping float32 path, do they only ever REMOVE energy?
//
// docs/plan.md section 4.2 defines the tier; a tier-3 failure with tiers 1 and 2 green points at a
// loss or dispersion filter whose magnitude response exceeds unity somewhere, never at the junction
// algebra and never at the assembly.
//
// ---------------------------------------------------------------------------------------------
// THE TWO ITEMS TASK P2.2 DEFERRED HERE, AND WHAT THEY WERE DECIDED AS (carry-forward B7)
// ---------------------------------------------------------------------------------------------
// 1. TIER-3's REDUCED SCOPE. P2.2 could only run tier 3 against six UNCOUPLED strings, because the
//    bridge carried no load until this task, and it recorded that as a deliberate reduction. The
//    scope is now the full one section 4.2 describes: the coupled network, at an admittance grid,
//    at all three rates, over a 10 s decay, on the float32 instantiation. The P2.2 case
//    (tests/dsp/DamperEnergyTests.cpp, "ENERGY/T3: lossy network only dissipates with dampers
//    engaged") is deliberately LEFT WHERE IT IS rather than folded in: it is the damper's gate, it
//    still passes, and moving it would lose the record of which task owns which claim.
//
// 2. THE ENGAGED RUN STOPS MEASURING AT ZERO. With every damper engaged the network reaches
//    EXACTLY zero energy partway through the render -- the silence watchdog clears each string once
//    its bridge-outgoing wave has stayed under -100 dBFS for a whole window -- and from there the
//    monotonicity assertion is comparing 0 with 0 and gates nothing.
//    DECISION: that is ACCEPTABLE, and it is made non-vacuous rather than papered over. Reaching
//    zero is the correct behaviour and the thing a damper is FOR; what would not be acceptable is a
//    case that reached zero in its second block and then reported a green 10-second gate. So the
//    engaged run below asserts a MINIMUM number of blocks with strictly positive energy before the
//    network is allowed to be silent, and prints the block at which it got there. The alternative
//    considered and rejected was disabling the watchdog for the test: it would gate a topology that
//    does not ship.
//
// ---------------------------------------------------------------------------------------------
// THE FLOAT32 MEASUREMENT FLOOR (Task P2.4 finding -- read this before touching kTolerance)
// ---------------------------------------------------------------------------------------------
// Bidirectional coupling gives the decay a tail the uncoupled network never had. Once the strings
// are damped, the bridge resonator keeps re-driving them out of its own residual, so instead of
// being cleared at the silence watchdog's floor the state keeps shrinking for thousands of blocks.
// Run far enough down, the float32 path stops being able to represent its own recursions, and
// energyEstimate() -- a sum of squares of that state -- stops being a measurement of anything.
//
// TWO SEPARATE ARITHMETIC FLOORS, and they are at different depths:
//
//   (a) WITHOUT the FTZ/DAZ guard the state itself goes SUBNORMAL (around 1e-44, quantized to
//       multiples of 1.4e-45) and the functional wanders by tens of per cent at total energies of
//       ~1e-87. That configuration ships nowhere: ScopedFtzDazGuard has been the first thing
//       PluginProcessor::processBlock constructs since Task P1.1, so every render here engages it
//       -- not as a convenience, but because tier 3 claims to measure the SHIPPING float32 path.
//   (b) WITH the guard, the floor moves but does not vanish, and the reason is worth stating
//       exactly because it is not "rounding". The recursions multiply the state by coefficients as
//       small as ~1e-2 (the loss filter's b1, the dispersion allpass coefficient, a crossfade
//       weight). Under FTZ a product that lands below the smallest NORMAL float, 1.18e-38, is
//       flushed to zero -- which does not merely round the state, it CHANGES THE RECURSION: an
//       allpass whose a*x term has vanished is no longer an allpass, and the closed-form storage
//       derived for it is no longer its storage. With ~10^3 stored values per network that starts
//       around |x| ~ 1e-36, i.e. a total energy of ~1e-69.
//
// So the monotonicity assertion is gated down to kMeasurementFloor and REPORTED below it. That is
// a statement about the dynamic range over which float32 can be measured, not a tolerance: the
// floor sits 29 decades above where (b) begins and roughly 390 dB below the start of every run
// here, and the cases print the level at which the last gated block sat so the margin is visible
// rather than asserted.
//
// THE DIAGNOSIS IS TESTED, NOT ASSUMED. "ENERGY/T3: the float32 arithmetic floor is a measurement
// limit, not a passivity failure" below runs the identical scenario on the `double` instantiation,
// where the same decay has another 200 decades of headroom, and requires it to be clean over the
// whole run. If the junction were active, that is where it would show.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr int kBlock = 128;
constexpr int kStrings = 6;
constexpr int kChord[kStrings] = {40, 45, 50, 55, 59, 64};

// docs/plan.md section 4.2 tier 3: "no block may exceed its predecessor by more than a 1e-6
// relative tolerance (absorbing float32 state rounding)".
constexpr double kTolerance = 1.0e-6;

// The level below which float32 stops being able to measure its own storage functional -- see the
// header. Derived (the FTZ flush boundary reached through the smallest coefficients in the
// recursions puts the phenomenon at ~1e-69) with 29 decades of margin, not fitted to a failure.
constexpr double kMeasurementFloor = 1.0e-40;

NoteEvent noteEvent(NoteEventType type, int sampleOffset, int midiNote, int stringIndex) {
    NoteEvent event{};
    event.type = type;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = 0.8f;
    event.pluckPosition = (type == NoteEventType::NoteOn) ? 0.28f : cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = (type == NoteEventType::NoteOn) ? 0.5f : cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

struct Tier3Run {
    std::vector<double> energy;
    std::vector<float> bridge;
    double worstGrowth = 0.0; // over blocks ABOVE kMeasurementFloor -- the gated quantity
    std::size_t worstAt = 0;
    double worstGrowthBelowFloor = 0.0; // reported, never gated; see the header
    double lowestGatedEnergy = 0.0;     // the level the assertion still bound at
    std::size_t gatedBlocks = 0;
    std::size_t positiveBlocks = 0;
    std::size_t firstZeroBlock = 0; // == energy.size() when it never reached zero
    unsigned long long unbridgedTicks = 0;
};

struct Tier3Spec {
    double sampleRate = 48000.0;
    double seconds = 10.0;
    float coupling = 0.35f;
    float resonanceHz = 180.0f;
    float damping = 0.5f;
    bool engageDampers = false;
    FractionalDelayKind kind = FractionalDelayKind::Lagrange3;
};

Tier3Run runLossyNetwork(const Tier3Spec& spec) {
    StringNetworkParams params;
    params.pickupPosition01 = 0.87f;
    params.bridge.couplingStrength = spec.coupling;
    params.bridge.resonanceHz = spec.resonanceHz;
    params.bridge.damping = spec.damping;

    StringNetwork<float> network; // the SHIPPING float32 instantiation, per section 4.2 tier 3
    network.prepare(spec.sampleRate, kBlock, spec.kind);
    network.setNumStrings(kStrings);
    network.setParams(params);
    network.reset();

    BlockEventQueue events;
    for (int s = 0; s < kStrings; ++s)
        events.push(noteEvent(NoteEventType::NoteOn, s * 17, kChord[s], s));

    const auto totalBlocks = static_cast<int>(spec.seconds * spec.sampleRate / kBlock);
    const int excitationBlocks = std::max(8, static_cast<int>(0.05 * spec.sampleRate / kBlock));

    Tier3Run out;
    out.energy.reserve(static_cast<std::size_t>(totalBlocks));
    // THE SHIPPING CONFIGURATION, not a convenience -- see the note at the top of this file.
    // PluginProcessor::processBlock constructs one of these first, so a float32 render without it
    // is not the path tier 3 claims to measure.
    const cnpg::dsp::ScopedFtzDazGuard denormalGuard;
    for (int b = 0; b < totalBlocks; ++b) {
        if (spec.engageDampers && b == excitationBlocks) {
            BlockEventQueue offs;
            for (int s = 0; s < kStrings; ++s)
                offs.push(noteEvent(NoteEventType::NoteOff, 0, kChord[s], s));
            network.process(offs, kBlock);
        } else {
            network.process(events, kBlock);
        }
        for (int n = 0; n < kBlock; ++n)
            out.bridge.push_back(network.bridgeOutputBuffer()[n]);
        if (b >= excitationBlocks)
            out.energy.push_back(network.energyEstimate());
    }
    out.unbridgedTicks = network.unbridgedTicks();

    out.firstZeroBlock = out.energy.size();
    for (std::size_t k = 0; k < out.energy.size(); ++k) {
        if (out.energy[k] > 0.0)
            ++out.positiveBlocks;
        else if (out.firstZeroBlock == out.energy.size())
            out.firstZeroBlock = k;
    }
    out.lowestGatedEnergy = out.energy.empty() ? 0.0 : out.energy.front();
    for (std::size_t k = 1; k < out.energy.size(); ++k) {
        const double previous = out.energy[k - 1];
        // 0 -> 0 is not growth; guarding it is what lets the engaged run keep measuring past the
        // point where the dampers have done their job (see the DECISION note at the top).
        if (!(previous > 0.0))
            continue;
        const double growth = out.energy[k] / previous - 1.0;
        if (previous >= kMeasurementFloor) {
            ++out.gatedBlocks;
            out.lowestGatedEnergy = std::min(out.lowestGatedEnergy, previous);
            if (growth > out.worstGrowth) {
                out.worstGrowth = growth;
                out.worstAt = k;
            }
        } else {
            out.worstGrowthBelowFloor = std::max(out.worstGrowthBelowFloor, growth);
        }
    }
    return out;
}

// Schroeder backward integration, in dB, then a least-squares slope over the -5 .. -35 dB span.
// Section 4.2's companion check: "a companion check on bridgeOutputBuffer() instead fits a decay
// from Schroeder backward integration (or a windowed-max envelope over at least one beat period)
// and requires the fitted decay slope to be negative after excitation".
double schroederSlopeDbPerSecond(const std::vector<float>& signal, double sampleRate, double& spanDbOut) {
    spanDbOut = 0.0;
    if (signal.size() < 64)
        return 0.0;
    std::vector<double> backward(signal.size() + 1, 0.0);
    for (std::size_t k = signal.size(); k-- > 0;) {
        const double x = static_cast<double>(signal[k]);
        backward[k] = backward[k + 1] + x * x;
    }
    if (!(backward.front() > 0.0))
        return 0.0;
    const double norm = backward.front();

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    std::size_t points = 0;
    double lowestDb = 0.0;
    for (std::size_t k = 0; k + 1 < backward.size(); k += 64) {
        const double value = backward[k] / norm;
        if (!(value > 0.0))
            break;
        const double db = 10.0 * std::log10(value);
        if (db > -5.0)
            continue;
        if (db < -35.0)
            break;
        const double t = static_cast<double>(k) / sampleRate;
        sumX += t;
        sumY += db;
        sumXX += t * t;
        sumXY += t * db;
        lowestDb = db;
        ++points;
    }
    if (points < 8)
        return 0.0;
    spanDbOut = -5.0 - lowestDb;
    const double n = static_cast<double>(points);
    const double denom = n * sumXX - sumX * sumX;
    return (denom != 0.0) ? (n * sumXY - sumX * sumY) / denom : 0.0;
}

} // namespace

TEST_CASE("ENERGY/T3: a lossy coupled network only dissipates over a 10 s decay", "[energy]") {
    // Task P2.4 acceptance: "losses enabled, float32: monotone-decreasing block-RMS envelope of
    // energyEstimate() over a 10 s decay, no growth at any tested admittance setting."
    struct Point {
        float coupling;
        float resonanceHz;
        float damping;
    };
    const Point points[] = {
        {0.35f, 180.0f, 0.5f}, {1.0f, 180.0f, 0.5f},   {1.0f, 80.0f, 0.01f}, {1.0f, 8000.0f, 0.05f},
        {0.5f, 400.0f, 10.0f}, {0.1f, 2000.0f, 0.25f}, {0.0f, 180.0f, 0.5f},
    };

    double worstOverall = 0.0;
    for (const Point& point : points) {
        Tier3Spec spec;
        spec.coupling = point.coupling;
        spec.resonanceHz = point.resonanceHz;
        spec.damping = point.damping;
        const Tier3Run run = runLossyNetwork(spec);

        REQUIRE(run.energy.size() > 100);
        REQUIRE(run.energy.front() > 0.0); // non-vacuous: there really was energy to dissipate
        REQUIRE(run.unbridgedTicks == 0);

        std::cout << "[energy] T3 coupling " << point.coupling << " resonance " << point.resonanceHz << " Hz damping "
                  << point.damping << ": worst per-block growth " << run.worstGrowth << " at block " << run.worstAt
                  << " (limit " << kTolerance << "), energy " << run.energy.front() << " -> " << run.energy.back()
                  << " over " << run.energy.size() << " blocks; " << run.gatedBlocks << " blocks gated down to "
                  << run.lowestGatedEnergy << " ("
                  << (10.0 * std::log10(std::max(run.lowestGatedEnergy, 1e-300) / run.energy.front()))
                  << " dB below the start), worst growth below the floor " << run.worstGrowthBelowFloor << "\n";

        INFO("coupling " << point.coupling << " resonance " << point.resonanceHz << " damping " << point.damping
                         << ": worst growth " << run.worstGrowth << " at block " << run.worstAt);
        REQUIRE(run.worstGrowth <= kTolerance);
        // A free-ringing string has not decayed to nothing in 10 s at the default material, so the
        // run is measuring a live decay for its whole length rather than a floor.
        REQUIRE(run.energy.back() > 0.0);
        worstOverall = std::max(worstOverall, run.worstGrowth);
    }
    std::cout << "[energy] T3 worst per-block growth over the admittance grid: " << worstOverall << "\n";
}

TEST_CASE("ENERGY/T3: a lossy coupled network only dissipates at every sample rate", "[energy]") {
    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
            Tier3Spec spec;
            spec.sampleRate = sampleRate;
            spec.kind = kind;
            spec.seconds = 5.0;
            const Tier3Run run = runLossyNetwork(spec);
            REQUIRE(run.energy.size() > 100);
            REQUIRE(run.energy.front() > 0.0);
            INFO("rate " << sampleRate << " kind " << (kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1")
                         << ": worst growth " << run.worstGrowth);
            REQUIRE(run.worstGrowth <= kTolerance);
            std::cout << "[energy] T3 " << (kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1") << " @ "
                      << sampleRate << " Hz: worst per-block growth " << run.worstGrowth << "\n";
        }
    }
}

TEST_CASE("ENERGY/T3: dampers engaged accelerate the decay and it stays monotone", "[energy]") {
    // The engaged repeat section 4.2 asks for, plus the DECISION recorded at the top of this file
    // about what "stops measuring at zero" is allowed to mean.
    Tier3Spec spec;
    spec.engageDampers = true;
    spec.seconds = 10.0;
    const Tier3Run engaged = runLossyNetwork(spec);

    Tier3Spec idleSpec;
    idleSpec.seconds = 10.0;
    const Tier3Run idle = runLossyNetwork(idleSpec);

    const double engagedSeconds = static_cast<double>(engaged.firstZeroBlock * kBlock) / spec.sampleRate;
    std::cout << "[energy] T3 dampers engaged: worst per-block growth " << engaged.worstGrowth << " (limit "
              << kTolerance << ") over " << engaged.gatedBlocks << " gated blocks down to " << engaged.lowestGatedEnergy
              << "; worst growth below the measurement floor " << engaged.worstGrowthBelowFloor
              << " (reported, not gated -- see this file's header); energy reached exactly 0 at block "
              << engaged.firstZeroBlock << " of " << engaged.energy.size() << " (" << engagedSeconds
              << " s after sampling began), " << engaged.positiveBlocks
              << " blocks measured a live decay. Idle run: " << idle.positiveBlocks << " of " << idle.energy.size()
              << " blocks positive, final energy " << idle.energy.back() << "\n";

    REQUIRE(engaged.worstGrowth <= kTolerance);
    REQUIRE(engaged.energy.front() > 0.0);
    // THE NON-VACUITY GUARD for "stops measuring at zero": at least a second of real decay must
    // have been gated before the network is allowed to be silent. Without this the case would pass
    // on an implementation that silenced everything in block 1.
    REQUIRE(engaged.positiveBlocks > static_cast<std::size_t>(1.0 * spec.sampleRate / kBlock));
    // ...and the dampers really did their job, which is the other half of the claim.
    REQUIRE(engaged.energy.back() == 0.0);
    REQUIRE(idle.energy.back() > 0.0);
    REQUIRE(engaged.firstZeroBlock < idle.energy.size());
}

TEST_CASE("ENERGY/T3: the bridge output's Schroeder slope is negative after excitation", "[energy]") {
    // THE COMPANION CHECK Task P2.2 deferred here (carry-forward B7). Section 4.2 is explicit about
    // why it cannot simply be block-RMS monotonicity on the bridge signal: "coupled strings beat,
    // and beating is periodic envelope growth". The bridge output of six strings is exactly that
    // signal, so its envelope legitimately rises and falls; what must fall is the INTEGRATED decay.
    //
    // Reported for every admittance point, so the tier's headline claim (energyEstimate() never
    // grows) is paired with a measurement on the artifact a listener actually hears.
    struct Point {
        float coupling;
        float resonanceHz;
        float damping;
    };
    const Point points[] = {
        {0.35f, 180.0f, 0.5f},
        {1.0f, 180.0f, 0.5f},
        {1.0f, 80.0f, 0.01f},
        {0.1f, 2000.0f, 0.25f},
    };

    for (const Point& point : points) {
        Tier3Spec spec;
        spec.coupling = point.coupling;
        spec.resonanceHz = point.resonanceHz;
        spec.damping = point.damping;
        spec.seconds = 6.0;
        const Tier3Run run = runLossyNetwork(spec);

        // Skip the excitation, so "after excitation" is what is being fitted.
        const auto skip = static_cast<std::size_t>(0.2 * spec.sampleRate);
        REQUIRE(run.bridge.size() > skip);
        std::vector<float> tail(run.bridge.begin() + static_cast<std::ptrdiff_t>(skip), run.bridge.end());

        double spanDb = 0.0;
        const double slope = schroederSlopeDbPerSecond(tail, spec.sampleRate, spanDb);
        float peak = 0.0f;
        for (float value : tail)
            peak = std::max(peak, std::fabs(value));

        std::cout << "[energy] T3 bridgeOutputBuffer Schroeder slope, coupling " << point.coupling << " resonance "
                  << point.resonanceHz << " Hz damping " << point.damping << ": " << slope << " dB/s over a " << spanDb
                  << " dB span; bridge-output peak " << peak << "\n";

        INFO("coupling " << point.coupling << " resonance " << point.resonanceHz << " damping " << point.damping);
        REQUIRE(peak > 0.0f);   // non-vacuous: there is a bridge signal to fit
        REQUIRE(spanDb > 10.0); // ...and it really decayed, rather than the fit running out of curve
        REQUIRE(slope < 0.0);
    }
}

TEST_CASE("ENERGY/T3: the float32 arithmetic floor is a measurement limit, not a passivity failure", "[energy]") {
    // THE EXPERIMENT BEHIND THE MEASUREMENT FLOOR at the top of this file, kept as a standing case
    // because the finding is exactly the kind that gets re-lost and then "fixed" by widening a
    // tolerance. Three renders of the same engaged scenario:
    //
    //   (a) float32 WITHOUT the guard -- the configuration that ships nowhere. Its state decays
    //       into the subnormal range (~1e-44, quantized to multiples of 1.4e-45) and the storage
    //       functional wanders by tens of per cent at total energies around 1e-87.
    //   (b) DOUBLE, same scenario, same code path. If the junction were active this is where it
    //       would show: double reaches the same levels with ~200 decades of headroom left, so it is
    //       measuring physics where float32 is measuring its own floor. It must be clean over the
    //       WHOLE run, floor or no floor, and that is what says (a) and (c) are arithmetic.
    //   (c) float32 WITH the guard -- what tier 3 actually gates, scored over its full range and
    //       again over the range above kMeasurementFloor.
    //
    // The case ASSERTS the diagnosis rather than describing it: (b) must be clean everywhere and
    // (a) must not be. If (a) ever became clean the case would fail, and the right response would
    // be to delete it, not to keep it.
    constexpr int kBlocks = 1400;
    constexpr int kExcitation = 60;

    auto run = [](auto sampleTag, bool guard) {
        using SampleT = decltype(sampleTag);
        StringNetworkParams params;
        params.pickupPosition01 = 0.87f;
        StringNetwork<SampleT> network;
        network.prepare(48000.0, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(kStrings);
        network.setParams(params);
        network.reset();

        BlockEventQueue events;
        for (int s = 0; s < kStrings; ++s)
            events.push(noteEvent(NoteEventType::NoteOn, s * 17, kChord[s], s));

        std::vector<double> energy;
        {
            // Constructed conditionally, restored on scope exit either way. MXCSR is per thread and
            // Catch2's runner is single-threaded, but leaving it modified is exactly the leak
            // Task P2.3 root-caused, so it is scoped rather than set.
            const cnpg::dsp::ScopedFtzDazGuard scoped;
            if (!guard) {
                // Clear the bits the guard just set; `scoped` still restores the ENTRY MXCSR on the
                // way out, which is the invariant Task P2.3 root-caused and pinned.
                _mm_setcsr(_mm_getcsr() & ~cnpg::dsp::ScopedFtzDazGuard::kFtzDazMask);
            }
            for (int b = 0; b < kBlocks; ++b) {
                if (b == kExcitation) {
                    BlockEventQueue offs;
                    for (int s = 0; s < kStrings; ++s)
                        offs.push(noteEvent(NoteEventType::NoteOff, 0, kChord[s], s));
                    network.process(offs, kBlock);
                } else {
                    network.process(events, kBlock);
                }
                if (b >= kExcitation)
                    energy.push_back(network.energyEstimate());
            }
        }

        struct Score {
            double worstAnywhere = 0.0;
            double worstAboveFloor = 0.0;
            double smallestPositive = 1.0e300;
            double energyAtWorst = 0.0;
        };
        Score score;
        for (std::size_t k = 1; k < energy.size(); ++k) {
            if (!(energy[k - 1] > 0.0))
                continue;
            const double growth = energy[k] / energy[k - 1] - 1.0;
            if (growth > score.worstAnywhere) {
                score.worstAnywhere = growth;
                score.energyAtWorst = energy[k - 1];
            }
            if (energy[k - 1] >= kMeasurementFloor)
                score.worstAboveFloor = std::max(score.worstAboveFloor, growth);
            score.smallestPositive = std::min(score.smallestPositive, energy[k - 1]);
        }
        return score;
    };

    const auto unguardedFloat = run(float{}, false);
    const auto plainDouble = run(double{}, false);
    const auto guardedFloat = run(float{}, true);

    std::cout << "[energy] T3 arithmetic-floor diagnosis (engaged 6-string decay, identical scenario):\n"
              << "  float32, NO FTZ/DAZ (ships nowhere): worst growth anywhere " << unguardedFloat.worstAnywhere
              << " at energy " << unguardedFloat.energyAtWorst << "; above the floor " << unguardedFloat.worstAboveFloor
              << "; smallest energy reached " << unguardedFloat.smallestPositive << "\n"
              << "  double   (THE CONTROL)             : worst growth anywhere " << plainDouble.worstAnywhere
              << " at energy " << plainDouble.energyAtWorst << "; smallest energy reached "
              << plainDouble.smallestPositive << "\n"
              << "  float32, WITH FTZ/DAZ (shipping)   : worst growth anywhere " << guardedFloat.worstAnywhere
              << " at energy " << guardedFloat.energyAtWorst << "; above the floor " << guardedFloat.worstAboveFloor
              << "; smallest energy reached " << guardedFloat.smallestPositive << " (tier-3 limit " << kTolerance
              << ", floor " << kMeasurementFloor << ")\n";

    // THE CONTROL. Clean over its WHOLE run, with no floor of any kind: the junction is passive and
    // the assembly around it does not pump. Everything else in this case is about arithmetic.
    REQUIRE(plainDouble.worstAnywhere <= kTolerance);
    // ...and it really did follow the decay far past where float32 gave up, so the control is not
    // clean merely because it stopped early.
    REQUIRE(plainDouble.smallestPositive < 1.0e-60);

    // Both float32 runs are dirty SOMEWHERE, which is what makes this a diagnosis rather than a
    // preference -- and what would make a future change that removed the floor visible here.
    REQUIRE(unguardedFloat.worstAnywhere > kTolerance);
    REQUIRE(guardedFloat.worstAnywhere > kTolerance);
    // The unguarded run's floor is far deeper (subnormal state) than the guarded run's (flushed
    // intermediate products), which is the whole reason the shipping guard is engaged.
    REQUIRE(unguardedFloat.smallestPositive < guardedFloat.smallestPositive);
    REQUIRE(guardedFloat.energyAtWorst < kMeasurementFloor);
    REQUIRE(unguardedFloat.energyAtWorst < kMeasurementFloor);
    // ...and above the stated floor the shipping configuration meets the plan's bound.
    REQUIRE(guardedFloat.worstAboveFloor <= kTolerance);
}

// ---------------------------------------------------------------------------------------------
// carry-forward B6: lossStorageWeight validity, which tier 3 leans on
// ---------------------------------------------------------------------------------------------

TEST_CASE("ENERGY/T3: the loop-loss filter's storage weight really is a storage function", "[energy]") {
    // carry-forward B6, a P1.4 deferral: "lossStorageWeight validity is unchecked and tier-3 leans
    // on it. Validate it or state explicitly why it cannot be invalid."
    //
    // IT CAN BE INVALID, in principle. The weight p is the vertex of the parabola
    // f(p) = -B^2 p^2 + K p - 1, which has a non-negative branch only when K^2 >= 4 B^2; if that
    // branch is empty, the vertex is still returned and is NOT a storage function, so
    // energyEstimate() stops being a Lyapunov function and tier 3 gates nothing. The closed form is
    // in dsp/src/WaveguideString.cpp; re-typing it here would prove nothing, so this case checks
    // the DISSIPATION INEQUALITY the weight is supposed to satisfy, over the whole shipping
    // parameter range, for the state-space (A, B, C, D) = (a1, a1 b0 + b1, 1, b0) it was derived
    // for. The coefficients come out of the shipped string via lossStorageProbe(), so this is a
    // check of what the audio path is actually running.
    //
    // EXACTLY, NOT BY SAMPLING (P2.4 review, M1). The inequality
    //
    //     p (a1 s + B x)^2 - p s^2  <=  x^2 - (s + b0 x)^2 ,   B = a1 b0 + b1
    //
    // is a homogeneous quadratic form in (s, x), so "holds for all (s, x)" is exactly "the 2x2
    // matrix is positive semidefinite" -- two scalar conditions, not a vote among probe directions.
    // The first draft sampled 24 directions, which can only ever say "it did not fail HERE": a form
    // that is indefinite along a direction between two probes passes it. This is the ONLY gate
    // covering lossStorage_, so it is worth the three lines. Written out, with
    // Q = [[q_ss, q_sx], [q_sx, q_xx]] the matrix of (supply - dissipation):
    //
    //     q_ss = p (1 - a1^2) - 1
    //     q_sx = -(p a1 B + b0)
    //     q_xx = 1 - b0^2 - p B^2
    //
    // and PSD <=> q_ss >= 0 and q_xx >= 0 and det Q = q_ss q_xx - q_sx^2 >= 0. Both the determinant
    // and the smaller eigenvalue are reported, because "the determinant is positive" alone would
    // also be satisfied by a NEGATIVE definite form.
    double smallestDeterminant = 1.0e300;
    double smallestEigenvalue = 1.0e300;
    double worstAt[4] = {0.0, 0.0, 0.0, 0.0};
    int points = 0;
    double smallestWeight = 1.0e300;
    double largestWeight = 0.0;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        for (int knobLow = 0; knobLow <= 10; ++knobLow) {
            for (int knobHigh = 0; knobHigh <= 10; ++knobHigh) {
                for (int midiNote : {21, 40, 69, 96, 108}) {
                    WaveguideString<double> string;
                    string.prepare(sampleRate, kBlock, FractionalDelayKind::Lagrange3);
                    WaveguideStringParams params;
                    params.f0Hz = static_cast<float>(440.0 * std::exp2((static_cast<double>(midiNote) - 69.0) / 12.0));
                    params.stringMaterial.lossGainLow = static_cast<float>(knobLow) / 10.0f;
                    params.stringMaterial.lossGainHigh = static_cast<float>(knobHigh) / 10.0f;
                    string.setParams(params);
                    string.reset();

                    const auto probe = string.lossStorageProbe();
                    REQUIRE(probe.bypassed == false);
                    REQUIRE(std::isfinite(probe.storageWeight));
                    // A storage function must be POSITIVE DEFINITE: V = p s^2 with p <= 0 is not a
                    // storage at all.
                    INFO("rate " << sampleRate << " lossLow " << knobLow << " lossHigh " << knobHigh << " note "
                                 << midiNote << ": b0 " << probe.b0 << " b1 " << probe.b1 << " a1 " << probe.a1 << " p "
                                 << probe.storageWeight);
                    REQUIRE(probe.storageWeight > 0.0);
                    smallestWeight = std::min(smallestWeight, probe.storageWeight);
                    largestWeight = std::max(largestWeight, probe.storageWeight);

                    const double bb = probe.a1 * probe.b0 + probe.b1;
                    const double p = probe.storageWeight;
                    const double qss = p * (1.0 - probe.a1 * probe.a1) - 1.0;
                    const double qsx = -(p * probe.a1 * bb + probe.b0);
                    const double qxx = 1.0 - probe.b0 * probe.b0 - p * bb * bb;
                    const double determinant = qss * qxx - qsx * qsx;
                    // Smaller eigenvalue of a symmetric 2x2, in closed form. Reported alongside the
                    // determinant because a positive determinant is also consistent with a NEGATIVE
                    // definite form, which would be the exact opposite of a storage function.
                    const double halfTrace = 0.5 * (qss + qxx);
                    const double radius = std::sqrt(std::max(0.0, halfTrace * halfTrace - determinant));
                    const double lambdaMin = halfTrace - radius;

                    INFO("rate " << sampleRate << " lossLow " << knobLow << " lossHigh " << knobHigh << " note "
                                 << midiNote << ": q = [[" << qss << ", " << qsx << "], [" << qsx << ", " << qxx
                                 << "]], det " << determinant << ", lambda_min " << lambdaMin);
                    // POSITIVE SEMIDEFINITE, exactly: both diagonal entries non-negative and the
                    // determinant non-negative. Equivalent to "the dissipation inequality holds for
                    // EVERY (s, x)", which is what a storage function has to mean.
                    REQUIRE(qss >= 0.0);
                    REQUIRE(qxx >= 0.0);
                    REQUIRE(determinant >= 0.0);
                    REQUIRE(lambdaMin >= 0.0);
                    if (determinant < smallestDeterminant) {
                        smallestDeterminant = determinant;
                        worstAt[0] = sampleRate;
                        worstAt[1] = static_cast<double>(knobLow);
                        worstAt[2] = static_cast<double>(knobHigh);
                        worstAt[3] = static_cast<double>(midiNote);
                    }
                    smallestEigenvalue = std::min(smallestEigenvalue, lambdaMin);
                    ++points;
                }
            }
        }
    }

    std::cout << "[energy] T3 lossStorageWeight validity (carry-forward B6): the supply-minus-dissipation form is "
              << "positive semidefinite -- i.e. the inequality holds for EVERY (s, x), not merely at sampled "
              << "directions -- at all " << points << " (rate x lossLow x lossHigh x note) points. Smallest "
              << "determinant " << smallestDeterminant << " at " << worstAt[0] << " Hz, knobs " << worstAt[1] << "/"
              << worstAt[2] << ", MIDI " << worstAt[3] << "; smallest eigenvalue " << smallestEigenvalue
              << "; p ranged " << smallestWeight << " .. " << largestWeight << "\n";

    // Non-vacuous: the probe must have exercised a range of filters, not one.
    REQUIRE(largestWeight > smallestWeight);
    // Strictly INSIDE the feasible region everywhere, not on its edge -- a form that were merely
    // semidefinite would make the storage marginal rather than strict.
    REQUIRE(smallestDeterminant > 0.0);
    REQUIRE(smallestEigenvalue > 0.0);
}

#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/IBridgePort.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"
#include "support/ClickMetric.h"
#include "support/SpectralAnalysis.h"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// BridgePortContractTests -- the INTERFACE-level suite any IBridgePort must pass (docs/plan.md Task
// P2.4 file list: "interface-level suite any IBridgePort must pass -- written now, exercised against
// BridgeJunction"). Task P2.5's SympatheticResonatorBus, if the passivity timebox ever fires, adds
// one line to kPortAdapters below and inherits every case here; that is the whole point of Q17's
// design-for-fallback.
//
// It also carries the two things carry-forward B3 and B4 asked this task to hand forward as DATA
// rather than as prose:
//
//   B4 -- JUNCTION LIVENESS. The bridge port fails OPEN: a port that is mis-wired or never ticked
//         leaves each string reflecting off its own internal rigid -1, so the instrument still
//         makes sound, just an uncoupled one. Nothing about that is observable from the output,
//         because the thing that is missing was never heard. The counters asserted below make it
//         loud.
//   B3 -- THE SEAM'S DELAY CONTRIBUTION, measured at all three rates, so Task P2.7 starts from a
//         number instead of rediscovering one.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::BridgeAdmittanceParams;
using cnpg::dsp::BridgeJunction;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RigidBridgeTermination;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr int kBlock = 128;
constexpr double kRate = 48000.0;
constexpr std::array<double, 3> kRates{44100.0, 48000.0, 96000.0};

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

// Per-implementation adapter, in the shape docs/plan.md section 4.1 describes for the shared
// contract harness: "a per-module adapter struct that knows how to construct, feed silence, and
// feed a canonical excitation".
struct JunctionAdapter {
    using Port = BridgeJunction<double>;
    static const char* name() { return "BridgeJunction<double>"; }
    // A loaded junction is not a pass-through, so a contract that expects one would be wrong; this
    // flag is what lets the shared cases state the difference instead of testing the weaker claim
    // for both.
    static constexpr bool storesEnergy = true;
};

struct RigidAdapter {
    using Port = RigidBridgeTermination<double>;
    static const char* name() { return "RigidBridgeTermination<double>"; }
    static constexpr bool storesEnergy = false;
};

template <typename Adapter> typename Adapter::Port makePort(double sampleRate, int ports, float coupling) {
    std::vector<float> impedances(static_cast<std::size_t>(ports), 1.0f);
    typename Adapter::Port port;
    port.prepare(sampleRate, kBlock, ports, impedances.data());
    BridgeAdmittanceParams admittance;
    admittance.couplingStrength = coupling;
    port.setAdmittance(admittance);
    port.reset();
    return port;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// the shared IBridgePort contract
// ---------------------------------------------------------------------------------------------

TEMPLATE_TEST_CASE("CONTRACT: IBridgePort silence in, silence out before excitation", "[contract]", JunctionAdapter,
                   RigidAdapter) {
    for (double sampleRate : kRates) {
        for (int ports = 1; ports <= cnpg::dsp::kMaxStrings; ++ports) {
            auto port = makePort<TestType>(sampleRate, ports, 0.5f);
            INFO(TestType::name() << " at " << sampleRate << " Hz, " << ports << " ports");
            REQUIRE(port.isQuiescent());
            REQUIRE(port.storageEnergy() == 0.0);

            std::array<double, cnpg::dsp::kMaxStrings> incident{};
            std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
            for (int n = 0; n < 4096; ++n) {
                for (double& value : outgoing)
                    value = 12345.0; // must be overwritten, not merely left alone
                port.scatter(incident.data(), outgoing.data(), ports);
                for (int p = 0; p < ports; ++p)
                    REQUIRE(outgoing[static_cast<std::size_t>(p)] == 0.0);
                REQUIRE(port.bridgeOutput() == 0.0);
            }
            REQUIRE(port.isQuiescent());
            REQUIRE(port.storageEnergy() == 0.0);
        }
    }
}

TEMPLATE_TEST_CASE("CONTRACT: IBridgePort reset is idempotent and complete", "[contract]", JunctionAdapter,
                   RigidAdapter) {
    // docs/plan.md section 4.1: "excite the module, process >= 1 s, call reset(), then process
    // silence. Output must be sample-identical to a fresh prepared instance ... Calling reset()
    // twice in a row must equal calling it once."
    constexpr int ports = 6;
    auto exercised = makePort<TestType>(kRate, ports, 0.7f);
    auto fresh = makePort<TestType>(kRate, ports, 0.7f);

    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
    for (int n = 0; n < 48000; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.017 * n + 0.4 * p);
        exercised.scatter(incident.data(), outgoing.data(), ports);
    }
    if (TestType::storesEnergy)
        REQUIRE(exercised.storageEnergy() > 0.0); // there really was state to clear

    exercised.reset();
    exercised.reset(); // twice equals once
    REQUIRE(exercised.isQuiescent());
    REQUIRE(exercised.storageEnergy() == 0.0);

    std::array<double, cnpg::dsp::kMaxStrings> outA{};
    std::array<double, cnpg::dsp::kMaxStrings> outB{};
    for (int n = 0; n < 8192; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::cos(0.003 * n - 0.11 * p);
        exercised.scatter(incident.data(), outA.data(), ports);
        fresh.scatter(incident.data(), outB.data(), ports);
        for (int p = 0; p < ports; ++p)
            REQUIRE(outA[static_cast<std::size_t>(p)] == outB[static_cast<std::size_t>(p)]);
        REQUIRE(exercised.bridgeOutput() == fresh.bridgeOutput());
    }
}

TEMPLATE_TEST_CASE("CONTRACT: IBridgePort prepare is re-entrant", "[contract]", JunctionAdapter, RigidAdapter) {
    // Same claim docs/plan.md section 4.1 makes for every module: a second prepare() at a different
    // rate must leave the port indistinguishable from a freshly constructed one prepared there.
    // Load COEFFICIENTS are sample-rate dependent (the bilinear discretization), so this is a real
    // question for BridgeJunction rather than a formality.
    constexpr int ports = 4;
    auto reprepared = makePort<TestType>(44100.0, 8, 0.4f);
    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> scratch{};
    for (int n = 0; n < 4096; ++n) {
        for (int p = 0; p < 8; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.021 * n + p);
        reprepared.scatter(incident.data(), scratch.data(), 8);
    }

    std::vector<float> impedances(static_cast<std::size_t>(ports), 1.0f);
    reprepared.prepare(96000.0, 64, ports, impedances.data());
    BridgeAdmittanceParams admittance;
    admittance.couplingStrength = 0.9f;
    reprepared.setAdmittance(admittance);
    reprepared.reset();

    auto virgin = makePort<TestType>(96000.0, ports, 0.9f);
    std::array<double, cnpg::dsp::kMaxStrings> outA{};
    std::array<double, cnpg::dsp::kMaxStrings> outB{};
    for (int n = 0; n < 16384; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.0071 * n) * (1.0 + 0.1 * p);
        reprepared.scatter(incident.data(), outA.data(), ports);
        virgin.scatter(incident.data(), outB.data(), ports);
        for (int p = 0; p < ports; ++p)
            REQUIRE(outA[static_cast<std::size_t>(p)] == outB[static_cast<std::size_t>(p)]);
    }
}

TEMPLATE_TEST_CASE("CONTRACT: IBridgePort at couplingStrength 0 is exactly the rigid termination", "[contract]",
                   JunctionAdapter, RigidAdapter) {
    // docs/plan.md section 2.6 gives couplingStrength == 0 a CONTRACT -- "0 = strings fully
    // decoupled AND bridgeOutput() == 0" -- and every later body/chamber feature depends on that
    // being reachable and on it NOT being the shipping default. Both halves asserted bit-exactly
    // against the trivial termination, which is the object the limit is supposed to be.
    constexpr int ports = 8;
    auto port = makePort<TestType>(kRate, ports, 0.0f);
    RigidBridgeTermination<double> rigid;
    rigid.prepare(kRate, kBlock, ports, nullptr);

    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> outA{};
    std::array<double, cnpg::dsp::kMaxStrings> outB{};
    for (int n = 0; n < 20000; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.013 * n + 0.7 * p);
        port.scatter(incident.data(), outA.data(), ports);
        rigid.scatter(incident.data(), outB.data(), ports);
        for (int p = 0; p < ports; ++p) {
            REQUIRE(outA[static_cast<std::size_t>(p)] == outB[static_cast<std::size_t>(p)]);
            REQUIRE(outA[static_cast<std::size_t>(p)] == -incident[static_cast<std::size_t>(p)]);
        }
        REQUIRE(port.bridgeOutput() == 0.0);
    }
    REQUIRE(port.storageEnergy() == 0.0);
}

TEST_CASE("CONTRACT: BridgeJunction reaches quiescence, and decoupling releases its store", "[contract]") {
    // THE DEFECT THIS GATES (found at the P2.4 review). isQuiescent() was an exact-zero test on
    // states that decay GEOMETRICALLY, so in a `double` it could only become true by underflow --
    // roughly 700 dB down, minutes of silence. Two consequences, both measured before the fix:
    //
    //   - StringNetwork's `bridgeMayDrive` never went false, so the idle-string skip never fired
    //     again after the first note and energyEstimate() sat at 1.16e-05 after 107 SECONDS of
    //     silence (a control run reached 4.29e-84);
    //   - and in the RIGID branch it was worse than slow, it was permanent: v is identically 0
    //     there, so the mass state merely alternates sign and the spring state never moves. Setting
    //     couplingStrength to 0 after playing froze 0.1864 of storage indefinitely, and turning the
    //     knob back up released it into the strings out of nothing.
    //
    // Both halves are asserted here, and the second one with the knob-turn that made it audible.
    constexpr int ports = 6;
    auto port = makePort<JunctionAdapter>(kRate, ports, 0.35f);

    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
    for (int n = 0; n < 4096; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.021 * n + 0.5 * p);
        port.scatter(incident.data(), outgoing.data(), ports);
    }
    const double driven = port.storageEnergy();
    REQUIRE(driven > 0.0);
    REQUIRE_FALSE(port.isQuiescent()); // in the state this case claims to test

    // (a) LOADED: silence must actually reach quiescence, in a time a player would call "the note
    // ended" rather than in a time nobody waits.
    incident.fill(0.0);
    int samplesToQuiescence = -1;
    for (int n = 0; n < static_cast<int>(20.0 * kRate); ++n) {
        port.scatter(incident.data(), outgoing.data(), ports);
        if (port.isQuiescent()) {
            samplesToQuiescence = n;
            break;
        }
    }
    REQUIRE(samplesToQuiescence >= 0);
    const double secondsToQuiescence = static_cast<double>(samplesToQuiescence) / kRate;
    REQUIRE(secondsToQuiescence < 5.0);

    // (b) RIGID: decoupling releases the store rather than freezing it, and the released junction
    // then produces NOTHING from zero incident -- which is what "fully decoupled" has to mean.
    auto frozen = makePort<JunctionAdapter>(kRate, ports, 0.35f);
    for (int n = 0; n < 4096; ++n) {
        for (int p = 0; p < ports; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.021 * n + 0.5 * p);
        frozen.scatter(incident.data(), outgoing.data(), ports);
    }
    const double beforeDecoupling = frozen.storageEnergy();
    REQUIRE(beforeDecoupling > 0.0);

    BridgeAdmittanceParams decoupled;
    decoupled.couplingStrength = 0.0f;
    frozen.setAdmittance(decoupled);
    incident.fill(0.0);
    // THE TRANSITION SAMPLE IS SEPARATED FROM THE REST DELIBERATELY, because it is a real
    // discontinuity and pretending otherwise would be the wrong kind of green.
    //
    // Discarding a non-zero store cannot be continuous -- the outgoing wave steps from whatever the
    // bridge was radiating to exactly zero. There is no version of this that is both "the knob
    // disconnects the body" and "nothing steps". (Nor is the alternative continuous: the true
    // mu -> 0 LIMIT is an infinitely heavy mass, whose state freezes rather than releasing, which is
    // the freeze this fix exists to remove. The rigid branch is a DECISION about what the knob
    // means, not a limit, and it is documented as one.)
    //
    // What is gated is that the step happens ONCE and is bounded by a measured constant. Note that
    // this branch IS user-reachable -- kBridgeMinMobilityRatio includes couplingStrength == 0, which
    // is the Bridge Coupling parameter's minimum -- so what the gesture costs through the network is
    // measured too, in "CONTRACT: dragging Bridge Coupling to zero is click-free through the
    // network" below. This case is the junction-level half.
    double transitionPeak = 0.0;
    frozen.scatter(incident.data(), outgoing.data(), ports);
    for (int p = 0; p < ports; ++p)
        transitionPeak = std::max(transitionPeak, std::fabs(outgoing[static_cast<std::size_t>(p)]));

    double peakFromNothing = 0.0;
    for (int n = 0; n < 200000; ++n) {
        frozen.scatter(incident.data(), outgoing.data(), ports);
        for (int p = 0; p < ports; ++p)
            peakFromNothing = std::max(peakFromNothing, std::fabs(outgoing[static_cast<std::size_t>(p)]));
    }

    std::cout << "[contract] bridge quiescence: driven store " << driven << " -> quiescent after "
              << secondsToQuiescence << " s of silence (threshold " << cnpg::dsp::kBridgeQuiescentEnergy
              << "); decoupling released " << beforeDecoupling << " of store in ONE sample of magnitude "
              << transitionPeak << ", after which 200 000 silent samples produced a peak outgoing wave of "
              << peakFromNothing << "\n";

    REQUIRE(frozen.storageEnergy() == 0.0);
    REQUIRE(frozen.isQuiescent());
    // Nothing from nothing, for ever, from the sample after the transition.
    REQUIRE(peakFromNothing == 0.0);
    // ...and the transition is bounded by a MEASURED constant, not by a theorem. `sqrt(2*storage)`
    // was tried first and is worthless as a gate: with unit port impedances it equals ||s||, and
    // |v| = 2|<(sqrt(Z_M),sqrt(Z_K)), s>| / sigma <= 2 sqrt(L) ||s|| / (sum Z_i + L) <= ||s||
    // whenever sum Z_i >= 1, by (sqrt(L) - 1)^2 >= 0. It holds identically, so it read 0.058 against
    // a bound of 4.77 -- 82x loose, and a change making the step 50x larger would still have passed.
    // (Found by the P2.4 re-review; the same class as the P2.1 ruling that an assertion which cannot
    // fail is not a test.) 0.08 is the measured 0.058078 with ~38% headroom, on a probe whose
    // incident waves are unit-amplitude sines -- an absolute number, so it moves if the step does.
    REQUIRE(transitionPeak <= 0.08);
    // Non-vacuous in the other direction: the transition really is a step, not nothing.
    REQUIRE(transitionPeak > 0.0);

    // ...and the same thing through the network, which is where it mattered: a network that has been
    // silent must report a storage functional heading for zero rather than parked above it.
    StringNetworkParams params;
    params.bridge.couplingStrength = 0.35f;
    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(6);
    network.setParams(params);
    network.reset();
    BlockEventQueue events;
    events.push(noteOn(0, 45, 0));
    for (int b = 0; b < 200; ++b)
        network.process(events, kBlock);
    REQUIRE(network.energyEstimate() > 0.0);
    BlockEventQueue silence;
    for (int b = 0; b < static_cast<int>(30.0 * kRate / kBlock); ++b)
        network.process(silence, kBlock);
    const double afterSilence = network.energyEstimate();
    std::cout << "[contract] network storage functional after 30 s of silence: " << afterSilence << "\n";
    REQUIRE(afterSilence < 1.0e-11);
}

TEST_CASE("CONTRACT: dragging Bridge Coupling to zero is click-free through the network", "[contract]") {
    // THE GESTURE A USER CAN ACTUALLY MAKE, gated (P2.4 re-review, N1/N2/N4). `bridgeCoupling` is an
    // APVTS parameter over the full 0..1 unit range, so its MINIMUM is exactly the value that enters
    // BridgeJunction's rigid branch and discards the junction's store. Earlier text in this file and
    // in BridgeJunction claimed that branch fired "at a value no listener and no test can reach";
    // that was false, and it is why the transition had never been measured through the network at
    // all. It is reachable by dragging one slider to its stop while the instrument rings.
    //
    // ---------------------------------------------------------------------------------------------
    // THE DECOMPOSITION, which is the whole point of this case
    // ---------------------------------------------------------------------------------------------
    // Decoupling the bridge is a LEGITIMATE timbral change -- the strings stop coupling, so of course
    // the render differs. A raw click reading across the gesture therefore measures two things at
    // once and cannot say whether the store release contributed anything. The control separates
    // them: a gesture to `couplingStrength = 1e-6` is acoustically the same decoupling but stays in
    // the LOADED branch, so the junction's store DECAYS instead of being discarded. The difference
    // between the two readings is the store release's own contribution, and that is what is gated.
    constexpr int kStrings = 6;
    constexpr int kChord[kStrings] = {40, 45, 50, 55, 59, 64};
    constexpr int kTailBlocks = 200;
    constexpr double kPreSeconds = 0.25;
    constexpr double kPostSeconds = 0.06;
    // Bound on the store release's own contribution. Chosen from the measurement, with the
    // reasoning stated at the gate at the end of this case.
    constexpr double kStoreReleaseLimitDb = 0.05;

    struct Config {
        const char* what;
        float resonanceHz;
        float damping;
        float from;
        int ringBlocks; // when the knob lands
    };
    // The shipping default, plus the corners that make the DISCARDED STORE as large as possible
    // relative to what the strings are doing when the knob lands -- which is the quantity being
    // decomposed out. That means full coupling into a high-Q mode sitting on the chord (the junction
    // holds the most), and landing the gesture LATE in the decay (the strings are quietest, so the
    // store's share of the total motion is largest).
    const Config configs[] = {
        {"shipping default", 180.0f, 0.5f, 0.35f, 400},
        {"full coupling, high-Q mode on the chord", 165.0f, 0.02f, 1.0f, 400},
        {"...and landed late in the decay", 165.0f, 0.02f, 1.0f, 1400},
        {"...high-Q on the chord root", 82.4f, 0.01f, 1.0f, 1400},
    };

    // `to` < 0 means "hold" (the frozen reference). rampBlocks > 0 makes it an automation ramp
    // rather than an instant jump.
    auto render = [&](const Config& config, float to, int rampBlocks) {
        const int kRingBlocks = config.ringBlocks;
        StringNetworkParams params;
        params.pickupPosition01 = 0.87f;
        params.exciter.noiseAmount = 0.0f;
        params.stringMaterial.lossGainLow = 1.0f; // sustain, so the chord is still audible at the gesture
        params.stringMaterial.lossGainHigh = 1.0f;
        params.bridge.couplingStrength = config.from;
        params.bridge.resonanceHz = config.resonanceHz;
        params.bridge.damping = config.damping;

        StringNetwork<float> network;
        network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(kStrings);
        network.setParams(params);
        network.reset();

        BlockEventQueue events;
        for (int s = 0; s < kStrings; ++s)
            events.push(noteOn(s * 13, kChord[s], s));

        std::vector<float> mix;
        const int total = kRingBlocks + kTailBlocks;
        mix.reserve(static_cast<std::size_t>(total) * static_cast<std::size_t>(kBlock));
        for (int b = 0; b < total; ++b) {
            if (to >= 0.0f && b >= kRingBlocks) {
                const int step = b - kRingBlocks;
                const float t = (rampBlocks <= 0)
                                    ? 1.0f
                                    : std::min(1.0f, static_cast<float>(step) / static_cast<float>(rampBlocks));
                params.bridge.couplingStrength = config.from + (to - config.from) * t;
                network.setParams(params);
            }
            network.process(events, kBlock);
            const int channels = network.tapBuffers().numStrings();
            for (int n = 0; n < kBlock; ++n) {
                float sum = 0.0f;
                for (int s = 0; s < channels; ++s) {
                    const float* tap = network.tapBuffers().channel(s, 0);
                    if (tap != nullptr && network.tapBuffers().isActive(s))
                        sum += tap[n];
                }
                mix.push_back(sum);
            }
        }
        REQUIRE(network.unbridgedTicks() == 0);
        return mix;
    };

    // THE SPAN STARTS AT THE GESTURE, not before it. All four renders are bit-identical up to that
    // sample, so a span with pre-roll takes its peak |dx| from the shared part and every reading
    // comes out at exactly 0 dB -- measured, and it is the metric's documented blind spot (a
    // non-scale-invariant peak comparison masked by concurrent legitimate motion, ClickMetric.h).
    // The pre-window is still rendered and is used below to assert the reference has motion at all.
    double worstStoreRelease = 0.0;
    double worstAbsolute = 0.0;
    const char* worstAt = "";
    for (const Config& config : configs) {
        const auto gestureSample = static_cast<std::size_t>(config.ringBlocks) * static_cast<std::size_t>(kBlock);
        const auto preBegin = gestureSample - static_cast<std::size_t>(kPreSeconds * kRate);
        const auto spanBegin = gestureSample;
        const auto spanEnd = gestureSample + static_cast<std::size_t>(kPostSeconds * kRate);

        const std::vector<float> frozen = render(config, -1.0f, 0);
        const std::vector<float> toZero = render(config, 0.0f, 0);    // enters the rigid branch
        const std::vector<float> toTiny = render(config, 1.0e-6f, 0); // stays loaded: the control
        const std::vector<float> ramped = render(config, 0.0f, 94);   // ~200 ms at 48 kHz / 128

        const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(frozen, kRate, spanBegin, spanEnd);
        const double zeroDb =
            cnpg::test::clickExcessDb(cnpg::test::measureClick(toZero, kRate, spanBegin, spanEnd), reference);
        const double tinyDb =
            cnpg::test::clickExcessDb(cnpg::test::measureClick(toTiny, kRate, spanBegin, spanEnd), reference);
        const double rampDb =
            cnpg::test::clickExcessDb(cnpg::test::measureClick(ramped, kRate, spanBegin, spanEnd), reference);
        const double storeRelease = zeroDb - tinyDb;

        std::cout << "[contract] Bridge Coupling -> 0 (" << config.what << "): instant jump " << zeroDb
                  << " dB, control jump to 1e-6 (stays loaded, store decays) " << tinyDb
                  << " dB, so the STORE RELEASE contributes " << storeRelease << " dB; 200 ms ramp " << rampDb
                  << " dB (click limit " << cnpg::test::kClickMetricToleranceDb << ")\n";

        // IN the state this case claims to test: the renders really differ, and the reference really
        // has motion to normalise against.
        INFO(config.what);
        REQUIRE(reference.metric(reference) > 0.0);
        // The chord really is still sounding when the knob lands: the pre-gesture window has real
        // motion, so this is a gesture over a ringing instrument and not over a decayed one.
        REQUIRE(cnpg::test::measureClick(frozen, kRate, preBegin, gestureSample).peakWindowAbsDiff > 0.0);
        REQUIRE(cnpg::test::measureClick(toZero, kRate, spanBegin, spanEnd).nonFiniteSamples == 0);
        bool differs = false;
        for (std::size_t n = spanEnd; n < frozen.size() && !differs; ++n)
            differs = (toZero[n] != frozen[n]);
        REQUIRE(differs);

        // THE GATE. Not the absolute reading: at the worst configuration that is dominated by the
        // legitimate loss of coupling, which the control measures at nearly the same value. What the
        // discarded store contributes on top of it is what this task introduced, and it is what must
        // stay inaudible.
        if (storeRelease > worstStoreRelease) {
            worstStoreRelease = storeRelease;
            worstAt = config.what;
        }
        worstAbsolute = std::max(worstAbsolute, zeroDb);
        // A ramped automation move -- what a host actually sends -- must clear the click gate
        // outright.
        REQUIRE(rampDb <= cnpg::test::kClickMetricToleranceDb);
    }

    std::cout << "[contract] worst store-release contribution over " << std::size(configs)
              << " configurations: " << worstStoreRelease << " dB (" << worstAt << "), limit " << kStoreReleaseLimitDb
              << " dB; worst ABSOLUTE gesture reading " << worstAbsolute << " dB (click limit "
              << cnpg::test::kClickMetricToleranceDb << ")\n";

    // THE GATE, against a MEASURED constant with stated headroom rather than one that cannot fail.
    // Worst measured over these configurations is ~1.5e-4 dB; the limit is 0.05 dB, a few hundred
    // times that, which is the smallest round number that is still unambiguously inaudible and that
    // a real regression would have to clear. (The P2.4 re-review measured up to 0.24 dB in a
    // configuration it did not specify; that is larger than anything reachable here, and is quoted
    // in the report as theirs rather than folded into this limit as mine.)
    REQUIRE(worstStoreRelease <= kStoreReleaseLimitDb);
    // Non-vacuous: if this ever reads exactly 0 the control has stopped being a control (both
    // gestures would be taking the same branch).
    REQUIRE(worstStoreRelease > 0.0);
    // ...and the whole gesture, store release included, clears the standard click gate at every
    // configuration tested.
    REQUIRE(worstAbsolute <= cnpg::test::kClickMetricToleranceDb);
}

TEST_CASE("CONTRACT: BridgeJunction scatter is realtime-safe under a live parameter stream", "[contract]") {
    auto port = makePort<JunctionAdapter>(kRate, cnpg::dsp::kMaxStrings, 0.35f);
    cnpg::test::resetAllocationCount();
    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
    BridgeAdmittanceParams admittance;
    for (int n = 0; n < 200000; ++n) {
        if ((n % 128) == 0) {
            admittance.resonanceHz = 100.0f + 0.01f * static_cast<float>(n % 100000);
            admittance.damping = 0.05f + 0.00001f * static_cast<float>(n % 50000);
            admittance.couplingStrength = 0.5f + 0.5f * std::sin(0.0001f * static_cast<float>(n));
            port.setAdmittance(admittance);
        }
        for (int p = 0; p < cnpg::dsp::kMaxStrings; ++p)
            incident[static_cast<std::size_t>(p)] = std::sin(0.01 * n + p);
        port.scatter(incident.data(), outgoing.data(), cnpg::dsp::kMaxStrings);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// ---------------------------------------------------------------------------------------------
// B4: junction liveness -- a port that does not drive is a SILENT wrong answer
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: every rendered string tick is driven by the bridge port", "[contract]") {
    // THE FAILURE MODE THIS EXISTS FOR (carry-forward B4, originally a P1.4 note):
    // WaveguideString::tick() falls back to its own rigid -1 whenever no reflection has been
    // supplied for that sample. That fallback is correct for an isolated string and it is a silent
    // wrong answer inside a network -- the instrument still plays, in tune, at the right level,
    // with no sympathetic resonance and no bridge output, and nothing fails. A missed
    // railAcceptFromBridge on one string in one block is invisible to every level-based test in
    // the suite.
    //
    // So the counters are asserted directly, and BOTH ways: the coupled network must show zero
    // fallback ticks, and an isolated string driven WITHOUT a port must show only fallback ticks
    // -- otherwise the assertion would be satisfied by a counter that never increments.
    constexpr int kStrings = 6;
    constexpr int kBlocks = 300;

    StringNetworkParams params;
    params.bridge.couplingStrength = 0.5f;
    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(kStrings);
    network.setParams(params);
    network.reset();
    REQUIRE(network.bridgeDrivenTicks() == 0); // reset really cleared the evidence
    REQUIRE(network.unbridgedTicks() == 0);

    BlockEventQueue events;
    events.push(noteOn(0, 40, 0));
    for (int b = 0; b < kBlocks; ++b)
        network.process(events, kBlock);

    const unsigned long long driven = network.bridgeDrivenTicks();
    const unsigned long long unbridged = network.unbridgedTicks();
    std::cout << "[contract] junction liveness over " << kBlocks << " blocks x " << kBlock << " samples x " << kStrings
              << " strings: " << driven << " bridge-driven ticks, " << unbridged << " fell back to the internal rigid "
              << "-1\n";

    REQUIRE(unbridged == 0);
    // Non-vacuous, and quantitatively so: every string in the loop is ticked on every rendered
    // sample once the bridge is moving, so the count is the trip count times the sample count --
    // not merely "greater than zero", which a single driven tick would satisfy.
    REQUIRE(driven > static_cast<unsigned long long>(kBlocks) * static_cast<unsigned long long>(kBlock));

    // THE NEGATIVE CONTROL. Same class, no port: the counter that must be zero above must be
    // non-zero here, or it is not measuring anything.
    WaveguideString<float> isolated;
    isolated.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    isolated.reset();
    for (int n = 0; n < 1000; ++n)
        isolated.tick();
    REQUIRE(isolated.internalReflectionTicks() == 1000);
    REQUIRE(isolated.bridgeReflectionTicks() == 0);
    // ...and it flips the moment a reflection IS supplied.
    for (int n = 0; n < 500; ++n) {
        isolated.railAcceptFromBridge(-isolated.railOutgoingAtBridge());
        isolated.tick();
    }
    REQUIRE(isolated.internalReflectionTicks() == 1000);
    REQUIRE(isolated.bridgeReflectionTicks() == 500);
}

TEST_CASE("CONTRACT: a substituted bridge port really replaces the internal junction", "[contract]") {
    // setBridgePort() is the seam docs/plan.md Q17 designs the P2.5 fallback around. If it silently
    // kept using the internal BridgeJunction, the fallback protocol would have nowhere to land --
    // and it would take a listening pass to notice. A rigid substitute is the cleanest probe: it
    // must decouple the strings AND zero the bridge output, on a network whose own admittance says
    // otherwise.
    constexpr int kStrings = 2;
    constexpr int kBlocks = 200;

    auto render = [](bool substitute, std::vector<float>& bridgeOut) {
        StringNetworkParams params;
        params.bridge.couplingStrength = 1.0f;
        StringNetwork<float> network;
        RigidBridgeTermination<float> rigid;
        network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(kStrings);
        network.setParams(params);
        if (substitute)
            network.setBridgePort(rigid);
        network.reset();

        BlockEventQueue events;
        events.push(noteOn(0, 45, 0));
        std::vector<float> tapOfSilentString;
        for (int b = 0; b < kBlocks; ++b) {
            network.process(events, kBlock);
            const float* other = network.tapBuffers().channel(1, 0);
            for (int n = 0; n < kBlock; ++n) {
                bridgeOut.push_back(network.bridgeOutputBuffer()[n]);
                tapOfSilentString.push_back(other != nullptr ? other[n] : 0.0f);
            }
        }
        REQUIRE(network.unbridgedTicks() == 0); // the substitute drives the strings too
        float peak = 0.0f;
        for (float value : tapOfSilentString)
            peak = std::max(peak, std::fabs(value));
        return peak;
    };

    std::vector<float> coupledBridge;
    std::vector<float> rigidBridge;
    const float coupledCrosstalk = render(false, coupledBridge);
    const float rigidCrosstalk = render(true, rigidBridge);

    float coupledBridgePeak = 0.0f;
    for (float value : coupledBridge)
        coupledBridgePeak = std::max(coupledBridgePeak, std::fabs(value));
    for (float value : rigidBridge)
        REQUIRE(value == 0.0f); // a rigid bridge does not move

    std::cout << "[contract] setBridgePort substitution: unplucked string peak " << coupledCrosstalk
              << " with the internal junction vs " << rigidCrosstalk << " with a rigid substitute; bridge-output peak "
              << coupledBridgePeak << " vs 0\n";

    REQUIRE(coupledBridgePeak > 0.0f);
    REQUIRE(coupledCrosstalk > 0.0f);
    REQUIRE(rigidCrosstalk == 0.0f);
}

// ---------------------------------------------------------------------------------------------
// B3: the seam's delay contribution, measured, at all three rates -- data for Task P2.7
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: the bridge seam costs exactly one loop sample and the solve pays it back", "[contract][tuning]") {
    // carry-forward B3: "the bridge seam adds one loop sample when driven. Tuning is affected.
    // cnpg_calibrate (P2.7) must absorb it, and P2.7 must NOT assume P1.4's tuning numbers carry
    // over the seam. Measure and report the seam's actual delay contribution at all three rates."
    //
    // THE DERIVATION, so the number is checkable rather than merely printed. StringNetwork's loop
    // reads railOutgoingAtBridge() (which tick n-1 produced), scatters, hands the reflection back,
    // and only then ticks -- so the wave that leaves the string at sample n-1 re-enters it at
    // sample n instead of within tick n-1 itself. That is one sample, independent of rate and of
    // note, because a scattering junction is memoryless in its incident waves: the LOAD's dynamics
    // add to the reflection (sM, sK) rather than delaying it.
    //
    // WaveguideString::setBridgePortDriven(true) subtracts exactly that from the loop-length solve,
    // so realizedLoopDelaySamples() -- which is the total, seam included -- still equals fs/f0.
    // What the table below reports is what the seam would have cost if it were NOT paid back,
    // which is the figure P2.7 needs to know is already handled.
    std::cout << "[tuning] bridge seam delay contribution (carry-forward B3, for Task P2.7):\n";
    double worstResidual = 0.0;

    for (double sampleRate : kRates) {
        for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
            for (int midiNote : {21, 40, 69, 96, 108}) {
                const double f0 = cnpg::test::midiNoteToHz(midiNote);
                WaveguideString<double> free;
                WaveguideString<double> seamed;
                for (WaveguideString<double>* string : {&free, &seamed}) {
                    string->prepare(sampleRate, kBlock, kind);
                    WaveguideStringParams params;
                    params.f0Hz = static_cast<float>(f0);
                    string->setParams(params);
                    string->setAnalyticTuningCompensation(0.0f);
                    string->reset();
                }
                seamed.setBridgePortDriven(true);

                REQUIRE(free.bridgePortDriven() == false);
                REQUIRE(seamed.bridgePortDriven() == true);

                // Both solves must land on the same TOTAL loop delay, fs / f0 -- the seamed one by
                // using one sample less rail.
                const double target = sampleRate / f0;
                const double freeDelay = free.realizedLoopDelaySamples();
                const double seamedDelay = seamed.realizedLoopDelaySamples();
                const double residual = std::fabs(seamedDelay - target);
                worstResidual = std::max(worstResidual, residual);

                // What the seam would have cost, in cents, had the solve not paid it back:
                // one sample of a period of fs/f0 samples.
                const double uncompensatedCents = 1200.0 / std::log(2.0) * (f0 / sampleRate);

                if (kind == FractionalDelayKind::Lagrange3 && midiNote == 108)
                    std::cout << "  " << sampleRate << " Hz: 1.000 sample = " << (1000.0 / sampleRate)
                              << " ms; uncompensated that would be " << uncompensatedCents
                              << " cents at MIDI 108 (worst note in the design envelope)\n";

                INFO("rate " << sampleRate << " note " << midiNote << " kind "
                             << (kind == FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1") << ": free "
                             << freeDelay << " seamed " << seamedDelay << " target " << target << " uncompensated "
                             << uncompensatedCents << " cents");
                // The bound is the LOOP-LENGTH SOLVER's own precision, not the seam's: the free
                // string misses fs/f0 by the same amount the seamed one does (worst 1.1e-5 samples
                // over this grid, which at MIDI 40 / 44.1 kHz is 2e-8 of a period, i.e. 3.5e-5
                // cents -- five orders under the +/-2 cent gate). What this case is measuring is
                // that declaring the seam does not move that residual, and the two REQUIREs are
                // written against the same number for exactly that reason.
                REQUIRE(std::fabs(freeDelay - target) < 1.0e-3);
                REQUIRE(residual < 1.0e-3);
                REQUIRE(std::fabs(residual - std::fabs(freeDelay - target)) < 1.0e-6);
                REQUIRE(WaveguideString<double>::kBridgeSeamDelaySamples == 1.0);
            }
        }
    }
    std::cout << "  worst |realized loop delay - fs/f0|: " << worstResidual
              << " samples, with and without the seam alike (the loop solver's own precision)\n";
    REQUIRE(worstResidual < 1.0e-3);
}

TEST_CASE("TUNING: what the bridge seam would cost if the solve did not pay it back", "[tuning]") {
    // The other half of carry-forward B3, and the half that is a MEASUREMENT rather than an
    // internal number: drive an isolated string through a rigid bridge port at the network's own
    // one-sample offset, once with setBridgePortDriven(true) and once without, and measure the
    // pitch that comes out. The difference is what P2.7 would have been handed if this task had
    // shipped the seam without the compensation, and it is large -- which is why it is measured
    // here rather than asserted to be small somewhere else.
    constexpr int kMidiNote = 69;
    constexpr double kSeconds = 3.0;

    std::cout << "[tuning] the seam's UNCOMPENSATED cost, measured (carry-forward B3):\n";
    double worstAgreement = 0.0;

    for (double sampleRate : kRates) {
        auto renderPitch = [&](bool declareSeam) {
            WaveguideString<double> string;
            string.prepare(sampleRate, 512, FractionalDelayKind::Lagrange3);
            WaveguideStringParams params;
            params.f0Hz = static_cast<float>(cnpg::test::midiNoteToHz(kMidiNote));
            params.stringMaterial.lossGainLow = 1.0f; // sustain end, so 3 s of tone exists to analyse
            params.stringMaterial.lossGainHigh = 1.0f;
            string.setParams(params);
            string.setAnalyticTuningCompensation(0.0f);
            string.setBridgePortDriven(declareSeam);
            string.reset();

            cnpg::dsp::PluckExciter<double> exciter;
            exciter.prepare(sampleRate, 512);
            cnpg::dsp::PluckExciterParams exciterParams;
            exciterParams.noiseAmount = 0.0f;
            exciter.setParams(exciterParams);
            exciter.trigger(0.8f, 0.28f, 0.5f);

            cnpg::dsp::RigidBridgeTermination<double> rigid;
            rigid.prepare(sampleRate, 512, 1, nullptr);

            const auto total = static_cast<std::size_t>(kSeconds * sampleRate);
            std::vector<double> tap;
            tap.reserve(total);
            for (std::size_t n = 0; n < total; ++n) {
                const double excitation = exciter.renderSample();
                if (excitation != 0.0)
                    string.injectAt(exciter.latchedPosition01(), excitation);
                tap.push_back(string.readTapAt(0.87f));
                // EXACTLY the network's ordering: last tick's outgoing wave, scattered, handed back
                // before this tick. That ordering is the seam.
                const double incident = string.railOutgoingAtBridge();
                double reflected = 0.0;
                rigid.scatter(&incident, &reflected, 1);
                string.railAcceptFromBridge(reflected);
                string.tick();
            }
            REQUIRE(string.internalReflectionTicks() == 0); // the port really drove every tick
            const auto discard = static_cast<std::size_t>(0.5 * sampleRate);
            std::vector<double> analysis(tap.begin() + static_cast<std::ptrdiff_t>(discard), tap.end());
            const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(analysis, sampleRate);
            const double target = cnpg::test::midiNoteToHz(kMidiNote);
            return cnpg::test::findPeakHz(spectrum, target, 120.0);
        };

        const double target = cnpg::test::midiNoteToHz(kMidiNote);
        const double compensated = renderPitch(true);
        const double uncompensated = renderPitch(false);
        REQUIRE(compensated > 0.0);
        REQUIRE(uncompensated > 0.0);
        const double compensatedCents = cnpg::test::centsBetween(compensated, target);
        const double uncompensatedCents = cnpg::test::centsBetween(uncompensated, target);
        // One sample of a period of fs/f0 samples, in cents.
        const double predictedCents = -1200.0 / std::log(2.0) * (target / sampleRate);
        const double agreement = std::fabs(uncompensatedCents - predictedCents);
        worstAgreement = std::max(worstAgreement, agreement);

        std::cout << "  " << sampleRate << " Hz, MIDI " << kMidiNote << ": WITH the compensation " << compensatedCents
                  << " cents; WITHOUT it " << uncompensatedCents << " cents (predicted " << predictedCents
                  << " -- one sample of a " << (sampleRate / target) << "-sample loop)\n";

        // The compensation works: the seamed, compensated string is in tune to the same accuracy an
        // unseamed one is.
        REQUIRE(std::fabs(compensatedCents) < 2.0);
        // ...and the uncompensated one is off by the predicted amount, which is what says the
        // compensation is doing something rather than that the seam never cost anything.
        REQUIRE(agreement < 1.5);
        REQUIRE(std::fabs(uncompensatedCents) > 5.0);
    }
    std::cout << "  worst |measured - predicted| for the uncompensated case: " << worstAgreement << " cents\n";
    REQUIRE(worstAgreement < 1.5);
}

TEST_CASE("TUNING: the coupled topology's residual tuning error, measured at three rates", "[tuning]") {
    // docs/plan.md section 4.5 accepts tuning "against the shipping topology", which from Task P2.4
    // is a StringNetwork with a LOADED bridge. The seam's own sample is paid back by the solve (the
    // case above), so what is left is the LOAD's phase response: a bridge resonance pulls the
    // partials near it, which is physics and not an error -- it is the same mechanism that produces
    // dead spots on a real instrument.
    //
    // This case does NOT re-gate ±2 cents. docs/plan.md section 4.5 binds the full 88-note gate to
    // the P2 calibration-table case, and carry-forward B3 explicitly assigns absorbing the coupled
    // topology's residual to Task P2.7. What this case does is MEASURE it, at the shipping default,
    // at all three rates, and hand P2.7 the table -- plus a loose sanity bound so a residual that
    // suddenly became enormous would still fail something.
    constexpr double kSanityCents = 35.0;
    constexpr double kRenderSeconds = 5.0;
    constexpr int kNotes[] = {33, 40, 45, 52, 57, 64, 69, 76, 84, 96};

    std::cout << "[tuning] coupled-topology residual at couplingStrength "
              << StringNetworkParams{}.bridge.couplingStrength << ", resonance "
              << StringNetworkParams{}.bridge.resonanceHz << " Hz, damping " << StringNetworkParams{}.bridge.damping
              << " (carry-forward B3 -- Task P2.7 absorbs this):\n";

    double worstCents = 0.0;
    int worstNote = 0;
    double worstRate = 0.0;

    for (double sampleRate : kRates) {
        double rateWorst = 0.0;
        int rateWorstNote = 0;
        for (int midiNote : kNotes) {
            StringNetworkParams params;
            params.pickupPosition01 = 0.87f;
            // Sustain end of the loss knobs, exactly as the [tuning] sweep does and for the same
            // reason: at the default material the high notes have decayed to nothing before the
            // analysis window opens. Same filters, same code path, a value inside the shipping
            // range.
            params.stringMaterial.lossGainLow = 1.0f;
            params.stringMaterial.lossGainHigh = 1.0f;
            params.exciter.noiseAmount = 0.0f;

            StringNetwork<float> network;
            network.prepare(sampleRate, 512, FractionalDelayKind::Lagrange3);
            network.setNumStrings(1);
            network.setParams(params);
            network.reset();
            // IN the state this case claims to measure: the shipping default really is a LOADED
            // bridge, and the string really is being driven through it.
            REQUIRE(network.internalBridgeJunction().currentCouplingStrength() > 0.0f);
            REQUIRE(network.internalBridgeJunction().instantaneousMobility() > 0.0);

            BlockEventQueue events;
            events.push(noteOn(0, midiNote, 0));
            std::vector<double> tap;
            const auto total = static_cast<std::size_t>(kRenderSeconds * sampleRate);
            tap.reserve(total);
            while (tap.size() < total) {
                network.process(events, 512);
                const float* channel = network.tapBuffers().channel(0, 0);
                for (int n = 0; n < 512 && tap.size() < total; ++n)
                    tap.push_back(static_cast<double>(channel[n]));
            }
            REQUIRE(network.unbridgedTicks() == 0);

            // Discard the attack, exactly as section 4.5's recipe does.
            const auto discard = static_cast<std::size_t>(0.5 * sampleRate);
            std::vector<double> analysis(tap.begin() + static_cast<std::ptrdiff_t>(discard), tap.end());
            const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(analysis, sampleRate);
            const double target = cnpg::test::midiNoteToHz(midiNote);
            const double measured = cnpg::test::findPeakHz(spectrum, target, cnpg::test::kTuningSearchCents);
            INFO("rate " << sampleRate << " MIDI " << midiNote << " target " << target << " Hz measured " << measured
                         << " Hz");
            REQUIRE(measured > 0.0);
            const double cents = cnpg::test::centsBetween(measured, target);
            if (std::fabs(cents) > rateWorst) {
                rateWorst = std::fabs(cents);
                rateWorstNote = midiNote;
            }
            if (std::fabs(cents) > worstCents) {
                worstCents = std::fabs(cents);
                worstNote = midiNote;
                worstRate = sampleRate;
            }
            REQUIRE(std::fabs(cents) <= kSanityCents);
        }
        std::cout << "  " << sampleRate << " Hz: worst |error| " << rateWorst << " cents at MIDI " << rateWorstNote
                  << "\n";
    }
    std::cout << "  overall worst " << worstCents << " cents at MIDI " << worstNote << " / " << worstRate
              << " Hz (sanity bound " << kSanityCents << "; the +/-2 cent gate binds at Task P2.7's calibration "
              << "table, docs/plan.md section 4.5)\n";
    // Non-vacuous in the other direction too: if this ever reads exactly 0 the render stopped
    // going through the bridge.
    REQUIRE(worstCents > 0.0);
}

TEST_CASE("TUNING: the coupled residual is a function of three LIVE parameters", "[tuning]") {
    // THE DATA TASK P2.7 STARTS FROM, and the reason its scheduled mechanism may not be the right
    // one (P2.4 review, I1). A calibration table indexed by MIDI note can represent a residual that
    // is a function of the NOTE. This one is a function of the note AND of three parameters the user
    // can turn while playing -- and the sign reverses across resonance, so it is not even a matter
    // of scaling one table. Measured here rather than argued, so P2.7's scope decision is made
    // against numbers.
    //
    // This case does NOT try to solve that. It measures, prints and pins the shape.
    constexpr int kProbeNote = 45; // where the shipping default's residual is worst
    constexpr double kProbeRate = 48000.0;
    constexpr double kRenderSeconds = 5.0;

    auto residualCents = [&](float coupling, float resonanceHz, float damping) {
        StringNetworkParams params;
        params.pickupPosition01 = 0.87f;
        params.stringMaterial.lossGainLow = 1.0f;
        params.stringMaterial.lossGainHigh = 1.0f;
        params.exciter.noiseAmount = 0.0f;
        params.bridge.couplingStrength = coupling;
        params.bridge.resonanceHz = resonanceHz;
        params.bridge.damping = damping;

        StringNetwork<float> network;
        network.prepare(kProbeRate, 512, FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        network.setParams(params);
        network.reset();
        REQUIRE(network.internalBridgeJunction().currentCouplingStrength() == coupling);

        BlockEventQueue events;
        events.push(noteOn(0, kProbeNote, 0));
        std::vector<double> tap;
        const auto total = static_cast<std::size_t>(kRenderSeconds * kProbeRate);
        tap.reserve(total);
        while (tap.size() < total) {
            network.process(events, 512);
            const float* channel = network.tapBuffers().channel(0, 0);
            for (int n = 0; n < 512 && tap.size() < total; ++n)
                tap.push_back(static_cast<double>(channel[n]));
        }
        REQUIRE(network.unbridgedTicks() == 0);
        const auto discard = static_cast<std::size_t>(0.5 * kProbeRate);
        std::vector<double> analysis(tap.begin() + static_cast<std::ptrdiff_t>(discard), tap.end());
        const double target = cnpg::test::midiNoteToHz(kProbeNote);
        const double measured =
            cnpg::test::findPeakHz(cnpg::test::computeSpectrum(analysis, kProbeRate), target, 120.0);
        REQUIRE(measured > 0.0);
        return cnpg::test::centsBetween(measured, target);
    };

    const auto defaults = StringNetworkParams{}.bridge;
    std::cout << "[tuning] the coupled residual against each LIVE parameter (MIDI " << kProbeNote << ", " << kProbeRate
              << " Hz) -- data for Task P2.7:\n";

    std::cout << "  couplingStrength (resonance " << defaults.resonanceHz << ", damping " << defaults.damping << "):";
    double atZeroCoupling = 0.0;
    double atFullCoupling = 0.0;
    for (float coupling : {0.0f, 0.35f, 1.0f}) {
        const double cents = residualCents(coupling, defaults.resonanceHz, defaults.damping);
        std::cout << "  " << coupling << " -> " << cents << " cents;";
        if (coupling == 0.0f)
            atZeroCoupling = cents;
        if (coupling == 1.0f)
            atFullCoupling = cents;
    }
    std::cout << "\n";

    std::cout << "  resonanceHz (coupling " << defaults.couplingStrength << ", damping " << defaults.damping << "):";
    double lowestResonance = 0.0;
    double highestResonance = 0.0;
    for (float resonance : {80.0f, 110.0f, 180.0f, 2000.0f}) {
        const double cents = residualCents(defaults.couplingStrength, resonance, defaults.damping);
        std::cout << "  " << resonance << " Hz -> " << cents << " cents;";
        if (resonance == 80.0f)
            lowestResonance = cents;
        if (resonance == 2000.0f)
            highestResonance = cents;
    }
    std::cout << "\n";

    std::cout << "  damping (coupling " << defaults.couplingStrength << ", resonance " << defaults.resonanceHz << "):";
    for (float damping : {0.01f, 0.5f, 10.0f}) {
        const double cents = residualCents(defaults.couplingStrength, defaults.resonanceHz, damping);
        std::cout << "  " << damping << " -> " << cents << " cents;";
    }
    std::cout << "\n";

    // THE SHAPE, pinned. A note-indexed table can absorb a residual that depends on the note; these
    // three assertions are the statement that this one does not depend only on the note.
    REQUIRE(std::fabs(atZeroCoupling) < 0.5); // decoupled: the analytic solve is exact
    REQUIRE(std::fabs(atFullCoupling) > 2.0 * std::fabs(atZeroCoupling) + 5.0); // and it grows with coupling
    // ...and the SIGN REVERSES across resonance, which is the part no single correction curve
    // indexed by note can represent.
    REQUIRE(lowestResonance * highestResonance < 0.0);
    std::cout << "  SHAPE: the residual is 0 when decoupled, grows with coupling, and REVERSES SIGN across the "
                 "resonance sweep ("
              << lowestResonance << " cents at 80 Hz vs " << highestResonance
              << " cents at 2000 Hz) -- so it is not a function of the MIDI note alone, which is what a "
                 "note-indexed calibration table can represent. Task P2.7 scope question, surfaced not solved.\n";
}

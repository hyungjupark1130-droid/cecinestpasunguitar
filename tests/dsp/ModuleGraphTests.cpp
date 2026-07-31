// Task P1.9 -- ModuleGraph's freeze-discipline battery (docs/plan.md section 4.2,
// "CONTRACT: ModuleGraph freeze discipline"): cycle rejection, freeze, latency summation, and
// chain-equivalence vs direct calls.
//
// The chain-equivalence case is the load-bearing one. Through P2 the plugin runs the HARD-WIRED
// chain and ModuleGraph is exercised only here (ModuleGraph.h); P4 is when routing moves into the
// graph for real. That cutover is only a refactor if routing the same modules through the graph
// produces BIT-IDENTICAL audio to calling them directly in sequence -- otherwise it is a silent
// voicing change hiding inside an architecture change. This file is what makes that assertable
// now, three phases before anyone depends on it.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/ModuleGraph.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/TriodeStage.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

using namespace cnpg::dsp;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// ---------------------------------------------------------------------------------------------
// Synthetic nodes: the topology/latency cases need modules whose behaviour is trivially
// predictable, so a failure points at ModuleGraph rather than at whichever real module it wrapped.
// ---------------------------------------------------------------------------------------------

// Multiplies by a fixed gain and reports a fixed latency. Counts its own process() calls and
// records the last block size it saw, so the tests can assert scheduling rather than only audio.
class GainNode final : public IBlockModule {
  public:
    GainNode(float gain, int latency) : gain_(gain), latency_(latency) {}

    void prepare(double sampleRate, int maxBlockSize) override {
        preparedRate = sampleRate;
        preparedBlock = maxBlockSize;
        ++prepareCalls;
    }

    void reset() noexcept override { ++resetCalls; }

    void process(const Sample* in, Sample* out, int numSamples) noexcept override {
        ++processCalls;
        lastNumSamples = numSamples;
        for (int n = 0; n < numSamples; ++n)
            out[n] = in[n] * gain_;
    }

    int latencySamples() const noexcept override { return latency_; }

    double preparedRate = 0.0;
    int preparedBlock = 0;
    int prepareCalls = 0;
    int resetCalls = 0;
    int processCalls = 0;
    int lastNumSamples = 0;

  private:
    float gain_;
    int latency_;
};

// ---------------------------------------------------------------------------------------------
// IBlockModule adapters over the real monitoring-chain modules.
//
// The concrete dsp/ modules deliberately do NOT implement IBlockModule themselves (ModuleGraph.h):
// they keep their natural signatures, and a thin adapter bridges each one. Through P2 those
// adapters live here, in the tests. The Oversampler adapter is the reason the seam exists at all --
// it owns the wrapped nonlinearity, prepares it at the OVERSAMPLED rate and block size
// (Oversampler.h's wiring note), and is the only node in the P1 chain with a latency to report.
// ---------------------------------------------------------------------------------------------

class PickupNode final : public IBlockModule {
  public:
    void prepare(double sampleRate, int maxBlockSize) override { pickup.prepare(sampleRate, maxBlockSize); }
    void reset() noexcept override { pickup.reset(); }
    void process(const Sample* in, Sample* out, int numSamples) noexcept override {
        pickup.processMono(in, out, numSamples);
    }
    int latencySamples() const noexcept override { return 0; }

    PickupTap pickup;
};

class OversampledTriodeNode final : public IBlockModule {
  public:
    void prepare(double sampleRate, int maxBlockSize) override {
        oversampler.prepare(sampleRate, maxBlockSize, Oversampler::kDefaultFactor);
        triode.prepare(sampleRate * oversampler.factor(), maxBlockSize * oversampler.factor());
    }

    void reset() noexcept override {
        oversampler.reset();
        triode.reset();
    }

    void process(const Sample* in, Sample* out, int numSamples) noexcept override {
        oversampler.processWrapped(in, out, numSamples, [this](Sample* buffer, int numUpsampled) noexcept {
            triode.process(buffer, buffer, numUpsampled);
        });
    }

    int latencySamples() const noexcept override { return oversampler.latencySamples(); }

    Oversampler oversampler;
    TriodeStage triode;
};

class CabNode final : public IBlockModule {
  public:
    void prepare(double sampleRate, int maxBlockSize) override { cab.prepare(sampleRate, maxBlockSize); }
    void reset() noexcept override { cab.reset(); }
    void process(const Sample* in, Sample* out, int numSamples) noexcept override { cab.process(in, out, numSamples); }
    int latencySamples() const noexcept override { return CabFilter::latencySamples(); }

    CabFilter cab;
};

class OutputGainNode final : public IBlockModule {
  public:
    void prepare(double sampleRate, int maxBlockSize) override { gain.prepare(sampleRate, maxBlockSize); }
    void reset() noexcept override { gain.reset(); }
    void process(const Sample* in, Sample* out, int numSamples) noexcept override { gain.process(in, out, numSamples); }
    int latencySamples() const noexcept override { return 0; }

    OutputGain gain;
};

class LimiterNode final : public IBlockModule {
  public:
    void prepare(double sampleRate, int maxBlockSize) override { limiter.prepare(sampleRate, maxBlockSize); }
    void reset() noexcept override { limiter.reset(); }
    void process(const Sample* in, Sample* out, int numSamples) noexcept override {
        limiter.process(in, out, numSamples);
    }
    int latencySamples() const noexcept override { return SoftClipLimiter::latencySamples(); }

    SoftClipLimiter limiter;
};

// A stimulus with real transients and real level: a decaying pluck-like burst riding a chord of
// partials, hot enough to push the triode's curve and the limiter's knee, so chain-equivalence is
// asserted over a signal that actually exercises every stage's nonlinearity.
std::vector<Sample> makeStimulus(int numSamples, double sampleRate) {
    std::vector<Sample> signal(static_cast<std::size_t>(numSamples));
    for (int n = 0; n < numSamples; ++n) {
        const double t = static_cast<double>(n) / sampleRate;
        const double envelope = std::exp(-3.0 * t);
        const double partials = std::sin(kTwoPi * 220.0 * t) + 0.6 * std::sin(kTwoPi * 660.0 * t) +
                                0.35 * std::sin(kTwoPi * 1870.0 * t) + 0.2 * std::sin(kTwoPi * 4400.0 * t);
        signal[static_cast<std::size_t>(n)] = static_cast<Sample>(0.55 * envelope * partials);
    }
    return signal;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Cycle rejection
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: ModuleGraph connect rejects cycles and invalid NodeIds", "[contract]") {
    // docs/plan.md section 2.13: "Cycles are rejected -- the P4 feedback path is an explicit
    // delay-bearing node, not a graph cycle." Checked at every length a cycle can have.
    GainNode a(1.0f, 0);
    GainNode b(1.0f, 0);
    GainNode c(1.0f, 0);

    ModuleGraph graph;
    graph.prepare(48000.0, 128);
    const auto idA = graph.addNode(a);
    const auto idB = graph.addNode(b);
    const auto idC = graph.addNode(c);
    REQUIRE(idA == 0);
    REQUIRE(idB == 1);
    REQUIRE(idC == 2);
    REQUIRE(graph.numNodes() == 3);

    SECTION("a self-edge is the shortest cycle") { REQUIRE_FALSE(graph.connect(idA, idA)); }

    SECTION("a two-node cycle") {
        REQUIRE(graph.connect(idA, idB));
        REQUIRE_FALSE(graph.connect(idB, idA));
    }

    SECTION("a three-node cycle") {
        REQUIRE(graph.connect(idA, idB));
        REQUIRE(graph.connect(idB, idC));
        REQUIRE_FALSE(graph.connect(idC, idA));
    }

    SECTION("a diamond is NOT a cycle and must be accepted") {
        REQUIRE(graph.connect(idA, idB));
        REQUIRE(graph.connect(idA, idC));
        GainNode d(1.0f, 0);
        const auto idD = graph.addNode(d);
        REQUIRE(graph.connect(idB, idD));
        REQUIRE(graph.connect(idC, idD)); // fan-in, not a cycle
    }

    SECTION("invalid NodeIds are rejected in either position") {
        REQUIRE_FALSE(graph.connect(-1, idA));
        REQUIRE_FALSE(graph.connect(idA, -1));
        REQUIRE_FALSE(graph.connect(idA, 99));
        REQUIRE_FALSE(graph.connect(99, idA));
    }

    SECTION("a duplicate edge is rejected") {
        REQUIRE(graph.connect(idA, idB));
        REQUIRE_FALSE(graph.connect(idA, idB));
    }

    SECTION("adding the same module twice is rejected") {
        REQUIRE(graph.addNode(a) == ModuleGraph::kInvalidNode);
        REQUIRE(graph.numNodes() == 3);
    }

    SECTION("a rejected edge changes nothing: the graph still freezes and runs") {
        REQUIRE(graph.connect(idA, idB));
        REQUIRE_FALSE(graph.connect(idB, idA)); // rejected
        graph.setInputNode(idA);
        graph.setOutputNode(idB);
        graph.freeze();
        REQUIRE(graph.isFrozen());

        std::vector<Sample> in(16, 0.25f);
        std::vector<Sample> out(16, 0.0f);
        graph.process(in.data(), out.data(), 16);
        for (Sample s : out)
            REQUIRE(s == 0.25f); // A -> B, both unity: a lingering half-added edge would show here
    }
}

// ---------------------------------------------------------------------------------------------
// Freeze discipline
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: ModuleGraph is inert until frozen, and every mutation unfreezes it", "[contract]") {
    GainNode a(2.0f, 0);
    GainNode b(3.0f, 0);

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    REQUIRE_FALSE(graph.isFrozen());

    const auto idA = graph.addNode(a);
    const auto idB = graph.addNode(b);
    graph.setInputNode(idA);
    graph.setOutputNode(idB);
    REQUIRE(graph.connect(idA, idB));
    REQUIRE_FALSE(graph.isFrozen());

    std::vector<Sample> in(32, 0.1f);
    std::vector<Sample> out(32, -1.0f);

    // Unfrozen: audibly inert passthrough, and NO node runs (ModuleGraph.h).
    graph.process(in.data(), out.data(), 32);
    for (Sample s : out)
        REQUIRE(s == 0.1f);
    REQUIRE(a.processCalls == 0);
    REQUIRE(b.processCalls == 0);

    graph.freeze();
    REQUIRE(graph.isFrozen());
    graph.process(in.data(), out.data(), 32);
    for (Sample s : out)
        REQUIRE(s == 0.1f * 2.0f * 3.0f);
    REQUIRE(a.processCalls == 1);
    REQUIRE(b.processCalls == 1);

    // Every mutation drops the graph back to inert until it is re-frozen -- a caller that forgets
    // must never get a half-built topology running over a stale buffer plan.
    GainNode c(5.0f, 0);
    SECTION("addNode unfreezes") {
        graph.addNode(c);
        REQUIRE_FALSE(graph.isFrozen());
    }
    SECTION("connect unfreezes") {
        const auto idC = graph.addNode(c);
        graph.freeze();
        REQUIRE(graph.connect(idB, idC));
        REQUIRE_FALSE(graph.isFrozen());
    }
    SECTION("a REJECTED connect does not unfreeze") {
        REQUIRE_FALSE(graph.connect(idB, idA)); // cycle
        REQUIRE(graph.isFrozen());
    }
    SECTION("setInputNode / setOutputNode unfreeze") {
        graph.setOutputNode(idA);
        REQUIRE_FALSE(graph.isFrozen());
    }
    SECTION("an invalid setOutputNode does not unfreeze, and keeps the old setting") {
        graph.setOutputNode(99);
        REQUIRE(graph.isFrozen());
        graph.process(in.data(), out.data(), 32);
        for (Sample s : out)
            REQUIRE(s == 0.1f * 2.0f * 3.0f);
    }
    SECTION("prepare unfreezes, because it resizes the buffer plan") {
        graph.prepare(44100.0, 64);
        REQUIRE_FALSE(graph.isFrozen());
    }
}

TEST_CASE("CONTRACT: ModuleGraph prepares every node exactly once, whatever the add/prepare order", "[contract]") {
    GainNode addedBefore(1.0f, 0);
    GainNode addedAfter(1.0f, 0);

    ModuleGraph graph;
    graph.addNode(addedBefore);
    graph.prepare(96000.0, 512);
    graph.addNode(addedAfter);

    REQUIRE(addedBefore.prepareCalls == 1);
    REQUIRE(addedAfter.prepareCalls == 1);
    REQUIRE(addedBefore.preparedRate == 96000.0);
    REQUIRE(addedAfter.preparedRate == 96000.0);
    REQUIRE(addedBefore.preparedBlock == 512);
    REQUIRE(addedAfter.preparedBlock == 512);
}

TEST_CASE("CONTRACT: ModuleGraph reset forwards to every node", "[contract]") {
    GainNode a(1.0f, 0);
    GainNode b(1.0f, 0);
    GainNode offPath(1.0f, 0); // not on the input -> output path, but still owned by the graph

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    const auto idA = graph.addNode(a);
    const auto idB = graph.addNode(b);
    graph.addNode(offPath);
    REQUIRE(graph.connect(idA, idB));
    graph.setInputNode(idA);
    graph.setOutputNode(idB);
    graph.freeze();

    graph.reset();
    REQUIRE(a.resetCalls == 1);
    REQUIRE(b.resetCalls == 1);
    REQUIRE(offPath.resetCalls == 1);
    REQUIRE(graph.isFrozen()); // reset() is not a mutation
}

TEST_CASE("CONTRACT: ModuleGraph runs every node, including nodes off the input -> output path", "[contract]") {
    // ModuleGraph.h: a stateful module whose state stopped advancing because it was temporarily off
    // the path would resume from stale memory and click when it came back.
    GainNode onPath(1.0f, 0);
    GainNode offPath(1.0f, 0);

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    const auto idOn = graph.addNode(onPath);
    graph.addNode(offPath);
    graph.setInputNode(idOn);
    graph.setOutputNode(idOn);
    graph.freeze();

    std::vector<Sample> in(16, 0.5f);
    std::vector<Sample> out(16, 0.0f);
    graph.process(in.data(), out.data(), 16);

    REQUIRE(onPath.processCalls == 1);
    REQUIRE(offPath.processCalls == 1);
    REQUIRE(offPath.lastNumSamples == 16);
}

// ---------------------------------------------------------------------------------------------
// Latency summation
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: ModuleGraph totalLatencySamples sums along the frozen path", "[contract]") {
    // docs/plan.md section 4.2: "totalLatencySamples() equals the sum of the members' reported
    // latencies".
    GainNode a(1.0f, 5);
    GainNode b(1.0f, 7);
    GainNode c(1.0f, 11);

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    const auto idA = graph.addNode(a);
    const auto idB = graph.addNode(b);
    const auto idC = graph.addNode(c);
    REQUIRE(graph.connect(idA, idB));
    REQUIRE(graph.connect(idB, idC));
    graph.setInputNode(idA);
    graph.setOutputNode(idC);

    REQUIRE(graph.totalLatencySamples() == 0); // unfrozen: nothing to report yet
    graph.freeze();
    REQUIRE(graph.totalLatencySamples() == 5 + 7 + 11);

    SECTION("a shorter output point sums only what precedes it") {
        graph.setOutputNode(idB);
        graph.freeze();
        REQUIRE(graph.totalLatencySamples() == 5 + 7);
    }

    SECTION("a node off the path contributes nothing") {
        GainNode offPath(1.0f, 1000);
        graph.addNode(offPath);
        graph.freeze();
        REQUIRE(graph.totalLatencySamples() == 5 + 7 + 11);
    }

    SECTION("with no path from input to output, the reported latency is zero") {
        GainNode isolated(1.0f, 13);
        const auto idIsolated = graph.addNode(isolated);
        graph.setOutputNode(idIsolated);
        graph.freeze();
        REQUIRE(graph.totalLatencySamples() == 0);
    }
}

TEST_CASE("CONTRACT: ModuleGraph reports the LONGEST path's latency when branches disagree", "[contract]") {
    // ModuleGraph.h: the graph REPORTS path latency, it does not align branches (per-branch delay
    // compensation is P4's, arriving with the first topology that needs it). The number a host is
    // told must therefore be the longest path, not an average and not whichever path was added
    // first -- reporting less would leave the slowest branch late even after host compensation.
    GainNode source(1.0f, 1);
    GainNode fastBranch(1.0f, 2);
    GainNode slowBranch(1.0f, 40);
    GainNode sink(1.0f, 3);

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    const auto idSource = graph.addNode(source);
    const auto idFast = graph.addNode(fastBranch);
    const auto idSlow = graph.addNode(slowBranch);
    const auto idSink = graph.addNode(sink);
    REQUIRE(graph.connect(idSource, idFast));
    REQUIRE(graph.connect(idSource, idSlow));
    REQUIRE(graph.connect(idFast, idSink));
    REQUIRE(graph.connect(idSlow, idSink));
    graph.setInputNode(idSource);
    graph.setOutputNode(idSink);
    graph.freeze();

    REQUIRE(graph.totalLatencySamples() == 1 + 40 + 3);
}

TEST_CASE("CONTRACT: ModuleGraph sums fan-in", "[contract]") {
    // Two branches of known gain converging on one node: the node must see their SUM. Plain
    // addition, no per-branch gain (ModuleGraph.h).
    GainNode source(1.0f, 0);
    GainNode branchA(2.0f, 0);
    GainNode branchB(5.0f, 0);
    GainNode sink(1.0f, 0);

    ModuleGraph graph;
    graph.prepare(48000.0, 64);
    const auto idSource = graph.addNode(source);
    const auto idA = graph.addNode(branchA);
    const auto idB = graph.addNode(branchB);
    const auto idSink = graph.addNode(sink);
    REQUIRE(graph.connect(idSource, idA));
    REQUIRE(graph.connect(idSource, idB));
    REQUIRE(graph.connect(idA, idSink));
    REQUIRE(graph.connect(idB, idSink));
    graph.setInputNode(idSource);
    graph.setOutputNode(idSink);
    graph.freeze();

    std::vector<Sample> in(16, 0.5f);
    std::vector<Sample> out(16, 0.0f);
    graph.process(in.data(), out.data(), 16);

    for (Sample s : out)
        REQUIRE(s == 0.5f * (2.0f + 5.0f));
}

// ---------------------------------------------------------------------------------------------
// Chain equivalence vs direct calls
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: ModuleGraph runs the P1 monitoring chain sample-identically to direct calls", "[contract]") {
    // docs/plan.md section 4.2: "process() on the frozen hard-wired P2 chain (PickupTap ->
    // Oversampler(TriodeStage) -> CabFilter -> OutputGain -> SoftClipLimiter wrapped as
    // IBlockModule adapters) is sample-identical to calling the modules directly in sequence".
    //
    // BIT-identical, not approximately: the P4 cutover from the hard-wired chain to the graph must
    // be a refactor, and anything short of bit-identity here would make it a voicing change.
    constexpr int kMaxBlock = 128;
    constexpr int kBlocks = 60;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        const std::vector<Sample> stimulus = makeStimulus(kBlocks * kMaxBlock, sampleRate);

        // --- graph path -------------------------------------------------------------------
        PickupNode pickupNode;
        OversampledTriodeNode triodeNode;
        CabNode cabNode;
        OutputGainNode gainNode;
        LimiterNode limiterNode;

        ModuleGraph graph;
        graph.prepare(sampleRate, kMaxBlock);
        const auto idPickup = graph.addNode(pickupNode);
        const auto idTriode = graph.addNode(triodeNode);
        const auto idCab = graph.addNode(cabNode);
        const auto idGain = graph.addNode(gainNode);
        const auto idLimiter = graph.addNode(limiterNode);
        REQUIRE(graph.connect(idPickup, idTriode));
        REQUIRE(graph.connect(idTriode, idCab));
        REQUIRE(graph.connect(idCab, idGain));
        REQUIRE(graph.connect(idGain, idLimiter));
        graph.setInputNode(idPickup);
        graph.setOutputNode(idLimiter);
        graph.freeze();

        pickupNode.pickup.setParams(PickupTapParams{});
        triodeNode.triode.setParams(TriodeStageParams{});
        cabNode.cab.setParams(CabFilterParams{});
        OutputGainParams gainParams;
        gainParams.gainDb = 6.0f; // a non-unity gain, so the stage is provably in the path
        gainNode.gain.setParams(gainParams);
        limiterNode.limiter.setParams(SoftClipLimiterParams{});
        graph.reset();

        // --- direct path ------------------------------------------------------------------
        PickupTap pickup;
        Oversampler oversampler;
        TriodeStage triode;
        CabFilter cab;
        OutputGain outputGain;
        SoftClipLimiter limiter;

        pickup.prepare(sampleRate, kMaxBlock);
        oversampler.prepare(sampleRate, kMaxBlock, Oversampler::kDefaultFactor);
        triode.prepare(sampleRate * oversampler.factor(), kMaxBlock * oversampler.factor());
        cab.prepare(sampleRate, kMaxBlock);
        outputGain.prepare(sampleRate, kMaxBlock);
        limiter.prepare(sampleRate, kMaxBlock);

        pickup.setParams(PickupTapParams{});
        triode.setParams(TriodeStageParams{});
        cab.setParams(CabFilterParams{});
        outputGain.setParams(gainParams);
        limiter.setParams(SoftClipLimiterParams{});

        pickup.reset();
        oversampler.reset();
        triode.reset();
        cab.reset();
        outputGain.reset();
        limiter.reset();

        // Separate buffers on both sides, so this compares ROUTING and nothing else -- an in-place
        // direct chain would additionally be testing each module's aliasing behaviour.
        std::vector<Sample> graphOut(static_cast<std::size_t>(kMaxBlock), 0.0f);
        std::vector<Sample> stageA(static_cast<std::size_t>(kMaxBlock), 0.0f);
        std::vector<Sample> stageB(static_cast<std::size_t>(kMaxBlock), 0.0f);

        bool sawNonZero = false;
        for (int block = 0; block < kBlocks; ++block) {
            const Sample* in = stimulus.data() + static_cast<std::size_t>(block) * static_cast<std::size_t>(kMaxBlock);

            graph.process(in, graphOut.data(), kMaxBlock);

            pickup.processMono(in, stageA.data(), kMaxBlock);
            oversampler.processWrapped(
                stageA.data(), stageB.data(), kMaxBlock,
                [&triode](Sample* buffer, int numUpsampled) noexcept { triode.process(buffer, buffer, numUpsampled); });
            cab.process(stageB.data(), stageA.data(), kMaxBlock);
            outputGain.process(stageA.data(), stageB.data(), kMaxBlock);
            limiter.process(stageB.data(), stageA.data(), kMaxBlock);

            for (int n = 0; n < kMaxBlock; ++n) {
                INFO("rate " << sampleRate << " block " << block << " sample " << n);
                REQUIRE(graphOut[static_cast<std::size_t>(n)] == stageA[static_cast<std::size_t>(n)]);
                sawNonZero |= (graphOut[static_cast<std::size_t>(n)] != 0.0f);
            }
        }
        REQUIRE(sawNonZero); // non-vacuity: two silent chains would also be "identical"

        // The graph's reported latency is the chain's real one: only the oversampled triode island
        // carries any, and it is what PluginProcessor reports to the host today.
        REQUIRE(graph.totalLatencySamples() == triodeNode.oversampler.latencySamples());
        REQUIRE(graph.totalLatencySamples() == 3); // 2x, the shipping default (Oversampler.h)
    }
}

// ---------------------------------------------------------------------------------------------
// Realtime contract
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: ModuleGraph process allocates nothing", "[contract]") {
    constexpr int kMaxBlock = 256;
    constexpr int kBlocks = 500;

    PickupNode pickupNode;
    OversampledTriodeNode triodeNode;
    CabNode cabNode;
    OutputGainNode gainNode;
    LimiterNode limiterNode;
    GainNode fanIn(0.5f, 0);

    ModuleGraph graph;
    graph.prepare(48000.0, kMaxBlock);
    const auto idPickup = graph.addNode(pickupNode);
    const auto idTriode = graph.addNode(triodeNode);
    const auto idCab = graph.addNode(cabNode);
    const auto idGain = graph.addNode(gainNode);
    const auto idLimiter = graph.addNode(limiterNode);
    const auto idFanIn = graph.addNode(fanIn);
    REQUIRE(graph.connect(idPickup, idTriode));
    REQUIRE(graph.connect(idTriode, idCab));
    REQUIRE(graph.connect(idCab, idGain));
    REQUIRE(graph.connect(idGain, idLimiter));
    // A second edge into the limiter, so the fan-in mixing path (not only the single-predecessor
    // fast path) is covered by the allocation guard too.
    REQUIRE(graph.connect(idPickup, idFanIn));
    REQUIRE(graph.connect(idFanIn, idLimiter));
    graph.setInputNode(idPickup);
    graph.setOutputNode(idLimiter);
    graph.freeze();
    graph.reset();

    const std::vector<Sample> stimulus = makeStimulus(kMaxBlock, 48000.0);
    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);

    cnpg::test::resetAllocationCount();
    for (int block = 0; block < kBlocks; ++block)
        graph.process(stimulus.data(), out.data(), kMaxBlock);
    REQUIRE(cnpg::test::allocationCount() == 0);
}

TEST_CASE("CONTRACT: ModuleGraph process clamps to the prepared block size and tolerates in-place use", "[contract]") {
    constexpr int kMaxBlock = 32;

    GainNode a(2.0f, 0);
    ModuleGraph graph;
    graph.prepare(48000.0, kMaxBlock);
    const auto idA = graph.addNode(a);
    graph.setInputNode(idA);
    graph.setOutputNode(idA);
    graph.freeze();

    constexpr Sample kSentinel = 12345.0f;
    std::vector<Sample> buffer(static_cast<std::size_t>(kMaxBlock) * 2, kSentinel);
    for (int n = 0; n < kMaxBlock; ++n)
        buffer[static_cast<std::size_t>(n)] = 0.25f;

    // In-place, and deliberately over-requesting: 2x the prepared block size.
    graph.process(buffer.data(), buffer.data(), kMaxBlock * 2);

    for (int n = 0; n < kMaxBlock; ++n)
        REQUIRE(buffer[static_cast<std::size_t>(n)] == 0.5f);
    for (int n = kMaxBlock; n < kMaxBlock * 2; ++n)
        REQUIRE(buffer[static_cast<std::size_t>(n)] == kSentinel); // untouched past the clamp
    REQUIRE(a.lastNumSamples == kMaxBlock);
}

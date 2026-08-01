#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/StringNetwork.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::ScopedFtzDazGuard;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

// DenormalTests -- docs/plan.md section 4.6 and Task P1.5 step 7. One 60 s decay tail at
// 44.1 kHz, rendered in 128-sample blocks with the same FTZ/DAZ RAII guard the plugin's
// processBlock uses engaged around each block, feeds two named cases:
//
//   DENORMAL: long tail leaves no subnormal state   -- state inspection; runs in BOTH CI jobs.
//   DENORMAL: no CPU blow-up in the tail            -- timing ratio; LOCAL-ONLY (section 4.9
//                                                      lists it as skipped on both runners,
//                                                      being flaky on shared hardware).
//
// The local-only case is tagged "[.]" alongside "[denormal]": CTest never discovers a hidden
// case, so neither CI job can run it, while `cnpg_tests.exe "[denormal]"` -- the exact command
// docs/plan.md section 4.10 gives for the dev machine -- still does, because a hidden case runs
// when the spec matches one of its tags.
//
// P1 SCOPE. Section 4.6 describes the render as "a 6-string StringNetwork + full monitoring
// chain". The 6 coupled strings are rendered here (StringNetwork already carries them; only the
// allocator is single-string in P1), but the monitoring chain -- PickupTap, TriodeStage,
// Oversampler, CabFilter, SoftClipLimiter -- lands across P1.6-P1.8, so this render stops at the
// domain boundary. Task P2.8's CorpusSweepTests re-runs the state-inspection case over the full
// chain with dampers and bridge coupling.

namespace {

constexpr double kSampleRate = 44100.0;
constexpr int kBlockSize = 128;
constexpr double kTailSeconds = 60.0;
constexpr int kNumStrings = 6;

// Standard tuning, one note per string: the "6-string network plucked once" of section 4.6.
constexpr int kChordNotes[kNumStrings] = {40, 45, 50, 55, 59, 64};

struct TailRender {
    std::vector<float> finalTaps;   // the last block of every string's tap channel, concatenated
    std::vector<float> finalBridge; // the last block of the bridge feed
    std::vector<double> blockMicros;
    double firstSecondPeak = 0.0;
    double energyAfterTail = 0.0;
    bool allFinite = true;
    std::size_t samplesScanned = 0;
};

TailRender renderDecayTail(bool timed) {
    StringNetworkParams params;
    params.pickupPosition01 = 0.87f;

    StringNetwork<float> network;
    network.prepare(kSampleRate, kBlockSize, FractionalDelayKind::Lagrange3);
    network.setNumStrings(kNumStrings);
    network.setParams(params);
    network.reset();

    BlockEventQueue events;
    for (int s = 0; s < kNumStrings; ++s) {
        NoteEvent event{};
        event.type = NoteEventType::NoteOn;
        event.sampleOffset = s * 8; // a strummed chord: non-decreasing offsets, one per string
        event.stringIndex = static_cast<std::uint8_t>(s);
        event.channel = 0;
        event.midiNote = static_cast<std::uint8_t>(kChordNotes[s]);
        event.velocity = 0.9f;
        event.pluckPosition = 0.28f;
        event.hardness = 0.5f;
        events.push(event);
    }

    const auto totalBlocks = static_cast<int>(kTailSeconds * kSampleRate / kBlockSize);
    const int firstSecondBlocks = static_cast<int>(kSampleRate / kBlockSize);

    TailRender result;
    result.blockMicros.reserve(static_cast<std::size_t>(totalBlocks));

    for (int block = 0; block < totalBlocks; ++block) {
        const auto start = std::chrono::steady_clock::now();
        {
            // The plugin's own guard, engaged around each block exactly as processBlock does.
            const ScopedFtzDazGuard ftzDazGuard;
            network.process(events, kBlockSize);
        }
        if (timed) {
            const auto elapsed = std::chrono::steady_clock::now() - start;
            result.blockMicros.push_back(
                static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()) / 1000.0);
        }

        // Scan every sample of the whole 60 s for NaN/Inf, and track the loud opening so the
        // "no subnormals at the end" assertion cannot be satisfied by a render that was silent
        // from the start.
        const auto& taps = network.tapBuffers();
        for (int s = 0; s < kNumStrings; ++s) {
            const float* channel = taps.channel(s, 0);
            for (int n = 0; n < kBlockSize; ++n) {
                result.allFinite &= std::isfinite(channel[n]);
                if (block < firstSecondBlocks)
                    result.firstSecondPeak =
                        std::max(result.firstSecondPeak, std::fabs(static_cast<double>(channel[n])));
            }
            result.samplesScanned += static_cast<std::size_t>(kBlockSize);
        }
        for (int n = 0; n < kBlockSize; ++n)
            result.allFinite &= std::isfinite(network.bridgeOutputBuffer()[n]);
    }

    const auto& taps = network.tapBuffers();
    for (int s = 0; s < kNumStrings; ++s) {
        const float* channel = taps.channel(s, 0);
        result.finalTaps.insert(result.finalTaps.end(), channel, channel + kBlockSize);
    }
    result.finalBridge.assign(network.bridgeOutputBuffer(), network.bridgeOutputBuffer() + kBlockSize);
    result.energyAfterTail = network.energyEstimate();
    return result;
}

double median(std::vector<double> values) {
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

} // namespace

TEST_CASE("DENORMAL: long tail leaves no subnormal state", "[denormal]") {
    const TailRender tail = renderDecayTail(false);

    INFO("scanned " << tail.samplesScanned << " tap samples over " << kTailSeconds << " s");
    REQUIRE(tail.allFinite);
    REQUIRE(tail.firstSecondPeak > 0.001); // the chord really did sound before decaying away

    // No value anywhere in the exposed state probes is classified FP_SUBNORMAL: the final tap
    // buffers, the final bridge feed, and the storage functional (docs/plan.md section 4.6).
    for (std::size_t i = 0; i < tail.finalTaps.size(); ++i) {
        INFO("final tap sample " << i << " = " << tail.finalTaps[i]);
        REQUIRE(std::fpclassify(tail.finalTaps[i]) != FP_SUBNORMAL);
    }
    for (std::size_t i = 0; i < tail.finalBridge.size(); ++i) {
        INFO("final bridge sample " << i << " = " << tail.finalBridge[i]);
        REQUIRE(std::fpclassify(tail.finalBridge[i]) != FP_SUBNORMAL);
    }
    INFO("energyEstimate after the tail = " << tail.energyAfterTail);
    REQUIRE(std::isfinite(tail.energyAfterTail));
    REQUIRE(std::fpclassify(tail.energyAfterTail) != FP_SUBNORMAL);

    std::cout << "[denormal] 60 s / " << kNumStrings << "-string tail: peak in the first second "
              << tail.firstSecondPeak << ", energyEstimate after the tail " << tail.energyAfterTail << "\n";
}

// Timing-based, therefore local-only (docs/plan.md section 4.9). Hidden from CTest via "[.]";
// run it with `cnpg_tests.exe "[denormal]"` on the dev machine.
TEST_CASE("DENORMAL: no CPU blow-up in the tail", "[.][denormal]") {
    const TailRender tail = renderDecayTail(true);
    REQUIRE(tail.allFinite);

    const int blocksPerSecond = static_cast<int>(kSampleRate / kBlockSize);
    REQUIRE(static_cast<int>(tail.blockMicros.size()) > 6 * blocksPerSecond);

    const std::vector<double> loud(tail.blockMicros.begin(), tail.blockMicros.begin() + blocksPerSecond);
    const std::vector<double> quiet(tail.blockMicros.end() - 5 * blocksPerSecond, tail.blockMicros.end());

    const double loudMedian = median(loud);
    const double quietMedian = median(quiet);
    const double ratio = quietMedian / std::max(loudMedian, 1.0e-9);
    std::cout << "[denormal] block time median: first 1 s " << loudMedian << " us, final 5 s " << quietMedian
              << " us, ratio " << ratio << " (limit 2.0)\n";
    INFO("loud median " << loudMedian << " us, quiet median " << quietMedian << " us, ratio " << ratio);
    REQUIRE(quietMedian <= 2.0 * loudMedian);
}

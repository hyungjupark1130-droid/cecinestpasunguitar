#pragma once

// P1Chain -- the hard-wired P1 signal chain, assembled in ONE place and shared by every headless
// dsp/-only executable that has to render "what the plugin actually plays".
//
//   StringNetwork -> PickupTap -> Oversampler(TriodeStage, bypassable) -> CabFilter (bypassable)
//                 -> OutputGain -> SoftClipLimiter
//
// This is exactly the chain docs/plan.md sections 2.11/2.13 lock and plugin/src/PluginProcessor.cpp
// wires, at the plugin's own shipped defaults (plugin/src/Parameters.cpp) -- see
// makeDefaultP1ChainParams() below for the one place a dsp/ param struct's own default differs
// from what ships.
//
// Why a shared header rather than a copy per executable (Task P1.11). Task P1.10 landed this
// assembly inside tests/bench/BenchMain.cpp as a private `BenchChain`, correctly noting at the
// time that cnpg_bench and cnpg_tests could not share one (cnpg_tests links Catch2, cnpg_bench
// must not). Task P1.11 adds a THIRD consumer, cnpg_render, which is in exactly the same position
// as cnpg_bench -- dsp/-only, no Catch2 -- and whose whole purpose (rendering the corpus for the
// author's listening pass, docs/plan.md section 4.8) is only meaningful if what it renders is
// bit-identical to what the benchmark measures and what the plugin plays. Two hand-maintained
// copies of a six-module chain plus a six-call setParams cascade is precisely the kind of thing
// that silently diverges by one parameter, so the assembly moved here and both executables now
// include it. This header is header-only, JUCE-free and Catch2-free, so it costs nothing to
// include from either target (tests/bench/CMakeLists.txt and tests/render/CMakeLists.txt put
// tests/ on the include path for it).
//
// The per-block setParams cascade is part of the contract, not an implementation detail.
// PluginProcessor::renderChunk() unconditionally re-applies a freshly-read parameter snapshot to
// all six modules via setParams() BEFORE every process() call, every block, regardless of whether
// any value actually changed (docs/plan.md Task P1.1's once-per-block-snapshot design reads the
// APVTS atomics unconditionally). processBlock() below reproduces that exactly. For cnpg_bench
// that cascade is a real, unavoidable per-block cost that must sit inside the timed region (see
// tests/bench/BenchMain.cpp's file-level comment); for cnpg_render it is what makes a per-block
// automation lane (tests/corpus/07_param_sweeps_midnote.json) behave exactly like a host
// automating the same parameter. Both get it from the same code.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"
#include "cnpg/dsp/WaveguideString.h"

#include <cstddef>
#include <vector>

namespace cnpg::test {

// The per-block parameter set P1Chain::processBlock() re-applies every block, standing in for
// plugin/src/Parameters.h's cnpg::params::Snapshot (which cannot be included here -- it is built
// over juce::AudioProcessorValueTreeState, and these executables link cnpg_dsp only). Field-for-
// field the same aggregate, in the same order.
struct P1ChainParams {
    cnpg::dsp::StringNetworkParams network; // includes the nested exciter + material params
    int numStrings = 1;                     // mirrors cnpg::params::Snapshot::numStrings (Task P2.1)
    cnpg::dsp::PickupTapParams pickup;
    cnpg::dsp::TriodeStageParams triode;
    cnpg::dsp::CabFilterParams cab;
    cnpg::dsp::OutputGainParams outputGain;
    cnpg::dsp::SoftClipLimiterParams limiter;
};

// The SHIPPED defaults (matching tests/dsp/MonitoringChainTests.cpp's ChainHarness::setDefaults()
// and plugin/src/Parameters.cpp's APVTS defaults) -- note the one place a dsp/ param struct's own
// default differs from what ships: TriodeStageParams::outputTrimDb defaults to 0 dB ("a trim's
// natural default is no trim", TriodeStage.h), while the plugin's APVTS default -- and this
// aggregate -- set it to kUnityGainOutputTrimDb, which is what makes the chain unity at nominal.
// Getting that one field wrong would mean measuring, and listening to, levels this instrument does
// not actually produce.
inline P1ChainParams makeDefaultP1ChainParams() {
    P1ChainParams params;
    params.network = cnpg::dsp::StringNetworkParams{};
    params.pickup = cnpg::dsp::PickupTapParams{};
    params.triode = cnpg::dsp::TriodeStageParams{};
    params.triode.outputTrimDb = cnpg::dsp::kUnityGainOutputTrimDb;
    params.cab = cnpg::dsp::CabFilterParams{};
    params.outputGain = cnpg::dsp::OutputGainParams{};
    params.limiter = cnpg::dsp::SoftClipLimiterParams{};
    return params;
}

struct P1Chain {
    cnpg::dsp::StringNetwork<float> network;
    cnpg::dsp::PickupTap pickup;
    cnpg::dsp::Oversampler oversampler;
    cnpg::dsp::TriodeStage triode;
    cnpg::dsp::CabFilter cab;
    cnpg::dsp::OutputGain outputGain;
    cnpg::dsp::SoftClipLimiter limiter;

    // The chain's mono working buffer; after processBlock() it holds that block's output samples
    // (the same buffer PluginProcessor::renderChunk() writes the host's channel through).
    std::vector<cnpg::dsp::Sample> mono;

    // Message thread; allocates. Mirrors PluginProcessor::prepareToPlay() module for module,
    // including the oversampled island's own rate/block size (Oversampler.h's wiring note for
    // P1.9: the wrapped nonlinearity runs at factor * sampleRate on blocks of up to
    // maxBlockSize * factor samples).
    void prepare(double sampleRate, int blockSize, int numStrings, int oversampleFactorRequested) {
        network.prepare(sampleRate, blockSize, cnpg::dsp::FractionalDelayKind::Lagrange3);
        network.setNumStrings(numStrings);
        pickup.prepare(sampleRate, blockSize);
        oversampler.prepare(sampleRate, blockSize, oversampleFactorRequested);
        triode.prepare(sampleRate * oversampler.factor(), blockSize * oversampler.factor());
        cab.prepare(sampleRate, blockSize);
        outputGain.prepare(sampleRate, blockSize);
        limiter.prepare(sampleRate, blockSize);

        mono.assign(static_cast<std::size_t>(blockSize), 0.0f);
    }

    void reset() noexcept {
        network.reset();
        pickup.reset();
        oversampler.reset();
        triode.reset();
        cab.reset();
        outputGain.reset();
        limiter.reset();
    }

    // Mirrors PluginProcessor::renderChunk() exactly: the six setParams() calls run first, EVERY
    // block, then the six process() calls. See the file-level comment for why the cascade belongs
    // inside this function rather than being hoisted out of a caller's render loop.
    void processBlock(const P1ChainParams& params, cnpg::dsp::BlockEventQueue& events, int numSamples) noexcept {
        network.setParams(params.network);
        // Applied every block, after setParams and before process, exactly as
        // PluginProcessor::renderChunk() does it -- see there for why that order and not the other.
        network.setNumStrings(params.numStrings);
        pickup.setParams(params.pickup);
        triode.setParams(params.triode);
        cab.setParams(params.cab);
        outputGain.setParams(params.outputGain);
        limiter.setParams(params.limiter);

        network.process(events, numSamples);
        pickup.process(network.tapBuffers(), mono.data(), numSamples);
        oversampler.processWrapped(mono.data(), mono.data(), numSamples,
                                   [this](cnpg::dsp::Sample* buffer, int numUpsampled) noexcept {
                                       triode.process(buffer, buffer, numUpsampled);
                                   });
        cab.process(mono.data(), mono.data(), numSamples);
        outputGain.process(mono.data(), mono.data(), numSamples);
        limiter.process(mono.data(), mono.data(), numSamples); // safety clip LAST
    }
};

} // namespace cnpg::test

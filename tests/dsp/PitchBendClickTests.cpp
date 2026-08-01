#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

// PitchBendClickTests -- docs/plan.md Task P1.5 step 6, verbatim: "render a 5 s MIDI 40 note at
// 48 kHz with pitchBendSemitones driven by a 2 Hz full-depth +/-2 st sine; assert (a) no NaN/Inf
// anywhere in the tap buffer, and (b) excluding the first 50 ms, the maximum absolute
// sample-to-sample difference during the bend never exceeds 4x the maximum observed for the same
// note rendered with static pitch at equal amplitude."
//
// The bend is retargeted once per 128-sample block, as a host would deliver a pitch-wheel stream:
// the click-freedom on trial is StringNetwork's per-sample-smoothed retune path absorbing that
// staircase, not an artificially smooth per-sample input.

namespace {

constexpr double kSampleRate = 48000.0;
constexpr int kBlockSize = 128;
constexpr int kMidiNote = 40;
constexpr double kRenderSeconds = 5.0;
constexpr double kBendHz = 2.0;
constexpr double kExcludeSeconds = 0.05;
constexpr double kClickRatioLimit = 4.0;
constexpr float kPickup = 0.87f;
constexpr double kTwoPi = 6.283185307179586;

std::vector<double> renderNote(double bendDepthSemitones, float lossKnob = 0.5f) {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    params.stringMaterial.lossGainLow = lossKnob;
    params.stringMaterial.lossGainHigh = lossKnob;

    StringNetwork<float> network;
    network.prepare(kSampleRate, kBlockSize, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset();

    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = 0;
    event.stringIndex = 0;
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(kMidiNote);
    event.velocity = 0.8f;
    event.pluckPosition = 0.28f;
    event.hardness = 0.5f;

    BlockEventQueue events;
    events.push(event);

    const auto totalBlocks = static_cast<int>(kRenderSeconds * kSampleRate / kBlockSize);
    std::vector<double> out;
    out.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlockSize));

    for (int block = 0; block < totalBlocks; ++block) {
        const double t = static_cast<double>(block) * static_cast<double>(kBlockSize) / kSampleRate;
        params.pitchBendSemitones = static_cast<float>(bendDepthSemitones * std::sin(kTwoPi * kBendHz * t));
        network.setParams(params);
        network.process(events, kBlockSize);
        const float* channel = network.tapBuffers().channel(0, 0);
        for (int n = 0; n < kBlockSize; ++n)
            out.push_back(static_cast<double>(channel[n]));
    }
    return out;
}

// Peak absolute sample-to-sample difference after `skip` samples, measured on the render
// normalized to unit peak over that same window -- the "at equal amplitude" the criterion asks
// for, since a bent note and a static one do not decay at identical rates.
double normalizedPeakFirstDifference(const std::vector<double>& samples, std::size_t skip, double& peakOut) {
    double peak = 0.0;
    for (std::size_t i = skip; i < samples.size(); ++i)
        peak = std::max(peak, std::fabs(samples[i]));
    peakOut = peak;
    if (peak <= 0.0)
        return 0.0;

    double worst = 0.0;
    for (std::size_t i = skip + 1; i < samples.size(); ++i)
        worst = std::max(worst, std::fabs(samples[i] - samples[i - 1]));
    return worst / peak;
}

} // namespace

TEST_CASE("CONTRACT: StringNetwork global pitch bend is click-free under continuous modulation", "[contract]") {
    const std::vector<double> bent = renderNote(2.0);
    const std::vector<double> still = renderNote(0.0);
    REQUIRE(bent.size() == still.size());

    // (a) no NaN/Inf anywhere in the tap buffer.
    for (std::size_t i = 0; i < bent.size(); ++i) {
        INFO("sample " << i);
        REQUIRE(std::isfinite(bent[i]));
    }

    // (b) the sample-to-sample difference criterion, excluding the first 50 ms.
    const auto skip = static_cast<std::size_t>(kExcludeSeconds * kSampleRate);
    double bentPeak = 0.0;
    double stillPeak = 0.0;
    const double bentDiff = normalizedPeakFirstDifference(bent, skip, bentPeak);
    const double stillDiff = normalizedPeakFirstDifference(still, skip, stillPeak);

    REQUIRE(bentPeak > 0.001); // non-vacuous: both renders really are sounding
    REQUIRE(stillPeak > 0.001);
    REQUIRE(stillDiff > 0.0);

    const double ratio = bentDiff / stillDiff;
    std::cout << "[contract] pitch-bend click metric: bent max |dx| " << bentDiff << " (peak " << bentPeak
              << "), static max |dx| " << stillDiff << " (peak " << stillPeak << "), ratio " << ratio << " (limit "
              << kClickRatioLimit << ")\n";
    INFO("bent " << bentDiff << " vs static " << stillDiff << " -> ratio " << ratio);
    REQUIRE(ratio <= kClickRatioLimit);
}

TEST_CASE("CONTRACT: StringNetwork static pitch bend lands on the bent target", "[contract]") {
    // The other half of step 4: the bend must not merely be smooth, it must be in tune. Same
    // estimator as the [tuning] gate (docs/plan.md section 4.5) and the same +/-2 cent criterion,
    // measured here through the shipping StringNetwork topology rather than the isolated string.
    //
    // The material knobs run at their sustain end for the same reason the [tuning] sweep does:
    // the mandated 2^18-sample window starts 0.5 s after the pluck, and at the default material
    // MIDI 40's tail is far too short to fill it. Same code path, same compensation, only a
    // parameter value inside the shipping range.
    constexpr double kGateCents = 2.0;
    constexpr std::size_t kAnalysisLength = std::size_t{1} << 18;
    const auto discard = static_cast<std::size_t>(0.5 * kSampleRate);

    for (float bend : {-2.0f, 2.0f}) {
        StringNetworkParams params;
        params.pickupPosition01 = kPickup;
        params.pitchBendSemitones = bend;
        params.stringMaterial.lossGainLow = 1.0f;
        params.stringMaterial.lossGainHigh = 1.0f;
        // DECOUPLED BRIDGE (Task P2.4). This case is docs/plan.md section 4.5's "TUNING: static bend
        // accuracy" -- does a constant pitchBendSemitones land the string on the bent target within
        // +/-2 cents? The BRIDGE LOAD pulls partials near its resonance (measured: worst 4.9 cents
        // at MIDI 45 with the shipping admittance, see "TUNING: the coupled topology's residual
        // tuning error" in tests/dsp/BridgePortContractTests.cpp), which would fold a physical
        // effect into a measurement of the bend parameter's arithmetic and gate the wrong thing.
        // Section 4.5 assigns the coupled topology's residual to Task P2.7's calibration table;
        // carry-forward B3 assigns MEASURING it to P2.4, and it is measured -- there, not here.
        params.bridge.couplingStrength = 0.0f;

        StringNetwork<float> network;
        network.prepare(kSampleRate, kBlockSize, FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        network.setParams(params);
        network.reset();

        NoteEvent event{};
        event.type = NoteEventType::NoteOn;
        event.sampleOffset = 0;
        event.stringIndex = 0;
        event.channel = 0;
        event.midiNote = static_cast<std::uint8_t>(kMidiNote);
        event.velocity = 0.8f;
        event.pluckPosition = 0.28f;
        event.hardness = 0.5f;
        BlockEventQueue events;
        events.push(event);

        std::vector<double> tap;
        tap.reserve(kAnalysisLength);
        std::size_t rendered = 0;
        while (tap.size() < kAnalysisLength) {
            network.process(events, kBlockSize);
            const float* channel = network.tapBuffers().channel(0, 0);
            for (int n = 0; n < kBlockSize && tap.size() < kAnalysisLength; ++n, ++rendered)
                if (rendered >= discard)
                    tap.push_back(static_cast<double>(channel[n]));
        }

        const double target = cnpg::test::midiNoteToHz(kMidiNote) * std::exp2(static_cast<double>(bend) / 12.0);
        const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(tap, kSampleRate, kAnalysisLength);
        const double measured = cnpg::test::findPeakHz(spectrum, target, cnpg::test::kTuningSearchCents);
        REQUIRE(measured > 0.0);
        const double cents = cnpg::test::centsBetween(measured, target);
        std::cout << "[contract] static bend " << bend << " st through StringNetwork: target " << target
                  << " Hz, measured " << measured << " Hz (" << cents << " cents)\n";
        INFO("bend " << bend << ": " << cents << " cents");
        REQUIRE(std::fabs(cents) <= kGateCents);
    }
}

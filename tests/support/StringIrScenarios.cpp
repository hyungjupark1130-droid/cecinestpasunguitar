#include "support/StringIrScenarios.h"

#include "support/SpectralAnalysis.h"

#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace cnpg::test {

std::string variantName(cnpg::dsp::FractionalDelayKind kind) {
    return kind == cnpg::dsp::FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1";
}

std::string scenarioFileName(int midiNote) {
    char buffer[128];
    // docs/plan.md section 4.3 example: "midi069_pluck28_tap87".
    std::snprintf(buffer, sizeof(buffer), "midi%03d_pluck28_tap87", midiNote);
    return std::string(buffer);
}

const char* chordChannelName(ChordIrChannel channel) { return channel == ChordIrChannel::Tap ? "tap" : "bridge"; }

namespace {
// One implementation, two captured channels -- so the bridge features can never describe a
// different render from the one the tap golden froze.
std::vector<double> renderStringIrChannel(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote,
                                          bool bridgeChannel) {
    // docs/plan.md section 4.3: "a single-string StringNetwork (1 active string, damper
    // transparent, default BridgeAdmittanceParams)". The damper is transparent because nothing
    // engages it; the bridge admittance is the shipping default, which from Task P2.4 is a LOADED
    // junction (docs/decisions/0006) rather than the rigid termination P1 shipped -- which is why
    // these goldens were regenerated at that task.
    cnpg::dsp::StringNetworkParams params;
    params.pickupPosition01 = kStringIrTapPosition;
    params.stringMaterial = cnpg::dsp::StringMaterialParams{}; // documented defaults
    params.exciter.noiseAmount = kStringIrNoiseAmount;

    cnpg::dsp::StringNetwork<float> network;
    network.prepare(sampleRate, kStringIrBlockSize, kind);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset(); // snaps the pickup smoother onto kStringIrTapPosition for the first sample

    cnpg::dsp::NoteEvent noteOn{};
    noteOn.type = cnpg::dsp::NoteEventType::NoteOn;
    noteOn.sampleOffset = 0;
    noteOn.stringIndex = 0;
    noteOn.channel = 0;
    noteOn.midiNote = static_cast<std::uint8_t>(midiNote);
    noteOn.velocity = kStringIrVelocity;
    noteOn.pluckPosition = kStringIrPluckPosition;
    noteOn.hardness = kStringIrHardness;

    cnpg::dsp::BlockEventQueue events;
    events.push(noteOn);

    const auto count = static_cast<std::size_t>(kStringIrSeconds * sampleRate);
    std::vector<double> out;
    out.reserve(count);
    while (out.size() < count) {
        const auto wanted = static_cast<int>(std::min<std::size_t>(kStringIrBlockSize, count - out.size()));
        network.process(events, wanted);
        const float* channel = bridgeChannel ? network.bridgeOutputBuffer() : network.tapBuffers().channel(0, 0);
        for (int n = 0; n < wanted; ++n)
            out.push_back(static_cast<double>(channel[n]));
    }
    return out;
}
} // namespace

std::vector<double> renderStringIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    return renderStringIrChannel(kind, sampleRate, midiNote, false);
}

std::vector<double> renderStringIrBridge(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    return renderStringIrChannel(kind, sampleRate, midiNote, true);
}

std::vector<double> renderChordIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, ChordIrChannel channel) {
    // docs/plan.md section 4.3: "A sixth scenario captures the full 6-string network playing one
    // open E-major chord (tests bridge coupling regression)." Everything except the note set and
    // the string count is the single-string recipe, so the two scenarios differ in exactly the
    // thing under test.
    cnpg::dsp::StringNetworkParams params;
    params.pickupPosition01 = kChordIrTapPosition;
    params.stringMaterial = cnpg::dsp::StringMaterialParams{}; // documented defaults
    params.exciter.noiseAmount = kStringIrNoiseAmount;
    // The SHIPPING bridge admittance, left at its struct default on purpose: this scenario exists
    // to freeze the coupled behaviour, so it must be rendered at the coupling that ships.

    cnpg::dsp::StringNetwork<float> network;
    network.prepare(sampleRate, kStringIrBlockSize, kind);
    network.setNumStrings(static_cast<int>(kChordIrNotes.size()));
    network.setParams(params);
    network.reset();

    cnpg::dsp::BlockEventQueue events;
    for (std::size_t s = 0; s < kChordIrNotes.size(); ++s) {
        cnpg::dsp::NoteEvent noteOn{};
        noteOn.type = cnpg::dsp::NoteEventType::NoteOn;
        // Strummed, not struck: 6 ms between strings at 48 kHz, scaled with the rate so the gesture
        // is the same gesture everywhere rather than the same sample count.
        noteOn.sampleOffset = static_cast<int>(static_cast<double>(s) * 0.006 * sampleRate);
        noteOn.stringIndex = static_cast<std::uint8_t>(s);
        noteOn.channel = 0;
        noteOn.midiNote = static_cast<std::uint8_t>(kChordIrNotes[s]);
        noteOn.velocity = kStringIrVelocity;
        noteOn.pluckPosition = kStringIrPluckPosition;
        noteOn.hardness = kStringIrHardness;
        events.push(noteOn);
    }

    const auto count = static_cast<std::size_t>(kStringIrSeconds * sampleRate);
    std::vector<double> out;
    out.reserve(count);
    while (out.size() < count) {
        const auto wanted = static_cast<int>(std::min<std::size_t>(kStringIrBlockSize, count - out.size()));
        network.process(events, wanted);
        if (channel == ChordIrChannel::Bridge) {
            const float* bridge = network.bridgeOutputBuffer();
            for (int n = 0; n < wanted; ++n)
                out.push_back(static_cast<double>(bridge[n]));
        } else {
            const int strings = network.tapBuffers().numStrings();
            for (int n = 0; n < wanted; ++n) {
                double sum = 0.0;
                for (int s = 0; s < strings; ++s) {
                    const float* tap = network.tapBuffers().channel(s, 0);
                    if (tap != nullptr && network.tapBuffers().isActive(s))
                        sum += static_cast<double>(tap[n]);
                }
                out.push_back(sum);
            }
        }
    }
    return out;
}

StringIrFeatures extractChordIrFeatures(const std::vector<double>& samples, double sampleRate) {
    StringIrFeatures features;
    features.attackRmsDbfs = rmsDbfs(samples, sampleRate, 0.1);
    features.partialHz.assign(8, 0.0); // see the header: not well posed for a six-note chord
    features.bandT60.reserve(kStringIrT60Bands.size());
    for (double centre : kStringIrT60Bands)
        features.bandT60.push_back(bandT60Seconds(samples, sampleRate, centre));
    return features;
}

StringIrFeatures extractStringIrFeatures(const std::vector<double>& samples, double sampleRate, int midiNote) {
    StringIrFeatures features;
    features.attackRmsDbfs = rmsDbfs(samples, sampleRate, 0.1);

    // One spectrum, eight partial searches -- computing a fresh FFT per partial would multiply
    // the suite's runtime by eight for no extra information.
    const Spectrum spectrum = computeSpectrum(samples, sampleRate);
    const double f0 = midiNoteToHz(midiNote);
    features.partialHz.assign(8, 0.0);
    for (int k = 1; k <= 8; ++k) {
        const double target = f0 * static_cast<double>(k);
        if (target >= 0.45 * sampleRate)
            continue;
        // Partials are dispersion-stretched, so a wider window than the fundamental's +/-80
        // cents is needed; +/-100 cents still cannot reach the neighbouring partial for k <= 8.
        features.partialHz[static_cast<std::size_t>(k - 1)] = findPeakHz(spectrum, target, 100.0);
    }

    features.bandT60.reserve(kStringIrT60Bands.size());
    for (double centre : kStringIrT60Bands)
        features.bandT60.push_back(bandT60Seconds(samples, sampleRate, centre));

    return features;
}

} // namespace cnpg::test

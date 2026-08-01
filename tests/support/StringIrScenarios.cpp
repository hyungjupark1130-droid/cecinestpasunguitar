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

std::vector<double> renderStringIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    // docs/plan.md section 4.3: "a single-string StringNetwork (1 active string, damper
    // transparent, default BridgeAdmittanceParams)". The damper is transparent because
    // DamperJunction does not exist yet (P2.2) and the bridge admittance is at its default
    // because P1's port is the rigid termination (P2.4 loads it) -- both of which is what
    // "transparent" and "default" mean at this point in the plan, not a deviation from it.
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
        const float* channel = network.tapBuffers().channel(0, 0);
        for (int n = 0; n < wanted; ++n)
            out.push_back(static_cast<double>(channel[n]));
    }
    return out;
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

#include "support/StringIrScenarios.h"

#include "support/SpectralAnalysis.h"

#include "cnpg/dsp/PluckExciter.h"

#include <cmath>
#include <cstdio>

namespace cnpg::test {

std::string variantName(cnpg::dsp::FractionalDelayKind kind) {
    return kind == cnpg::dsp::FractionalDelayKind::Lagrange3 ? "lagrange3" : "thiran1";
}

std::string scenarioFileName(int midiNote) {
    char buffer[64];
    // docs/plan.md section 4.3 example: "midi069_pluck28_tap87".
    std::snprintf(buffer, sizeof(buffer), "midi%03d_pluck28_tap87", midiNote);
    return std::string(buffer);
}

std::vector<double> renderStringIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    cnpg::dsp::WaveguideString<float> string;
    string.prepare(sampleRate, 512, kind);

    cnpg::dsp::WaveguideStringParams params;
    params.f0Hz = static_cast<float>(midiNoteToHz(midiNote));
    params.bendSemitones = 0.0f;
    params.material = cnpg::dsp::StringMaterialParams{}; // documented defaults
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset();

    cnpg::dsp::PluckExciter<float> exciter;
    exciter.prepare(sampleRate, 512);
    cnpg::dsp::PluckExciterParams exciterParams;
    exciterParams.noiseAmount = kStringIrNoiseAmount;
    exciter.setParams(exciterParams);
    exciter.trigger(kStringIrVelocity, kStringIrPluckPosition, kStringIrHardness);

    const auto count = static_cast<std::size_t>(kStringIrSeconds * sampleRate);
    std::vector<double> out(count);
    for (std::size_t n = 0; n < count; ++n) {
        const float excitation = exciter.renderSample();
        if (excitation != 0.0f)
            string.injectAt(exciter.latchedPosition01(), excitation);
        out[n] = static_cast<double>(string.readTapAt(kStringIrTapPosition));
        string.tick();
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

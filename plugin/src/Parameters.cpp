#include "Parameters.h"

namespace cnpg::params {

namespace {

// AudioUnit parameter-ordering version hint (docs/plan.md Task P1.1: "parameter IDs"); every
// P1 parameter ships in the same release, so every one uses the same hint.
constexpr int kParameterVersionHint = 1;

juce::ParameterID makeParameterID(const char* id) { return juce::ParameterID(id, kParameterVersionHint); }

} // namespace

juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout() {
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    const juce::NormalisableRange<float> unitRange(0.0f, 1.0f);
    const juce::NormalisableRange<float> trimDbRange(-24.0f, 24.0f);

    // Task P1.9 gain staging. The pickup trim is the stage that lands a single string on the
    // -18 dBFS per-string nominal (docs/plan.md Task P1.9 step 3), and the calibrated value is
    // measured, not chosen: cnpg::dsp::kNominalPickupTrimDb (see PickupTap.h for the measurement
    // and the exact reference scenario). Both the default AND the range's centre are read from
    // that one constant, so the plugin's default state IS the calibrated state, the user keeps a
    // symmetric +/-24 dB of trim around it, and the dsp-side gate test
    // (tests/dsp/MonitoringChainTests.cpp, which drives PickupTapParams{} directly) cannot drift
    // apart from what the plugin actually ships.
    const float pickupTrimDefaultDb = cnpg::dsp::PickupTapParams{}.outputGainDb;
    const juce::NormalisableRange<float> pickupTrimDbRange(pickupTrimDefaultDb - 24.0f, pickupTrimDefaultDb + 24.0f);

    // Exciter
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterDefaultPosition),
                                                           "Exciter Position", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterDefaultHardness),
                                                           "Exciter Hardness", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterNoiseAmount), "Exciter Noise",
                                                           unitRange, 0.0f));

    // Material
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::materialLossGainLow),
                                                           "Material Loss Low", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::materialLossGainHigh),
                                                           "Material Loss High", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::materialDispersionAmount),
                                                           "Material Dispersion", unitRange, 0.0f));

    // Pickup
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupResonanceHz), "Pickup Resonance",
                                                           juce::NormalisableRange<float>(100.0f, 8000.0f, 1.0f, 0.4f),
                                                           2500.0f,
                                                           juce::AudioParameterFloatAttributes().withLabel("Hz")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupQ), "Pickup Q",
                                                           juce::NormalisableRange<float>(0.1f, 10.0f), 2.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupOutputGainDb),
                                                           "Pickup Output Gain", pickupTrimDbRange, pickupTrimDefaultDb,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupPosition01), "Pickup Position",
                                                           unitRange, 0.5f));

    // Triode
    layout.add(
        std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::triodeDrive), "Triode Drive", unitRange, 0.5f));
    // Task P1.9 gain staging, part two. TriodeStage's small-signal gain in the SAMPLE domain is
    // kGridVoltsPerFullScale at the default drive (+13.98 dB) -- see the block comment above
    // cnpg::dsp::kUnityGainOutputTrimDb in TriodeStage.h. This trim takes exactly that back out,
    // so a single string arriving at the -18 dBFS per-string nominal leaves the triode at
    // -18 dBFS too and the whole ~+16 dB summing budget still fits under the limiter ceiling.
    // Derived from the circuit constant, not measured and rounded, and centred on like the pickup
    // trim so the user keeps a symmetric +/-24 dB either side of the calibrated value.
    const juce::NormalisableRange<float> triodeTrimDbRange(cnpg::dsp::kUnityGainOutputTrimDb - 24.0f,
                                                           cnpg::dsp::kUnityGainOutputTrimDb + 24.0f);
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        makeParameterID(ID::triodeOutputTrimDb), "Triode Output Trim", triodeTrimDbRange,
        cnpg::dsp::kUnityGainOutputTrimDb, juce::AudioParameterFloatAttributes().withLabel("dB")));
    layout.add(std::make_unique<juce::AudioParameterBool>(makeParameterID(ID::triodeBypass), "Triode Bypass", false));

    // Monitoring chain
    layout.add(std::make_unique<juce::AudioParameterBool>(makeParameterID(ID::cabBypass), "Cab Bypass", false));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::limiterCeilingDb), "Limiter Ceiling",
                                                           juce::NormalisableRange<float>(-12.0f, 0.0f), -0.3f,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::outputGainDb), "Output Gain",
                                                           trimDbRange, 0.0f,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));

    // Global
    layout.add(std::make_unique<juce::AudioParameterChoice>(makeParameterID(ID::retriggerMode), "Retrigger Mode",
                                                            juce::StringArray{"Physical", "Synth"}, 0));

    return layout;
}

RawParameterPointers collectRawParameterPointers(const juce::AudioProcessorValueTreeState& apvts) {
    RawParameterPointers params;

    params.exciterDefaultPosition = apvts.getRawParameterValue(ID::exciterDefaultPosition);
    params.exciterDefaultHardness = apvts.getRawParameterValue(ID::exciterDefaultHardness);
    params.exciterNoiseAmount = apvts.getRawParameterValue(ID::exciterNoiseAmount);

    params.materialLossGainLow = apvts.getRawParameterValue(ID::materialLossGainLow);
    params.materialLossGainHigh = apvts.getRawParameterValue(ID::materialLossGainHigh);
    params.materialDispersionAmount = apvts.getRawParameterValue(ID::materialDispersionAmount);

    params.pickupResonanceHz = apvts.getRawParameterValue(ID::pickupResonanceHz);
    params.pickupQ = apvts.getRawParameterValue(ID::pickupQ);
    params.pickupOutputGainDb = apvts.getRawParameterValue(ID::pickupOutputGainDb);
    params.pickupPosition01 = apvts.getRawParameterValue(ID::pickupPosition01);

    params.triodeDrive = apvts.getRawParameterValue(ID::triodeDrive);
    params.triodeOutputTrimDb = apvts.getRawParameterValue(ID::triodeOutputTrimDb);
    params.triodeBypass = apvts.getRawParameterValue(ID::triodeBypass);

    params.cabBypass = apvts.getRawParameterValue(ID::cabBypass);
    params.limiterCeilingDb = apvts.getRawParameterValue(ID::limiterCeilingDb);
    params.outputGainDb = apvts.getRawParameterValue(ID::outputGainDb);

    params.retriggerMode = apvts.getRawParameterValue(ID::retriggerMode);

    return params;
}

Snapshot snapshotParameters(const RawParameterPointers& params) noexcept {
    Snapshot snapshot;

    snapshot.stringNetwork.exciter.defaultPosition = params.exciterDefaultPosition->load();
    snapshot.stringNetwork.exciter.defaultHardness = params.exciterDefaultHardness->load();
    snapshot.stringNetwork.exciter.noiseAmount = params.exciterNoiseAmount->load();

    snapshot.stringNetwork.material.lossGainLow = params.materialLossGainLow->load();
    snapshot.stringNetwork.material.lossGainHigh = params.materialLossGainHigh->load();
    snapshot.stringNetwork.material.dispersionAmount = params.materialDispersionAmount->load();

    snapshot.stringNetwork.pickupPosition01 = params.pickupPosition01->load();
    // AudioParameterChoice reports its selected index as a float via getRawParameterValue();
    // index 0 -> Physical, 1 -> Synth (matches the choices list in createParameterLayout()).
    snapshot.stringNetwork.retriggerMode =
        params.retriggerMode->load() >= 0.5f ? cnpg::dsp::RetriggerMode::Synth : cnpg::dsp::RetriggerMode::Physical;

    snapshot.pickup.resonanceHz = params.pickupResonanceHz->load();
    snapshot.pickup.q = params.pickupQ->load();
    snapshot.pickup.outputGainDb = params.pickupOutputGainDb->load();

    snapshot.triode.drive = params.triodeDrive->load();
    snapshot.triode.outputTrimDb = params.triodeOutputTrimDb->load();
    // AudioParameterBool reports 0.0f/1.0f via getRawParameterValue().
    snapshot.triode.bypass = params.triodeBypass->load() >= 0.5f;

    snapshot.cab.bypass = params.cabBypass->load() >= 0.5f;

    snapshot.limiter.ceilingDb = params.limiterCeilingDb->load();

    snapshot.outputGain.gainDb = params.outputGainDb->load();

    return snapshot;
}

} // namespace cnpg::params

#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

// cnpg_plugin's top-level juce::AudioProcessor. Owns the frozen bus layout, the state-version
// scaffold, and the P0 walking-skeleton sine-tone proof. Plugin identity (PRODUCT_NAME,
// COMPANY_NAME, PLUGIN_MANUFACTURER_CODE Hjpk, PLUGIN_CODE Cnpg) is set in plugin/CMakeLists.txt,
// pinned there permanently, and is not repeated here. Real dsp wiring (APVTS, PluckExciter,
// StringNetwork, PickupTap, ...) starts landing in P1.1 -- see docs/plan.md sections 1.4 and 2.
class PluginProcessor final : public juce::AudioProcessor {
  public:
    PluginProcessor();
    ~PluginProcessor() override = default;

    //==============================================================================
    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

    // Frozen from P0 (docs/plan.md section 1.4): main out == stereo AND sidechain in
    // {AudioChannelSet::disabled(), AudioChannelSet::stereo()}; every other layout is rejected.
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;

    // Locked: no VST3 double-precision path in v1 (docs/plan.md section 1.4; pinned P0.4).
    bool supportsDoublePrecisionProcessing() const override;

    void processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages) override;
    using AudioProcessor::processBlock;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;
    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram(int index) override;
    const juce::String getProgramName(int index) override;
    void changeProgramName(int index, const juce::String& newName) override;

    //==============================================================================
    // XML root carries integer attribute cnpgStateVersion; unknown or missing versions are
    // rejected to defaults. This is a P0 scaffold only -- full APVTS-backed state arrives in
    // P1.1 and replaces this blob entirely (docs/plan.md Task P1.1 step 4).
    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    static constexpr int kCnpgStateVersion = 1;

  private:
    // P0 walking-skeleton sine-tone proof only (docs/plan.md Task P0.4 step 3): a continuous
    // 440 Hz tone at -18 dBFS peak, duplicated mono to every output channel. Replaced by real
    // excitation/string/pickup dsp wiring starting P1.1.
    double currentSampleRate_ = 44100.0;
    double sinePhase_ = 0.0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PluginProcessor)
};

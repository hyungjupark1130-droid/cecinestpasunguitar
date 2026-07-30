#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include "Parameters.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"

// cnpg_plugin's top-level juce::AudioProcessor. Owns the frozen bus layout, the APVTS-backed
// state (Task P1.1), and the P0 walking-skeleton sine-tone proof (now gain-staged through the
// real cnpg::dsp::OutputGain module, driven by the APVTS outputGainDb parameter). Plugin
// identity (PRODUCT_NAME, COMPANY_NAME, PLUGIN_MANUFACTURER_CODE Hjpk, PLUGIN_CODE Cnpg) is set
// in plugin/CMakeLists.txt, pinned there permanently, and is not repeated here. The remaining
// dsp wiring (PluckExciter, StringNetwork, PickupTap, ...) lands incrementally starting P1.2 --
// see docs/plan.md sections 1.4 and 2.
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
    // APVTS-backed state (docs/plan.md Task P1.1 step 4): getStateInformation() writes the
    // APVTS's own ValueTree as XML with an added integer attribute cnpgStateVersion;
    // setStateInformation() rejects to defaults on a root-tag mismatch (also how the P0.4
    // scaffold blob -- root tag "CNPG_PLUGIN_STATE", not the APVTS's "PARAMETERS" -- is safely
    // rejected: no session-compatibility guarantee before P5, docs/plan.md Global Constraints)
    // or an unrecognized version.
    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    // Bumped from the P0.4 scaffold's 1: the APVTS-backed schema landing in P1.1 is a different,
    // incompatible shape from the P0 XML blob, even though the root-tag check above already
    // rejects the old blob on its own.
    static constexpr int kCnpgStateVersion = 2;

  private:
    // Initialized in the constructor's member-initializer list (PluginProcessor.cpp), matching
    // this class's existing convention of keeping juce::AudioProcessor base-class construction
    // arguments there rather than as an in-class default member initializer.
    juce::AudioProcessorValueTreeState apvts;

    // Cached once, after apvts exists, in the constructor body: valid for apvts's lifetime,
    // which is the same as this processor's (docs/plan.md: APVTS "must have the same lifetime
    // as the processor"). Reading through these pointers on the audio thread is the "trivial
    // atomic-read adapter" the once-per-block snapshot relies on (docs/plan.md Task P1.1
    // acceptance criteria) -- no getRawParameterValue() string lookups on the audio thread.
    cnpg::params::RawParameterPointers rawParams_;

    // First real dsp/ module wired into processBlock (docs/plan.md Task P1.1): the P0
    // walking-skeleton sine-tone proof (docs/plan.md Task P0.4 step 3, still a continuous
    // 440 Hz tone at -18 dBFS peak before this gain stage) is now gain-staged through it, driven
    // by the APVTS outputGainDb parameter every block. The remaining P1 chain (PluckExciter,
    // StringNetwork, PickupTap, TriodeStage, CabFilter, SoftClipLimiter) lands starting P1.2 and
    // eventually replaces the sine tone entirely (docs/plan.md Task P1.9).
    cnpg::dsp::OutputGain outputGain_;

    double currentSampleRate_ = 44100.0;
    double sinePhase_ = 0.0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PluginProcessor)
};

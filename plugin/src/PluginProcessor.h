#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include "MidiConverter.h"
#include "Parameters.h"
#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/NoteAllocator.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"

// cnpg_plugin's top-level juce::AudioProcessor. Owns the frozen bus layout, the APVTS-backed state
// (Task P1.1), and -- from Task P1.9 -- the whole hard-wired P1 instrument chain. The P0.4
// walking-skeleton sine tone is gone: processBlock now renders real plucked-string audio driven by
// host MIDI. Plugin identity (PRODUCT_NAME, COMPANY_NAME, PLUGIN_MANUFACTURER_CODE Hjpk,
// PLUGIN_CODE Cnpg) is set in plugin/CMakeLists.txt, pinned there permanently, and is not repeated
// here.
//
// -----------------------------------------------------------------------------------------------
// The locked chain (docs/plan.md sections 2.11 and 2.13).
// -----------------------------------------------------------------------------------------------
//
//   host MIDI -> MidiConverter -> NoteAllocator -> BlockEventQueue
//             -> StringNetwork -> PickupTap -> Oversampler(TriodeStage, bypassable)
//             -> CabFilter (bypassable) -> OutputGain -> SoftClipLimiter -> mono duplicated to the
//                stereo bus
//
// The safety clip is LAST by design, so "rendered peak <= ceilingDb" is enforceable regardless of
// what any stage ahead of it does. The order is hard-wired rather than routed: cnpg::dsp::ModuleGraph
// exists and is tested (tests/dsp/ModuleGraphTests.cpp proves routing this exact chain through it is
// sample-identical to these direct calls), but runtime routing does not turn on until P4.

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

    // Stays 1 through all of P1 (docs/plan.md Task P2.1 is what bumps this to 2, and keeps it 2
    // through all of P2; P2.1's own acceptance criterion requires loading a genuine version-1 P1
    // state without crashing, which requires a version-1 P1 state to actually exist). The
    // root-tag check above already rejects the P0.4 XML scaffold blob on its own, regardless of
    // this integer, since that blob's root tag ("CNPG_PLUGIN_STATE") never matches the APVTS's
    // ("PARAMETERS").
    static constexpr int kCnpgStateVersion = 1;

  private:
    // One chunk of at most preparedBlockSize_ samples through the whole chain, writing
    // `numSamples` samples into `output`. See processBlock() for why the chunking exists.
    void renderChunk(const cnpg::params::Snapshot& snapshot, const cnpg::dsp::RawMidiEvent* events, int numEvents,
                     float* output, int numSamples) noexcept;

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

    // The chain, in signal order. Every one of these is prepared in prepareToPlay() and driven
    // from the once-per-block APVTS snapshot in processBlock().
    cnpg::dsp::NoteAllocator noteAllocator_;
    cnpg::dsp::StringNetwork<float> stringNetwork_;
    cnpg::dsp::PickupTap pickupTap_;
    cnpg::dsp::Oversampler oversampler_;
    cnpg::dsp::TriodeStage triode_;
    cnpg::dsp::CabFilter cabFilter_;
    cnpg::dsp::OutputGain outputGain_;
    cnpg::dsp::SoftClipLimiter limiter_;

    // Realtime scratch, all sized on the message thread in prepareToPlay().
    cnpg::dsp::RawMidiEvent rawMidiEvents_[cnpg::midi::kMaxEventsPerBlock]{};
    cnpg::dsp::BlockEventQueue noteEvents_;

    double currentSampleRate_ = 44100.0;
    int preparedBlockSize_ = 0;

    // Global pitch bend in semitones, held across blocks: the MIDI pitch wheel is a latched
    // controller, not an event the string consumes, so its last value has to survive until the
    // host sends another one. Not an APVTS parameter by design (see StringNetworkParams).
    float pitchBendSemitones_ = 0.0f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PluginProcessor)
};

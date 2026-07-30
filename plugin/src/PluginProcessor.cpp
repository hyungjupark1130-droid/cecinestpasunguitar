#include "PluginProcessor.h"

#include <cmath>

namespace {

// P0 walking-skeleton sine-tone proof (docs/plan.md Task P0.4 step 3). This is a fixed test
// stimulus wired directly into the plugin shell, not a dsp/ module -- it is gain-staged through
// cnpg::dsp::OutputGain starting P1.1, and is replaced entirely once the real
// excitation/string/pickup/triode chain lands (docs/plan.md Task P1.9).
constexpr double kSineFrequencyHz = 440.0;
constexpr float kSineLevelDbfs = -18.0f;

float dbToLinear(float dB) noexcept { return std::pow(10.0f, dB / 20.0f); }

const float kSineAmplitudeLinear = dbToLinear(kSineLevelDbfs);

// APVTS-backed state (docs/plan.md Task P1.1 step 4): the attribute name is the only piece of
// the P0.4 scaffold that survives -- the root tag is now the APVTS's own ValueTree type
// ("PARAMETERS", set at construction below), not this scaffold's "CNPG_PLUGIN_STATE".
const juce::Identifier kStateVersionAttribute("cnpgStateVersion");

} // namespace

//==============================================================================
PluginProcessor::PluginProcessor()
    : juce::AudioProcessor(juce::AudioProcessor::BusesProperties()
                               .withOutput("Output", juce::AudioChannelSet::stereo(), true)
                               .withInput("Sidechain", juce::AudioChannelSet::stereo(), false)),
      apvts(*this, nullptr, "PARAMETERS", cnpg::params::createParameterLayout()) {
    // Message thread, after apvts finishes constructing: see the RawParameterPointers doc
    // comment in Parameters.h for why caching these once here (rather than looking parameters
    // up by ID on the audio thread) is required for the once-per-block snapshot to be
    // realtime-safe.
    rawParams_ = cnpg::params::collectRawParameterPointers(apvts);
}

void PluginProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
    currentSampleRate_ = sampleRate;
    sinePhase_ = 0.0;
    outputGain_.prepare(sampleRate, samplesPerBlock);
}

void PluginProcessor::releaseResources() {}

bool PluginProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const {
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    const auto sidechain = layouts.getMainInputChannelSet();
    return sidechain == juce::AudioChannelSet::disabled() || sidechain == juce::AudioChannelSet::stereo();
}

bool PluginProcessor::supportsDoublePrecisionProcessing() const {
    // Locked: no VST3 double-precision path in v1 (docs/plan.md section 1.4; pinned P0.4).
    return false;
}

void PluginProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages) {
    juce::ignoreUnused(midiMessages);
    // Instantiated first, per docs/plan.md Task P1.1 step 3: every dsp/ call below this point
    // (currently just OutputGain::process) runs with FTZ/DAZ engaged. Replaces
    // juce::ScopedNoDenormals with the dsp-side, headless-testable equivalent (docs/plan.md
    // section 2 file tree: "there is no plugin DenormalGuard.h").
    const cnpg::dsp::ScopedFtzDazGuard ftzDazGuard;

    // Once-per-block APVTS snapshot (docs/plan.md Task P1.1 step 2): a trivial atomic-read
    // adapter, no allocation or locking. Only outputGain is consumed by a real dsp/ module so
    // far -- the rest becomes live as PluckExciter/StringNetwork/PickupTap/TriodeStage/
    // CabFilter/SoftClipLimiter land starting P1.2 (docs/plan.md Task P1.9 wires the full
    // chain).
    const cnpg::params::Snapshot snapshot = cnpg::params::snapshotParameters(rawParams_);

    const int numSamples = buffer.getNumSamples();
    const int numOutputChannels = getTotalNumOutputChannels();

    if (numOutputChannels <= 0)
        return;

    const double phaseIncrement = juce::MathConstants<double>::twoPi * kSineFrequencyHz / currentSampleRate_;

    auto* firstChannel = buffer.getWritePointer(0);
    double phase = sinePhase_;

    for (int i = 0; i < numSamples; ++i) {
        firstChannel[i] = static_cast<float>(std::sin(phase)) * kSineAmplitudeLinear;

        phase += phaseIncrement;
        if (phase >= juce::MathConstants<double>::twoPi)
            phase -= juce::MathConstants<double>::twoPi;
    }

    sinePhase_ = phase;

    // Gain-stage the sine tone through the real OutputGain module, retargeted from this block's
    // snapshot every call before process() runs (docs/plan.md Task P1.1 known ledger item: this
    // ordering closes the P0.3 gap where a host re-prepare's unity reset could otherwise survive
    // for a whole block). Default outputGainDb is 0 dB (unity), so the P0.5/P0.8 -18 dBFS
    // reference level is unchanged unless the parameter is moved.
    outputGain_.setParams(snapshot.outputGain);
    outputGain_.process(firstChannel, firstChannel, numSamples);

    // Duplicate mono to every remaining output channel (docs/plan.md Task P0.4 step 3).
    for (int channel = 1; channel < numOutputChannels; ++channel)
        buffer.copyFrom(channel, 0, firstChannel, numSamples);
}

//==============================================================================
juce::AudioProcessorEditor* PluginProcessor::createEditor() { return new juce::GenericAudioProcessorEditor(*this); }

bool PluginProcessor::hasEditor() const { return true; }

//==============================================================================
const juce::String PluginProcessor::getName() const { return JucePlugin_Name; }

bool PluginProcessor::acceptsMidi() const {
#if JucePlugin_WantsMidiInput
    return true;
#else
    return false;
#endif
}

bool PluginProcessor::producesMidi() const {
#if JucePlugin_ProducesMidiOutput
    return true;
#else
    return false;
#endif
}

bool PluginProcessor::isMidiEffect() const {
#if JucePlugin_IsMidiEffect
    return true;
#else
    return false;
#endif
}

double PluginProcessor::getTailLengthSeconds() const { return 0.0; }

//==============================================================================
int PluginProcessor::getNumPrograms() {
    // Hosts don't all cope well with 0 programs, so this is 1 even without real program support.
    return 1;
}

int PluginProcessor::getCurrentProgram() { return 0; }

void PluginProcessor::setCurrentProgram(int index) { juce::ignoreUnused(index); }

const juce::String PluginProcessor::getProgramName(int index) {
    juce::ignoreUnused(index);
    return {};
}

void PluginProcessor::changeProgramName(int index, const juce::String& newName) { juce::ignoreUnused(index, newName); }

//==============================================================================
void PluginProcessor::getStateInformation(juce::MemoryBlock& destData) {
    const juce::ValueTree state = apvts.copyState();
    const std::unique_ptr<juce::XmlElement> xml(state.createXml());
    xml->setAttribute(kStateVersionAttribute, kCnpgStateVersion);
    copyXmlToBinary(*xml, destData);
}

void PluginProcessor::setStateInformation(const void* data, int sizeInBytes) {
    const std::unique_ptr<juce::XmlElement> xml(getXmlFromBinary(data, sizeInBytes));

    // Also how a P0.4-scaffold blob is safely rejected: its root tag is "CNPG_PLUGIN_STATE",
    // never apvts.state.getType() ("PARAMETERS"), so it falls through to defaults here without
    // reading getIntAttribute at all (no session-compatibility guarantee before P5, docs/plan.md
    // Global Constraints).
    if (xml == nullptr || !xml->hasTagName(apvts.state.getType()))
        return; // malformed, foreign, or pre-APVTS blob: reject to defaults

    if (xml->getIntAttribute(kStateVersionAttribute, -1) != kCnpgStateVersion)
        return; // unknown version: reject to defaults (docs/plan.md Task P0.4 step 4)

    apvts.replaceState(juce::ValueTree::fromXml(*xml));
}

//==============================================================================
// JUCE's plugin-client wrappers (VST3, Standalone, ...) call this factory to instantiate the
// processor; every format's entry point ultimately routes through it.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new PluginProcessor(); }

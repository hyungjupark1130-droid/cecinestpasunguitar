#include "PluginProcessor.h"

#include <cmath>

namespace {

// P0 walking-skeleton sine-tone proof (docs/plan.md Task P0.4 step 3). This is a fixed test
// stimulus wired directly into the plugin shell, not a dsp/ module -- it is replaced once real
// excitation/string/pickup dsp lands starting P1.1.
constexpr double kSineFrequencyHz = 440.0;
constexpr float kSineLevelDbfs = -18.0f;

float dbToLinear(float dB) noexcept { return std::pow(10.0f, dB / 20.0f); }

const float kSineAmplitudeLinear = dbToLinear(kSineLevelDbfs);

// State-version XML scaffold (docs/plan.md Task P0.4 step 4). Replaced by APVTS-backed state
// in P1.1; the root tag name and attribute name are internal to this scaffold and do not need
// to survive that migration.
const juce::Identifier kStateRootTag("CNPG_PLUGIN_STATE");
const juce::Identifier kStateVersionAttribute("cnpgStateVersion");

} // namespace

//==============================================================================
PluginProcessor::PluginProcessor()
    : juce::AudioProcessor(juce::AudioProcessor::BusesProperties()
                               .withOutput("Output", juce::AudioChannelSet::stereo(), true)
                               .withInput("Sidechain", juce::AudioChannelSet::stereo(), false)) {}

void PluginProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
    juce::ignoreUnused(samplesPerBlock);
    currentSampleRate_ = sampleRate;
    sinePhase_ = 0.0;
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
    juce::ScopedNoDenormals noDenormals;

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
    juce::XmlElement xml(kStateRootTag);
    xml.setAttribute(kStateVersionAttribute, kCnpgStateVersion);
    copyXmlToBinary(xml, destData);
}

void PluginProcessor::setStateInformation(const void* data, int sizeInBytes) {
    const std::unique_ptr<juce::XmlElement> xml(getXmlFromBinary(data, sizeInBytes));

    if (xml == nullptr || !xml->hasTagName(kStateRootTag))
        return; // malformed or foreign blob: reject to defaults

    if (xml->getIntAttribute(kStateVersionAttribute, -1) != kCnpgStateVersion)
        return; // unknown version: reject to defaults (docs/plan.md Task P0.4 step 4)

    // Nothing else to restore yet -- full APVTS-backed state arrives in P1.1.
}

//==============================================================================
// JUCE's plugin-client wrappers (VST3, Standalone, ...) call this factory to instantiate the
// processor; every format's entry point ultimately routes through it.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new PluginProcessor(); }

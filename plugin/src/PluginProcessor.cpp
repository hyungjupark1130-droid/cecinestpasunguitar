#include "PluginProcessor.h"

#include <algorithm>

#include "cnpg/dsp/MidiTranslation.h"

namespace {

// APVTS-backed state (docs/plan.md Task P1.1 step 4): the attribute name is the only piece of
// the P0.4 scaffold that survives -- the root tag is now the APVTS's own ValueTree type
// ("PARAMETERS", set at construction below), not this scaffold's "CNPG_PLUGIN_STATE".
const juce::Identifier kStateVersionAttribute("cnpgStateVersion");

// P1 is the single-string vertical slice (docs/plan.md Task P1.5 "P1 SCOPE"): StringNetwork
// preallocates all kMaxStrings strings but runs one, and NoteAllocator targets that one. P2.1 is
// the scale-out.
constexpr int kP1NumStrings = 1;

// MIDI status nibbles this file reads directly. Note on/off and CC are interpreted dsp-side by
// cnpg::dsp::NoteAllocator; the pitch wheel is NOT a note event -- it is a latched global
// controller feeding StringNetworkParams::pitchBendSemitones -- so it is the one message the
// plugin layer picks out of the raw stream itself, via cnpg::dsp::pitchWheelToSemitones (which is
// where the actual interpretation lives, keeping the mapping JUCE-free and headless-testable).
constexpr std::uint8_t kStatusTypeMask = 0xF0u;
constexpr std::uint8_t kPitchWheelStatus = 0xE0u;

// 14-bit pitch-wheel value from a (data1 = LSB, data2 = MSB) pair.
int pitchWheelValue(std::uint8_t data1, std::uint8_t data2) noexcept {
    return (static_cast<int>(data2) << 7) | static_cast<int>(data1);
}

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
    currentSampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    // A host may legitimately report 0 here (JUCE's own docs allow it when the block size is not
    // known in advance); every dsp/ module clamps its process() call against the size it was
    // prepared with, so a zero would silence the plugin permanently rather than merely under-size
    // a buffer. One sample is the smallest honest floor.
    preparedBlockSize_ = std::max(1, samplesPerBlock);

    noteAllocator_.prepare(kP1NumStrings);

    // FractionalDelayKind::Lagrange3 is the shipping default recorded in
    // docs/decisions/0002-fractional-delay.md; it is fixed for the life of one prepare().
    stringNetwork_.prepare(currentSampleRate_, preparedBlockSize_, cnpg::dsp::FractionalDelayKind::Lagrange3);
    stringNetwork_.setNumStrings(kP1NumStrings);

    pickupTap_.prepare(currentSampleRate_, preparedBlockSize_);

    oversampler_.prepare(currentSampleRate_, preparedBlockSize_, cnpg::dsp::Oversampler::kDefaultFactor);
    // The wrapped nonlinearity runs at factor * sampleRate on blocks of up to
    // maxBlockSize * factor samples, so it is prepared for THAT rate and block size, not the
    // host's (cnpg/dsp/Oversampler.h, "Wiring note for P1.9").
    triode_.prepare(currentSampleRate_ * oversampler_.factor(), preparedBlockSize_ * oversampler_.factor());

    cabFilter_.prepare(currentSampleRate_, preparedBlockSize_);
    outputGain_.prepare(currentSampleRate_, preparedBlockSize_);
    limiter_.prepare(currentSampleRate_, preparedBlockSize_);

    noteEvents_.clear();
    pitchBendSemitones_ = 0.0f;

    // Report the oversampler's passband group delay to the host (cnpg/dsp/Oversampler.h): 3
    // samples at the shipping 2x default. Re-read after every prepare(), because it is a function
    // of the factor -- never a hardcoded constant here.
    setLatencySamples(oversampler_.latencySamples());
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

void PluginProcessor::renderChunk(const cnpg::params::Snapshot& snapshot, const cnpg::dsp::RawMidiEvent* events,
                                  int numEvents, float* output, int numSamples) noexcept {
    // Pitch wheel: latched, applied from the start of this chunk. It is not routed through
    // NoteAllocator (it is not a note event) and it is not sample-accurate -- StringNetwork's own
    // f0 smoother is what makes a bend click-free (docs/plan.md Task P1.5), so resolving the wheel
    // at chunk granularity costs at most one chunk of lead and buys a single setParams() per
    // chunk instead of one per message.
    for (int i = 0; i < numEvents; ++i) {
        if (static_cast<std::uint8_t>(events[i].status & kStatusTypeMask) == kPitchWheelStatus)
            pitchBendSemitones_ = cnpg::dsp::pitchWheelToSemitones(pitchWheelValue(events[i].data1, events[i].data2));
    }

    cnpg::dsp::StringNetworkParams networkParams = snapshot.stringNetwork;
    networkParams.pitchBendSemitones = pitchBendSemitones_;
    stringNetwork_.setParams(networkParams);

    // Every module is retargeted from this block's snapshot BEFORE its process() call, which is
    // what keeps a host re-prepare's reset-to-defaults from surviving a whole block (docs/plan.md
    // Task P1.1 ledger item, originally noted against OutputGain::prepare()'s unity reset).
    pickupTap_.setParams(snapshot.pickup);
    triode_.setParams(snapshot.triode);
    cabFilter_.setParams(snapshot.cab);
    outputGain_.setParams(snapshot.outputGain);
    limiter_.setParams(snapshot.limiter);

    noteEvents_.clear();
    noteAllocator_.allocate(events, numEvents, noteEvents_);

    stringNetwork_.process(noteEvents_, numSamples);
    pickupTap_.process(stringNetwork_.tapBuffers(), output, numSamples);

    // The triode always runs inside the oversampled island, bypassed or not: TriodeStageParams::
    // bypass makes the wrapped stage a bit-exact passthrough, but the halfband round trip stays in
    // the path so the plugin's real latency does not change with a parameter the host was never
    // told about. The halfbands are >90 dB transparent in-band (Oversampler.h), so "audition the
    // raw string" still means the raw string.
    oversampler_.processWrapped(output, output, numSamples, [this](float* samples, int numUpsampled) noexcept {
        triode_.process(samples, samples, numUpsampled);
    });

    cabFilter_.process(output, output, numSamples);
    outputGain_.process(output, output, numSamples);
    limiter_.process(output, output, numSamples); // safety clip LAST (docs/plan.md section 2.11)
}

void PluginProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages) {
    // Instantiated first, per docs/plan.md Task P1.1 step 3: every dsp/ call below this point runs
    // with FTZ/DAZ engaged. Replaces juce::ScopedNoDenormals with the dsp-side, headless-testable
    // equivalent (docs/plan.md section 2 file tree: "there is no plugin DenormalGuard.h").
    const cnpg::dsp::ScopedFtzDazGuard ftzDazGuard;

    // Once-per-block APVTS snapshot (docs/plan.md Task P1.1 step 2): a trivial atomic-read
    // adapter, no allocation or locking. Every dsp/ module in the chain is now driven from it.
    const cnpg::params::Snapshot snapshot = cnpg::params::snapshotParameters(rawParams_);

    const int numSamples = buffer.getNumSamples();
    const int numOutputChannels = getTotalNumOutputChannels();

    if (numOutputChannels <= 0 || numSamples <= 0)
        return;

    // Never prepared (JUCE guarantees prepareToPlay first, so this is a host-contract violation
    // rather than an expected state): emit silence. It is a guard against a HANG, not merely
    // against bad audio -- the chunk loop below advances by preparedBlockSize_ samples, so a zero
    // would leave chunkStart where it was and spin forever on the audio thread.
    if (preparedBlockSize_ <= 0) {
        for (int channel = 0; channel < numOutputChannels; ++channel)
            buffer.clear(channel, 0, numSamples);
        return;
    }

    // juce::MidiBuffer -> cnpg::dsp::RawMidiEvent tuples, in the buffer's own (sample-offset
    // non-decreasing) order. No interpretation happens here -- that is NoteAllocator's and
    // MidiTranslation's job (docs/plan.md section 2.14).
    const int numRawEvents = cnpg::midi::convertMidiBuffer(midiMessages, rawMidiEvents_);

    // Clamp every offset into this block up front. JUCE hands messages back in non-decreasing
    // sample order within [0, numSamples), so this is a guard against a malformed host rather
    // than routine work -- but it is also what lets the chunk loop below assume every event is
    // consumed by the final chunk, instead of needing a "leftovers" path that would be dead code
    // in every real session and untested in every other one.
    for (int i = 0; i < numRawEvents; ++i)
        rawMidiEvents_[i].sampleOffset = std::clamp(rawMidiEvents_[i].sampleOffset, 0, numSamples - 1);

    float* const monoChannel = buffer.getWritePointer(0);

    // Chunking. Every dsp/ module clamps its process() call to the maxBlockSize it was prepared
    // with, so a host handing in more samples than prepareToPlay() promised would otherwise leave
    // the tail of the buffer holding whatever was there before. Splitting the block instead means
    // the plugin is correct for any numSamples, at the cost of one loop that does nothing in the
    // overwhelmingly common single-chunk case.
    int chunkStart = 0;
    int eventCursor = 0;

    while (chunkStart < numSamples) {
        const int chunkLength = std::min(preparedBlockSize_, numSamples - chunkStart);
        const int chunkEnd = chunkStart + chunkLength;

        // Rebase this chunk's events onto chunk-local offsets, in place -- rawMidiEvents_ is this
        // processor's own scratch, and each event is visited exactly once across all chunks. The
        // clamp covers a malformed host offset (negative, or past the block) as well as the rebase.
        const int firstEvent = eventCursor;
        while (eventCursor < numRawEvents && rawMidiEvents_[eventCursor].sampleOffset < chunkEnd) {
            rawMidiEvents_[eventCursor].sampleOffset =
                std::clamp(rawMidiEvents_[eventCursor].sampleOffset - chunkStart, 0, chunkLength - 1);
            ++eventCursor;
        }

        renderChunk(snapshot, rawMidiEvents_ + firstEvent, eventCursor - firstEvent, monoChannel + chunkStart,
                    chunkLength);

        chunkStart = chunkEnd;
    }

    // Duplicate mono to every remaining output channel (docs/plan.md Task P0.4 step 3).
    for (int channel = 1; channel < numOutputChannels; ++channel)
        buffer.copyFrom(channel, 0, monoChannel, numSamples);
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

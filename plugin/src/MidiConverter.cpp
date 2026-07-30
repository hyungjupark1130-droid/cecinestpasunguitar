#include "MidiConverter.h"

#include "cnpg/dsp/MidiTranslation.h"

namespace cnpg::midi {

int convertMidiBuffer(const juce::MidiBuffer& midiMessages,
                      cnpg::dsp::RawMidiEvent (&outEvents)[kMaxEventsPerBlock]) noexcept {
    std::size_t count = 0;

    for (const auto metadata : midiMessages) {
        if (count >= kMaxEventsPerBlock)
            break;

        const auto status = static_cast<std::uint8_t>(metadata.numBytes > 0 ? metadata.data[0] : 0);
        const auto data1 = static_cast<std::uint8_t>(metadata.numBytes > 1 ? metadata.data[1] : 0);
        const auto data2 = static_cast<std::uint8_t>(metadata.numBytes > 2 ? metadata.data[2] : 0);

        outEvents[count] = cnpg::dsp::translateRawMidi(status, data1, data2, metadata.samplePosition);
        ++count;
    }

    return static_cast<int>(count);
}

} // namespace cnpg::midi

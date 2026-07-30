#pragma once

#include <cstdint>

#include "cnpg/dsp/NoteAllocator.h" // RawMidiEvent

// MidiTranslation -- see docs/plan.md section 2.14. Keeps the entire MIDI-interpretation path
// JUCE-free and headless-testable: translateRawMidi() is a pure, stateless mapping from one raw
// host-MIDI tuple to one cnpg::dsp::RawMidiEvent, over which NoteAllocator::allocate then builds
// the string-assigned NoteEvent stream. plugin/src/MidiConverter.h/.cpp is the thin adapter over
// JUCE's MidiBuffer type that calls this function once per host MIDI message; it has no MIDI
// interpretation of its own. Zero JUCE includes.

namespace cnpg::dsp {

// Pure function: one raw host-MIDI tuple in, one RawMidiEvent out. status/data1/data2/
// sampleOffset are copied through unchanged; channel is derived from the status byte's low
// nibble (standard MIDI channel voice message encoding). Stateless, allocation-free, JUCE-free;
// safe to call from any thread, including the audio thread.
inline RawMidiEvent translateRawMidi(std::uint8_t status, std::uint8_t data1, std::uint8_t data2,
                                     std::int32_t sampleOffset) noexcept {
    return RawMidiEvent{sampleOffset, status, data1, data2, static_cast<std::uint8_t>(status & 0x0Fu)};
}

} // namespace cnpg::dsp

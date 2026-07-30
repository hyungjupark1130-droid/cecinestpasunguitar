#pragma once

#include <juce_audio_basics/juce_audio_basics.h>

#include <cstddef>

#include "cnpg/dsp/NoteAllocator.h" // cnpg::dsp::RawMidiEvent, cnpg::dsp::BlockEventQueue

// MidiConverter -- see docs/plan.md section 2.14 ("plugin-side plugin/src/MidiConverter.h/.cpp
// is a thin juce::MidiBuffer -> tuples adapter with no logic of its own") and the section 2 file
// tree ("MidiConverter.h # Thin adapter: juce::MidiBuffer -> raw (status, data1, data2,
// sampleOffset) tuples for cnpg::dsp MidiTranslation ... MidiConverter.cpp # Adapter body;
// tuples flow through MidiTranslation into NoteAllocator::allocate"). All MIDI interpretation
// (note on/off, CC64, velocity mapping, channel derivation, ...) lives dsp-side in
// cnpg::dsp::translateRawMidi / cnpg::dsp::NoteAllocator; this file only extracts the raw
// (status, data1, data2, sampleOffset) tuple from each message in a juce::MidiBuffer, hands each
// one to translateRawMidi, and collects the resulting RawMidiEvent stream -- ready to feed
// NoteAllocator::allocate directly. No interpretation of its own.

namespace cnpg::midi {

// Sized to match cnpg::dsp::BlockEventQueue's own capacity, since the RawMidiEvent stream
// produced here feeds directly into NoteAllocator::allocate -> BlockEventQueue.
inline constexpr std::size_t kMaxEventsPerBlock = cnpg::dsp::BlockEventQueue::capacity();

// Converts every message in midiMessages into a cnpg::dsp::RawMidiEvent (sample offset
// preserved via each message's own metadata.samplePosition; channel preserved through the
// status byte, derived downstream by translateRawMidi) and writes up to kMaxEventsPerBlock of
// them into outEvents, in the juce::MidiBuffer's own iteration order (sample-offset
// non-decreasing). Returns the number of events written; messages beyond capacity are dropped.
// Realtime-safe: no allocation, no juce::MidiMessage construction, no MIDI interpretation.
int convertMidiBuffer(const juce::MidiBuffer& midiMessages,
                      cnpg::dsp::RawMidiEvent (&outEvents)[kMaxEventsPerBlock]) noexcept;

} // namespace cnpg::midi

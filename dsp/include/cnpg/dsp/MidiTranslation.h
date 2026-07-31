#pragma once

#include <cstdint>

#include "cnpg/dsp/Common.h"        // kPitchBendRangeSemitones
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

// Centre of the 14-bit MIDI pitch wheel: 0..16383 with 8192 as "no bend". The range is
// deliberately asymmetric in raw units (8192 steps down, 8191 up) -- MIDI has no exact
// representation of full-scale up -- so the two sides are scaled by their own span and full-scale
// in each direction maps to exactly +/-kPitchBendRangeSemitones.
inline constexpr int kPitchWheelCentre = 8192;
inline constexpr int kPitchWheelMax = 16383;

// Pure function: 14-bit pitch-wheel value -> semitones for StringNetworkParams::pitchBendSemitones
// (docs/plan.md Task P1.5 step 4, "MIDI pitch-wheel mapped +/-2 semitones"). Lives here rather
// than in the plugin so the whole MIDI-interpretation path stays JUCE-free and headless-testable;
// PluginProcessor reads the wheel from the host MIDI buffer and passes the raw value straight in
// (Task P1.9). Out-of-range values are clamped, so a malformed message can never bend past the
// range the string rails are sized for. Stateless, allocation-free; safe on the audio thread.
inline float pitchWheelToSemitones(int wheelValue) noexcept {
    const int clamped = (wheelValue < 0) ? 0 : ((wheelValue > kPitchWheelMax) ? kPitchWheelMax : wheelValue);
    const int offset = clamped - kPitchWheelCentre;
    const float span = static_cast<float>(offset >= 0 ? (kPitchWheelMax - kPitchWheelCentre) : kPitchWheelCentre);
    return kPitchBendRangeSemitones * static_cast<float>(offset) / span;
}

} // namespace cnpg::dsp

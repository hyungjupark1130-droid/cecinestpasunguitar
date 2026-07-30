#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"

// NoteAllocator -- see docs/plan.md section 2.12. Maps raw host MIDI to string-assigned
// NoteEvents (strings are monophonic). Task P1.2 lands the P1 scope only: prepare(1), every
// NoteOn assigned stringIndex 0, monophonic last-note priority (a new NoteOn always claims
// string 0, displacing whatever was sounding, with no synthesized NoteOff for the displaced
// note -- the same "stale NoteOff" contract the full P2.6 steal policy formalizes, already true
// in the trivial single-string case), NoteOffs matched to the currently sounding note, channel
// carried opaquely end-to-end (the P5 MPE seam), and CC64 fully ignored (sustainActive() always
// returns false). AllocationMode/NoteAllocatorParams are accepted and stored by setParams() but
// their GuitarFingering/FreeZones assignment logic -- and CC64 sustain deferral -- is P2.6 work
// (docs/plan.md "P2.6 -- NoteAllocator modes, retrigger semantics, CC64 sustain"). Zero JUCE
// includes.

namespace cnpg::dsp {

// Minimal host-MIDI carrier; built by the plugin layer (plugin/src/MidiConverter.h/.cpp) via
// cnpg::dsp::translateRawMidi (MidiTranslation.h), one per host MIDI message.
struct RawMidiEvent {
    std::int32_t sampleOffset; // 0..numSamples-1
    std::uint8_t status;       // note on/off, CC (CC64 consumed here from P2.6)
    std::uint8_t data1;
    std::uint8_t data2;
    std::uint8_t channel; // opaque through P2; MPE key in P5
};

enum class AllocationMode : std::uint8_t {
    GuitarFingering, // emulation fingering logic over the configured open-string tuning (P2.6)
    FreeZones        // user-defined note zone per string (P2.6)
};

struct StringZone {
    std::uint8_t lowNote;
    std::uint8_t highNote;
}; // inclusive, FreeZones mode (P2.6)

struct NoteAllocatorParams {
    AllocationMode mode = AllocationMode::GuitarFingering;
    std::array<std::uint8_t, kMaxStrings> openStringMidiNote{}; // GuitarFingering tuning reference (P2.6)
    std::array<StringZone, kMaxStrings> zones{};                // FreeZones assignment table (P2.6)
};

static_assert(std::is_trivially_copyable_v<NoteAllocatorParams>,
              "NoteAllocatorParams must stay trivially copyable for the realtime APVTS snapshot path.");

class NoteAllocator {
  public:
    // Message thread; may allocate. P1 scope: numStrings is expected to be 1 (the P1 vertical
    // slice is a single monophonic voice); full per-string state for numStrings in 2..kMaxStrings
    // lands with the P2.6 allocation-mode logic. Calls reset().
    void prepare(int numStrings);

    // Realtime-safe. Releases the currently-sounding note tracking; does not touch params_.
    void reset() noexcept;

    // Realtime-safe; only retargets params_ (a trivial copy). GuitarFingering/FreeZones modes
    // are accepted and stored here but have no effect on allocate() until P2.6.
    void setParams(const NoteAllocatorParams& p) noexcept;

    // Consumes one block of raw MIDI (non-decreasing sampleOffset, matching MidiConverter's
    // output order) and pushes string-assigned NoteEvents into outEvents. P1 scope: every NoteOn
    // is assigned stringIndex 0 (monophonic last-note priority -- a new NoteOn always claims the
    // string, displacing whatever was sounding, with no synthesized NoteOff for the displaced
    // note, matching the P2.6 steal contract); a NoteOff is emitted only if it matches the
    // (channel, midiNote) currently owning the string, otherwise it is a stale NoteOff and is
    // dropped; notes outside kMinMidiNote..kMaxMidiNote are rejected (no NoteEvent emitted); CC
    // messages (including CC64) are no-ops. Never allocates.
    void allocate(const RawMidiEvent* events, int numEvents, BlockEventQueue& outEvents) noexcept;

    // -1 if no string currently sounds (channel, midiNote); 0 if it is the P1 single string.
    int stringForNote(std::uint8_t channel, std::uint8_t midiNote) const noexcept;

    // Always false in P1 -- CC64 sustain deferral is P2.6 work.
    bool sustainActive() const noexcept;

  private:
    int numStrings_ = 1;
    NoteAllocatorParams params_{};

    bool stringSounding_ = false;
    std::uint8_t soundingChannel_ = 0;
    std::uint8_t soundingNote_ = 0;
};

} // namespace cnpg::dsp

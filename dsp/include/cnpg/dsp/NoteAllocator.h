#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"

// NoteAllocator -- see docs/plan.md section 2.12. Maps raw host MIDI to string-assigned
// NoteEvents (strings are monophonic). Task P1.2 landed the P1 scope (every NoteOn on string 0,
// monophonic last-note priority, CC64 ignored); Task P2.6 -- this file's current state -- lands the
// whole of it: both AllocationModes, the steal policy, CC64 sustain deferral, and the diagnostics
// counters that make a silent drop loud. Channel is carried opaquely end-to-end (the P5 MPE seam).
// Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// "IDLE" MEANS "OWNS NO NOTE" -- NOT "IS SILENT" (Task P2.4's entry condition for this task)
// ---------------------------------------------------------------------------------------------
// Since Task P2.4 the strings share a loaded bridge, so a string with no note of its own can be
// ringing sympathetically. That makes "is this string free?" and "is this string silent?" two
// different questions, and this class answers only the first one: it tracks (string -> owned
// (channel, note)) and nothing else. It never looks at StringNetwork, never asks whether a string
// is making sound, and could not act on the answer if it did.
//
// That is deliberate, and it is the correct half:
//   - Taking sympathetic ringing as "busy" would make the allocator refuse perfectly good strings
//     and steal ones that are actually playing, on an instrument where after the first chord EVERY
//     string is ringing a little.
//   - Taking it as "free" -- which is what this class does -- is right for allocation, and leaves
//     the audible consequence (a fresh attack on a sympathetically ringing string discards that
//     motion) where it belongs: in StringNetwork, which is the only thing that can see it. The
//     same ownership predicate decides both, so the two levels cannot drift apart -- StringNetwork
//     treats "sounding_ || releasing_" as "owns a note" for exactly the same reason.
//
// A CC64-held NoteOff does NOT free the string. The note is still ringing (that is what the pedal
// does), so the string is still busy and a later NoteOn has to steal it like any other.

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
}; // inclusive, FreeZones mode; lowNote > highNote is a legal EMPTY zone

// The playable span above an open string, in semitones: 24 frets, the plan's figure.
inline constexpr int kFingeringFretSpan = 24;

// The controller for CC64 sustain, and the value at or above which the pedal reads as DOWN. Both
// are MIDI, not taste: 64 is the standard sustain controller and 64 is the standard on/off split
// for a continuous controller used as a switch.
inline constexpr std::uint8_t kSustainPedalController = 64;
inline constexpr std::uint8_t kSustainPedalDownThreshold = 64;

// Default open-string tuning. Slots 0..5 are EADGBE (the plan's {40, 45, 50, 55, 59, 64}); slots 6
// and 7 carry the low B and F# an extended-range 7- and 8-string instrument adds, so the full
// default set IS the standard 8-string tuning F#1 B1 E2 A2 D3 G3 B3 E4 -- just written with the
// six-string spelling first, because slots 0..5 have to stay EADGBE for the shipped 6-string
// default. Nothing in this class reads the array in pitch order: only fret distance and
// least-recently-used decide, and neither cares which slot a note came from.
inline constexpr std::array<std::uint8_t, kMaxStrings> kDefaultOpenStringMidiNote{40, 45, 50, 55, 59, 64, 35, 30};

namespace detail {

// FreeZones' default table is the fingering table: each string's zone is exactly the 24-fret span
// GuitarFingering would give it. A value-initialized std::array<StringZone> would be all {0, 0},
// i.e. every zone empty except for MIDI note 0 -- so the mode's default would drop every note the
// user played and increment a counter about it. A default that silences the instrument is not a
// default.
constexpr std::array<StringZone, kMaxStrings> defaultStringZones() noexcept {
    std::array<StringZone, kMaxStrings> zones{};
    for (std::size_t s = 0; s < static_cast<std::size_t>(kMaxStrings); ++s) {
        const int open = static_cast<int>(kDefaultOpenStringMidiNote[s]);
        const int high = open + kFingeringFretSpan;
        zones[s].lowNote = static_cast<std::uint8_t>(open);
        zones[s].highNote = static_cast<std::uint8_t>(high > 127 ? 127 : high);
    }
    return zones;
}

constexpr std::array<bool, kMaxStrings> allStringsEnabled() noexcept {
    std::array<bool, kMaxStrings> enabled{};
    for (bool& value : enabled)
        value = true;
    return enabled;
}

} // namespace detail

struct NoteAllocatorParams {
    AllocationMode mode = AllocationMode::GuitarFingering;
    std::array<std::uint8_t, kMaxStrings> openStringMidiNote = kDefaultOpenStringMidiNote;
    std::array<StringZone, kMaxStrings> zones = detail::defaultStringZones();

    // The ACTIVE string count and the per-string mute, mirrored from StringNetwork's own surface
    // (StringNetwork::setNumStrings and StringNetworkParams::PerString::enabled). They are here
    // rather than only there because the allocator has to make the same decision the network does:
    // StringNetwork::handleEvent silently drops a NoteEvent addressed to a string outside the count
    // or to a disabled one, so an allocator that did not know the count would assign notes into
    // nothing and the drop would be invisible. prepare() sets the CAPACITY; this is the count in
    // force, and the effective count is the smaller of the two.
    int activeStringCount = kMaxStrings;
    std::array<bool, kMaxStrings> stringEnabled = detail::allStringsEnabled();
};

static_assert(std::is_trivially_copyable_v<NoteAllocatorParams>,
              "NoteAllocatorParams must stay trivially copyable for the realtime APVTS snapshot path.");

class NoteAllocator {
  public:
    // Message thread; may allocate (it does not, today -- every array is a fixed-size member).
    // `numStrings` is the CAPACITY, clamped to 1..kMaxStrings; the count actually in force is
    // min(capacity, NoteAllocatorParams::activeStringCount), so a plugin prepares for kMaxStrings
    // once and automates the count through setParams(). Calls reset().
    void prepare(int numStrings);

    // Realtime-safe. Releases every string's note ownership, drops every CC64-held NoteOff, lifts
    // the pedal, and zeroes the diagnostics counters (the EventQueue::clear convention). Does not
    // touch params_.
    void reset() noexcept;

    // Realtime-safe; only retargets params_ (a trivial copy).
    void setParams(const NoteAllocatorParams& p) noexcept;

    // Consumes one block of raw MIDI (non-decreasing sampleOffset, matching MidiConverter's output
    // order) and pushes string-assigned NoteEvents into outEvents, themselves non-decreasing in
    // sampleOffset. Never allocates, never blocks; everything it cannot do it COUNTS (see the
    // diagnostics below).
    //
    // NoteOn. Notes outside kMinMidiNote..kMaxMidiNote are rejected outright and counted
    // (outOfRangeNoteCount()). Otherwise the
    // candidate set is every enabled in-count string that can play the note -- GuitarFingering:
    // openStringMidiNote[i] <= note <= openStringMidiNote[i] + kFingeringFretSpan; FreeZones: the
    // note lies inside the inclusive zone. Among candidates that own no note, the one with the
    // lowest fret position wins (FreeZones has no fret, so every candidate ties there), ties going
    // to the least recently triggered. If every candidate owns a note, the least recently triggered
    // one is STOLEN: the new NoteOn is emitted onto it, no NoteOff is synthesized for the displaced
    // note, and that note loses its ownership -- so its later NoteOff, direct or CC64-held, is a
    // stale NoteOff and is dropped. A note no candidate can play is UNASSIGNABLE: no NoteEvent,
    // unassignableNoteCount() increments, and stringForNote() reports -1.
    //
    // A NoteOn for a (channel, note) some string already owns goes back to THAT string, wherever it
    // is, rather than opening a second copy elsewhere: at most one string may own a given
    // (channel, note), and that invariant is what makes NoteOff matching well defined. The one
    // exception is an owner that can no longer be ADDRESSED -- it left the active count, or its
    // enable went false, both of which are automatable while the note is held. That ownership is
    // released and the note is reassigned like any other, because StringNetwork::handleEvent drops
    // every event addressed to such a string and a restrike handed back to it would vanish with no
    // counter moving anywhere. Releasing before reassigning is what keeps the invariant.
    //
    // NoteOff (including a NoteOn with velocity 0). Emitted only if the (channel, note) still owns
    // a string. With the pedal down it is HELD instead, and the string stays owned.
    //
    // CC64. Value >= kSustainPedalDownThreshold is pedal-down; while down, NoteOffs are held per
    // string. On pedal-up every held NoteOff is emitted at the pedal-release sample offset in
    // ascending string order. A NoteOn landing on a string with a held NoteOff for the same note
    // cancels that held NoteOff and retriggers -- and a steal cancels it too, by taking the
    // ownership the held NoteOff needed.
    void allocate(const RawMidiEvent* events, int numEvents, BlockEventQueue& outEvents) noexcept;

    // The string currently owning (channel, midiNote), or -1 if no string does -- which is also the
    // answer for a note that was unassignable and for a note whose string has since been stolen.
    int stringForNote(std::uint8_t channel, std::uint8_t midiNote) const noexcept;

    // The pedal's state right now.
    bool sustainActive() const noexcept;

    // ---- diagnostics ---------------------------------------------------------------------------
    // All three are cumulative since construction, prepare() or the last reset(), matching
    // EventQueue::droppedCount()'s convention. They exist because every failure mode this class has
    // is SILENT -- a note that produces no NoteEvent sounds exactly like a note that was never
    // played -- and a counter nobody reads is a counter that lies, so every [contract] case in the
    // suite asserts all three, zero everywhere except where the case is about the drop itself.
    //
    // THREE counters, not one, because the three failures need three different fixes: the note was
    // outside the instrument's design envelope (fix the input), the note was inside it and the
    // tuning or zone table had nowhere to put it (fix the table), or the allocation succeeded and
    // the destination queue was full (a note that never stops).

    // NoteOns that no enabled, in-count string could play (outside every fingering span, or outside
    // every zone). The mode's designed-in failure, not a bug: a 6-string EADGBE instrument cannot
    // play MIDI 30, and saying so is better than transposing it somewhere the player did not ask
    // for.
    std::uint32_t unassignableNoteCount() const noexcept;

    // NoteOns outside kMinMidiNote..kMaxMidiNote, rejected before the assignment policy ever sees
    // them. Separate from unassignableNoteCount() because the assignment policy genuinely never ran:
    // a FreeZones table spanning all of MIDI has somewhere to put every note and still cannot accept
    // MIDI 20, since WaveguideString's rails are sized for the design envelope and nothing outside
    // it could sound. Counted rather than dropped in silence so cnpg_render fails a corpus that
    // contains one instead of reporting every statistic as healthy.
    std::uint32_t outOfRangeNoteCount() const noexcept;

    // NoteEvents this class produced and the destination queue refused because it was full
    // (BlockEventQueue holds 256 per block, shared with everything else the host sent). Distinct
    // from unassignableNoteCount(): the allocation SUCCEEDED and the event was still lost, which
    // for a NoteOff means a note that never stops.
    std::uint32_t queueOverflowCount() const noexcept;

    // ---- direct state observation (tests, and later the GUI) -----------------------------------
    // The three questions a test has to be able to ask without inferring them from emitted events:
    // which note a string owns, and whether that note's NoteOff is sitting under the pedal.

    // The MIDI note `stringIndex` owns, or -1 if it owns none. -1 is exactly "idle" in this class's
    // sense -- see the file header for why that is not "silent".
    int ownedNote(int stringIndex) const noexcept;
    int ownedChannel(int stringIndex) const noexcept;
    bool sustainHoldPending(int stringIndex) const noexcept;

    // The count actually in force: min(the prepared capacity, params_.activeStringCount), clamped
    // to 1..kMaxStrings.
    int effectiveStringCount() const noexcept;

  private:
    // -1 if the note is unassignable. Realtime-safe, allocation-free, O(kMaxStrings).
    int chooseString(std::uint8_t channel, std::uint8_t midiNote) const noexcept;
    // "StringNetwork would accept an event for this string": in the active count AND enabled. The
    // same two questions StringNetwork::handleEvent asks before dropping an event on the floor.
    bool stringAddressable(int stringIndex) const noexcept;
    bool stringCanPlay(int stringIndex, int midiNote) const noexcept;
    void emitNoteOn(int stringIndex, const RawMidiEvent& raw, BlockEventQueue& outEvents) noexcept;
    void emitNoteOff(int stringIndex, std::int32_t sampleOffset, BlockEventQueue& outEvents) noexcept;

    int capacityStrings_ = 1;
    NoteAllocatorParams params_{};

    // Ownership, one entry per string (SoA, same convention as StringNetwork).
    std::array<bool, kMaxStrings> owned_{};
    std::array<std::uint8_t, kMaxStrings> ownerChannel_{};
    std::array<std::uint8_t, kMaxStrings> ownerNote_{};

    // CC64: a NoteOff that arrived while the pedal was down, waiting for the pedal to come up. At
    // most one per string, because a string owns at most one note.
    std::array<bool, kMaxStrings> heldNoteOff_{};
    std::array<std::uint8_t, kMaxStrings> heldVelocity_{};

    // LRU. One monotonic counter stamped onto a string every time it is triggered. It is BOTH
    // orderings the policy needs and they cannot disagree: "least recently used" among idle
    // candidates and "least recently triggered" among stealable ones are the same number read with
    // a different question in mind. A never-triggered string carries 0 and therefore sorts oldest,
    // which is what makes the first six notes of a chord spread across the six strings instead of
    // piling onto one.
    std::array<std::uint64_t, kMaxStrings> lastTriggerSequence_{};
    std::uint64_t triggerSequence_ = 0;

    bool sustainDown_ = false;

    std::uint32_t unassignableNoteCount_ = 0;
    std::uint32_t outOfRangeNoteCount_ = 0;
    std::uint32_t queueOverflowCount_ = 0;
};

} // namespace cnpg::dsp

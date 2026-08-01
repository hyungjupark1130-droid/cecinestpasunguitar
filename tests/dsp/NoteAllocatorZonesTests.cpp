#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/NoteAllocator.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

// NoteAllocatorZonesTests -- docs/plan.md Task P2.6, AllocationMode::FreeZones.
//
// FreeZones is the mode with no instrument behind it: the user draws a note range per string and
// the allocator routes by that table alone. Two things follow, and they are what this file gates.
// Overlapping zones are LEGAL, so a note that several strings claim has to be resolved by
// least-recently-used and nothing else -- which is what makes repeated notes ALTERNATE rather than
// pile onto the lowest-numbered string. And a zone table is a partial map: a note in no zone is
// UNASSIGNABLE, and the only trace it leaves is a counter.

namespace {

using cnpg::dsp::AllocationMode;
using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::NoteAllocator;
using cnpg::dsp::NoteAllocatorParams;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RawMidiEvent;
using cnpg::dsp::StringZone;

RawMidiEvent noteOn(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t velocity = 100,
                    std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x90u | channel), note, velocity, channel};
}

RawMidiEvent noteOff(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x80u | channel), note, 0, channel};
}

std::vector<NoteEvent> drain(BlockEventQueue& queue) {
    std::vector<NoteEvent> events;
    std::int32_t previousOffset = -1;
    while (const NoteEvent* event = queue.peek()) {
        REQUIRE(event->sampleOffset >= previousOffset);
        previousOffset = event->sampleOffset;
        events.push_back(*event);
        queue.pop();
    }
    return events;
}

void requireNoDrops(const NoteAllocator& allocator) {
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

// Three strings whose zones all contain MIDI 60, so every allocation decision for that note is
// decided by LRU alone.
NoteAllocatorParams overlappingThree() {
    NoteAllocatorParams params;
    params.mode = AllocationMode::FreeZones;
    params.activeStringCount = 3;
    params.zones[0] = StringZone{55, 67};
    params.zones[1] = StringZone{55, 67};
    params.zones[2] = StringZone{55, 67};
    for (std::size_t s = 3; s < static_cast<std::size_t>(cnpg::dsp::kMaxStrings); ++s)
        params.zones[s] = StringZone{1, 0}; // empty, and out of the count anyway
    return params;
}

} // namespace

TEST_CASE("CONTRACT: FreeZones alternates overlapping zones under repeated notes", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(overlappingThree());

    // Six strikes of the SAME note, each fully released before the next -- so every strike sees
    // three idle candidates and the only thing separating them is when each was last used.
    std::vector<int> visited;
    for (int strike = 0; strike < 6; ++strike) {
        const RawMidiEvent on[] = {noteOn(0, 60)};
        BlockEventQueue onOut;
        allocator.allocate(on, 1, onOut);
        const std::vector<NoteEvent> emitted = drain(onOut);
        REQUIRE(emitted.size() == 1);
        REQUIRE(emitted[0].type == NoteEventType::NoteOn);
        visited.push_back(static_cast<int>(emitted[0].stringIndex));

        // IN STATE: this strike really did face a free choice. Exactly one string is busy (the one
        // it just took) and the other two are idle, so the assignment below is LRU deciding rather
        // than availability deciding for it.
        int idle = 0;
        for (int s = 0; s < 3; ++s)
            if (allocator.ownedNote(s) < 0)
                ++idle;
        REQUIRE(idle == 2);

        const RawMidiEvent off[] = {noteOff(64, 60)};
        BlockEventQueue offOut;
        allocator.allocate(off, 1, offOut);
        REQUIRE(drain(offOut).size() == 1);
    }

    std::cout << "[contract] FreeZones LRU over three overlapping zones, six strikes of MIDI 60:";
    for (int s : visited)
        std::cout << " s" << s;
    std::cout << "\n";

    // ROUND ROBIN. Stated as the two properties that matter rather than as a literal sequence: no
    // string is used twice in a row, and all three are used inside any window of three -- which is
    // what "alternate" means and what a lowest-index-wins policy fails.
    for (std::size_t i = 1; i < visited.size(); ++i) {
        INFO("strike " << i);
        REQUIRE(visited[i] != visited[i - 1]);
    }
    for (std::size_t i = 0; i + 2 < visited.size(); ++i) {
        const bool allThree =
            (visited[i] != visited[i + 1]) && (visited[i + 1] != visited[i + 2]) && (visited[i] != visited[i + 2]);
        INFO("window starting at strike " << i);
        REQUIRE(allThree);
    }
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: FreeZones steals the least-recently-triggered string when every zone is busy", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(overlappingThree());

    // Three different notes, all inside all three zones, held simultaneously.
    const RawMidiEvent held[] = {noteOn(0, 60), noteOn(1, 62), noteOn(2, 64)};
    BlockEventQueue heldOut;
    allocator.allocate(held, 3, heldOut);
    const std::vector<NoteEvent> emitted = drain(heldOut);
    REQUIRE(emitted.size() == 3);
    const int firstString = static_cast<int>(emitted[0].stringIndex);

    for (int s = 0; s < 3; ++s) {
        INFO("string " << s);
        REQUIRE(allocator.ownedNote(s) >= 0); // in state: nothing idle, so the steal path is forced
    }

    const RawMidiEvent fourth[] = {noteOn(10, 65)};
    BlockEventQueue fourthOut;
    allocator.allocate(fourth, 1, fourthOut);
    const std::vector<NoteEvent> stolen = drain(fourthOut);

    REQUIRE(stolen.size() == 1); // no synthesized NoteOff for the displaced note
    REQUIRE(static_cast<int>(stolen[0].stringIndex) == firstString);
    REQUIRE(allocator.stringForNote(0, 60) == -1); // MIDI 60 was struck first and lost its string
    REQUIRE(allocator.stringForNote(0, 65) == firstString);
    std::cout << "[contract] FreeZones steal: MIDI 65 took string " << firstString
              << ", displacing the first-struck note (MIDI 60)\n";
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a note in no zone emits nothing, counts, and reports no string", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(overlappingThree()); // zones cover 55..67 only

    const RawMidiEvent events[] = {noteOn(0, 40), noteOn(10, 90), noteOn(20, 60)};
    BlockEventQueue out;
    allocator.allocate(events, 3, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 1); // only the in-zone note produced a NoteEvent
    REQUIRE(emitted[0].midiNote == 60);

    // All three assertions the criterion names, together: no event, the counter moved, and the
    // note reports no string.
    REQUIRE(allocator.unassignableNoteCount() == 2);
    REQUIRE(allocator.queueOverflowCount() == 0);
    REQUIRE(allocator.stringForNote(0, 40) == -1);
    REQUIRE(allocator.stringForNote(0, 90) == -1);
    std::cout << "[contract] FreeZones unassignable: MIDI 40 and 90 fall outside every zone, counter reads "
              << allocator.unassignableNoteCount() << "\n";

    // ...and a NoteOff for a note that was never assigned is stale rather than an error.
    const RawMidiEvent offs[] = {noteOff(30, 40)};
    BlockEventQueue offOut;
    allocator.allocate(offs, 1, offOut);
    REQUIRE(offOut.empty());
}

TEST_CASE("CONTRACT: an empty zone claims nothing, including its own bounds", "[contract]") {
    // lowNote > highNote is the natural spelling of "this string is not in the map" -- it comes out
    // of an inclusive test with no special case -- and it has to claim NOTHING, not the two notes
    // its bounds happen to name. A build that normalised the pair by swapping it would silently
    // turn every user's "disabled" zone into a two-note-wide live one.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);

    NoteAllocatorParams params;
    params.mode = AllocationMode::FreeZones;
    params.activeStringCount = 2;
    params.zones[0] = StringZone{70, 60}; // inverted: empty
    params.zones[1] = StringZone{64, 64}; // degenerate but legal: exactly one note
    allocator.setParams(params);

    const RawMidiEvent events[] = {noteOn(0, 60), noteOn(4, 70), noteOn(8, 64)};
    BlockEventQueue out;
    allocator.allocate(events, 3, out);
    const std::vector<NoteEvent> emitted = drain(out);

    REQUIRE(emitted.size() == 1);
    REQUIRE(emitted[0].midiNote == 64);
    REQUIRE(emitted[0].stringIndex == 1);
    REQUIRE(allocator.unassignableNoteCount() == 2); // both of the inverted zone's own bounds
    REQUIRE(allocator.queueOverflowCount() == 0);
}

TEST_CASE("CONTRACT: FreeZones respects the active string count", "[contract]") {
    // A zone table is sized for kMaxStrings and the count is a live parameter, so a string can be
    // in the table and out of the instrument at the same time. Assigning to one would be a note
    // StringNetwork drops on arrival -- silently, since the allocator's own counters would read
    // zero.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);

    NoteAllocatorParams params;
    params.mode = AllocationMode::FreeZones;
    params.activeStringCount = 1;
    for (auto& zone : params.zones)
        zone = StringZone{0, 127}; // every string could take every note...
    allocator.setParams(params);

    const RawMidiEvent events[] = {noteOn(0, 60), noteOn(4, 62)};
    BlockEventQueue out;
    allocator.allocate(events, 2, out);
    const std::vector<NoteEvent> emitted = drain(out);

    // ...but only one string exists, so the second note steals the first's string rather than
    // landing on a string outside the count.
    REQUIRE(emitted.size() == 2);
    REQUIRE(emitted[0].stringIndex == 0);
    REQUIRE(emitted[1].stringIndex == 0);
    REQUIRE(allocator.effectiveStringCount() == 1);
    REQUIRE(allocator.stringForNote(0, 60) == -1); // displaced
    REQUIRE(allocator.stringForNote(0, 62) == 0);
    requireNoDrops(allocator);
}

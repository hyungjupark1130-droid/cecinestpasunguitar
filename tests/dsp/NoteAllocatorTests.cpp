#include "cnpg/dsp/NoteAllocator.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using cnpg::dsp::AllocationMode;
using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::NoteAllocator;
using cnpg::dsp::NoteAllocatorParams;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RawMidiEvent;

namespace {

RawMidiEvent noteOn(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t velocity, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x90u | channel), note, velocity, channel};
}

RawMidiEvent noteOff(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x80u | channel), note, 0, channel};
}

RawMidiEvent controlChange(std::int32_t sampleOffset, std::uint8_t controller, std::uint8_t value,
                           std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0xB0u | channel), controller, value, channel};
}

} // namespace

TEST_CASE("NoteAllocator: a NoteOn is assigned stringIndex 0 and a matching NoteOff engages it", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOff(20, 60)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 2, outEvents);

    REQUIRE(outEvents.size() == 2);

    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOn);
    REQUIRE(outEvents.peek()->stringIndex == 0);
    REQUIRE(outEvents.peek()->midiNote == 60);
    REQUIRE(outEvents.peek()->sampleOffset == 0);
    outEvents.pop();

    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOff);
    REQUIRE(outEvents.peek()->stringIndex == 0);
    REQUIRE(outEvents.peek()->midiNote == 60);
    REQUIRE(outEvents.peek()->sampleOffset == 20);
}

TEST_CASE("NoteAllocator: monophonic last-note priority steals the string; the displaced note's stale NoteOff is "
          "dropped",
          "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    // NoteOn A (60) -> NoteOn B (64) steals the string before A is released -> NoteOff A (stale,
    // dropped, A no longer owns string 0) -> NoteOff B (engages the damper).
    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOn(10, 64, 100), noteOff(20, 60), noteOff(30, 64)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 4, outEvents);

    // NoteOn A, NoteOn B, NoteOff B -- NoteOff A produced nothing.
    REQUIRE(outEvents.size() == 3);

    REQUIRE(outEvents.peek()->midiNote == 60);
    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOn);
    outEvents.pop();

    REQUIRE(outEvents.peek()->midiNote == 64);
    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOn);
    outEvents.pop();

    REQUIRE(outEvents.peek()->midiNote == 64);
    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOff);
    outEvents.pop();

    REQUIRE(outEvents.empty());
}

TEST_CASE("NoteAllocator: emitted events carry kUnspecifiedNoteParam for pluck position and hardness", "[contract]") {
    // Plain MIDI note-on carries neither a pluck position nor a hardness, so the allocator says so
    // explicitly instead of inventing a plausible-looking 0.5: StringNetwork then resolves both
    // against PluckExciterParams::defaultPosition/defaultHardness (docs/plan.md section 2.3, "used
    // when the note event carries no explicit position"), which is the ONLY thing that makes the
    // APVTS Exciter Position and Exciter Hardness knobs audible in P1.
    //
    // Asserted here rather than left to StringNetwork's own resolution tests, because those pass
    // whatever the allocator emits: reverting these two fields to a literal 0.5 would silence both
    // knobs for the whole phase without turning a single other assertion red.
    const float unspecified = cnpg::dsp::kUnspecifiedNoteParam;

    // The precedence in StringNetwork::resolveNoteParam is "an in-range event value wins", so the
    // sentinel MUST sit outside 0..1 or the fallback can never fire and the check above inverts
    // silently.
    REQUIRE_FALSE((unspecified >= 0.0f && unspecified <= 1.0f));

    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOff(20, 60)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 2, outEvents);
    REQUIRE(outEvents.size() == 2);

    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOn);
    REQUIRE(outEvents.peek()->pluckPosition == unspecified);
    REQUIRE(outEvents.peek()->hardness == unspecified);
    outEvents.pop();

    // A NoteOff excites nothing, so both fields are equally unspecified there.
    REQUIRE(outEvents.peek()->type == NoteEventType::NoteOff);
    REQUIRE(outEvents.peek()->pluckPosition == unspecified);
    REQUIRE(outEvents.peek()->hardness == unspecified);
}

TEST_CASE("NoteAllocator: velocity maps data2 0..127 onto 0..1", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    {
        const RawMidiEvent events[] = {noteOn(0, 60, 127)};
        BlockEventQueue outEvents;
        allocator.allocate(events, 1, outEvents);
        REQUIRE(outEvents.peek()->velocity == 1.0f);
    }

    allocator.reset();

    {
        const RawMidiEvent events[] = {noteOn(0, 60, 64)};
        BlockEventQueue outEvents;
        allocator.allocate(events, 1, outEvents);
        REQUIRE(outEvents.peek()->velocity == Catch::Approx(64.0f / 127.0f));
    }
}

TEST_CASE("NoteAllocator: notes outside kMinMidiNote..kMaxMidiNote are rejected", "[contract]") {
    // The design-envelope rejection, ISOLATED from the assignment policy. A zone table covering all
    // of MIDI is what isolates it: under the default GuitarFingering tuning the in-range boundary
    // note (MIDI 21) is also unassignable -- a 6-string EADGBE instrument cannot play it -- so a
    // single "one event came out" assertion would pass for the wrong reason and would keep passing
    // if the range check were deleted. Here every in-range note has somewhere to go, so the only
    // thing that can drop one is the range check itself.
    NoteAllocator allocator;
    allocator.prepare(1);

    NoteAllocatorParams params;
    params.mode = AllocationMode::FreeZones;
    params.zones[0] = cnpg::dsp::StringZone{0, 127};
    allocator.setParams(params);

    const RawMidiEvent events[] = {
        noteOn(0, static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote - 1), 100),  // below range
        noteOn(10, static_cast<std::uint8_t>(cnpg::dsp::kMaxMidiNote + 1), 100), // above range
        noteOn(20, static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote), 100),     // in-range boundary
    };
    BlockEventQueue outEvents;
    allocator.allocate(events, 3, outEvents);

    REQUIRE(outEvents.size() == 1);
    REQUIRE(outEvents.peek()->midiNote == static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote));

    // A rejected note is NOT an unassignable one, and the counters say so: an out-of-envelope note
    // never reaches the assignment policy at all, so the counter that reports "the policy had
    // nowhere to put this" must stay at zero.
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

TEST_CASE("NoteAllocator: channel is carried opaquely through into the NoteEvent", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {noteOn(0, 60, 100, 5)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 1, outEvents);

    REQUIRE(outEvents.peek()->channel == 5);
}

TEST_CASE("NoteAllocator: offsets out of allocate stay non-decreasing", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOff(15, 60), noteOn(15, 64, 100), noteOff(50, 64)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 4, outEvents);

    std::int32_t previous = -1;
    while (!outEvents.empty()) {
        REQUIRE(outEvents.peek()->sampleOffset >= previous);
        previous = outEvents.peek()->sampleOffset;
        outEvents.pop();
    }
}

TEST_CASE("NoteAllocator: every CC except 64 is ignored, and CC64 itself emits no NoteEvent", "[contract]") {
    // Task P2.6 gave CC64 meaning (tests/dsp/SustainPedalTests.cpp is where that meaning is gated).
    // What survives here is the half that did not change: the allocator consumes CC messages and
    // never turns one into a note, and every controller other than 64 is somebody else's business.
    NoteAllocator allocator;
    allocator.prepare(1);

    REQUIRE_FALSE(allocator.sustainActive());

    const RawMidiEvent events[] = {noteOn(0, 60, 100), controlChange(5, 1, 127), // mod wheel: ignored
                                   controlChange(6, 11, 64),                     // expression: ignored
                                   noteOff(10, 60)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 4, outEvents);

    REQUIRE_FALSE(allocator.sustainActive()); // no CC64 arrived, so the pedal never moved
    REQUIRE(outEvents.size() == 2);           // the two CC messages produced no NoteEvent
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);

    // ...and CC64 itself is a pedal, not a note: it moves state and emits nothing on its own.
    allocator.reset();
    const RawMidiEvent pedalOnly[] = {controlChange(0, 64, 127)};
    BlockEventQueue pedalEvents;
    allocator.allocate(pedalOnly, 1, pedalEvents);
    REQUIRE(allocator.sustainActive());
    REQUIRE(pedalEvents.empty());
}

TEST_CASE("NoteAllocator: setParams switches AllocationMode live, on the next note", "[contract]") {
    // The mode is a realtime parameter read per block, so the only thing this case can assert
    // without duplicating the two mode suites is that a change TAKES EFFECT and takes effect on the
    // next NoteOn rather than at some later reset. It is checked with a table that makes the two
    // modes disagree: MIDI 60 fingers cheapest on string 4 (open 59, fret 1) under the default
    // EADGBE, while a zone table that lists it only on string 1 must send it there instead.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);

    NoteAllocatorParams fingering;
    allocator.setParams(fingering);

    const RawMidiEvent first[] = {noteOn(0, 60, 100)};
    BlockEventQueue firstOut;
    allocator.allocate(first, 1, firstOut);
    REQUIRE(firstOut.peek()->stringIndex == 4);
    REQUIRE(allocator.stringForNote(0, 60) == 4);

    allocator.reset();

    NoteAllocatorParams zones;
    zones.mode = AllocationMode::FreeZones;
    for (auto& zone : zones.zones)
        zone = cnpg::dsp::StringZone{1, 0};         // deliberately empty everywhere...
    zones.zones[1] = cnpg::dsp::StringZone{55, 70}; // ...except string 1
    allocator.setParams(zones);

    const RawMidiEvent second[] = {noteOn(0, 60, 100)};
    BlockEventQueue secondOut;
    allocator.allocate(second, 1, secondOut);
    REQUIRE(secondOut.peek()->stringIndex == 1);
    REQUIRE(allocator.stringForNote(0, 60) == 1);
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

// -- allocate() performs no heap allocation ---------------------------------------------------
// Uses the shared counting-new instrumentation in tests/support/AllocationGuard.h/.cpp
// (docs/plan.md Task P1.2 acceptance criteria: "allocate performs no allocation (counting-new
// test)"). The global operator new/delete overrides that make the counting possible live in
// exactly one translation unit, AllocationGuard.cpp -- not here -- since Task P1.5's own
// counting-new requirement (docs/plan.md line 1078) needs the identical mechanism in a second
// test file, and a second definition of operator new(std::size_t) would be a duplicate-symbol
// link error.

TEST_CASE("NoteAllocator: allocate() performs no heap allocation", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOff(10, 60)};
    BlockEventQueue outEvents;

    // Warm-up call so the measured call below isn't the very first invocation of any code path.
    allocator.allocate(events, 2, outEvents);
    outEvents.clear();

    cnpg::test::resetAllocationCount();
    allocator.allocate(events, 2, outEvents);

    REQUIRE(cnpg::test::allocationCount() == 0);
}

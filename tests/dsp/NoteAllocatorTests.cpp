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
    NoteAllocator allocator;
    allocator.prepare(1);

    const RawMidiEvent events[] = {
        noteOn(0, static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote - 1), 100),  // below range
        noteOn(10, static_cast<std::uint8_t>(cnpg::dsp::kMaxMidiNote + 1), 100), // above range
        noteOn(20, static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote), 100),     // in-range boundary
    };
    BlockEventQueue outEvents;
    allocator.allocate(events, 3, outEvents);

    REQUIRE(outEvents.size() == 1);
    REQUIRE(outEvents.peek()->midiNote == static_cast<std::uint8_t>(cnpg::dsp::kMinMidiNote));
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

TEST_CASE("NoteAllocator: CC64 is a no-op in P1 -- sustainActive() always returns false", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    REQUIRE_FALSE(allocator.sustainActive());

    const RawMidiEvent events[] = {noteOn(0, 60, 100), controlChange(5, 64, 127), noteOff(10, 60)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 3, outEvents);

    REQUIRE_FALSE(allocator.sustainActive());
    REQUIRE(outEvents.size() == 2); // the CC64 message itself produced no NoteEvent
}

TEST_CASE("NoteAllocator: AllocationMode values are accepted by setParams without affecting P1 assignment",
          "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(1);

    NoteAllocatorParams params;
    params.mode = AllocationMode::FreeZones;
    allocator.setParams(params);

    const RawMidiEvent events[] = {noteOn(0, 60, 100)};
    BlockEventQueue outEvents;
    allocator.allocate(events, 1, outEvents);

    REQUIRE(outEvents.peek()->stringIndex == 0); // still monophonic single-string in P1
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

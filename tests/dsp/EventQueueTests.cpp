#include "cnpg/dsp/EventQueue.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::EventQueue;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;

namespace {

NoteEvent makeNoteOn(std::int32_t sampleOffset, std::uint8_t midiNote) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = 0;
    event.channel = 0;
    event.midiNote = midiNote;
    event.velocity = 1.0f;
    event.pluckPosition = 0.5f;
    event.hardness = 0.5f;
    return event;
}

} // namespace

TEST_CASE("EventQueue: push/pop preserves FIFO (push) order", "[contract]") {
    EventQueue<8> queue;

    REQUIRE(queue.push(makeNoteOn(0, 40)));
    REQUIRE(queue.push(makeNoteOn(10, 41)));
    REQUIRE(queue.push(makeNoteOn(20, 42)));

    REQUIRE(queue.size() == 3);
    REQUIRE_FALSE(queue.empty());

    REQUIRE(queue.peek()->midiNote == 40);
    queue.pop();
    REQUIRE(queue.peek()->midiNote == 41);
    queue.pop();
    REQUIRE(queue.peek()->midiNote == 42);
    queue.pop();

    REQUIRE(queue.empty());
    REQUIRE(queue.peek() == nullptr);
}

TEST_CASE("EventQueue: capacity-256 overflow increments droppedCount without corrupting queued events", "[contract]") {
    BlockEventQueue queue;

    constexpr int totalPushes = 300; // exact number from docs/plan.md Task P1.2 acceptance criteria
    int acceptedCount = 0;
    for (int i = 0; i < totalPushes; ++i) {
        if (queue.push(makeNoteOn(i, 21)))
            ++acceptedCount;
    }

    REQUIRE(acceptedCount == 256);
    REQUIRE(queue.size() == 256);
    REQUIRE(queue.droppedCount() == 44);

    // No corruption: draining returns exactly the first 256 pushes, in push order, untouched by
    // the 44 pushes that failed once the queue was full.
    for (int i = 0; i < 256; ++i) {
        REQUIRE(queue.peek() != nullptr);
        REQUIRE(queue.peek()->sampleOffset == i);
        queue.pop();
    }
    REQUIRE(queue.empty());
}

TEST_CASE("EventQueue: clear() resets size and droppedCount to zero", "[contract]") {
    BlockEventQueue queue;
    for (int i = 0; i < 260; ++i)
        queue.push(makeNoteOn(i, 21));

    REQUIRE(queue.droppedCount() == 4);

    queue.clear();
    REQUIRE(queue.size() == 0);
    REQUIRE(queue.empty());
    REQUIRE(queue.droppedCount() == 0);
}

TEST_CASE("EventQueue: capacity() reports the compile-time template parameter", "[contract]") {
    STATIC_REQUIRE(BlockEventQueue::capacity() == 256);
    STATIC_REQUIRE(EventQueue<4>::capacity() == 4);
}

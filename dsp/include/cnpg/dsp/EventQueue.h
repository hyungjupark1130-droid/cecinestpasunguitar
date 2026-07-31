#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

// EventQueue -- see docs/plan.md section 2.2 ("NoteEvent and fixed-capacity EventQueue"; this
// file is that draft, transcribed verbatim). Note events are sample-accurate within a block and
// pre-assigned to a string by NoteAllocator (Task P1.2, NoteAllocator.h). The queue is
// preallocated at fixed capacity, alloc-free, and consumed in non-decreasing sampleOffset order
// by StringNetwork::process (Task P1.5). Push order must be non-decreasing in sampleOffset;
// violation is a caller bug (debug-build assert only -- NDEBUG builds, including the Release
// configuration ctest runs under in CI, do not pay for the check). Zero JUCE includes.

namespace cnpg::dsp {

enum class NoteEventType : std::uint8_t {
    NoteOn, // velocity, pluckPosition, hardness valid; retrigger semantics per RetriggerMode (P2)
    NoteOff // engages damper with felt time constant (unless deferred by sustain upstream, P2)
};

// Written into NoteEvent::pluckPosition / ::hardness when the event carries no per-note value of
// its own, which is every event NoteAllocator emits in P1 -- plain MIDI note-on has nowhere to
// put a pluck position, and the P5 MPE seam is what eventually supplies one. Any value outside
// 0..1 means the same thing; this is simply the spelling the allocator uses. StringNetwork
// resolves it against PluckExciterParams::defaultPosition / ::defaultHardness (docs/plan.md
// section 2.3: "used when the note event carries no explicit position"), which is what keeps the
// APVTS Exciter Position / Exciter Hardness knobs live.
inline constexpr float kUnspecifiedNoteParam = -1.0f;

struct NoteEvent {
    NoteEventType type;
    std::int32_t sampleOffset; // 0..numSamples-1, offset within the current block
    std::uint8_t stringIndex;  // 0..kMaxStrings-1, assigned by NoteAllocator
    std::uint8_t channel;      // carried opaquely through P2; MPE seam for P5
    std::uint8_t midiNote;     // kMinMidiNote..kMaxMidiNote
    float velocity;            // 0..1; scales amplitude, adds mild hardness increase
    float pluckPosition;       // 0..1 fraction of string length, latched at note-on; or
                               // kUnspecifiedNoteParam (see above)
    float hardness;            // 0..1 exciter hardness at note-on; or kUnspecifiedNoteParam
};

// Fixed-capacity ring buffer of NoteEvent, alloc-free after construction: all storage is a
// std::array<NoteEvent, Capacity> member, never the heap. push() reports and counts overflow
// instead of growing; peek()/pop() give FIFO (push-order) access to the earliest remaining event.
template <std::size_t Capacity> class EventQueue {
  public:
    // Never allocates. Debug-build assert: e.sampleOffset must be >= the most recently pushed
    // event's sampleOffset (non-decreasing push order is a caller contract, not enforced in
    // release builds). Returns false and counts the drop in droppedCount() if the queue is
    // already full; the event is not stored and no existing entry is disturbed (no corruption
    // on overflow).
    bool push(const NoteEvent& e) noexcept {
        assert(size_ == 0 || e.sampleOffset >= events_[(head_ + size_ - 1) % Capacity].sampleOffset);

        if (size_ == Capacity) {
            ++droppedCount_;
            return false;
        }

        events_[(head_ + size_) % Capacity] = e;
        ++size_;
        return true;
    }

    // Earliest remaining event (push order), or nullptr if empty.
    const NoteEvent* peek() const noexcept { return size_ == 0 ? nullptr : &events_[head_]; }

    // Precondition: !empty().
    void pop() noexcept {
        assert(size_ > 0);
        head_ = (head_ + 1) % Capacity;
        --size_;
    }

    void clear() noexcept {
        head_ = 0;
        size_ = 0;
        droppedCount_ = 0;
    }

    std::size_t size() const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }

    // Diagnostics: total push() calls that found the queue full since construction or the last
    // clear(). Reset by clear(), not by draining via pop().
    std::uint32_t droppedCount() const noexcept { return droppedCount_; }

    static constexpr std::size_t capacity() noexcept { return Capacity; }

  private:
    std::array<NoteEvent, Capacity> events_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    std::uint32_t droppedCount_ = 0;
};

using BlockEventQueue = EventQueue<256>; // the queue type passed into StringNetwork::process (P1.5)

} // namespace cnpg::dsp

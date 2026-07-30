#include "cnpg/dsp/NoteAllocator.h"

namespace cnpg::dsp {

namespace {

constexpr std::uint8_t kStatusTypeMask = 0xF0u;
constexpr std::uint8_t kNoteOffStatus = 0x80u;
constexpr std::uint8_t kNoteOnStatus = 0x90u;
constexpr std::uint8_t kControlChangeStatus = 0xB0u;

float velocityToUnit(std::uint8_t data2) noexcept { return static_cast<float>(data2) / 127.0f; }

} // namespace

void NoteAllocator::prepare(int numStrings) {
    numStrings_ = numStrings; // P1 scope: full per-string sizing arrives with P2.6
    reset();
}

void NoteAllocator::reset() noexcept {
    stringSounding_ = false;
    soundingChannel_ = 0;
    soundingNote_ = 0;
}

void NoteAllocator::setParams(const NoteAllocatorParams& p) noexcept { params_ = p; }

void NoteAllocator::allocate(const RawMidiEvent* events, int numEvents, BlockEventQueue& outEvents) noexcept {
    for (int i = 0; i < numEvents; ++i) {
        const RawMidiEvent& raw = events[i];
        const std::uint8_t statusType = static_cast<std::uint8_t>(raw.status & kStatusTypeMask);

        if (statusType == kControlChangeStatus)
            continue; // every CC, including CC64, is a no-op until P2.6

        const bool isNoteOff = statusType == kNoteOffStatus || (statusType == kNoteOnStatus && raw.data2 == 0);
        const bool isNoteOn = statusType == kNoteOnStatus && raw.data2 != 0;

        if (isNoteOn) {
            if (raw.data1 < kMinMidiNote || raw.data1 > kMaxMidiNote)
                continue; // outside the playable range: rejected, no NoteEvent emitted

            NoteEvent event{};
            event.type = NoteEventType::NoteOn;
            event.sampleOffset = raw.sampleOffset;
            event.stringIndex = 0; // P1 scope: single monophonic string
            event.channel = raw.channel;
            event.midiNote = raw.data1;
            event.velocity = velocityToUnit(raw.data2);
            event.pluckPosition = 0.5f; // P1: no per-note position source yet; neutral default
            event.hardness = 0.5f;      // P1: no per-note hardness source yet; neutral default

            outEvents.push(event);

            stringSounding_ = true;
            soundingChannel_ = raw.channel;
            soundingNote_ = raw.data1;
        } else if (isNoteOff) {
            // Monophonic last-note priority: only a NoteOff matching the (channel, midiNote)
            // that currently owns string 0 engages the damper; a stale NoteOff -- the note was
            // already displaced by a later NoteOn -- is dropped (docs/plan.md section 2.12).
            if (!stringSounding_ || soundingChannel_ != raw.channel || soundingNote_ != raw.data1)
                continue;

            NoteEvent event{};
            event.type = NoteEventType::NoteOff;
            event.sampleOffset = raw.sampleOffset;
            event.stringIndex = 0;
            event.channel = raw.channel;
            event.midiNote = raw.data1;
            event.velocity = velocityToUnit(raw.data2);
            event.pluckPosition = 0.0f;
            event.hardness = 0.0f;

            outEvents.push(event);
            stringSounding_ = false;
        }
    }
}

int NoteAllocator::stringForNote(std::uint8_t channel, std::uint8_t midiNote) const noexcept {
    return (stringSounding_ && soundingChannel_ == channel && soundingNote_ == midiNote) ? 0 : -1;
}

bool NoteAllocator::sustainActive() const noexcept { return false; }

} // namespace cnpg::dsp

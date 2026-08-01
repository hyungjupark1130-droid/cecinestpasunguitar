#include "cnpg/dsp/NoteAllocator.h"

#include <algorithm>
#include <limits>

namespace cnpg::dsp {

namespace {

constexpr std::uint8_t kStatusTypeMask = 0xF0u;
constexpr std::uint8_t kNoteOffStatus = 0x80u;
constexpr std::uint8_t kNoteOnStatus = 0x90u;
constexpr std::uint8_t kControlChangeStatus = 0xB0u;

float velocityToUnit(std::uint8_t data2) noexcept { return static_cast<float>(data2) / 127.0f; }

} // namespace

void NoteAllocator::prepare(int numStrings) {
    capacityStrings_ = std::clamp(numStrings, 1, kMaxStrings);
    reset();
}

void NoteAllocator::reset() noexcept {
    owned_.fill(false);
    ownerChannel_.fill(0);
    ownerNote_.fill(0);
    heldNoteOff_.fill(false);
    heldVelocity_.fill(0);
    lastTriggerSequence_.fill(0);
    triggerSequence_ = 0;
    sustainDown_ = false;
    unassignableNoteCount_ = 0;
    outOfRangeNoteCount_ = 0;
    unaddressableNoteOffCount_ = 0;
    queueOverflowCount_ = 0;
}

void NoteAllocator::setParams(const NoteAllocatorParams& p) noexcept { params_ = p; }

int NoteAllocator::effectiveStringCount() const noexcept {
    return std::clamp(std::min(capacityStrings_, params_.activeStringCount), 1, kMaxStrings);
}

bool NoteAllocator::stringAddressable(int stringIndex) const noexcept {
    // The one predicate that has to agree with StringNetwork::handleEvent's own two early returns
    // (dsp/src/StringNetwork.cpp: `stringIndex >= numStrings_` and `!perString[i].enabled`). A
    // NoteEvent addressed to a string that fails either is dropped THERE, silently and with no
    // counter anywhere, so this class must never emit one.
    return stringIndex >= 0 && stringIndex < effectiveStringCount() &&
           params_.stringEnabled[static_cast<std::size_t>(stringIndex)];
}

bool NoteAllocator::stringCanPlay(int stringIndex, int midiNote) const noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);
    if (params_.mode == AllocationMode::GuitarFingering) {
        const int open = static_cast<int>(params_.openStringMidiNote[index]);
        return midiNote >= open && midiNote <= open + kFingeringFretSpan;
    }
    // FreeZones: inclusive, and a zone whose low is above its high is simply empty -- no note
    // satisfies both halves, which is the natural spelling of "this string is not in the map".
    const StringZone zone = params_.zones[index];
    return midiNote >= static_cast<int>(zone.lowNote) && midiNote <= static_cast<int>(zone.highNote);
}

int NoteAllocator::chooseString(std::uint8_t channel, std::uint8_t midiNote) const noexcept {
    // 1. AT MOST ONE STRING OWNS A GIVEN (channel, note). If one already does AND that string can
    //    still be addressed, the NoteOn goes back there -- it is a retrigger, and StringNetwork's
    //    RetriggerMode is what decides what that sounds like.
    //
    //    "AND CAN STILL BE ADDRESSED" is the whole of the second clause, and it is not defensive
    //    programming. numStrings and stringEnabled[i] are both automatable APVTS parameters, so the
    //    owning string can leave the active count or be muted between the note-on and its restrike
    //    (docs/listening/P2.6-ableton-checks.md Check B is exactly that gesture). Returning it here
    //    put a NoteEvent on a string StringNetwork::handleEvent then dropped, with NEITHER counter
    //    moving -- the silent drop this class exists to make impossible. allocate() releases such a
    //    stale ownership before calling here, so the reassignment below cannot produce a second
    //    owner and the "at most one" invariant that makes NoteOff matching well defined still holds.
    const int owner = stringForNote(channel, midiNote);
    if (stringAddressable(owner))
        return owner;

    const int count = effectiveStringCount();
    const int note = static_cast<int>(midiNote);

    int bestIdle = -1;
    int bestIdleFret = std::numeric_limits<int>::max();
    std::uint64_t bestIdleSequence = 0;

    int bestSteal = -1;
    std::uint64_t bestStealSequence = 0;

    for (int s = 0; s < count; ++s) {
        const auto index = static_cast<std::size_t>(s);
        if (!params_.stringEnabled[index])
            continue;
        if (!stringCanPlay(s, note))
            continue;

        if (!owned_[index]) {
            // GuitarFingering picks the lowest fret position; FreeZones has no fret, so every
            // candidate ties at 0 and the LRU tie-break decides on its own -- which is exactly what
            // makes overlapping zones alternate under repeated notes.
            const int fret = (params_.mode == AllocationMode::GuitarFingering)
                                 ? note - static_cast<int>(params_.openStringMidiNote[index])
                                 : 0;
            if (bestIdle < 0 || fret < bestIdleFret ||
                (fret == bestIdleFret && lastTriggerSequence_[index] < bestIdleSequence)) {
                bestIdle = s;
                bestIdleFret = fret;
                bestIdleSequence = lastTriggerSequence_[index];
            }
        } else if (bestSteal < 0 || lastTriggerSequence_[index] < bestStealSequence) {
            // The steal policy is LRU only: a candidate that is already playing has no fret
            // preference to express, and displacing the note the player struck longest ago is the
            // one choice that does not depend on where the new note happens to sit.
            bestSteal = s;
            bestStealSequence = lastTriggerSequence_[index];
        }
    }

    return bestIdle >= 0 ? bestIdle : bestSteal;
}

void NoteAllocator::emitNoteOn(int stringIndex, const RawMidiEvent& raw, BlockEventQueue& outEvents) noexcept {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = raw.sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = raw.channel;
    event.midiNote = raw.data1;
    event.velocity = velocityToUnit(raw.data2);
    // Nothing in plain MIDI note-on carries a pluck position or hardness, so the event says so
    // explicitly instead of inventing a value: StringNetwork then resolves both against
    // PluckExciterParams::defaultPosition / ::defaultHardness, which is what makes the APVTS
    // Exciter Position / Exciter Hardness knobs audible. A per-note source (P5 MPE) fills these in
    // with real values without touching this contract.
    event.pluckPosition = kUnspecifiedNoteParam;
    event.hardness = kUnspecifiedNoteParam;

    if (!outEvents.push(event))
        ++queueOverflowCount_;
}

void NoteAllocator::emitNoteOff(int stringIndex, std::int32_t sampleOffset, BlockEventQueue& outEvents) noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);

    NoteEvent event{};
    event.type = NoteEventType::NoteOff;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = ownerChannel_[index];
    event.midiNote = ownerNote_[index];
    event.velocity = velocityToUnit(heldVelocity_[index]);
    event.pluckPosition = kUnspecifiedNoteParam; // NoteOff excites nothing; both are unused
    event.hardness = kUnspecifiedNoteParam;

    if (!outEvents.push(event))
        ++queueOverflowCount_;
}

void NoteAllocator::allocate(const RawMidiEvent* events, int numEvents, BlockEventQueue& outEvents) noexcept {
    for (int i = 0; i < numEvents; ++i) {
        const RawMidiEvent& raw = events[i];
        const std::uint8_t statusType = static_cast<std::uint8_t>(raw.status & kStatusTypeMask);

        if (statusType == kControlChangeStatus) {
            if (raw.data1 != kSustainPedalController)
                continue; // every other CC is somebody else's business

            const bool down = raw.data2 >= kSustainPedalDownThreshold;
            if (down == sustainDown_)
                continue; // a pedal that did not move changes nothing, and re-emitting would double
            sustainDown_ = down;

            if (!down) {
                // PEDAL UP. Every held NoteOff fires at the pedal-release offset, ascending by
                // string index -- one deterministic order, so a six-string chord under the pedal
                // damps in the same order every time and the queue's non-decreasing contract is
                // trivially satisfied (they all share one offset). Iterated over every slot rather
                // than the in-count ones: a string that left the active count while holding a note
                // still has to give it back, or the note is stuck for the life of the instance.
                for (int s = 0; s < kMaxStrings; ++s) {
                    const auto index = static_cast<std::size_t>(s);
                    if (!owned_[index] || !heldNoteOff_[index])
                        continue;
                    // ...and a string that left the count or was muted UNDER the pedal gives its
                    // note back on the counter instead of into an event StringNetwork would
                    // discard. Same rule as the direct NoteOff below, for the same reason, and the
                    // reason this loop still visits every slot rather than only the in-count ones:
                    // the OWNERSHIP has to be released either way.
                    if (stringAddressable(s))
                        emitNoteOff(s, raw.sampleOffset, outEvents);
                    else
                        ++unaddressableNoteOffCount_;
                    owned_[index] = false;
                    heldNoteOff_[index] = false;
                }
            }
            continue;
        }

        const bool isNoteOff = statusType == kNoteOffStatus || (statusType == kNoteOnStatus && raw.data2 == 0);
        const bool isNoteOn = statusType == kNoteOnStatus && raw.data2 != 0;

        if (isNoteOn) {
            if (raw.data1 < kMinMidiNote || raw.data1 > kMaxMidiNote) {
                // Outside the design envelope: rejected, no NoteEvent emitted -- and COUNTED, on its
                // own counter. Counted because "everything it cannot do it COUNTS" has to include
                // this one: cnpg_render fails a corpus on a non-zero drop counter, and before this
                // counter existed a corpus containing MIDI 20 or 109 would have rendered clean.
                // On its OWN counter because the fix a reader has to make differs: this says the
                // INPUT is outside the instrument's 21..108 design envelope, while
                // unassignableNoteCount() says the note was inside it and the tuning or zone table
                // had nowhere to put it.
                ++outOfRangeNoteCount_;
                continue;
            }

            // A stale ownership held by a string that can no longer be addressed is released here,
            // BEFORE the choice, so chooseString() reassigns the note somewhere audible instead of
            // handing it back to a string StringNetwork will drop it on. Releasing first is what
            // keeps "at most one string owns a given (channel, note)" true across the reassignment.
            const int staleOwner = stringForNote(raw.channel, raw.data1);
            if (staleOwner >= 0 && !stringAddressable(staleOwner)) {
                const auto staleIndex = static_cast<std::size_t>(staleOwner);
                // ...and if that ownership was carrying a CC64-HELD NoteOff, the NoteOff dies here.
                // It is the same event as the two sites above and below -- a note-off whose string
                // was automated out from under it -- so it is counted on the same counter, and
                // NoteAllocator.h's "every way an emitted event can fail to arrive is counted" stays
                // true rather than having to be narrowed around this line. Nothing is stuck either
                // way (the string is being ramped silent by StringNetwork regardless, and the note
                // is about to be reassigned), but an uncounted discard is exactly the class of
                // silence this whole counter set exists to break.
                if (heldNoteOff_[staleIndex])
                    ++unaddressableNoteOffCount_;
                owned_[staleIndex] = false;
                heldNoteOff_[staleIndex] = false;
            }

            const int target = chooseString(raw.channel, raw.data1);
            if (target < 0) {
                // Unassignable: no enabled in-count string can play this note. Dropped silently and
                // COUNTED -- see unassignableNoteCount().
                ++unassignableNoteCount_;
                continue;
            }

            const auto index = static_cast<std::size_t>(target);
            emitNoteOn(target, raw, outEvents);

            // Ownership moves to the new note. If a note was displaced (a steal) it loses its
            // ownership here and no NoteOff is synthesized for it, so its own later NoteOff -- and
            // any CC64-held NoteOff it had -- becomes stale and is dropped. If the SAME note is
            // being restruck under the pedal, this is what cancels its pending NoteOff.
            owned_[index] = true;
            ownerChannel_[index] = raw.channel;
            ownerNote_[index] = raw.data1;
            heldNoteOff_[index] = false;
            heldVelocity_[index] = 0;
            lastTriggerSequence_[index] = ++triggerSequence_;
        } else if (isNoteOff) {
            const int owner = stringForNote(raw.channel, raw.data1);
            if (owner < 0)
                continue; // stale: the note no longer owns a string (stolen, or never assigned)

            const auto index = static_cast<std::size_t>(owner);
            if (!stringAddressable(owner)) {
                // THE NoteOff HALF of the restrike bug fixes wave 1 fixed and reported. The owning
                // string left the active count or was muted while the note was held, so
                // StringNetwork::handleEvent would discard this event at its own early returns with
                // nothing moving anywhere. There is no reassignment available here -- a NoteOff is
                // not a note looking for somewhere to sound, it is the end of one -- so the answer
                // is the counter, and the ownership is released exactly as it would have been.
                //
                // Not emitted at all, rather than emitted-and-discarded: emitting occupies a queue
                // slot that a real event may need, and an event this class KNOWS will be dropped is
                // one it should not push. Releasing the ownership here is what keeps the string free
                // for the next note and keeps a later stale NoteOff for the same (channel, note)
                // from finding an owner.
                //
                // The string is silent either way -- StringNetwork ramps a removed or disabled
                // string out over kEnableRampSeconds and clears sounding_/releasing_ when the ramp
                // lands on zero -- so nothing is stuck. What was missing was the diagnostic.
                ++unaddressableNoteOffCount_;
                owned_[index] = false;
                heldNoteOff_[index] = false;
                continue;
            }
            if (sustainDown_) {
                // Held, not emitted. The string stays OWNED: the note is still ringing, which is
                // what the pedal is for, so the string is not free for a later NoteOn to take
                // without stealing it.
                heldNoteOff_[index] = true;
                heldVelocity_[index] = raw.data2;
                continue;
            }

            heldVelocity_[index] = raw.data2;
            emitNoteOff(owner, raw.sampleOffset, outEvents);
            owned_[index] = false;
            heldNoteOff_[index] = false;
        }
    }
}

int NoteAllocator::stringForNote(std::uint8_t channel, std::uint8_t midiNote) const noexcept {
    for (int s = 0; s < kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        if (owned_[index] && ownerChannel_[index] == channel && ownerNote_[index] == midiNote)
            return s;
    }
    return -1;
}

bool NoteAllocator::sustainActive() const noexcept { return sustainDown_; }

std::uint32_t NoteAllocator::unassignableNoteCount() const noexcept { return unassignableNoteCount_; }

std::uint32_t NoteAllocator::outOfRangeNoteCount() const noexcept { return outOfRangeNoteCount_; }

std::uint32_t NoteAllocator::unaddressableNoteOffCount() const noexcept { return unaddressableNoteOffCount_; }

std::uint32_t NoteAllocator::queueOverflowCount() const noexcept { return queueOverflowCount_; }

int NoteAllocator::ownedNote(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return -1;
    const auto index = static_cast<std::size_t>(stringIndex);
    return owned_[index] ? static_cast<int>(ownerNote_[index]) : -1;
}

int NoteAllocator::ownedChannel(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return -1;
    const auto index = static_cast<std::size_t>(stringIndex);
    return owned_[index] ? static_cast<int>(ownerChannel_[index]) : -1;
}

bool NoteAllocator::sustainHoldPending(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return false;
    const auto index = static_cast<std::size_t>(stringIndex);
    return owned_[index] && heldNoteOff_[index];
}

} // namespace cnpg::dsp

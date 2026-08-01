#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/NoteAllocator.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

// NoteAllocatorFingeringTests -- docs/plan.md Task P2.6, AllocationMode::GuitarFingering.
//
// The mode's whole job is to answer "which string would a player have used?" for a stream of MIDI
// notes that carries no such information, and the acceptance criteria are stated as three concrete
// musical facts: E2 lands on the low E string unfretted; an open C-major chord spreads across five
// distinct strings at minimal fret positions; a seventh simultaneous note has to take a string off
// somebody, and it takes it off whoever struck longest ago.
//
// Every case in this file also asserts ALL THREE diagnostics counters. They are the only evidence a
// drop ever leaves -- a NoteOn that produced no NoteEvent sounds exactly like a note nobody played
// -- so a counter that is only ever read in the one case that expects it is a counter nobody is
// checking.

namespace {

using cnpg::dsp::AllocationMode;
using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteAllocator;
using cnpg::dsp::NoteAllocatorParams;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::RawMidiEvent;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;

RawMidiEvent noteOn(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t velocity = 100,
                    std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x90u | channel), note, velocity, channel};
}

RawMidiEvent noteOff(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x80u | channel), note, 0, channel};
}

// The SHIPPED configuration: six strings, EADGBE, guitar fingering. Spelled out rather than left to
// the struct defaults because the defaults describe an EIGHT-string instrument (slots 6 and 7 carry
// the extended-range low B and F#), and every musical claim in this file is a six-string claim.
NoteAllocatorParams sixStringGuitar() {
    NoteAllocatorParams params;
    params.mode = AllocationMode::GuitarFingering;
    params.activeStringCount = 6;
    return params;
}

// The one thing that must be true of every case that is not about a drop.
void requireNoDrops(const NoteAllocator& allocator) {
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.outOfRangeNoteCount() == 0);
    REQUIRE(allocator.unaddressableNoteOffCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

std::vector<NoteEvent> drain(BlockEventQueue& queue) {
    std::vector<NoteEvent> events;
    std::int32_t previousOffset = -1;
    while (const NoteEvent* event = queue.peek()) {
        // The queue's own contract, asserted on every drain in this file rather than in one case:
        // push order must be non-decreasing in sampleOffset, and a Debug build asserts it inside
        // push(). Checking it here as well means the Release suite -- which is what CI runs -- also
        // sees a violation instead of only the Debug one nobody builds.
        REQUIRE(event->sampleOffset >= previousOffset);
        previousOffset = event->sampleOffset;
        events.push_back(*event);
        queue.pop();
    }
    return events;
}

} // namespace

TEST_CASE("CONTRACT: GuitarFingering puts E2 on the low E string, unfretted", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 40)};
    BlockEventQueue out;
    allocator.allocate(events, 1, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 1);
    REQUIRE(emitted[0].stringIndex == 0);
    REQUIRE(emitted[0].midiNote == 40);
    REQUIRE(allocator.stringForNote(0, 40) == 0);
    REQUIRE(allocator.ownedNote(0) == 40);

    // Fret 0, and asserted as the FRET rather than as "string 0": string 0 is also the only
    // candidate for MIDI 40, so an assignment policy that had no fret preference at all would pass
    // a string-index check here and fail the chord case below. Stating the fret is what makes this
    // case about fingering.
    const int fret = 40 - static_cast<int>(sixStringGuitar().openStringMidiNote[0]);
    REQUIRE(fret == 0);
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: GuitarFingering spreads an open C major over five strings at minimal frets", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    const NoteAllocatorParams params = sixStringGuitar();
    allocator.setParams(params);

    // C E G C E, struck as a strum from the lowest note up -- the order a player produces them in,
    // which matters: the policy is sequential and a chord delivered high-to-low is a different
    // question (asserted separately below).
    const RawMidiEvent chord[] = {noteOn(0, 48), noteOn(6, 52), noteOn(12, 55), noteOn(18, 60), noteOn(24, 64)};
    BlockEventQueue out;
    allocator.allocate(chord, 5, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 5);

    std::array<int, 5> assigned{};
    for (std::size_t i = 0; i < emitted.size(); ++i)
        assigned[i] = static_cast<int>(emitted[i].stringIndex);

    // FIVE DISTINCT STRINGS. Checked as a set rather than by five equality assertions, so the claim
    // in the acceptance criterion ("five distinct strings") is the thing being tested.
    std::array<int, 5> sorted = assigned;
    std::sort(sorted.begin(), sorted.end());
    REQUIRE(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());

    // MINIMAL FRET POSITIONS, printed as the fingering they are. The policy is not told what an
    // open C major looks like; it is told to take the lowest fret among idle candidates. What comes
    // out is x32010 -- the actual open C-major shape a guitarist plays -- which is the strongest
    // evidence available that the rule is the right rule and not merely a rule.
    static const int kExpectedString[5] = {1, 2, 3, 4, 5};
    static const int kExpectedFret[5] = {3, 2, 0, 1, 0};
    static const int kNote[5] = {48, 52, 55, 60, 64};

    std::cout << "[contract] GuitarFingering, open C major (48 52 55 60 64):";
    for (std::size_t i = 0; i < emitted.size(); ++i) {
        const int stringIndex = assigned[i];
        const int fret = kNote[i] - static_cast<int>(params.openStringMidiNote[static_cast<std::size_t>(stringIndex)]);
        std::cout << " " << kNote[i] << "->s" << stringIndex << "/fret" << fret;
        INFO("note " << kNote[i]);
        REQUIRE(stringIndex == kExpectedString[i]);
        REQUIRE(fret == kExpectedFret[i]);
        REQUIRE(fret >= 0);
        REQUIRE(fret <= cnpg::dsp::kFingeringFretSpan);
        REQUIRE(allocator.stringForNote(0, static_cast<std::uint8_t>(kNote[i])) == stringIndex);
    }
    std::cout << "  (= x32010, the open C shape)\n";

    // NON-VACUITY. Every one of these notes had somewhere else to go -- string 0 alone spans MIDI
    // 40..64 and therefore contains all five -- so a policy with no fret preference at all would
    // have put the whole chord on string 0 (with four steals) and would still have emitted five
    // events. What separates that from what happened is the FRET, which is why this case asserts
    // frets and why it asserts here that the alternative really existed.
    for (std::size_t i = 0; i < emitted.size(); ++i) {
        int candidates = 0;
        for (int s = 0; s < 6; ++s) {
            const int open = static_cast<int>(params.openStringMidiNote[static_cast<std::size_t>(s)]);
            const int fret = kNote[i] - open;
            if (fret >= 0 && fret <= cnpg::dsp::kFingeringFretSpan)
                ++candidates;
        }
        INFO("note " << kNote[i] << " had " << candidates << " candidate strings");
        REQUIRE(candidates >= 2);
        // ...and specifically, string 0 could have taken it. A lowest-index policy would have.
        const int fretOnString0 = kNote[i] - static_cast<int>(params.openStringMidiNote[0]);
        REQUIRE(fretOnString0 >= 0);
        REQUIRE(fretOnString0 <= cnpg::dsp::kFingeringFretSpan);
        REQUIRE(assigned[i] != 0);
    }

    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: GuitarFingering steals the least-recently-triggered candidate for a seventh note", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    // Six notes filling all six strings, then a seventh with nowhere idle to go.
    const RawMidiEvent six[] = {noteOn(0, 48),  // -> string 1 (first triggered)
                                noteOn(1, 52),  // -> string 2
                                noteOn(2, 55),  // -> string 3
                                noteOn(3, 60),  // -> string 4
                                noteOn(4, 64),  // -> string 5
                                noteOn(5, 43)}; // -> string 0 (last triggered)
    BlockEventQueue out;
    allocator.allocate(six, 6, out);
    REQUIRE(drain(out).size() == 6);

    // IN THE STATE THIS CASE CLAIMS TO EXERCISE: every string owns a note, so no idle candidate can
    // exist and the steal path is the only path left. Asserted, not assumed.
    for (int s = 0; s < 6; ++s) {
        INFO("string " << s);
        REQUIRE(allocator.ownedNote(s) >= 0);
    }

    // MIDI 57 (A3) fits strings 0..3 and not 4 or 5 (their open notes are above it). Among those
    // four, string 1 was triggered FIRST -- it took the chord's first note -- so it is the one the
    // steal must take, even though string 0 was also a candidate and is the one a "lowest index" or
    // "highest fret" rule would pick.
    const RawMidiEvent seventh[] = {noteOn(10, 57)};
    BlockEventQueue stolenOut;
    allocator.allocate(seventh, 1, stolenOut);
    const std::vector<NoteEvent> stolen = drain(stolenOut);

    REQUIRE(stolen.size() == 1); // the NEW note only: no NoteOff is synthesized for the displaced one
    REQUIRE(stolen[0].type == NoteEventType::NoteOn);
    REQUIRE(stolen[0].stringIndex == 1);
    std::cout << "[contract] GuitarFingering steal: MIDI 57 took string " << static_cast<int>(stolen[0].stringIndex)
              << " from MIDI 48 (candidates were strings 0-3; 0 was the most recently triggered)\n";

    // The displaced note lost its string, so it is no longer anywhere.
    REQUIRE(allocator.stringForNote(0, 48) == -1);
    REQUIRE(allocator.stringForNote(0, 57) == 1);
    REQUIRE(allocator.ownedNote(1) == 57);

    // ...and its NoteOff is therefore stale and produces nothing, while the stealing note's does.
    const RawMidiEvent offs[] = {noteOff(20, 48), noteOff(30, 57)};
    BlockEventQueue offOut;
    allocator.allocate(offs, 2, offOut);
    const std::vector<NoteEvent> emittedOffs = drain(offOut);
    REQUIRE(emittedOffs.size() == 1);
    REQUIRE(emittedOffs[0].type == NoteEventType::NoteOff);
    REQUIRE(emittedOffs[0].midiNote == 57);
    REQUIRE(emittedOffs[0].sampleOffset == 30);

    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a stale NoteOff engages no damper and the stealing note's does", "[contract]") {
    // The acceptance criterion end to end, through the real network rather than only through the
    // allocator's bookkeeping: NoteOn A -> steal via NoteOn B on the same string -> NoteOff A must
    // produce no damper engagement, and NoteOff B must.
    //
    // One string, so B has nowhere to go but A's string and the steal is forced.
    NoteAllocator allocator;
    allocator.prepare(1);
    NoteAllocatorParams params = sixStringGuitar();
    params.activeStringCount = 1;
    allocator.setParams(params);

    StringNetworkParams networkParams;
    networkParams.bridge.couplingStrength = 0.0f; // one string; nothing to couple to
    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(networkParams);
    network.reset();

    auto pump = [&](const RawMidiEvent* raw, int count, int blocks) {
        BlockEventQueue queue;
        allocator.allocate(raw, count, queue);
        for (int b = 0; b < blocks; ++b)
            network.process(queue, kBlock);
    };

    const RawMidiEvent noteA[] = {noteOn(0, 60)};
    pump(noteA, 1, 100);
    REQUIRE(network.damperEngagement(0) == 0.0f);
    REQUIRE(network.energyEstimate() > 0.0); // A really is ringing

    const RawMidiEvent noteB[] = {noteOn(0, 64)};
    pump(noteB, 1, 20);
    REQUIRE(allocator.stringForNote(0, 60) == -1); // A was displaced
    REQUIRE(allocator.stringForNote(0, 64) == 0);

    // NoteOff A: stale. The allocator emits nothing, so the damper never hears about it.
    const RawMidiEvent offA[] = {noteOff(0, 60)};
    BlockEventQueue staleQueue;
    allocator.allocate(offA, 1, staleQueue);
    REQUIRE(staleQueue.empty()); // asserted at the allocator AND at the damper below
    for (int b = 0; b < 40; ++b)
        network.process(staleQueue, kBlock);
    REQUIRE(network.damperEngagement(0) == 0.0f);
    const double energyAfterStaleOff = network.energyEstimate();
    REQUIRE(energyAfterStaleOff > 0.0);

    // NoteOff B: the note that owns the string. The felt comes down.
    const RawMidiEvent offB[] = {noteOff(0, 64)};
    pump(offB, 1, 20);
    const float engagement = network.damperEngagement(0);
    std::cout << "[contract] stale NoteOff: engagement after NoteOff A " << 0.0f << ", after NoteOff B " << engagement
              << "\n";
    REQUIRE(engagement > 0.0f);

    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a NoteOn with velocity 0 is a NoteOff", "[contract]") {
    // Implemented since Task P1.2 and never asserted -- the P1.2 ledger flagged it. It matters
    // because many hosts and most MIDI files use running-status note-ons with velocity 0 as their
    // ONLY note-off, so a build that ignored this would hold every note forever in those sessions
    // and be perfectly fine in the others.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 60, 100), noteOn(40, 60, 0)}; // 0x90 with velocity 0
    BlockEventQueue out;
    allocator.allocate(events, 2, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 2);
    REQUIRE(emitted[0].type == NoteEventType::NoteOn);
    REQUIRE(emitted[1].type == NoteEventType::NoteOff);
    REQUIRE(emitted[1].sampleOffset == 40);
    REQUIRE(allocator.stringForNote(0, 60) == -1); // the string was genuinely released
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a same-pitch NoteOn returns to the string that already owns it", "[contract]") {
    // The P1.2 ledger's third untested item. A second NoteOn for a note nobody released is a
    // retrigger, and it has to land where the note already is: if it opened a SECOND copy on
    // another string, stringForNote() would have two answers and the eventual NoteOff would release
    // only one of them -- a note that never stops, on an instrument with strings to spare.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent first[] = {noteOn(0, 60)};
    BlockEventQueue firstOut;
    allocator.allocate(first, 1, firstOut);
    const std::vector<NoteEvent> firstEmitted = drain(firstOut);
    REQUIRE(firstEmitted.size() == 1);
    const int owner = static_cast<int>(firstEmitted[0].stringIndex);
    REQUIRE(owner == 4); // MIDI 60 fingers cheapest on the B string, fret 1

    // IN STATE: the note is still held, and five other strings are idle and could take it.
    REQUIRE(allocator.ownedNote(owner) == 60);
    int idleCandidates = 0;
    for (int s = 0; s < 6; ++s)
        if (s != owner && allocator.ownedNote(s) < 0)
            ++idleCandidates;
    REQUIRE(idleCandidates == 5);

    const RawMidiEvent again[] = {noteOn(64, 60)};
    BlockEventQueue againOut;
    allocator.allocate(again, 1, againOut);
    const std::vector<NoteEvent> againEmitted = drain(againOut);
    REQUIRE(againEmitted.size() == 1);
    REQUIRE(againEmitted[0].type == NoteEventType::NoteOn);
    REQUIRE(static_cast<int>(againEmitted[0].stringIndex) == owner);

    // One string owns it, and only one.
    int owners = 0;
    for (int s = 0; s < cnpg::dsp::kMaxStrings; ++s)
        if (allocator.ownedNote(s) == 60)
            ++owners;
    REQUIRE(owners == 1);

    // ...so ONE NoteOff releases it.
    const RawMidiEvent release[] = {noteOff(100, 60)};
    BlockEventQueue releaseOut;
    allocator.allocate(release, 1, releaseOut);
    REQUIRE(drain(releaseOut).size() == 1);
    REQUIRE(allocator.stringForNote(0, 60) == -1);
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: the same pitch on two channels is two notes on two strings", "[contract]") {
    // Ownership is keyed on (channel, note), not on note. Through P2 the channel is carried
    // opaquely and no host uses it to mean anything, but the P5 MPE seam turns channel into the
    // per-note key, and an allocator that collapsed two channels onto one entry would make that
    // change a rewrite instead of a routing decision.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 60, 100, 0), noteOn(4, 60, 100, 3)};
    BlockEventQueue out;
    allocator.allocate(events, 2, out);
    const std::vector<NoteEvent> emitted = drain(out);

    REQUIRE(emitted.size() == 2);
    REQUIRE(emitted[0].channel == 0);
    REQUIRE(emitted[1].channel == 3);
    REQUIRE(emitted[0].stringIndex != emitted[1].stringIndex);
    REQUIRE(allocator.stringForNote(0, 60) == static_cast<int>(emitted[0].stringIndex));
    REQUIRE(allocator.stringForNote(3, 60) == static_cast<int>(emitted[1].stringIndex));
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a note no string can finger is dropped and counted", "[contract]") {
    // GuitarFingering's designed-in failure. A six-string EADGBE instrument cannot play MIDI 30 --
    // that is not a bug, it is what a guitar is -- so the allocator says so instead of transposing
    // the note somewhere the player did not ask for. The whole of the evidence it leaves is the
    // counter, which is why this case asserts the counter and every other case asserts it is zero.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 30), noteOn(10, 100), noteOn(20, 40)};
    BlockEventQueue out;
    allocator.allocate(events, 3, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 1); // only the playable one
    REQUIRE(emitted[0].midiNote == 40);

    // 30 is below the low E; 100 is above the high E's 24th fret (64 + 24 = 88). Both are inside
    // kMinMidiNote..kMaxMidiNote, so the design-envelope check did not reject them -- this really
    // is the assignment policy declining, which is what the counter is for.
    REQUIRE(allocator.unassignableNoteCount() == 2);
    REQUIRE(allocator.unaddressableNoteOffCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
    REQUIRE(allocator.stringForNote(0, 30) == -1);
    REQUIRE(allocator.stringForNote(0, 100) == -1);
    std::cout << "[contract] unassignable under 6-string EADGBE: MIDI 30 and MIDI 100 both dropped, counter reads "
              << allocator.unassignableNoteCount() << "\n";
}

TEST_CASE("CONTRACT: a disabled string is not a candidate", "[contract]") {
    // The allocator and StringNetwork have to agree about which strings exist, because
    // StringNetwork::handleEvent silently drops an event addressed to a disabled string: assigning
    // to one would be a note that vanishes with no counter moving anywhere.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    NoteAllocatorParams params = sixStringGuitar();
    allocator.setParams(params);

    const RawMidiEvent enabled[] = {noteOn(0, 60)};
    BlockEventQueue enabledOut;
    allocator.allocate(enabled, 1, enabledOut);
    REQUIRE(drain(enabledOut)[0].stringIndex == 4); // the cheapest fingering
    allocator.reset();

    params.stringEnabled[4] = false;
    allocator.setParams(params);

    const RawMidiEvent muted[] = {noteOn(0, 60)};
    BlockEventQueue mutedOut;
    allocator.allocate(muted, 1, mutedOut);
    const std::vector<NoteEvent> emitted = drain(mutedOut);
    REQUIRE(emitted.size() == 1);
    REQUIRE(emitted[0].stringIndex == 3); // the next cheapest, fret 5 on the G string
    requireNoDrops(allocator);

    // ...and with every candidate muted the note is unassignable rather than assigned into silence.
    allocator.reset();
    for (auto& flag : params.stringEnabled)
        flag = false;
    allocator.setParams(params);
    const RawMidiEvent none[] = {noteOn(0, 60)};
    BlockEventQueue noneOut;
    allocator.allocate(none, 1, noneOut);
    REQUIRE(noneOut.empty());
    REQUIRE(allocator.unassignableNoteCount() == 1);
    REQUIRE(allocator.unaddressableNoteOffCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

TEST_CASE("CONTRACT: a restrike whose string left the count is reassigned, never dropped in silence", "[contract]") {
    // THE FAILURE THIS PINS, and it shipped. "At most one string owns a given (channel, note)" was
    // implemented by searching every slot and returning the owner WHEREVER it was -- including a
    // slot outside the active count or behind a false stringEnabled. StringNetwork::handleEvent
    // drops such an event on the floor (its `stringIndex >= numStrings_` and `!perString[i].enabled`
    // early returns), so the restrike produced no sound and NEITHER counter moved: a NoteOn that
    // vanished with every diagnostic reading healthy, which is precisely what this class's whole
    // diagnostics surface exists to make impossible.
    //
    // Reachable by ordinary use rather than by a contrived state. `numStrings` and `stringEnabled[i]`
    // are both shipped automatable APVTS parameters, and docs/listening/P2.6-ableton-checks.md
    // Check B is a count-riding gesture over held notes.
    //
    // The fix is not "count it as a drop": it is to RELEASE the unreachable ownership and reassign,
    // because a string the player can no longer hear is not a string that still owns a note. The
    // drop counter is the answer only when no reachable string can play the note at all -- the third
    // section below.
    SECTION("the active count shrinks under a held note") {
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params; // the 8-string default table
        allocator.setParams(params);

        const RawMidiEvent first[] = {noteOn(0, 60)};
        BlockEventQueue firstOut;
        allocator.allocate(first, 1, firstOut);
        const std::vector<NoteEvent> firstEmitted = drain(firstOut);
        REQUIRE(firstEmitted.size() == 1);
        REQUIRE(firstEmitted[0].stringIndex == 4); // fret 1 on the B string of the 8-string default
        REQUIRE(allocator.ownedNote(4) == 60);

        // The count rides down while the note is held -- Check B's gesture.
        params.activeStringCount = 3;
        allocator.setParams(params);
        // NON-VACUITY: the owning string really is outside the count now, which is the whole
        // premise. Asserted before the restrike, not inferred from its result.
        REQUIRE(allocator.effectiveStringCount() == 3);
        REQUIRE(4 >= allocator.effectiveStringCount());

        const RawMidiEvent restrike[] = {noteOn(64, 60)};
        BlockEventQueue restrikeOut;
        allocator.allocate(restrike, 1, restrikeOut);
        const std::vector<NoteEvent> emitted = drain(restrikeOut);
        REQUIRE(emitted.size() == 1);
        // Reassigned to the cheapest fingering INSIDE the count (fret 10 on string 2), and the
        // unreachable ownership is gone -- so the "at most one string owns a (channel, note)"
        // invariant that makes NoteOff matching well defined still holds.
        REQUIRE(emitted[0].stringIndex == 2);
        REQUIRE(allocator.ownedNote(4) == -1);
        REQUIRE(allocator.ownedNote(2) == 60);
        REQUIRE(allocator.stringForNote(0, 60) == 2);
        requireNoDrops(allocator);

        // THE DROP WAS REAL, not notional. The same NoteOn addressed to string 4 -- which is exactly
        // what the pre-fix path emitted -- is silently discarded by a three-string network, while
        // the reassigned one sounds. Same event, same network configuration, one field different.
        auto renderOne = [](const NoteEvent& event) {
            StringNetworkParams networkParams;
            networkParams.bridge.couplingStrength = 0.0f;
            StringNetwork<float> network;
            network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
            network.setNumStrings(3);
            network.setParams(networkParams);
            network.reset();
            BlockEventQueue queue;
            queue.push(event);
            for (int b = 0; b < 40; ++b)
                network.process(queue, kBlock);
            return network.energyEstimate();
        };
        NoteEvent addressedToTheLostString = emitted[0];
        addressedToTheLostString.stringIndex = 4;
        REQUIRE(renderOne(addressedToTheLostString) == 0.0); // what shipped: nothing at all
        REQUIRE(renderOne(emitted[0]) > 0.0);                // what ships now

        // ...and the note can still be stopped. A restrike that vanished also took its NoteOff with
        // it, because the NoteOff would have gone to the same unreachable string.
        const RawMidiEvent release[] = {noteOff(96, 60)};
        BlockEventQueue releaseOut;
        allocator.allocate(release, 1, releaseOut);
        const std::vector<NoteEvent> released = drain(releaseOut);
        REQUIRE(released.size() == 1);
        REQUIRE(released[0].stringIndex == 2);
        REQUIRE(allocator.stringForNote(0, 60) == -1);
        requireNoDrops(allocator);
    }

    SECTION("the owning string is muted under a held note") {
        // The same failure through the other automatable door. NOTE that the pre-existing "a
        // disabled string is not a candidate" case calls reset() before flipping the flag, which is
        // what kept this path unexercised: reset() drops the ownership the bug needed.
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params = sixStringGuitar();
        allocator.setParams(params);

        const RawMidiEvent first[] = {noteOn(0, 60)};
        BlockEventQueue firstOut;
        allocator.allocate(first, 1, firstOut);
        REQUIRE(drain(firstOut)[0].stringIndex == 4);
        REQUIRE(allocator.ownedNote(4) == 60);

        params.stringEnabled[4] = false; // no reset(): the ownership survives the mute
        allocator.setParams(params);

        const RawMidiEvent restrike[] = {noteOn(64, 60)};
        BlockEventQueue restrikeOut;
        allocator.allocate(restrike, 1, restrikeOut);
        const std::vector<NoteEvent> emitted = drain(restrikeOut);
        REQUIRE(emitted.size() == 1);
        REQUIRE(emitted[0].stringIndex == 3); // fret 5 on the G string, the next cheapest
        REQUIRE(allocator.ownedNote(4) == -1);
        REQUIRE(allocator.stringForNote(0, 60) == 3);
        requireNoDrops(allocator);
    }

    SECTION("no reachable string can play it -- then, and only then, it is a counted drop") {
        // The reassignment is not a licence to put the note anywhere. MIDI 88 is the 24th fret of
        // the high E and nothing else on a six-string EADGBE can reach it, so when the count leaves
        // that string behind there is genuinely nowhere to go -- and the answer is the counter, not
        // silence.
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params = sixStringGuitar();
        allocator.setParams(params);

        const RawMidiEvent first[] = {noteOn(0, 88)};
        BlockEventQueue firstOut;
        allocator.allocate(first, 1, firstOut);
        REQUIRE(drain(firstOut)[0].stringIndex == 5);
        REQUIRE(allocator.ownedNote(5) == 88);

        params.activeStringCount = 3;
        allocator.setParams(params);

        const RawMidiEvent restrike[] = {noteOn(64, 88)};
        BlockEventQueue restrikeOut;
        allocator.allocate(restrike, 1, restrikeOut);
        REQUIRE(restrikeOut.empty());
        REQUIRE(allocator.unassignableNoteCount() == 1);
        REQUIRE(allocator.outOfRangeNoteCount() == 0); // 88 is well inside 21..108
        REQUIRE(allocator.unaddressableNoteOffCount() == 0);
        REQUIRE(allocator.queueOverflowCount() == 0);
        REQUIRE(allocator.stringForNote(0, 88) == -1);
        REQUIRE(allocator.ownedNote(5) == -1);
        std::cout << "[contract] restrike onto a string the count left behind: MIDI 60 reassigned inside the count, "
                     "MIDI 88 counted unassignable (counter reads "
                  << allocator.unassignableNoteCount() << ")\n";
    }
}

TEST_CASE("CONTRACT: a NoteOff whose string left the count is counted, never emitted into nothing", "[contract]") {
    // THE OTHER HALF of the case above, reported unfixed by fixes wave 1 and closed here. The
    // restrike half had somewhere to go -- a NoteOn is a note looking for a string, so it is
    // REASSIGNED. A NoteOff is not: it is the end of a note, and the only string it could ever be
    // addressed to is the one that has just become unreachable. So the answer is the counter.
    //
    // Why it matters rather than being bookkeeping: a NoteEvent addressed to a string outside the
    // active count (or behind a false stringEnabled) is discarded by StringNetwork::handleEvent at
    // its own early returns, with no counter moving anywhere. That is the same silent drop the
    // restrike half was, on the same automatable gesture (docs/listening/P2.6-ableton-checks.md
    // Check B rides numStrings under held notes), and it sits directly next to the wave-2 ownership
    // finding -- StringNetwork's "owns a note" is now `sounding_` alone, exactly the flag this
    // undelivered NoteOff would have cleared.

    SECTION("a direct NoteOff after the count shrank") {
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params = sixStringGuitar();
        allocator.setParams(params);

        const RawMidiEvent on[] = {noteOn(0, 60)};
        BlockEventQueue onOut;
        allocator.allocate(on, 1, onOut);
        REQUIRE(drain(onOut)[0].stringIndex == 4); // fret 1 on the B string
        REQUIRE(allocator.ownedNote(4) == 60);
        requireNoDrops(allocator);

        params.activeStringCount = 3;
        allocator.setParams(params);
        // NON-VACUITY, before the NoteOff rather than inferred from it: the owning string really is
        // unreachable now.
        REQUIRE(allocator.effectiveStringCount() == 3);
        REQUIRE(4 >= allocator.effectiveStringCount());

        const RawMidiEvent off[] = {noteOff(64, 60)};
        BlockEventQueue offOut;
        allocator.allocate(off, 1, offOut);

        // No event -- and that is the FIX, not the bug: what shipped emitted one here and
        // StringNetwork threw it away with every diagnostic reading healthy.
        REQUIRE(offOut.empty());
        REQUIRE(allocator.unaddressableNoteOffCount() == 1);
        REQUIRE(allocator.unassignableNoteCount() == 0); // a NoteOff is never "unassignable"
        REQUIRE(allocator.outOfRangeNoteCount() == 0);   // 60 is well inside 21..108
        REQUIRE(allocator.queueOverflowCount() == 0);    // the queue was empty; nothing was refused
        // The ownership is released all the same, so the string is free for the next note and a
        // second NoteOff for the same note finds no owner and does not double-count.
        REQUIRE(allocator.ownedNote(4) == -1);
        REQUIRE(allocator.stringForNote(0, 60) == -1);
        allocator.allocate(off, 1, offOut);
        REQUIRE(offOut.empty());
        REQUIRE(allocator.unaddressableNoteOffCount() == 1);

        std::cout << "[contract] NoteOff to a string the count left behind: no event emitted, "
                  << "unaddressableNoteOffCount reads " << allocator.unaddressableNoteOffCount()
                  << ", ownership released (ownedNote(4) == " << allocator.ownedNote(4) << ")\n";
    }

    SECTION("the owning string is muted instead") {
        // The other automatable door, same as the restrike case's second section, and again with no
        // reset() -- reset() would drop the ownership the path needs.
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params = sixStringGuitar();
        allocator.setParams(params);

        const RawMidiEvent on[] = {noteOn(0, 60)};
        BlockEventQueue onOut;
        allocator.allocate(on, 1, onOut);
        REQUIRE(drain(onOut)[0].stringIndex == 4);

        params.stringEnabled[4] = false;
        allocator.setParams(params);

        const RawMidiEvent off[] = {noteOff(64, 60)};
        BlockEventQueue offOut;
        allocator.allocate(off, 1, offOut);
        REQUIRE(offOut.empty());
        REQUIRE(allocator.unaddressableNoteOffCount() == 1);
        REQUIRE(allocator.ownedNote(4) == -1);
    }

    SECTION("a CC64-held NoteOff released by a pedal-up") {
        // The pedal-up loop deliberately visits EVERY slot, including strings outside the count,
        // because the ownership has to come back or the note is stuck for the life of the instance.
        // That is still true; what changes is that the ones it cannot address are counted instead of
        // emitted. Both halves are asserted in one go: string 4 leaves the count under the pedal,
        // string 1 does not, and the pedal-up emits exactly one event.
        NoteAllocator allocator;
        allocator.prepare(cnpg::dsp::kMaxStrings);
        NoteAllocatorParams params = sixStringGuitar();
        allocator.setParams(params);

        const RawMidiEvent chord[] = {noteOn(0, 60), noteOn(1, 46)};
        BlockEventQueue chordOut;
        allocator.allocate(chord, 2, chordOut);
        const std::vector<NoteEvent> struck = drain(chordOut);
        REQUIRE(struck.size() == 2);
        REQUIRE(allocator.ownedNote(4) == 60); // MIDI 60: fret 1 on the B string
        REQUIRE(allocator.ownedNote(1) == 46); // MIDI 46: fret 1 on the A string

        const RawMidiEvent pedalDown[] = {RawMidiEvent{2, 0xB0u, cnpg::dsp::kSustainPedalController, 127, 0}};
        BlockEventQueue pedalOut;
        allocator.allocate(pedalDown, 1, pedalOut);
        const RawMidiEvent releases[] = {noteOff(3, 60), noteOff(4, 46)};
        allocator.allocate(releases, 2, pedalOut);
        REQUIRE(pedalOut.empty()); // both held by the pedal
        REQUIRE(allocator.sustainHoldPending(4));
        REQUIRE(allocator.sustainHoldPending(1));

        params.activeStringCount = 3; // string 4 leaves; string 1 stays
        allocator.setParams(params);

        const RawMidiEvent pedalUp[] = {RawMidiEvent{5, 0xB0u, cnpg::dsp::kSustainPedalController, 0, 0}};
        allocator.allocate(pedalUp, 1, pedalOut);
        const std::vector<NoteEvent> emitted = drain(pedalOut);
        REQUIRE(emitted.size() == 1);
        REQUIRE(emitted[0].stringIndex == 1);
        REQUIRE(emitted[0].midiNote == 46);
        REQUIRE(allocator.unaddressableNoteOffCount() == 1);
        // Both ownerships are gone, addressable or not: the pedal-up is the end of both notes.
        REQUIRE(allocator.ownedNote(4) == -1);
        REQUIRE(allocator.ownedNote(1) == -1);
        REQUIRE(allocator.queueOverflowCount() == 0);
    }
}

TEST_CASE("CONTRACT: a full event queue is counted, never blocked on", "[contract]") {
    // The other drop, and the more dangerous one: the allocation SUCCEEDED and the event was lost
    // anyway because BlockEventQueue's 256 slots were full. For a NoteOff that is a note that never
    // stops, so it must be visible. Nothing here may allocate or block -- the queue reports the
    // refusal and the allocator counts it.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    std::vector<RawMidiEvent> events;
    events.reserve(300);
    for (int i = 0; i < 300; ++i)
        events.push_back(noteOn(0, static_cast<std::uint8_t>(48 + (i % 8))));

    BlockEventQueue out;
    allocator.allocate(events.data(), static_cast<int>(events.size()), out);

    REQUIRE(out.size() == BlockEventQueue::capacity());
    REQUIRE(out.droppedCount() > 0);
    REQUIRE(allocator.queueOverflowCount() == out.droppedCount());
    std::cout << "[contract] queue overflow: " << events.size() << " NoteOns into a " << BlockEventQueue::capacity()
              << "-slot queue -> " << allocator.queueOverflowCount() << " counted refusals\n";
    REQUIRE(allocator.unassignableNoteCount() == 0);
}

TEST_CASE("CONTRACT: allocate() performs no heap allocation in either mode", "[contract]") {
    // docs/plan.md Task P2.6: "All allocator paths are alloc-free after prepare (debug allocation
    // guard) and drop-counted, never blocking." Every path is exercised inside the measured region:
    // both modes, an idle assignment, a steal, a NoteOff, a pedal press, a pedal release that emits
    // six held NoteOffs at once, an out-of-envelope note, an unassignable note, a NoteOff whose
    // string left the active count, and a queue overflow.
    //
    // "IN EITHER MODE" IS DRIVEN, NOT ASSERTED FROM THE OUTSIDE. The first version of this case said
    // "either mode" in its title and only ever configured GuitarFingering. The claim happened to be
    // true -- FreeZones differs from GuitarFingering only in two integer comparisons inside
    // stringCanPlay(), and neither branch touches memory -- but "true and untested" is what the
    // title was quietly asserting, so the same event stream now runs a second time under FreeZones
    // INSIDE the same guarded region.
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);

    // The two configurations, differing ONLY in the mode and its table. Both are six strings over
    // the same span, so the same event stream exercises the same paths in each: the zones below are
    // exactly the 24-fret spans GuitarFingering derives from the same open notes, which is what
    // makes the two runs comparable rather than merely both present.
    NoteAllocatorParams fingering = sixStringGuitar();
    NoteAllocatorParams freeZones = sixStringGuitar();
    freeZones.mode = AllocationMode::FreeZones;
    for (int s = 0; s < cnpg::dsp::kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        const int open = static_cast<int>(fingering.openStringMidiNote[index]);
        freeZones.zones[index].lowNote = static_cast<std::uint8_t>(open);
        freeZones.zones[index].highNote =
            static_cast<std::uint8_t>(std::min(open + cnpg::dsp::kFingeringFretSpan, 127));
    }
    allocator.setParams(fingering);

    std::vector<RawMidiEvent> events;
    events.push_back(noteOn(0, 40));
    events.push_back(noteOn(1, 45));
    events.push_back(noteOn(2, 50));
    events.push_back(noteOn(3, 55));
    events.push_back(noteOn(4, 59));
    events.push_back(noteOn(5, 64));
    events.push_back(RawMidiEvent{6, 0xB0u, cnpg::dsp::kSustainPedalController, 127, 0}); // pedal down
    events.push_back(noteOff(7, 40));
    events.push_back(noteOff(8, 45));
    events.push_back(noteOff(9, 50));
    events.push_back(noteOff(10, 55));
    events.push_back(noteOff(11, 59));
    events.push_back(noteOff(12, 64));
    events.push_back(RawMidiEvent{13, 0xB0u, cnpg::dsp::kSustainPedalController, 0, 0}); // pedal up: six NoteOffs
    events.push_back(noteOn(14, 30));                                                    // unassignable
    events.push_back(noteOn(15, 20));                                                    // out of envelope
    events.push_back(noteOn(16, 57));                                                    // idle assignment
    events.push_back(noteOn(17, 57));                                                    // same-note retrigger
    for (int i = 0; i < 300; ++i)
        events.push_back(noteOn(18, 60)); // overflow the queue

    BlockEventQueue out;

    // Warm-up so the measured call is not the first invocation of any path, in EITHER mode: a first
    // call under FreeZones inside the measured region would be measuring cold code rather than the
    // steady state, which is the whole reason the warm-up exists.
    allocator.allocate(events.data(), static_cast<int>(events.size()), out);
    out.clear();
    allocator.reset();
    allocator.setParams(freeZones);
    allocator.allocate(events.data(), static_cast<int>(events.size()), out);
    out.clear();
    allocator.reset();
    allocator.setParams(fingering);

    struct Exercised {
        std::uint32_t unassignable = 0;
        std::uint32_t outOfRange = 0;
        std::uint32_t overflow = 0;
        int ownerOf57 = -1;
    };

    cnpg::test::resetAllocationCount();

    // Pass one: GuitarFingering.
    allocator.allocate(events.data(), static_cast<int>(events.size()), out);
    const Exercised guitar{allocator.unassignableNoteCount(), allocator.outOfRangeNoteCount(),
                           allocator.queueOverflowCount(), allocator.stringForNote(0, 57)};

    // Pass two: the SAME stream under FreeZones, inside the SAME guarded region. clear(), reset()
    // and setParams() are each realtime-safe by their own contracts, so putting them here measures
    // them rather than hiding them outside the region.
    out.clear();
    allocator.reset();
    allocator.setParams(freeZones);
    allocator.allocate(events.data(), static_cast<int>(events.size()), out);
    const Exercised zones{allocator.unassignableNoteCount(), allocator.outOfRangeNoteCount(),
                          allocator.queueOverflowCount(), allocator.stringForNote(0, 57)};

    // Pass three: the fourth counter's path. It cannot be a row in the stream above, because it
    // needs a params move to land UNDER a held note -- which is exactly the gesture it exists for.
    // Still inside the guarded region: setParams() is realtime-safe by its own contract, so putting
    // the count reduction here measures it rather than hiding it outside.
    out.clear();
    allocator.reset();
    allocator.setParams(fingering);
    const RawMidiEvent held[] = {noteOn(0, 60)};
    allocator.allocate(held, 1, out);
    NoteAllocatorParams shrunk = fingering;
    shrunk.activeStringCount = 3; // MIDI 60 sits on string 4, which this leaves behind
    allocator.setParams(shrunk);
    const RawMidiEvent undeliverable[] = {noteOff(1, 60)};
    allocator.allocate(undeliverable, 1, out);
    const std::uint32_t undeliverableNoteOffs = allocator.unaddressableNoteOffCount();

    REQUIRE(cnpg::test::allocationCount() == 0);

    // The third pass really drove the branch it was added for.
    REQUIRE(undeliverableNoteOffs > 0);

    // ...and BOTH passes really exercised the paths, rather than the allocation counter merely
    // staying at zero because the second mode declined everything early. Each mode is asserted to
    // have dropped on all three counters AND to have assigned: MIDI 57 lies inside the D string's
    // 24-fret span and inside the zone derived from it, so an owner of -1 in either pass would mean
    // that pass never reached the assignment policy at all.
    for (const Exercised& pass : {guitar, zones}) {
        REQUIRE(pass.unassignable > 0);
        REQUIRE(pass.outOfRange > 0);
        REQUIRE(pass.overflow > 0);
        REQUIRE(pass.ownerOf57 >= 0);
    }
    // The two tables describe the same candidate set over this note stream, so the modes must agree
    // about what they could NOT place.
    REQUIRE(zones.unassignable == guitar.unassignable);
    REQUIRE(zones.outOfRange == guitar.outOfRange);

    // They are not required to agree about WHERE a placeable note went, and they do not -- which is
    // the whole difference between them, and it is what makes the second pass a test of FreeZones
    // rather than a second copy of the first. GuitarFingering breaks ties by fret position (MIDI 57
    // is fret 2 on the D string, the cheapest of the four candidates); FreeZones has no fret to
    // prefer, so every candidate ties and least-recently-used decides. Asserted rather than glossed
    // over: a build in which setParams() silently failed to take the new mode would otherwise pass
    // this case as "both modes ran".
    REQUIRE(guitar.ownerOf57 == 3);
    REQUIRE(zones.ownerOf57 != guitar.ownerOf57);
}

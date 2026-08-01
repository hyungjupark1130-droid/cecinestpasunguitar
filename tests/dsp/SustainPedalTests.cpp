#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/NoteAllocator.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/ClickMetric.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

// SustainPedalTests -- docs/plan.md Task P2.6, CC64.
//
// The pedal is the one MIDI controller that changes WHEN a note-off happens rather than what it
// does, so every claim here is about timing and ownership: while it is down a NoteOff is HELD and
// the string keeps its note; when it comes up every held NoteOff fires at the pedal's own sample;
// and a note struck again under the pedal cancels the release that was waiting for it.
//
// The last case in the file is the one the coupled bridge made worth writing: six strings held
// under the pedal on ONE shared junction, all six dampers engaging on the same sample. That is the
// densest simultaneous state change the instrument can be asked for, and it is exactly the shape
// that produces a click if anything in the chain steps rather than ramps.

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
constexpr float kPickup = 0.87f;

RawMidiEvent noteOn(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t velocity = 100,
                    std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x90u | channel), note, velocity, channel};
}

RawMidiEvent noteOff(std::int32_t sampleOffset, std::uint8_t note, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0x80u | channel), note, 0, channel};
}

RawMidiEvent pedal(std::int32_t sampleOffset, std::uint8_t value, std::uint8_t channel = 0) {
    return RawMidiEvent{sampleOffset, static_cast<std::uint8_t>(0xB0u | channel), cnpg::dsp::kSustainPedalController,
                        value, channel};
}

std::vector<NoteEvent> drain(BlockEventQueue& queue) {
    std::vector<NoteEvent> events;
    std::int32_t previousOffset = -1;
    while (const NoteEvent* event = queue.peek()) {
        // THE QUEUE ORDER CONTRACT, checked on every drain in this file. A Debug build asserts it
        // inside push(); CI has no Debug job, so the Release suite has to see it too.
        REQUIRE(event->sampleOffset >= previousOffset);
        previousOffset = event->sampleOffset;
        events.push_back(*event);
        queue.pop();
    }
    return events;
}

void requireNoDrops(const NoteAllocator& allocator) {
    REQUIRE(allocator.unassignableNoteCount() == 0);
    REQUIRE(allocator.outOfRangeNoteCount() == 0);
    REQUIRE(allocator.unaddressableNoteOffCount() == 0);
    REQUIRE(allocator.queueOverflowCount() == 0);
}

NoteAllocatorParams sixStringGuitar() {
    NoteAllocatorParams params;
    params.mode = AllocationMode::GuitarFingering;
    params.activeStringCount = 6;
    return params;
}

StringNetworkParams networkParams(float coupling) {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    params.bridge.couplingStrength = coupling;
    return params;
}

template <typename SampleT> SampleT peakOf(const std::vector<SampleT>& samples) {
    SampleT peak = SampleT(0);
    for (SampleT value : samples)
        peak = std::max(peak, static_cast<SampleT>(std::fabs(value)));
    return peak;
}

double rmsOver(const std::vector<float>& samples, std::size_t begin, std::size_t end) {
    end = std::min(end, samples.size());
    if (begin >= end)
        return 0.0;
    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i)
        sum += static_cast<double>(samples[i]) * static_cast<double>(samples[i]);
    return std::sqrt(sum / static_cast<double>(end - begin));
}

} // namespace

TEST_CASE("CONTRACT: CC64 down holds a NoteOff and the string keeps its note", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 60), pedal(10, 127), noteOff(20, 60)};
    BlockEventQueue out;
    allocator.allocate(events, 3, out);

    const std::vector<NoteEvent> emitted = drain(out);
    REQUIRE(emitted.size() == 1); // the NoteOn only; the NoteOff is under the pedal
    REQUIRE(emitted[0].type == NoteEventType::NoteOn);
    REQUIRE(allocator.sustainActive());

    // THE STRING IS STILL BUSY, which is the whole of what the pedal does at this level: the note
    // is still ringing, so the string is not free and a later NoteOn has to steal it. Asserted on
    // the ownership directly, not inferred from the absence of an event.
    const int owner = static_cast<int>(emitted[0].stringIndex);
    REQUIRE(allocator.ownedNote(owner) == 60);
    REQUIRE(allocator.stringForNote(0, 60) == owner);
    REQUIRE(allocator.sustainHoldPending(owner));
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: CC64 up emits the held NoteOff at the pedal-release offset", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent held[] = {noteOn(0, 60), pedal(10, 127), noteOff(20, 60)};
    BlockEventQueue heldOut;
    allocator.allocate(held, 3, heldOut);
    const std::vector<NoteEvent> heldEmitted = drain(heldOut);
    REQUIRE(heldEmitted.size() == 1);
    const int owner = static_cast<int>(heldEmitted[0].stringIndex);
    REQUIRE(allocator.sustainHoldPending(owner)); // in state: there IS something waiting

    constexpr std::int32_t kReleaseOffset = 77;
    const RawMidiEvent up[] = {pedal(kReleaseOffset, 0)};
    BlockEventQueue upOut;
    allocator.allocate(up, 1, upOut);
    const std::vector<NoteEvent> released = drain(upOut);

    REQUIRE(released.size() == 1);
    REQUIRE(released[0].type == NoteEventType::NoteOff);
    REQUIRE(released[0].midiNote == 60);
    REQUIRE(static_cast<int>(released[0].stringIndex) == owner);
    // AT THE PEDAL'S OWN SAMPLE, not at the NoteOff's original offset (20) and not at 0.
    REQUIRE(released[0].sampleOffset == kReleaseOffset);

    REQUIRE_FALSE(allocator.sustainActive());
    REQUIRE_FALSE(allocator.sustainHoldPending(owner));
    REQUIRE(allocator.ownedNote(owner) == -1); // the string is free now
    requireNoDrops(allocator);

    // A second pedal-up changes nothing and emits nothing -- a pedal that did not move is not an
    // event, and re-emitting would double every note-off a wobbling controller produced.
    const RawMidiEvent again[] = {pedal(90, 0)};
    BlockEventQueue againOut;
    allocator.allocate(again, 1, againOut);
    REQUIRE(againOut.empty());
}

TEST_CASE("CONTRACT: re-striking a held-off note cancels its pending NoteOff", "[contract]") {
    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    const RawMidiEvent events[] = {noteOn(0, 60), pedal(4, 127), noteOff(8, 60), noteOn(12, 60)};
    BlockEventQueue out;
    allocator.allocate(events, 4, out);
    const std::vector<NoteEvent> emitted = drain(out);

    // Two NoteOns and no NoteOff: the held release was cancelled by the restrike, so the note is
    // simply sounding again.
    REQUIRE(emitted.size() == 2);
    REQUIRE(emitted[0].type == NoteEventType::NoteOn);
    REQUIRE(emitted[1].type == NoteEventType::NoteOn);
    REQUIRE(emitted[1].sampleOffset == 12);
    REQUIRE(emitted[0].stringIndex == emitted[1].stringIndex); // and on the SAME string

    const int owner = static_cast<int>(emitted[0].stringIndex);
    REQUIRE_FALSE(allocator.sustainHoldPending(owner));
    REQUIRE(allocator.ownedNote(owner) == 60);

    // The pedal coming up now must emit NOTHING: the release it was holding no longer exists. A
    // build that merely deferred the note-off without cancelling it would damp a note the player is
    // still holding down, which is the audible failure this case is about.
    const RawMidiEvent up[] = {pedal(100, 0)};
    BlockEventQueue upOut;
    allocator.allocate(up, 1, upOut);
    REQUIRE(upOut.empty());
    REQUIRE(allocator.ownedNote(owner) == 60); // still held by the player
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a steal cancels the displaced note's CC64-held NoteOff", "[contract]") {
    // docs/plan.md section 2.12: "a later NoteOff -- INCLUDING A CC64-HELD NoteOff -- for a note
    // that no longer owns its string is dropped". A held NoteOff is the more dangerous half: it is
    // already inside the allocator, so a build that queued it by (string, note) and fired it on
    // pedal-up would damp whatever note happened to be on that string by then.
    NoteAllocator allocator;
    allocator.prepare(1);
    NoteAllocatorParams params = sixStringGuitar();
    params.activeStringCount = 1;
    allocator.setParams(params);

    const RawMidiEvent events[] = {noteOn(0, 60), pedal(4, 127), noteOff(8, 60), noteOn(12, 64)};
    BlockEventQueue out;
    allocator.allocate(events, 4, out);
    const std::vector<NoteEvent> emitted = drain(out);

    REQUIRE(emitted.size() == 2); // NoteOn 60, NoteOn 64 -- no NoteOff for either
    REQUIRE(emitted[1].midiNote == 64);
    REQUIRE(allocator.ownedNote(0) == 64);
    REQUIRE(allocator.stringForNote(0, 60) == -1);
    REQUIRE_FALSE(allocator.sustainHoldPending(0)); // in state: nothing is waiting for 60 any more

    // Pedal up: 64 is still held by the player, so nothing fires.
    const RawMidiEvent up[] = {pedal(60, 0)};
    BlockEventQueue upOut;
    allocator.allocate(up, 1, upOut);
    REQUIRE(upOut.empty());
    REQUIRE(allocator.ownedNote(0) == 64);
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: a NoteOff under the pedal engages no damper until the pedal comes up", "[contract]") {
    // The criterion end to end, through the real network: with the pedal down a NoteOff must reach
    // no damper at all, and the string must go on decaying at its undamped rate. Both halves are
    // asserted -- the DIRECT state (engagement is exactly 0) and the audible consequence (the tail
    // is as loud as a render where no note-off was ever sent) -- because either alone is passable
    // by the wrong code: a damper that engaged and did nothing would fail the first, and an
    // allocator that simply dropped the note-off would pass both until the pedal came up.
    NoteAllocator allocator;
    allocator.prepare(1);
    NoteAllocatorParams allocatorParams = sixStringGuitar();
    allocatorParams.activeStringCount = 1;
    allocator.setParams(allocatorParams);

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(networkParams(0.0f)); // one string: nothing to couple to
    network.reset();

    std::vector<float> tap;
    auto pump = [&](const RawMidiEvent* raw, int count, int blocks) {
        BlockEventQueue queue;
        allocator.allocate(raw, count, queue);
        for (int b = 0; b < blocks; ++b) {
            network.process(queue, kBlock);
            const float* channel = network.tapBuffers().channel(0, 0);
            tap.insert(tap.end(), channel, channel + kBlock);
        }
    };

    const RawMidiEvent start[] = {noteOn(0, 60), pedal(1, 127)};
    pump(start, 2, 100);
    REQUIRE(allocator.sustainActive());
    REQUIRE(network.damperEngagement(0) == 0.0f);
    const std::size_t heldStart = tap.size();

    const RawMidiEvent release[] = {noteOff(0, 60)};
    pump(release, 1, 200);                        // ~530 ms of "released" note, under the pedal
    REQUIRE(network.damperEngagement(0) == 0.0f); // NOTHING reached the damper
    REQUIRE(allocator.sustainHoldPending(0));

    const double heldTailRms = rmsOver(tap, tap.size() - static_cast<std::size_t>(20 * kBlock), tap.size());

    // The control: the identical render with no pedal at all, so the note-off really does damp.
    NoteAllocator control;
    control.prepare(1);
    control.setParams(allocatorParams);
    StringNetwork<float> controlNetwork;
    controlNetwork.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    controlNetwork.setNumStrings(1);
    controlNetwork.setParams(networkParams(0.0f));
    controlNetwork.reset();

    std::vector<float> controlTap;
    auto controlPump = [&](const RawMidiEvent* raw, int count, int blocks) {
        BlockEventQueue queue;
        control.allocate(raw, count, queue);
        for (int b = 0; b < blocks; ++b) {
            controlNetwork.process(queue, kBlock);
            const float* channel = controlNetwork.tapBuffers().channel(0, 0);
            controlTap.insert(controlTap.end(), channel, channel + kBlock);
        }
    };
    const RawMidiEvent controlStart[] = {noteOn(0, 60)};
    controlPump(controlStart, 1, 100);
    const RawMidiEvent controlRelease[] = {noteOff(0, 60)};
    controlPump(controlRelease, 1, 10);                 // 27 ms: the felt is down and the watchdog has not fired yet
    REQUIRE(controlNetwork.damperEngagement(0) > 0.0f); // the control really was damped
    controlPump(nullptr, 0, 190);
    const double dampedTailRms =
        rmsOver(controlTap, controlTap.size() - static_cast<std::size_t>(20 * kBlock), controlTap.size());

    const double heldDb = 20.0 * std::log10(std::max(heldTailRms, 1e-30));
    const double dampedDb = 20.0 * std::log10(std::max(dampedTailRms, 1e-30));
    std::cout << "[contract] CC64 held vs damped tail RMS 530 ms after the note-off: " << heldDb << " dBFS vs "
              << dampedDb << " dBFS (difference " << (heldDb - dampedDb) << " dB)\n";
    REQUIRE(heldDb > dampedDb + 20.0); // the pedal is doing something large and audible
    REQUIRE(heldStart < tap.size());

    // Now the pedal comes up, and the felt finally comes down.
    const RawMidiEvent up[] = {pedal(0, 0)};
    pump(up, 1, 20);
    REQUIRE(network.damperEngagement(0) > 0.0f);
    requireNoDrops(allocator);
}

TEST_CASE("CONTRACT: six strings under the pedal damp together on one sample without a click", "[contract]") {
    // The densest simultaneous state change the instrument has, on the topology that makes it
    // dense: six notes held under the pedal on ONE loaded bridge, released by a single CC64-up, so
    // six DamperJunctions engage on the same sample and every one of them is coupled to the other
    // five.
    //
    // Three separate claims, because no one of them implies the others:
    //   1. QUEUE. Exactly six NoteOffs, all at the pedal's own offset, in ascending string order,
    //      non-decreasing (which is what the Debug-only push() assert checks and CI never runs).
    //   2. STATE. All six dampers really engage -- the direct assertion the P2.1 ruling requires,
    //      since a click reading alone cannot tell "nothing happened" from "it happened smoothly".
    //   3. AUDIO. Nothing clicks, measured against the same render with the pedal-up moved later,
    //      i.e. a reference that differs only in when the six dampers land.
    constexpr int kNotes = 6;
    static const std::uint8_t kChord[kNotes] = {40, 47, 52, 56, 59, 64}; // an open E major

    NoteAllocator allocator;
    allocator.prepare(cnpg::dsp::kMaxStrings);
    allocator.setParams(sixStringGuitar());

    // Build the whole gesture once and render it twice, differing ONLY in the pedal-up offset.
    auto render = [&](bool releasePedal, std::vector<float>& mix, std::array<float, kNotes>& engagementOut,
                      std::vector<NoteEvent>& releaseEvents) {
        NoteAllocator local;
        local.prepare(cnpg::dsp::kMaxStrings);
        local.setParams(sixStringGuitar());

        StringNetwork<float> network;
        network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(kNotes);
        network.setParams(networkParams(StringNetworkParams{}.bridge.couplingStrength)); // the shipping load
        network.reset();

        auto renderBlocks = [&](BlockEventQueue& queue, int blocks) {
            std::vector<float> summed(static_cast<std::size_t>(kBlock), 0.0f);
            for (int b = 0; b < blocks; ++b) {
                network.process(queue, kBlock);
                // The PEAK engagement each damper reaches, not its value at some later block. A
                // damped high string goes silent and the watchdog clears it -- which snaps its
                // engagement back to 0 -- so a reading taken "afterwards" reports 0 for a string
                // that damped perfectly. The peak is the quantity the claim is about.
                for (int s = 0; s < kNotes; ++s)
                    engagementOut[static_cast<std::size_t>(s)] =
                        std::max(engagementOut[static_cast<std::size_t>(s)], network.damperEngagement(s));
                std::fill(summed.begin(), summed.end(), 0.0f);
                for (int s = 0; s < kNotes; ++s) {
                    const float* channel = network.tapBuffers().channel(s, 0);
                    if (channel == nullptr)
                        continue;
                    for (int n = 0; n < kBlock; ++n)
                        summed[static_cast<std::size_t>(n)] += channel[n];
                }
                mix.insert(mix.end(), summed.begin(), summed.end());
            }
        };

        auto pump = [&](const RawMidiEvent* raw, int count, int blocks, std::vector<NoteEvent>* capture) {
            BlockEventQueue queue;
            local.allocate(raw, count, queue);
            if (capture != nullptr) {
                // Drained (which is also where the non-decreasing-offset contract is checked) and
                // pushed straight back, so the render below sees exactly the stream that was
                // asserted rather than a second, separately built copy of it.
                *capture = drain(queue);
                for (const NoteEvent& event : *capture)
                    REQUIRE(queue.push(event));
            }
            renderBlocks(queue, blocks);
        };

        std::vector<RawMidiEvent> strum;
        strum.push_back(pedal(0, 127));
        for (int i = 0; i < kNotes; ++i)
            strum.push_back(noteOn(static_cast<std::int32_t>(1 + i * 6), kChord[static_cast<std::size_t>(i)]));
        pump(strum.data(), static_cast<int>(strum.size()), 120, nullptr); // ~320 ms of ringing chord

        std::vector<RawMidiEvent> lift;
        for (int i = 0; i < kNotes; ++i)
            lift.push_back(noteOff(static_cast<std::int32_t>(i), kChord[static_cast<std::size_t>(i)]));
        pump(lift.data(), static_cast<int>(lift.size()), 40, nullptr); // all six held under the pedal

        for (int s = 0; s < kNotes; ++s)
            REQUIRE(network.damperEngagement(s) == 0.0f); // in state: nothing damped yet

        if (releasePedal) {
            const RawMidiEvent up[] = {pedal(64, 0)};
            pump(up, 1, 60, &releaseEvents);
        } else {
            BlockEventQueue empty;
            renderBlocks(empty, 60); // the reference: identical, except the pedal never comes up
        }

        REQUIRE(local.unassignableNoteCount() == 0);
        REQUIRE(local.unaddressableNoteOffCount() == 0);
        REQUIRE(local.queueOverflowCount() == 0);
    };

    std::vector<float> released;
    std::vector<float> stillHeld;
    std::array<float, kNotes> releasedEngagement{};
    std::array<float, kNotes> heldEngagement{};
    std::vector<NoteEvent> releaseEvents;
    std::vector<NoteEvent> unused;

    render(true, released, releasedEngagement, releaseEvents);
    render(false, stillHeld, heldEngagement, unused);

    REQUIRE(released.size() == stillHeld.size());
    REQUIRE(peakOf(stillHeld) > 0.001f); // non-vacuous: the chord really is ringing

    // 1. QUEUE. Six NoteOffs, one offset, ascending string order.
    REQUIRE(releaseEvents.size() == static_cast<std::size_t>(kNotes));
    for (std::size_t i = 0; i < releaseEvents.size(); ++i) {
        INFO("released event " << i);
        REQUIRE(releaseEvents[i].type == NoteEventType::NoteOff);
        REQUIRE(releaseEvents[i].sampleOffset == 64);
        if (i > 0)
            REQUIRE(releaseEvents[i].stringIndex > releaseEvents[i - 1].stringIndex);
    }

    // 2. STATE. All six really engaged, and the control's six really did not.
    std::cout << "[contract] six dampers on one sample, PEAK engagement over the 160 ms that follow:";
    for (int s = 0; s < kNotes; ++s) {
        std::cout << " s" << s << "=" << releasedEngagement[static_cast<std::size_t>(s)];
        INFO("string " << s);
        REQUIRE(releasedEngagement[static_cast<std::size_t>(s)] > 0.5f);
        REQUIRE(heldEngagement[static_cast<std::size_t>(s)] == 0.0f);
    }
    std::cout << "\n";

    // 3. AUDIO. The span starts one block before the pedal-up and runs 20 blocks past it, so the
    // reference's median describes the chord at the moment of the change rather than an average
    // over a decay that has run away from it (ClickMetric.h, "choosing the analysed span").
    const std::size_t pedalSample = static_cast<std::size_t>(160 * kBlock) + 64;
    const std::size_t spanBegin = pedalSample - static_cast<std::size_t>(kBlock);
    const std::size_t spanEnd = pedalSample + static_cast<std::size_t>(20 * kBlock);

    const cnpg::test::ClickMeasurement reference = cnpg::test::measureClick(stillHeld, kRate, spanBegin, spanEnd);
    const cnpg::test::ClickMeasurement measured = cnpg::test::measureClick(released, kRate, spanBegin, spanEnd);
    REQUIRE(reference.medianAbsDiff > 0.0); // non-degenerate reference
    REQUIRE(measured.nonFiniteSamples == 0);
    REQUIRE(measured.subnormalSamples == 0);
    REQUIRE(reference.nonFiniteSamples == 0);

    const double excessDb = cnpg::test::clickExcessDb(measured, reference);

    // The standing negative control, carried inside the gate so it cannot go vacuous: the reference
    // render with the six dampers replaced by a hard cut.
    //
    // LEVEL-PLACED, not placed at the pedal sample, and that is the P2.2 lesson rather than a
    // convenience. A hard cut's peak |dx| is the sample value it lands on, so a cut that happens to
    // fall near a zero crossing of a six-string sum is a cut of almost nothing -- it read 0.27 dB
    // when placed blind, i.e. the control passed the gate it was supposed to fail, which would have
    // left the gate provably toothless. The cut is therefore placed at the LOUDEST sample inside the
    // window it is allowed to use, which is the largest discontinuity this signal can be given
    // there.
    std::size_t cutSample = pedalSample;
    float cutLevel = 0.0f;
    for (std::size_t i = pedalSample; i < std::min(spanEnd, stillHeld.size()); ++i) {
        if (std::fabs(stillHeld[i]) > cutLevel) {
            cutLevel = std::fabs(stillHeld[i]);
            cutSample = i;
        }
    }
    REQUIRE(cutLevel > 0.0f);
    std::vector<float> hardCut = stillHeld;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(cutSample), hardCut.end(), 0.0f);
    const cnpg::test::ClickMeasurement hardCutMeasurement =
        cnpg::test::measureClick(hardCut, kRate, spanBegin, spanEnd);
    const double hardCutExcessDb = cnpg::test::clickExcessDb(hardCutMeasurement, reference);

    std::cout << "[contract] six-damper pedal release click excess " << excessDb << " dB (limit "
              << cnpg::test::kClickMetricToleranceDb << " dB); level-placed hard-cut negative control "
              << hardCutExcessDb << " dB at sample +" << (cutSample - pedalSample) << " (level " << cutLevel << ")\n";
    INFO("pedal release excess " << excessDb << " dB, hard-cut control " << hardCutExcessDb << " dB");
    REQUIRE(hardCutExcessDb > cnpg::test::kClickMetricToleranceDb); // the gate has teeth...
    REQUIRE(excessDb <= cnpg::test::kClickMetricToleranceDb);       // ...and the release clears it
}

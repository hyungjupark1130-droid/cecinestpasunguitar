#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/IBridgePort.h"
#include "cnpg/dsp/MidiTranslation.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::PluckExciter;
using cnpg::dsp::PluckExciterParams;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr int kMidiNote = 45;
constexpr float kPickup = 0.87f;
constexpr float kPluckPosition = 0.28f;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex = 0, float velocity = 0.8f,
                 float pluckPosition = kPluckPosition, float hardness = 0.5f) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = velocity;
    event.pluckPosition = pluckPosition;
    event.hardness = hardness;
    return event;
}

NoteEvent noteOff(int sampleOffset, int midiNote, int stringIndex = 0) {
    NoteEvent event = noteOn(sampleOffset, midiNote, stringIndex);
    event.type = NoteEventType::NoteOff;
    event.pluckPosition = cnpg::dsp::kUnspecifiedNoteParam;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

StringNetworkParams defaultParams() {
    StringNetworkParams params;
    params.pickupPosition01 = kPickup;
    return params;
}

// prepare -> setParams -> reset is the documented order for a render that must start on its
// parameters exactly: reset() snaps the pickup-position smoother onto its target, so the tap does
// not glide in from the prepared default over the first 8 ms.
template <typename SampleT>
void configure(StringNetwork<SampleT>& network, const StringNetworkParams& params, double sampleRate = kRate,
               int maxBlockSize = kBlock, FractionalDelayKind kind = FractionalDelayKind::Lagrange3,
               int numStrings = 1) {
    network.prepare(sampleRate, maxBlockSize, kind);
    network.setNumStrings(numStrings);
    network.setParams(params);
    network.reset();
}

// Renders `numBlocks` blocks and returns the concatenated tap channel of `stringIndex`.
template <typename SampleT>
std::vector<SampleT> renderTap(StringNetwork<SampleT>& network, BlockEventQueue& events, int numBlocks,
                               int stringIndex = 0, int blockSize = kBlock) {
    std::vector<SampleT> out;
    out.reserve(static_cast<std::size_t>(numBlocks) * static_cast<std::size_t>(blockSize));
    for (int b = 0; b < numBlocks; ++b) {
        network.process(events, blockSize);
        const SampleT* channel = network.tapBuffers().channel(stringIndex);
        REQUIRE(channel != nullptr);
        out.insert(out.end(), channel, channel + blockSize);
    }
    return out;
}

template <typename SampleT> SampleT peakOf(const std::vector<SampleT>& samples) {
    SampleT peak = SampleT(0);
    for (SampleT s : samples)
        peak = std::max(peak, static_cast<SampleT>(std::fabs(s)));
    return peak;
}

// An IBridgePort that records everything it is handed. bridgeOutput() deliberately returns a
// non-zero signal (the summed incident waves) so the bridge-buffer plumbing is testable before
// BridgeJunction exists, and scatter() returns a deliberately absurd reflection so P1's "the
// port's outgoing waves are not routed back into the strings" boundary is observable.
class RecordingPort final : public cnpg::dsp::IBridgePort<float> {
  public:
    void prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) override {
        incidentLog.reserve(1u << 16); // scatter() is noexcept; never grow from inside it
        prepareCalls++;
        preparedPorts = numPorts;
        preparedRate = sampleRate;
        preparedBlockSize = maxBlockSize;
        impedanceOfPort0 = (portImpedances != nullptr && numPorts > 0) ? portImpedances[0] : -1.0f;
    }
    void reset() noexcept override { resetCalls++; }

    void scatter(const float* incident, float* outgoing, int numPorts) noexcept override {
        float sum = 0.0f;
        for (int port = 0; port < numPorts; ++port) {
            sum += incident[port];
            // Deliberately absurd: nothing physical reflects like this. If P1 routed the port's
            // outgoing waves back into the strings, the render would explode.
            outgoing[port] = 17.0f * incident[port] + 1.0f;
        }
        lastSum = sum;
        incidentLog.push_back(numPorts > 0 ? incident[0] : 0.0f);
    }

    float bridgeOutput() const noexcept override { return lastSum; }
    void setLossBypassed(bool bypass) noexcept override { lossBypassed = bypass; }

    int prepareCalls = 0;
    int resetCalls = 0;
    int preparedPorts = 0;
    double preparedRate = 0.0;
    int preparedBlockSize = 0;
    float impedanceOfPort0 = -1.0f;
    bool lossBypassed = false;
    float lastSum = 0.0f;
    std::vector<float> incidentLog;
};

} // namespace

// ---------------------------------------------------------------------------------------------
// event consumption -- the P1.5 acceptance criterion
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork consumes events at their exact sample offsets", "[contract]") {
    // docs/plan.md Task P1.5 acceptance criterion 1: "events at offsets 0, 63, 127 in a
    // 128-sample block excite the string starting at exactly those samples (impulse-alignment
    // check on the tap buffer)".
    //
    // Checked as a shift identity rather than as "the first non-zero sample is at index k",
    // because the two are not the same claim: the excitation is injected at pluckPosition 0.28
    // and read at pickupPosition 0.87, so the wave takes a few samples to reach the tap, and the
    // burst envelope is exactly 0 at its own first sample by design. The shift identity subsumes
    // both -- it asserts the render is bit-identical to the offset-0 render delayed by exactly k
    // samples, with nothing at all before sample k.
    constexpr int kBlocks = 24;

    auto render = [](int offset) {
        StringNetwork<float> network;
        configure(network, defaultParams());
        BlockEventQueue events;
        events.push(noteOn(offset, kMidiNote));
        return renderTap(network, events, kBlocks);
    };

    const std::vector<float> reference = render(0);
    REQUIRE(peakOf(reference) > 0.001f); // non-vacuous: the offset-0 render really does sound

    for (int offset : {63, 127}) {
        const std::vector<float> shifted = render(offset);
        REQUIRE(shifted.size() == reference.size());

        for (int n = 0; n < offset; ++n) {
            INFO("offset " << offset << ": sample " << n << " precedes the event");
            REQUIRE(shifted[static_cast<std::size_t>(n)] == 0.0f);
        }
        for (std::size_t i = 0; i + static_cast<std::size_t>(offset) < shifted.size(); ++i) {
            INFO("offset " << offset << ": sample " << (i + static_cast<std::size_t>(offset)));
            REQUIRE(shifted[i + static_cast<std::size_t>(offset)] == reference[i]);
        }
    }
}

TEST_CASE("CONTRACT: StringNetwork fires several events inside one block at their own offsets", "[contract]") {
    // The same identity with all three offsets present at once, which is what the queue actually
    // sees: three NoteOns in a 128-sample block. Each retrigger at a NEW pitch re-initializes the
    // string, so the render after the last event must equal a render that only contains that last
    // event, delayed to the same offset.
    StringNetwork<float> combined;
    configure(combined, defaultParams());
    BlockEventQueue events;
    events.push(noteOn(0, 40));
    events.push(noteOn(63, 45));
    events.push(noteOn(127, 52));
    const std::vector<float> together = renderTap(combined, events, 24);

    StringNetwork<float> lastOnly;
    configure(lastOnly, defaultParams());
    BlockEventQueue lastEvent;
    lastEvent.push(noteOn(127, 52));
    const std::vector<float> alone = renderTap(lastOnly, lastEvent, 24);

    for (std::size_t i = 127; i < together.size(); ++i) {
        INFO("sample " << i);
        REQUIRE(together[i] == alone[i]);
    }
    REQUIRE(peakOf(together) > 0.001f);
}

TEST_CASE("CONTRACT: StringNetwork clamps an out-of-block event onto the block's last sample", "[contract]") {
    // The queue's contract is 0..numSamples-1, but a caller bug must lose no note: an offset past
    // the block lands on its final sample instead of firing at no sample at all.
    StringNetwork<float> network;
    configure(network, defaultParams());
    BlockEventQueue events;
    events.push(noteOn(4096, kMidiNote));
    const std::vector<float> rendered = renderTap(network, events, 24);

    StringNetwork<float> reference;
    configure(reference, defaultParams());
    BlockEventQueue clamped;
    clamped.push(noteOn(kBlock - 1, kMidiNote));
    const std::vector<float> expected = renderTap(reference, clamped, 24);

    REQUIRE(peakOf(rendered) > 0.001f);
    for (std::size_t i = 0; i < rendered.size(); ++i)
        REQUIRE(rendered[i] == expected[i]);
}

// ---------------------------------------------------------------------------------------------
// realtime contract
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork process allocates nothing", "[contract]") {
    // docs/plan.md Task P1.5 acceptance criterion 4: "process verified alloc-free (counting-new
    // test wrapping a 1000-block run)". The run deliberately includes everything the realtime
    // path can do inside process(): event consumption, note-offs and their release completing,
    // a continuously modulated bend re-solving the loop length every sample, and a moving pickup.
    constexpr int kBlocks = 1000;

    StringNetwork<float> network;
    StringNetworkParams params = defaultParams();
    configure(network, params);

    BlockEventQueue events;

    cnpg::test::resetAllocationCount();
    for (int b = 0; b < kBlocks; ++b) {
        if ((b % 50) == 0)
            events.push(noteOn(b % kBlock, 40 + (b % 24)));
        if ((b % 50) == 25)
            events.push(noteOff((b % kBlock), 40 + ((b - 25) % 24)));

        const auto t = static_cast<float>(b) * static_cast<float>(kBlock) / static_cast<float>(kRate);
        params.pitchBendSemitones = 2.0f * std::sin(6.2831853f * 2.0f * t);
        params.pickupPosition01 = 0.5f + 0.4f * std::sin(6.2831853f * 0.7f * t);
        network.setParams(params);
        network.process(events, kBlock);
        (void)network.tapBuffers().channel(0);
        (void)network.bridgeOutputBuffer();
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

TEST_CASE("CONTRACT: StringNetwork is silent and inactive before excitation", "[contract]") {
    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        StringNetwork<float> network;
        configure(network, defaultParams(), sampleRate);
        BlockEventQueue events;

        for (int b = 0; b < 32; ++b) {
            network.process(events, kBlock);
            const auto& taps = network.tapBuffers();
            REQUIRE(taps.numStrings() == 1);
            REQUIRE(taps.numSamples() == kBlock);
            REQUIRE_FALSE(taps.isActive(0));
            for (int n = 0; n < kBlock; ++n) {
                REQUIRE(taps.channel(0)[n] == 0.0f);
                REQUIRE(network.bridgeOutputBuffer()[n] == 0.0f);
            }
        }
        REQUIRE(network.energyEstimate() == 0.0);
    }
}

TEST_CASE("CONTRACT: StringNetwork reset is idempotent and complete", "[contract]") {
    // docs/plan.md section 4.1: reset() must leave the module indistinguishable from a fresh
    // prepared instance, and for StringNetwork specifically energyEstimate() must read exactly 0.
    StringNetwork<float> excited;
    configure(excited, defaultParams());
    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));
    renderTap(excited, events, 400); // ~1 s of ringing

    // Leave a bend mid-glide as well as the rails full, so reset() has something to clear on both
    // fronts.
    StringNetworkParams moving = defaultParams();
    moving.pitchBendSemitones = 1.7f;
    excited.setParams(moving);
    renderTap(excited, events, 1);

    excited.reset();
    excited.reset(); // twice must equal once
    REQUIRE(excited.energyEstimate() == 0.0);

    StringNetwork<float> fresh;
    configure(fresh, moving);

    BlockEventQueue a;
    BlockEventQueue b;
    a.push(noteOn(0, kMidiNote));
    b.push(noteOn(0, kMidiNote));
    const std::vector<float> afterReset = renderTap(excited, a, 64);
    const std::vector<float> fromFresh = renderTap(fresh, b, 64);

    REQUIRE(peakOf(afterReset) > 0.001f); // re-plucked, not silence-vs-silence
    for (std::size_t i = 0; i < afterReset.size(); ++i)
        REQUIRE(afterReset[i] == fromFresh[i]);
}

// ---------------------------------------------------------------------------------------------
// topology: the network must not perturb the string the [tuning] gate measures
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork renders the isolated string bit-exactly", "[contract]") {
    // The [tuning] gate (docs/plan.md section 4.5, +/-2 cents) and the [regression] goldens both
    // measure a note rendered through this topology. P1's network wraps the string in an exciter,
    // a release envelope at unity and a bridge port whose reflection is not routed back, and none
    // of that may move a sample -- if it did, every cent measured on the isolated string would
    // stop applying to the shipping path. Asserted at both bend extremes, since the bend reaches
    // the string through StringNetwork's own parameter plumbing.
    //
    // P2.2 inserts DamperJunction into the loop, at which point this identity legitimately ends;
    // that task owns re-pointing the tuning gate at the network.
    constexpr int kBlocks = 200;

    for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
        for (float bend : {0.0f, 2.0f, -2.0f}) {
            for (int note : {40, 69, 108}) {
                StringNetworkParams params = defaultParams();
                params.pitchBendSemitones = bend;
                params.exciter.noiseAmount = 0.25f;

                StringNetwork<float> network;
                configure(network, params, kRate, kBlock, kind);
                BlockEventQueue events;
                events.push(noteOn(0, note));
                const std::vector<float> viaNetwork = renderTap(network, events, kBlocks);

                WaveguideString<float> string;
                string.prepare(kRate, kBlock, kind);
                WaveguideStringParams stringParams;
                stringParams.f0Hz = static_cast<float>(440.0 * std::exp2((static_cast<double>(note) - 69.0) / 12.0));
                stringParams.bendSemitones = bend;
                string.setParams(stringParams);
                string.setAnalyticTuningCompensation(0.0f);
                string.reset();

                PluckExciter<float> exciter;
                exciter.prepare(kRate, kBlock);
                PluckExciterParams exciterParams;
                exciterParams.noiseAmount = 0.25f;
                exciter.setParams(exciterParams);
                exciter.trigger(0.8f, kPluckPosition, 0.5f);

                std::vector<float> direct(viaNetwork.size());
                for (std::size_t n = 0; n < direct.size(); ++n) {
                    const float excitation = exciter.renderSample();
                    if (excitation != 0.0f)
                        string.injectAt(exciter.latchedPosition01(), excitation);
                    direct[n] = string.readTapAt(kPickup);
                    string.tick();
                }

                INFO("kind " << (kind == FractionalDelayKind::Lagrange3 ? "Lagrange3" : "Thiran1") << " note " << note
                             << " bend " << bend);
                REQUIRE(peakOf(viaNetwork) > 0.001f);
                for (std::size_t n = 0; n < direct.size(); ++n)
                    REQUIRE(viaNetwork[n] == direct[n]);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// note handling: retrigger, release, and how the exciter defaults are resolved
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork plucks a same-pitch retrigger over the ringing state", "[contract]") {
    constexpr int kRingBlocks = 200; // ~0.5 s of ringing before the retrigger
    constexpr int kTailBlocks = 40;

    // Render `retriggerNote` (or nothing at all) onto a string that has been ringing on kMidiNote
    // for kRingBlocks, and return only the tail after that moment.
    auto renderRetrigger = [](int retriggerNote) {
        StringNetwork<float> network;
        configure(network, defaultParams());
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote));
        renderTap(network, events, kRingBlocks);

        BlockEventQueue retrigger;
        if (retriggerNote > 0)
            retrigger.push(noteOn(0, retriggerNote));
        return renderTap(network, retrigger, kTailBlocks);
    };

    // A fresh string plucked at that same moment, i.e. the retrigger's own contribution alone.
    auto renderFreshPluck = [](int note) {
        StringNetwork<float> network;
        configure(network, defaultParams());
        BlockEventQueue idle;
        renderTap(network, idle, kRingBlocks); // silence, so the string is untouched
        BlockEventQueue events;
        events.push(noteOn(0, note));
        return renderTap(network, events, kTailBlocks);
    };

    const std::vector<float> ringingOnly = renderRetrigger(-1);
    const std::vector<float> samePitch = renderRetrigger(kMidiNote);
    const std::vector<float> freshPluck = renderFreshPluck(kMidiNote);

    REQUIRE(peakOf(ringingOnly) > 0.001f);
    REQUIRE(peakOf(freshPluck) > 0.001f);

    // "Plucks OVER the ringing state" has an exact form in a linear waveguide: the retriggered
    // render is the still-ringing render plus the fresh pluck's own response, superposed. Nothing
    // was cleared (the ringing term survives) and the pluck did happen (the fresh term is there).
    const float peak = std::max(peakOf(samePitch), peakOf(ringingOnly));
    float worst = 0.0f;
    for (std::size_t i = 0; i < samePitch.size(); ++i)
        worst = std::max(worst, std::fabs(samePitch[i] - (ringingOnly[i] + freshPluck[i])));
    INFO("worst superposition residual " << worst << " against peak " << peak);
    REQUIRE(worst <= 1.0e-5f * peak);

    // A pitch-changing retrigger re-initializes the string at the new pitch instead -- the P1
    // stand-in for the full Physical/Synth semantics, which need DamperJunction and land in P2.6.
    // "Re-initialized" is exact too: the tail is bit-identical to the same note plucked on a
    // string that never rang at all.
    const std::vector<float> newPitch = renderRetrigger(kMidiNote + 5);
    const std::vector<float> freshNewPitch = renderFreshPluck(kMidiNote + 5);
    for (std::size_t i = 0; i < newPitch.size(); ++i)
        REQUIRE(newPitch[i] == freshNewPitch[i]);
}

TEST_CASE("CONTRACT: StringNetwork NoteOff runs a fast release and then clears the string", "[contract]") {
    StringNetwork<float> network;
    configure(network, defaultParams());

    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));
    const std::vector<float> ringing = renderTap(network, events, 200);
    const float ringingPeak = peakOf(ringing);
    REQUIRE(ringingPeak > 0.001f);
    REQUIRE(network.tapBuffers().isActive(0));

    BlockEventQueue release;
    release.push(noteOff(0, kMidiNote));

    // kReleaseSeconds is -60 dB; the state clear follows at the -100 dB floor. 250 ms covers both
    // with margin at any of the supported rates.
    const int blocks = static_cast<int>(0.25 * kRate / kBlock);
    const std::vector<float> tail = renderTap(network, release, blocks);

    const std::vector<float> lastBlock(tail.end() - kBlock, tail.end());
    INFO("ringing peak " << ringingPeak << ", final block peak " << peakOf(lastBlock));
    REQUIRE(peakOf(lastBlock) == 0.0f);
    REQUIRE(network.energyEstimate() == 0.0);
    REQUIRE_FALSE(network.tapBuffers().isActive(0));

    // A stale NoteOff for an already-released note is a no-op, not a second release.
    BlockEventQueue stale;
    stale.push(noteOff(0, kMidiNote));
    renderTap(network, stale, 1);
    REQUIRE(network.energyEstimate() == 0.0);
}

TEST_CASE("CONTRACT: StringNetwork resolves unspecified note parameters against the exciter defaults", "[contract]") {
    // docs/plan.md section 2.3: PluckExciterParams::defaultPosition/defaultHardness are "used when
    // the note event carries no explicit position". NoteAllocator emits kUnspecifiedNoteParam for
    // both in P1 (plain MIDI carries neither), so this resolution is what makes the APVTS Exciter
    // Position / Exciter Hardness knobs audible at all.
    auto render = [](float eventPosition, float eventHardness, float defaultPosition, float defaultHardness) {
        StringNetworkParams params = defaultParams();
        params.exciter.defaultPosition = defaultPosition;
        params.exciter.defaultHardness = defaultHardness;
        StringNetwork<float> network;
        configure(network, params);
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote, 0, 0.8f, eventPosition, eventHardness));
        return renderTap(network, events, 32);
    };

    const float unspecified = cnpg::dsp::kUnspecifiedNoteParam;

    // An unspecified event resolves to exactly the parameter values...
    const std::vector<float> viaDefaults = render(unspecified, unspecified, 0.31f, 0.62f);
    const std::vector<float> viaEvent = render(0.31f, 0.62f, 0.5f, 0.5f);
    REQUIRE(peakOf(viaDefaults) > 0.001f);
    for (std::size_t i = 0; i < viaDefaults.size(); ++i)
        REQUIRE(viaDefaults[i] == viaEvent[i]);

    // ...and the parameter really is what is being heard: moving it moves the render.
    const std::vector<float> movedDefault = render(unspecified, unspecified, 0.13f, 0.62f);
    bool differs = false;
    for (std::size_t i = 0; i < movedDefault.size(); ++i)
        differs |= (movedDefault[i] != viaDefaults[i]);
    REQUIRE(differs);

    // An event that DOES carry a position wins over the parameter.
    const std::vector<float> explicitWins = render(0.31f, 0.62f, 0.13f, 0.9f);
    for (std::size_t i = 0; i < explicitWins.size(); ++i)
        REQUIRE(explicitWins[i] == viaEvent[i]);
}

// ---------------------------------------------------------------------------------------------
// tap buffers and multi-string addressing
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork tap buffers are SoA and describe the block", "[contract]") {
    constexpr int kStrings = 3;
    StringNetwork<float> network;
    configure(network, defaultParams(), kRate, kBlock, FractionalDelayKind::Lagrange3, kStrings);

    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote, 1)); // middle string only
    renderTap(network, events, 40, 1);

    const auto& taps = network.tapBuffers();
    REQUIRE(taps.numStrings() == kStrings);
    REQUIRE(taps.numSamples() == kBlock);
    REQUIRE(taps.channel(-1) == nullptr);
    REQUIRE(taps.channel(kStrings) == nullptr);
    REQUIRE_FALSE(taps.isActive(-1));
    REQUIRE_FALSE(taps.isActive(kStrings));

    // One contiguous run per string, stride == the prepared maxBlockSize.
    REQUIRE(taps.channel(1) - taps.channel(0) == kBlock);
    REQUIRE(taps.channel(2) - taps.channel(1) == kBlock);

    REQUIRE(taps.isActive(1));
    REQUIRE_FALSE(taps.isActive(0));
    REQUIRE_FALSE(taps.isActive(2));
    for (int n = 0; n < kBlock; ++n) {
        REQUIRE(taps.channel(0)[n] == 0.0f);
        REQUIRE(taps.channel(2)[n] == 0.0f);
    }
    bool middleSounds = false;
    for (int n = 0; n < kBlock; ++n)
        middleSounds |= (taps.channel(1)[n] != 0.0f);
    REQUIRE(middleSounds);

    // A short block reports its own length; the stride does not shrink with it.
    network.process(events, 17);
    REQUIRE(network.tapBuffers().numSamples() == 17);
    REQUIRE(network.tapBuffers().channel(1) - network.tapBuffers().channel(0) == kBlock);
}

TEST_CASE("CONTRACT: StringNetwork disables a string completely", "[contract]") {
    // The semantics P2.1 relies on: a disabled string is skipped by the loop, reports inactive,
    // and presents a zero incident wave at its bridge port slot. (The click-free enable RAMP is
    // P2.1's; nothing in P1 can toggle this at runtime.)
    StringNetworkParams params = defaultParams();
    params.perString[0].enabled = false;

    StringNetwork<float> network;
    configure(network, params);
    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));
    const std::vector<float> rendered = renderTap(network, events, 40);

    REQUIRE(peakOf(rendered) == 0.0f);
    REQUIRE_FALSE(network.tapBuffers().isActive(0));
    REQUIRE(network.energyEstimate() == 0.0);
}

TEST_CASE("CONTRACT: StringNetwork smooths the pickup position per sample", "[contract]") {
    // docs/plan.md Task P1.5 step 3: the fractional pickup tap is computed PER SAMPLE inside the
    // loop, not once per block. Under a fast continuous sweep -- the case P2.3 eventually gates
    // with the full click metric -- that is the whole difference between a smooth tap move and a
    // staircase: at MIDI 45 / 48 kHz each rail spans ~218 samples, so a per-block-only position
    // would jump the read point by several samples at every block boundary and the tap would
    // audibly step. Measured exactly like the pitch-bend criterion: peak sample-to-sample
    // difference of the swept render against the static one, both normalized to their own peak,
    // with a 4x ceiling.
    constexpr int kBlocks = 750;             // 2 s at 48 kHz / 128
    constexpr float kSweepHz = 8.0f;         // fast enough that a per-BLOCK position would step
                                             // the read point by ~13 samples every block
    constexpr float kClickRatioLimit = 1.5f; // measured: 0.82 per-sample, 5.7 per-block

    auto render = [](bool sweep) {
        StringNetworkParams params = defaultParams();
        params.pickupPosition01 = 0.5f;

        StringNetwork<float> network;
        configure(network, params);
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote));

        std::vector<float> out;
        out.reserve(static_cast<std::size_t>(kBlocks) * static_cast<std::size_t>(kBlock));
        for (int b = 0; b < kBlocks; ++b) {
            if (sweep) {
                const auto t = static_cast<float>(b) * static_cast<float>(kBlock) / static_cast<float>(kRate);
                params.pickupPosition01 = 0.5f + 0.45f * std::sin(6.2831853f * kSweepHz * t);
                network.setParams(params);
            }
            network.process(events, kBlock);
            const float* channel = network.tapBuffers().channel(0);
            out.insert(out.end(), channel, channel + kBlock);
        }
        return out;
    };

    auto normalizedPeakStep = [](const std::vector<float>& samples) {
        const std::size_t skip = static_cast<std::size_t>(0.05 * kRate); // the pluck transient
        float peak = 0.0f;
        for (std::size_t i = skip; i < samples.size(); ++i)
            peak = std::max(peak, std::fabs(samples[i]));
        float worst = 0.0f;
        for (std::size_t i = skip + 1; i < samples.size(); ++i)
            worst = std::max(worst, std::fabs(samples[i] - samples[i - 1]));
        return (peak > 0.0f) ? worst / peak : 0.0f;
    };

    const std::vector<float> swept = render(true);
    const std::vector<float> fixed = render(false);
    const float sweptStep = normalizedPeakStep(swept);
    const float fixedStep = normalizedPeakStep(fixed);

    // The tap must actually TRACK the sweep, not merely fail to click: a smoother that is too
    // slow to follow (or one that is never advanced) would satisfy the click bound trivially by
    // leaving the position parked. Compared as energies so the bar is about the whole render, not
    // one sample.
    double sweptEnergy = 0.0;
    double differenceEnergy = 0.0;
    for (std::size_t i = 0; i < swept.size(); ++i) {
        sweptEnergy += static_cast<double>(swept[i]) * static_cast<double>(swept[i]);
        const double delta = static_cast<double>(swept[i]) - static_cast<double>(fixed[i]);
        differenceEnergy += delta * delta;
    }
    REQUIRE(sweptEnergy > 0.0);
    INFO("sweep-vs-static difference energy ratio " << (differenceEnergy / sweptEnergy));
    REQUIRE(differenceEnergy >= 0.1 * sweptEnergy);

    REQUIRE(fixedStep > 0.0f);
    std::cout << "[contract] moving-pickup click metric: swept max |dx| " << sweptStep << ", static " << fixedStep
              << ", ratio " << (sweptStep / fixedStep) << " (limit " << kClickRatioLimit << ")\n";
    INFO("swept max |dx| " << sweptStep << " vs static " << fixedStep << " -> ratio " << (sweptStep / fixedStep));
    REQUIRE(sweptStep <= kClickRatioLimit * fixedStep);
}

// ---------------------------------------------------------------------------------------------
// bridge port seam
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: StringNetwork drives the bridge port but keeps P1's internal termination", "[contract]") {
    // The P1 boundary, stated as a test: the port SEES every string's outgoing bridge wave and
    // its bridgeOutput() fills bridgeOutputBuffer(), but its reflected waves are not routed back
    // into the strings, because that insertion costs one extra sample of loop and the loop-length
    // term that subtracts it again is BridgeJunction's (P2.4). A port returning a deliberately
    // absurd reflection must therefore not move a single audio sample in P1.
    RecordingPort port;

    StringNetwork<float> network;
    configure(network, defaultParams());
    network.setBridgePort(port);
    REQUIRE(port.prepareCalls == 1);
    REQUIRE(port.preparedPorts == cnpg::dsp::kMaxStrings); // prepared for capacity, not the count
    REQUIRE(port.preparedRate == kRate);
    REQUIRE(port.preparedBlockSize == kBlock);
    REQUIRE(port.impedanceOfPort0 == 1.0f);

    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));
    const std::vector<float> withPort = renderTap(network, events, 40);

    StringNetwork<float> reference;
    configure(reference, defaultParams());
    BlockEventQueue referenceEvents;
    referenceEvents.push(noteOn(0, kMidiNote));
    const std::vector<float> withInternalTermination = renderTap(reference, referenceEvents, 40);

    REQUIRE(peakOf(withPort) > 0.001f);
    for (std::size_t i = 0; i < withPort.size(); ++i)
        REQUIRE(withPort[i] == withInternalTermination[i]);

    // The port is genuinely driven, not merely held: it saw non-zero incident waves...
    REQUIRE(port.incidentLog.size() == withPort.size());
    bool sawIncident = false;
    for (float value : port.incidentLog)
        sawIncident |= (value != 0.0f);
    REQUIRE(sawIncident);

    // ...and its bridgeOutput() is what lands in bridgeOutputBuffer().
    const float* bridge = network.bridgeOutputBuffer();
    const std::size_t lastBlockStart = withPort.size() - static_cast<std::size_t>(kBlock);
    for (int n = 0; n < kBlock; ++n)
        REQUIRE(bridge[n] == port.incidentLog[lastBlockStart + static_cast<std::size_t>(n)]);

    network.setLosslessTestMode(true);
    REQUIRE(port.lossBypassed);
    network.setLosslessTestMode(false);
    REQUIRE_FALSE(port.lossBypassed);
}

TEST_CASE("CONTRACT: StringNetwork's P1 bridge output is identically zero", "[contract]") {
    // The rigid termination carries no load, so there is no load velocity to feed a body node:
    // bridgeOutput() is 0 by construction (IBridgePort.h). Asserted rather than assumed, because
    // docs/plan.md section 4.3 captures bridgeOutputBuffer() as a golden channel and this is the
    // reason P1's goldens do not -- a 3 s run of zeros per scenario is not a reference.
    StringNetwork<float> network;
    configure(network, defaultParams());
    BlockEventQueue events;
    events.push(noteOn(0, kMidiNote));

    for (int b = 0; b < 200; ++b) {
        network.process(events, kBlock);
        for (int n = 0; n < kBlock; ++n)
            REQUIRE(network.bridgeOutputBuffer()[n] == 0.0f);
    }
    REQUIRE(network.energyEstimate() > 0.0); // the string really was ringing throughout
}

TEST_CASE("CONTRACT: StringNetwork injectFeedback is audibly inert in P1", "[contract]") {
    // The P4 seam: declared now, implemented in P4. Calling it must not perturb one sample.
    auto render = [](bool callFeedback) {
        StringNetwork<float> network;
        configure(network, defaultParams());
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote));

        std::vector<float> feed(kBlock, 0.7f);
        std::vector<float> out;
        for (int b = 0; b < 40; ++b) {
            if (callFeedback)
                network.injectFeedback(feed.data(), kBlock, 3.5f, 0.9f);
            network.process(events, kBlock);
            const float* channel = network.tapBuffers().channel(0);
            out.insert(out.end(), channel, channel + kBlock);
        }
        return out;
    };

    const std::vector<float> quiet = render(false);
    const std::vector<float> fed = render(true);
    REQUIRE(peakOf(quiet) > 0.001f);
    for (std::size_t i = 0; i < quiet.size(); ++i)
        REQUIRE(quiet[i] == fed[i]);
}

// ---------------------------------------------------------------------------------------------
// MIDI pitch-wheel mapping (docs/plan.md Task P1.5 step 4)
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: pitch wheel maps onto the +/-2 semitone bend range", "[contract]") {
    using cnpg::dsp::pitchWheelToSemitones;

    REQUIRE(pitchWheelToSemitones(cnpg::dsp::kPitchWheelCentre) == 0.0f);
    REQUIRE(pitchWheelToSemitones(cnpg::dsp::kPitchWheelMax) == cnpg::dsp::kPitchBendRangeSemitones);
    REQUIRE(pitchWheelToSemitones(0) == -cnpg::dsp::kPitchBendRangeSemitones);

    // Out-of-range input is clamped, never extrapolated: the rails are only sized for this range.
    REQUIRE(pitchWheelToSemitones(-1) == -cnpg::dsp::kPitchBendRangeSemitones);
    REQUIRE(pitchWheelToSemitones(999999) == cnpg::dsp::kPitchBendRangeSemitones);

    // Monotone, and half-scale lands on half the range either side.
    REQUIRE(std::fabs(pitchWheelToSemitones(cnpg::dsp::kPitchWheelCentre + 4096) - 1.0f) < 0.001f);
    REQUIRE(std::fabs(pitchWheelToSemitones(cnpg::dsp::kPitchWheelCentre - 4096) + 1.0f) < 0.001f);
    float previous = -3.0f;
    for (int wheel = 0; wheel <= cnpg::dsp::kPitchWheelMax; wheel += 37) {
        const float semitones = pitchWheelToSemitones(wheel);
        REQUIRE(semitones >= previous);
        previous = semitones;
    }
}

#include "cnpg/dsp/PickupTap.h"

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::PickupTap;
using cnpg::dsp::PickupTapParams;
using cnpg::dsp::Sample;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// ---------------------------------------------------------------------------------------------
// Reference RLC-biquad math -- independently re-derived from docs/plan.md section 2.8 (the RBJ
// constant-SKIRT-gain bandpass PickupTap.cpp's computeCoeffs() implements -- the bilinear
// discretization of the displacement->EMF transfer function s*w0^2/(s^2+(w0/Q)s+w0^2); see
// PickupTap.h for why the transducer's own differentiation makes this the physically correct
// topology, peak gain == q by construction), NOT shared code with dsp/src/PickupTap.cpp. This is
// the "analytic RLC magnitude response" the acceptance criteria names: a closed-form evaluation
// of the z-domain transfer function, independent of running any samples through a filter.
// ---------------------------------------------------------------------------------------------

struct RefCoeffs {
    double b0, b1, b2, a1, a2;
};

RefCoeffs referenceCoeffs(double resonanceHz, double q, double sampleRate) {
    const double w0 = kTwoPi * resonanceHz / sampleRate;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double a0 = 1.0 + alpha;
    const double b0 = std::sin(w0) / 2.0 / a0;
    return RefCoeffs{b0, 0.0, -b0, (-2.0 * std::cos(w0)) / a0, (1.0 - alpha) / a0};
}

double referenceMagnitude(const RefCoeffs& c, double testHz, double sampleRate) {
    const double w = kTwoPi * testHz / sampleRate;
    const std::complex<double> z = std::polar(1.0, -w); // z^-1 on the unit circle
    const std::complex<double> num = c.b0 + c.b1 * z + c.b2 * z * z;
    const std::complex<double> den = 1.0 + c.a1 * z + c.a2 * z * z;
    return std::abs(num / den);
}

double dBOf(double linear) { return 20.0 * std::log10(std::max(linear, 1.0e-12)); }

PickupTapParams makeParams(float resonanceHz, float q, float outputGainDb) {
    PickupTapParams p;
    p.resonanceHz = resonanceHz;
    p.q = q;
    p.outputGainDb = outputGainDb;
    return p;
}

// Renders a sine of frequency testHz through PickupTap::processMono in maxBlockSize-sized
// chunks, discards a settle prefix (long enough for the biquad's own pole to ring down for any
// q this suite exercises), and returns the measured linear gain: steady-state output RMS over
// the measured window, divided by the input's own RMS (amplitude / sqrt(2)).
double measureMagnitude(PickupTap& tap, int maxBlockSize, double testHz, double sampleRate, double settleSeconds,
                        double measureSeconds, double amplitude = 0.5) {
    const int settleSamples = static_cast<int>(settleSeconds * sampleRate);
    const int measureSamples = static_cast<int>(measureSeconds * sampleRate);
    const int total = settleSamples + measureSamples;

    std::vector<Sample> in(static_cast<std::size_t>(total));
    std::vector<Sample> out(static_cast<std::size_t>(total));
    const double w = kTwoPi * testHz / sampleRate;
    for (int n = 0; n < total; ++n)
        in[static_cast<std::size_t>(n)] = static_cast<Sample>(amplitude * std::sin(w * static_cast<double>(n)));

    int offset = 0;
    while (offset < total) {
        const int chunk = std::min(maxBlockSize, total - offset);
        tap.processMono(in.data() + offset, out.data() + offset, chunk);
        offset += chunk;
    }

    double sumSquares = 0.0;
    for (int n = settleSamples; n < total; ++n) {
        const double s = static_cast<double>(out[static_cast<std::size_t>(n)]);
        sumSquares += s * s;
    }
    const double outputRms = std::sqrt(sumSquares / static_cast<double>(measureSamples));
    const double inputRms = amplitude / std::sqrt(2.0);
    return outputRms / inputRms;
}

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = 0.8f;
    event.pluckPosition = 0.28f;
    event.hardness = 0.5f;
    return event;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// sine sweep vs. the analytic RLC magnitude response
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap RLC biquad magnitude matches the analytic response within 0.5 dB", "[contract]") {
    // docs/plan.md Task P1.6 acceptance criterion: "sine sweep through the biquad matches the
    // analytic RLC magnitude response within +/-0.5 dB at 20 log-spaced frequencies". Measured
    // through processMono() -- see PickupTap.h for why process(taps, ...) itself cannot be
    // driven by a synthetic sine (a StringTapBuffers can only come from a real StringNetwork).
    constexpr float kResonanceHz = 2500.0f;
    constexpr int kMaxBlock = 512;
    constexpr int kNumFreqs = 20;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        for (float q : {0.7f, 2.0f, 6.0f}) {
            PickupTap tap;
            tap.prepare(sampleRate, kMaxBlock);
            tap.setParams(makeParams(kResonanceHz, q, 0.0f)); // 0 dB trim isolates the biquad itself
            tap.reset();                                      // settle the coefficient ramp instantly

            const RefCoeffs ref = referenceCoeffs(kResonanceHz, q, sampleRate);

            const double loHz = 50.0;
            const double hiHz = 0.4 * sampleRate;
            for (int i = 0; i < kNumFreqs; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(kNumFreqs - 1);
                const double testHz = loHz * std::pow(hiHz / loHz, t);

                const double expectedDb = dBOf(referenceMagnitude(ref, testHz, sampleRate));
                const double measuredDb = dBOf(measureMagnitude(tap, kMaxBlock, testHz, sampleRate, 0.05, 0.15));

                INFO("rate " << sampleRate << " q " << q << " testHz " << testHz << " expectedDb " << expectedDb
                             << " measuredDb " << measuredDb);
                REQUIRE(std::fabs(measuredDb - expectedDb) <= 0.5);

                tap.reset(); // clear filter memory before the next frequency point
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// resonance peak frequency accuracy
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap resonance peak lands within 1% of resonanceHz", "[contract]") {
    // docs/plan.md Task P1.6 acceptance criterion: "Resonance peak frequency measured within
    // +/-1% of resonanceHz for q in {0.7, 2, 6}". A fine linear grid over +/-10% of resonanceHz,
    // refined by a parabola fit through the best point's log-magnitude (dB) and its neighbours --
    // the same technique tests/support/SpectralAnalysis.cpp's findPeakHz uses -- so the estimate
    // is not limited to the grid's own resolution.
    constexpr float kResonanceHz = 2500.0f;
    constexpr int kMaxBlock = 512;
    constexpr int kGridPoints = 41;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        for (float q : {0.7f, 2.0f, 6.0f}) {
            PickupTap tap;
            tap.prepare(sampleRate, kMaxBlock);
            tap.setParams(makeParams(kResonanceHz, q, 0.0f));
            tap.reset();

            std::array<double, kGridPoints> freqs{};
            std::array<double, kGridPoints> magsDb{};
            const double loHz = static_cast<double>(kResonanceHz) * 0.9;
            const double hiHz = static_cast<double>(kResonanceHz) * 1.1;
            for (int i = 0; i < kGridPoints; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(kGridPoints - 1);
                freqs[static_cast<std::size_t>(i)] = loHz + (hiHz - loHz) * t;
                const double linear =
                    measureMagnitude(tap, kMaxBlock, freqs[static_cast<std::size_t>(i)], sampleRate, 0.1, 0.3);
                magsDb[static_cast<std::size_t>(i)] = dBOf(linear);
                tap.reset();
            }

            int peakIndex = 0;
            for (int i = 1; i < kGridPoints; ++i)
                if (magsDb[static_cast<std::size_t>(i)] > magsDb[static_cast<std::size_t>(peakIndex)])
                    peakIndex = i;

            double peakHz = freqs[static_cast<std::size_t>(peakIndex)];
            if (peakIndex > 0 && peakIndex < kGridPoints - 1) {
                const double left = magsDb[static_cast<std::size_t>(peakIndex - 1)];
                const double mid = magsDb[static_cast<std::size_t>(peakIndex)];
                const double right = magsDb[static_cast<std::size_t>(peakIndex + 1)];
                const double denom = left - 2.0 * mid + right;
                if (std::fabs(denom) > 1.0e-12) {
                    double delta = 0.5 * (left - right) / denom;
                    delta = std::clamp(delta, -1.0, 1.0);
                    const double step = freqs[1] - freqs[0];
                    peakHz += delta * step;
                }
            }

            const double errorRatio = std::fabs(peakHz - static_cast<double>(kResonanceHz)) / kResonanceHz;
            INFO("rate " << sampleRate << " q " << q << " peakHz " << peakHz << " errorRatio " << errorRatio);
            REQUIRE(errorRatio <= 0.01);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// inactive strings contribute exactly zero
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap sums only active strings; an untriggered string contributes zero", "[contract]") {
    // A 2-string network where string 0 is plucked and rings, and string 1 is never triggered
    // (silent, isActive() false for the whole render). PickupTap's output must be bit-identical
    // to a 1-string network carrying only string 0 through the identical events/params.
    constexpr int kMaxBlock = 128;
    constexpr int kMidiNote = 45;
    constexpr int kBlocks = 40;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        StringNetworkParams netParams;
        netParams.pickupPosition01 = 0.6f;

        StringNetwork<float> twoString;
        twoString.prepare(sampleRate, kMaxBlock, FractionalDelayKind::Lagrange3);
        twoString.setNumStrings(2);
        twoString.setParams(netParams);
        twoString.reset();
        BlockEventQueue twoStringEvents;
        twoStringEvents.push(noteOn(0, kMidiNote, 0));

        StringNetwork<float> oneString;
        oneString.prepare(sampleRate, kMaxBlock, FractionalDelayKind::Lagrange3);
        oneString.setNumStrings(1);
        oneString.setParams(netParams);
        oneString.reset();
        BlockEventQueue oneStringEvents;
        oneStringEvents.push(noteOn(0, kMidiNote, 0));

        PickupTap tapTwoString;
        tapTwoString.prepare(sampleRate, kMaxBlock);
        tapTwoString.setParams(makeParams(2500.0f, 2.0f, 0.0f));
        tapTwoString.reset();

        PickupTap tapOneString;
        tapOneString.prepare(sampleRate, kMaxBlock);
        tapOneString.setParams(makeParams(2500.0f, 2.0f, 0.0f));
        tapOneString.reset();

        std::vector<Sample> twoStringOut;
        std::vector<Sample> oneStringOut;
        twoStringOut.reserve(static_cast<std::size_t>(kBlocks) * static_cast<std::size_t>(kMaxBlock));
        oneStringOut.reserve(twoStringOut.capacity());

        for (int b = 0; b < kBlocks; ++b) {
            twoString.process(twoStringEvents, kMaxBlock);
            oneString.process(oneStringEvents, kMaxBlock);

            REQUIRE(twoString.tapBuffers().isActive(0));
            REQUIRE_FALSE(twoString.tapBuffers().isActive(1));

            std::vector<Sample> chunkA(static_cast<std::size_t>(kMaxBlock));
            std::vector<Sample> chunkB(static_cast<std::size_t>(kMaxBlock));
            tapTwoString.process(twoString.tapBuffers(), chunkA.data(), kMaxBlock);
            tapOneString.process(oneString.tapBuffers(), chunkB.data(), kMaxBlock);

            twoStringOut.insert(twoStringOut.end(), chunkA.begin(), chunkA.end());
            oneStringOut.insert(oneStringOut.end(), chunkB.begin(), chunkB.end());
        }

        bool nonZero = false;
        for (Sample s : twoStringOut)
            nonZero |= (s != 0.0f);
        REQUIRE(nonZero);

        REQUIRE(twoStringOut.size() == oneStringOut.size());
        for (std::size_t i = 0; i < twoStringOut.size(); ++i)
            REQUIRE(twoStringOut[i] == oneStringOut[i]);
    }
}

// ---------------------------------------------------------------------------------------------
// process() never reads/writes past what taps actually holds
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap process clamps to taps.numSamples(), not just its own maxBlockSize", "[contract]") {
    // A StringNetwork's tap buffers are only valid for taps.numSamples() samples per string (a
    // short last block, or -- as exercised here -- a network prepared with a smaller maxBlockSize
    // than this PickupTap's own leaves taps.numSamples() < the caller's requested numSamples).
    // Reading past that walks into the next string's stride, or past the whole tap storage once
    // every string's slot is exhausted: process() must clamp against BOTH taps.numSamples() and
    // its own maxBlockSize_, not just the latter.
    constexpr int kNetworkBlock = 32;
    constexpr int kTapMaxBlock = 128; // 4x the network's block: the mismatch this test exercises
    // Highest supported note (shortest loop, ~10.5 samples at 44.1 kHz per WaveguideString.h) so
    // the excitation has unambiguously reached the default pickup tap well inside a 32-sample
    // window at every rate this loops over -- this test is about the clamp, not about timing a
    // pluck-to-tap arrival.
    constexpr int kMidiNote = 108;
    constexpr Sample kSentinel = 12345.0f;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        StringNetwork<float> network;
        network.prepare(sampleRate, kNetworkBlock, FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        StringNetworkParams netParams;
        network.setParams(netParams);
        network.reset();
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote, 0));
        network.process(events, kNetworkBlock); // a full block: taps.numSamples() == kNetworkBlock
        REQUIRE(network.tapBuffers().numSamples() == kNetworkBlock);

        PickupTap tap;
        tap.prepare(sampleRate, kTapMaxBlock);
        tap.setParams(makeParams(2500.0f, 2.0f, 0.0f));
        tap.reset();

        std::vector<Sample> out(static_cast<std::size_t>(kTapMaxBlock), kSentinel);
        // Deliberately over-requests: kTapMaxBlock (128) exceeds taps.numSamples() (32).
        tap.process(network.tapBuffers(), out.data(), kTapMaxBlock);

        // Untouched beyond the valid window -- proves process() did not read (or write) past
        // taps.numSamples() samples per string.
        for (int n = kNetworkBlock; n < kTapMaxBlock; ++n) {
            INFO("rate " << sampleRate << " sample " << n);
            REQUIRE(out[static_cast<std::size_t>(n)] == kSentinel);
        }

        // The valid portion is still computed correctly: bit-identical to a fresh PickupTap fed
        // the SAME taps with a correctly-sized request.
        PickupTap reference;
        reference.prepare(sampleRate, kTapMaxBlock);
        reference.setParams(makeParams(2500.0f, 2.0f, 0.0f));
        reference.reset();
        std::vector<Sample> expected(static_cast<std::size_t>(kNetworkBlock), 0.0f);
        reference.process(network.tapBuffers(), expected.data(), kNetworkBlock);

        bool sounded = false;
        for (int n = 0; n < kNetworkBlock; ++n) {
            REQUIRE(std::isfinite(out[static_cast<std::size_t>(n)]));
            REQUIRE(out[static_cast<std::size_t>(n)] == expected[static_cast<std::size_t>(n)]);
            sounded |= (out[static_cast<std::size_t>(n)] != 0.0f);
        }
        REQUIRE(sounded);
    }
}

// ---------------------------------------------------------------------------------------------
// parameter steps: no NaN, no jump beyond 6 dB over the settled signal's own natural jump
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap parameter steps stay finite and within 6 dB of the settled jump", "[contract]") {
    // docs/plan.md Task P1.6 acceptance criterion: "parameter steps produce no NaN and no
    // sample-to-sample jump > 6 dB in a steady sine". The baseline for "jump" is the LARGER of
    // the OLD and NEW steady states' own natural adjacent-sample delta -- not just the new one --
    // because a smooth ramp legitimately CONTINUES from wherever the old signal was (a gradual
    // transition from a loud resonance to a quiet one necessarily passes through the loud state's
    // own amplitude on its way down; that is not a click). What must stay bounded is the RAMP's
    // own transient contribution beyond whichever endpoint already has the larger natural swing.
    // The measured window starts one sample BEFORE the step so the step boundary itself --
    // comparing the last pre-step sample to the first post-step one -- is part of what gets
    // checked, not skipped.
    constexpr int kMaxBlock = 256;
    // A large, two-sided jump on purpose (frequency, Q and +/-12 dB gain all move at once): the
    // acceptance criterion is about a real worst-case parameter step, not a gentle one.
    const PickupTapParams oldParams = makeParams(2000.0f, 6.0f, 12.0f);
    const PickupTapParams newParams = makeParams(300.0f, 0.7f, -12.0f);
    const double testHz = static_cast<double>(oldParams.resonanceHz);

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        const int settleSamples = static_cast<int>(0.05 * sampleRate);
        const int measureSamples = static_cast<int>(0.1 * sampleRate);
        const int total = settleSamples + measureSamples;
        const double w = kTwoPi * testHz / sampleRate;

        std::vector<Sample> sine(static_cast<std::size_t>(total));
        for (int n = 0; n < total; ++n)
            sine[static_cast<std::size_t>(n)] = static_cast<Sample>(0.5 * std::sin(w * static_cast<double>(n)));

        // s/n describe a window; the comparisons start at the SECOND element so a window that
        // begins one sample before a region of interest still captures the delta INTO it.
        auto worstAdjacentDelta = [](const Sample* s, int n) {
            float worst = 0.0f;
            for (int i = 1; i < n; ++i)
                worst = std::max(worst, std::fabs(s[i] - s[i - 1]));
            return worst;
        };

        auto processChunked = [&](PickupTap& tap, const Sample* in, Sample* out, int count) {
            int offset = 0;
            while (offset < count) {
                const int chunk = std::min(kMaxBlock, count - offset);
                tap.processMono(in + offset, out + offset, chunk);
                offset += chunk;
            }
        };

        auto steadyStateJump = [&](const PickupTapParams& p) {
            PickupTap refTap;
            refTap.prepare(sampleRate, kMaxBlock);
            refTap.setParams(p);
            refTap.reset();
            std::vector<Sample> refOut(static_cast<std::size_t>(total));
            processChunked(refTap, sine.data(), refOut.data(), total);
            return worstAdjacentDelta(refOut.data() + settleSamples, measureSamples);
        };

        const float oldJump = steadyStateJump(oldParams);
        const float newJump = steadyStateJump(newParams);
        const float refJump = std::max(oldJump, newJump);
        REQUIRE(refJump > 0.0f);

        // Transition: settle under oldParams, then step to newParams mid-stream (no reset()) at
        // the settle/measure boundary, continuing to feed the SAME phase-continuous sine.
        PickupTap tap;
        tap.prepare(sampleRate, kMaxBlock);
        tap.setParams(oldParams);
        tap.reset();
        std::vector<Sample> out(static_cast<std::size_t>(total));
        processChunked(tap, sine.data(), out.data(), settleSamples);
        tap.setParams(newParams); // the step itself
        processChunked(tap, sine.data() + settleSamples, out.data() + settleSamples, measureSamples);

        for (int n = 0; n < total; ++n)
            REQUIRE(std::isfinite(out[static_cast<std::size_t>(n)]));

        // Includes the boundary sample pair: index settleSamples-1 (last pre-step sample) is the
        // first element of this window, so its delta into settleSamples (first post-step sample)
        // is the window's first comparison.
        const float steppedJump = worstAdjacentDelta(out.data() + settleSamples - 1, measureSamples + 1);
        const double jumpDb = 20.0 * std::log10(static_cast<double>(steppedJump) / static_cast<double>(refJump));
        INFO("rate " << sampleRate << " oldJump " << oldJump << " newJump " << newJump << " steppedJump " << steppedJump
                     << " jumpDb " << jumpDb);
        REQUIRE(jumpDb <= 6.0);
    }
}

// ---------------------------------------------------------------------------------------------
// realtime contract
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap process and processMono allocate nothing", "[contract]") {
    constexpr int kMaxBlock = 256;
    constexpr int kBlocks = 200;

    PickupTap tap;
    tap.prepare(48000.0, kMaxBlock);
    tap.setParams(makeParams(3000.0f, 3.0f, -3.0f));
    tap.reset();

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock), 0.1f);
    std::vector<Sample> monoOut(static_cast<std::size_t>(kMaxBlock), 0.0f);
    std::vector<Sample> tapsOut(static_cast<std::size_t>(kMaxBlock), 0.0f);

    StringNetwork<float> network;
    network.prepare(48000.0, kMaxBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(2);
    StringNetworkParams netParams;
    network.setParams(netParams);
    network.reset();
    BlockEventQueue events;
    events.push(noteOn(0, 50, 0));

    cnpg::test::resetAllocationCount();
    for (int b = 0; b < kBlocks; ++b) {
        if (b == 50)
            tap.setParams(makeParams(5000.0f, 5.0f, 3.0f)); // exercise the ramp path too

        tap.processMono(in.data(), monoOut.data(), kMaxBlock);

        network.process(events, kMaxBlock);
        tap.process(network.tapBuffers(), tapsOut.data(), kMaxBlock);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// ---------------------------------------------------------------------------------------------
// module lifecycle
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: PickupTap reset collapses a pending ramp and clears filter memory", "[contract]") {
    constexpr int kMaxBlock = 128;
    PickupTap tap;
    tap.prepare(44100.0, kMaxBlock);
    tap.setParams(makeParams(2500.0f, 2.0f, 0.0f));
    tap.reset();

    // Drive a steady tone so the biquad carries real (non-zero) memory.
    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
    const double w = kTwoPi * 1000.0 / 44100.0;
    for (int n = 0; n < kMaxBlock; ++n)
        in[static_cast<std::size_t>(n)] = static_cast<Sample>(0.5 * std::sin(w * static_cast<double>(n)));
    tap.processMono(in.data(), out.data(), kMaxBlock);
    bool sounded = false;
    for (Sample s : out)
        sounded |= (s != 0.0f);
    REQUIRE(sounded);

    // Retarget without letting the ramp run, then reset(): the very next call must already sit
    // at the new target with no ringing carried over from the old filter memory -- i.e. it must
    // equal a FRESH tap prepared directly with the new params, bit-for-bit.
    tap.setParams(makeParams(6000.0f, 6.0f, 6.0f));
    tap.reset();

    PickupTap fresh;
    fresh.prepare(44100.0, kMaxBlock);
    fresh.setParams(makeParams(6000.0f, 6.0f, 6.0f));
    fresh.reset();

    std::vector<Sample> afterReset(static_cast<std::size_t>(kMaxBlock), 0.0f);
    std::vector<Sample> fromFresh(static_cast<std::size_t>(kMaxBlock), 0.0f);
    tap.processMono(in.data(), afterReset.data(), kMaxBlock);
    fresh.processMono(in.data(), fromFresh.data(), kMaxBlock);

    for (int n = 0; n < kMaxBlock; ++n)
        REQUIRE(afterReset[static_cast<std::size_t>(n)] == fromFresh[static_cast<std::size_t>(n)]);
}

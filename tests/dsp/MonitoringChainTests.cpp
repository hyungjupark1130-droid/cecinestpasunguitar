// Task P1.9 -- the monitoring-chain helpers (CabFilter, SoftClipLimiter), the -18 dBFS
// gain-staging gate, and the full hard-wired P1 chain's NaN-free contract.
//
// The chain under test here is the one docs/plan.md section 2.11/2.13 locks and
// plugin/src/PluginProcessor.cpp wires:
//
//   StringNetwork -> PickupTap -> Oversampler(TriodeStage, bypassable) -> CabFilter (bypassable)
//                 -> OutputGain -> SoftClipLimiter
//
// with the safety clip LAST, which is what makes "rendered peak <= ceilingDb" enforceable. The
// ChainHarness below assembles exactly that, at the same defaults, so a level measured here is a
// level the plugin actually produces.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

using namespace cnpg::dsp;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

double dbOf(double linear) { return 20.0 * std::log10(std::max(linear, 1.0e-30)); }

float dbToLinear(double db) { return static_cast<float>(std::pow(10.0, db / 20.0)); }

// ---------------------------------------------------------------------------------------------
// Shared fixtures
// ---------------------------------------------------------------------------------------------

NoteEvent noteOn(int sampleOffset, int midiNote, float velocity) {
    NoteEvent e{};
    e.type = NoteEventType::NoteOn;
    e.sampleOffset = sampleOffset;
    e.stringIndex = 0;
    e.channel = 0;
    e.midiNote = static_cast<std::uint8_t>(midiNote);
    e.velocity = velocity;
    // kUnspecifiedNoteParam is what NoteAllocator emits for plain MIDI note-on, so the exciter
    // resolves position/hardness against its own defaults -- i.e. exactly the plugin's own path.
    e.pluckPosition = kUnspecifiedNoteParam;
    e.hardness = kUnspecifiedNoteParam;
    return e;
}

NoteEvent noteOff(int sampleOffset, int midiNote) {
    NoteEvent e = noteOn(sampleOffset, midiNote, 0.0f);
    e.type = NoteEventType::NoteOff;
    return e;
}

// Deterministic PRNG for the random-parameter sweep. xorshift32 with a fixed seed: the sweep must
// be reproducible from the test binary alone, never dependent on wall-clock time.
class Rng {
  public:
    explicit Rng(std::uint32_t seed) : state_(seed == 0 ? 1u : seed) {}

    std::uint32_t next() noexcept {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    float unit() noexcept { return static_cast<float>(next() >> 8) / static_cast<float>(1u << 24); }

    float range(float lo, float hi) noexcept { return lo + (hi - lo) * unit(); }

    int intRange(int lo, int hi) noexcept {
        return lo + static_cast<int>(next() % static_cast<std::uint32_t>(hi - lo + 1));
    }

  private:
    std::uint32_t state_;
};

// The locked P1 chain, assembled exactly as plugin/src/PluginProcessor.cpp assembles it.
struct ChainHarness {
    StringNetwork<float> network;
    PickupTap pickup;
    Oversampler oversampler;
    TriodeStage triode;
    CabFilter cab;
    OutputGain outputGain;
    SoftClipLimiter limiter;

    std::vector<Sample> mono;
    std::vector<Sample> prelimiter; // probe point: what reaches the safety clip
    int maxBlockSize = 0;

    void prepare(double sampleRate, int blockSize) {
        maxBlockSize = blockSize;

        network.prepare(sampleRate, blockSize, FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        pickup.prepare(sampleRate, blockSize);
        oversampler.prepare(sampleRate, blockSize, Oversampler::kDefaultFactor);
        // The wrapped nonlinearity runs at factor * sampleRate on blocks of up to
        // maxBlockSize * factor samples (Oversampler.h's wiring note).
        triode.prepare(sampleRate * oversampler.factor(), blockSize * oversampler.factor());
        cab.prepare(sampleRate, blockSize);
        outputGain.prepare(sampleRate, blockSize);
        limiter.prepare(sampleRate, blockSize);

        mono.assign(static_cast<std::size_t>(blockSize), 0.0f);
        prelimiter.assign(static_cast<std::size_t>(blockSize), 0.0f);
    }

    // The SHIPPED defaults -- i.e. what plugin/src/Parameters.cpp's APVTS defaults evaluate to,
    // not simply each param struct's own default-constructed state. The one place those differ is
    // the triode's output trim: TriodeStageParams defaults it to 0 dB (a trim's natural default is
    // no trim) while the assembled chain sets it to kUnityGainOutputTrimDb, which is what makes
    // the chain unity at nominal (see TriodeStage.h). Getting that wrong here would mean measuring
    // levels this instrument does not actually produce.
    void setDefaults() {
        network.setParams(StringNetworkParams{});
        pickup.setParams(PickupTapParams{});

        TriodeStageParams triodeParams;
        triodeParams.outputTrimDb = kUnityGainOutputTrimDb;
        triode.setParams(triodeParams);

        cab.setParams(CabFilterParams{});
        outputGain.setParams(OutputGainParams{});
        limiter.setParams(SoftClipLimiterParams{});
    }

    void reset() noexcept {
        network.reset();
        pickup.reset();
        oversampler.reset();
        triode.reset();
        cab.reset();
        outputGain.reset();
        limiter.reset();
    }

    // One block through the whole chain. Writes the post-CabFilter/post-OutputGain signal (i.e.
    // what the safety clip is handed) into `prelimiter` as well, so a test can assert finiteness
    // BEFORE the limiter's own non-finite guard could mask an upstream regression.
    void processBlock(BlockEventQueue& events, int numSamples) noexcept {
        network.process(events, numSamples);
        pickup.process(network.tapBuffers(), mono.data(), numSamples);
        oversampler.processWrapped(
            mono.data(), mono.data(), numSamples,
            [this](Sample* buffer, int numUpsampled) noexcept { triode.process(buffer, buffer, numUpsampled); });
        cab.process(mono.data(), mono.data(), numSamples);
        outputGain.process(mono.data(), mono.data(), numSamples);
        std::copy(mono.begin(), mono.begin() + numSamples, prelimiter.begin());
        limiter.process(mono.data(), mono.data(), numSamples);
    }
};

// Steady-state magnitude of a CabFilter at `testHz`, measured by running a real sine through
// process() (not by evaluating the coefficients -- the point is to measure the shipped code).
double measureCabMagnitude(double sampleRate, double testHz, int maxBlockSize) {
    CabFilter filter;
    filter.prepare(sampleRate, maxBlockSize);
    filter.setParams(CabFilterParams{});

    const int settleSamples = static_cast<int>(0.05 * sampleRate);
    const int measureSamples = static_cast<int>(0.15 * sampleRate);
    const int total = settleSamples + measureSamples;
    const double amplitude = 0.5;
    const double w = kTwoPi * testHz / sampleRate;

    std::vector<Sample> in(static_cast<std::size_t>(total));
    std::vector<Sample> out(static_cast<std::size_t>(total));
    for (int n = 0; n < total; ++n)
        in[static_cast<std::size_t>(n)] = static_cast<Sample>(amplitude * std::sin(w * static_cast<double>(n)));

    int offset = 0;
    while (offset < total) {
        const int chunk = std::min(maxBlockSize, total - offset);
        filter.process(in.data() + offset, out.data() + offset, chunk);
        offset += chunk;
    }

    double sumSquares = 0.0;
    for (int n = settleSamples; n < total; ++n) {
        const double s = static_cast<double>(out[static_cast<std::size_t>(n)]);
        sumSquares += s * s;
    }
    const double outputRms = std::sqrt(sumSquares / static_cast<double>(measureSamples));
    return outputRms / (amplitude / std::sqrt(2.0));
}

// Frequency at which the response first crosses -3.0103 dB, found by bisection over a bracket the
// caller guarantees is monotone (which a 2nd-order lowpass's magnitude is, everywhere above DC).
double findMinus3dBHz(double sampleRate, int maxBlockSize, double loHz, double hiHz) {
    constexpr double kTargetDb = -3.0102999566398120; // 1/sqrt(2), spelled exactly
    for (int iteration = 0; iteration < 24; ++iteration) {
        const double midHz = 0.5 * (loHz + hiHz);
        if (dbOf(measureCabMagnitude(sampleRate, midHz, maxBlockSize)) > kTargetDb)
            loHz = midHz;
        else
            hiHz = midHz;
    }
    return 0.5 * (loHz + hiHz);
}

// Peak (and mean, for the DC readout) of the pickup output for one plucked note at the shipped
// defaults. This is the exact scenario PickupTap.h's kNominalPickupTrimDb was calibrated against.
struct LevelReading {
    double peak = 0.0;
    double mean = 0.0;
};

LevelReading measurePickupLevel(double sampleRate, int midiNote, float velocity, double seconds,
                                float trimOffsetDb = 0.0f) {
    constexpr int kBlock = 128;
    const int totalSamples = static_cast<int>(seconds * sampleRate);

    StringNetwork<float> network;
    network.prepare(sampleRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(StringNetworkParams{});
    network.reset();

    PickupTap pickup;
    pickup.prepare(sampleRate, kBlock);
    PickupTapParams pickupParams; // shipped defaults, including the calibrated trim
    pickupParams.outputGainDb += trimOffsetDb;
    pickup.setParams(pickupParams);
    pickup.reset();

    BlockEventQueue events;
    events.push(noteOn(0, midiNote, velocity));

    std::vector<Sample> out(static_cast<std::size_t>(kBlock), 0.0f);
    LevelReading reading;
    double sum = 0.0;
    int rendered = 0;
    while (rendered < totalSamples) {
        network.process(events, kBlock);
        pickup.process(network.tapBuffers(), out.data(), kBlock);
        for (int n = 0; n < kBlock; ++n) {
            const double v = static_cast<double>(out[static_cast<std::size_t>(n)]);
            reading.peak = std::max(reading.peak, std::fabs(v));
            sum += v;
        }
        rendered += kBlock;
    }
    reading.mean = sum / static_cast<double>(rendered);
    return reading;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// CabFilter: the fixed design cutoff
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: CabFilter -3 dB point lands within 10% of its fixed design cutoff", "[contract]") {
    // docs/plan.md Task P1.9 acceptance criterion: "CabFilter -3 dB point within +/-10% of its
    // fixed design cutoff". Measured by running real sines through process() and bisecting for the
    // crossing, at every supported rate.
    constexpr int kMaxBlock = 512;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        CabFilter probe;
        probe.prepare(sampleRate, kMaxBlock);
        const double designCutoffHz = probe.effectiveCutoffHz();
        REQUIRE(designCutoffHz == CabFilter::kDesignCutoffHz); // the low-rate clamp must stay idle here

        const double measuredHz = findMinus3dBHz(sampleRate, kMaxBlock, 0.5 * designCutoffHz, 2.0 * designCutoffHz);
        const double errorRatio = std::fabs(measuredHz - designCutoffHz) / designCutoffHz;

        INFO("rate " << sampleRate << " design " << designCutoffHz << " measured " << measuredHz << " error "
                     << errorRatio);
        REQUIRE(errorRatio <= 0.10);
    }
}

TEST_CASE("CONTRACT: CabFilter magnitude matches an analytic 2nd-order Butterworth lowpass within 0.5 dB",
          "[contract]") {
    // The -3 dB criterion alone cannot tell a 2nd-order lowpass from a 1st-order one placed at the
    // same corner (both cross -3 dB there), nor a Butterworth from a differently-damped 2nd-order
    // section that happens to cross -3 dB nearby. Comparing the whole measured magnitude curve
    // against an independently re-derived analytic response pins the order, the Q and the cutoff at
    // once -- the same technique tests/dsp/PickupTapTests.cpp uses for the RLC biquad.
    //
    // The reference below re-derives the RBJ cookbook lowpass from LITERAL design values (5 kHz,
    // Q = 1/sqrt(2)), deliberately not from CabFilter's own kDesignCutoffHz/kDesignQ: a change to
    // either constant is then a change this test can see, rather than one it silently follows.
    constexpr int kMaxBlock = 512;
    constexpr int kNumFreqs = 20;
    constexpr double kRefCutoffHz = 5000.0;
    constexpr double kRefQ = 0.7071067811865475244;

    auto referenceMagnitudeDb = [](double testHz, double sampleRate) {
        const double w0 = kTwoPi * kRefCutoffHz / sampleRate;
        const double cosW0 = std::cos(w0);
        const double alpha = std::sin(w0) / (2.0 * kRefQ);
        const double a0 = 1.0 + alpha;
        const double b0 = ((1.0 - cosW0) * 0.5) / a0;
        const double b1 = (1.0 - cosW0) / a0;
        const double b2 = b0;
        const double a1 = (-2.0 * cosW0) / a0;
        const double a2 = (1.0 - alpha) / a0;

        const double w = kTwoPi * testHz / sampleRate;
        const std::complex<double> z = std::polar(1.0, -w); // z^-1 on the unit circle
        const std::complex<double> num = b0 + b1 * z + b2 * z * z;
        const std::complex<double> den = 1.0 + a1 * z + a2 * z * z;
        return dbOf(std::abs(num / den));
    };

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        const double loHz = 50.0;
        const double hiHz = 0.4 * sampleRate;
        for (int i = 0; i < kNumFreqs; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(kNumFreqs - 1);
            const double testHz = loHz * std::pow(hiHz / loHz, t);

            const double expectedDb = referenceMagnitudeDb(testHz, sampleRate);
            const double measuredDb = dbOf(measureCabMagnitude(sampleRate, testHz, kMaxBlock));

            INFO("rate " << sampleRate << " testHz " << testHz << " expectedDb " << expectedDb << " measuredDb "
                         << measuredDb);
            REQUIRE(std::fabs(measuredDb - expectedDb) <= 0.5);
        }
    }
}

TEST_CASE("CONTRACT: CabFilter bypass is a bit-exact passthrough that keeps filter state running", "[contract]") {
    constexpr int kMaxBlock = 256;
    constexpr double kSampleRate = 48000.0;

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
    const double w = kTwoPi * 8000.0 / kSampleRate; // above the cutoff, so filtered != dry
    for (int n = 0; n < kMaxBlock; ++n)
        in[static_cast<std::size_t>(n)] = static_cast<Sample>(0.5 * std::sin(w * static_cast<double>(n)));

    CabFilter bypassed;
    bypassed.prepare(kSampleRate, kMaxBlock);
    CabFilterParams params;
    params.bypass = true;
    bypassed.setParams(params);

    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
    bypassed.process(in.data(), out.data(), kMaxBlock);
    for (int n = 0; n < kMaxBlock; ++n)
        REQUIRE(out[static_cast<std::size_t>(n)] == in[static_cast<std::size_t>(n)]); // bit-exact

    // Filter state kept advancing while bypassed (CabFilter.h's "Bypass" section): un-bypassing
    // must produce the same samples a never-bypassed filter fed the identical signal would, not the
    // transient of a filter resuming from memory frozen a block ago.
    params.bypass = false;
    bypassed.setParams(params);
    std::vector<Sample> afterBypass(static_cast<std::size_t>(kMaxBlock), 0.0f);
    bypassed.process(in.data(), afterBypass.data(), kMaxBlock);

    CabFilter neverBypassed;
    neverBypassed.prepare(kSampleRate, kMaxBlock);
    neverBypassed.setParams(CabFilterParams{});
    std::vector<Sample> scratch(static_cast<std::size_t>(kMaxBlock), 0.0f);
    neverBypassed.process(in.data(), scratch.data(), kMaxBlock); // the block that was bypassed above
    std::vector<Sample> reference(static_cast<std::size_t>(kMaxBlock), 0.0f);
    neverBypassed.process(in.data(), reference.data(), kMaxBlock);

    bool differsFromDry = false;
    for (int n = 0; n < kMaxBlock; ++n) {
        REQUIRE(afterBypass[static_cast<std::size_t>(n)] == reference[static_cast<std::size_t>(n)]);
        differsFromDry |= (afterBypass[static_cast<std::size_t>(n)] != in[static_cast<std::size_t>(n)]);
    }
    REQUIRE(differsFromDry); // the un-bypassed filter really does something (non-vacuity)
}

TEST_CASE("CONTRACT: CabFilter reset clears filter memory", "[contract]") {
    constexpr int kMaxBlock = 128;
    constexpr double kSampleRate = 44100.0;

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
    const double w = kTwoPi * 1000.0 / kSampleRate;
    for (int n = 0; n < kMaxBlock; ++n)
        in[static_cast<std::size_t>(n)] = static_cast<Sample>(0.5 * std::sin(w * static_cast<double>(n)));

    CabFilter filter;
    filter.prepare(kSampleRate, kMaxBlock);
    filter.setParams(CabFilterParams{});
    std::vector<Sample> warm(static_cast<std::size_t>(kMaxBlock), 0.0f);
    filter.process(in.data(), warm.data(), kMaxBlock);
    filter.reset();

    CabFilter fresh;
    fresh.prepare(kSampleRate, kMaxBlock);
    fresh.setParams(CabFilterParams{});

    std::vector<Sample> afterReset(static_cast<std::size_t>(kMaxBlock), 0.0f);
    std::vector<Sample> fromFresh(static_cast<std::size_t>(kMaxBlock), 0.0f);
    filter.process(in.data(), afterReset.data(), kMaxBlock);
    fresh.process(in.data(), fromFresh.data(), kMaxBlock);

    for (int n = 0; n < kMaxBlock; ++n)
        REQUIRE(afterReset[static_cast<std::size_t>(n)] == fromFresh[static_cast<std::size_t>(n)]);
}

// ---------------------------------------------------------------------------------------------
// SoftClipLimiter: the ceiling is a bound, not a suggestion
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: SoftClipLimiter never exceeds ceilingDb + 0.1 dB for a +12 dB overshoot input", "[contract]") {
    // docs/plan.md Task P1.9 acceptance criterion, verbatim. Swept over the whole APVTS ceiling
    // range and every supported rate; the stimulus is a sine at ceiling + 12 dB, plus (because the
    // shape's asymptote claim is stronger than the criterion) a +40 dB one and a full-scale DC-ish
    // block.
    constexpr int kMaxBlock = 256;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        for (float ceilingDb : {-0.3f, -1.0f, -6.0f, -12.0f}) {
            for (double overshootDb : {12.0, 40.0}) {
                SoftClipLimiter limiter;
                limiter.prepare(sampleRate, kMaxBlock);
                SoftClipLimiterParams params;
                params.ceilingDb = ceilingDb;
                limiter.setParams(params);
                limiter.reset(); // settle the ceiling ramp: the bound must hold from sample 0

                const float amplitude = dbToLinear(static_cast<double>(ceilingDb) + overshootDb);
                const double w = kTwoPi * 997.0 / sampleRate;

                std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
                std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
                for (int n = 0; n < kMaxBlock; ++n)
                    in[static_cast<std::size_t>(n)] =
                        static_cast<Sample>(amplitude * std::sin(w * static_cast<double>(n)));
                limiter.process(in.data(), out.data(), kMaxBlock);

                double peak = 0.0;
                for (Sample s : out) {
                    REQUIRE(std::isfinite(s));
                    peak = std::max(peak, std::fabs(static_cast<double>(s)));
                }

                INFO("rate " << sampleRate << " ceilingDb " << ceilingDb << " overshootDb " << overshootDb << " peakDb "
                             << dbOf(peak));
                REQUIRE(dbOf(peak) <= static_cast<double>(ceilingDb) + 0.1);

                // Non-vacuity: the stage must actually be engaging, not passing a quiet signal.
                REQUIRE(dbOf(peak) >= static_cast<double>(ceilingDb) - 3.0);
            }
        }
    }
}

TEST_CASE("CONTRACT: SoftClipLimiter bounds any finite input, however absurd", "[contract]") {
    // The shape's ceiling is an ASYMPTOTE (SoftClipLimiter.h property 3), so the bound cannot be
    // escaped by feeding it more level -- including values a broken upstream stage might produce.
    constexpr int kMaxBlock = 64;
    constexpr float kCeilingDb = -0.3f;

    SoftClipLimiter limiter;
    limiter.prepare(48000.0, kMaxBlock);
    SoftClipLimiterParams params;
    params.ceilingDb = kCeilingDb;
    limiter.setParams(params);
    limiter.reset();

    const std::vector<Sample> extremes{1.0f,    -1.0f,   10.0f,    -10.0f,  1.0e6f,
                                       -1.0e6f, 1.0e30f, -1.0e30f, 3.4e38f, -3.4e38f};
    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock), 0.0f);
    for (std::size_t i = 0; i < extremes.size(); ++i)
        in[i] = extremes[i];

    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
    limiter.process(in.data(), out.data(), kMaxBlock);

    const double ceiling = static_cast<double>(limiter.ceilingLinear());
    for (int n = 0; n < kMaxBlock; ++n) {
        const double v = static_cast<double>(out[static_cast<std::size_t>(n)]);
        INFO("sample " << n << " in " << in[static_cast<std::size_t>(n)] << " out " << v);
        REQUIRE(std::isfinite(v));
        REQUIRE(std::fabs(v) <= ceiling);
    }
}

TEST_CASE("CONTRACT: SoftClipLimiter is the bit-exact identity below its knee", "[contract]") {
    // SoftClipLimiter.h property 1: a safety clip that coloured everything below the ceiling would
    // be an unmeasurable permanent colouration on the whole instrument. Checked over the ENTIRE
    // sub-knee range, at every ceiling in the APVTS range.
    constexpr int kMaxBlock = 512;

    for (float ceilingDb : {-0.3f, -1.0f, -6.0f, -12.0f}) {
        SoftClipLimiter limiter;
        limiter.prepare(48000.0, kMaxBlock);
        SoftClipLimiterParams params;
        params.ceilingDb = ceilingDb;
        limiter.setParams(params);
        limiter.reset();

        const float knee = static_cast<float>(SoftClipLimiter::kKneeStart) * limiter.ceilingLinear();

        std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
        for (int n = 0; n < kMaxBlock; ++n) {
            // A full sweep from -knee to +knee, endpoints included (the knee itself is the identity
            // branch's inclusive upper bound).
            const float t = static_cast<float>(n) / static_cast<float>(kMaxBlock - 1);
            in[static_cast<std::size_t>(n)] = knee * (2.0f * t - 1.0f);
        }

        std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
        limiter.process(in.data(), out.data(), kMaxBlock);
        for (int n = 0; n < kMaxBlock; ++n)
            REQUIRE(out[static_cast<std::size_t>(n)] == in[static_cast<std::size_t>(n)]);
    }
}

TEST_CASE("CONTRACT: SoftClipLimiter replaces a non-finite sample with silence", "[contract]") {
    // The chain's last line of defence (SoftClipLimiter.h): a NaN or Inf must never reach the host.
    constexpr int kMaxBlock = 16;

    SoftClipLimiter limiter;
    limiter.prepare(48000.0, kMaxBlock);
    limiter.setParams(SoftClipLimiterParams{});
    limiter.reset();

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock), 0.1f);
    in[2] = std::numeric_limits<float>::quiet_NaN();
    in[5] = std::numeric_limits<float>::infinity();
    in[9] = -std::numeric_limits<float>::infinity();

    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
    limiter.process(in.data(), out.data(), kMaxBlock);

    for (Sample s : out)
        REQUIRE(std::isfinite(s));
    REQUIRE(out[2] == 0.0f);
    REQUIRE(out[5] == 0.0f);
    REQUIRE(out[9] == 0.0f);
    REQUIRE(out[0] == 0.1f); // the finite neighbours are untouched (below the knee: identity)
}

// ---------------------------------------------------------------------------------------------
// Gain staging: the -18 dBFS gate
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: single-string peak at the pickup output is -18 dBFS +/- 1 dB", "[contract]") {
    // docs/plan.md Task P1.9 acceptance criterion: "measured single-string peak = -18 dBFS +/- 1 dB
    // at the pickup output (automated test)". The calibration reference is the scenario documented
    // on PickupTap.h's kNominalPickupTrimDb: MIDI 45 (A2, the guitar's open A string) at velocity
    // 1.0, every other parameter at its shipped default -- which is what PickupTapParams{},
    // StringNetworkParams{} and the plugin's own APVTS defaults all evaluate to.
    constexpr int kCalibrationNote = 45;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        const LevelReading reading = measurePickupLevel(sampleRate, kCalibrationNote, 1.0f, 2.0);
        const double peakDb = dbOf(reading.peak);

        INFO("rate " << sampleRate << " peakDbFS " << peakDb);
        REQUIRE(peakDb >= -19.0);
        REQUIRE(peakDb <= -17.0);
    }
}

TEST_CASE("CONTRACT: kUnityGainOutputTrimDb makes TriodeStage unity small-signal gain in the sample domain",
          "[contract]") {
    // The second half of the chain's gain staging (TriodeStage.h's "Sample-domain gain staging"
    // block). The triode's sample-domain small-signal gain at the default drive is
    // kGridVoltsPerFullScale; kUnityGainOutputTrimDb is DERIVED as exactly -20*log10 of that, and
    // plugin/src/Parameters.cpp ships it as the triodeOutputTrimDb default. This test is what keeps
    // the derivation honest: change either constant without the other and it fails.
    constexpr int kMaxBlock = 64;
    // Small enough to stay in the curve's linear region -- this measures the SMALL-SIGNAL slope,
    // which is the only gain the constant claims to cancel (the header documents the +0.58 dB that
    // real signal levels add on top, deliberately untrimmed).
    constexpr float kProbeAmplitude = 1.0e-4f;

    TriodeStage stage;
    stage.prepare(48000.0, kMaxBlock);
    TriodeStageParams params; // default drive 0.5
    params.outputTrimDb = kUnityGainOutputTrimDb;
    stage.setParams(params);
    stage.reset();

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock), 0.0f);
    in[0] = kProbeAmplitude;
    in[1] = -kProbeAmplitude;
    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), 0.0f);
    stage.process(in.data(), out.data(), kMaxBlock);

    // The stage inverts (TriodeStage.h): a positive input gives a negative output of equal
    // magnitude once the trim has cancelled kGridVoltsPerFullScale.
    const double gainPositive = static_cast<double>(out[0]) / static_cast<double>(kProbeAmplitude);
    const double gainNegative = static_cast<double>(out[1]) / static_cast<double>(-kProbeAmplitude);
    INFO("gainPositive " << gainPositive << " gainNegative " << gainNegative);
    REQUIRE(std::fabs(gainPositive + 1.0) <= 0.01);
    REQUIRE(std::fabs(gainNegative + 1.0) <= 0.01);

    // ... and the constant really is the derivation, not a rounded measurement.
    REQUIRE(std::fabs(static_cast<double>(kUnityGainOutputTrimDb) +
                      20.0 * std::log10(TriodeStage::kGridVoltsPerFullScale)) <= 1.0e-6);
}

TEST_CASE("CONTRACT: the -18 dBFS pickup level is a calibration, not something clamping onto -18 dBFS", "[contract]") {
    // Guards the gate above against passing for the wrong reason. If anything in the chain were
    // clamping, compressing or normalising the level onto -18 dBFS, the gate would still pass while
    // meaning nothing. Two independent checks that it is a genuine open-loop calibration:
    //
    //  (a) The trim is a pure linear gain, so trimming 6 dB off must move the measured peak by
    //      exactly 6 dB -- an exact number, not a window.
    //  (b) The level tracks velocity monotonically. (NOT by exactly 6 dB per halving: PluckExciter
    //      also nudges the latched hardness with velocity (docs/plan.md locked decision Q1), which
    //      changes the burst's spectrum and therefore how much of it the pickup's bandpass passes.
    //      Measured here at 48 kHz: 7.26 dB between velocity 1.0 and 0.5, not 6.02.)
    constexpr int kCalibrationNote = 45;
    constexpr double kSampleRate = 48000.0;

    const double nominalPeakDb = dbOf(measurePickupLevel(kSampleRate, kCalibrationNote, 1.0f, 2.0).peak);
    const double trimmedPeakDb = dbOf(measurePickupLevel(kSampleRate, kCalibrationNote, 1.0f, 2.0, -6.0f).peak);

    INFO("nominal " << nominalPeakDb << " trimmed " << trimmedPeakDb);
    REQUIRE(std::fabs((nominalPeakDb - trimmedPeakDb) - 6.0206) <= 0.05);

    double previousDb = -1000.0;
    for (float velocity : {0.25f, 0.5f, 1.0f}) {
        const double peakDb = dbOf(measurePickupLevel(kSampleRate, kCalibrationNote, velocity, 2.0).peak);
        INFO("velocity " << velocity << " peakDb " << peakDb);
        REQUIRE(peakDb > previousDb);
        previousDb = peakDb;
    }
}

// ---------------------------------------------------------------------------------------------
// Full chain
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: full chain stays NaN-free over a 60 s random-parameter sweep", "[contract]") {
    // docs/plan.md Task P1.9 acceptance criterion: "full-chain NaN-free 60 s random-parameter sweep
    // at 44.1/48/96 kHz". Every parameter in the chain is re-randomised every block (including the
    // two bypass switches and the retrigger mode), notes are triggered and released at random
    // offsets, and the sweep deliberately visits the extremes of every range -- this is a
    // robustness sweep, not a musical one.
    //
    // Finiteness is asserted at TWO points: the chain output, and the PRE-limiter probe. The
    // limiter's own non-finite guard (SoftClipLimiter.h) would otherwise make the output-only
    // assertion unable to fail, no matter what the rest of the chain produced.
    constexpr int kMaxBlock = 256;
    constexpr double kSweepSeconds = 60.0;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        ChainHarness chain;
        chain.prepare(sampleRate, kMaxBlock);
        chain.setDefaults();
        chain.reset();

        Rng rng(0xC0FFEEu);
        BlockEventQueue events;
        // Counted in RENDERED SAMPLES, not in blocks: block sizes are randomised too, so a
        // block-counted loop would sweep only half the audio the criterion asks for.
        const long long targetSamples = static_cast<long long>(kSweepSeconds * sampleRate);
        long long renderedSamples = 0;
        long long nonZeroSamples = 0;
        int soundingNote = -1;
        int block = 0;

        while (renderedSamples < targetSamples) {
            ++block;
            const int numSamples = rng.intRange(1, kMaxBlock);

            StringNetworkParams np;
            np.retriggerMode = (rng.unit() < 0.5f) ? RetriggerMode::Physical : RetriggerMode::Synth;
            np.pitchBendSemitones = rng.range(-kPitchBendRangeSemitones, kPitchBendRangeSemitones);
            np.pickupPosition01 = rng.unit();
            np.stringMaterial.lossGainLow = rng.unit();
            np.stringMaterial.lossGainHigh = rng.unit();
            np.stringMaterial.dispersionAmount = rng.unit();
            np.exciter.defaultPosition = rng.unit();
            np.exciter.defaultHardness = rng.unit();
            np.exciter.noiseAmount = rng.unit();
            chain.network.setParams(np);

            PickupTapParams pp;
            pp.resonanceHz = rng.range(100.0f, 8000.0f);
            pp.q = rng.range(0.1f, 10.0f);
            pp.outputGainDb = rng.range(kNominalPickupTrimDb - 24.0f, kNominalPickupTrimDb + 24.0f);
            chain.pickup.setParams(pp);

            TriodeStageParams tp;
            tp.drive = rng.unit();
            tp.outputTrimDb = rng.range(-24.0f, 24.0f);
            tp.bypass = (rng.unit() < 0.25f);
            chain.triode.setParams(tp);

            CabFilterParams cp;
            cp.bypass = (rng.unit() < 0.5f);
            chain.cab.setParams(cp);

            OutputGainParams gp;
            gp.gainDb = rng.range(-24.0f, 24.0f);
            chain.outputGain.setParams(gp);

            SoftClipLimiterParams lp;
            lp.ceilingDb = rng.range(-12.0f, 0.0f);
            chain.limiter.setParams(lp);

            events.clear();
            if (rng.unit() < 0.10f) {
                if (soundingNote >= 0)
                    events.push(noteOff(rng.intRange(0, numSamples - 1), soundingNote));
                soundingNote = -1;
            } else if (rng.unit() < 0.10f) {
                soundingNote = rng.intRange(kMinMidiNote, kMaxMidiNote);
                events.push(noteOn(rng.intRange(0, numSamples - 1), soundingNote, rng.unit()));
            }

            chain.processBlock(events, numSamples);

            for (int n = 0; n < numSamples; ++n) {
                const Sample preLimiter = chain.prelimiter[static_cast<std::size_t>(n)];
                const Sample output = chain.mono[static_cast<std::size_t>(n)];
                if (!std::isfinite(preLimiter) || !std::isfinite(output)) {
                    INFO("rate " << sampleRate << " block " << block << " sample " << n << " preLimiter " << preLimiter
                                 << " output " << output);
                    REQUIRE(std::isfinite(preLimiter));
                    REQUIRE(std::isfinite(output));
                }
                nonZeroSamples += (output != 0.0f) ? 1 : 0;
            }
            renderedSamples += numSamples;
        }

        // Non-vacuity: a chain that produced pure silence -- or one that only sounded for a handful
        // of samples around each note-on -- would satisfy every finiteness assertion above without
        // ever exercising the code they are meant to gate. Requiring the majority of the swept
        // audio to be non-silent is what makes this a real 60 s workout of the chain.
        INFO("rate " << sampleRate << " rendered " << renderedSamples << " nonZero " << nonZeroSamples);
        REQUIRE(nonZeroSamples > renderedSamples / 2);
    }
}

TEST_CASE("CONTRACT: full chain output never exceeds the limiter ceiling", "[contract]") {
    // The reason docs/plan.md section 2.11 hard-wires the safety clip LAST: with the ceiling at the
    // end of the chain, "rendered peak <= ceilingDb" is enforceable no matter what the stages ahead
    // of it do. Driven hard on purpose -- maximum drive into +24 dB of output gain, which is far
    // past anything the instrument produces in normal use.
    constexpr int kMaxBlock = 256;
    constexpr float kCeilingDb = -0.3f;

    for (double sampleRate : {44100.0, 48000.0, 96000.0}) {
        ChainHarness chain;
        chain.prepare(sampleRate, kMaxBlock);
        chain.setDefaults();

        TriodeStageParams tp;
        tp.drive = 1.0f;
        tp.outputTrimDb = 24.0f;
        chain.triode.setParams(tp);

        OutputGainParams gp;
        gp.gainDb = 24.0f;
        chain.outputGain.setParams(gp);

        SoftClipLimiterParams lp;
        lp.ceilingDb = kCeilingDb;
        chain.limiter.setParams(lp);
        chain.reset(); // settle every ramp so the ceiling bounds the very first sample

        BlockEventQueue events;
        events.push(noteOn(0, 45, 1.0f));

        const int totalBlocks = static_cast<int>(1.0 * sampleRate / kMaxBlock);
        double peak = 0.0;
        for (int block = 0; block < totalBlocks; ++block) {
            chain.processBlock(events, kMaxBlock);
            for (int n = 0; n < kMaxBlock; ++n)
                peak = std::max(peak, std::fabs(static_cast<double>(chain.mono[static_cast<std::size_t>(n)])));
        }

        INFO("rate " << sampleRate << " peakDb " << dbOf(peak));
        REQUIRE(dbOf(peak) <= static_cast<double>(kCeilingDb) + 0.1);
        // Non-vacuity: this drive really does push the limiter into its knee.
        REQUIRE(dbOf(peak) >= static_cast<double>(kCeilingDb) - 1.0);
    }
}

TEST_CASE("CONTRACT: full chain process allocates nothing", "[contract]") {
    constexpr int kMaxBlock = 256;
    constexpr int kBlocks = 500;

    ChainHarness chain;
    chain.prepare(48000.0, kMaxBlock);
    chain.setDefaults();
    chain.reset();

    BlockEventQueue events;

    cnpg::test::resetAllocationCount();
    for (int block = 0; block < kBlocks; ++block) {
        events.clear();
        if (block % 97 == 0)
            events.push(noteOn(0, 45 + (block % 12), 0.9f));

        if (block % 31 == 0) {
            // Exercise every ramp path too, not only the settled one.
            PickupTapParams pp;
            pp.resonanceHz = 1500.0f + static_cast<float>(block);
            chain.pickup.setParams(pp);

            TriodeStageParams tp;
            tp.drive = 0.25f + 0.5f * static_cast<float>(block % 3);
            chain.triode.setParams(tp);

            SoftClipLimiterParams lp;
            lp.ceilingDb = -0.3f - static_cast<float>(block % 6);
            chain.limiter.setParams(lp);
        }

        chain.processBlock(events, kMaxBlock);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// ---------------------------------------------------------------------------------------------
// Report-only probe (hidden: "[.]" is never discovered by CTest, so it never runs in CI). Prints
// the level/DC/latency table Task P1.9's report records. Run it with:
//   build/bin/Release/cnpg_tests.exe "PROBE: P1.9 chain levels, DC and latency"
// ---------------------------------------------------------------------------------------------

TEST_CASE("PROBE: P1.9 chain levels, DC and latency", "[.][probe]") {
    constexpr int kMaxBlock = 128;

    std::printf("\n-- pickup output peak at shipped defaults, velocity 1.0 (dBFS) --\n");
    std::printf("%-8s", "note");
    for (double sampleRate : {44100.0, 48000.0, 96000.0})
        std::printf("%12.0f", sampleRate);
    std::printf("\n");
    for (int note : {21, 28, 33, 40, 45, 52, 57, 64, 76, 88, 96, 108}) {
        std::printf("%-8d", note);
        for (double sampleRate : {44100.0, 48000.0, 96000.0})
            std::printf("%12.3f", dbOf(measurePickupLevel(sampleRate, note, 1.0f, 2.0).peak));
        std::printf("\n");
    }

    // Full chain, at the shipped defaults and again with the whole documented +16 dB multi-string
    // summing budget applied at the pickup trim (what a maximum-velocity full chord presents to the
    // triode). The DC columns are what the chain does about the triode's signal-dependent DC
    // offset: nothing removes it -- PickupTap's bandpass has a DC zero but sits AHEAD of the
    // triode, and CabFilter is a lowpass -- so it reaches the output as measured here.
    auto renderChain = [&](double sampleRate, float pickupTrimOffsetDb) {
        ChainHarness chain;
        chain.prepare(sampleRate, kMaxBlock);
        chain.setDefaults();
        PickupTapParams pickupParams;
        pickupParams.outputGainDb += pickupTrimOffsetDb;
        chain.pickup.setParams(pickupParams);
        chain.reset();

        BlockEventQueue events;
        events.push(noteOn(0, 45, 1.0f));

        const int totalBlocks = static_cast<int>(2.0 * sampleRate / kMaxBlock);
        const int loudBlocks = static_cast<int>(0.1 * sampleRate / kMaxBlock);
        double outPeak = 0.0;
        double prePeak = 0.0;
        double sumAll = 0.0;
        double sumLoud = 0.0;
        for (int block = 0; block < totalBlocks; ++block) {
            chain.processBlock(events, kMaxBlock);
            for (int n = 0; n < kMaxBlock; ++n) {
                const double v = static_cast<double>(chain.mono[static_cast<std::size_t>(n)]);
                outPeak = std::max(outPeak, std::fabs(v));
                prePeak =
                    std::max(prePeak, std::fabs(static_cast<double>(chain.prelimiter[static_cast<std::size_t>(n)])));
                sumAll += v;
                if (block < loudBlocks)
                    sumLoud += v;
            }
        }
        std::printf("%-8.0f %-10.3f %-12.3f %-14.8f %-14.3f %-14.8f %-10d\n", sampleRate, dbOf(outPeak), dbOf(prePeak),
                    sumAll / static_cast<double>(totalBlocks * kMaxBlock),
                    dbOf(std::fabs(sumAll / static_cast<double>(totalBlocks * kMaxBlock))),
                    sumLoud / static_cast<double>(loudBlocks * kMaxBlock), chain.oversampler.latencySamples());
    };

    for (float trimOffsetDb : {0.0f, 16.0f}) {
        std::printf("\n-- full chain, MIDI 45 velocity 1.0, 2 s, pickup trim %+0.0f dB (%s) --\n",
                    static_cast<double>(trimOffsetDb),
                    trimOffsetDb == 0.0f ? "shipped defaults" : "the +16 dB summing budget");
        std::printf("%-8s %-10s %-12s %-14s %-14s %-14s %-10s\n", "rate", "outPeak", "preLimPeak", "DC(2s mean)",
                    "DC(dB)", "DC(0.1s mean)", "latency");
        for (double sampleRate : {44100.0, 48000.0, 96000.0})
            renderChain(sampleRate, trimOffsetDb);
    }

    REQUIRE(true);
}

#include "cnpg/dsp/Oversampler.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <iostream>
#include <vector>

using cnpg::dsp::Oversampler;
using cnpg::dsp::Sample;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHostRate = 48000.0;
constexpr int kMaxBlock = 512;

// |H(f)| of the polyphase halfband the designer produces, at `normFreq` in cycles/sample of the
// filter's OWN (upsampled) rate. Mirrors Oversampler.cpp's structure exactly: even-indexed
// coefficients in A0, odd in A1, H = 0.5*(A0(z^2) + z^-1 A1(z^2)).
double halfbandMagnitude(const std::vector<double>& coefs, double normFreq) {
    const std::complex<double> z = std::polar(1.0, 2.0 * kPi * normFreq);
    const std::complex<double> zInv2 = 1.0 / (z * z);
    std::complex<double> a0(1.0, 0.0);
    std::complex<double> a1(1.0, 0.0);
    for (std::size_t i = 0; i < coefs.size(); ++i) {
        const std::complex<double> section = (coefs[i] + zInv2) / (1.0 + coefs[i] * zInv2);
        ((i % 2) == 0 ? a0 : a1) *= section;
    }
    return std::abs(0.5 * (a0 + a1 / z));
}

double worstOverRange(const std::vector<double>& coefs, double lo, double hi, bool wantMax) {
    double worst = wantMax ? 0.0 : 1e300;
    constexpr int kSteps = 4000;
    for (int i = 0; i <= kSteps; ++i) {
        const double f = lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(kSteps);
        const double m = halfbandMagnitude(coefs, f);
        worst = wantMax ? std::max(worst, m) : std::min(worst, m);
    }
    return worst;
}

// A band-limited unit pulse: a Blackman-windowed sinc with its cutoff well inside the stage-0
// passband, centred at `centre`. Used instead of a bare impulse to measure the latency the
// Oversampler actually reports -- see Oversampler.h's "Latency" section for why a bare impulse
// measures the transition-band group-delay peak instead of the passband one.
std::vector<Sample> bandLimitedPulse(int length, int centre, double cutoffNorm) {
    std::vector<Sample> pulse(static_cast<std::size_t>(length), Sample{0});
    constexpr int kHalfWidth = 64;
    for (int i = -kHalfWidth; i <= kHalfWidth; ++i) {
        const double t = static_cast<double>(i);
        const double x = 2.0 * cutoffNorm * t;
        const double sinc = (i == 0) ? 1.0 : std::sin(kPi * x) / (kPi * x);
        const double phase = kPi * (t + kHalfWidth) / kHalfWidth;
        const double window = 0.42 - 0.5 * std::cos(phase) + 0.08 * std::cos(2.0 * phase);
        const int index = centre + i;
        if (index >= 0 && index < length)
            pulse[static_cast<std::size_t>(index)] = static_cast<Sample>(sinc * window);
    }
    return pulse;
}

// Lag, in base-rate samples, that maximises the cross-correlation of `out` against `in`, refined
// by a parabola through the log-magnitude of the peak and its neighbours. Returns the fractional
// lag; the caller rounds.
double measureDelay(const std::vector<Sample>& in, const std::vector<Sample>& out, int maxLag) {
    std::vector<double> correlation(static_cast<std::size_t>(maxLag + 1), 0.0);
    const int n = static_cast<int>(in.size());
    for (int lag = 0; lag <= maxLag; ++lag) {
        double acc = 0.0;
        for (int i = 0; i + lag < n; ++i)
            acc += static_cast<double>(in[static_cast<std::size_t>(i)]) *
                   static_cast<double>(out[static_cast<std::size_t>(i + lag)]);
        correlation[static_cast<std::size_t>(lag)] = acc;
    }
    int peak = 0;
    for (int lag = 1; lag <= maxLag; ++lag)
        if (correlation[static_cast<std::size_t>(lag)] > correlation[static_cast<std::size_t>(peak)])
            peak = lag;
    if (peak <= 0 || peak >= maxLag)
        return static_cast<double>(peak);
    const double left = correlation[static_cast<std::size_t>(peak - 1)];
    const double mid = correlation[static_cast<std::size_t>(peak)];
    const double right = correlation[static_cast<std::size_t>(peak + 1)];
    const double denom = left - 2.0 * mid + right;
    double delta = 0.0;
    if (std::fabs(denom) > 1e-300)
        delta = 0.5 * (left - right) / denom;
    delta = std::clamp(delta, -0.5, 0.5);
    return static_cast<double>(peak) + delta;
}

void runChunked(Oversampler& os, const std::vector<Sample>& in, std::vector<Sample>& out, int block) {
    out.assign(in.size(), Sample{0});
    int offset = 0;
    const int total = static_cast<int>(in.size());
    while (offset < total) {
        const int chunk = std::min(block, total - offset);
        os.processWrapped(in.data() + offset, out.data() + offset, chunk, [](Sample*, int) {});
        offset += chunk;
    }
}

} // namespace

TEST_CASE("CONTRACT: Oversampler snaps the factor down to a valid power of two", "[contract]") {
    Oversampler os;

    // Untouched before prepare(): the documented shipping default.
    CHECK(os.factor() == 2);

    struct Case {
        int requested;
        int expected;
    };
    // Snap DOWN to the largest valid power of two in [2, kMaxOversampling] -- see Oversampler.h.
    const Case cases[] = {{-4, 2}, {0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 4},
                          {5, 4},  {6, 4}, {7, 4}, {8, 8}, {9, 8}, {4096, 8}};
    for (const Case& c : cases) {
        os.prepare(kHostRate, kMaxBlock, c.requested);
        INFO("requested factor " << c.requested);
        CHECK(os.factor() == c.expected);
        CHECK(os.oversampledCapacity() == kMaxBlock * c.expected);
    }
}

TEST_CASE("CONTRACT: Oversampler halfband designs meet their stopband and passband specs", "[contract]") {
    // Pins the elliptic design itself (Oversampler.h's "halfband filters" section): the numbers
    // below are what docs/decisions/0003-adaa-vs-oversampling.md's aliasing tables were measured
    // against, so a design change has to move them deliberately.
    struct Expectation {
        int stage;
        double minStopbandDb; // rejection floor over [0.25 + t/2, 0.5]
        double maxRippleDb;   // |passband deviation| over [0, 0.25 - t/2]
    };
    const Expectation expectations[] = {{0, 91.0, 0.001}, {1, 70.0, 0.01}, {2, 70.0, 0.01}};

    for (const Expectation& e : expectations) {
        const Oversampler::HalfbandStageSpec spec = Oversampler::stageSpec(e.stage);
        const std::vector<double> coefs =
            Oversampler::designHalfbandCoefficients(spec.numCoefficients, spec.transitionBandwidth);

        INFO("stage " << e.stage << " with " << spec.numCoefficients << " coefficients, transition "
                      << spec.transitionBandwidth);
        REQUIRE(coefs.size() == static_cast<std::size_t>(spec.numCoefficients));

        // Strictly inside (0, 1) and ascending: both are load-bearing for the alternating branch
        // deal (Oversampler.cpp) and for stability (poles at +/- j*sqrt(a)).
        for (std::size_t i = 0; i < coefs.size(); ++i) {
            CHECK(coefs[i] > 0.0);
            CHECK(coefs[i] < 1.0);
            if (i > 0)
                CHECK(coefs[i] > coefs[i - 1]);
        }

        const double passEdge = 0.25 - spec.transitionBandwidth * 0.5;
        const double stopEdge = 0.25 + spec.transitionBandwidth * 0.5;

        const double worstStop = worstOverRange(coefs, stopEdge, 0.5, true);
        const double stopDb = -20.0 * std::log10(worstStop);
        CHECK(stopDb >= e.minStopbandDb);

        const double pbMin = worstOverRange(coefs, 0.0, passEdge, false);
        const double pbMax = worstOverRange(coefs, 0.0, passEdge, true);
        CHECK(std::fabs(20.0 * std::log10(pbMin)) <= e.maxRippleDb);
        CHECK(std::fabs(20.0 * std::log10(pbMax)) <= e.maxRippleDb);

        // Exactly -3.0103 dB at the halfband's own symmetry point -- the property that makes the
        // two branches power-complementary about a quarter of the rate.
        CHECK(halfbandMagnitude(coefs, 0.25) == Catch::Approx(std::sqrt(0.5)).margin(1e-9));

        std::cout << "[contract] halfband stage " << e.stage << ": " << spec.numCoefficients << " coefs, transition "
                  << spec.transitionBandwidth << ", stopband " << stopDb << " dB\n";
    }
}

TEST_CASE("CONTRACT: Oversampler latencySamples equals the measured passband delay", "[contract]") {
    // Task P1.8 acceptance criterion: latencySamples() equals the measured impulse delay through
    // processWrapped with an identity nonlinearity.
    for (int factor : {2, 4, 8}) {
        Oversampler os;
        os.prepare(kHostRate, kMaxBlock, factor);

        constexpr int kLength = 2048;
        constexpr int kCentre = 256;
        // Cutoff 0.2 of the base rate = 9.6 kHz at 48 kHz: deep inside stage 0's passband, so the
        // measurement sees the flat passband group delay and not the transition-band peak.
        const std::vector<Sample> in = bandLimitedPulse(kLength, kCentre, 0.2);
        std::vector<Sample> out;
        runChunked(os, in, out, 128);

        const double measured = measureDelay(in, out, 32);
        INFO("factor " << factor << ": reported " << os.latencySamples() << ", measured " << measured);
        CHECK(os.latencySamples() == static_cast<int>(std::lround(measured)));
        CHECK(std::fabs(measured - static_cast<double>(os.latencySamples())) <= 0.5);
        CHECK(os.latencySamples() > 0);

        std::cout << "[contract] Oversampler factor " << factor << ": latencySamples() = " << os.latencySamples()
                  << ", measured passband delay " << measured << " base samples\n";
    }
}

TEST_CASE("CONTRACT: Oversampler round trip is transparent across the audio band", "[contract]") {
    for (int factor : {2, 4, 8}) {
        Oversampler os;
        os.prepare(kHostRate, kMaxBlock, factor);

        // 20 Hz .. 15 kHz: everything a halfband whose passband edge sits at 23.4 kHz must pass at
        // unity. (The 15-20 kHz octave is still passband, but a 48 kHz-rate probe there needs a
        // longer settle than this case is worth; the design-level test above already pins the whole
        // passband analytically.)
        for (double freq : {20.0, 100.0, 1000.0, 4186.0, 10000.0, 15000.0}) {
            os.reset();
            constexpr int kLength = 16384;
            std::vector<Sample> in(static_cast<std::size_t>(kLength));
            for (int i = 0; i < kLength; ++i)
                in[static_cast<std::size_t>(i)] =
                    static_cast<Sample>(0.5 * std::sin(2.0 * kPi * freq * static_cast<double>(i) / kHostRate));
            std::vector<Sample> out;
            runChunked(os, in, out, 256);

            // RMS ratio over the settled second half, NOT a peak ratio: at 15 kHz a 48 kHz-sampled
            // sine has only 3.2 samples per cycle, so its sampled peak sits well below its true
            // amplitude and lands on a different phase in the (fractionally delayed) output than in
            // the input. RMS is phase-blind, and taking it as a ratio against the input's own RMS
            // over the same window cancels the partial-cycle bias both signals share.
            double inSumSq = 0.0;
            double outSumSq = 0.0;
            for (int i = kLength / 2; i < kLength; ++i) {
                const double x = static_cast<double>(in[static_cast<std::size_t>(i)]);
                const double y = static_cast<double>(out[static_cast<std::size_t>(i)]);
                inSumSq += x * x;
                outSumSq += y * y;
            }
            const double gainDb = 10.0 * std::log10(outSumSq / inSumSq);
            INFO("factor " << factor << " at " << freq << " Hz: gain " << gainDb << " dB");
            CHECK(std::fabs(gainDb) < 0.05);
        }
    }
}

TEST_CASE("CONTRACT: Oversampler split API matches processWrapped sample for sample", "[contract]") {
    for (int factor : {2, 4, 8}) {
        constexpr int kLength = 4096;
        std::vector<Sample> in(static_cast<std::size_t>(kLength));
        for (int i = 0; i < kLength; ++i) {
            const double t = static_cast<double>(i) / kHostRate;
            in[static_cast<std::size_t>(i)] =
                static_cast<Sample>(0.4 * std::sin(2.0 * kPi * 220.0 * t) + 0.2 * std::sin(2.0 * kPi * 3300.0 * t));
        }

        // A deliberately nonlinear callback, so the two paths only agree if the split legs really
        // are the same filters in the same order as the wrapped one.
        auto shaper = [](Sample* buffer, int count) {
            for (int i = 0; i < count; ++i)
                buffer[i] = static_cast<Sample>(std::tanh(3.0 * static_cast<double>(buffer[i])));
        };

        Oversampler wrapped;
        wrapped.prepare(kHostRate, kMaxBlock, factor);
        std::vector<Sample> wrappedOut(static_cast<std::size_t>(kLength), Sample{0});
        for (int offset = 0; offset < kLength; offset += kMaxBlock)
            wrapped.processWrapped(in.data() + offset, wrappedOut.data() + offset, kMaxBlock, shaper);

        Oversampler split;
        split.prepare(kHostRate, kMaxBlock, factor);
        std::vector<Sample> scratch(static_cast<std::size_t>(split.oversampledCapacity()), Sample{0});
        std::vector<Sample> splitOut(static_cast<std::size_t>(kLength), Sample{0});
        for (int offset = 0; offset < kLength; offset += kMaxBlock) {
            const int numUp = split.upsample(in.data() + offset, kMaxBlock, scratch.data());
            REQUIRE(numUp == kMaxBlock * factor);
            shaper(scratch.data(), numUp);
            split.downsample(scratch.data(), numUp, splitOut.data() + offset);
        }

        for (int i = 0; i < kLength; ++i) {
            INFO("factor " << factor << ", sample " << i);
            REQUIRE(wrappedOut[static_cast<std::size_t>(i)] == splitOut[static_cast<std::size_t>(i)]);
        }
    }
}

TEST_CASE("CONTRACT: Oversampler reset clears filter memory completely", "[contract]") {
    for (int factor : {2, 4, 8}) {
        Oversampler os;
        os.prepare(kHostRate, kMaxBlock, factor);

        constexpr int kLength = 1024;
        std::vector<Sample> loud(static_cast<std::size_t>(kLength));
        for (int i = 0; i < kLength; ++i)
            loud[static_cast<std::size_t>(i)] =
                static_cast<Sample>(0.9 * std::sin(2.0 * kPi * 997.0 * static_cast<double>(i) / kHostRate));

        std::vector<Sample> fresh;
        runChunked(os, loud, fresh, kMaxBlock);

        os.reset();
        std::vector<Sample> afterReset;
        runChunked(os, loud, afterReset, kMaxBlock);

        for (int i = 0; i < kLength; ++i)
            REQUIRE(fresh[static_cast<std::size_t>(i)] == afterReset[static_cast<std::size_t>(i)]);

        // And silence in after a reset is silence out, bit-exactly.
        os.reset();
        std::vector<Sample> silence(static_cast<std::size_t>(kLength), Sample{0});
        std::vector<Sample> silentOut;
        runChunked(os, silence, silentOut, kMaxBlock);
        for (int i = 0; i < kLength; ++i)
            REQUIRE(silentOut[static_cast<std::size_t>(i)] == Sample{0});
    }
}

TEST_CASE("CONTRACT: Oversampler handles numSamples from 1 to maxBlockSize and clamps beyond", "[contract]") {
    Oversampler os;
    os.prepare(kHostRate, kMaxBlock, 4);

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock * 2), Sample{0});
    for (std::size_t i = 0; i < in.size(); ++i)
        in[i] = static_cast<Sample>(0.3 * std::sin(0.01 * static_cast<double>(i)));
    std::vector<Sample> out(in.size(), Sample{-99});

    for (int n : {1, 2, 3, 7, 64, kMaxBlock - 1, kMaxBlock}) {
        os.reset();
        std::fill(out.begin(), out.end(), Sample{-99});
        os.processWrapped(in.data(), out.data(), n, [](Sample*, int) {});
        for (int i = 0; i < n; ++i) {
            INFO("n = " << n << ", sample " << i);
            REQUIRE(std::isfinite(out[static_cast<std::size_t>(i)]));
            REQUIRE(out[static_cast<std::size_t>(i)] != Sample{-99});
        }
        REQUIRE(out[static_cast<std::size_t>(n)] == Sample{-99}); // never writes past numSamples
    }

    // Over-long requests are clamped to maxBlockSize, exactly like every other module here.
    os.reset();
    std::fill(out.begin(), out.end(), Sample{-99});
    os.processWrapped(in.data(), out.data(), kMaxBlock * 2, [](Sample*, int) {});
    CHECK(out[static_cast<std::size_t>(kMaxBlock)] == Sample{-99});

    // Zero and negative counts are no-ops.
    std::fill(out.begin(), out.end(), Sample{-99});
    os.processWrapped(in.data(), out.data(), 0, [](Sample*, int) {});
    os.processWrapped(in.data(), out.data(), -5, [](Sample*, int) {});
    CHECK(out[0] == Sample{-99});
}

TEST_CASE("CONTRACT: Oversampler processWrapped allocates nothing", "[contract]") {
    Oversampler os;
    os.prepare(kHostRate, kMaxBlock, 8);

    std::vector<Sample> in(static_cast<std::size_t>(kMaxBlock));
    for (int i = 0; i < kMaxBlock; ++i)
        in[static_cast<std::size_t>(i)] =
            static_cast<Sample>(0.5 * std::sin(2.0 * kPi * 440.0 * static_cast<double>(i) / kHostRate));
    std::vector<Sample> out(static_cast<std::size_t>(kMaxBlock), Sample{0});
    std::vector<Sample> scratch(static_cast<std::size_t>(os.oversampledCapacity()), Sample{0});

    auto shaper = [](Sample* buffer, int count) {
        for (int i = 0; i < count; ++i)
            buffer[i] = static_cast<Sample>(std::tanh(static_cast<double>(buffer[i])));
    };

    cnpg::test::resetAllocationCount();
    for (int block = 0; block < 1000; ++block) {
        os.processWrapped(in.data(), out.data(), kMaxBlock, shaper);
        const int numUp = os.upsample(in.data(), kMaxBlock, scratch.data());
        os.downsample(scratch.data(), numUp, out.data());
        os.reset();
    }
    CHECK(cnpg::test::allocationCount() == 0);
}

TEST_CASE("CONTRACT: Oversampler in-place processWrapped matches the out-of-place result", "[contract]") {
    constexpr int kLength = 2048;
    std::vector<Sample> in(static_cast<std::size_t>(kLength));
    for (int i = 0; i < kLength; ++i)
        in[static_cast<std::size_t>(i)] =
            static_cast<Sample>(0.7 * std::sin(2.0 * kPi * 523.25 * static_cast<double>(i) / kHostRate));

    auto shaper = [](Sample* buffer, int count) {
        for (int i = 0; i < count; ++i)
            buffer[i] = static_cast<Sample>(std::tanh(2.0 * static_cast<double>(buffer[i])));
    };

    Oversampler a;
    a.prepare(kHostRate, kMaxBlock, 2);
    std::vector<Sample> outOfPlace(static_cast<std::size_t>(kLength), Sample{0});
    for (int offset = 0; offset < kLength; offset += kMaxBlock)
        a.processWrapped(in.data() + offset, outOfPlace.data() + offset, kMaxBlock, shaper);

    Oversampler b;
    b.prepare(kHostRate, kMaxBlock, 2);
    std::vector<Sample> inPlace = in;
    for (int offset = 0; offset < kLength; offset += kMaxBlock)
        b.processWrapped(inPlace.data() + offset, inPlace.data() + offset, kMaxBlock, shaper);

    for (int i = 0; i < kLength; ++i)
        REQUIRE(outOfPlace[static_cast<std::size_t>(i)] == inPlace[static_cast<std::size_t>(i)]);
}

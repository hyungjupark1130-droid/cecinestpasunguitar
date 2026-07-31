#include "cnpg/dsp/TriodeStage.h"

#include "support/AllocationGuard.h"
#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

using cnpg::dsp::KorenTriodeParams;
using cnpg::dsp::Sample;
using cnpg::dsp::TriodeStage;
using cnpg::dsp::TriodeStageParams;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

void processChunked(TriodeStage& stage, const Sample* in, Sample* out, int total, int maxBlock) {
    int offset = 0;
    while (offset < total) {
        const int chunk = std::min(maxBlock, total - offset);
        stage.process(in + offset, out + offset, chunk);
        offset += chunk;
    }
}

// A single-sample constant-input probe: with the stage already settled (setParams + reset()), the
// waveshaper is memoryless, so any sample of a constant-x buffer reads the same value -- this is
// just the cleanest way to read one point off the static curve through the public API.
float settledOutput(TriodeStage& stage, float x) {
    const std::array<Sample, 1> in{x};
    std::array<Sample, 1> out{0.0f};
    stage.process(in.data(), out.data(), 1);
    return out[0];
}

// Peak spectral magnitude within a few bins of `hz`, tolerant of the peak not landing exactly on
// a bin centre. Not shared code with tests/support/SpectralAnalysis.cpp's own findPeakHz -- this
// is a plain magnitude readout (no parabolic refinement needed here, only a comparable relative
// level for the fundamental-vs-harmonics THD ratio below).
double magnitudeNear(const cnpg::test::Spectrum& spectrum, double hz) {
    if (spectrum.fftSize == 0 || spectrum.magnitudeSquared.empty())
        return 0.0;
    const auto lastBin = static_cast<long>(spectrum.magnitudeSquared.size()) - 1;
    const auto center = static_cast<long>(std::lround(spectrum.hzToBin(hz)));
    const long lo = std::max(0L, center - 3);
    const long hi = std::min(lastBin, center + 3);
    if (hi < lo)
        return 0.0;
    double best = 0.0;
    for (long k = lo; k <= hi; ++k)
        best = std::max(best, spectrum.magnitudeSquared[static_cast<std::size_t>(k)]);
    return std::sqrt(best);
}

// THD ratio: RMS of harmonics 2..10 divided by the fundamental's own magnitude. A dimensionless
// ratio, not dB -- the acceptance criterion only needs monotonic ordering across drive settings.
double measureThdRatio(TriodeStage& stage, double sampleRate, int maxBlock, double freqHz, double amplitude,
                       int numSamples) {
    std::vector<Sample> in(static_cast<std::size_t>(numSamples));
    std::vector<Sample> out(static_cast<std::size_t>(numSamples), 0.0f);
    const double w = kTwoPi * freqHz / sampleRate;
    for (int i = 0; i < numSamples; ++i)
        in[static_cast<std::size_t>(i)] = static_cast<Sample>(amplitude * std::sin(w * static_cast<double>(i)));

    processChunked(stage, in.data(), out.data(), numSamples, maxBlock);

    std::vector<double> outD(static_cast<std::size_t>(numSamples));
    for (int i = 0; i < numSamples; ++i)
        outD[static_cast<std::size_t>(i)] = static_cast<double>(out[static_cast<std::size_t>(i)]);

    const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(outD, sampleRate);
    const double fundamental = magnitudeNear(spectrum, freqHz);
    double harmonicSumSq = 0.0;
    for (int h = 2; h <= 10; ++h) {
        const double m = magnitudeNear(spectrum, freqHz * static_cast<double>(h));
        harmonicSumSq += m * m;
    }
    return std::sqrt(harmonicSumSq) / std::max(fundamental, 1e-30);
}

} // namespace

// -------------------------------------------------------------------------------------------
// publishedEcc83()
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage publishedEcc83 matches the cited published Koren ECC83 parameter set exactly",
          "[contract]") {
    // docs/plan.md Task P1.7 acceptance criterion: "publishedEcc83() values match the cited
    // published set exactly (test asserts the literals)". Values per TriodeStage.cpp's own
    // citation (Koren 1996; the 12AX7 parameter set as commonly published in tube-amp SPICE
    // modelling references).
    const KorenTriodeParams p = TriodeStage::publishedEcc83();
    REQUIRE(p.mu == 100.0);
    REQUIRE(p.ex == 1.4);
    REQUIRE(p.kg1 == 1060.0);
    REQUIRE(p.kp == 600.0);
    REQUIRE(p.kvb == 300.0);
    REQUIRE(p.rgi == 2000.0);
}

// -------------------------------------------------------------------------------------------
// bypass
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage bypass passes input through bit-exactly", "[contract]") {
    constexpr int n = 512;
    TriodeStage stage;
    stage.prepare(44100.0, n); // maxBlockSize >= n: the whole buffer in one process() call

    // drive/outputTrimDb deliberately far from default: proves bypass ignores them entirely,
    // not merely "happens to look like a passthrough at the default settings".
    TriodeStageParams p;
    p.bypass = true;
    p.drive = 1.0f;
    p.outputTrimDb = 12.0f;
    stage.setParams(p);
    stage.reset();

    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.1373f) * 0.83f;

    std::vector<Sample> out(n, -999.0f);
    stage.process(in.data(), out.data(), n);

    for (int i = 0; i < n; ++i)
        REQUIRE(out[static_cast<std::size_t>(i)] == in[static_cast<std::size_t>(i)]);
}

// -------------------------------------------------------------------------------------------
// zero input -> DC-removed zero output
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage zero input produces bit-exact zero output", "[contract]") {
    // docs/plan.md Task P1.7 acceptance criterion / section 1.5's own nonlinear-stage DC contract
    // ("TriodeStage must be internally DC-compensated at its quiescent operating point so that
    // zero input produces zero output"). Exercised at default params, at extreme (settled) drive
    // and trim, and mid-ramp (a retarget with no reset() before the process() call) -- see
    // TriodeStage.cpp's waveshapeOne() for why this holds unconditionally: x == 0 forces the
    // table lookup's fractional index to land exactly on the table's forced-zero centre node,
    // regardless of the current drive/outputTrim gain.
    TriodeStage stage;
    stage.prepare(44100.0, 256);

    constexpr int n = 256;
    const std::vector<Sample> zeros(n, 0.0f);

    std::vector<Sample> out(n, 12345.0f);
    stage.process(zeros.data(), out.data(), n);
    for (int i = 0; i < n; ++i)
        REQUIRE(out[static_cast<std::size_t>(i)] == 0.0f);

    TriodeStageParams extreme;
    extreme.drive = 1.0f;
    extreme.outputTrimDb = 18.0f;
    stage.setParams(extreme);
    stage.reset();
    std::fill(out.begin(), out.end(), 12345.0f);
    stage.process(zeros.data(), out.data(), n);
    for (int i = 0; i < n; ++i)
        REQUIRE(out[static_cast<std::size_t>(i)] == 0.0f);

    // Mid-ramp: retarget without reset() so process() takes the ramping branch.
    stage.setParams(TriodeStageParams{});
    std::fill(out.begin(), out.end(), 12345.0f);
    stage.process(zeros.data(), out.data(), n);
    for (int i = 0; i < n; ++i)
        REQUIRE(out[static_cast<std::size_t>(i)] == 0.0f);
}

// -------------------------------------------------------------------------------------------
// THD monotonic across drive
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage THD increases monotonically across drive {0.25, 0.5, 1.0} on a 220 Hz sine",
          "[contract]") {
    constexpr double kSampleRate = 44100.0;
    constexpr int kMaxBlock = 512;
    constexpr double kFreqHz = 220.0;
    constexpr double kAmplitude = 0.12589254117941673; // -18 dBFS, the per-string pickup nominal
    constexpr int kNumSamples = 32768;

    const std::array<float, 3> drives{0.25f, 0.5f, 1.0f};
    std::array<double, 3> thd{};

    for (std::size_t i = 0; i < drives.size(); ++i) {
        TriodeStage stage;
        stage.prepare(kSampleRate, kMaxBlock);
        TriodeStageParams p;
        p.drive = drives[i];
        stage.setParams(p);
        stage.reset();

        thd[i] = measureThdRatio(stage, kSampleRate, kMaxBlock, kFreqHz, kAmplitude, kNumSamples);
        INFO("drive " << drives[i] << " thd " << thd[i]);
    }

    // Real curvature even at the lowest drive setting -- proves this is a genuine nonlinearity
    // measurement, not three flat zeros that trivially satisfy "monotone".
    REQUIRE(thd[0] > 0.0);
    REQUIRE(thd[1] > thd[0]);
    REQUIRE(thd[2] > thd[1]);
}

// -------------------------------------------------------------------------------------------
// deferred hooks: callable, audibly inert
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage setSupplyVoltage/setHeaterVoltage are callable and audibly inert", "[contract]") {
    // docs/plan.md Task P1.7 acceptance criterion: "both deferred hooks callable and audibly
    // inert (output golden-equal before/after calls)".
    constexpr double kSampleRate = 48000.0;
    constexpr int kMaxBlock = 256;
    constexpr int n = 512;

    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.091f) * 0.4f;

    auto render = [&](bool callHooks) {
        TriodeStage stage;
        stage.prepare(kSampleRate, kMaxBlock);
        TriodeStageParams p;
        p.drive = 0.7f;
        p.outputTrimDb = -2.0f;
        stage.setParams(p);
        stage.reset();
        if (callHooks) {
            stage.setSupplyVoltage(300.0f);
            stage.setHeaterVoltage(6.3f);
            stage.setSupplyVoltage(180.0f); // repeated, varied calls
            stage.setHeaterVoltage(0.0f);
        }
        std::vector<Sample> out(n, 0.0f);
        processChunked(stage, in.data(), out.data(), n, kMaxBlock);
        return out;
    };

    const std::vector<Sample> before = render(false);
    const std::vector<Sample> after = render(true);
    REQUIRE(before.size() == after.size());
    for (std::size_t i = 0; i < before.size(); ++i)
        REQUIRE(before[i] == after[i]);
}

TEST_CASE("CONTRACT: TriodeStage deferred hooks called mid-stream do not perturb already-settled output",
          "[contract]") {
    constexpr double kSampleRate = 44100.0;
    constexpr int kMaxBlock = 128;
    constexpr int n = 384;

    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.05f) * 0.6f;

    TriodeStage reference;
    reference.prepare(kSampleRate, kMaxBlock);
    reference.setParams(TriodeStageParams{});
    reference.reset();
    std::vector<Sample> expected(n, 0.0f);
    processChunked(reference, in.data(), expected.data(), n, kMaxBlock);

    TriodeStage withHooks;
    withHooks.prepare(kSampleRate, kMaxBlock);
    withHooks.setParams(TriodeStageParams{});
    withHooks.reset();
    std::vector<Sample> actual(n, 0.0f);
    int offset = 0;
    int block = 0;
    while (offset < n) {
        const int chunk = std::min(kMaxBlock, n - offset);
        if (block == 1) {
            withHooks.setSupplyVoltage(400.0f);
            withHooks.setHeaterVoltage(12.6f);
        }
        withHooks.process(in.data() + offset, actual.data() + offset, chunk);
        offset += chunk;
        ++block;
    }

    for (int i = 0; i < n; ++i)
        REQUIRE(actual[static_cast<std::size_t>(i)] == expected[static_cast<std::size_t>(i)]);
}

// -------------------------------------------------------------------------------------------
// loadTransferTable validation
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage loadTransferTable rejects malformed tables and accepts a well-formed one",
          "[contract]") {
    TriodeStage stage;
    stage.prepare(44100.0, 256);

    std::array<float, 4> vals{0.0f, 0.1f, 0.2f, 0.3f};

    TriodeStage::TransferTableView nullValues{nullptr, 8, -1.0f, 1.0f};
    REQUIRE_FALSE(stage.loadTransferTable(nullValues));

    std::array<float, 1> tooSmallVals{0.0f};
    TriodeStage::TransferTableView tooSmall{tooSmallVals.data(), 1, -1.0f, 1.0f};
    REQUIRE_FALSE(stage.loadTransferTable(tooSmall));

    TriodeStage::TransferTableView equalDomain{vals.data(), static_cast<int>(vals.size()), 1.0f, 1.0f};
    REQUIRE_FALSE(stage.loadTransferTable(equalDomain));

    TriodeStage::TransferTableView invertedDomain{vals.data(), static_cast<int>(vals.size()), 2.0f, -2.0f};
    REQUIRE_FALSE(stage.loadTransferTable(invertedDomain));

    TriodeStage::TransferTableView nanMin{vals.data(), static_cast<int>(vals.size()),
                                          std::numeric_limits<float>::quiet_NaN(), 1.0f};
    REQUIRE_FALSE(stage.loadTransferTable(nanMin));

    TriodeStage::TransferTableView infMax{vals.data(), static_cast<int>(vals.size()), -1.0f,
                                          std::numeric_limits<float>::infinity()};
    REQUIRE_FALSE(stage.loadTransferTable(infMax));

    std::array<float, 4> valsWithNan{0.0f, std::numeric_limits<float>::quiet_NaN(), 0.2f, 0.3f};
    TriodeStage::TransferTableView nonFiniteValue{valsWithNan.data(), static_cast<int>(valsWithNan.size()), -1.0f,
                                                  1.0f};
    REQUIRE_FALSE(stage.loadTransferTable(nonFiniteValue));

    TriodeStage::TransferTableView wellFormed{vals.data(), static_cast<int>(vals.size()), -1.0f, 1.0f};
    REQUIRE(stage.loadTransferTable(wellFormed));

    // Validation-only through P2: accepting a table must not change process()'s behavior (the
    // internally-solved Koren curve is always what process() runs).
    constexpr int n = 64;
    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.2f) * 0.3f;

    TriodeStage reference;
    reference.prepare(44100.0, 256);
    std::vector<Sample> expected(n, 0.0f);
    reference.process(in.data(), expected.data(), n);

    std::vector<Sample> actual(n, 0.0f);
    stage.process(in.data(), actual.data(), n);

    for (int i = 0; i < n; ++i)
        REQUIRE(actual[static_cast<std::size_t>(i)] == expected[static_cast<std::size_t>(i)]);
}

// -------------------------------------------------------------------------------------------
// ramp mechanics
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage drive/outputTrim ramp completes within one process() block", "[contract]") {
    TriodeStage stage;
    stage.prepare(44100.0, 256);
    stage.setParams(TriodeStageParams{});
    stage.reset();

    // 49 is deliberately NOT a power of two: process()'s ramp computes t = (n+1)*invN per sample
    // and relies on the LAST sample being assigned the target directly rather than trusting that
    // formula to land on exactly t == 1.0 (the OutputGain/PickupTap convention's own stated
    // rationale -- "no reliance on floating-point round-trip through double"). For many block
    // sizes count*(1.0/count) does happen to equal 1.0 exactly in IEEE-754 double (verified: every
    // power of two, and plenty of others), which would make that special-case look redundant --
    // but not for every count. 49 is the smallest count where it does NOT round-trip exactly
    // (count=49: 1.0/49.0, then 49.0*that, is measurably below 1.0), so this specific size is what
    // makes the "last sample assigned target" special-case load-bearing rather than a no-op -- see
    // this task's mutation-testing evidence in the P1.7 report.
    constexpr int n = 49;
    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.21f) * 0.3f;

    std::vector<Sample> warm(n, 0.0f);
    stage.process(in.data(), warm.data(), n); // settle at default

    stage.setParams(TriodeStageParams{1.0f, 9.0f, false}); // retarget, no reset(): ramps next call
    std::vector<Sample> ramped(n, 0.0f);
    stage.process(in.data(), ramped.data(), n);

    // This call starts already at the target, since the ramp completed within the previous call.
    std::vector<Sample> settledAgain(n, 0.0f);
    stage.process(in.data(), settledAgain.data(), n);

    TriodeStage direct;
    direct.prepare(44100.0, 256);
    direct.setParams(TriodeStageParams{1.0f, 9.0f, false});
    direct.reset();
    std::vector<Sample> directOut(n, 0.0f);
    direct.process(in.data(), directOut.data(), n);

    for (int i = 0; i < n; ++i)
        REQUIRE(settledAgain[static_cast<std::size_t>(i)] == directOut[static_cast<std::size_t>(i)]);
}

TEST_CASE("CONTRACT: TriodeStage reset collapses a pending drive/outputTrim ramp immediately", "[contract]") {
    TriodeStage stage;
    stage.prepare(44100.0, 256);
    stage.setParams(TriodeStageParams{0.5f, 0.0f, false});
    stage.reset();

    constexpr int n = 128;
    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.31f) * 0.2f;

    stage.setParams(TriodeStageParams{1.0f, 6.0f, false});
    stage.reset(); // must collapse the pending ramp immediately

    TriodeStage fresh;
    fresh.prepare(44100.0, 256);
    fresh.setParams(TriodeStageParams{1.0f, 6.0f, false});
    fresh.reset();

    std::vector<Sample> afterReset(n, 0.0f);
    std::vector<Sample> fromFresh(n, 0.0f);
    stage.process(in.data(), afterReset.data(), n);
    fresh.process(in.data(), fromFresh.data(), n);

    for (int i = 0; i < n; ++i)
        REQUIRE(afterReset[static_cast<std::size_t>(i)] == fromFresh[static_cast<std::size_t>(i)]);
}

// -------------------------------------------------------------------------------------------
// robustness: finite output across parameter/input extremes, including NaN/Inf automation
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage process stays finite across parameter extremes and NaN/Inf-poisoned automation",
          "[contract]") {
    TriodeStage stage;
    stage.prepare(44100.0, 256);

    const std::array<TriodeStageParams, 5> sweep{
        TriodeStageParams{0.0f, -60.0f, false},
        TriodeStageParams{2.0f, 24.0f, false},
        TriodeStageParams{std::numeric_limits<float>::quiet_NaN(), 0.0f, false},
        TriodeStageParams{0.5f, std::numeric_limits<float>::infinity(), false},
        TriodeStageParams{1.0f, 0.0f, true},
    };

    constexpr int n = 256;
    std::vector<Sample> in(n);
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] = std::sin(static_cast<float>(i) * 0.37f) * 4.0f; // deliberately hot

    for (const TriodeStageParams& p : sweep) {
        stage.setParams(p);
        std::vector<Sample> out(n, 0.0f);
        stage.process(in.data(), out.data(), n);
        for (int i = 0; i < n; ++i)
            REQUIRE(std::isfinite(out[static_cast<std::size_t>(i)]));
    }
}

// -------------------------------------------------------------------------------------------
// realtime contract
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage process allocates nothing", "[contract]") {
    constexpr int kMaxBlock = 256;
    TriodeStage stage;
    stage.prepare(48000.0, kMaxBlock);
    stage.setParams(TriodeStageParams{0.7f, 3.0f, false});
    stage.reset();

    std::vector<Sample> in(kMaxBlock, 0.05f);
    std::vector<Sample> out(kMaxBlock, 0.0f);

    cnpg::test::resetAllocationCount();
    for (int b = 0; b < 200; ++b) {
        if (b == 50)
            stage.setParams(TriodeStageParams{1.0f, -3.0f, false}); // exercise the ramp path
        if (b == 100)
            stage.setParams(TriodeStageParams{0.5f, 0.0f, true}); // exercise the bypass path
        stage.process(in.data(), out.data(), kMaxBlock);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

// -------------------------------------------------------------------------------------------
// physical fidelity: sign, asymmetry direction, and which half compresses (review finding 1/2)
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage inverts sign in the small-signal linear region", "[contract]") {
    // A common-cathode stage's own physics already inverts: grid voltage up -> plate current up
    // -> plate voltage down. A small positive grid-referred input must therefore produce a small
    // NEGATIVE output, and a small negative input a small POSITIVE output, at every drive level.
    for (float drive : {0.25f, 0.5f, 1.0f}) {
        TriodeStage stage;
        stage.prepare(44100.0, 256);
        TriodeStageParams p;
        p.drive = drive;
        stage.setParams(p);
        stage.reset();

        const float outPos = settledOutput(stage, 0.05f);
        const float outNeg = settledOutput(stage, -0.05f);
        INFO("drive " << drive << " outPos " << outPos << " outNeg " << outNeg);
        REQUIRE(outPos < 0.0f);
        REQUIRE(outNeg > 0.0f);
    }
}

TEST_CASE("CONTRACT: TriodeStage compresses the positive output half, not the negative one", "[contract]") {
    // docs/plan.md Task P1.7 review finding 1 (PHYSICAL FIDELITY GOVERNS): a real ECC83
    // common-cathode stage compresses the POSITIVE output half (grid-negative swing -> cutoff ->
    // plate voltage rises toward its Vb ceiling), not the negative one. Measured empirically
    // (drive 0.5: ratio ~0.90; drive 1.0: ratio ~0.85 -- see the task report for the full probe
    // table) and pinned here with a safety margin at two drive levels.
    for (float drive : {0.5f, 1.0f}) {
        TriodeStage stage;
        stage.prepare(44100.0, 256);
        TriodeStageParams p;
        p.drive = drive;
        stage.setParams(p);
        stage.reset();

        constexpr float x = 0.5f;
        const float outPos = settledOutput(stage, x);
        const float outNeg = settledOutput(stage, -x);
        const float ratio = std::fabs(outNeg) / std::fabs(outPos);
        INFO("drive " << drive << " outPos " << outPos << " outNeg " << outNeg << " ratio " << ratio);
        REQUIRE(ratio < 0.95f); // the negative-input (positive-output) side is the compressed one
    }
}

TEST_CASE("CONTRACT: TriodeStage's cutoff clip plateau sits on the negative-input side", "[contract]") {
    // Driving the grid deep negative reaches plate-current cutoff (Ip -> 0), a hard physical
    // ceiling on Vp (Vp -> Vb): the output should plateau (stop growing) as input keeps decreasing
    // past that point. The symmetric positive-input excursion has no equivalent hard ceiling in
    // this range (the grid-conduction clamp only slows growth, a linear attenuation -- see
    // buildTransferTable() in TriodeStage.cpp) and should still be visibly growing over the same
    // input range.
    TriodeStage stage;
    stage.prepare(44100.0, 256);
    TriodeStageParams p;
    p.drive = 1.0f;
    stage.setParams(p);
    stage.reset();

    const float negAtHalf = settledOutput(stage, -0.5f);
    const float negAtTwo = settledOutput(stage, -2.0f);
    const float posAtHalf = settledOutput(stage, 0.5f);
    const float posAtTwo = settledOutput(stage, 2.0f);

    INFO("negAtHalf " << negAtHalf << " negAtTwo " << negAtTwo << " posAtHalf " << posAtHalf << " posAtTwo "
                      << posAtTwo);
    REQUIRE(std::fabs(negAtTwo - negAtHalf) < 0.005f); // plateaued: essentially no further growth
    REQUIRE(std::fabs(posAtTwo - posAtHalf) > 0.1f);   // NOT plateaued: still growing substantially
}

// -------------------------------------------------------------------------------------------
// NaN/Inf AUDIO SAMPLES (not parameters -- see the "parameter extremes" case above for that)
// -------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: TriodeStage process stays finite and in-bounds when fed NaN/Inf audio samples", "[contract]") {
    // docs/plan.md Task P1.7 review finding 3: a NaN/Inf SAMPLE reaching the table-index
    // arithmetic unguarded is undefined behavior (std::clamp does not reject NaN; flooring and
    // casting a NaN double to int is UB, observed on this toolchain to yield an out-of-range
    // index) -- TriodeStage is dsp/'s first module that indexes a lookup table directly off an
    // audio sample. waveshapeOne() guards vin with std::isfinite before it reaches interpolate();
    // this pins that a malformed sample produces finite, in-bounds output, and is treated
    // identically to a silent (0.0f) sample at that position.
    TriodeStage stage;
    stage.prepare(44100.0, 256);
    stage.setParams(TriodeStageParams{});
    stage.reset();

    const std::vector<Sample> in{std::numeric_limits<Sample>::quiet_NaN(), std::numeric_limits<Sample>::infinity(),
                                 -std::numeric_limits<Sample>::infinity(), 0.3f,
                                 std::numeric_limits<Sample>::quiet_NaN(), -0.3f,
                                 std::numeric_limits<Sample>::infinity(),  0.0f};
    const int n = static_cast<int>(in.size());
    std::vector<Sample> out(static_cast<std::size_t>(n), 0.0f);
    stage.process(in.data(), out.data(), n);

    for (int i = 0; i < n; ++i) {
        INFO("i " << i << " in " << in[static_cast<std::size_t>(i)]);
        REQUIRE(std::isfinite(out[static_cast<std::size_t>(i)]));
    }

    // A NaN/Inf sample is treated as silence: bit-identical to the same request with those slots
    // replaced by 0.0f.
    const std::vector<Sample> sanitizedIn{0.0f, 0.0f, 0.0f, 0.3f, 0.0f, -0.3f, 0.0f, 0.0f};
    TriodeStage reference;
    reference.prepare(44100.0, 256);
    reference.setParams(TriodeStageParams{});
    reference.reset();
    std::vector<Sample> sanitizedOut(static_cast<std::size_t>(n), 0.0f);
    reference.process(sanitizedIn.data(), sanitizedOut.data(), n);

    for (int i = 0; i < n; ++i)
        REQUIRE(out[static_cast<std::size_t>(i)] == sanitizedOut[static_cast<std::size_t>(i)]);
}

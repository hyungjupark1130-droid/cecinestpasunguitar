#include "cnpg/dsp/PluckExciter.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>
#include <vector>

using cnpg::dsp::PluckExciter;
using cnpg::dsp::PluckExciterParams;

namespace {

constexpr double kSampleRate = 48000.0;
constexpr int kMaxBlockSize = 512;

// Renders every sample of the current burst (until isActive() goes false) into a vector.
template <typename SampleT> std::vector<SampleT> drainBurst(PluckExciter<SampleT>& exciter) {
    std::vector<SampleT> out;
    while (exciter.isActive())
        out.push_back(exciter.renderSample());
    return out;
}

template <typename SampleT> SampleT peakAbs(const std::vector<SampleT>& samples) {
    SampleT peak = SampleT(0);
    for (SampleT s : samples)
        peak = std::max(peak, static_cast<SampleT>(std::fabs(static_cast<double>(s))));
    return peak;
}

} // namespace

// -- latching: trigger() latches position/hardness; a mid-burst setParams() does not move them ---

TEMPLATE_TEST_CASE("PluckExciter: trigger() latches position and hardness for the life of the burst; a mid-burst "
                   "setParams() does not move them",
                   "[contract]", float, double) {
    PluckExciter<TestType> exciter;
    exciter.prepare(kSampleRate, kMaxBlockSize);

    exciter.trigger(0.8f, 0.3f, 0.7f);
    REQUIRE(exciter.latchedPosition01() == 0.3f);
    REQUIRE(exciter.isActive());

    // Render part of the burst, then change PluckExciterParams mid-burst. defaultPosition/
    // defaultHardness here are deliberately different from the triggered position/hardness --
    // trigger()'s own arguments are what get latched, so a mid-burst setParams() must leave
    // latchedPosition01() untouched.
    exciter.renderSample();
    exciter.renderSample();

    PluckExciterParams changed;
    changed.defaultPosition = 0.9f;
    changed.defaultHardness = 0.1f;
    changed.noiseAmount = 0.5f;
    exciter.setParams(changed);

    REQUIRE(exciter.latchedPosition01() == 0.3f); // unmoved by the mid-burst setParams()

    // Drain the rest of the burst; still unmoved once the burst has completed too.
    while (exciter.isActive())
        exciter.renderSample();
    REQUIRE(exciter.latchedPosition01() == 0.3f);
}

// -- determinism with noiseAmount == 0 -------------------------------------------------------

TEMPLATE_TEST_CASE("PluckExciter: with noiseAmount 0, the same trigger() on two freshly prepared instances "
                   "renders bit-identical bursts",
                   "[contract]", float, double) {
    PluckExciter<TestType> a;
    a.prepare(kSampleRate, kMaxBlockSize);
    PluckExciter<TestType> b;
    b.prepare(kSampleRate, kMaxBlockSize);

    PluckExciterParams params;
    params.noiseAmount = 0.0f;
    a.setParams(params);
    b.setParams(params);

    a.trigger(0.65f, 0.42f, 0.37f);
    b.trigger(0.65f, 0.42f, 0.37f);

    const std::vector<TestType> outA = drainBurst(a);
    const std::vector<TestType> outB = drainBurst(b);

    REQUIRE(outA.size() == outB.size());
    for (std::size_t i = 0; i < outA.size(); ++i)
        REQUIRE(outA[i] == outB[i]);

    // Also deterministic across repeated triggers on the very same instance (reset() first, since
    // the burst has already completed above -- this proves determinism doesn't depend on a
    // freshly-prepared instance specifically).
    a.reset();
    a.setParams(params);
    a.trigger(0.65f, 0.42f, 0.37f);
    const std::vector<TestType> outC = drainBurst(a);
    REQUIRE(outC.size() == outA.size());
    for (std::size_t i = 0; i < outC.size(); ++i)
        REQUIRE(outC[i] == outA[i]);
}

// -- silence and isActive() == false after burst end -----------------------------------------

TEMPLATE_TEST_CASE("PluckExciter: renderSample() returns 0 and isActive() is false before the first trigger() and "
                   "again once the burst completes",
                   "[contract]", float, double) {
    PluckExciter<TestType> exciter;
    exciter.prepare(kSampleRate, kMaxBlockSize);

    // Before any trigger(): inactive, silent.
    REQUIRE_FALSE(exciter.isActive());
    REQUIRE(exciter.renderSample() == TestType(0));

    exciter.trigger(1.0f, 0.5f, 0.5f);
    REQUIRE(exciter.isActive());

    while (exciter.isActive())
        exciter.renderSample();

    REQUIRE_FALSE(exciter.isActive());
    // Silent on every call after completion, not just the first.
    for (int i = 0; i < 8; ++i)
        REQUIRE(exciter.renderSample() == TestType(0));
    REQUIRE_FALSE(exciter.isActive());
}

// -- peak amplitude monotone in velocity -----------------------------------------------------

TEMPLATE_TEST_CASE("PluckExciter: burst peak amplitude is monotone non-decreasing in velocity", "[contract]", float,
                   double) {
    PluckExciter<TestType> exciter;
    exciter.prepare(kSampleRate, kMaxBlockSize);

    PluckExciterParams params;
    params.noiseAmount = 0.0f; // isolate the shape's peak from the noise-burst component
    exciter.setParams(params);

    TestType previousPeak = TestType(0);
    for (int step = 0; step <= 10; ++step) {
        const float velocity = static_cast<float>(step) / 10.0f;
        exciter.trigger(velocity, 0.5f, 0.5f);
        const TestType peak = peakAbs(drainBurst(exciter));

        REQUIRE(peak >= previousPeak);
        previousPeak = peak;
    }
    REQUIRE(previousPeak > TestType(0)); // sanity: the sweep actually reached a nonzero peak
}

// -- AC: burst peak at velocity 1.0, hardness 0.5 is within +/-1 dB of -18 dBFS --------------

TEMPLATE_TEST_CASE("PluckExciter: burst peak at velocity 1.0, hardness 0.5 is within +/-1 dB of the -18 dBFS "
                   "per-string nominal",
                   "[contract]", float, double) {
    PluckExciter<TestType> exciter;
    exciter.prepare(kSampleRate, kMaxBlockSize);

    PluckExciterParams params;
    params.noiseAmount = 0.0f;
    exciter.setParams(params);

    exciter.trigger(1.0f, 0.5f, 0.5f);
    const TestType peak = peakAbs(drainBurst(exciter));

    const double peakDb = 20.0 * std::log10(static_cast<double>(peak));
    REQUIRE(peakDb >= -19.0);
    REQUIRE(peakDb <= -17.0);
}

// -- AC: renderSample() measured alloc-free and NaN-free over 10^6 triggers with random params -

TEST_CASE("PluckExciter: renderSample() is alloc-free and NaN-free over 10^6 triggers with random parameters",
          "[contract]") {
    PluckExciter<float> exciter;
    exciter.prepare(kSampleRate, kMaxBlockSize);

    // Fixed seed: reproducible test runs, not a stand-in for the exciter's own (separately fixed)
    // internal noise seed -- this RNG only picks the fuzzed *parameters* below.
    std::mt19937 rng(12345u);
    std::uniform_real_distribution<float> unit01(0.0f, 1.0f);

    bool allFinite = true;

    cnpg::test::resetAllocationCount();
    for (int i = 0; i < 1'000'000; ++i) {
        PluckExciterParams params;
        params.defaultPosition = unit01(rng);
        params.defaultHardness = unit01(rng);
        params.noiseAmount = unit01(rng);
        exciter.setParams(params);

        const float velocity = unit01(rng);
        const float position01 = unit01(rng);
        const float hardness01 = unit01(rng);
        exciter.trigger(velocity, position01, hardness01);

        // Full drain of every one of the 10^6 triggered bursts (bounded to a handful of
        // milliseconds each by design -- see PluckExciter.h), not just a fixed-size prefix.
        while (exciter.isActive())
            allFinite &= std::isfinite(exciter.renderSample());
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
    REQUIRE(allFinite);
}

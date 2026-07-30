#include "cnpg/dsp/OutputGain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using cnpg::dsp::OutputGain;
using cnpg::dsp::OutputGainParams;
using cnpg::dsp::Sample;

namespace {

float dbToLinearRef(float gainDb)
{
    return std::pow(10.0f, gainDb / 20.0f);
}

}  // namespace

TEST_CASE("OutputGain: unity gain passes a buffer bit-exactly after smoothing settles", "[contract]")
{
    OutputGain gain;
    gain.prepare(44100.0, 512);

    // Start away from unity so "settles" is meaningfully exercised, not trivially true
    // from prepare()'s own default state.
    gain.setParams(OutputGainParams{-6.0f});
    std::vector<Sample> warm(64, 1.0f);
    std::vector<Sample> warmOut(64, 0.0f);
    gain.process(warm.data(), warmOut.data(), static_cast<int>(warm.size()));

    // Retarget to unity; one process() call fully settles the ramp by contract.
    gain.setParams(OutputGainParams{0.0f});
    std::vector<Sample> settle(64, 1.0f);
    std::vector<Sample> settleOut(64, 0.0f);
    gain.process(settle.data(), settleOut.data(), static_cast<int>(settle.size()));

    constexpr int numSamples = 256;
    std::vector<Sample> input(numSamples);
    for (int i = 0; i < numSamples; ++i)
        input[static_cast<size_t>(i)] = std::sin(static_cast<float>(i) * 0.1f) * 0.75f;
    std::vector<Sample> output(numSamples, 0.0f);

    gain.process(input.data(), output.data(), numSamples);

    for (int i = 0; i < numSamples; ++i)
        REQUIRE(output[static_cast<size_t>(i)] == input[static_cast<size_t>(i)]);
}

TEST_CASE("OutputGain: a 12 dB step reaches target within one block without exceeding it", "[contract]")
{
    OutputGain gain;
    gain.prepare(44100.0, 512);

    gain.setParams(OutputGainParams{0.0f});
    constexpr int numSamples = 128;
    std::vector<Sample> input(numSamples, 1.0f);
    std::vector<Sample> output(numSamples, 0.0f);
    gain.process(input.data(), output.data(), numSamples);  // settle at 0 dB (unity)

    gain.setParams(OutputGainParams{12.0f});
    gain.process(input.data(), output.data(), numSamples);

    const float targetLinear = dbToLinearRef(12.0f);

    // Never exceeds the target at any point in the ramp.
    for (int i = 0; i < numSamples; ++i)
        REQUIRE(output[static_cast<size_t>(i)] <= targetLinear + 1e-5f);

    // Reaches the target by the last sample of this same block.
    REQUIRE(output[static_cast<size_t>(numSamples - 1)] == Catch::Approx(targetLinear).margin(1e-6));

    // Monotonically non-decreasing across the upward ramp.
    for (int i = 1; i < numSamples; ++i)
        REQUIRE(output[static_cast<size_t>(i)] >= output[static_cast<size_t>(i - 1)] - 1e-6f);
}

TEST_CASE("OutputGain: reset() clears ramp state", "[contract]")
{
    OutputGain gain;
    gain.prepare(44100.0, 512);

    gain.setParams(OutputGainParams{0.0f});
    constexpr int numSamples = 64;
    std::vector<Sample> input(numSamples, 1.0f);
    std::vector<Sample> output(numSamples, 0.0f);
    gain.process(input.data(), output.data(), numSamples);  // settle at 0 dB / unity

    // Retarget without processing: current (unity) and target (+12 dB) now differ.
    gain.setParams(OutputGainParams{12.0f});
    gain.reset();  // must collapse the pending ramp immediately

    const float targetLinear = dbToLinearRef(12.0f);
    gain.process(input.data(), output.data(), numSamples);

    // Every sample of the very next block -- including the first -- is already at the
    // target: no gradual ramp remains after reset().
    for (int i = 0; i < numSamples; ++i)
        REQUIRE(output[static_cast<size_t>(i)] == Catch::Approx(targetLinear).margin(1e-6));
}

TEST_CASE("OutputGain: process handles numSamples from 1 to maxBlockSize", "[contract]")
{
    constexpr int maxBlockSize = 512;
    OutputGain gain;
    gain.prepare(44100.0, maxBlockSize);
    gain.setParams(OutputGainParams{0.0f});

    std::vector<Sample> input(static_cast<size_t>(maxBlockSize), 1.0f);
    std::vector<Sample> output(static_cast<size_t>(maxBlockSize), 0.0f);

    for (int numSamples = 1; numSamples <= maxBlockSize; ++numSamples)
    {
        const float gainDb = (numSamples % 2 == 0) ? 6.0f : -6.0f;
        gain.setParams(OutputGainParams{gainDb});
        gain.process(input.data(), output.data(), numSamples);

        const float expectedLinear = dbToLinearRef(gainDb);

        // The ramp always completes by the last sample of the block, whatever its size.
        REQUIRE(output[static_cast<size_t>(numSamples - 1)] == Catch::Approx(expectedLinear).margin(1e-5));

        for (int i = 0; i < numSamples; ++i)
            REQUIRE(std::isfinite(output[static_cast<size_t>(i)]));
    }
}

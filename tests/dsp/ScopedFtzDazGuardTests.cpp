#include "cnpg/dsp/ScopedFtzDazGuard.h"

#include <catch2/catch_test_macros.hpp>

#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <xmmintrin.h>
#endif

#include <cstdint>

using cnpg::dsp::ScopedFtzDazGuard;

namespace {

std::uint32_t readMxcsr() noexcept { return _mm_getcsr(); }

} // namespace

TEST_CASE("ScopedFtzDazGuard: sets FTZ and DAZ inside scope and restores prior MXCSR on exit", "[contract]") {
    // Start from a known baseline (FTZ/DAZ explicitly cleared) so the assertions below don't
    // depend on whatever ambient MXCSR state the test runner happens to start in.
    const std::uint32_t baseline = readMxcsr() & ~ScopedFtzDazGuard::kFtzDazMask;
    _mm_setcsr(baseline);
    REQUIRE((readMxcsr() & ScopedFtzDazGuard::kFtzDazMask) == 0);

    {
        ScopedFtzDazGuard guard;
        const std::uint32_t inside = readMxcsr();
        REQUIRE((inside & ScopedFtzDazGuard::kFtzDazMask) == ScopedFtzDazGuard::kFtzDazMask);
    }

    REQUIRE(readMxcsr() == baseline);
}

TEST_CASE("ScopedFtzDazGuard: restores a non-default prior MXCSR exactly, not just the FTZ/DAZ bits", "[contract]") {
    // Prior state already has FTZ/DAZ set (e.g. a caller's own outer guard, or another
    // library's ambient FP environment) plus some other bit (rounding mode) flipped from
    // whatever this test runner's default is, so the restore assertion is meaningful rather
    // than a coincidence of the default MXCSR value.
    const std::uint32_t priorRoundingBits = readMxcsr() & 0x6000u; // bits 13-14: rounding control
    const std::uint32_t flippedRounding = priorRoundingBits ^ 0x2000u;
    const std::uint32_t priorMxcsr = (readMxcsr() & ~0x6000u) | flippedRounding | ScopedFtzDazGuard::kFtzDazMask;
    _mm_setcsr(priorMxcsr);
    REQUIRE(readMxcsr() == priorMxcsr);

    {
        ScopedFtzDazGuard guard;
        REQUIRE((readMxcsr() & ScopedFtzDazGuard::kFtzDazMask) == ScopedFtzDazGuard::kFtzDazMask);
    }

    REQUIRE(readMxcsr() == priorMxcsr);
}

TEST_CASE("ScopedFtzDazGuard: a denormal float flushes to zero on multiply inside the guard's scope", "[contract]") {
    const std::uint32_t baseline = readMxcsr() & ~ScopedFtzDazGuard::kFtzDazMask;
    _mm_setcsr(baseline);

    volatile float denormal = 1.0e-40f; // smallest-exponent subnormal region for float32
    REQUIRE(denormal != 0.0f);

    {
        ScopedFtzDazGuard guard;
        volatile float flushed = denormal * 1.0f; // DAZ treats the input as zero before the multiply
        REQUIRE(flushed == 0.0f);
    }

    _mm_setcsr(baseline); // restore before the next TEST_CASE regardless of guard behavior
}

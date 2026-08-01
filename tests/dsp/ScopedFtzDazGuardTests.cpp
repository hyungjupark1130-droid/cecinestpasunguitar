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

// Restores the MXCSR this case was ENTERED with, whatever it did to it in between.
//
// This is not tidiness, it is a correctness fix for the whole binary. MXCSR is process-wide, and
// the case below deliberately flips the ROUNDING-CONTROL bits (13-14) to prove the guard restores
// more than FTZ/DAZ. Without this, that flipped rounding mode survived the case: every later test
// in the same process then ran under a non-default rounding mode, and the next case's "baseline"
// was computed from the already-flipped register, so the damage was invisible and permanent.
//
// The symptom was a binary that gave different answers on different runs of the SAME executable --
// measured at 0, 5 and 1 failures over three consecutive runs, concentrated in exactly the places a
// changed rounding mode breaks: REGRESSION/B float64 golden exactness (atol 1e-7) and the 1e-9
// smoother-settle snaps. CI never saw it, because catch_discover_tests launches one process per
// test case, which is also why the raw single-process binary printed in every task brief's
// verification method was not a valid verification path until this was fixed.
class ScopedMxcsrRestore {
  public:
    ScopedMxcsrRestore() noexcept : entry_(readMxcsr()) {}
    ~ScopedMxcsrRestore() { _mm_setcsr(entry_); }
    ScopedMxcsrRestore(const ScopedMxcsrRestore&) = delete;
    ScopedMxcsrRestore& operator=(const ScopedMxcsrRestore&) = delete;

    std::uint32_t entry() const noexcept { return entry_; }

  private:
    std::uint32_t entry_;
};

} // namespace

TEST_CASE("ScopedFtzDazGuard: sets FTZ and DAZ inside scope and restores prior MXCSR on exit", "[contract]") {
    const ScopedMxcsrRestore restoreOnExit; // this case writes MXCSR; the rest of the binary may not see it
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
    // THE case that made this necessary: it flips the rounding-control bits on a process-wide
    // register and, before this guard existed, walked out still holding them. See ScopedMxcsrRestore.
    const ScopedMxcsrRestore restoreOnExit;
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

TEST_CASE("ScopedFtzDazGuard: leaves the process MXCSR exactly as it found it", "[contract]") {
    // The invariant the two cases above now hold, asserted directly rather than left to the reader
    // to infer from two RAII declarations. A test that writes a process-wide FP control register and
    // does not put it back is not a local mess; it silently re-specifies the arithmetic every later
    // test in the binary runs under.
    const std::uint32_t entry = readMxcsr();
    {
        const ScopedMxcsrRestore restoreOnExit;
        REQUIRE(restoreOnExit.entry() == entry);
        _mm_setcsr((entry ^ 0x2000u) | ScopedFtzDazGuard::kFtzDazMask); // rounding flipped, FTZ/DAZ on
        REQUIRE(readMxcsr() != entry);
    }
    REQUIRE(readMxcsr() == entry);
}

TEST_CASE("ScopedFtzDazGuard: a denormal float flushes to zero on multiply inside the guard's scope", "[contract]") {
    const ScopedMxcsrRestore restoreOnExit;
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

#pragma once

#include <cstdint>

// ScopedFtzDazGuard -- see docs/plan.md section 2 file tree ("(P1) JUCE-free RAII FTZ/DAZ
// MXCSR guard, instantiated first in processBlock (dsp-side; there is no plugin
// DenormalGuard.h)"). Task P1.1. Zero JUCE includes: this is the headless-testable equivalent
// of JUCE's ScopedNoDenormals, living dsp-side so ScopedFtzDazGuardTests.cpp can read MXCSR
// directly without linking JUCE.
//
// x86/x86_64 only (this project's two CI targets -- windows-2022 and ubuntu-latest -- and the
// dev workstation are all x86_64; no ARM build exists through P2).

#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <xmmintrin.h>
#endif

namespace cnpg::dsp {

// RAII guard: on construction, sets the Flush-To-Zero (FTZ, MXCSR bit 15) and
// Denormals-Are-Zero (DAZ, MXCSR bit 6) control bits, leaving every other MXCSR bit (rounding
// mode, exception masks) untouched; on destruction, restores the exact MXCSR value observed at
// construction (not just the FTZ/DAZ bits), so nesting or interleaving with other MXCSR-aware
// code is safe. Not copyable or movable: it captures one construction-time MXCSR snapshot and
// must restore it from the same instance.
class ScopedFtzDazGuard {
  public:
    static constexpr std::uint32_t kFlushToZeroBit = 1u << 15;
    static constexpr std::uint32_t kDenormalsAreZeroBit = 1u << 6;
    static constexpr std::uint32_t kFtzDazMask = kFlushToZeroBit | kDenormalsAreZeroBit;

    ScopedFtzDazGuard() noexcept : previousMxcsr_(_mm_getcsr()) { _mm_setcsr(previousMxcsr_ | kFtzDazMask); }

    ~ScopedFtzDazGuard() noexcept { _mm_setcsr(previousMxcsr_); }

    ScopedFtzDazGuard(const ScopedFtzDazGuard&) = delete;
    ScopedFtzDazGuard& operator=(const ScopedFtzDazGuard&) = delete;
    ScopedFtzDazGuard(ScopedFtzDazGuard&&) = delete;
    ScopedFtzDazGuard& operator=(ScopedFtzDazGuard&&) = delete;

  private:
    std::uint32_t previousMxcsr_;
};

} // namespace cnpg::dsp

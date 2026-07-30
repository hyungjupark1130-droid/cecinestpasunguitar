#pragma once

#include <type_traits>

// CabFilter -- see docs/plan.md section 2.11. Task P1.1 lands CabFilterParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft: cutoff is fixed by
// design, not user-facing); the CabFilter class (bypassable fixed ~5 kHz 2nd-order lowpass)
// arrives in Task P1.9 and is added to this same header. Zero JUCE includes.

namespace cnpg::dsp {

struct CabFilterParams {
    bool bypass = false; // cutoff is fixed (~5 kHz, 2nd order) by design
};

static_assert(std::is_trivially_copyable_v<CabFilterParams>,
              "CabFilterParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

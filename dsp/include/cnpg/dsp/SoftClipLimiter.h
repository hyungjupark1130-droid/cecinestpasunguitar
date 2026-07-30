#pragma once

#include <type_traits>

// SoftClipLimiter -- see docs/plan.md section 2.11. Task P1.1 lands SoftClipLimiterParams only
// (the APVTS-backed P1 parameter surface, already its final shape per the draft: the soft-knee
// shape is fixed, not user-facing); the SoftClipLimiter class arrives in Task P1.9 and is added
// to this same header. Zero JUCE includes.

namespace cnpg::dsp {

struct SoftClipLimiterParams {
    float ceilingDb = -0.3f; // safety ceiling; soft-knee shape fixed
};

static_assert(std::is_trivially_copyable_v<SoftClipLimiterParams>,
              "SoftClipLimiterParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

#pragma once

#include <type_traits>

// PluckExciter -- see docs/plan.md section 2.3. Task P1.1 lands PluckExciterParams only (the
// APVTS-backed P1 parameter surface); the struct's three fields are already its final shape
// per the draft, so this is not a partial preview. The PluckExciter class itself (trigger(),
// renderSample(), latchedPosition01(), isActive()) arrives in Task P1.3 and is added to this
// same header. Zero JUCE includes.

namespace cnpg::dsp {

struct PluckExciterParams {
    float defaultPosition = 0.5f; // 0..1, used when the note event carries no explicit position
    float defaultHardness = 0.5f; // 0..1
    float noiseAmount = 0.0f;     // 0..1, small noise-burst component mixed into the pluck shape
};

static_assert(std::is_trivially_copyable_v<PluckExciterParams>,
              "PluckExciterParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

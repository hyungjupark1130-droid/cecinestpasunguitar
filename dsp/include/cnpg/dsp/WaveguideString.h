#pragma once

#include <type_traits>

// WaveguideString -- see docs/plan.md section 2.4. Task P1.1 lands StringMaterialParams only
// (the APVTS-backed P1 parameter surface, already its final shape per the draft);
// FractionalDelayKind, WaveguideStringParams, and the WaveguideString class itself arrive in
// Task P1.4 and are added to this same header. Zero JUCE includes.

namespace cnpg::dsp {

struct StringMaterialParams {      // material = preset/morph of loss + dispersion
    float lossGainLow = 0.5f;      // loop loss at low frequencies, 0..1
    float lossGainHigh = 0.5f;     // loop loss at high frequencies, 0..1
    float dispersionAmount = 0.0f; // 0..1 scaling of allpass-chain coefficients
};

static_assert(std::is_trivially_copyable_v<StringMaterialParams>,
              "StringMaterialParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

#pragma once

#include <type_traits>

// TriodeStage -- see docs/plan.md section 2.9. Task P1.1 lands TriodeStageParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft); KorenTriodeParams,
// TransferTableView, the TriodeStage class, and the deferred setSupplyVoltage/setHeaterVoltage
// hooks arrive in Task P1.7 and are added to this same header. Zero JUCE includes.

namespace cnpg::dsp {

struct TriodeStageParams {
    float drive = 0.5f;        // input gain into the waveshaper, calibrated against +16 dB summing headroom
    float outputTrimDb = 0.0f; // post-stage trim
    bool bypass = false;       // triode-bypass switch: audition the raw string (P1 monitoring chain)
};

static_assert(std::is_trivially_copyable_v<TriodeStageParams>,
              "TriodeStageParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

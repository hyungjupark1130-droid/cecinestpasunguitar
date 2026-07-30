#pragma once

#include <type_traits>

// PickupTap -- see docs/plan.md section 2.8. Task P1.1 lands PickupTapParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft); the PickupTap
// class (tap-buffer summing + RLC biquad) arrives in Task P1.6 and is added to this same
// header. Zero JUCE includes.

namespace cnpg::dsp {

struct PickupTapParams {
    float resonanceHz = 2500.0f; // RLC resonant frequency
    float q = 2.0f;              // resonance Q (loading)
    float outputGainDb = 0.0f;   // post-sum trim toward the -18 dBFS per-string nominal structure
};

static_assert(std::is_trivially_copyable_v<PickupTapParams>,
              "PickupTapParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

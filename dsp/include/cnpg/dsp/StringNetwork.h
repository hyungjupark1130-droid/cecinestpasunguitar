#pragma once

#include <cstdint>
#include <type_traits>

#include "cnpg/dsp/PluckExciter.h"

// StringNetwork -- see docs/plan.md section 2.7 (RetriggerMode, StringNetworkParams) and
// section 2.4 (StringMaterialParams). Task P1.1 lands:
//   - RetriggerMode: its final shape already.
//   - StringMaterialParams: its final shape already (docs/plan.md section 2.4).
//   - StringNetworkParams: only the fields Task P1.1 wires from APVTS (retriggerMode,
//     pickupPosition01, material, exciter). The full draft also carries
//     pitchBendSemitones (MIDI-pitch-wheel-driven, not an APVTS parameter),
//     damperPosition01, damper (DamperJunctionParams), bridge (BridgeAdmittanceParams), and
//     perString -- Task P1.5 (StringNetwork) extends this struct with those fields rather than
//     replacing it.
// StringTapBuffers, IBridgePort, and the StringNetwork class itself arrive in Task P1.5. Zero
// JUCE includes.

namespace cnpg::dsp {

enum class RetriggerMode : std::uint8_t {
    Physical, // same pitch: pluck over ringing state; new pitch: damper choke -> retune ramp -> re-excite
    Synth     // fast fade, full state reset, instant re-init at new pitch
};

struct StringMaterialParams {      // material = preset/morph of loss + dispersion
    float lossGainLow = 0.5f;      // loop loss at low frequencies, 0..1
    float lossGainHigh = 0.5f;     // loop loss at high frequencies, 0..1
    float dispersionAmount = 0.0f; // 0..1 scaling of allpass-chain coefficients
};

static_assert(std::is_trivially_copyable_v<StringMaterialParams>,
              "StringMaterialParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Task P1.1 subset of the full docs/plan.md section 2.7 StringNetworkParams -- see the file
// comment above for exactly which fields are still missing and which task adds them.
struct StringNetworkParams {
    RetriggerMode retriggerMode = RetriggerMode::Physical;
    float pickupPosition01 = 0.5f; // tap position; continuously modulatable while ringing (P2)
    StringMaterialParams material;
    PluckExciterParams exciter;
};

static_assert(std::is_trivially_copyable_v<StringNetworkParams>,
              "StringNetworkParams must stay trivially copyable for the realtime APVTS snapshot path.");

} // namespace cnpg::dsp

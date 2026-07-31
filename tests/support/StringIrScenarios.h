#pragma once

#include "cnpg/dsp/WaveguideString.h"

#include <array>
#include <string>
#include <vector>

// StringIrScenarios -- the single definition of the `string_ir` golden scenario set (docs/plan.md
// section 4.3), shared by the [regression] tests and by the `cnpg_regen_goldens` regeneration
// entry point so a golden can never be compared against a render produced by different code than
// the one that wrote it.
//
// P1.4 SCOPE NOTE: section 4.3 describes each scenario as a single-string StringNetwork render
// capturing both the tap channel and bridgeOutputBuffer(). StringNetwork lands in Task P1.5 and
// BridgeJunction in P2.4, so the P1.4 baseline captures the tap channel of the isolated
// WaveguideString driven directly by a PluckExciter, with the same note set, duration, excitation
// parameters and tap position. P1.5 re-renders these goldens through StringNetwork under the
// `Regenerate-Goldens:` trailer rule.

namespace cnpg::test {

// docs/plan.md section 4.3: "Scenario notes: MIDI {21, 45, 69, 93, 108}".
inline constexpr std::array<int, 5> kStringIrMidiNotes{21, 45, 69, 93, 108};
inline constexpr std::array<double, 3> kStringIrSampleRates{44100.0, 48000.0, 96000.0};

// docs/plan.md section 4.3 excitation: velocity 0.8, pluckPosition 0.28, hardness 0.5,
// noiseAmount 0.25, tap at pickupPosition01 = 0.87, 3.0 s capture.
inline constexpr float kStringIrVelocity = 0.8f;
inline constexpr float kStringIrPluckPosition = 0.28f;
inline constexpr float kStringIrHardness = 0.5f;
inline constexpr float kStringIrNoiseAmount = 0.25f;
inline constexpr float kStringIrTapPosition = 0.87f;
inline constexpr double kStringIrSeconds = 3.0;
inline constexpr double kStringIrGoldenAtol = 1e-7; // docs/plan.md section 4.3 layer (b)

// Mirrors kNoiseSeed in dsp/src/PluckExciter.cpp; recorded in the sidecar so a future change to
// the exciter's fixed seed shows up as an explicit golden regeneration rather than silently.
inline constexpr unsigned kStringIrNoiseSeed = 2463534242u;

// Layer-(a) octave-band centres, docs/plan.md section 4.3: "per-octave-band T60 from 63 Hz to
// 8 kHz".
inline constexpr std::array<double, 8> kStringIrT60Bands{63.0, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0};

std::string variantName(cnpg::dsp::FractionalDelayKind kind);
std::string scenarioFileName(int midiNote);

// Renders one scenario on the shipping float path and widens to double at capture time, exactly
// as docs/plan.md section 4.3 specifies for the .f64 golden.
std::vector<double> renderStringIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote);

// Layer-(a) reference features extracted from a rendered scenario.
struct StringIrFeatures {
    std::vector<double> partialHz; // partials 1..8; 0.0 for partials above Nyquist
    std::vector<double> bandT60;   // per kStringIrT60Bands; negative when the band is empty
    double attackRmsDbfs = -300.0; // RMS of the first 100 ms
};

StringIrFeatures extractStringIrFeatures(const std::vector<double>& samples, double sampleRate, int midiNote);

} // namespace cnpg::test

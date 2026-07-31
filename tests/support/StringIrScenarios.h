#pragma once

#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include <array>
#include <string>
#include <vector>

// StringIrScenarios -- the single definition of the `string_ir` golden scenario set (docs/plan.md
// section 4.3), shared by the [regression] tests and by the `cnpg_regen_goldens` regeneration
// entry point so a golden can never be compared against a render produced by different code than
// the one that wrote it.
//
// SCOPE NOTE. Section 4.3 describes each scenario as a single-string StringNetwork render
// capturing two signals: the tap channel from StringTapBuffers::channel(0) and
// bridgeOutputBuffer(). Task P1.5 moved the render onto StringNetwork (the P1.4 baseline drove an
// isolated WaveguideString, since StringNetwork did not exist yet), so the tap channel is now
// captured exactly as specified.
//
// The bridge channel is still NOT captured, and will not be until Task P2.4. P1's bridge is the
// trivial rigid termination of IBridgePort.h: it carries no load, so bridgeOutput() is identically
// zero and a "golden" of it would be 3 s of zeros per scenario -- 60 files of nothing, gating
// nothing. "CONTRACT: StringNetwork's P1 bridge output is identically zero" asserts that
// emptiness directly instead, which is the same information at none of the cost. P2.4, which
// gives the bridge a real admittance load, is where the second channel starts carrying signal and
// is also already scheduled to regenerate these goldens for the coupled network.

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
inline constexpr int kStringIrBlockSize = 512;      // block size the scenario is rendered in
inline constexpr double kStringIrGoldenAtol = 1e-7; // docs/plan.md section 4.3 layer (b)

// Mirrors kNoiseSeed in dsp/src/PluckExciter.cpp; recorded in the sidecar so a future change to
// the exciter's fixed seed shows up as an explicit golden regeneration rather than silently.
inline constexpr unsigned kStringIrNoiseSeed = 2463534242u;

// Layer-(a) octave-band centres, docs/plan.md section 4.3: "per-octave-band T60 from 63 Hz to
// 8 kHz".
inline constexpr std::array<double, 8> kStringIrT60Bands{63.0, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0};

std::string variantName(cnpg::dsp::FractionalDelayKind kind);
std::string scenarioFileName(int midiNote);

// Renders one scenario through the shipping single-string StringNetwork topology on the float
// path, widening to double at capture time exactly as docs/plan.md section 4.3 specifies for the
// .f64 golden.
std::vector<double> renderStringIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote);

// Layer-(a) reference features extracted from a rendered scenario.
struct StringIrFeatures {
    std::vector<double> partialHz; // partials 1..8; 0.0 for partials above Nyquist
    std::vector<double> bandT60;   // per kStringIrT60Bands; negative when the band is empty
    double attackRmsDbfs = -300.0; // RMS of the first 100 ms
};

StringIrFeatures extractStringIrFeatures(const std::vector<double>& samples, double sampleRate, int midiNote);

} // namespace cnpg::test

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
// The single-string `string_ir` scenarios still capture the TAP CHANNEL ONLY, and Task P2.4 kept
// it that way deliberately rather than by omission. Through P2.3 the reason was that there was
// nothing to capture -- P1's bridge was the rigid termination, so bridgeOutput() was identically
// zero and a golden of it would have been 3 s of zeros per scenario, 60 files of nothing. P2.4
// gives the bridge a real load, so the signal exists now; what has not changed is that on a SINGLE
// string it carries no information the tap does not. It is one more linear functional of the same
// state, and the tap channel already pins that state sample-exactly to 1e-7 over three seconds.
// Adding it would add ~45 MB of committed binaries per regeneration, for ever, to gate nothing new.
//
// Where the bridge channel IS the information is the COUPLED case, because inter-string coupling
// appears nowhere else, and that is exactly the scenario docs/plan.md section 4.3 adds at this task:
// `chord_ir`, the 6-string open-E-major chord, captured on BOTH channels. See kChordIrNotes below.

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

// docs/plan.md section 4.3: "A sixth scenario captures the full 6-string network playing one open
// E-major chord (tests bridge coupling regression)." Standard tuning, all six strings struck at
// once, at the shipping BridgeAdmittanceParams -- which from Task P2.4 means a LOADED bridge, so
// this render is the only place in the golden set where inter-string coupling appears at all.
inline constexpr std::array<int, 6> kChordIrNotes{40, 45, 50, 55, 59, 64}; // E2 A2 D3 G3 B3 E4
inline constexpr float kChordIrTapPosition = 0.87f;

// Which of the chord scenario's two captured signals a path refers to.
enum class ChordIrChannel { Tap, Bridge };
const char* chordChannelName(ChordIrChannel channel);

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

// Renders the 6-string open-E chord through the shipping StringNetwork topology and returns the
// requested channel: the SUM of the six tap channels (what the pickup would see) or
// bridgeOutputBuffer(). Both are float32 renders widened to double at capture time, exactly as
// section 4.3 specifies for the .f64 golden.
std::vector<double> renderChordIr(cnpg::dsp::FractionalDelayKind kind, double sampleRate, ChordIrChannel channel);

// Layer-(a) features for the chord scenario. PARTIAL TRACKING IS DELIBERATELY OMITTED and the
// reason is that it is not well posed here: "partial k of f0" assumes one f0, and this render has
// six, whose harmonic series interleave (E2's 2nd partial IS E3, its 3rd is near B3) so a ±100 cent
// search around k·f0 cannot say which string it found. What survives the change of scenario --
// per-octave-band T60 and attack RMS -- is what layer (a) is FOR: it is exactly the coupling's own
// signature, since bridge coupling changes decay rates band by band and changes nothing else.
StringIrFeatures extractChordIrFeatures(const std::vector<double>& samples, double sampleRate);

} // namespace cnpg::test

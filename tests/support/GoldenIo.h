#pragma once

#include "support/StringIrScenarios.h"

#include "cnpg/dsp/WaveguideString.h"

#include <filesystem>
#include <string>
#include <vector>

// GoldenIo -- reading/writing the float64 golden IRs and their JSON sidecars (docs/plan.md
// section 4.3 "Golden format" and "Directory scheme"). The sidecar is deliberately FLAT (no
// nested objects) so the tiny reader below stays a few dozen lines instead of pulling a JSON
// dependency into a JUCE-free test binary.

namespace cnpg::test {

// Bumped whenever the sidecar's field set changes. v2 replaced `generatorCommit` -- a commit SHA
// resolved at CMake configure time, which could only ever name the PARENT of the commit that
// carried the goldens -- with `dspSourceSha256`, a content hash of the renderer's own sources
// that is self-consistent inside the golden commit and checkable from any checkout. See
// tests/support/SourceHash.h and tests/data/golden/README.md.
inline constexpr int kGoldenSchemaVersion = 2;

// Mirrors cnpg::CnpgAudioProcessor::kCnpgStateVersion in plugin/src/PluginProcessor.h. cnpg_tests
// links no JUCE, so the value is restated here; docs/plan.md section 4.3 requires it in the
// sidecar so a state-version bump is visible in the golden provenance.
inline constexpr int kDspStateVersion = 1;

struct GoldenSidecar {
    int schemaVersion = kGoldenSchemaVersion;
    std::string dspSourceSha256; // cnpg::test::dspSourceHash() at render time
    std::string generatedUtc;
    int dspStateVersion = kDspStateVersion;
    double sampleRate = 0.0;
    std::string variant;
    int midiNote = 0;
    double excitationVelocity = 0.0;
    double excitationPluckPosition = 0.0;
    double excitationHardness = 0.0;
    double excitationNoiseAmount = 0.0;
    double noiseSeed = 0.0;
    double tapPosition = 0.0;
    double lengthSamples = 0.0;
    double atol = kStringIrGoldenAtol;
    StringIrFeatures features;
};

// tests/data/golden, from the CNPG_GOLDEN_DIR compile definition set in tests/CMakeLists.txt.
std::filesystem::path goldenRoot();
std::filesystem::path goldenDirectory(cnpg::dsp::FractionalDelayKind kind, double sampleRate);
std::filesystem::path goldenF64Path(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote);
std::filesystem::path goldenJsonPath(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote);

// The chord scenario (Task P2.4), under tests/data/golden/chord_ir/<variant>/<rate>/ -- the same
// directory scheme docs/plan.md section 4.3 gives string_ir, with the scenario name changed and the
// captured channel in the file name.
std::filesystem::path chordGoldenF64Path(cnpg::dsp::FractionalDelayKind kind, double sampleRate,
                                         ChordIrChannel channel);
std::filesystem::path chordGoldenJsonPath(cnpg::dsp::FractionalDelayKind kind, double sampleRate,
                                          ChordIrChannel channel);

bool readGoldenF64(const std::filesystem::path& path, std::vector<double>& out);
void writeGoldenF64(const std::filesystem::path& path, const std::vector<double>& samples);

bool readGoldenSidecar(const std::filesystem::path& path, GoldenSidecar& out);
void writeGoldenSidecar(const std::filesystem::path& path, const GoldenSidecar& sidecar);

} // namespace cnpg::test

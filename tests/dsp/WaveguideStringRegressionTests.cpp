#include "support/GoldenIo.h"
#include "support/SpectralAnalysis.h"
#include "support/StringIrScenarios.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

using cnpg::dsp::FractionalDelayKind;
using namespace cnpg::test;

namespace {

constexpr FractionalDelayKind kKinds[2] = {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1};

// docs/plan.md section 4.3 layer (a) tolerances.
constexpr double kPartialCentsTolerance = 2.0;
constexpr double kT60RelativeTolerance = 0.10;
constexpr double kAttackRmsDbTolerance = 1.5;

std::string utcNow() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ", utc.tm_year + 1900, utc.tm_mon + 1,
                  utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    return std::string(buffer);
}

GoldenSidecar makeSidecar(FractionalDelayKind kind, double sampleRate, int midiNote, const std::vector<double>& samples,
                          const StringIrFeatures& features) {
    GoldenSidecar sidecar;
    sidecar.generatorCommit = generatorCommit();
    sidecar.generatedUtc = utcNow();
    sidecar.sampleRate = sampleRate;
    sidecar.variant = variantName(kind);
    sidecar.midiNote = midiNote;
    sidecar.excitationVelocity = static_cast<double>(kStringIrVelocity);
    sidecar.excitationPluckPosition = static_cast<double>(kStringIrPluckPosition);
    sidecar.excitationHardness = static_cast<double>(kStringIrHardness);
    sidecar.excitationNoiseAmount = static_cast<double>(kStringIrNoiseAmount);
    sidecar.noiseSeed = static_cast<double>(kStringIrNoiseSeed);
    sidecar.tapPosition = static_cast<double>(kStringIrTapPosition);
    sidecar.lengthSamples = static_cast<double>(samples.size());
    sidecar.atol = kStringIrGoldenAtol;
    sidecar.features = features;
    return sidecar;
}

const char* missingGoldenHint() {
    return "Golden file missing. Build the cnpg_regen_goldens target to create it, then commit "
           "with a 'Regenerate-Goldens: <reason>' trailer (docs/plan.md section 1.6).";
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Layer (a): feature invariants -- survive an interpolator or filter-topology swap
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGRESSION/A: feature invariants", "[regression]") {
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (int midiNote : kStringIrMidiNotes) {
                const auto sidecarPath = goldenJsonPath(kind, sampleRate, midiNote);
                INFO("sidecar " << sidecarPath.string());
                INFO(missingGoldenHint());
                GoldenSidecar reference;
                REQUIRE(readGoldenSidecar(sidecarPath, reference));

                REQUIRE(reference.schemaVersion == kGoldenSchemaVersion);
                REQUIRE(reference.variant == variantName(kind));
                REQUIRE(reference.midiNote == midiNote);

                const std::vector<double> rendered = renderStringIr(kind, sampleRate, midiNote);
                const StringIrFeatures features = extractStringIrFeatures(rendered, sampleRate, midiNote);

                REQUIRE(features.partialHz.size() == reference.features.partialHz.size());
                for (std::size_t k = 0; k < features.partialHz.size(); ++k) {
                    const double referenceHz = reference.features.partialHz[k];
                    if (referenceHz <= 0.0) {
                        REQUIRE(features.partialHz[k] <= 0.0); // above Nyquist in both
                        continue;
                    }
                    const double cents = centsBetween(features.partialHz[k], referenceHz);
                    INFO("partial " << (k + 1) << " reference " << referenceHz << " Hz measured "
                                    << features.partialHz[k] << " Hz (" << cents << " cents)");
                    REQUIRE(std::fabs(cents) <= kPartialCentsTolerance);
                }

                REQUIRE(features.bandT60.size() == reference.features.bandT60.size());
                for (std::size_t b = 0; b < features.bandT60.size(); ++b) {
                    const double referenceT60 = reference.features.bandT60[b];
                    // A negative reference means the band carries too little energy at this note
                    // for a valid Schroeder fit; the fresh render must agree that it does.
                    if (referenceT60 <= 0.0) {
                        INFO("band " << kStringIrT60Bands[b] << " Hz expected empty, measured " << features.bandT60[b]);
                        REQUIRE(features.bandT60[b] <= 0.0);
                        continue;
                    }
                    INFO("band " << kStringIrT60Bands[b] << " Hz reference T60 " << referenceT60 << " s measured "
                                 << features.bandT60[b] << " s");
                    REQUIRE(features.bandT60[b] > 0.0);
                    REQUIRE(std::fabs(features.bandT60[b] - referenceT60) <= kT60RelativeTolerance * referenceT60);
                }

                INFO("attack RMS reference " << reference.features.attackRmsDbfs << " dBFS measured "
                                             << features.attackRmsDbfs << " dBFS");
                REQUIRE(std::fabs(features.attackRmsDbfs - reference.features.attackRmsDbfs) <= kAttackRmsDbTolerance);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Layer (b): float64 golden exactness -- toolchain-pinned, Windows/MSVC only (section 4.9)
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGRESSION/B: float64 golden exactness", "[regression]") {
#if !defined(_MSC_VER)
    SKIP("docs/plan.md section 4.9: layer (b) goldens are pinned to the MSVC toolchain and run in "
         "the Windows job only; the ubuntu portability job runs layer (a).");
#else
    double worstDiff = 0.0;
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (int midiNote : kStringIrMidiNotes) {
                const auto path = goldenF64Path(kind, sampleRate, midiNote);
                INFO("golden " << path.string());
                INFO(missingGoldenHint());
                std::vector<double> golden;
                REQUIRE(readGoldenF64(path, golden));

                const std::vector<double> rendered = renderStringIr(kind, sampleRate, midiNote);
                REQUIRE(rendered.size() == golden.size());

                double localWorst = 0.0;
                std::size_t worstIndex = 0;
                for (std::size_t i = 0; i < rendered.size(); ++i) {
                    const double diff = std::fabs(rendered[i] - golden[i]);
                    if (diff > localWorst) {
                        localWorst = diff;
                        worstIndex = i;
                    }
                }
                INFO("worst |diff| " << localWorst << " at sample " << worstIndex);
                REQUIRE(localWorst <= kStringIrGoldenAtol);
                worstDiff = std::max(worstDiff, localWorst);
            }
        }
    }
    std::cout << "[regression] layer (b): worst |golden diff| " << worstDiff << " (atol " << kStringIrGoldenAtol
              << ")\n";
#endif
}

// ---------------------------------------------------------------------------------------------
// Regeneration entry point -- hidden ("[.]"), so CTest never discovers or runs it. Driven by the
// cnpg_regen_goldens CMake target (docs/plan.md section 1.6).
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGRESSION/REGEN: rewrite string_ir goldens", "[.][regen]") {
    std::cout << "Regenerating string_ir goldens under " << goldenRoot().string() << "\n";
    std::cout << "commit " << generatorCommit() << "\n";

    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (int midiNote : kStringIrMidiNotes) {
                const std::vector<double> rendered = renderStringIr(kind, sampleRate, midiNote);
                const StringIrFeatures features = extractStringIrFeatures(rendered, sampleRate, midiNote);

                // Drift report against whatever is on disk right now (docs/plan.md section 1.6:
                // "prints a drift report (max abs sample diff and per-feature deltas versus the
                // previous goldens)").
                const auto f64Path = goldenF64Path(kind, sampleRate, midiNote);
                std::vector<double> previous;
                double maxDiff = -1.0;
                if (readGoldenF64(f64Path, previous) && previous.size() == rendered.size()) {
                    maxDiff = 0.0;
                    for (std::size_t i = 0; i < rendered.size(); ++i)
                        maxDiff = std::max(maxDiff, std::fabs(rendered[i] - previous[i]));
                }

                GoldenSidecar oldSidecar;
                const bool hadSidecar = readGoldenSidecar(goldenJsonPath(kind, sampleRate, midiNote), oldSidecar);

                char line[256];
                std::snprintf(line, sizeof(line), "%-9s %6.0f Hz MIDI %3d  maxSampleDiff %-12s f1 %+8.4f cents",
                              variantName(kind).c_str(), sampleRate, midiNote,
                              (maxDiff < 0.0) ? "(new)" : std::to_string(maxDiff).c_str(),
                              (hadSidecar && !oldSidecar.features.partialHz.empty() &&
                               oldSidecar.features.partialHz[0] > 0.0 && features.partialHz[0] > 0.0)
                                  ? centsBetween(features.partialHz[0], oldSidecar.features.partialHz[0])
                                  : 0.0);
                std::cout << line << "\n";

                writeGoldenF64(f64Path, rendered);
                writeGoldenSidecar(goldenJsonPath(kind, sampleRate, midiNote),
                                   makeSidecar(kind, sampleRate, midiNote, rendered, features));
            }
        }
    }
    std::cout << "Done. Commit with a 'Regenerate-Goldens: <reason>' trailer (docs/plan.md section 1.6).\n";
    SUCCEED();
}

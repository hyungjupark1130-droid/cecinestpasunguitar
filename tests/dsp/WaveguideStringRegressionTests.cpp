#include "support/GoldenIo.h"
#include "support/SourceHash.h"
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
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ", utc.tm_year + 1900, utc.tm_mon + 1,
                  utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    return std::string(buffer);
}

GoldenSidecar makeSidecar(FractionalDelayKind kind, double sampleRate, int midiNote, const std::vector<double>& samples,
                          const StringIrFeatures& features) {
    GoldenSidecar sidecar;
    sidecar.dspSourceSha256 = dspSourceHash();
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
    const std::vector<double> bridge = renderStringIrBridge(kind, sampleRate, midiNote);
    sidecar.bridgeFeatures = extractChordIrFeatures(bridge, sampleRate); // band T60 + attack RMS only
    sidecar.bridgeChecksum = checksumF64(bridge);
    return sidecar;
}

// The sidecar stores these as decimal text at the stream's default precision, and they originate
// as `float` knob values widened to double, so an exact == is the wrong comparison for the
// provenance checks -- 0.87f widens to 0.87000000476837158 but reads back as 0.87.
bool sameProvenance(double a, double b) { return std::fabs(a - b) <= 1e-6; }

const char* missingGoldenHint() {
    return "Golden file missing. Build the cnpg_regen_goldens target to create it, then commit "
           "with a 'Regenerate-Goldens: <reason>' trailer (docs/plan.md section 1.6).";
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Layer (a): feature invariants -- survive an interpolator or filter-topology swap
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGRESSION/A: feature invariants", "[regression]") {
    std::string provenanceHash;
    int bridgeBandsGated = 0;
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (int midiNote : kStringIrMidiNotes) {
                const auto sidecarPath = goldenJsonPath(kind, sampleRate, midiNote);
                INFO("sidecar " << sidecarPath.string());
                INFO(missingGoldenHint());
                GoldenSidecar reference;
                REQUIRE(readGoldenSidecar(sidecarPath, reference));

                // Provenance, not just features: a sidecar generated for a different rate, tap
                // position, excitation or exciter seed is not a reference for THIS render, and
                // layer (b) -- which would otherwise notice -- is MSVC-only.
                REQUIRE(reference.schemaVersion == kGoldenSchemaVersion);
                REQUIRE(reference.dspStateVersion == kDspStateVersion);
                // Provenance consistency: every sidecar in the set must name the SAME dsp/ source
                // tree, so a half-finished regeneration -- some files rewritten, some left from an
                // older tree -- fails here instead of shipping a set that no single checkout can
                // account for. Whether that tree is the CURRENT one is a different question, and
                // deliberately not asserted: see REGRESSION/P below.
                REQUIRE(reference.dspSourceSha256.size() == 64);
                if (provenanceHash.empty())
                    provenanceHash = reference.dspSourceSha256;
                REQUIRE(reference.dspSourceSha256 == provenanceHash);
                REQUIRE(reference.variant == variantName(kind));
                REQUIRE(reference.midiNote == midiNote);
                REQUIRE(reference.sampleRate == sampleRate);
                REQUIRE(sameProvenance(reference.tapPosition, static_cast<double>(kStringIrTapPosition)));
                REQUIRE(sameProvenance(reference.excitationVelocity, static_cast<double>(kStringIrVelocity)));
                REQUIRE(sameProvenance(reference.excitationPluckPosition, static_cast<double>(kStringIrPluckPosition)));
                REQUIRE(sameProvenance(reference.excitationHardness, static_cast<double>(kStringIrHardness)));
                REQUIRE(sameProvenance(reference.excitationNoiseAmount, static_cast<double>(kStringIrNoiseAmount)));
                REQUIRE(reference.noiseSeed == static_cast<double>(kStringIrNoiseSeed));
                REQUIRE(reference.atol == kStringIrGoldenAtol);

                const std::vector<double> rendered = renderStringIr(kind, sampleRate, midiNote);
                REQUIRE(reference.lengthSamples == static_cast<double>(rendered.size()));
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

                // THE BRIDGE CHANNEL (schema v3, Task P2.4). docs/plan.md section 4.3 captures two
                // signals per scenario; on a single string the second one is recorded as layer-(a)
                // features plus a checksum instead of a second .f64 -- see GoldenSidecar for the
                // reasoning and the plan-file amendment. What matters is that it is no longer
                // recorded as NOTHING: through P2.3 it was identically zero, and P2.4 made it a
                // signal that nothing was gating.
                REQUIRE(reference.bridgeChecksum.size() == 16);
                const std::vector<double> bridge = renderStringIrBridge(kind, sampleRate, midiNote);
                const StringIrFeatures bridgeFeatures = extractChordIrFeatures(bridge, sampleRate);

                double bridgePeak = 0.0;
                for (double sample : bridge)
                    bridgePeak = std::max(bridgePeak, std::fabs(sample));
                // Non-vacuous: a bridge channel of silence would match a reference of silence on
                // every line below, which is exactly the state this task took it out of.
                INFO("bridge peak " << bridgePeak);
                REQUIRE(bridgePeak > 0.0);

                int gatedBridgeBands = 0;
                for (std::size_t b = 0; b < bridgeFeatures.bandT60.size(); ++b) {
                    const double referenceT60 = reference.bridgeFeatures.bandT60[b];
                    if (referenceT60 <= 0.0) {
                        REQUIRE(bridgeFeatures.bandT60[b] <= 0.0);
                        continue;
                    }
                    INFO("bridge band " << kStringIrT60Bands[b] << " Hz reference T60 " << referenceT60
                                        << " s measured " << bridgeFeatures.bandT60[b] << " s");
                    REQUIRE(bridgeFeatures.bandT60[b] > 0.0);
                    REQUIRE(std::fabs(bridgeFeatures.bandT60[b] - referenceT60) <=
                            kT60RelativeTolerance * referenceT60);
                    ++gatedBridgeBands;
                }
                INFO("bridge attack RMS reference " << reference.bridgeFeatures.attackRmsDbfs << " dBFS measured "
                                                    << bridgeFeatures.attackRmsDbfs << " dBFS");
                REQUIRE(std::fabs(bridgeFeatures.attackRmsDbfs - reference.bridgeFeatures.attackRmsDbfs) <=
                        kAttackRmsDbTolerance);
                bridgeBandsGated += gatedBridgeBands;

                // SAMPLE-EXACTNESS, the part the features cannot give: the checksum moves if any
                // sample of the bridge render moves. Toolchain-sensitive in exactly the way layer
                // (b) is, so it runs where layer (b) runs.
#if defined(_MSC_VER)
                INFO("bridge checksum reference " << reference.bridgeChecksum << " measured " << checksumF64(bridge));
                REQUIRE(checksumF64(bridge) == reference.bridgeChecksum);
#endif
            }
        }
    }
    std::cout << "[regression] string_ir bridge channel: " << bridgeBandsGated
              << " band-T60 comparisons gated across 30 scenario/variant/rate combinations, plus attack RMS and a "
                 "sample-exact checksum (schema v3)\n";
    REQUIRE(bridgeBandsGated > 30);
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
// The coupled scenario (Task P2.4): docs/plan.md section 4.3's sixth scenario
// ---------------------------------------------------------------------------------------------

namespace {

constexpr ChordIrChannel kChordChannels[2] = {ChordIrChannel::Tap, ChordIrChannel::Bridge};

GoldenSidecar makeChordSidecar(FractionalDelayKind kind, double sampleRate, const std::vector<double>& samples,
                               const StringIrFeatures& features) {
    GoldenSidecar sidecar;
    sidecar.dspSourceSha256 = dspSourceHash();
    sidecar.generatedUtc = utcNow();
    sidecar.sampleRate = sampleRate;
    sidecar.variant = variantName(kind);
    sidecar.midiNote = kChordIrNotes.front(); // the chord's root, so the field is not meaningless
    sidecar.excitationVelocity = static_cast<double>(kStringIrVelocity);
    sidecar.excitationPluckPosition = static_cast<double>(kStringIrPluckPosition);
    sidecar.excitationHardness = static_cast<double>(kStringIrHardness);
    sidecar.excitationNoiseAmount = static_cast<double>(kStringIrNoiseAmount);
    sidecar.noiseSeed = static_cast<double>(kStringIrNoiseSeed);
    sidecar.tapPosition = static_cast<double>(kChordIrTapPosition);
    sidecar.lengthSamples = static_cast<double>(samples.size());
    sidecar.atol = kStringIrGoldenAtol;
    sidecar.features = features;
    return sidecar;
}

} // namespace

TEST_CASE("REGRESSION/A: coupled chord feature invariants", "[regression]") {
    // Layer (a) for the coupled scenario. Partial tracking is omitted (see
    // tests/support/StringIrScenarios.h for why it is not well posed across six fundamentals);
    // what IS gated is exactly the coupling's own signature -- per-octave-band T60, which is what
    // a bridge load changes, and nothing else in the render can.
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (ChordIrChannel channel : kChordChannels) {
                const auto sidecarPath = chordGoldenJsonPath(kind, sampleRate, channel);
                INFO("sidecar " << sidecarPath.string());
                INFO(missingGoldenHint());
                GoldenSidecar reference;
                REQUIRE(readGoldenSidecar(sidecarPath, reference));
                REQUIRE(reference.schemaVersion == kGoldenSchemaVersion);
                REQUIRE(reference.dspStateVersion == kDspStateVersion);
                REQUIRE(reference.variant == variantName(kind));
                REQUIRE(reference.sampleRate == sampleRate);

                const std::vector<double> rendered = renderChordIr(kind, sampleRate, channel);
                REQUIRE(reference.lengthSamples == static_cast<double>(rendered.size()));
                const StringIrFeatures features = extractChordIrFeatures(rendered, sampleRate);

                REQUIRE(features.bandT60.size() == reference.features.bandT60.size());
                int gatedBands = 0;
                for (std::size_t b = 0; b < features.bandT60.size(); ++b) {
                    const double referenceT60 = reference.features.bandT60[b];
                    if (referenceT60 <= 0.0) {
                        INFO("band " << kStringIrT60Bands[b] << " Hz expected empty, measured " << features.bandT60[b]);
                        REQUIRE(features.bandT60[b] <= 0.0);
                        continue;
                    }
                    INFO(chordChannelName(channel) << " band " << kStringIrT60Bands[b] << " Hz reference T60 "
                                                   << referenceT60 << " s measured " << features.bandT60[b] << " s");
                    REQUIRE(features.bandT60[b] > 0.0);
                    REQUIRE(std::fabs(features.bandT60[b] - referenceT60) <= kT60RelativeTolerance * referenceT60);
                    ++gatedBands;
                }
                // Non-vacuous: a scenario whose every band was empty would pass every line above.
                REQUIRE(gatedBands >= 3);

                INFO("attack RMS reference " << reference.features.attackRmsDbfs << " dBFS measured "
                                             << features.attackRmsDbfs << " dBFS");
                REQUIRE(std::fabs(features.attackRmsDbfs - reference.features.attackRmsDbfs) <= kAttackRmsDbTolerance);
            }
        }
    }
}

TEST_CASE("REGRESSION/B: coupled chord float64 golden exactness", "[regression]") {
#if !defined(_MSC_VER)
    SKIP("docs/plan.md section 4.9: layer (b) goldens are pinned to the MSVC toolchain and run in "
         "the Windows job only; the ubuntu portability job runs layer (a).");
#else
    double worstDiff = 0.0;
    double bridgePeak = 0.0;
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (ChordIrChannel channel : kChordChannels) {
                const auto path = chordGoldenF64Path(kind, sampleRate, channel);
                INFO("golden " << path.string());
                INFO(missingGoldenHint());
                std::vector<double> golden;
                REQUIRE(readGoldenF64(path, golden));

                const std::vector<double> rendered = renderChordIr(kind, sampleRate, channel);
                REQUIRE(rendered.size() == golden.size());

                double localWorst = 0.0;
                double peak = 0.0;
                for (std::size_t i = 0; i < rendered.size(); ++i) {
                    localWorst = std::max(localWorst, std::fabs(rendered[i] - golden[i]));
                    peak = std::max(peak, std::fabs(golden[i]));
                }
                INFO("worst |diff| " << localWorst);
                REQUIRE(localWorst <= kStringIrGoldenAtol);
                // Non-vacuous: the coupled scenario's whole point is that the bridge channel is a
                // signal now. A golden of silence would compare equal to a render of silence.
                REQUIRE(peak > 0.0);
                if (channel == ChordIrChannel::Bridge)
                    bridgePeak = std::max(bridgePeak, peak);
                worstDiff = std::max(worstDiff, localWorst);
            }
        }
    }
    std::cout << "[regression] chord_ir layer (b): worst |golden diff| " << worstDiff << " (atol "
              << kStringIrGoldenAtol << "); bridge-channel peak " << bridgePeak << "\n";
    REQUIRE(bridgePeak > 0.0);
#endif
}

// ---------------------------------------------------------------------------------------------
// Provenance -- the sidecar field that says WHICH code wrote these bytes
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGRESSION/P: golden provenance is verifiable", "[regression]") {
    // Schema v2 replaced `generatorCommit` with `dspSourceSha256` because a commit SHA resolved at
    // CMake configure time is structurally incapable of naming the commit that CONTAINS the
    // goldens -- the goldens are written before that commit exists, so the field always named its
    // parent and no checkout could ever check it. A content hash of dsp/include + dsp/src has no
    // such problem: the bytes it covers ship in the same commit as the goldens.
    //
    // This case guards the mechanism, not the freshness of the goldens. Freshness is what layers
    // (a) and (b) above are -- they re-render with the current code on every CI run. Asserting
    // "recorded hash == current hash" here would instead force a 60-file golden commit for any
    // edit anywhere under dsp/, including ones this scenario cannot see (a comment, or a module
    // like OutputGain that the render never touches), which would make the Regenerate-Goldens
    // trailer a lie in the common case.
    const std::string current = dspSourceHash();
    INFO("CNPG_SOURCE_DIR must point at a checkout containing dsp/include and dsp/src");
    REQUIRE(current.size() == 64);
    REQUIRE(current.find_first_not_of("0123456789abcdef") == std::string::npos);
    REQUIRE(dspSourceHash() == current); // deterministic: same tree, same digest, every call

    // The digest really is SHA-256, checkable against any other implementation: FIPS 180-4's own
    // published vectors, plus the empty string. Without this the sidecars would record 64 hex
    // characters that only this file could reproduce, which is not provenance.
    REQUIRE(sha256Hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    REQUIRE(sha256Hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    REQUIRE(sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    GoldenSidecar reference;
    REQUIRE(readGoldenSidecar(goldenJsonPath(FractionalDelayKind::Lagrange3, 48000.0, 69), reference));
    std::cout << "[regression] golden provenance: sidecars record dsp source sha256 " << reference.dspSourceSha256
              << "\n[regression] this checkout hashes to             " << current
              << (reference.dspSourceSha256 == current ? "  (match)"
                                                       : "  (differs -- dsp/ has moved since the "
                                                         "last regeneration; layers (a)/(b) above "
                                                         "are what decide whether that matters)")
              << "\n";
    SUCCEED();
}

// ---------------------------------------------------------------------------------------------
// Regeneration entry point -- hidden ("[.]"), so CTest never discovers or runs it. Driven by the
// cnpg_regen_goldens CMake target (docs/plan.md section 1.6).
//
// Deliberately NOT named "REGRESSION/...": Catch2 only skips hidden cases when the spec carries no
// filter at all, so a name glob someone would plausibly type -- cnpg_tests "REGRESSION*" -- would
// otherwise match this case and silently overwrite all 60 committed goldens as a side effect of
// running the tests that gate them. Tag filters and CTest are safe either way.
// ---------------------------------------------------------------------------------------------

TEST_CASE("REGEN: rewrite string_ir goldens", "[.][regen]") {
    std::cout << "Regenerating string_ir goldens under " << goldenRoot().string() << "\n";
    std::cout << "dsp source sha256 " << dspSourceHash() << "\n";

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

                char line[512];
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

    // ...and the coupled chord scenario (Task P2.4), both captured channels.
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kStringIrSampleRates) {
            for (ChordIrChannel channel : kChordChannels) {
                const std::vector<double> rendered = renderChordIr(kind, sampleRate, channel);
                const StringIrFeatures features = extractChordIrFeatures(rendered, sampleRate);

                const auto f64Path = chordGoldenF64Path(kind, sampleRate, channel);
                std::vector<double> previous;
                double maxDiff = -1.0;
                if (readGoldenF64(f64Path, previous) && previous.size() == rendered.size()) {
                    maxDiff = 0.0;
                    for (std::size_t i = 0; i < rendered.size(); ++i)
                        maxDiff = std::max(maxDiff, std::fabs(rendered[i] - previous[i]));
                }
                char line[512];
                std::snprintf(line, sizeof(line), "%-9s %6.0f Hz chord/%-6s maxSampleDiff %-12s attackRMS %+8.2f dBFS",
                              variantName(kind).c_str(), sampleRate, chordChannelName(channel),
                              (maxDiff < 0.0) ? "(new)" : std::to_string(maxDiff).c_str(), features.attackRmsDbfs);
                std::cout << line << "\n";

                writeGoldenF64(f64Path, rendered);
                writeGoldenSidecar(chordGoldenJsonPath(kind, sampleRate, channel),
                                   makeChordSidecar(kind, sampleRate, rendered, features));
            }
        }
    }
    std::cout << "Done. Commit with a 'Regenerate-Goldens: <reason>' trailer (docs/plan.md section 1.6).\n";
    SUCCEED();
}

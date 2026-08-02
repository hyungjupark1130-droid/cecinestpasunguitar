// cnpg_calibrate -- the VERIFICATION HARNESS for bridge tuning compensation across the
// (MIDI note x sample rate x bridge-parameter) grid. Task P2.7; docs/plan.md section P2.7 as amended
// by docs/decisions/0007-bridge-tuning-compensation.md. Links cnpg_dsp only -- headless, JUCE-free,
// no Python, no LTspice -- so it builds and runs in the ubuntu CI job exactly as cnpg_render does.
//
// ---------------------------------------------------------------------------------------------
// WHAT THIS TOOL IS NOW, AND WHAT IT DELIBERATELY IS NOT
// ---------------------------------------------------------------------------------------------
// The plan's original P2.7 built this tool as the MECHANISM: it measured the shipped implementation
// note by note and emitted a frozen per-MIDI-note cents table, compiled into cnpg_dsp and loaded at
// prepare(). ADR 0007 withdrew that mechanism, because the residual is not a per-note constant --
// it is a function of `couplingStrength`, `bridgeResonanceHz` and `bridgeDamping`, all live APVTS
// parameters, and it REVERSES SIGN across the resonance sweep. A one-dimensional note-indexed table
// structurally cannot represent that, and no amount of regeneration fixes it.
//
// So there is no table any more, and therefore NO GENERATED HEADER and no CSV-to-header agreement to
// check: the compensation is computed in closed form from the port's own admittance
// (dsp/include/cnpg/dsp/BridgeTuning.h). What the tool retains is the job ADR 0007 D6 keeps for it --
// VERIFYING that compensation across the grid, and emitting the result as a human-diffable artifact
// so a regression is visible in a code review rather than only in a test log.
//
// The residual-trim half of D6 ("a residual trim is used ONLY IF the analytic solution leaves a small
// systematic error") is NOT implemented, and that is a measurement rather than an omission: the worst
// residual inside the provisional Normal range is 0.770 cents against a +/-2 cent criterion, and at
// the shipping admittance it is 0.060. There is no systematic error left for a trim to remove, and
// shipping a correction table for a 0.06-cent residual would reintroduce exactly the frozen artifact
// ADR 0007 removed.
//
// ---------------------------------------------------------------------------------------------
// DETERMINISM
// ---------------------------------------------------------------------------------------------
// Every number here comes from a fixed render of a fixed configuration through a fixed estimator.
// There is no clock, no RNG, no filesystem iteration order and no floating-point summation whose
// order depends on anything but the loop bounds, so a second run on an unchanged tree writes
// byte-identical CSVs. `--verify-determinism` renders each grid point twice and compares, so the
// claim is checked rather than asserted.

#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/BridgeTuning.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/SpectralAnalysis.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// docs/plan.md section 4.5's render recipe, so this tool measures the same instrument the [tuning]
// gate does and the two sets of numbers are directly comparable.
constexpr double kRenderSeconds = 7.0;
constexpr double kDiscardSeconds = 0.5;
constexpr float kVelocity = 0.8f;
constexpr float kPluckPosition = 0.28f;
constexpr float kHardness = 0.5f;
constexpr float kTapPosition = 0.87f;
constexpr float kSweepLossKnob = 1.0f;

// The provisional Normal range of ADR 0007 D7, taken from the module that DECLARES it
// (dsp/include/cnpg/dsp/BridgeJunction.h) rather than restated here. The tool reports over a WIDER
// grid than that -- the Extended range is where the interesting failures are -- and only GATES inside
// it.
constexpr float kNormalCouplingMax = cnpg::dsp::kBridgeNormalCouplingMax;
constexpr float kNormalResonanceMaxHz = cnpg::dsp::kBridgeNormalResonanceMaxHz;
constexpr float kNormalDampingMin = cnpg::dsp::kBridgeNormalDampingMin;
constexpr float kNormalDampingMax = cnpg::dsp::kBridgeNormalDampingMax;
constexpr int kGateLowMidi = cnpg::dsp::kMinMidiNote;
constexpr int kGateHighMidi = 96;

std::size_t analysisLengthFor(double sampleRate) {
    return (sampleRate > 60000.0) ? (std::size_t{1} << 19) : (std::size_t{1} << 18);
}

struct GridPoint {
    float coupling;
    float resonanceHz;
    float damping;
};

double measureCentsError(double sampleRate, int midiNote, const GridPoint& point, double& compensationOut,
                         bool& convergedOut) {
    cnpg::dsp::StringNetworkParams params;
    params.pickupPosition01 = kTapPosition;
    params.stringMaterial.lossGainLow = kSweepLossKnob;
    params.stringMaterial.lossGainHigh = kSweepLossKnob;
    params.exciter.noiseAmount = 0.0f;
    params.bridge.couplingStrength = point.coupling;
    params.bridge.resonanceHz = point.resonanceHz;
    params.bridge.damping = point.damping;
    for (auto& perString : params.perString)
        perString.restMidiNote = static_cast<std::uint8_t>(midiNote);

    cnpg::dsp::StringNetwork<float> network;
    network.prepare(sampleRate, 512, cnpg::dsp::FractionalDelayKind::Lagrange3);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset();

    cnpg::dsp::NoteEvent noteOn{};
    noteOn.type = cnpg::dsp::NoteEventType::NoteOn;
    noteOn.sampleOffset = 0;
    noteOn.stringIndex = 0;
    noteOn.channel = 0;
    noteOn.midiNote = static_cast<std::uint8_t>(midiNote);
    noteOn.velocity = kVelocity;
    noteOn.pluckPosition = kPluckPosition;
    noteOn.hardness = kHardness;
    cnpg::dsp::BlockEventQueue events;
    events.push(noteOn);

    const auto total = static_cast<std::size_t>(kRenderSeconds * sampleRate);
    const auto discard = static_cast<std::size_t>(kDiscardSeconds * sampleRate);
    const std::size_t wanted = analysisLengthFor(sampleRate);

    std::vector<double> tap;
    tap.reserve(wanted);
    std::size_t rendered = 0;
    while (rendered < total && tap.size() < wanted) {
        network.process(events, 512);
        const float* channel = network.tapBuffers().channel(0, 0);
        for (int n = 0; n < 512 && tap.size() < wanted; ++n, ++rendered)
            if (rendered >= discard)
                tap.push_back(static_cast<double>(channel[n]));
    }

    compensationOut = network.bridgeCompensationSamples(0);
    convergedOut = network.bridgeTuningConverged(0);

    const double target = cnpg::test::midiNoteToHz(midiNote);
    const double measured = cnpg::test::findPeakHz(cnpg::test::computeSpectrum(tap, sampleRate, wanted), target,
                                                   cnpg::test::kTuningSearchCents);
    // No usable peak inside the estimator's search window is not "an error of 80 cents"; it is "the
    // fundamental is not where it was solved for". Emitted as the boundary + 1 so the two are
    // distinguishable in the CSV, and it fails the gate either way.
    return (measured > 0.0) ? cnpg::test::centsBetween(measured, target) : (cnpg::test::kTuningSearchCents + 1.0);
}

bool insideNormalRange(const GridPoint& point) {
    return point.coupling <= kNormalCouplingMax && point.resonanceHz <= kNormalResonanceMaxHz &&
           point.damping >= kNormalDampingMin && point.damping <= kNormalDampingMax;
}

void printUsage(std::FILE* stream) {
    std::fprintf(stream, "cnpg_calibrate -- bridge tuning verification across the note x admittance grid\n"
                         "\n"
                         "  --rates <a,b,c>          sample rates in Hz (default 44100,48000,96000)\n"
                         "  --out <dir>              output directory (default dsp/data/calibration)\n"
                         "  --verify-determinism     render every grid point twice and compare\n"
                         "  --help\n"
                         "\n"
                         "Exit code is nonzero if any point INSIDE the provisional Normal range\n"
                         "(ADR 0007 D7) exceeds the +/-2 cent criterion, or if determinism fails.\n");
}

} // namespace

int main(int argc, char** argv) {
    std::vector<double> rates{44100.0, 48000.0, 96000.0};
    fs::path outDir = "dsp/data/calibration";
    bool verifyDeterminism = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(stdout);
            return 0;
        }
        if (arg == "--verify-determinism") {
            verifyDeterminism = true;
            continue;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "cnpg_calibrate: %s needs a value\n", arg.c_str());
            return 1;
        }
        const std::string value = argv[++i];
        if (arg == "--rates") {
            rates.clear();
            std::size_t begin = 0;
            while (begin <= value.size()) {
                const std::size_t comma = value.find(',', begin);
                const std::string token = value.substr(begin, comma - begin);
                if (!token.empty())
                    rates.push_back(std::strtod(token.c_str(), nullptr));
                if (comma == std::string::npos)
                    break;
                begin = comma + 1;
            }
        } else if (arg == "--out") {
            outDir = value;
        } else {
            std::fprintf(stderr, "cnpg_calibrate: unknown option %s\n", arg.c_str());
            printUsage(stderr);
            return 1;
        }
    }
    if (rates.empty()) {
        std::fprintf(stderr, "cnpg_calibrate: --rates listed no usable value\n");
        return 1;
    }

    // The grid. Deliberately WIDER than the Normal range on every axis, because a verification
    // harness that only looked inside the guaranteed region could never show where the region ends.
    const float couplings[] = {0.0f, 0.2f, 0.35f, 0.6f, 1.0f};
    const float resonances[] = {20.0f, 110.0f, 180.0f, 330.0f, 660.0f, 2000.0f};
    const float dampings[] = {0.05f, 0.15f, 0.5f, 1.0f, 4.0f};
    // The note axis is subsampled to keep the artifact human-diffable and the run affordable; the
    // EXHAUSTIVE note sweep is the [tuning] grid gate's job (tests/dsp/TuningAccuracyTests.cpp
    // asserts every note from 21 to 96 at every rate). What this adds is the parameter axes.
    const int notes[] = {21, 33, 40, 45, 52, 57, 64, 69, 76, 81, 88, 93, 96};

    std::error_code ec;
    fs::create_directories(outDir, ec);

    double worstInsideNormal = 0.0;
    int worstNote = 0;
    double worstRate = 0.0;
    int failures = 0;
    long long points = 0;

    for (double rate : rates) {
        char nameBuffer[64];
        std::snprintf(nameBuffer, sizeof(nameBuffer), "tuning_verification_%.0f.csv", rate);
        const fs::path csvPath = outDir / nameBuffer;
        std::string csv;
        char line[512];
        std::snprintf(
            line, sizeof(line),
            "# cnpg bridge tuning verification v1, rate=%.0f, gate=+/-2.000 cents over MIDI %d-%d inside the "
            "provisional Normal range (ADR 0007 D7: coupling <= %.2f, resonance <= %.0f Hz, damping %.2f-%.2f)\n",
            rate, kGateLowMidi, kGateHighMidi, static_cast<double>(kNormalCouplingMax),
            static_cast<double>(kNormalResonanceMaxHz), static_cast<double>(kNormalDampingMin),
            static_cast<double>(kNormalDampingMax));
        csv += line;
        csv += "coupling,resonanceHz,damping,midiNote,targetHz,centsError,compensationSamples,converged,normalRange\n";

        for (float coupling : couplings)
            for (float resonance : resonances)
                for (float damping : dampings) {
                    const GridPoint point{coupling, resonance, damping};
                    for (int note : notes) {
                        double compensation = 0.0;
                        bool converged = true;
                        const double cents = measureCentsError(rate, note, point, compensation, converged);
                        ++points;

                        if (verifyDeterminism) {
                            double compensationAgain = 0.0;
                            bool convergedAgain = true;
                            const double again =
                                measureCentsError(rate, note, point, compensationAgain, convergedAgain);
                            if (again != cents || compensationAgain != compensation) {
                                std::fprintf(stderr,
                                             "cnpg_calibrate: DETERMINISM FAILED at rate %.0f note %d "
                                             "(c=%.2f res=%.0f z=%.2f): %.9f vs %.9f cents\n",
                                             rate, note, static_cast<double>(coupling), static_cast<double>(resonance),
                                             static_cast<double>(damping), cents, again);
                                return 1;
                            }
                        }

                        const bool inside = insideNormalRange(point) && note >= kGateLowMidi && note <= kGateHighMidi;
                        if (inside) {
                            if (std::fabs(cents) > worstInsideNormal) {
                                worstInsideNormal = std::fabs(cents);
                                worstNote = note;
                                worstRate = rate;
                            }
                            if (std::fabs(cents) > cnpg::dsp::kBridgeTuningGateCents) {
                                ++failures;
                                std::fprintf(stderr,
                                             "cnpg_calibrate: GATE FAILED -- %.3f cents at MIDI %d / %.0f Hz "
                                             "(c=%.2f res=%.0f z=%.2f), inside the Normal range\n",
                                             cents, note, rate, static_cast<double>(coupling),
                                             static_cast<double>(resonance), static_cast<double>(damping));
                            }
                        }

                        std::snprintf(line, sizeof(line), "%.3f,%.1f,%.3f,%d,%.4f,%.6f,%.6f,%d,%d\n",
                                      static_cast<double>(coupling), static_cast<double>(resonance),
                                      static_cast<double>(damping), note, cnpg::test::midiNoteToHz(note), cents,
                                      compensation, converged ? 1 : 0, inside ? 1 : 0);
                        csv += line;
                    }
                }

        // Written in one shot, in binary, with '\n' line endings written literally: the artifact is
        // committed and diffed, so it must be byte-identical on every platform that regenerates it.
        std::ofstream out(csvPath, std::ios::binary | std::ios::trunc);
        if (!out) {
            std::fprintf(stderr, "cnpg_calibrate: cannot write %s\n", csvPath.string().c_str());
            return 1;
        }
        out.write(csv.data(), static_cast<std::streamsize>(csv.size()));
        out.close();
        std::printf("  %s (%zu bytes)\n", csvPath.string().c_str(), csv.size());
        std::fflush(stdout);
    }

    std::printf("cnpg_calibrate: %lld grid points, worst |error| inside the provisional Normal range %.4f cents "
                "at MIDI %d / %.0f Hz (gate +/-%.1f), %d failure(s)%s\n",
                points, worstInsideNormal, worstNote, worstRate, cnpg::dsp::kBridgeTuningGateCents, failures,
                verifyDeterminism ? ", determinism verified" : "");
    return failures == 0 ? 0 : 1;
}

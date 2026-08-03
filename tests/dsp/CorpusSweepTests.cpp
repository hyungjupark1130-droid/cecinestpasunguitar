// Task P2.8 -- the automated sweep over the whole corpus, and the denormal state-inspection case
// extended to six coupled strings with dampers (docs/plan.md Task P2.8).
//
// Three cases:
//
//   CORPUS SWEEP: every phrase renders clean at 44.1/48/96 kHz   [contract]
//   CORPUS SWEEP: the click statistic catches a discontinuity    [contract]   (the non-vacuity arm)
//   DENORMAL: six coupled strings with dampers leave no subnormal state  [denormal][contract]
//
// The first drives the REAL cnpg_render executable, exactly as tests/dsp/RenderTests.cpp does and
// for the same reason: a sweep over test-local output would gate a signal path the listening pass
// never hears. This file's relationship to RenderTests.cpp is a widening, not a duplication --
// RenderTests renders ONE rate and checks NaN/subnormal/ceiling; this renders all THREE and adds
// the P2.1 click statistic, which is the acceptance criterion Task P2.8 carries and P1.11 did not.
//
// -----------------------------------------------------------------------------------------------
// THE CLICK STATISTIC OVER A WHOLE RENDER, AND THE FLOOR THAT MAKES IT MEAN ANYTHING
// -----------------------------------------------------------------------------------------------
//
// tests/support/ClickMetric.h's reading (b) -- clickExcessDb -- needs a REFERENCE render without
// the state change, and a corpus render has none: the render IS the device under test. What it does
// have that stands alone is ClickMeasurement::peakStepToLevel, the second companion: max over 10 ms
// windows of (window peak |dx|) / (window peak |x|). That is a pure shape number, and it is the one
// the header says survives a level change of any size because the yardstick is re-measured every
// 10 ms alongside the numerator.
//
// Applied to a whole corpus render it needs one addition, and the addition is load-bearing rather
// than a tidy-up. MEASURED, over all 8 phrases at all 3 rates with no floor: every phrase reads
// between 2.02 and 3.53, and a LEVEL-PLACED HARD CUT reads 1.00. The statistic ranks a clean render
// WORSE than a discontinuity, and it is not close. The cause is arithmetic and complete: a corpus
// phrase contains long spans that have decayed to a handful of LSBs, an alternating sequence of
// LSBs has |dx| up to 2*|x|, and those windows own the maximum. The number was measuring the
// quietest part of the file.
//
// So a window only counts if its own peak |x| is within kWindowFloorDb of the RENDER's peak. Below
// that, a window cannot contain an audible click -- it cannot contain an audible anything. With the
// floor at -60 dB the same 24 renders read 0.5557 at worst and the same hard cut still reads 1.0000.
//
// THE CRITERION IS peakStepToLevel < 1.0, AND IT IS DERIVED RATHER THAN FITTED. A window whose
// peak |dx| reaches its peak |x| is a window in which the signal traversed its own largest value
// between two adjacent samples. A hard cut placed on the loudest sample reads EXACTLY 1.0 by
// construction -- its |dx| IS the sample it lands on -- and a sign flip there reads 2.0. The bound
// is therefore the smallest one that both defects fail, and the corpus's own worst reading --
// 0.5557, on 01_chromatic_singles at 44.1 kHz -- sits 5.10 dB inside it.
//
// WHAT THE BOUND DOES NOT CLAIM. It is content-dependent: a sine at f reads 2*sin(pi*f/fs), which
// reaches 1.0 at f = fs/6 (8 kHz at 48 kHz). A render genuinely bright enough to put that much
// energy above fs/6 would fail this gate without containing a click. That is acceptable HERE and
// only here, because the corpus is a fixed, append-only set of inputs through a chain that ends in
// CabFilter -- the content is not free to change under the gate without a corpus version bump. The
// corpus's worst reading corresponds to about 0.09*fs, i.e. 4.2 kHz at 48 kHz.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"

#include "support/ClickMetric.h"
#include "support/P1Chain.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

using namespace cnpg::dsp;

namespace {

namespace fs = std::filesystem;

// A window is judged only if its own peak |x| is within this of the render's peak. See the header:
// without it the statistic measures the decayed tail's LSBs and ranks a clean render below a hard
// cut.
constexpr double kWindowFloorDb = -60.0;

// The criterion. See the header for the derivation; a level-placed hard cut reads exactly 1.0.
constexpr double kStepToLevelLimit = 1.0;

const char* const kRates[] = {"44100", "48000", "96000"};

fs::path renderExe() { return fs::path(CNPG_RENDER_EXE); }
fs::path corpusDir() { return fs::path(CNPG_CORPUS_DIR); }
fs::path scratchDir() { return renderExe().parent_path() / "corpus-sweep"; }

std::string quoted(const fs::path& path) { return "\"" + path.string() + "\""; }

int runRender(const std::string& arguments, const fs::path& logPath) {
    std::string command = quoted(renderExe()) + " " + arguments + " > " + quoted(logPath) + " 2>&1";
#if defined(_WIN32)
    // cmd.exe strips the first and last quote of the command line before parsing it; wrapping the
    // whole thing in one more pair is the documented fix (tests/dsp/RenderTests.cpp says the same).
    command = "\"" + command + "\"";
#endif
    return std::system(command.c_str());
}

std::string readTextFile(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        return "(no log captured at " + path.string() + ")";
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

// A deliberately independent RIFF reader, for the reason tests/dsp/RenderTests.cpp states: a writer
// that emitted a self-consistently wrong file still fails here.
struct WavContents {
    double sampleRate = 0.0;
    std::vector<float> samples;
};

std::uint32_t readLe32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}

WavContents readWav(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    REQUIRE(file.is_open());
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    REQUIRE(bytes.size() > 12);
    REQUIRE(std::string(bytes.begin(), bytes.begin() + 4) == "RIFF");
    REQUIRE(std::string(bytes.begin() + 8, bytes.begin() + 12) == "WAVE");

    WavContents out;
    std::size_t offset = 12;
    while (offset + 8 <= bytes.size()) {
        const std::string id(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                             bytes.begin() + static_cast<std::ptrdiff_t>(offset + 4));
        const std::uint32_t size = readLe32(bytes, offset + 4);
        const std::size_t body = offset + 8;
        if (id == "fmt " && size >= 16) {
            out.sampleRate = static_cast<double>(readLe32(bytes, body + 4));
        } else if (id == "data") {
            // Bounded by what is actually in the file, not by what the chunk header claims: a
            // truncated WAV is a diagnostic, not an over-read.
            REQUIRE(body + size <= bytes.size());
            out.samples.resize(size / sizeof(float));
            if (!out.samples.empty())
                std::memcpy(out.samples.data(), bytes.data() + body, out.samples.size() * sizeof(float));
        }
        offset = body + size + (size & 1u);
    }
    REQUIRE(out.sampleRate > 0.0);
    REQUIRE(!out.samples.empty());
    return out;
}

struct SweepReading {
    double stepToLevel = 0.0;
    std::size_t worstWindowStart = 0;
    long long nonFinite = 0;
    long long subnormal = 0;
    double peak = 0.0;
    std::size_t windowsJudged = 0;
};

// The whole-render statistic. Built out of cnpg::test::measureClick per 10 ms window, so the
// per-window numbers are the shared definition's and this file only adds the floor and the maximum.
SweepReading sweepRender(const std::vector<float>& samples, double sampleRate) {
    SweepReading out;
    for (const float sample : samples) {
        if (!std::isfinite(sample)) {
            ++out.nonFinite;
            continue;
        }
        if (std::fpclassify(sample) == FP_SUBNORMAL)
            ++out.subnormal;
        out.peak = std::max(out.peak, std::fabs(static_cast<double>(sample)));
    }

    const double floorLevel = out.peak * std::pow(10.0, kWindowFloorDb / 20.0);
    const auto window = static_cast<std::size_t>(std::llround(cnpg::test::kClickMetricWindowSeconds * sampleRate));
    if (window < 2)
        return out;

    for (std::size_t begin = 0; begin + window <= samples.size(); begin += window) {
        const cnpg::test::ClickMeasurement measurement =
            cnpg::test::measureClick(samples, sampleRate, begin, begin + window);
        if (measurement.peakAbsSample < floorLevel || measurement.peakAbsSample <= 0.0)
            continue;
        ++out.windowsJudged;
        if (measurement.peakStepToLevel > out.stepToLevel) {
            out.stepToLevel = measurement.peakStepToLevel;
            out.worstWindowStart = begin;
        }
    }
    return out;
}

// How many phrases the manifest carries. Counted from the file rather than hardcoded, so a phrase
// added without a render is a FAILURE here instead of a file nobody looked for.
int manifestPhraseCount() {
    const std::string text = readTextFile(corpusDir() / "corpus.json");
    int count = 0;
    for (std::size_t at = text.find("\"file\""); at != std::string::npos; at = text.find("\"file\"", at + 1))
        ++count;
    return count;
}

} // namespace

TEST_CASE("CORPUS SWEEP: every phrase renders clean at 44.1/48/96 kHz", "[contract]") {
    const fs::path outputDir = scratchDir();
    std::error_code ec;
    fs::remove_all(outputDir, ec);
    fs::create_directories(outputDir, ec);
    REQUIRE(fs::is_directory(outputDir));

    const fs::path logPath = outputDir / "render.log";
    const std::string arguments = "--corpus " + quoted(corpusDir()) + " --out " + quoted(outputDir) +
                                  " --rates 44100,48000,96000 --verify-determinism";
    const int exitCode = runRender(arguments, logPath);
    INFO("cnpg_render log:\n" << readTextFile(logPath));
    REQUIRE(exitCode == 0);

    const int expectedPhrases = manifestPhraseCount();
    REQUIRE(expectedPhrases >= 8); // the P1 four plus Task P2.8's four; never fewer

    double worstStepToLevel = 0.0;
    std::string worstName;

    for (const char* rate : kRates) {
        const fs::path rateDir = outputDir / rate;
        INFO("rate directory " << rateDir.string());
        REQUIRE(fs::is_directory(rateDir));

        std::vector<fs::path> wavs;
        for (const fs::directory_entry& entry : fs::directory_iterator(rateDir))
            if (entry.path().extension() == ".wav")
                wavs.push_back(entry.path());
        std::sort(wavs.begin(), wavs.end());
        INFO("rate " << rate << " produced " << wavs.size() << " WAV(s)");
        REQUIRE(static_cast<int>(wavs.size()) == expectedPhrases);

        for (const fs::path& wav : wavs) {
            const WavContents contents = readWav(wav);
            INFO("render " << wav.filename().string() << " at " << rate << " Hz");
            REQUIRE(contents.sampleRate == std::atof(rate));

            const SweepReading reading = sweepRender(contents.samples, contents.sampleRate);
            INFO("nonFinite " << reading.nonFinite << ", subnormal " << reading.subnormal << ", peak " << reading.peak
                              << ", windows judged " << reading.windowsJudged << ", peakStepToLevel "
                              << reading.stepToLevel << " at sample " << reading.worstWindowStart);

            REQUIRE(reading.nonFinite == 0);
            REQUIRE(reading.subnormal == 0);
            // Not a silent render: the statistic below is meaningless on silence, and a corpus
            // phrase that produced nothing would satisfy every other assertion here.
            REQUIRE(reading.peak > 0.001);
            REQUIRE(reading.windowsJudged > 10);
            // Below the SoftClipLimiter ceiling. Re-measured off the FILE rather than taken from
            // the tool's own log, so a tool that mis-reported its own peak still fails here.
            REQUIRE(20.0 * std::log10(reading.peak) < static_cast<double>(SoftClipLimiterParams{}.ceilingDb) + 0.01);
            REQUIRE(reading.stepToLevel < kStepToLevelLimit);

            if (reading.stepToLevel > worstStepToLevel) {
                worstStepToLevel = reading.stepToLevel;
                worstName = wav.filename().string() + " @ " + rate;
            }
        }
    }

    std::cout << "[contract] corpus sweep: " << expectedPhrases << " phrase(s) x 3 rates, worst peakStepToLevel "
              << worstStepToLevel << " (" << worstName << ") against a limit of " << kStepToLevelLimit << "\n";
}

// -------------------------------------------------------------------------------------------
// THE NON-VACUITY ARM (project ruling 7: every gate is shown to fail on the defect it exists to
// catch). Two constructed discontinuities, both LEVEL-PLACED per tests/support/ClickMetric.h's
// placement rule -- on the render's own loudest sample, never at a round index.
// -------------------------------------------------------------------------------------------
TEST_CASE("CORPUS SWEEP: the click statistic catches a discontinuity", "[contract]") {
    const fs::path outputDir = scratchDir() / "vacuity";
    std::error_code ec;
    fs::remove_all(outputDir, ec);
    fs::create_directories(outputDir, ec);

    const fs::path logPath = outputDir / "render.log";
    const std::string arguments = "--midi " + quoted(corpusDir() / "02_open_chords.mid") + " --out " +
                                  quoted(outputDir / "vacuity.wav") + " --samplerate 48000";
    const int exitCode = runRender(arguments, logPath);
    INFO("cnpg_render log:\n" << readTextFile(logPath));
    REQUIRE(exitCode == 0);

    const WavContents clean = readWav(outputDir / "vacuity.wav");
    const SweepReading cleanReading = sweepRender(clean.samples, clean.sampleRate);
    INFO("clean peakStepToLevel " << cleanReading.stepToLevel);
    REQUIRE(cleanReading.stepToLevel < kStepToLevelLimit);

    // The placement: the loudest sample of the render under test, read off that render. Not a round
    // index, not a block boundary -- ClickMetric.h's rule binds controls as hard as perturbations,
    // and a blindly placed cut lands near a zero crossing and passes the gate it exists to fail.
    //
    // One index is excluded, and excluding it is part of the same rule rather than an exception to
    // it: the LAST sample of a 10 ms window. A discontinuity placed there falls ACROSS the window
    // boundary, so the two windows that straddle it each see one side of the step and neither
    // measures it -- which would make the reading a property of the analysis grid's phase, exactly
    // what the placement rule exists to remove.
    const auto window =
        static_cast<std::size_t>(std::llround(cnpg::test::kClickMetricWindowSeconds * clean.sampleRate));
    REQUIRE(window >= 2);
    std::size_t loudest = 1;
    for (std::size_t i = 1; i + 1 < clean.samples.size(); ++i)
        if ((i + 1) % window != 0 && std::fabs(clean.samples[i]) > std::fabs(clean.samples[loudest]))
            loudest = i;
    REQUIRE(std::fabs(clean.samples[loudest]) > 0.001f);

    // Zeroed from the sample AFTER the loudest one, so the step is exactly |x[loudest]| and the
    // window's own peak |x| is exactly |x[loudest]| -- the ratio is 1.0 by construction, which is
    // the derivation the criterion is written from and not a number read off this corpus.
    std::vector<float> hardCut = clean.samples;
    std::fill(hardCut.begin() + static_cast<std::ptrdiff_t>(loudest + 1), hardCut.end(), 0.0f);
    const SweepReading cutReading = sweepRender(hardCut, clean.sampleRate);

    std::vector<float> signFlip = clean.samples;
    signFlip[loudest] = -signFlip[loudest];
    const SweepReading flipReading = sweepRender(signFlip, clean.sampleRate);

    std::cout << "[contract] corpus-sweep vacuity: clean " << cleanReading.stepToLevel << ", level-placed hard cut "
              << cutReading.stepToLevel << ", level-placed sign flip " << flipReading.stepToLevel << ", limit "
              << kStepToLevelLimit << "\n";

    INFO("hard cut " << cutReading.stepToLevel << ", sign flip " << flipReading.stepToLevel);
    REQUIRE(cutReading.stepToLevel >= kStepToLevelLimit);
    REQUIRE(flipReading.stepToLevel >= kStepToLevelLimit);
    // ...and the separation is real rather than marginal: the defects must clear the clean render's
    // own reading by more than the P2 click criterion's own 3 dB.
    REQUIRE(20.0 * std::log10(cutReading.stepToLevel / cleanReading.stepToLevel) > cnpg::test::kClickMetricToleranceDb);
}

// -------------------------------------------------------------------------------------------
// The denormal state-inspection case, extended to six coupled strings WITH DAMPERS and the full
// monitoring chain (docs/plan.md Task P2.8; tests/dsp/DenormalTests.cpp's P1 SCOPE note names this
// case as its successor). The timing-ratio case stays local-only per section 4.6 and is NOT
// duplicated here.
// -------------------------------------------------------------------------------------------
namespace {

constexpr double kDenormalSampleRate = 44100.0;
constexpr int kDenormalBlockSize = 128;
constexpr double kDenormalTailSeconds = 60.0;
constexpr int kDenormalStrings = 6;
constexpr int kDenormalChordNotes[kDenormalStrings] = {40, 45, 50, 55, 59, 64};

// WHY THIS SCANS THE WHOLE TAIL AND SNAPSHOTS THE DEEPEST LIVE STATE, rather than inspecting the
// final block. Measured on the first cut of this case: after 60 s the network's energyEstimate() is
// exactly 0 and every damper's engagement has snapped back to 0 -- StringNetwork's silence watchdog
// has cleared every string, so "the final block carries no subnormal state" is satisfied by a state
// that is identically zero. That is the vacuity this project keeps finding. The interesting window
// is the DECAY, so every sample of the whole tail is scanned and the state is snapshotted at the
// last block that still produced a non-zero output sample -- the deepest state that exists.
struct DamperedTail {
    std::vector<float> deepestTaps;
    std::vector<float> deepestBridge;
    std::vector<float> deepestOutput;
    std::vector<float> deepestDamperState; // engagement, loss depth and position of every string
    double energyAtDeepest = 0.0;
    double firstSecondPeak = 0.0;
    double maxDamperEngagement = 0.0;
    int lastNonZeroBlock = -1;
    int totalBlocks = 0;
    bool allFinite = true;
    long long subnormalOutputSamples = 0;
    long long subnormalTapSamples = 0;
    long long subnormalBridgeSamples = 0;
};

// `guarded` selects whether the plugin's own ScopedFtzDazGuard is engaged around each block. It is
// the non-vacuity control: the case asserts no subnormal state WITH the guard, and that subnormals
// really do appear WITHOUT it, so "no subnormals" is the guard working rather than a render that
// could never have produced one.
DamperedTail renderDamperedTail(bool guarded, double tailSeconds) {
    cnpg::test::P1ChainParams params = cnpg::test::makeDefaultP1ChainParams();
    params.numStrings = kDenormalStrings;
    params.network.pickupPosition01 = 0.87f;

    cnpg::test::P1Chain chain;
    chain.prepare(kDenormalSampleRate, kDenormalBlockSize, kDenormalStrings, Oversampler::kDefaultFactor);
    chain.reset();

    BlockEventQueue events;
    for (int s = 0; s < kDenormalStrings; ++s) {
        NoteEvent event{};
        event.type = NoteEventType::NoteOn;
        event.sampleOffset = s * 8;
        event.stringIndex = static_cast<std::uint8_t>(s);
        event.channel = 0;
        event.midiNote = static_cast<std::uint8_t>(kDenormalChordNotes[s]);
        event.velocity = 0.9f;
        event.pluckPosition = 0.28f;
        event.hardness = 0.5f;
        events.push(event);
    }

    const auto totalBlocks = static_cast<int>(tailSeconds * kDenormalSampleRate / kDenormalBlockSize);
    const int firstSecondBlocks = static_cast<int>(kDenormalSampleRate / kDenormalBlockSize);
    // The note-offs, i.e. what makes this the DAMPERED case: the released strings' DamperJunctions
    // engage and their share of the tail decays through a real resistive two-port rather than
    // freely.
    //
    // HALF THE CHORD IS RELEASED AND HALF IS LEFT RINGING, and that is not a stylistic choice. With
    // all six released the network reaches digital silence 0.54 s after the note-off (measured: last
    // non-zero block 877 of 20671) -- a matched resistive termination on every string absorbs that
    // fast, and the silence watchdog then clears everything. A "60 s decay tail" that is 59.5 s of
    // zeros cannot exhibit a denormal, so the case would assert nothing. Releasing strings 0, 2 and
    // 4 keeps a real long decay in the network AND puts three engaged dampers in it, which is also
    // what a player does: damping some strings of a chord and letting the rest ring.
    const int releaseBlock = firstSecondBlocks * 2;

    DamperedTail result;
    result.totalBlocks = totalBlocks;
    for (int block = 0; block < totalBlocks; ++block) {
        if (block == releaseBlock) {
            events.clear();
            for (int s = 0; s < kDenormalStrings; s += 2) {
                NoteEvent event{};
                event.type = NoteEventType::NoteOff;
                event.sampleOffset = 0;
                event.stringIndex = static_cast<std::uint8_t>(s);
                event.channel = 0;
                event.midiNote = static_cast<std::uint8_t>(kDenormalChordNotes[s]);
                events.push(event);
            }
        }

        if (guarded) {
            const ScopedFtzDazGuard ftzDazGuard;
            chain.processBlock(params, events, kDenormalBlockSize);
        } else {
            chain.processBlock(params, events, kDenormalBlockSize);
        }
        events.clear();

        bool blockNonZero = false;
        for (int n = 0; n < kDenormalBlockSize; ++n) {
            const float sample = chain.mono[static_cast<std::size_t>(n)];
            result.allFinite &= std::isfinite(sample);
            if (std::fpclassify(sample) == FP_SUBNORMAL)
                ++result.subnormalOutputSamples;
            if (sample != 0.0f)
                blockNonZero = true;
            if (block < firstSecondBlocks)
                result.firstSecondPeak = std::max(result.firstSecondPeak, std::fabs(static_cast<double>(sample)));
        }
        const auto& taps = chain.network.tapBuffers();
        for (int s = 0; s < kDenormalStrings; ++s) {
            const float* channel = taps.channel(s, 0);
            for (int n = 0; n < kDenormalBlockSize; ++n) {
                result.allFinite &= std::isfinite(channel[n]);
                if (std::fpclassify(channel[n]) == FP_SUBNORMAL)
                    ++result.subnormalTapSamples;
            }
            result.maxDamperEngagement =
                std::max(result.maxDamperEngagement, static_cast<double>(chain.network.damperEngagement(s)));
        }
        for (int n = 0; n < kDenormalBlockSize; ++n) {
            const float sample = chain.network.bridgeOutputBuffer()[n];
            result.allFinite &= std::isfinite(sample);
            if (std::fpclassify(sample) == FP_SUBNORMAL)
                ++result.subnormalBridgeSamples;
        }

        if (blockNonZero) {
            result.lastNonZeroBlock = block;
            result.deepestTaps.clear();
            result.deepestDamperState.clear();
            for (int s = 0; s < kDenormalStrings; ++s) {
                const float* channel = taps.channel(s, 0);
                result.deepestTaps.insert(result.deepestTaps.end(), channel, channel + kDenormalBlockSize);
                result.deepestDamperState.push_back(chain.network.damperEngagement(s));
                result.deepestDamperState.push_back(chain.network.damperLossDepth(s));
                result.deepestDamperState.push_back(chain.network.damperPosition01(s));
            }
            result.deepestBridge.assign(chain.network.bridgeOutputBuffer(),
                                        chain.network.bridgeOutputBuffer() + kDenormalBlockSize);
            result.deepestOutput.assign(chain.mono.begin(), chain.mono.end());
            result.energyAtDeepest = chain.network.energyEstimate();
        }
    }

    return result;
}

long long countSubnormal(const std::vector<float>& values) {
    long long count = 0;
    for (const float value : values)
        if (std::fpclassify(value) == FP_SUBNORMAL)
            ++count;
    return count;
}

} // namespace

TEST_CASE("DENORMAL: six coupled strings with dampers leave no subnormal state", "[denormal][contract]") {
    const DamperedTail tail = renderDamperedTail(true, kDenormalTailSeconds);

    INFO("60 s tail, " << kDenormalStrings << " coupled strings, dampers engaged at 2 s, full monitoring chain");
    REQUIRE(tail.allFinite);
    // The chord really did sound, so nothing below can be satisfied by a render that was silent
    // from the start.
    REQUIRE(tail.firstSecondPeak > 0.001);
    // ...and every damper really did engage, or this is the P1 free-decay case under a new name.
    // 0.95 rather than 0.99 because the felt ramp is a one-pole and the silence watchdog clears the
    // string -- snapping engagement back to 0 -- before it asymptotes: MEASURED max 0.9753, i.e.
    // about 3.7 time constants of the 40 ms felt ramp. The bound is set below the measurement, not
    // fitted to it: anything above 0.95 is a damper that has done essentially all of its work.
    INFO("max damper engagement over the tail " << tail.maxDamperEngagement);
    REQUIRE(tail.maxDamperEngagement > 0.95);
    // ...and the damped decay ran for a long time before the watchdog cleared it, so the scan below
    // covers a real decay rather than one second of sound and fifty-nine of zeros.
    INFO("last non-zero block " << tail.lastNonZeroBlock << " of " << tail.totalBlocks);
    REQUIRE(tail.lastNonZeroBlock > tail.totalBlocks / 10);

    INFO("whole-tail subnormals: output " << tail.subnormalOutputSamples << ", taps " << tail.subnormalTapSamples
                                          << ", bridge " << tail.subnormalBridgeSamples);
    REQUIRE(tail.subnormalOutputSamples == 0);
    REQUIRE(tail.subnormalTapSamples == 0);
    REQUIRE(tail.subnormalBridgeSamples == 0);

    // The deepest LIVE state -- see DamperedTail's comment for why the final block is the wrong
    // place to look for it.
    const long long tapSubnormals = countSubnormal(tail.deepestTaps);
    const long long bridgeSubnormals = countSubnormal(tail.deepestBridge);
    const long long outputSubnormals = countSubnormal(tail.deepestOutput);
    const long long damperSubnormals = countSubnormal(tail.deepestDamperState);
    INFO("deepest-live-state subnormals: taps " << tapSubnormals << ", bridge " << bridgeSubnormals << ", output "
                                                << outputSubnormals << ", damper state " << damperSubnormals);
    REQUIRE(tapSubnormals == 0);
    REQUIRE(bridgeSubnormals == 0);
    REQUIRE(outputSubnormals == 0);
    REQUIRE(damperSubnormals == 0);

    INFO("energyEstimate at the deepest live state = " << tail.energyAtDeepest);
    REQUIRE(std::isfinite(tail.energyAtDeepest));
    REQUIRE(std::fpclassify(tail.energyAtDeepest) != FP_SUBNORMAL);

    std::cout << "[denormal] 60 s / 6 coupled strings with dampers + monitoring chain: peak in the first second "
              << tail.firstSecondPeak << ", last non-zero block " << tail.lastNonZeroBlock << "/" << tail.totalBlocks
              << ", energyEstimate at the deepest live state " << tail.energyAtDeepest << ", subnormal samples "
              << tail.subnormalOutputSamples << "/" << tail.subnormalTapSamples << "/" << tail.subnormalBridgeSamples
              << "\n";
}

// The non-vacuity arm (project ruling 7). "No subnormal state" has to be the FTZ/DAZ guard working
// rather than a render that could never have produced one -- so the identical render runs with the
// guard removed and this case asserts that subnormals DO appear. If they ever stop appearing here,
// the case above has gone toothless and this one is what says so.
TEST_CASE("DENORMAL: the six-string dampered case is guarded, not merely lucky", "[denormal][contract]") {
    const DamperedTail unguarded = renderDamperedTail(false, kDenormalTailSeconds);
    REQUIRE(unguarded.allFinite);
    REQUIRE(unguarded.firstSecondPeak > 0.001);

    const long long total =
        unguarded.subnormalOutputSamples + unguarded.subnormalTapSamples + unguarded.subnormalBridgeSamples;
    std::cout << "[denormal] the same render WITHOUT ScopedFtzDazGuard: subnormal samples output "
              << unguarded.subnormalOutputSamples << ", taps " << unguarded.subnormalTapSamples << ", bridge "
              << unguarded.subnormalBridgeSamples << " (the guarded arm reads 0/0/0)\n";
    INFO("unguarded subnormal samples: output " << unguarded.subnormalOutputSamples << ", taps "
                                                << unguarded.subnormalTapSamples << ", bridge "
                                                << unguarded.subnormalBridgeSamples);
    REQUIRE(total > 0);
}

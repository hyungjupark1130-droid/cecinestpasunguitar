// Task P1.11 -- the [contract] gate over cnpg_render and the P1 MIDI corpus (docs/plan.md
// section 4.8).
//
// Two cases, both tagged [contract] so both CI jobs run and gate on them (docs/plan.md section
// 4.9's test-to-CI mapping row "Contract battery, EventQueue, ModuleGraph, render determinism
// smoke"):
//
//   RENDER: determinism smoke        -- the same phrase rendered twice in-process and twice
//                                       out-of-process must produce byte-identical WAVs.
//   RENDER: P1 corpus renders clean  -- every corpus phrase renders NaN-free, subnormal-free and
//                                       below the SoftClipLimiter ceiling.
//
// Both drive the REAL cnpg_render executable ($<TARGET_FILE:cnpg_render>, baked in as
// CNPG_RENDER_EXE by tests/CMakeLists.txt) rather than a second copy of its render loop living
// here. That is deliberate and is the whole point: a determinism test that rendered through
// test-local code would prove that the TEST is deterministic, and a clip scan over test-local
// output would gate a signal path the listening pass never hears. The in-process half of the
// determinism check is delegated to the tool's own --verify-determinism flag, which renders the
// phrase twice from two independently prepared chains inside one process and refuses to write
// anything if the two differ -- so "twice in-process and once out-of-process" is checked across one
// renderer implementation, not two.
//
// Cost: the whole corpus is about 140 s of audio, which renders in well under a second (the P1
// single-string chain measures 0.263 % of one core at 48 kHz / 128-sample blocks --
// docs/bench/p1-baseline.md), so these cases stay comfortably inside the [contract] battery's
// budget rather than needing the hidden "[.]"-tag isolation this repo reserves for genuinely heavy
// report generators.

#include "cnpg/dsp/SoftClipLimiter.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

using namespace cnpg::dsp;

namespace {

namespace fs = std::filesystem;

// -------------------------------------------------------------------------------------------
// Driving the tool
// -------------------------------------------------------------------------------------------

fs::path renderExe() { return fs::path(CNPG_RENDER_EXE); }
fs::path corpusDir() { return fs::path(CNPG_CORPUS_DIR); }

// Scratch output lives next to the built binary rather than in the system temp directory: it is
// writable on every runner by construction (the build just wrote an executable there), it is
// removed by deleting the build tree like every other build product, and a failing render's WAV is
// left somewhere a human can actually go and listen to it.
fs::path scratchDir() { return renderExe().parent_path() / "render-tests"; }

std::string quoted(const fs::path& path) { return "\"" + path.string() + "\""; }

// Runs cnpg_render with `arguments`, capturing its stdout+stderr into `logPath`. Returns the
// process exit code (or a non-zero value if the shell itself could not run it).
int runRender(const std::string& arguments, const fs::path& logPath) {
    std::string command = quoted(renderExe()) + " " + arguments + " > " + quoted(logPath) + " 2>&1";
#if defined(_WIN32)
    // cmd.exe strips the FIRST and LAST quote of the command line before parsing it, so a command
    // that both starts with a quoted program path and ends with a quoted redirect target loses one
    // quote from each and mis-parses. Wrapping the whole thing in one more pair is the documented
    // fix; it is a no-op for the argument values themselves.
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

std::vector<std::uint8_t> readBinaryFile(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    REQUIRE(file.is_open());
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

void resetScratchDir(const fs::path& directory) {
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    REQUIRE(fs::is_directory(directory));
}

// -------------------------------------------------------------------------------------------
// Reading back what the tool wrote
// -------------------------------------------------------------------------------------------
//
// A deliberately independent reader: it parses the RIFF chunks itself rather than sharing code with
// tests/render/WavWriter.cpp, so a writer that emitted a self-consistently wrong file (a mislabelled
// format tag, a data chunk whose length disagrees with what was written) still fails here.

struct WavContents {
    int formatTag = 0;
    int numChannels = 0;
    int bitsPerSample = 0;
    double sampleRate = 0.0;
    std::vector<float> samples;
};

std::uint32_t readLe32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}

std::uint16_t readLe16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    const unsigned low = bytes[offset];
    const unsigned high = bytes[offset + 1];
    return static_cast<std::uint16_t>(low | (high << 8));
}

bool tagAt(const std::vector<std::uint8_t>& bytes, std::size_t offset, const char* tag) {
    return std::memcmp(bytes.data() + offset, tag, 4) == 0;
}

WavContents readWav(const fs::path& path) {
    const std::vector<std::uint8_t> bytes = readBinaryFile(path);
    REQUIRE(bytes.size() > 12);
    REQUIRE(tagAt(bytes, 0, "RIFF"));
    REQUIRE(tagAt(bytes, 8, "WAVE"));
    // The RIFF size field counts everything after it, so it and the file length must agree.
    REQUIRE(readLe32(bytes, 4) == static_cast<std::uint32_t>(bytes.size() - 8));

    WavContents wav;
    bool sawFormat = false;
    bool sawData = false;

    std::size_t offset = 12;
    while (offset + 8 <= bytes.size()) {
        const std::uint32_t chunkSize = readLe32(bytes, offset + 4);
        const std::size_t body = offset + 8;
        REQUIRE(body + chunkSize <= bytes.size());

        if (tagAt(bytes, offset, "fmt ")) {
            REQUIRE(chunkSize >= 16);
            wav.formatTag = readLe16(bytes, body);
            wav.numChannels = readLe16(bytes, body + 2);
            wav.sampleRate = static_cast<double>(readLe32(bytes, body + 4));
            wav.bitsPerSample = readLe16(bytes, body + 14);
            sawFormat = true;
        } else if (tagAt(bytes, offset, "data")) {
            REQUIRE(sawFormat);
            REQUIRE(wav.bitsPerSample == 32);
            REQUIRE(chunkSize % sizeof(float) == 0);
            wav.samples.resize(chunkSize / sizeof(float));
            for (std::size_t i = 0; i < wav.samples.size(); ++i) {
                const std::uint32_t raw = readLe32(bytes, body + i * sizeof(float));
                float sample = 0.0f;
                std::memcpy(&sample, &raw, sizeof(sample));
                wav.samples[i] = sample;
            }
            sawData = true;
        }

        offset = body + chunkSize + (chunkSize % 2); // RIFF chunks are word-aligned
    }

    REQUIRE(sawFormat);
    REQUIRE(sawData);
    return wav;
}

double dbOf(double linear) { return 20.0 * std::log10(std::max(linear, 1.0e-30)); }

// -------------------------------------------------------------------------------------------
// The scan
// -------------------------------------------------------------------------------------------

void scanRender(const fs::path& path) {
    INFO("render: " << path.string());

    const WavContents wav = readWav(path);
    CHECK(wav.formatTag == 3); // WAVE_FORMAT_IEEE_FLOAT
    CHECK(wav.numChannels == 1);
    CHECK(wav.bitsPerSample == 32);
    CHECK(wav.sampleRate == 48000.0);
    REQUIRE(wav.samples.size() > 0);

    long long nonFinite = 0;
    long long subnormal = 0;
    double peak = 0.0;

    for (const float sample : wav.samples) {
        if (!std::isfinite(sample)) {
            ++nonFinite;
            continue;
        }
        if (std::fpclassify(sample) == FP_SUBNORMAL)
            ++subnormal;
        peak = std::max(peak, std::fabs(static_cast<double>(sample)));
    }

    INFO("peak " << dbOf(peak) << " dBFS over " << wav.samples.size() << " samples");

    CHECK(nonFinite == 0);

    // Denormal hygiene, the artifact-level counterpart of the [denormal] state-inspection case:
    // cnpg_render engages the same ScopedFtzDazGuard the plugin's processBlock() does, so nothing
    // the chain produces can be subnormal. A subnormal in the file means the guard was not engaged
    // over the render -- which is exactly the regression that would make a long decay tail cost
    // orders of magnitude more CPU in a real host.
    CHECK(subnormal == 0);

    // Clip-free, per the Task P1.11 acceptance criterion: SoftClipLimiter is hard-wired LAST in the
    // chain precisely so this bound is enforceable on everything upstream (SoftClipLimiter.h). The
    // ceiling is an asymptote rather than a clamp, so the mathematical bound is |y| < ceiling; the
    // tolerance below only covers float rounding of a value already under it.
    const double ceilingLinear = std::pow(10.0, static_cast<double>(SoftClipLimiterParams{}.ceilingDb) / 20.0);
    CHECK(peak <= ceilingLinear * 1.0001);

    // A renderer that wrote silence would satisfy every assertion above. The quietest P1 corpus
    // phrase is comfortably above this floor (a single string is calibrated to a -18 dBFS nominal,
    // docs/plan.md section 1.9), so -60 dBFS only catches "nothing was rendered at all".
    CHECK(dbOf(peak) > -60.0);
}

const char* const kP1Phrases[] = {"01_chromatic_singles", "03_legato_retrigger", "05_low_string_bends",
                                  "07_param_sweeps_midnote"};

} // namespace

// -------------------------------------------------------------------------------------------

TEST_CASE("RENDER: determinism smoke", "[contract]") {
    const fs::path scratch = scratchDir() / "determinism";
    resetScratchDir(scratch);

    const fs::path midi = corpusDir() / "05_low_string_bends.mid";
    REQUIRE(fs::exists(midi));

    const fs::path firstWav = scratch / "05_low_string_bends_run1.wav";
    const fs::path secondWav = scratch / "05_low_string_bends_run2.wav";
    const fs::path verifiedWav = scratch / "05_low_string_bends_verified.wav";
    const fs::path log = scratch / "render.log";

    // Exactly the Task P1.11 acceptance-criterion command, run twice.
    const std::string common = "--midi " + quoted(midi) + " --samplerate 48000 --blocksize 128 --out ";

    const int firstExit = runRender(common + quoted(firstWav), log);
    INFO("cnpg_render (run 1) log:\n" << readTextFile(log));
    REQUIRE(firstExit == 0);

    const int secondExit = runRender(common + quoted(secondWav), log);
    INFO("cnpg_render (run 2) log:\n" << readTextFile(log));
    REQUIRE(secondExit == 0);

    // Out-of-process identity: two separate runs of the same binary on the same input must produce
    // byte-identical FILES, header included -- not merely identical sample streams.
    const std::vector<std::uint8_t> firstBytes = readBinaryFile(firstWav);
    const std::vector<std::uint8_t> secondBytes = readBinaryFile(secondWav);
    REQUIRE(firstBytes.size() == secondBytes.size());
    CHECK(firstBytes == secondBytes);

    // In-process identity: the tool renders the phrase twice from two independently prepared chains
    // and exits non-zero if they differ, so a zero exit here IS the in-process half of the check.
    const int verifyExit = runRender(common + quoted(verifiedWav) + " --verify-determinism", log);
    INFO("cnpg_render (--verify-determinism) log:\n" << readTextFile(log));
    REQUIRE(verifyExit == 0);

    // ...and the verified render must still be the same audio as the plain ones, which is what
    // rules out --verify-determinism being self-consistently wrong (two identical renders of
    // something other than what the plain path renders).
    const std::vector<std::uint8_t> verifiedBytes = readBinaryFile(verifiedWav);
    CHECK(verifiedBytes == firstBytes);

    scanRender(firstWav);
}

TEST_CASE("RENDER: every corpus phrase renders NaN-free, denormal-free and below the limiter ceiling", "[contract]") {
    const fs::path scratch = scratchDir() / "corpus";
    resetScratchDir(scratch);

    const fs::path log = scratch / "render.log";
    const int exitCode = runRender(
        "--corpus " + quoted(corpusDir()) + " --out " + quoted(scratch) + " --samplerate 48000 --blocksize 128", log);
    INFO("cnpg_render (--corpus) log:\n" << readTextFile(log));
    REQUIRE(exitCode == 0);

    // Every WAV the run produced is scanned, and every P1 phrase must be among them. Written this
    // way round rather than against a hardcoded list of four filenames so Task P2.8's four new
    // phrases are picked up and scanned automatically when they land, without this test needing an
    // edit to keep gating what it is supposed to gate.
    std::vector<fs::path> produced;
    for (const fs::directory_entry& entry : fs::directory_iterator(scratch)) {
        if (entry.is_regular_file() && entry.path().extension() == ".wav")
            produced.push_back(entry.path());
    }
    std::sort(produced.begin(), produced.end());
    REQUIRE(produced.size() >= std::size(kP1Phrases));

    for (const fs::path& render : produced)
        scanRender(render);

    for (const char* phrase : kP1Phrases) {
        INFO("phrase: " << phrase);
        const std::string prefix = std::string(phrase) + "__cv";
        const auto match = std::find_if(produced.begin(), produced.end(), [&](const fs::path& render) {
            const std::string name = render.filename().string();
            // "<stem>__cv<corpusVersion>_s<source hash>.wav" -- docs/plan.md section 4.8 requires the
            // corpus version and a provenance field in the filename so a listening note is
            // attributable. The version itself is deliberately not asserted here: it increments every
            // time the corpus grows, and pinning it would make this test a maintenance tax on adding
            // a phrase.
            //
            // `_s`, not `_g`, from Task P2.7 (carry-forward C2): the field was a CONFIGURE-TIME git
            // commit, which is resolved when CMake last ran rather than when the binary was built --
            // a build/ tree configured at 77b0430 filed renders of the code at 1ccfcb1 as
            // `..._cv1_g77b0430.wav`, so two different code states produced identical filenames. It
            // is now a RENDER-TIME content hash over the bytes a render is a function of
            // (tests/support/SourceHash.h::renderSourceHash), and the prefix changed with it so
            // nobody reads it as a commit.
            return name.rfind(prefix, 0) == 0 && name.find("_s") != std::string::npos;
        });
        CHECK(match != produced.end());
    }
}

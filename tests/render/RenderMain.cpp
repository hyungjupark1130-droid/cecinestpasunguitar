// cnpg_render -- headless MIDI-to-WAV renderer over the P1 dsp/ chain (Task P1.11, docs/plan.md
// section 4.8). Links cnpg_dsp ONLY -- zero JUCE, zero Catch2 -- so it builds and runs on the
// ubuntu dsp-only CI job exactly like cnpg_dsp/cnpg_tests/cnpg_bench do (docs/plan.md section 1.5's
// naming registry). It is not a test; it is the tool that produces the artifacts the author's
// listening pass judges, and the tool the [contract] determinism/scan test drives
// (tests/dsp/RenderTests.cpp).
//
// -----------------------------------------------------------------------------------------------
// What it renders, and why that is the same thing the plugin plays.
// -----------------------------------------------------------------------------------------------
//
// The signal chain and its per-block six-call setParams cascade come from cnpg::test::P1Chain
// (tests/support/P1Chain.h), which is the SAME code cnpg_bench measures -- see that header for why
// the assembly lives there rather than being copied into each executable. On top of the chain, this
// file reproduces the rest of PluginProcessor::renderChunk() exactly:
//
//   - MIDI arrives as (status, data1, data2, sampleOffset) tuples, the shape docs/plan.md section
//     2.14 locks, built here by MidiFileReader instead of by plugin/src/MidiConverter.cpp from a
//     juce::MidiBuffer. Both feed cnpg::dsp::translateRawMidi() the identical tuple.
//   - The pitch wheel is picked out of the raw stream by this layer (it is a latched global
//     controller, not a note event) and mapped through cnpg::dsp::pitchWheelToSemitones(), latched
//     per block -- the same chunk-granular resolution renderChunk() uses, for the same reason
//     (StringNetwork's own f0 smoother is what makes a bend click-free).
//   - Everything else goes through cnpg::dsp::NoteAllocator::allocate() into a BlockEventQueue.
//   - The whole per-block call runs under a ScopedFtzDazGuard, exactly as processBlock() does, so
//     the rendered samples carry the plugin's real denormal behaviour rather than an offline
//     variant of it.
//
// P1 is the single-string vertical slice (docs/plan.md Task P1.5 "P1 SCOPE"), so the network runs
// one string and NoteAllocator is prepared for one, matching PluginProcessor's kP1NumStrings.
//
// -----------------------------------------------------------------------------------------------
// CLI.
// -----------------------------------------------------------------------------------------------
//
//   cnpg_render --midi <file> --out <wav> [--sidecar <json>] [--samplerate R] [--blocksize B]
//               [--verify-determinism]
//   cnpg_render --corpus <dir> --out <dir>  [--samplerate R] [--blocksize B] [--verify-determinism]
//
// --midi/--out is the locked P1 form (Task P1.11 step 1): one MIDI file in, one named WAV out, with
// --out taken literally so an acceptance command can name its own output path. --corpus renders
// every phrase in a corpus manifest (tests/corpus/corpus.json) in one run, and there --out names a
// DIRECTORY: the per-phrase filenames are generated, and they embed the corpus version and the git
// hash, because docs/plan.md section 4.8 requires exactly that ("Render filenames embed corpus
// version + git hash so listening notes are attributable") and a caller-supplied literal filename
// structurally cannot. That is also the shape Task P2.8's own command line already assumes
// (`--corpus tests\corpus ... --out renders\p2`), so --out doing double duty is the locked
// spelling rather than a local invention. In --corpus mode each phrase's manifest entry supplies
// its base parameters, its RetriggerMode, and its automation sidecar; in --midi mode the defaults
// apply unless --sidecar names one explicitly.
//
// --verify-determinism renders the phrase TWICE inside this one process, from two independently
// prepared chains, and requires the two sample streams to be bit-identical before anything is
// written. Together with the caller running the tool twice and byte-comparing the files, that is
// exactly the "twice in-process and once out-of-process" check docs/plan.md section 4.8 specifies
// for `RENDER: determinism smoke` -- and it checks a single renderer implementation rather than
// comparing this one against a second, hand-duplicated copy of the render loop living in a test.
//
// -----------------------------------------------------------------------------------------------
// Determinism.
// -----------------------------------------------------------------------------------------------
//
// Nothing in this file reads the wall clock, draws from an unseeded RNG, or depends on filesystem
// iteration order (the corpus render order is the manifest's array order, not a directory scan).
// The one stochastic component in the chain, PluckExciter's noise burst, reseeds to a fixed
// non-time-based constant on every prepare()/reset() (PluckExciter.h) -- and at the shipped default
// PluckExciterParams::noiseAmount of 0 it contributes nothing at all. Renders are therefore
// byte-identical run to run for the same binary and the same inputs; cross-toolchain identity is
// not claimed (docs/plan.md section 4.8 explicitly does not claim it either).

#include "MidiFileReader.h"
#include "WavWriter.h"

#include "support/P1Chain.h"

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/MidiTranslation.h"
#include "cnpg/dsp/NoteAllocator.h"
#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/ScopedFtzDazGuard.h"
#include "cnpg/dsp/StringNetwork.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#ifndef CNPG_RENDER_GIT_COMMIT
#define CNPG_RENDER_GIT_COMMIT "unknown" // safety net if a non-CMake build forgets to define this
#endif

namespace {

namespace fs = std::filesystem;

using cnpg::render::MidiFileContents;
using cnpg::render::MidiFileEvent;

// Silence rendered after the last MIDI event, so every phrase's own decay and release tail is
// inside the artifact the listening pass hears (checklist item 11, "gaps between phrases decay to
// digital silence"). Fixed rather than manifest-driven on purpose: the tail must not be something a
// phrase can shorten, or "the note was still ringing when the file ended" becomes a per-phrase
// authoring accident instead of a property of the render. StringNetwork's P1 release envelope
// reaches its clear-out floor in 460 ms (StringNetwork.h), so 2 s covers the release plus a freely
// ringing low string's remaining decay.
constexpr double kTailSeconds = 2.0;

// -------------------------------------------------------------------------------------------
// A minimal JSON reader.
// -------------------------------------------------------------------------------------------
//
// cnpg_render reads two JSON documents: the corpus manifest (tests/corpus/corpus.json) and a
// phrase's automation sidecar (e.g. tests/corpus/07_param_sweeps_midnote.json). Both are authored
// in this repo, so this parser only has to cover real JSON correctly and refuse everything else
// with a message naming the offset -- the same trade tests/support/GoldenIo.cpp makes for the
// golden sidecars ("a tiny reader ... instead of pulling a JSON dependency into a JUCE-free
// binary"), except that these documents are nested, so a flat key/value scanner will not do.
// Numbers are parsed as double via std::strtod; strings support the standard escapes including
// \uXXXX (encoded to UTF-8), because refusing them would be a silent trap for a future
// description field.
class JsonValue {
  public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<JsonValue> elements;                        // Array
    std::vector<std::pair<std::string, JsonValue>> members; // Object (insertion order preserved)

    bool isNull() const noexcept { return type == Type::Null; }

    const JsonValue* find(const std::string& key) const noexcept {
        if (type != Type::Object)
            return nullptr;
        for (const auto& member : members) {
            if (member.first == key)
                return &member.second;
        }
        return nullptr;
    }
};

class JsonParser {
  public:
    JsonParser(const std::string& text, std::string& error) : text_(text), error_(error) {}

    bool parse(JsonValue& out) {
        skipWhitespace();
        if (!parseValue(out))
            return false;
        skipWhitespace();
        if (pos_ != text_.size()) {
            fail("trailing characters after the top-level value");
            return false;
        }
        return true;
    }

  private:
    bool fail(const std::string& what) {
        error_ = "JSON parse error at offset " + std::to_string(pos_) + ": " + what;
        return false;
    }

    void skipWhitespace() noexcept {
        while (pos_ < text_.size()) {
            const char c = text_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
                ++pos_;
            else
                break;
        }
    }

    bool literal(const char* word) {
        const std::size_t length = std::strlen(word);
        if (text_.compare(pos_, length, word) != 0)
            return false;
        pos_ += length;
        return true;
    }

    bool parseValue(JsonValue& out) {
        if (pos_ >= text_.size())
            return fail("unexpected end of document");

        switch (text_[pos_]) {
        case '{':
            return parseObject(out);
        case '[':
            return parseArray(out);
        case '"':
            out.type = JsonValue::Type::String;
            return parseString(out.text);
        case 't':
            if (!literal("true"))
                return fail("expected 'true'");
            out.type = JsonValue::Type::Bool;
            out.boolean = true;
            return true;
        case 'f':
            if (!literal("false"))
                return fail("expected 'false'");
            out.type = JsonValue::Type::Bool;
            out.boolean = false;
            return true;
        case 'n':
            if (!literal("null"))
                return fail("expected 'null'");
            out.type = JsonValue::Type::Null;
            return true;
        default:
            return parseNumber(out);
        }
    }

    bool parseObject(JsonValue& out) {
        out = JsonValue{};
        out.type = JsonValue::Type::Object;
        ++pos_; // '{'
        skipWhitespace();
        if (pos_ < text_.size() && text_[pos_] == '}') {
            ++pos_;
            return true;
        }
        for (;;) {
            skipWhitespace();
            if (pos_ >= text_.size() || text_[pos_] != '"')
                return fail("expected a string key");
            std::string key;
            if (!parseString(key))
                return false;
            skipWhitespace();
            if (pos_ >= text_.size() || text_[pos_] != ':')
                return fail("expected ':' after an object key");
            ++pos_;
            skipWhitespace();
            JsonValue value;
            if (!parseValue(value))
                return false;
            out.members.emplace_back(std::move(key), std::move(value));
            skipWhitespace();
            if (pos_ < text_.size() && text_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (pos_ < text_.size() && text_[pos_] == '}') {
                ++pos_;
                return true;
            }
            return fail("expected ',' or '}' in object");
        }
    }

    bool parseArray(JsonValue& out) {
        out = JsonValue{};
        out.type = JsonValue::Type::Array;
        ++pos_; // '['
        skipWhitespace();
        if (pos_ < text_.size() && text_[pos_] == ']') {
            ++pos_;
            return true;
        }
        for (;;) {
            skipWhitespace();
            JsonValue value;
            if (!parseValue(value))
                return false;
            out.elements.push_back(std::move(value));
            skipWhitespace();
            if (pos_ < text_.size() && text_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (pos_ < text_.size() && text_[pos_] == ']') {
                ++pos_;
                return true;
            }
            return fail("expected ',' or ']' in array");
        }
    }

    void appendUtf8(std::string& out, unsigned int codepoint) {
        if (codepoint < 0x80u) {
            out.push_back(static_cast<char>(codepoint));
        } else if (codepoint < 0x800u) {
            out.push_back(static_cast<char>(0xC0u | (codepoint >> 6)));
            out.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
        } else {
            out.push_back(static_cast<char>(0xE0u | (codepoint >> 12)));
            out.push_back(static_cast<char>(0x80u | ((codepoint >> 6) & 0x3Fu)));
            out.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
        }
    }

    bool parseHex4(unsigned int& out) {
        if (pos_ + 4 > text_.size())
            return fail("truncated \\u escape");
        unsigned int value = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[pos_++];
            value <<= 4;
            if (c >= '0' && c <= '9')
                value |= static_cast<unsigned int>(c - '0');
            else if (c >= 'a' && c <= 'f')
                value |= static_cast<unsigned int>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                value |= static_cast<unsigned int>(c - 'A' + 10);
            else
                return fail("non-hex digit in \\u escape");
        }
        out = value;
        return true;
    }

    bool parseString(std::string& out) {
        out.clear();
        ++pos_; // opening quote
        for (;;) {
            if (pos_ >= text_.size())
                return fail("unterminated string");
            const char c = text_[pos_++];
            if (c == '"')
                return true;
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (pos_ >= text_.size())
                return fail("unterminated escape sequence");
            const char escape = text_[pos_++];
            switch (escape) {
            case '"':
                out.push_back('"');
                break;
            case '\\':
                out.push_back('\\');
                break;
            case '/':
                out.push_back('/');
                break;
            case 'b':
                out.push_back('\b');
                break;
            case 'f':
                out.push_back('\f');
                break;
            case 'n':
                out.push_back('\n');
                break;
            case 'r':
                out.push_back('\r');
                break;
            case 't':
                out.push_back('\t');
                break;
            case 'u': {
                unsigned int codepoint = 0;
                if (!parseHex4(codepoint))
                    return false;
                // Surrogate pair: a high surrogate must be followed by \uDC00-\uDFFF.
                if (codepoint >= 0xD800u && codepoint <= 0xDBFFu && pos_ + 1 < text_.size() && text_[pos_] == '\\' &&
                    text_[pos_ + 1] == 'u') {
                    pos_ += 2;
                    unsigned int low = 0;
                    if (!parseHex4(low))
                        return false;
                    if (low < 0xDC00u || low > 0xDFFFu)
                        return fail("invalid low surrogate in \\u escape pair");
                    const unsigned int combined = 0x10000u + ((codepoint - 0xD800u) << 10) + (low - 0xDC00u);
                    out.push_back(static_cast<char>(0xF0u | (combined >> 18)));
                    out.push_back(static_cast<char>(0x80u | ((combined >> 12) & 0x3Fu)));
                    out.push_back(static_cast<char>(0x80u | ((combined >> 6) & 0x3Fu)));
                    out.push_back(static_cast<char>(0x80u | (combined & 0x3Fu)));
                    break;
                }
                appendUtf8(out, codepoint);
                break;
            }
            default:
                return fail("unknown escape sequence");
            }
        }
    }

    bool parseNumber(JsonValue& out) {
        const char* begin = text_.c_str() + pos_;
        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(begin, &end);
        if (end == begin)
            return fail("expected a value");
        if (errno == ERANGE || !std::isfinite(value))
            return fail("number is out of range or non-finite");
        pos_ += static_cast<std::size_t>(end - begin);
        out.type = JsonValue::Type::Number;
        out.number = value;
        return true;
    }

    const std::string& text_;
    std::string& error_;
    std::size_t pos_ = 0;
};

bool readTextFile(const fs::path& path, std::string& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        error = path.string() + ": cannot open file for reading";
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    return true;
}

bool readJsonFile(const fs::path& path, JsonValue& out, std::string& error) {
    std::string text;
    if (!readTextFile(path, text, error))
        return false;
    // Strip a UTF-8 BOM: an editor on Windows can add one, and it is not JSON whitespace.
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        text.erase(0, 3);
    std::string parseError;
    JsonParser parser(text, parseError);
    if (!parser.parse(out)) {
        error = path.string() + ": " + parseError;
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------------------------
// The automatable parameter vocabulary.
// -------------------------------------------------------------------------------------------
//
// One name set, shared by a manifest entry's `params` object (constant values for the whole
// phrase) and by an automation sidecar's lanes (values that move during it). The names are
// deliberately the plugin's own APVTS parameter IDs (plugin/src/Parameters.h, namespace ID), so a
// corpus phrase and a DAW automation lane refer to the same parameter by the same name -- a corpus
// author never has to translate between two vocabularies, and a mismatch between them cannot creep
// in silently.
//
// Only continuously-valued parameters appear here. The three booleans/enums of the P1 surface
// (triodeBypass, cabBypass, retriggerMode) are per-phrase settings, not lanes: they are read from a
// manifest entry's own fields, because interpolating "bypass" between breakpoints is not a
// meaningful operation and a lane that silently thresholded at 0.5 would be a trap.
enum class AutomatableParam {
    ExciterDefaultPosition,
    ExciterDefaultHardness,
    ExciterNoiseAmount,
    MaterialLossGainLow,
    MaterialLossGainHigh,
    MaterialDispersionAmount,
    PickupPosition01,
    DamperPosition01,
    PickupResonanceHz,
    PickupQ,
    PickupOutputGainDb,
    TriodeDrive,
    TriodeOutputTrimDb,
    OutputGainDb,
    LimiterCeilingDb
};

struct ParamNameEntry {
    const char* name;
    AutomatableParam param;
};

constexpr ParamNameEntry kParamNames[] = {
    {"exciterDefaultPosition", AutomatableParam::ExciterDefaultPosition},
    {"exciterDefaultHardness", AutomatableParam::ExciterDefaultHardness},
    {"exciterNoiseAmount", AutomatableParam::ExciterNoiseAmount},
    {"materialLossGainLow", AutomatableParam::MaterialLossGainLow},
    {"materialLossGainHigh", AutomatableParam::MaterialLossGainHigh},
    {"materialDispersionAmount", AutomatableParam::MaterialDispersionAmount},
    {"pickupPosition01", AutomatableParam::PickupPosition01},
    // damperPosition01 is stored by StringNetworkParams and is INERT in P1 -- DamperJunction is
    // Task P2.2 (StringNetwork.h's P1 SCOPE note). It is accepted here so a P2 corpus phrase does
    // not need a renderer change, and tests/corpus/README.md says plainly that a P1 render will not
    // respond to it; accepting-and-documenting beats rejecting a name the manifest schema will
    // need one phase later.
    {"damperPosition01", AutomatableParam::DamperPosition01},
    {"pickupResonanceHz", AutomatableParam::PickupResonanceHz},
    {"pickupQ", AutomatableParam::PickupQ},
    {"pickupOutputGainDb", AutomatableParam::PickupOutputGainDb},
    {"triodeDrive", AutomatableParam::TriodeDrive},
    {"triodeOutputTrimDb", AutomatableParam::TriodeOutputTrimDb},
    {"outputGainDb", AutomatableParam::OutputGainDb},
    {"limiterCeilingDb", AutomatableParam::LimiterCeilingDb},
};

bool lookupParamName(const std::string& name, AutomatableParam& out) noexcept {
    for (const ParamNameEntry& entry : kParamNames) {
        if (name == entry.name) {
            out = entry.param;
            return true;
        }
    }
    return false;
}

std::string knownParamNames() {
    std::string all;
    for (const ParamNameEntry& entry : kParamNames) {
        if (!all.empty())
            all += ", ";
        all += entry.name;
    }
    return all;
}

void applyParam(cnpg::test::P1ChainParams& params, AutomatableParam which, float value) noexcept {
    switch (which) {
    case AutomatableParam::ExciterDefaultPosition:
        params.network.exciter.defaultPosition = value;
        break;
    case AutomatableParam::ExciterDefaultHardness:
        params.network.exciter.defaultHardness = value;
        break;
    case AutomatableParam::ExciterNoiseAmount:
        params.network.exciter.noiseAmount = value;
        break;
    case AutomatableParam::MaterialLossGainLow:
        params.network.material.lossGainLow = value;
        break;
    case AutomatableParam::MaterialLossGainHigh:
        params.network.material.lossGainHigh = value;
        break;
    case AutomatableParam::MaterialDispersionAmount:
        params.network.material.dispersionAmount = value;
        break;
    case AutomatableParam::PickupPosition01:
        params.network.pickupPosition01 = value;
        break;
    case AutomatableParam::DamperPosition01:
        params.network.damperPosition01 = value;
        break;
    case AutomatableParam::PickupResonanceHz:
        params.pickup.resonanceHz = value;
        break;
    case AutomatableParam::PickupQ:
        params.pickup.q = value;
        break;
    case AutomatableParam::PickupOutputGainDb:
        params.pickup.outputGainDb = value;
        break;
    case AutomatableParam::TriodeDrive:
        params.triode.drive = value;
        break;
    case AutomatableParam::TriodeOutputTrimDb:
        params.triode.outputTrimDb = value;
        break;
    case AutomatableParam::OutputGainDb:
        params.outputGain.gainDb = value;
        break;
    case AutomatableParam::LimiterCeilingDb:
        params.limiter.ceilingDb = value;
        break;
    }
}

// -------------------------------------------------------------------------------------------
// Automation lanes.
// -------------------------------------------------------------------------------------------
//
// A lane is a parameter plus a list of breakpoints, linearly interpolated between them and held
// flat outside the first and last. docs/plan.md section 4.8 calls these "sample-stamped
// breakpoints" and that is what the renderer applies: `time` is authored in SECONDS and resolved
// to an exact sample stamp at the render rate. Seconds rather than raw sample indices in the file
// itself because the same sidecar has to render correctly at 44.1, 48 and 96 kHz (Task P2.8 renders
// the whole corpus at all three) -- a fixed sample index would name a different musical moment at
// each rate, and would silently desynchronise the sweep from the MIDI phrase it is sweeping over.
//
// The value applied to a block is the lane evaluated at that block's FIRST sample, held for the
// block: that is not an approximation of what a host does, it is exactly what a host does -- the
// APVTS snapshot is read once per block (docs/plan.md Task P1.1) and every module's own smoother is
// what turns a per-block step into a continuous ramp (the per-block linear ramp convention
// OutputGain/PickupTap/TriodeStage/SoftClipLimiter share, and StringNetwork's per-sample position
// smoother). Rendering automation any finer than the plugin can actually receive it would make the
// corpus sweeps a test of something the instrument never does.
struct Breakpoint {
    double timeSeconds = 0.0;
    double value = 0.0;
};

struct AutomationLane {
    AutomatableParam param = AutomatableParam::TriodeDrive;
    std::string name;
    std::vector<Breakpoint> breakpoints; // non-decreasing in timeSeconds, at least one entry

    double valueAt(double seconds) const noexcept {
        if (breakpoints.empty())
            return 0.0;
        if (seconds <= breakpoints.front().timeSeconds)
            return breakpoints.front().value;
        if (seconds >= breakpoints.back().timeSeconds)
            return breakpoints.back().value;

        // Linear scan from the cached cursor: evaluation is strictly forward in time across a
        // render, so this is O(1) amortized per block without needing a binary search.
        while (cursor_ + 1 < breakpoints.size() && breakpoints[cursor_ + 1].timeSeconds <= seconds)
            ++cursor_;
        while (cursor_ > 0 && breakpoints[cursor_].timeSeconds > seconds)
            --cursor_;

        const Breakpoint& a = breakpoints[cursor_];
        const Breakpoint& b = breakpoints[cursor_ + 1];
        const double span = b.timeSeconds - a.timeSeconds;
        if (!(span > 0.0))
            return b.value; // coincident breakpoints: a step, taking the later value
        const double t = (seconds - a.timeSeconds) / span;
        return a.value + t * (b.value - a.value);
    }

    void rewind() noexcept { cursor_ = 0; }

  private:
    mutable std::size_t cursor_ = 0;
};

bool parseAutomation(const fs::path& path, std::vector<AutomationLane>& out, std::string& error) {
    out.clear();

    JsonValue root;
    if (!readJsonFile(path, root, error))
        return false;
    if (root.type != JsonValue::Type::Object) {
        error = path.string() + ": automation sidecar must be a JSON object";
        return false;
    }

    const JsonValue* lanes = root.find("lanes");
    if (lanes == nullptr || lanes->type != JsonValue::Type::Array) {
        error = path.string() + ": automation sidecar must carry a \"lanes\" array";
        return false;
    }

    for (const JsonValue& lane : lanes->elements) {
        if (lane.type != JsonValue::Type::Object) {
            error = path.string() + ": every entry of \"lanes\" must be an object";
            return false;
        }
        const JsonValue* param = lane.find("param");
        if (param == nullptr || param->type != JsonValue::Type::String) {
            error = path.string() + ": a lane is missing its \"param\" name";
            return false;
        }

        AutomationLane parsed;
        parsed.name = param->text;
        if (!lookupParamName(parsed.name, parsed.param)) {
            error = path.string() + ": unknown automation parameter \"" + parsed.name + "\"; known names are " +
                    knownParamNames();
            return false;
        }

        const JsonValue* breakpoints = lane.find("breakpoints");
        if (breakpoints == nullptr || breakpoints->type != JsonValue::Type::Array || breakpoints->elements.empty()) {
            error = path.string() + ": lane \"" + parsed.name + "\" needs a non-empty \"breakpoints\" array";
            return false;
        }

        double previousTime = -std::numeric_limits<double>::infinity();
        for (const JsonValue& breakpoint : breakpoints->elements) {
            const JsonValue* time = breakpoint.find("time");
            const JsonValue* value = breakpoint.find("value");
            if (time == nullptr || time->type != JsonValue::Type::Number || value == nullptr ||
                value->type != JsonValue::Type::Number) {
                error = path.string() + ": lane \"" + parsed.name +
                        "\" has a breakpoint without numeric \"time\" and \"value\" fields";
                return false;
            }
            if (time->number < previousTime) {
                error = path.string() + ": lane \"" + parsed.name + "\" breakpoints must be non-decreasing in time";
                return false;
            }
            previousTime = time->number;
            parsed.breakpoints.push_back(Breakpoint{time->number, value->number});
        }

        out.push_back(std::move(parsed));
    }

    return true;
}

// -------------------------------------------------------------------------------------------
// Corpus manifest.
// -------------------------------------------------------------------------------------------

struct PhraseEntry {
    std::string file;    // MIDI filename, relative to the corpus directory
    std::string sidecar; // automation sidecar filename, or empty
    std::string description;
    std::string retriggerModeName = "Physical";
    cnpg::dsp::RetriggerMode retriggerMode = cnpg::dsp::RetriggerMode::Physical;
    long long seed = 0;
    double durationSeconds = 0.0; // the manifest's recorded expected render length
    std::vector<std::pair<AutomatableParam, double>> paramOverrides;
};

struct CorpusManifest {
    long long corpusVersion = 0;
    std::vector<PhraseEntry> phrases;
};

bool parseParamOverrides(const JsonValue& object, const fs::path& manifestPath, const std::string& phrase,
                         std::vector<std::pair<AutomatableParam, double>>& out, std::string& error) {
    for (const auto& member : object.members) {
        AutomatableParam which{};
        if (!lookupParamName(member.first, which)) {
            error = manifestPath.string() + ": phrase \"" + phrase + "\" sets unknown parameter \"" + member.first +
                    "\"; known names are " + knownParamNames();
            return false;
        }
        if (member.second.type != JsonValue::Type::Number) {
            error = manifestPath.string() + ": phrase \"" + phrase + "\" sets \"" + member.first +
                    "\" to a non-numeric value";
            return false;
        }
        out.emplace_back(which, member.second.number);
    }
    return true;
}

bool parseManifest(const fs::path& path, CorpusManifest& out, std::string& error) {
    out = CorpusManifest{};

    JsonValue root;
    if (!readJsonFile(path, root, error))
        return false;
    if (root.type != JsonValue::Type::Object) {
        error = path.string() + ": the corpus manifest must be a JSON object";
        return false;
    }

    const JsonValue* version = root.find("corpusVersion");
    if (version == nullptr || version->type != JsonValue::Type::Number) {
        error = path.string() + ": the corpus manifest must carry an integer \"corpusVersion\"";
        return false;
    }
    out.corpusVersion = static_cast<long long>(version->number);

    const JsonValue* phrases = root.find("phrases");
    if (phrases == nullptr || phrases->type != JsonValue::Type::Array) {
        error = path.string() + ": the corpus manifest must carry a \"phrases\" array";
        return false;
    }

    for (const JsonValue& phrase : phrases->elements) {
        if (phrase.type != JsonValue::Type::Object) {
            error = path.string() + ": every entry of \"phrases\" must be an object";
            return false;
        }

        PhraseEntry entry;

        const JsonValue* file = phrase.find("file");
        if (file == nullptr || file->type != JsonValue::Type::String || file->text.empty()) {
            error = path.string() + ": a phrase entry is missing its \"file\" name";
            return false;
        }
        entry.file = file->text;

        if (const JsonValue* sidecar = phrase.find("sidecar"); sidecar != nullptr && !sidecar->isNull()) {
            if (sidecar->type != JsonValue::Type::String) {
                error = path.string() + ": phrase \"" + entry.file + "\" has a non-string \"sidecar\"";
                return false;
            }
            entry.sidecar = sidecar->text;
        }

        if (const JsonValue* description = phrase.find("description");
            description != nullptr && description->type == JsonValue::Type::String)
            entry.description = description->text;

        if (const JsonValue* mode = phrase.find("retriggerMode"); mode != nullptr && !mode->isNull()) {
            if (mode->type != JsonValue::Type::String) {
                error = path.string() + ": phrase \"" + entry.file + "\" has a non-string \"retriggerMode\"";
                return false;
            }
            if (mode->text == "Physical") {
                entry.retriggerMode = cnpg::dsp::RetriggerMode::Physical;
            } else if (mode->text == "Synth") {
                entry.retriggerMode = cnpg::dsp::RetriggerMode::Synth;
            } else {
                error = path.string() + ": phrase \"" + entry.file + "\" has retriggerMode \"" + mode->text +
                        "\"; expected \"Physical\" or \"Synth\"";
                return false;
            }
            entry.retriggerModeName = mode->text;
        }

        if (const JsonValue* seed = phrase.find("seed"); seed != nullptr && seed->type == JsonValue::Type::Number)
            entry.seed = static_cast<long long>(seed->number);

        if (const JsonValue* duration = phrase.find("durationSeconds");
            duration != nullptr && duration->type == JsonValue::Type::Number)
            entry.durationSeconds = duration->number;

        if (const JsonValue* params = phrase.find("params"); params != nullptr && !params->isNull()) {
            if (params->type != JsonValue::Type::Object) {
                error = path.string() + ": phrase \"" + entry.file + "\" has a non-object \"params\"";
                return false;
            }
            if (!parseParamOverrides(*params, path, entry.file, entry.paramOverrides, error))
                return false;
        }

        out.phrases.push_back(std::move(entry));
    }

    if (out.phrases.empty()) {
        error = path.string() + ": the corpus manifest lists no phrases";
        return false;
    }

    return true;
}

// -------------------------------------------------------------------------------------------
// The render itself.
// -------------------------------------------------------------------------------------------

struct RenderSpec {
    fs::path midiPath;
    fs::path sidecarPath; // empty if the phrase has no automation
    cnpg::dsp::RetriggerMode retriggerMode = cnpg::dsp::RetriggerMode::Physical;
    std::vector<std::pair<AutomatableParam, double>> paramOverrides;
    double sampleRate = 48000.0;
    int blockSize = 128;

    // The manifest's recorded render length, in seconds; 0 in --midi mode (no manifest to check
    // against). When present it is verified, not trusted -- see renderOne().
    double expectedDurationSeconds = 0.0;
};

// How far a phrase's actual rendered length may sit from the manifest's recorded durationSeconds
// before the render is treated as a manifest error. The render length is a pure function of the
// MIDI file's last event plus kTailSeconds, so any real disagreement means the manifest went stale
// against the corpus -- and a stale "expected render duration" is exactly the kind of quiet drift
// an append-only, versioned corpus exists to prevent. The tolerance only absorbs the manifest's own
// three-decimal rounding and the sample-rate rounding of the tick-to-sample conversion.
constexpr double kDurationToleranceSeconds = 0.01;

struct RenderStats {
    long long numSamples = 0;
    double durationSeconds = 0.0;
    float peak = 0.0f;
    double rms = 0.0;
    double dcOffset = 0.0;
    long long nonFiniteSamples = 0;
    long long subnormalSamples = 0;
    long long midiEvents = 0;
    std::uint32_t droppedNoteEvents = 0;
};

// MIDI status nibbles this layer reads directly, exactly as plugin/src/PluginProcessor.cpp does:
// the pitch wheel is not a note event, so it never goes through NoteAllocator.
constexpr std::uint8_t kStatusTypeMask = 0xF0u;
constexpr std::uint8_t kPitchWheelStatus = 0xE0u;

int pitchWheelValue(std::uint8_t data1, std::uint8_t data2) noexcept {
    return (static_cast<int>(data2) << 7) | static_cast<int>(data1);
}

// Renders one phrase into `samples`. Everything it touches is constructed fresh here, so calling it
// twice in a row with the same arguments is exactly the in-process determinism check
// --verify-determinism performs.
void renderPhrase(const MidiFileContents& midi, const RenderSpec& spec, std::vector<AutomationLane>& lanes,
                  long long totalSamples, std::vector<float>& samples, RenderStats& stats) {
    samples.assign(static_cast<std::size_t>(totalSamples), 0.0f);
    stats = RenderStats{};
    stats.numSamples = totalSamples;
    stats.durationSeconds = static_cast<double>(totalSamples) / spec.sampleRate;
    stats.midiEvents = static_cast<long long>(midi.events.size());

    for (AutomationLane& lane : lanes)
        lane.rewind();

    // P1 is the single-string vertical slice, matching PluginProcessor's kP1NumStrings.
    constexpr int kP1NumStrings = 1;

    cnpg::test::P1Chain chain;
    chain.prepare(spec.sampleRate, spec.blockSize, kP1NumStrings, cnpg::dsp::Oversampler::kDefaultFactor);
    chain.reset();

    cnpg::dsp::NoteAllocator allocator;
    allocator.prepare(kP1NumStrings);

    cnpg::test::P1ChainParams baseParams = cnpg::test::makeDefaultP1ChainParams();
    baseParams.network.retriggerMode = spec.retriggerMode;
    for (const auto& setting : spec.paramOverrides)
        applyParam(baseParams, setting.first, static_cast<float>(setting.second));

    cnpg::dsp::BlockEventQueue noteEvents;
    std::vector<cnpg::dsp::RawMidiEvent> rawEvents;
    rawEvents.reserve(64);

    float pitchBendSemitones = 0.0f;
    std::size_t eventCursor = 0;
    long long rendered = 0;

    double peak = 0.0;
    double sumSquares = 0.0;
    double sum = 0.0;

    while (rendered < totalSamples) {
        const int numSamples = static_cast<int>(std::min<long long>(spec.blockSize, totalSamples - rendered));
        const long long blockEnd = rendered + numSamples;

        // Collect this block's MIDI, rebased onto block-local offsets. MidiFileReader hands events
        // back non-decreasing in sample, which is the order NoteAllocator::allocate() requires.
        rawEvents.clear();
        while (eventCursor < midi.events.size() && midi.events[eventCursor].sample < blockEnd) {
            const MidiFileEvent& event = midi.events[eventCursor];
            const auto offset = static_cast<std::int32_t>(
                std::clamp<long long>(event.sample - rendered, 0, static_cast<long long>(numSamples) - 1));
            rawEvents.push_back(cnpg::dsp::translateRawMidi(event.status, event.data1, event.data2, offset));
            ++eventCursor;
        }

        // Pitch wheel: latched from this block's messages, applied from the start of the block --
        // the same chunk-granular resolution PluginProcessor::renderChunk() uses.
        for (const cnpg::dsp::RawMidiEvent& event : rawEvents) {
            if (static_cast<std::uint8_t>(event.status & kStatusTypeMask) == kPitchWheelStatus)
                pitchBendSemitones = cnpg::dsp::pitchWheelToSemitones(pitchWheelValue(event.data1, event.data2));
        }

        cnpg::test::P1ChainParams blockParams = baseParams;
        const double blockStartSeconds = static_cast<double>(rendered) / spec.sampleRate;
        for (const AutomationLane& lane : lanes)
            applyParam(blockParams, lane.param, static_cast<float>(lane.valueAt(blockStartSeconds)));
        blockParams.network.pitchBendSemitones = pitchBendSemitones;

        noteEvents.clear();
        allocator.allocate(rawEvents.data(), static_cast<int>(rawEvents.size()), noteEvents);
        stats.droppedNoteEvents += noteEvents.droppedCount();

        {
            // Exactly the guard PluginProcessor::processBlock() engages around the identical calls.
            const cnpg::dsp::ScopedFtzDazGuard ftzDazGuard;
            chain.processBlock(blockParams, noteEvents, numSamples);
        }

        for (int n = 0; n < numSamples; ++n) {
            const float sample = chain.mono[static_cast<std::size_t>(n)];
            samples[static_cast<std::size_t>(rendered + n)] = sample;

            if (!std::isfinite(sample)) {
                ++stats.nonFiniteSamples;
                continue;
            }
            if (std::fpclassify(sample) == FP_SUBNORMAL)
                ++stats.subnormalSamples;

            const double magnitude = std::fabs(static_cast<double>(sample));
            peak = std::max(peak, magnitude);
            sumSquares += static_cast<double>(sample) * static_cast<double>(sample);
            sum += static_cast<double>(sample);
        }

        rendered = blockEnd;
    }

    const double count = static_cast<double>(std::max<long long>(totalSamples, 1));
    stats.peak = static_cast<float>(peak);
    stats.rms = std::sqrt(sumSquares / count);
    stats.dcOffset = sum / count;
}

double dbOf(double linear) { return 20.0 * std::log10(std::max(linear, 1.0e-30)); }

// -------------------------------------------------------------------------------------------
// CLI
// -------------------------------------------------------------------------------------------

struct RenderArgs {
    std::string midi;
    std::string corpus;
    std::string out;
    std::string sidecar;
    double sampleRate = 48000.0;
    int blockSize = 128;
    bool verifyDeterminism = false;
};

void printUsage(std::FILE* stream) {
    std::fprintf(stream,
                 "usage: cnpg_render --midi <file> --out <wav> [--sidecar <json>] [--samplerate R] "
                 "[--blocksize B] [--verify-determinism]\n"
                 "       cnpg_render --corpus <dir> --out <dir> [--samplerate R] [--blocksize B] "
                 "[--verify-determinism]\n"
                 "  --midi <file>          one Standard MIDI File to render\n"
                 "  --corpus <dir>         a corpus directory containing corpus.json; renders every phrase\n"
                 "  --out <path>           output WAV file (--midi mode) or output directory (--corpus mode)\n"
                 "  --sidecar <json>       automation sidecar for --midi mode (--corpus reads it from the manifest)\n"
                 "  --samplerate R         render sample rate in Hz (default 48000)\n"
                 "  --blocksize B          render block size in samples (default 128)\n"
                 "  --verify-determinism   render each phrase twice in-process and require bit-identical output\n");
}

bool parseDouble(const char* text, double& out) noexcept {
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(value))
        return false;
    out = value;
    return true;
}

bool parseInt(const char* text, int& out) noexcept {
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE || value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max())
        return false;
    out = static_cast<int>(value);
    return true;
}

bool parseArgs(int argc, char** argv, RenderArgs& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        const auto nextValue = [&](const char* flagName) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "cnpg_render: missing value for %s\n", flagName);
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--midi") {
            const char* value = nextValue("--midi");
            if (value == nullptr)
                return false;
            args.midi = value;
        } else if (arg == "--corpus") {
            const char* value = nextValue("--corpus");
            if (value == nullptr)
                return false;
            args.corpus = value;
        } else if (arg == "--out") {
            const char* value = nextValue("--out");
            if (value == nullptr)
                return false;
            args.out = value;
        } else if (arg == "--sidecar") {
            const char* value = nextValue("--sidecar");
            if (value == nullptr)
                return false;
            args.sidecar = value;
        } else if (arg == "--samplerate") {
            const char* value = nextValue("--samplerate");
            if (value == nullptr || !parseDouble(value, args.sampleRate)) {
                std::fprintf(stderr, "cnpg_render: invalid --samplerate value\n");
                return false;
            }
        } else if (arg == "--blocksize") {
            const char* value = nextValue("--blocksize");
            if (value == nullptr || !parseInt(value, args.blockSize)) {
                std::fprintf(stderr, "cnpg_render: invalid --blocksize value\n");
                return false;
            }
        } else if (arg == "--verify-determinism") {
            args.verifyDeterminism = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(stdout);
            std::exit(0);
        } else {
            std::fprintf(stderr, "cnpg_render: unknown argument '%s'\n", arg.c_str());
            return false;
        }
    }
    return true;
}

// One phrase, start to finish: read the MIDI and any sidecar, render (twice if asked), write the
// WAV, print the summary. Returns false with everything already reported on stderr.
bool renderOne(const RenderSpec& spec, const fs::path& outputPath, bool verifyDeterminism, const std::string& label) {
    MidiFileContents midi;
    std::string error;
    if (!cnpg::render::readMidiFile(spec.midiPath, spec.sampleRate, midi, error)) {
        std::fprintf(stderr, "cnpg_render: %s\n", error.c_str());
        return false;
    }

    std::vector<AutomationLane> lanes;
    if (!spec.sidecarPath.empty() && !parseAutomation(spec.sidecarPath, lanes, error)) {
        std::fprintf(stderr, "cnpg_render: %s\n", error.c_str());
        return false;
    }

    const long long tailSamples = static_cast<long long>(std::llround(kTailSeconds * spec.sampleRate));
    const long long totalSamples = std::max<long long>(midi.lastEventSample + tailSamples, tailSamples);

    if (spec.expectedDurationSeconds > 0.0) {
        const double actualSeconds = static_cast<double>(totalSamples) / spec.sampleRate;
        if (std::fabs(actualSeconds - spec.expectedDurationSeconds) > kDurationToleranceSeconds) {
            std::fprintf(stderr,
                         "cnpg_render: %s renders %.3f s but the manifest records durationSeconds %.3f -- "
                         "the manifest is stale against the corpus\n",
                         label.c_str(), actualSeconds, spec.expectedDurationSeconds);
            return false;
        }
    }

    std::vector<float> samples;
    RenderStats stats;
    renderPhrase(midi, spec, lanes, totalSamples, samples, stats);

    if (verifyDeterminism) {
        std::vector<float> second;
        RenderStats secondStats;
        renderPhrase(midi, spec, lanes, totalSamples, second, secondStats);
        if (second.size() != samples.size() ||
            std::memcmp(second.data(), samples.data(), samples.size() * sizeof(float)) != 0) {
            std::fprintf(stderr,
                         "cnpg_render: %s is NOT deterministic -- two in-process renders of the same input "
                         "differ\n",
                         label.c_str());
            return false;
        }
    }

    if (!cnpg::render::writeWavFloat32(outputPath, samples, 1, spec.sampleRate, error)) {
        std::fprintf(stderr, "cnpg_render: %s\n", error.c_str());
        return false;
    }

    std::printf("  %-34s -> %s\n", label.c_str(), outputPath.string().c_str());
    std::printf("    %lld samples (%.3f s), %lld MIDI event(s), peak %.2f dBFS, rms %.2f dBFS, dc %.2f dBFS\n",
                stats.numSamples, stats.durationSeconds, stats.midiEvents, dbOf(stats.peak), dbOf(stats.rms),
                dbOf(std::fabs(stats.dcOffset)));
    std::printf("    nonFinite=%lld subnormal=%lld droppedNoteEvents=%u%s\n", stats.nonFiniteSamples,
                stats.subnormalSamples, stats.droppedNoteEvents,
                verifyDeterminism ? " determinism=verified(2 in-process renders bit-identical)" : "");
    std::fflush(stdout);

    if (stats.nonFiniteSamples > 0) {
        std::fprintf(stderr, "cnpg_render: %s produced %lld non-finite sample(s)\n", label.c_str(),
                     stats.nonFiniteSamples);
        return false;
    }
    if (stats.droppedNoteEvents > 0) {
        std::fprintf(stderr, "cnpg_render: %s dropped %u note event(s) -- the block event queue overflowed\n",
                     label.c_str(), stats.droppedNoteEvents);
        return false;
    }

    return true;
}

// Render filenames in --corpus mode: "<phrase stem>__cv<corpusVersion>_g<git hash>.wav"
// (docs/plan.md section 4.8: "Render filenames embed corpus version + git hash so listening notes
// are attributable"). The git hash is baked in at configure time (tests/render/CMakeLists.txt) and
// carries the same caveat cnpg_bench's does: it necessarily names the PARENT of the commit that
// lands the binary, which is fine for attributing a listening note and is not the self-consistent
// content hash the golden sidecars need (tests/support/SourceHash.h explains that distinction).
std::string renderFileName(const std::string& midiFileName, long long corpusVersion) {
    std::string stem = midiFileName;
    const auto dot = stem.find_last_of('.');
    if (dot != std::string::npos)
        stem.erase(dot);
    return stem + "__cv" + std::to_string(corpusVersion) + "_g" + CNPG_RENDER_GIT_COMMIT + ".wav";
}

} // namespace

int main(int argc, char** argv) {
    RenderArgs args;
    if (!parseArgs(argc, argv, args)) {
        printUsage(stderr);
        return 1;
    }

    if (args.midi.empty() == args.corpus.empty()) {
        std::fprintf(stderr, "cnpg_render: specify exactly one of --midi or --corpus\n");
        printUsage(stderr);
        return 1;
    }
    if (args.out.empty()) {
        std::fprintf(stderr, "cnpg_render: --out is required\n");
        printUsage(stderr);
        return 1;
    }
    if (!args.sidecar.empty() && !args.corpus.empty()) {
        std::fprintf(stderr, "cnpg_render: --sidecar applies to --midi mode only "
                             "(--corpus reads each phrase's sidecar from the manifest)\n");
        return 1;
    }
    if (!(args.sampleRate > 0.0) || args.sampleRate > 500000.0) {
        std::fprintf(stderr, "cnpg_render: --samplerate must be a positive, finite value\n");
        return 1;
    }
    if (args.blockSize < 1 || args.blockSize > 1000000) {
        std::fprintf(stderr, "cnpg_render: --blocksize must be within 1..1000000\n");
        return 1;
    }

    std::printf("cnpg_render -- P1 chain, %.0f Hz, %d-sample blocks, git %s\n", args.sampleRate, args.blockSize,
                CNPG_RENDER_GIT_COMMIT);
    // stdout is block-buffered when it is a pipe or a file -- which is exactly how the [contract]
    // test and any CI step capture it -- while stderr is not, so without these flushes a diagnostic
    // would land in the captured log ABOVE the progress lines that led to it. Flushing at each
    // point where the next thing written might be an error keeps the two streams interleaved in the
    // order they actually happened.
    std::fflush(stdout);

    if (!args.midi.empty()) {
        RenderSpec spec;
        spec.midiPath = fs::path(args.midi);
        spec.sidecarPath = args.sidecar.empty() ? fs::path{} : fs::path(args.sidecar);
        spec.sampleRate = args.sampleRate;
        spec.blockSize = args.blockSize;

        const fs::path outputPath(args.out);
        if (!renderOne(spec, outputPath, args.verifyDeterminism, spec.midiPath.filename().string()))
            return 1;
        return 0;
    }

    const fs::path corpusDir(args.corpus);
    const fs::path manifestPath = corpusDir / "corpus.json";
    CorpusManifest manifest;
    std::string error;
    if (!parseManifest(manifestPath, manifest, error)) {
        std::fprintf(stderr, "cnpg_render: %s\n", error.c_str());
        return 1;
    }

    std::printf("  corpus %s (corpusVersion %lld, %d phrase(s))\n", corpusDir.string().c_str(), manifest.corpusVersion,
                static_cast<int>(manifest.phrases.size()));
    std::fflush(stdout);

    const fs::path outputDir(args.out);
    for (const PhraseEntry& phrase : manifest.phrases) {
        RenderSpec spec;
        spec.midiPath = corpusDir / phrase.file;
        spec.sidecarPath = phrase.sidecar.empty() ? fs::path{} : corpusDir / phrase.sidecar;
        spec.retriggerMode = phrase.retriggerMode;
        spec.paramOverrides = phrase.paramOverrides;
        spec.sampleRate = args.sampleRate;
        spec.blockSize = args.blockSize;
        spec.expectedDurationSeconds = phrase.durationSeconds;

        const fs::path outputPath = outputDir / renderFileName(phrase.file, manifest.corpusVersion);
        if (!renderOne(spec, outputPath, args.verifyDeterminism, phrase.file))
            return 1;
    }

    return 0;
}

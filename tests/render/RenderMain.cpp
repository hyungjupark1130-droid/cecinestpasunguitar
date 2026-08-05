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
// DIRECTORY: the per-phrase filenames are generated, and they embed the corpus version and a
// provenance digest, because docs/plan.md section 4.8 requires exactly that ("Render filenames embed
// corpus version + a render-time content hash so listening notes are attributable", as amended by
// this task's carry-forward C2) and a caller-supplied literal filename structurally cannot. That is
// also the shape Task P2.8's own command line already assumes
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

#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/IBridgePort.h"
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
#include <system_error>
#include <utility>
#include <vector>

#include "support/SourceHash.h"

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

    // Array: `values` holds the elements and `keys` is empty.
    // Object: `values` holds the member values and `keys` their names, index-parallel, in document
    // order.
    //
    // Two parallel vectors rather than the obvious std::vector<std::pair<std::string, JsonValue>>,
    // and the reason is not style. std::vector is one of the three containers the standard
    // explicitly permits to be instantiated with an INCOMPLETE type (complete before any member is
    // used), which is exactly what makes a recursive node like this legal at all. std::pair carries
    // no such permission, so instantiating std::pair<std::string, JsonValue> inside JsonValue's own
    // definition is ill-formed. MSVC and libstdc++-under-GCC accept it anyway; libstdc++ under
    // Clang correctly rejects it ("template argument must be a complete class type"), which is how
    // this was caught -- by the ubuntu (clang) CI job, on the first push of this file.
    std::vector<JsonValue> values;
    std::vector<std::string> keys;

    bool isNull() const noexcept { return type == Type::Null; }

    const JsonValue* find(const std::string& key) const noexcept {
        if (type != Type::Object)
            return nullptr;
        for (std::size_t i = 0; i < keys.size() && i < values.size(); ++i) {
            if (keys[i] == key)
                return &values[i];
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
            out.keys.push_back(std::move(key));
            out.values.push_back(std::move(value));
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
            out.values.push_back(std::move(value));
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
// Task P2.8 added the P2 half of this vocabulary: the three bridge-admittance parameters, the two
// damper parameters that are not the position, and the eight per-string tuning offsets. All of them
// are APVTS parameters the plugin already ships (plugin/src/Parameters.h), and all of them are
// continuous, so they are lanes on the same footing as the P1 set. The DISCRETE P2 parameters --
// retriggerMode and numStrings -- are a separate vocabulary below, because a linearly interpolated
// "mode" is the trap this comment's P1 half already refused.
enum class AutomatableParam {
    ExciterDefaultPosition,
    ExciterDefaultHardness,
    ExciterNoiseAmount,
    StringMaterialLossGainLow,
    StringMaterialLossGainHigh,
    StringMaterialDispersionAmount,
    PickupPosition01,
    DamperPosition01,
    DamperMaxLoss,
    DamperFeltTimeMs,
    BridgeCoupling,
    BridgeResonanceHz,
    BridgeDamping,
    StringTuningOffsetCents0,
    StringTuningOffsetCents1,
    StringTuningOffsetCents2,
    StringTuningOffsetCents3,
    StringTuningOffsetCents4,
    StringTuningOffsetCents5,
    StringTuningOffsetCents6,
    StringTuningOffsetCents7,
    PickupResonanceHz,
    PickupQ,
    PickupOutputGainDb,
    TriodeDrive,
    TriodeOutputTrimDb,
    OutputGainDb,
    LimiterCeilingDb
};

// -----------------------------------------------------------------------------------------------
// THE DISCRETE PARAMETERS ARE **NOT** LANES, AND THE REASON IS A MEASUREMENT (Task P2.8)
// -----------------------------------------------------------------------------------------------
//
// docs/plan.md's P2.8 step text asks for "positions, coupling, material, retriggerMode switches
// driven alongside MIDI". The first three are lanes above. The fourth, and `numStrings` with it, are
// NOT, and this file carried a stepped-lane implementation of both for part of the task before it
// was removed. Recording why, because "we could add it later" is not the same claim as "it would
// work":
//
//   - `retriggerMode` as a mid-render lane has no technical obstacle, but nothing would drive it.
//     The plan's own file list spells the deliverable as "per-retriggerMode VARIANT RENDERS of
//     03_legato_retrigger.mid (render configurations, not a new MIDI file)", which is what
//     kRenderVariants below produces. A parse-and-apply path that no shipped corpus sidecar and no
//     shipped variant ever exercises is exactly the vacuity this project keeps finding, so it is not
//     shipped.
//
//   - `numStrings` as a mid-render lane is worse than unexercised: the gesture it exists for CANNOT
//     be a corpus render as the tool is specified. A count reduction under a held chord takes
//     strings out of the active set while they own notes, so their note-offs become undeliverable --
//     NoteAllocator counts them on unaddressableNoteOffCount(), by design, and renderOne() FAILS the
//     render on a non-zero reading, also by design (see there: a non-zero count means something moved
//     a count the render did not ask to move). Both behaviours are correct and neither should be
//     weakened to let a listening artifact exist. The gesture therefore stays a HOST check --
//     docs/listening/P2.6-ableton-checks.md check B, which is checklist item 16's only evidence and
//     is documented as such in the checklist.
//
// Both remain per-phrase manifest fields (`retriggerMode`, `numStrings`), which is where a value
// that does not move belongs.

struct ParamNameEntry {
    const char* name;
    AutomatableParam param;
};

constexpr ParamNameEntry kParamNames[] = {
    {"exciterDefaultPosition", AutomatableParam::ExciterDefaultPosition},
    {"exciterDefaultHardness", AutomatableParam::ExciterDefaultHardness},
    {"exciterNoiseAmount", AutomatableParam::ExciterNoiseAmount},
    // Task P2.1 renamed these three APVTS ids material* -> stringMaterial* (ADR 0004: Material/Wood
    // belongs to the BODY module a later phase adds). BOTH spellings are accepted, and the legacy
    // one is not deprecation cruft: tests/corpus/07_param_sweeps_midnote.json is corpus v1 data,
    // and tests/corpus/README.md's versioning rule is that an existing phrase's BYTES NEVER CHANGE
    // -- that is what makes "cv1" in a render filename identify exactly one set of inputs, and what
    // the P1 listening notes are attributable against. Rewriting a frozen sidecar to chase a source
    // rename would break that guarantee for a purely cosmetic gain, so the renderer absorbs the
    // rename instead. New phrases (P2.8 onward) use the stringMaterial* spelling.
    {"stringMaterialLossGainLow", AutomatableParam::StringMaterialLossGainLow},
    {"stringMaterialLossGainHigh", AutomatableParam::StringMaterialLossGainHigh},
    {"stringMaterialDispersionAmount", AutomatableParam::StringMaterialDispersionAmount},
    {"materialLossGainLow", AutomatableParam::StringMaterialLossGainLow},           // corpus v1 spelling
    {"materialLossGainHigh", AutomatableParam::StringMaterialLossGainHigh},         // corpus v1 spelling
    {"materialDispersionAmount", AutomatableParam::StringMaterialDispersionAmount}, // corpus v1 spelling
    {"pickupPosition01", AutomatableParam::PickupPosition01},
    // damperPosition01 is stored by StringNetworkParams and is INERT in P1 -- DamperJunction is
    // Task P2.2 (StringNetwork.h's P1 SCOPE note). It is accepted here so a P2 corpus phrase does
    // not need a renderer change, and tests/corpus/README.md says plainly that a P1 render will not
    // respond to it; accepting-and-documenting beats rejecting a name the manifest schema will
    // need one phase later.
    {"damperPosition01", AutomatableParam::DamperPosition01},
    // ---- the P2 additions (Task P2.8) --------------------------------------------------------
    {"damperMaxLoss", AutomatableParam::DamperMaxLoss},
    {"damperFeltTimeMs", AutomatableParam::DamperFeltTimeMs},
    {"bridgeCoupling", AutomatableParam::BridgeCoupling},
    {"bridgeResonanceHz", AutomatableParam::BridgeResonanceHz},
    {"bridgeDamping", AutomatableParam::BridgeDamping},
    {"stringTuningOffsetCents0", AutomatableParam::StringTuningOffsetCents0},
    {"stringTuningOffsetCents1", AutomatableParam::StringTuningOffsetCents1},
    {"stringTuningOffsetCents2", AutomatableParam::StringTuningOffsetCents2},
    {"stringTuningOffsetCents3", AutomatableParam::StringTuningOffsetCents3},
    {"stringTuningOffsetCents4", AutomatableParam::StringTuningOffsetCents4},
    {"stringTuningOffsetCents5", AutomatableParam::StringTuningOffsetCents5},
    {"stringTuningOffsetCents6", AutomatableParam::StringTuningOffsetCents6},
    {"stringTuningOffsetCents7", AutomatableParam::StringTuningOffsetCents7},
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
    case AutomatableParam::StringMaterialLossGainLow:
        params.network.stringMaterial.lossGainLow = value;
        break;
    case AutomatableParam::StringMaterialLossGainHigh:
        params.network.stringMaterial.lossGainHigh = value;
        break;
    case AutomatableParam::StringMaterialDispersionAmount:
        params.network.stringMaterial.dispersionAmount = value;
        break;
    case AutomatableParam::PickupPosition01:
        params.network.pickupPosition01 = value;
        break;
    case AutomatableParam::DamperPosition01:
        params.network.damperPosition01 = value;
        break;
    case AutomatableParam::DamperMaxLoss:
        params.network.damper.maxLoss = value;
        break;
    case AutomatableParam::DamperFeltTimeMs:
        params.network.damper.feltTimeConstantMs = value;
        break;
    case AutomatableParam::BridgeCoupling:
        params.network.bridge.couplingStrength = value;
        break;
    case AutomatableParam::BridgeResonanceHz:
        params.network.bridge.resonanceHz = value;
        break;
    case AutomatableParam::BridgeDamping:
        params.network.bridge.damping = value;
        break;
    case AutomatableParam::StringTuningOffsetCents0:
    case AutomatableParam::StringTuningOffsetCents1:
    case AutomatableParam::StringTuningOffsetCents2:
    case AutomatableParam::StringTuningOffsetCents3:
    case AutomatableParam::StringTuningOffsetCents4:
    case AutomatableParam::StringTuningOffsetCents5:
    case AutomatableParam::StringTuningOffsetCents6:
    case AutomatableParam::StringTuningOffsetCents7: {
        const auto slot = static_cast<std::size_t>(static_cast<int>(which) -
                                                   static_cast<int>(AutomatableParam::StringTuningOffsetCents0));
        params.network.perString[slot].tuningOffsetCents = value;
        break;
    }
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

struct Automation {
    std::vector<AutomationLane> continuous;

    void rewind() noexcept {
        for (AutomationLane& lane : continuous)
            lane.rewind();
    }
};

bool parseAutomation(const fs::path& path, Automation& out, std::string& error) {
    out = Automation{};

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

    for (const JsonValue& lane : lanes->values) {
        if (lane.type != JsonValue::Type::Object) {
            error = path.string() + ": every entry of \"lanes\" must be an object";
            return false;
        }
        const JsonValue* param = lane.find("param");
        if (param == nullptr || param->type != JsonValue::Type::String) {
            error = path.string() + ": a lane is missing its \"param\" name";
            return false;
        }

        const std::string name = param->text;
        AutomatableParam continuousParam{};
        if (!lookupParamName(name, continuousParam)) {
            error =
                path.string() + ": unknown automation parameter \"" + name + "\"; known names are " + knownParamNames();
            return false;
        }

        const JsonValue* breakpoints = lane.find("breakpoints");
        if (breakpoints == nullptr || breakpoints->type != JsonValue::Type::Array || breakpoints->values.empty()) {
            error = path.string() + ": lane \"" + name + "\" needs a non-empty \"breakpoints\" array";
            return false;
        }

        std::vector<Breakpoint> parsedBreakpoints;
        double previousTime = -std::numeric_limits<double>::infinity();
        for (const JsonValue& breakpoint : breakpoints->values) {
            const JsonValue* time = breakpoint.find("time");
            const JsonValue* value = breakpoint.find("value");
            if (time == nullptr || time->type != JsonValue::Type::Number || value == nullptr ||
                value->type != JsonValue::Type::Number) {
                error = path.string() + ": lane \"" + name +
                        "\" has a breakpoint without numeric \"time\" and \"value\" fields";
                return false;
            }
            if (time->number < previousTime) {
                error = path.string() + ": lane \"" + name + "\" breakpoints must be non-decreasing in time";
                return false;
            }
            previousTime = time->number;
            parsedBreakpoints.push_back(Breakpoint{time->number, value->number});
        }

        AutomationLane parsed;
        parsed.name = name;
        parsed.param = continuousParam;
        parsed.breakpoints = std::move(parsedBreakpoints);
        out.continuous.push_back(std::move(parsed));
    }

    return true;
}

// -------------------------------------------------------------------------------------------
// Corpus manifest.
// -------------------------------------------------------------------------------------------

// THE DEFAULTS BELOW ARE THE P1 SINGLE-STRING CONFIGURATION, AND THAT IS DELIBERATE (Task P2.8).
//
// tests/corpus/README.md rule 1 freezes a committed phrase's bytes, and a phrase entry's fields are
// part of what "cv1 identifies exactly one set of inputs" means. The P1 entries (01, 03, 05, 07)
// therefore do not carry `numStrings` or `allocationMode`, and their absence has to keep meaning
// what it meant when they were written: one string, FreeZones over the whole of MIDI. That is not a
// nostalgic default -- it is what makes each of those phrases still MEAN what its own `exercises`
// field says. Phrase 01 is a chromatic run from MIDI 21 to 108, which the shipped 6-string EADGBE
// fingering table cannot play at all (it spans 40..88, so 40 notes would come back UNASSIGNABLE and
// the render would fail); phrase 03's legato slurs are retriggers only while every note lands on
// the same string, and on six strings each slur note would take a free string instead and there
// would be no legato left to judge. The P2 entries (02, 04, 06, 08) state their configuration
// explicitly, and it is the plugin's shipped default: 6 strings, GuitarFingering.
struct PhraseEntry {
    std::string file;    // MIDI filename, relative to the corpus directory
    std::string sidecar; // automation sidecar filename, or empty
    std::string description;
    std::string retriggerModeName = "Physical";
    cnpg::dsp::RetriggerMode retriggerMode = cnpg::dsp::RetriggerMode::Physical;
    int numStrings = 1;
    std::string allocationModeName = "FreeZones";
    cnpg::dsp::AllocationMode allocationMode = cnpg::dsp::AllocationMode::FreeZones;
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
    for (std::size_t i = 0; i < object.keys.size() && i < object.values.size(); ++i) {
        const std::string& name = object.keys[i];
        const JsonValue& value = object.values[i];

        AutomatableParam which{};
        if (!lookupParamName(name, which)) {
            error = manifestPath.string() + ": phrase \"" + phrase + "\" sets unknown parameter \"" + name +
                    "\"; known names are " + knownParamNames();
            return false;
        }
        if (value.type != JsonValue::Type::Number) {
            error = manifestPath.string() + ": phrase \"" + phrase + "\" sets \"" + name + "\" to a non-numeric value";
            return false;
        }
        out.emplace_back(which, value.number);
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

    for (const JsonValue& phrase : phrases->values) {
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

        if (const JsonValue* strings = phrase.find("numStrings"); strings != nullptr && !strings->isNull()) {
            if (strings->type != JsonValue::Type::Number) {
                error = path.string() + ": phrase \"" + entry.file + "\" has a non-numeric \"numStrings\"";
                return false;
            }
            const auto requested = static_cast<int>(std::llround(strings->number));
            if (requested < 1 || requested > cnpg::dsp::kMaxStrings) {
                error = path.string() + ": phrase \"" + entry.file + "\" asks for " + std::to_string(requested) +
                        " string(s); the instrument has 1.." + std::to_string(cnpg::dsp::kMaxStrings);
                return false;
            }
            entry.numStrings = requested;
        }

        if (const JsonValue* mode = phrase.find("allocationMode"); mode != nullptr && !mode->isNull()) {
            if (mode->type != JsonValue::Type::String) {
                error = path.string() + ": phrase \"" + entry.file + "\" has a non-string \"allocationMode\"";
                return false;
            }
            if (mode->text == "GuitarFingering") {
                entry.allocationMode = cnpg::dsp::AllocationMode::GuitarFingering;
            } else if (mode->text == "FreeZones") {
                entry.allocationMode = cnpg::dsp::AllocationMode::FreeZones;
            } else {
                error = path.string() + ": phrase \"" + entry.file + "\" has allocationMode \"" + mode->text +
                        "\"; expected \"GuitarFingering\" or \"FreeZones\"";
                return false;
            }
            entry.allocationModeName = mode->text;
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
    int numStrings = 1;
    cnpg::dsp::AllocationMode allocationMode = cnpg::dsp::AllocationMode::FreeZones;
    std::vector<std::pair<AutomatableParam, double>> paramOverrides;
    // THE NEGATIVE CONTROL (Task P2.8, carry-forward C2). When set, the bridge in the loop is a
    // port that forwards every IBridgePort call to a real BridgeJunction and overrides exactly one
    // method -- reflectionPhaseDelaySamples() -- to report 0. That is Task P2.4's instrument: same
    // scattering, same storage, same dissipation, no phase-delay term in the loop-length solve. Any
    // difference between the two renders is P2.7's compensation and can be nothing else.
    bool bridgePhaseBlind = false;
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
    // NoteAllocator's own drop (Task P2.6): a NoteOn no enabled in-count string could play. It is a
    // DIFFERENT failure from a queue overflow and it has to be reported separately, because a note
    // the allocator declined and a note nobody played sound exactly alike -- and because a
    // misconfigured allocator is now a way for this tool to render a corpus phrase with notes
    // missing and still report every other statistic as healthy.
    std::uint32_t unassignableNotes = 0;
    // ...and the third one, which is about the CORPUS rather than about the allocator's tables: a
    // NoteOn outside kMinMidiNote..kMaxMidiNote. Reported separately because the fix differs -- this
    // one says the .mid file asks for a note the instrument was never designed to sound.
    std::uint32_t outOfRangeNotes = 0;
    // ...and the fourth: a NoteOff whose string had left the active count or been muted, which
    // StringNetwork would have discarded. This tool never automates numStrings or stringEnabled, so
    // a non-zero reading here means something in the chain moved a count the render did not ask to
    // move -- which is exactly why it is reported and failed on rather than merely available.
    std::uint32_t unaddressableNoteOffs = 0;
};

// MIDI status nibbles this layer reads directly, exactly as plugin/src/PluginProcessor.cpp does:
// the pitch wheel is not a note event, so it never goes through NoteAllocator.
constexpr std::uint8_t kStatusTypeMask = 0xF0u;
constexpr std::uint8_t kPitchWheelStatus = 0xE0u;

int pitchWheelValue(std::uint8_t data1, std::uint8_t data2) noexcept {
    return (static_cast<int>(data2) << 7) | static_cast<int>(data1);
}

// The negative control for carry-forward C2 and for the [contract] gate that pins it: a port that
// IS Task P2.4's bridge. Every call forwards to a real BridgeJunction -- it scatters, stores and
// dissipates identically -- and exactly one method is overridden to report 0, which is the P2.7
// compensation removed and nothing else. The same construction lives in
// tests/dsp/TuningAccuracyTests.cpp for the same reason; it is duplicated here rather than shared
// because cnpg_render links cnpg_dsp only and that file is a Catch2 translation unit.
class PhaseBlindBridgePort final : public cnpg::dsp::IBridgePort<cnpg::dsp::Sample> {
  public:
    void prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) override {
        inner_.prepare(sampleRate, maxBlockSize, numPorts, portImpedances);
    }
    void reset() noexcept override { inner_.reset(); }
    void scatter(const cnpg::dsp::Sample* incident, cnpg::dsp::Sample* outgoing, int numPorts) noexcept override {
        inner_.scatter(incident, outgoing, numPorts);
    }
    cnpg::dsp::Sample bridgeOutput() const noexcept override { return inner_.bridgeOutput(); }
    void setLossBypassed(bool bypass) noexcept override { inner_.setLossBypassed(bypass); }
    void setAdmittance(const cnpg::dsp::BridgeAdmittanceParams& p) noexcept override { inner_.setAdmittance(p); }
    bool isQuiescent() const noexcept override { return inner_.isQuiescent(); }
    cnpg::dsp::Sample64 storageEnergy() const noexcept override { return inner_.storageEnergy(); }

    // THE ONE DIFFERENCE.
    double reflectionPhaseDelaySamples(int portIndex, double frequencyHz, int numPorts) const noexcept override {
        (void)portIndex;
        (void)frequencyHz;
        (void)numPorts;
        return 0.0;
    }

  private:
    cnpg::dsp::BridgeJunction<cnpg::dsp::Sample> inner_;
};

// Renders one phrase into `samples`. Everything it touches is constructed fresh here, so calling it
// twice in a row with the same arguments is exactly the in-process determinism check
// --verify-determinism performs.
void renderPhrase(const MidiFileContents& midi, const RenderSpec& spec, Automation& automation, long long totalSamples,
                  std::vector<float>& samples, RenderStats& stats) {
    samples.assign(static_cast<std::size_t>(totalSamples), 0.0f);
    stats = RenderStats{};
    stats.numSamples = totalSamples;
    stats.durationSeconds = static_cast<double>(totalSamples) / spec.sampleRate;
    stats.midiEvents = static_cast<long long>(midi.events.size());

    automation.rewind();

    const int numStrings = std::clamp(spec.numStrings, 1, cnpg::dsp::kMaxStrings);

    cnpg::test::P1Chain chain;
    PhaseBlindBridgePort blindPort;
    chain.prepare(spec.sampleRate, spec.blockSize, numStrings, cnpg::dsp::Oversampler::kDefaultFactor);
    if (spec.bridgePhaseBlind)
        chain.network.setBridgePort(blindPort);
    chain.reset();

    cnpg::dsp::NoteAllocator allocator;
    // The CAPACITY, exactly as PluginProcessor::prepareToPlay() does it: prepare() is
    // message-thread-only and the count is a realtime parameter, so the count travels on
    // NoteAllocatorParams::activeStringCount below and is re-applied every block.
    allocator.prepare(cnpg::dsp::kMaxStrings);

    cnpg::dsp::NoteAllocatorParams allocatorParams;
    allocatorParams.mode = spec.allocationMode;
    allocatorParams.activeStringCount = numStrings;
    if (spec.allocationMode == cnpg::dsp::AllocationMode::FreeZones) {
        // FULL-RANGE ZONES, stated explicitly rather than inherited (Task P2.6). The allocator's
        // own FreeZones default is the fingering table, which spans MIDI 40..88 on six strings and
        // would silently drop every corpus note outside it -- phrase 01 is a chromatic sweep from
        // MIDI 21. A full-range table is what "put it on a string" means for a phrase whose
        // manifest entry does not name an allocation mode; see PhraseEntry for why that default is
        // the P1 one.
        for (auto& zone : allocatorParams.zones)
            zone = cnpg::dsp::StringZone{0, 127};
    }
    allocator.setParams(allocatorParams);

    cnpg::test::P1ChainParams baseParams = cnpg::test::makeDefaultP1ChainParams();
    baseParams.network.retriggerMode = spec.retriggerMode;
    baseParams.numStrings = numStrings;
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
        for (const AutomationLane& lane : automation.continuous)
            applyParam(blockParams, lane.param, static_cast<float>(lane.valueAt(blockStartSeconds)));
        blockParams.network.pitchBendSemitones = pitchBendSemitones;

        // The allocator is retargeted from the same block's values, immediately before it is used,
        // exactly as PluginProcessor::renderChunk() does it -- so a numStrings lane moves the count
        // the allocator assigns against and the count StringNetwork accepts events for on the same
        // block, and the two cannot disagree about which strings a note may land on.
        if (blockParams.numStrings != allocatorParams.activeStringCount) {
            allocatorParams.activeStringCount = blockParams.numStrings;
            allocator.setParams(allocatorParams);
        }

        noteEvents.clear();
        allocator.allocate(rawEvents.data(), static_cast<int>(rawEvents.size()), noteEvents);
        stats.droppedNoteEvents += noteEvents.droppedCount();
        stats.unassignableNotes = allocator.unassignableNoteCount();         // cumulative; not a per-block sum
        stats.outOfRangeNotes = allocator.outOfRangeNoteCount();             // cumulative; not a per-block sum
        stats.unaddressableNoteOffs = allocator.unaddressableNoteOffCount(); // cumulative, likewise

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
    std::vector<double> sampleRates{48000.0};
    int blockSize = 128;
    bool verifyDeterminism = false;
    bool variants = false;
    bool bridgePhaseBlind = false;
    // --rates was given (as opposed to --samplerate or the default). It selects the OUTPUT LAYOUT,
    // and that is not a cosmetic difference: a render filename carries the phrase, the variant, the
    // corpus version and the source digest, and deliberately NOT the sample rate
    // (tests/corpus/README.md rule 3), so three rates written into one directory would be three
    // renders with one name and only the last would survive. Measured on the first cut of this
    // flag: `--rates 44100,48000,96000` produced 8 files, all of them 96 kHz. --rates therefore
    // writes <out>/<rate>/, one subdirectory per rate, and --samplerate keeps the flat layout the
    // P1 command and tests/dsp/RenderTests.cpp already use.
    bool perRateDirectories = false;
};

void printUsage(std::FILE* stream) {
    std::fprintf(stream,
                 "usage: cnpg_render --midi <file> --out <wav> [--sidecar <json>] [--samplerate R] "
                 "[--blocksize B] [--verify-determinism] [--bridge-phase-blind]\n"
                 "       cnpg_render --corpus <dir> --out <dir> [--rates R1,R2,...] [--blocksize B] "
                 "[--verify-determinism] [--variants] [--bridge-phase-blind]\n"
                 "  --midi <file>          one Standard MIDI File to render\n"
                 "  --corpus <dir>         a corpus directory containing corpus.json; renders every phrase\n"
                 "  --out <path>           output WAV file (--midi mode) or output directory (--corpus mode)\n"
                 "  --sidecar <json>       automation sidecar for --midi mode (--corpus reads it from the manifest)\n"
                 "  --samplerate R         render sample rate in Hz (default 48000); one rate\n"
                 "  --rates R1,R2,...      render at every listed rate (--corpus mode), into <out>/<rate>/ --\n"
                 "                         the filename carries no rate, so one directory per rate\n"
                 "  --blocksize B          render block size in samples (default 128)\n"
                 "  --verify-determinism   render each phrase twice in-process and require bit-identical output\n"
                 "  --variants             also render the built-in P2 comparison configurations (Task P2.8)\n"
                 "  --bridge-phase-blind   attach a bridge port that reports zero reflection phase delay --\n"
                 "                         Task P2.4's instrument, the negative control for P2.7's compensation\n");
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
            double rate = 0.0;
            if (value == nullptr || !parseDouble(value, rate)) {
                std::fprintf(stderr, "cnpg_render: invalid --samplerate value\n");
                return false;
            }
            args.sampleRates.assign(1, rate);
        } else if (arg == "--rates") {
            const char* value = nextValue("--rates");
            if (value == nullptr)
                return false;
            std::vector<double> rates;
            const std::string list = value;
            std::size_t start = 0;
            while (start <= list.size()) {
                const std::size_t comma = list.find(',', start);
                const std::string field =
                    list.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
                double rate = 0.0;
                if (field.empty() || !parseDouble(field.c_str(), rate)) {
                    std::fprintf(stderr, "cnpg_render: invalid --rates value '%s'\n", field.c_str());
                    return false;
                }
                rates.push_back(rate);
                if (comma == std::string::npos)
                    break;
                start = comma + 1;
            }
            args.sampleRates = std::move(rates);
            args.perRateDirectories = true;
        } else if (arg == "--variants") {
            args.variants = true;
        } else if (arg == "--bridge-phase-blind") {
            args.bridgePhaseBlind = true;
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

    Automation automation;
    if (!spec.sidecarPath.empty() && !parseAutomation(spec.sidecarPath, automation, error)) {
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
    renderPhrase(midi, spec, automation, totalSamples, samples, stats);

    if (verifyDeterminism) {
        std::vector<float> second;
        RenderStats secondStats;
        renderPhrase(midi, spec, automation, totalSamples, second, secondStats);
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

    // THE GAIN-STRUCTURE LINE (Task P2.8's acceptance criterion "rendered WAVs peak below the
    // SoftClipLimiter ceiling on default settings"). The ceiling in force is the one this render
    // actually ran with -- a phrase or a variant may automate limiterCeilingDb, and comparing
    // against the struct default would then be comparing against a number nothing enforced.
    // Reported as HEADROOM rather than as a bare peak, because "below the ceiling" is a distance
    // and a listening note that records the peak alone cannot say how close it came.
    cnpg::test::P1ChainParams ceilingParams = cnpg::test::makeDefaultP1ChainParams();
    for (const auto& setting : spec.paramOverrides)
        applyParam(ceilingParams, setting.first, static_cast<float>(setting.second));
    double ceilingDb = static_cast<double>(ceilingParams.limiter.ceilingDb);
    for (const AutomationLane& lane : automation.continuous)
        if (lane.param == AutomatableParam::LimiterCeilingDb)
            for (const Breakpoint& breakpoint : lane.breakpoints)
                ceilingDb = std::max(ceilingDb, breakpoint.value);

    std::printf("  %-46s -> %s\n", label.c_str(), outputPath.string().c_str());
    std::printf("    %lld samples (%.3f s), %lld MIDI event(s), peak %.2f dBFS, rms %.2f dBFS, dc %.2f dBFS\n",
                stats.numSamples, stats.durationSeconds, stats.midiEvents, dbOf(stats.peak), dbOf(stats.rms),
                dbOf(std::fabs(stats.dcOffset)));
    std::printf("    limiter ceiling %.2f dBFS, headroom %.2f dB\n", ceilingDb, ceilingDb - dbOf(stats.peak));
    std::printf("    nonFinite=%lld subnormal=%lld droppedNoteEvents=%u unassignableNotes=%u outOfRangeNotes=%u "
                "unaddressableNoteOffs=%u%s\n",
                stats.nonFiniteSamples, stats.subnormalSamples, stats.droppedNoteEvents, stats.unassignableNotes,
                stats.outOfRangeNotes, stats.unaddressableNoteOffs,
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
    if (stats.unassignableNotes > 0) {
        std::fprintf(stderr,
                     "cnpg_render: %s left %u note(s) unassigned -- NoteAllocator had no string that could play "
                     "them, so the render is missing notes the phrase contains\n",
                     label.c_str(), stats.unassignableNotes);
        return false;
    }
    if (stats.outOfRangeNotes > 0) {
        std::fprintf(stderr,
                     "cnpg_render: %s contains %u note(s) outside the instrument's MIDI %d..%d design envelope, "
                     "which were rejected before allocation -- the render is missing notes the phrase contains\n",
                     label.c_str(), stats.outOfRangeNotes, cnpg::dsp::kMinMidiNote, cnpg::dsp::kMaxMidiNote);
        return false;
    }
    if (stats.unaddressableNoteOffs > 0) {
        std::fprintf(stderr,
                     "cnpg_render: %s left %u note-off(s) undeliverable -- the owning string had left the active "
                     "count or been disabled, so StringNetwork would have discarded them\n",
                     label.c_str(), stats.unaddressableNoteOffs);
        return false;
    }
    // The limiter is hard-wired LAST in the chain and its ceiling is a horizontal asymptote
    // (SoftClipLimiter.h), so a peak above it is not a loud render -- it is the limiter having been
    // bypassed, mis-ordered, or fed a non-finite sample the scan above did not reach. Failed on
    // rather than merely printed for that reason. The 0.01 dB slack absorbs the float32 round trip
    // through the WAV's own sample values, nothing more.
    if (dbOf(stats.peak) > ceilingDb + 0.01) {
        std::fprintf(stderr,
                     "cnpg_render: %s peaks at %.4f dBFS, ABOVE the %.4f dBFS SoftClipLimiter ceiling that ran "
                     "last in its own chain\n",
                     label.c_str(), dbOf(stats.peak), ceilingDb);
        return false;
    }

    return true;
}

// The digest stamped into filenames and printed in the banner, resolved ONCE per process because it
// walks the source tree (Task P2.7, carry-forward C2). "unknown" if the tree cannot be read -- a
// source tarball with no checkout around it -- which is the same fallback the configure-time hash it
// replaces had, and it never fails a render.
const std::string& renderSourceDigest() {
    static const std::string digest = [] {
        const std::string full = cnpg::test::renderSourceHash();
        return full.empty() ? std::string("unknown") : full.substr(0, 12);
    }();
    return digest;
}

// Render filenames in --corpus mode: "<phrase stem>__cv<corpusVersion>_s<source hash>.wav"
// (docs/plan.md section 4.8, as amended by this task's carry-forward C2: "Render filenames embed
// corpus version + a render-time content hash so listening notes are attributable").
//
// *** THE FIELD IS A CONTENT HASH, NOT A COMMIT, AND THE PREFIX IS `s` RATHER THAN `g` SO NOBODY
// READS IT AS ONE (Task P2.7, carry-forward C2). *** It used to be a configure-time
// `git rev-parse --short HEAD`, which is resolved when CMake last ran and not when the binary was
// built or run: a build/ tree configured at 77b0430 produced renders from the code at 1ccfcb1 and
// filed them as `..._cv1_g77b0430.wav`, so TWO DIFFERENT CODE STATES PRODUCED IDENTICAL FILENAMES --
// exactly the confusion this field exists to prevent, discovered in the task before the listening
// pass that depends on it. The digest is computed at RENDER time over dsp/include, dsp/src,
// tests/support/P1Chain.h and tests/render, so it names the bytes that produced the audio and is
// verifiable from any checkout with no repository history at all. See tests/support/SourceHash.h.
std::string renderFileName(const std::string& midiFileName, long long corpusVersion, const std::string& variant) {
    std::string stem = midiFileName;
    const auto dot = stem.find_last_of('.');
    if (dot != std::string::npos)
        stem.erase(dot);
    if (!variant.empty())
        stem += "__" + variant;
    // THE `__cv<n>_s<hash>` SUFFIX STAYS TERMINAL, and a variant name goes in FRONT of it. The
    // procedure a person follows (docs/listening/physical-plausibility-checklist.md step 4) reads
    // the corpus version and the source digest off the END of a filename; putting the variant name
    // after them would break the one thing that makes a listening note attributable.
    return stem + "__cv" + std::to_string(corpusVersion) + "_s" + renderSourceDigest() + ".wav";
}

// -------------------------------------------------------------------------------------------
// The built-in P2 comparison configurations (Task P2.8).
// -------------------------------------------------------------------------------------------
//
// RENDER CONFIGURATIONS, NOT CORPUS PHRASES -- docs/plan.md's P2.8 file list says so in as many
// words about the per-retriggerMode pair, and the same reasoning covers the rest: a comparison is a
// set of parameter settings over material that already exists, and adding a MIDI file per setting
// would bump the corpus version for something that is not new material
// (tests/corpus/README.md rule 2).
//
// They live here, in the tool, rather than in a JSON file beside the corpus, for the same reason:
// tests/corpus/ is the append-only record of what is PLAYED, and these are statements about how it
// is rendered. A variant inherits everything from its phrase's manifest entry and overrides only
// what it names.
struct VariantOverride {
    const char* name;
    double value;
};

struct RenderVariant {
    const char* phraseFile;
    const char* suffix;        // goes into the filename, ahead of the __cv<n>_s<hash> tail
    const char* purpose;       // one line, printed with the render and quoted in the listening document
    const char* retriggerMode; // nullptr = inherit the phrase's own
    VariantOverride overrides[6] = {{nullptr, 0.0}, {nullptr, 0.0}, {nullptr, 0.0},
                                    {nullptr, 0.0}, {nullptr, 0.0}, {nullptr, 0.0}};
};

// The near-unison pair, spelled once. Phrase 02's section C plays MIDI 45 (which GuitarFingering
// puts on string 1, fret 0) and MIDI 46 (string 0, fret 6). The two offsets below bring those two
// strings to 111.60 Hz and 113.22 Hz -- 25.00 cents apart, the separation ADR 0007 D7.0 measured
// collapsing to 0.003 cents at coupling 0.35.
//
// WHY THE MIDI IS A SEMITONE AND THE INSTRUMENT MAKES THE UNISON. NoteAllocator's invariant is that
// at most one string owns a given (channel, note), so two simultaneous NoteOns for the SAME note on
// one channel are a retrigger of one string, not two strings at one pitch -- and a guitar unison is
// always the same MIDI note. The two ways out are per-note channels (which is the P5 MPE seam, and
// which a host that rewrites the channel on a clip would silently undo, so the phrase would not
// survive the live replay the checklist asks for) or the instrument's own per-string tuning
// offsets, which are shipped +/-50-cent APVTS parameters. This uses the second. The MIDI reads as a
// minor second; what SOUNDS is a 25-cent pair, on strings 0 and 1, which is the configuration under
// test.
#define CNPG_NEAR_UNISON_OFFSETS {"stringTuningOffsetCents0", -50.0}, {"stringTuningOffsetCents1", 25.0}

constexpr RenderVariant kRenderVariants[] = {
    // ---- the per-RetriggerMode pair over phrase 03 (docs/plan.md P2.8 file list) --------------
    {"03_legato_retrigger.mid",
     "retrigPhysical",
     "checklist 3/17/27 -- Physical: the old note continues into the new one over a 30 ms glide",
     "Physical",
     {}},
    {"03_legato_retrigger.mid",
     "retrigSynth",
     "checklist 4/27 -- Synth: fade, clear, instant restart at the new pitch, no trace of the old",
     "Synth",
     {}},

    // ---- the couplingStrength ladder over phrase 02 (ADR 0007 D4) ----------------------------
    // Six values, and each one is a MEASURED boundary rather than a round number:
    //   0.00  the decoupled control. Without it "less coupling" has no zero and the beat-depth
    //         claim has no baseline; it is also bridgeOutput() == 0 by construction.
    //   0.10  ADR 0007 D4's own low anchor: beat depth 10.08 dB, against 3.59 at 0.35.
    //   0.20  the highest value D7.0 measured NOT to mode-lock at the sustain material
    //         (25.045 cents of separation survive).
    //   0.30  where the two string materials DISAGREE: 0.003 cents at the sustain material,
    //         25.14 cents at the default one. The most informative single point in the set.
    //   0.32  the highest DEFAULT-material value measured not to lock (25.18 cents).
    //   0.35  the value that shipped as the default through 2026-08-04, which locks at both
    //         materials (0.33 cents at the default material, a +24.67-cent pull). It is ALSO
    //         kBridgeNormalCouplingMax, and it is no longer the default: on 2026-08-05 the default
    //         moved to 0.20 -- the 0.20 rung above -- by author delegation (ADR 0007 D7.2). The
    //         ladder is unchanged, because what it exists to compare is the slider, not the default.
    {"02_open_chords.mid",
     "coupling000",
     "coupling 0.00 -- the decoupled control",
     nullptr,
     {{"bridgeCoupling", 0.00}}},
    {"02_open_chords.mid", "coupling010", "coupling 0.10 -- beat depth 10.08 dB", nullptr, {{"bridgeCoupling", 0.10}}},
    {"02_open_chords.mid",
     "coupling020",
     "coupling 0.20 -- highest measured NON-locking value",
     nullptr,
     {{"bridgeCoupling", 0.20}}},
    {"02_open_chords.mid",
     "coupling030",
     "coupling 0.30 -- the two string materials disagree here",
     nullptr,
     {{"bridgeCoupling", 0.30}}},
    {"02_open_chords.mid",
     "coupling032",
     "coupling 0.32 -- highest non-locking at the DEFAULT material",
     nullptr,
     {{"bridgeCoupling", 0.32}}},
    {"02_open_chords.mid",
     "coupling035",
     "coupling 0.35 -- the Normal-range ceiling and the pre-2026-08-05 default; locks at both materials",
     nullptr,
     {{"bridgeCoupling", 0.35}}},

    // ---- the same ladder with the near-unison pair detuned (ADR 0007 D5 criterion 4) ----------
    {"02_open_chords.mid",
     "unison000",
     "25-cent pair, coupling 0.00 -- what NOT locking sounds like",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.00}}},
    {"02_open_chords.mid",
     "unison010",
     "25-cent pair, coupling 0.10",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.10}}},
    {"02_open_chords.mid",
     "unison020",
     "25-cent pair, coupling 0.20 -- 25.045 cents survive",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.20}}},
    {"02_open_chords.mid",
     "unison030",
     "25-cent pair, coupling 0.30 -- the material-dependent boundary",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.30}}},
    {"02_open_chords.mid",
     "unison032",
     "25-cent pair, coupling 0.32",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.32}}},
    {"02_open_chords.mid",
     "unison035",
     "25-cent pair, coupling 0.35 -- separation collapses to 0.003 cents",
     nullptr,
     {CNPG_NEAR_UNISON_OFFSETS, {"bridgeCoupling", 0.35}}},

    // ---- at and outside the provisional Normal range (ADR 0007 D7/D7.0, criterion 5) ---------
    // Criterion (5) is "the bridge still behaves as an INSTRUMENT COMPONENT rather than an overt
    // resonant effect", and it cannot be judged from inside the box alone: a boundary is only a
    // boundary if what is on the other side of it sounds different.
    {"02_open_chords.mid",
     "rangeResCeiling",
     "resonance 330 Hz / damping 0.15 / coupling 0.35 -- the worst corner INSIDE the box (0.770 cents)",
     nullptr,
     {{"bridgeResonanceHz", 330.0}, {"bridgeDamping", 0.15}, {"bridgeCoupling", 0.35}}},
    {"02_open_chords.mid",
     "rangeResOutside",
     "resonance 500 Hz -- OUTSIDE the box, no tuning guarantee",
     nullptr,
     {{"bridgeResonanceHz", 500.0}, {"bridgeDamping", 0.15}, {"bridgeCoupling", 0.35}}},
    {"02_open_chords.mid",
     "rangeDampCeiling",
     "damping 1.00 -- the box's damping ceiling",
     nullptr,
     {{"bridgeDamping", 1.00}}},
    {"02_open_chords.mid",
     "rangeDampOutside",
     "damping 4.00 -- the slider stop, OUTSIDE the box; the worst note migrates to the TOP",
     nullptr,
     {{"bridgeDamping", 4.00}}},
    {"02_open_chords.mid",
     "rangeCouplingOutside",
     "coupling 1.00 -- the slider stop, OUTSIDE the box; -14.06..+13.41 cents uncompensated",
     nullptr,
     {{"bridgeCoupling", 1.00}}},
};

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
    if (args.sampleRates.empty()) {
        std::fprintf(stderr, "cnpg_render: no sample rate requested\n");
        return 1;
    }
    for (const double rate : args.sampleRates) {
        if (!(rate > 0.0) || rate > 500000.0) {
            std::fprintf(stderr, "cnpg_render: every rate must be a positive, finite value\n");
            return 1;
        }
    }
    if (args.sampleRates.size() > 1 && !args.midi.empty()) {
        std::fprintf(stderr, "cnpg_render: --rates lists more than one rate, which needs --corpus "
                             "(--midi mode writes one named file and cannot name several)\n");
        return 1;
    }
    if (args.variants && args.corpus.empty()) {
        std::fprintf(stderr, "cnpg_render: --variants applies to --corpus mode only\n");
        return 1;
    }
    if (args.blockSize < 1 || args.blockSize > 1000000) {
        std::fprintf(stderr, "cnpg_render: --blocksize must be within 1..1000000\n");
        return 1;
    }

    std::printf("cnpg_render -- %d-sample blocks, source %s%s\n", args.blockSize, renderSourceDigest().c_str(),
                args.bridgePhaseBlind ? ", BRIDGE PHASE-BLIND (the P2.4 negative control, NOT the instrument)" : "");
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
        spec.sampleRate = args.sampleRates.front();
        spec.blockSize = args.blockSize;
        spec.bridgePhaseBlind = args.bridgePhaseBlind;

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

    // The base configuration of one phrase, before any variant is layered on it.
    const auto specForPhrase = [&](const PhraseEntry& phrase, double sampleRate) {
        RenderSpec spec;
        spec.midiPath = corpusDir / phrase.file;
        spec.sidecarPath = phrase.sidecar.empty() ? fs::path{} : corpusDir / phrase.sidecar;
        spec.retriggerMode = phrase.retriggerMode;
        spec.numStrings = phrase.numStrings;
        spec.allocationMode = phrase.allocationMode;
        spec.paramOverrides = phrase.paramOverrides;
        spec.bridgePhaseBlind = args.bridgePhaseBlind;
        spec.sampleRate = sampleRate;
        spec.blockSize = args.blockSize;
        spec.expectedDurationSeconds = phrase.durationSeconds;
        return spec;
    };

    for (const double sampleRate : args.sampleRates) {
        // See RenderArgs::perRateDirectories: the filename does not carry the rate, so several rates
        // need several directories or they overwrite each other.
        char rateName[32];
        std::snprintf(rateName, sizeof(rateName), "%.0f", sampleRate);
        const fs::path rateDir = args.perRateDirectories ? outputDir / rateName : outputDir;
        std::error_code directoryError;
        fs::create_directories(rateDir, directoryError);

        std::printf("  --- %.0f Hz -> %s ---\n", sampleRate, rateDir.string().c_str());
        std::fflush(stdout);

        for (const PhraseEntry& phrase : manifest.phrases) {
            const RenderSpec spec = specForPhrase(phrase, sampleRate);
            const fs::path outputPath = rateDir / renderFileName(phrase.file, manifest.corpusVersion, std::string{});
            const std::string label = phrase.file + " [" + std::to_string(phrase.numStrings) + " string(s), " +
                                      phrase.allocationModeName + ", " + phrase.retriggerModeName + "]";
            if (!renderOne(spec, outputPath, args.verifyDeterminism, label))
                return 1;
        }

        if (!args.variants)
            continue;

        for (const RenderVariant& variant : kRenderVariants) {
            const PhraseEntry* phrase = nullptr;
            for (const PhraseEntry& candidate : manifest.phrases)
                if (candidate.file == variant.phraseFile)
                    phrase = &candidate;
            if (phrase == nullptr) {
                // A variant naming a phrase the manifest does not carry is a BUILD-TIME mistake that
                // would otherwise show up as a missing WAV nobody looked for -- so it fails the run.
                std::fprintf(stderr, "cnpg_render: variant '%s' names phrase '%s', which is not in %s\n",
                             variant.suffix, variant.phraseFile, manifestPath.string().c_str());
                return 1;
            }

            RenderSpec spec = specForPhrase(*phrase, sampleRate);
            if (variant.retriggerMode != nullptr)
                spec.retriggerMode = (std::string(variant.retriggerMode) == "Synth")
                                         ? cnpg::dsp::RetriggerMode::Synth
                                         : cnpg::dsp::RetriggerMode::Physical;
            for (const VariantOverride& setting : variant.overrides) {
                if (setting.name == nullptr)
                    continue;
                AutomatableParam which{};
                if (!lookupParamName(setting.name, which)) {
                    std::fprintf(stderr, "cnpg_render: variant '%s' sets unknown parameter '%s'\n", variant.suffix,
                                 setting.name);
                    return 1;
                }
                spec.paramOverrides.emplace_back(which, setting.value);
            }

            const fs::path outputPath = rateDir / renderFileName(phrase->file, manifest.corpusVersion, variant.suffix);
            const std::string label = std::string(variant.suffix) + ": " + variant.purpose;
            if (!renderOne(spec, outputPath, args.verifyDeterminism, label))
                return 1;
        }
    }

    return 0;
}

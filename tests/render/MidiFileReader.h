#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

// MidiFileReader -- a minimal, JUCE-free Standard MIDI File parser, sized for exactly one job:
// reading tests/corpus/*.mid for cnpg_render (docs/plan.md section 4.8, Task P1.11). It is
// deliberately NOT a general SMF library. docs/plan.md section 1.4's locked structure keeps JUCE
// out of the whole headless domain, so "just use juce::MidiFile" is not available here; and the
// corpus is a small, in-repo, append-only set of files this project authors itself
// (tests/corpus/README.md), so the parser only has to cover what those files actually contain --
// plus enough of the standard to fail loudly, with a readable message, on anything else.
//
// SUPPORTED: header chunk (MThd) with format 0 or 1 and metrical division (ticks per quarter
// note); any number of tracks, merged into one timeline; channel voice messages (note off/on,
// polyphonic aftertouch, control change, program change, channel pressure, pitch wheel), including
// running status; the set-tempo meta event (FF 51 03), applied as a full tempo MAP rather than a
// single leading value; all other meta events and both SysEx forms, skipped correctly by their
// declared length; unknown non-MThd/MTrk top-level chunks, skipped by their declared length (the
// standard requires readers to do this).
//
// REJECTED, with an explicit error rather than a guess: SMPTE (negative) division -- the corpus is
// authored in metrical time and a wrong-by-a-factor timeline is far worse than a refusal; format 2
// (independent, non-simultaneous sequences, which have no single merged timeline to render);
// truncated or malformed chunks; a running-status data byte with no preceding status.
//
// TIMING. Events come out stamped in SAMPLES at a caller-supplied sample rate, not in ticks or
// seconds: the renderer's whole timeline is sample-indexed, and doing the conversion once, here,
// against the tempo map is what keeps RenderMain.cpp free of tick arithmetic. The conversion is
// pure integer/double arithmetic over the file's own bytes -- no wall-clock read, no RNG -- so the
// same file at the same rate always yields the identical event stream, which is one half of
// cnpg_render's byte-identical-render contract (the other half being the dsp/ chain's own
// determinism).

namespace cnpg::render {

// One channel voice message, resolved onto the render timeline. Deliberately the same
// (status, data1, data2, sampleOffset) tuple shape docs/plan.md section 2.14 locks for the
// plugin's MIDI seam, so RenderMain.cpp feeds cnpg::dsp::translateRawMidi()/NoteAllocator exactly
// what plugin/src/MidiConverter.cpp feeds them -- with `sample` rebased per block.
struct MidiFileEvent {
    long long sample;    // absolute sample index from the start of the file, at the requested rate
    std::uint8_t status; // channel voice status byte, running status already resolved
    std::uint8_t data1;
    std::uint8_t data2; // 0 for one-data-byte messages (program change, channel pressure)
};

struct MidiFileContents {
    // Non-decreasing in `sample`. Simultaneous events keep their file order: track order first,
    // then position within the track (a stable sort over the merged list), so a note-off and the
    // note-on that replaces it at the same tick stay in the order the file author wrote them.
    std::vector<MidiFileEvent> events;

    int format = 0;                // 0 or 1
    int numTracks = 0;             // as declared by MThd (and as actually parsed -- they must match)
    int ticksPerQuarter = 0;       // metrical division
    long long lastEventSample = 0; // 0 if there are no channel voice events at all
    double lastEventSeconds = 0.0;
};

// Reads `path` and resolves every channel voice message onto a sample timeline at `sampleRate`.
// Returns false and fills `error` with a human-readable, path-prefixed message on any failure
// (missing file, malformed chunk, unsupported division/format); `out` is left unspecified in that
// case. Never throws for malformed input -- a corpus file that will not parse is a diagnostic, not
// an exception.
bool readMidiFile(const std::filesystem::path& path, double sampleRate, MidiFileContents& out, std::string& error);

} // namespace cnpg::render

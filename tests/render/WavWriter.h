#pragma once

#include <filesystem>
#include <string>
#include <vector>

// WavWriter -- a minimal, JUCE-free RIFF/WAVE writer for exactly the one format cnpg_render
// produces: mono, 32-bit IEEE float, dither-free (docs/plan.md section 4.8, "Renders are
// single-threaded, float32, dither-free 32-bit-float WAV"). Same reasoning as MidiFileReader.h:
// JUCE is structurally absent from the headless domain, and pulling a WAV library in for a
// 44-byte header plus a memcpy would be the wrong trade.
//
// Float, not 16/24-bit integer, and that choice is load-bearing rather than a convenience:
//
//   - Dither-free is only meaningful for a float file. Writing float32 samples to a 16- or 24-bit
//     integer file requires a quantization decision, and every honest one of those adds noise the
//     listening pass would then be judging alongside the instrument. A 32-bit float file stores
//     the chain's own samples exactly -- what the author hears is the renderer's output, not a
//     re-quantization of it.
//   - Byte-identical re-renders (the [contract] determinism smoke, RenderTests.cpp) are a claim
//     about the RENDERER. An integer conversion step in between would be one more place for a
//     difference to hide or, worse, to be rounded away.
//   - The header carries no timestamp, no software/creation-date tag, and no LIST/INFO chunk, so
//     two renders of the same input produce two byte-identical FILES, not merely two identical
//     sample streams.
//
// The chunk layout matches what libsndfile writes for float WAV (and therefore what every DAW and
// editor on the listening path already reads without complaint): an 18-byte `fmt ` chunk carrying
// WAVE_FORMAT_IEEE_FLOAT with an explicit cbSize of 0, the `fact` chunk the standard requires for
// every non-PCM format, then `data`. Everything is written byte by byte in explicit little-endian
// order rather than by memcpy'ing a struct, so the output is identical on any host regardless of
// its own endianness or struct padding.

namespace cnpg::render {

// Writes `samples` (interleaved if numChannels > 1; cnpg_render only ever writes mono) as a
// 32-bit float WAVE file at `path`, creating parent directories as needed. Returns false with a
// human-readable `error` on any failure -- an unwritable path, a non-finite sample rate, or a
// short write. Non-finite SAMPLE values are written through unchanged rather than being sanitized:
// this is the artifact a NaN scan runs over, so a writer that quietly cleaned them would defeat
// the test that exists to catch them.
bool writeWavFloat32(const std::filesystem::path& path, const std::vector<float>& samples, int numChannels,
                     double sampleRate, std::string& error);

} // namespace cnpg::render

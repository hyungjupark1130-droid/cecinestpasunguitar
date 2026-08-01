#include "WavWriter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <system_error>

namespace cnpg::render {

namespace {

constexpr std::uint16_t kWaveFormatIeeeFloat = 3;
constexpr std::uint16_t kBitsPerSample = 32;

void appendLe16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    const unsigned wide = value;
    out.push_back(static_cast<std::uint8_t>(wide & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((wide >> 8) & 0xFFu));
}

void appendLe32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFu));
}

void appendTag(std::vector<std::uint8_t>& out, const char (&tag)[5]) {
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>(tag[i]));
}

// IEEE-754 binary32 bit pattern, then four explicit little-endian bytes. std::memcpy (rather than
// a reinterpret_cast) is the defined way to read a float's object representation; doing the byte
// split by hand afterwards is what makes the output identical on a big-endian host too.
void appendFloatLe(std::vector<std::uint8_t>& out, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    appendLe32(out, bits);
}

} // namespace

bool writeWavFloat32(const std::filesystem::path& path, const std::vector<float>& samples, int numChannels,
                     double sampleRate, std::string& error) {
    error.clear();

    if (numChannels < 1) {
        error = path.string() + ": channel count must be at least 1";
        return false;
    }
    if (!(sampleRate > 0.0) || !std::isfinite(sampleRate)) {
        error = path.string() + ": sample rate must be positive and finite";
        return false;
    }

    const std::uint64_t dataBytes = static_cast<std::uint64_t>(samples.size()) * sizeof(float);
    // RIFF sizes are unsigned 32-bit; 4 GiB of float samples is ~6.2 hours of 48 kHz mono, far
    // past anything the corpus can produce, but a silent wrap here would write a corrupt file.
    if (dataBytes > 0xFFFFFFFFull - 64ull) {
        error = path.string() + ": render is too long for a 32-bit RIFF size field";
        return false;
    }

    const std::uint32_t rate = static_cast<std::uint32_t>(sampleRate + 0.5);
    const std::uint16_t channels = static_cast<std::uint16_t>(numChannels);
    const std::uint16_t blockAlign = static_cast<std::uint16_t>(channels * (kBitsPerSample / 8));
    const std::uint32_t byteRate = rate * blockAlign;
    const std::uint32_t frames = blockAlign == 0 ? 0u : static_cast<std::uint32_t>(dataBytes / blockAlign);

    // 4 ("WAVE") + (8 + 18) fmt + (8 + 4) fact + (8 + dataBytes) data
    const std::uint32_t riffSize = static_cast<std::uint32_t>(4 + 26 + 12 + 8 + dataBytes);

    std::vector<std::uint8_t> header;
    header.reserve(64);

    appendTag(header, "RIFF");
    appendLe32(header, riffSize);
    appendTag(header, "WAVE");

    appendTag(header, "fmt ");
    appendLe32(header, 18); // non-PCM formats carry the cbSize field, so 18 rather than 16
    appendLe16(header, kWaveFormatIeeeFloat);
    appendLe16(header, channels);
    appendLe32(header, rate);
    appendLe32(header, byteRate);
    appendLe16(header, blockAlign);
    appendLe16(header, kBitsPerSample);
    appendLe16(header, 0); // cbSize: no extension bytes follow

    appendTag(header, "fact");
    appendLe32(header, 4);
    appendLe32(header, frames);

    appendTag(header, "data");
    appendLe32(header, static_cast<std::uint32_t>(dataBytes));

    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec && !std::filesystem::is_directory(parent)) {
            error = path.string() + ": cannot create output directory (" + ec.message() + ")";
            return false;
        }
    }

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        error = path.string() + ": cannot open file for writing";
        return false;
    }

    file.write(reinterpret_cast<const char*>(header.data()), static_cast<std::streamsize>(header.size()));

    // Samples are converted in chunks rather than one std::ofstream::write per sample: the byte
    // layout is identical either way, this is simply the difference between one syscall per
    // 8192 samples and one per sample on a multi-minute render.
    constexpr std::size_t kChunkSamples = 8192;
    std::vector<std::uint8_t> chunk;
    chunk.reserve(kChunkSamples * sizeof(float));

    for (std::size_t i = 0; i < samples.size(); i += kChunkSamples) {
        chunk.clear();
        const std::size_t end = std::min(samples.size(), i + kChunkSamples);
        for (std::size_t n = i; n < end; ++n)
            appendFloatLe(chunk, samples[n]);
        file.write(reinterpret_cast<const char*>(chunk.data()), static_cast<std::streamsize>(chunk.size()));
    }

    file.flush();
    if (!file.good()) {
        error = path.string() + ": write failed";
        return false;
    }

    return true;
}

} // namespace cnpg::render

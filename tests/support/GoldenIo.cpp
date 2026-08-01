#include "support/GoldenIo.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#ifndef CNPG_GOLDEN_DIR
#error "CNPG_GOLDEN_DIR must be defined by tests/CMakeLists.txt"
#endif

namespace cnpg::test {

namespace {

std::string rateFolder(double sampleRate) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(sampleRate + 0.5));
    return std::string(buffer);
}

// Minimal flat-JSON value lookup: finds "key" and returns the raw text of its value.
bool findValue(const std::string& text, const std::string& key, std::string& out) {
    const std::string needle = "\"" + key + "\"";
    const std::size_t keyPos = text.find(needle);
    if (keyPos == std::string::npos)
        return false;
    std::size_t pos = text.find(':', keyPos + needle.size());
    if (pos == std::string::npos)
        return false;
    ++pos;
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n' || text[pos] == '\r' || text[pos] == '\t'))
        ++pos;
    if (pos >= text.size())
        return false;

    std::size_t end = pos;
    if (text[pos] == '"') {
        end = text.find('"', pos + 1);
        if (end == std::string::npos)
            return false;
        out = text.substr(pos + 1, end - pos - 1);
        return true;
    }
    if (text[pos] == '[') {
        end = text.find(']', pos);
        if (end == std::string::npos)
            return false;
        out = text.substr(pos + 1, end - pos - 1);
        return true;
    }
    while (end < text.size() && text[end] != ',' && text[end] != '\n' && text[end] != '}')
        ++end;
    out = text.substr(pos, end - pos);
    return true;
}

bool readDouble(const std::string& text, const std::string& key, double& out) {
    std::string raw;
    if (!findValue(text, key, raw))
        return false;
    try {
        out = std::stod(raw);
    } catch (...) {
        return false;
    }
    return true;
}

bool readArray(const std::string& text, const std::string& key, std::vector<double>& out) {
    std::string raw;
    if (!findValue(text, key, raw))
        return false;
    out.clear();
    std::stringstream stream(raw);
    std::string item;
    while (std::getline(stream, item, ',')) {
        try {
            out.push_back(std::stod(item));
        } catch (...) {
            return false;
        }
    }
    return true;
}

void writeArray(std::ostream& os, const char* key, const std::vector<double>& values, int precision) {
    os << "  \"" << key << "\": [";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0)
            os << ", ";
        char buffer[512];
        std::snprintf(buffer, sizeof(buffer), "%.*g", precision, values[i]);
        os << buffer;
    }
    os << "],\n";
}

} // namespace

std::filesystem::path goldenRoot() { return std::filesystem::path(CNPG_GOLDEN_DIR); }

std::filesystem::path goldenDirectory(cnpg::dsp::FractionalDelayKind kind, double sampleRate) {
    return goldenRoot() / "string_ir" / variantName(kind) / rateFolder(sampleRate);
}

std::filesystem::path goldenF64Path(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    return goldenDirectory(kind, sampleRate) / (scenarioFileName(midiNote) + ".f64");
}

std::filesystem::path goldenJsonPath(cnpg::dsp::FractionalDelayKind kind, double sampleRate, int midiNote) {
    return goldenDirectory(kind, sampleRate) / (scenarioFileName(midiNote) + ".json");
}

namespace {
std::filesystem::path chordDirectory(cnpg::dsp::FractionalDelayKind kind, double sampleRate) {
    return goldenRoot() / "chord_ir" / variantName(kind) / rateFolder(sampleRate);
}
std::string chordFileName(ChordIrChannel channel) { return std::string("open_e_major_") + chordChannelName(channel); }
} // namespace

std::filesystem::path chordGoldenF64Path(cnpg::dsp::FractionalDelayKind kind, double sampleRate,
                                         ChordIrChannel channel) {
    return chordDirectory(kind, sampleRate) / (chordFileName(channel) + ".f64");
}

std::filesystem::path chordGoldenJsonPath(cnpg::dsp::FractionalDelayKind kind, double sampleRate,
                                          ChordIrChannel channel) {
    return chordDirectory(kind, sampleRate) / (chordFileName(channel) + ".json");
}

bool readGoldenF64(const std::filesystem::path& path, std::vector<double>& out) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
        return false;
    file.seekg(0, std::ios::end);
    const auto bytes = static_cast<std::streamoff>(file.tellg());
    file.seekg(0, std::ios::beg);
    if (bytes <= 0 || (bytes % static_cast<std::streamoff>(sizeof(double))) != 0)
        return false;
    out.resize(static_cast<std::size_t>(bytes) / sizeof(double));
    file.read(reinterpret_cast<char*>(out.data()), bytes);
    return static_cast<bool>(file);
}

void writeGoldenF64(const std::filesystem::path& path, const std::vector<double>& samples) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(reinterpret_cast<const char*>(samples.data()),
               static_cast<std::streamsize>(samples.size() * sizeof(double)));
}

bool readGoldenSidecar(const std::filesystem::path& path, GoldenSidecar& out) {
    std::ifstream file(path);
    if (!file)
        return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    double value = 0.0;
    if (readDouble(text, "schemaVersion", value))
        out.schemaVersion = static_cast<int>(value);
    findValue(text, "dspSourceSha256", out.dspSourceSha256);
    findValue(text, "generatedUtc", out.generatedUtc);
    if (readDouble(text, "dspStateVersion", value))
        out.dspStateVersion = static_cast<int>(value);
    readDouble(text, "sampleRate", out.sampleRate);
    findValue(text, "variant", out.variant);
    if (readDouble(text, "midiNote", value))
        out.midiNote = static_cast<int>(value);
    readDouble(text, "excitationVelocity", out.excitationVelocity);
    readDouble(text, "excitationPluckPosition", out.excitationPluckPosition);
    readDouble(text, "excitationHardness", out.excitationHardness);
    readDouble(text, "excitationNoiseAmount", out.excitationNoiseAmount);
    readDouble(text, "noiseSeed", out.noiseSeed);
    readDouble(text, "tapPosition", out.tapPosition);
    readDouble(text, "lengthSamples", out.lengthSamples);
    readDouble(text, "atol", out.atol);
    readDouble(text, "attackRmsDbfs", out.features.attackRmsDbfs);
    readArray(text, "partialHz", out.features.partialHz);
    readArray(text, "bandT60Seconds", out.features.bandT60);
    return true;
}

void writeGoldenSidecar(const std::filesystem::path& path, const GoldenSidecar& sidecar) {
    std::filesystem::create_directories(path.parent_path());
    // std::ios::binary so the sidecar is byte-identical on every platform: .gitattributes
    // normalizes the repo to LF, and a text-mode stream on Windows would write CRLF into the
    // working copy on every regeneration.
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    file << "{\n";
    file << "  \"schemaVersion\": " << sidecar.schemaVersion << ",\n";
    file << "  \"dspSourceSha256\": \"" << sidecar.dspSourceSha256 << "\",\n";
    file << "  \"generatedUtc\": \"" << sidecar.generatedUtc << "\",\n";
    file << "  \"dspStateVersion\": " << sidecar.dspStateVersion << ",\n";
    file << "  \"sampleRate\": " << static_cast<long long>(sidecar.sampleRate) << ",\n";
    file << "  \"variant\": \"" << sidecar.variant << "\",\n";
    file << "  \"midiNote\": " << sidecar.midiNote << ",\n";
    file << "  \"excitationVelocity\": " << sidecar.excitationVelocity << ",\n";
    file << "  \"excitationPluckPosition\": " << sidecar.excitationPluckPosition << ",\n";
    file << "  \"excitationHardness\": " << sidecar.excitationHardness << ",\n";
    file << "  \"excitationNoiseAmount\": " << sidecar.excitationNoiseAmount << ",\n";
    file << "  \"noiseSeed\": " << static_cast<long long>(sidecar.noiseSeed) << ",\n";
    file << "  \"tapPosition\": " << sidecar.tapPosition << ",\n";
    file << "  \"lengthSamples\": " << static_cast<long long>(sidecar.lengthSamples) << ",\n";
    {
        char buffer[512];
        std::snprintf(buffer, sizeof(buffer), "%.1e", sidecar.atol);
        file << "  \"atol\": " << buffer << ",\n";
    }
    writeArray(file, "partialHz", sidecar.features.partialHz, 12);
    writeArray(file, "bandT60Seconds", sidecar.features.bandT60, 8);
    {
        char buffer[512];
        std::snprintf(buffer, sizeof(buffer), "%.8g", sidecar.features.attackRmsDbfs);
        file << "  \"attackRmsDbfs\": " << buffer << "\n";
    }
    file << "}\n";
}

} // namespace cnpg::test

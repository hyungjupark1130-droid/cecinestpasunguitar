#include "support/SourceHash.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#ifndef CNPG_SOURCE_DIR
#error "CNPG_SOURCE_DIR must be defined by tests/CMakeLists.txt"
#endif

namespace cnpg::test {

namespace {

// ---------------------------------------------------------------------------------------------
// SHA-256 (FIPS 180-4). Vendored rather than pulled in as a dependency: it is 60 lines, the test
// binary links nothing but cnpg_dsp and Catch2 by design, and "SHA-256" is exactly the kind of
// claim a reader must be able to check against an outside implementation -- which is what the
// published-vector assertions in the provenance test do.
// ---------------------------------------------------------------------------------------------

constexpr std::uint32_t kRoundConstants[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

std::uint32_t rotr(std::uint32_t value, int bits) noexcept { return (value >> bits) | (value << (32 - bits)); }

void compress(std::array<std::uint32_t, 8>& state, const unsigned char* block) noexcept {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i)
        w[i] = (static_cast<std::uint32_t>(block[i * 4]) << 24) | (static_cast<std::uint32_t>(block[i * 4 + 1]) << 16) |
               (static_cast<std::uint32_t>(block[i * 4 + 2]) << 8) | static_cast<std::uint32_t>(block[i * 4 + 3]);
    for (int i = 16; i < 64; ++i) {
        const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    std::array<std::uint32_t, 8> v = state;
    for (int i = 0; i < 64; ++i) {
        const std::uint32_t s1 = rotr(v[4], 6) ^ rotr(v[4], 11) ^ rotr(v[4], 25);
        const std::uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
        const std::uint32_t temp1 = v[7] + s1 + ch + kRoundConstants[i] + w[i];
        const std::uint32_t s0 = rotr(v[0], 2) ^ rotr(v[0], 13) ^ rotr(v[0], 22);
        const std::uint32_t maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
        const std::uint32_t temp2 = s0 + maj;

        v[7] = v[6];
        v[6] = v[5];
        v[5] = v[4];
        v[4] = v[3] + temp1;
        v[3] = v[2];
        v[2] = v[1];
        v[1] = v[0];
        v[0] = temp1 + temp2;
    }
    for (int i = 0; i < 8; ++i)
        state[static_cast<std::size_t>(i)] += v[static_cast<std::size_t>(i)];
}

} // namespace

std::string sha256Hex(const std::string& bytes) {
    std::array<std::uint32_t, 8> state{0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                       0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

    const auto* data = reinterpret_cast<const unsigned char*>(bytes.data());
    const std::size_t length = bytes.size();

    std::size_t offset = 0;
    for (; offset + 64 <= length; offset += 64)
        compress(state, data + offset);

    // Padding: 0x80, then zeros, then the message length in bits as a big-endian 64-bit integer.
    std::vector<unsigned char> tail(data + offset, data + length);
    tail.push_back(0x80);
    while ((tail.size() % 64) != 56)
        tail.push_back(0x00);
    const std::uint64_t bitLength = static_cast<std::uint64_t>(length) * 8u;
    for (int i = 7; i >= 0; --i)
        tail.push_back(static_cast<unsigned char>((bitLength >> (i * 8)) & 0xFFu));
    for (std::size_t i = 0; i < tail.size(); i += 64)
        compress(state, tail.data() + i);

    std::string hex;
    hex.reserve(64);
    for (std::uint32_t word : state) {
        char buffer[16];
        std::snprintf(buffer, sizeof(buffer), "%08x", word);
        hex += buffer;
    }
    return hex;
}

std::string sourceHashOf(const std::vector<std::string>& repoRelativeRoots) {
    namespace fs = std::filesystem;
    const fs::path root(CNPG_SOURCE_DIR);

    std::vector<std::string> relativePaths;
    for (const std::string& subdirectory : repoRelativeRoots) {
        const fs::path entry = root / subdirectory;
        std::error_code ec;
        // A root may be a single FILE as well as a directory (Task P2.7: cnpg_render's digest names
        // tests/support/P1Chain.h explicitly, because the chain assembly is part of what a render is
        // a function of and the rest of tests/support is not).
        if (fs::is_regular_file(entry, ec)) {
            relativePaths.push_back(fs::relative(entry, root, ec).generic_string());
            continue;
        }
        if (!fs::is_directory(entry, ec))
            return {};
        for (fs::recursive_directory_iterator it(entry, ec), end; it != end && !ec; it.increment(ec)) {
            if (!it->is_regular_file(ec))
                continue;
            relativePaths.push_back(fs::relative(it->path(), root, ec).generic_string());
        }
    }
    if (relativePaths.empty())
        return {};

    // Byte-wise path order, so the digest does not depend on directory-iteration order (which is
    // filesystem-defined) or on the machine's locale.
    std::sort(relativePaths.begin(), relativePaths.end());

    std::string stream;
    for (const std::string& relative : relativePaths) {
        stream += relative;
        stream += '\n';

        std::ifstream file(root / relative, std::ios::binary);
        if (!file)
            return {};
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string contents = buffer.str();
        // Normalize CRLF checkouts onto the repository's own LF form (.gitattributes sets
        // `* text=auto eol=lf`), so the digest is a property of the content, not of the checkout.
        contents.erase(std::remove(contents.begin(), contents.end(), '\r'), contents.end());
        stream += contents;
    }
    return sha256Hex(stream);
}

std::string dspSourceHash() { return sourceHashOf({"dsp/include", "dsp/src"}); }

std::string renderSourceHash() {
    // See SourceHash.h for why these four and not others. Order is irrelevant to the digest (the
    // paths are sorted below), and is written physics-first so the list reads as what it is.
    return sourceHashOf({"dsp/include", "dsp/src", "tests/support/P1Chain.h", "tests/render"});
}

} // namespace cnpg::test

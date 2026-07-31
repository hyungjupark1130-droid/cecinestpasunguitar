#pragma once

#include <string>

// SourceHash -- the content hash that gives the golden sidecars honest, verifiable provenance
// (docs/plan.md section 4.3 "generator git commit"; tests/data/golden/README.md documents the
// workflow).
//
// WHY A CONTENT HASH AND NOT A COMMIT SHA. The sidecar's provenance field used to be
// `generatorCommit`, resolved by CMake at configure time from `git rev-parse HEAD`. That field is
// structurally incapable of being right: goldens are written BEFORE the commit that contains
// them exists, so the recorded SHA always named the parent commit, and no checkout could ever
// verify the claim. A hash of the renderer's own source has none of that problem -- the bytes it
// covers are committed in the very same commit as the goldens, so the claim is self-consistent
// inside that commit and checkable from any checkout, forever, with no repository history and no
// git at all.
//
// THE RECIPE, exactly (it is a published format, not an implementation detail -- anyone must be
// able to reproduce it):
//
//   For every regular file under dsp/include and dsp/src, recursively, ordered by byte-wise
//   comparison of its repository-relative path written with '/' separators, feed into SHA-256:
//       <relative path> '\n' <file bytes with every 0x0D removed>
//
//   The CR stripping makes the digest identical on a CRLF checkout and an LF one, so "verifiable
//   against any checkout" means any checkout, not just an LF one. The equivalent shell command,
//   run from the repository root, is in tests/data/golden/README.md.

namespace cnpg::test {

// Digest of the dsp/ sources in the checkout this binary was configured from, as 64 lowercase hex
// characters. Empty if the directories cannot be read. Message-thread/test use: it walks the
// source tree and reads every file.
std::string dspSourceHash();

// SHA-256 of an arbitrary byte string, as 64 lowercase hex characters. Exposed so the provenance
// case can test the primitive against published vectors rather than trusting the digests it
// writes into 60 committed sidecars.
std::string sha256Hex(const std::string& bytes);

} // namespace cnpg::test

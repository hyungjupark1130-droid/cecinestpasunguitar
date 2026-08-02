#pragma once

#include <string>
#include <vector>

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

// The same recipe over an explicit list of repository-relative roots (directories or single files),
// so a caller that depends on more than dsp/ can name what it depends on. Added at Task P2.7 for
// cnpg_render; see renderSourceHash(). Empty if any named root cannot be read.
std::string sourceHashOf(const std::vector<std::string>& repoRelativeRoots);

// *** THE DIGEST cnpg_render STAMPS INTO ITS FILENAMES (Task P2.7, carry-forward C2). ***
//
// It replaces a configure-time `git rev-parse --short HEAD`, and the replacement is not cosmetic.
// docs/plan.md section 4.8 required "render filenames embed corpus version + git hash so listening
// notes are attributable", and the configure-time value could not do that: it is resolved when CMake
// last ran, not when the binary was built or run. (Section 4.8 now says "corpus version + a
// render-time content hash" in both plan copies, amended by this task -- a plan that still mandated a
// git hash would mandate something the tree no longer produces, and P2.8 is the pass that reads it.)
// Observed on this repository -- a build/ tree
// configured at 77b0430 produced renders from the code at 1ccfcb1 and filed them as
// `..._cv1_g77b0430.wav`, three commits stale, so TWO DIFFERENT CODE STATES PRODUCED IDENTICAL
// FILENAMES. That is exactly the confusion the field exists to prevent, and P2.8 is the listening
// pass whose notes depend on it. tests/render/CMakeLists.txt claimed the configure-time hash
// "necessarily names the PARENT of the commit that lands the binary"; it names neither reliably.
//
// A content hash computed at RENDER time has none of that problem, for the same reason it fixed the
// golden sidecars: the bytes it covers are the bytes that produced the audio, so the claim is
// self-consistent and checkable from any checkout, with no repository history and no git at all.
//
// The roots are what a render is actually a function of: the physics (dsp/include, dsp/src), the
// chain assembly the renderer shares with cnpg_bench and cnpg_tests (tests/support/P1Chain.h), and
// the renderer itself (tests/render). It deliberately does NOT cover the corpus -- the corpus has its
// own version field in the same filename, and conflating the two would change every render's name
// whenever a phrase was added.
std::string renderSourceHash();

// SHA-256 of an arbitrary byte string, as 64 lowercase hex characters. Exposed so the provenance
// case can test the primitive against published vectors rather than trusting the digests it
// writes into 60 committed sidecars.
std::string sha256Hex(const std::string& bytes);

} // namespace cnpg::test

# tests/data/golden/

Committed float64 impulse-response baselines for the `[regression]` suite (`docs/plan.md`
sections 4.3 and 1.6).

## Layout

```
tests/data/golden/<scenario>/<variant>/<rate>/<name>.f64
tests/data/golden/<scenario>/<variant>/<rate>/<name>.json
```

- `<scenario>` — `string_ir` (landed in Task P1.4). Later scenarios get their own folder.
- `<variant>` — `lagrange3` or `thiran1`, the `cnpg::dsp::FractionalDelayKind` the render used.
  Both are maintained: `docs/decisions/0002-fractional-delay.md` ships `lagrange3`, and the losing
  variant stays golden-covered so it cannot bit-rot.
- `<rate>` — `44100`, `48000`, `96000`.
- `<name>` — e.g. `midi069_pluck28_tap87`.

`.f64` is raw little-endian `double`: the shipping float32 render widened to double at capture
time. `.json` is the sidecar carrying provenance (schema version, dsp source hash, generation
date, dsp state version), the exact render configuration (rate, variant, MIDI note, excitation
parameters, exciter noise seed, tap position, length, comparison `atol`) and the layer-(a)
reference features (partial frequencies 1–8, per-octave-band T60, attack-window RMS in dBFS).

Since Task P1.5 the render is a single-string `StringNetwork` fed one `NoteOn`, as `docs/plan.md`
section 4.3 specifies. The second capture that section describes, `bridgeOutputBuffer()`, is not
committed: through P1 the bridge is the rigid termination of `IBridgePort.h`, which carries no
load, so that buffer is identically zero — `CONTRACT: StringNetwork's P1 bridge output is
identically zero` asserts exactly that instead of committing 60 files of silence. Task P2.4 loads
the bridge and regenerates these goldens for the coupled network; that is when the channel starts
carrying signal.

## Provenance: `dspSourceSha256` (schema v2)

**What it is.** SHA-256 over every file in `dsp/include` and `dsp/src` as they stood when the
render ran. Formally: for every regular file under those two directories, recursively, ordered by
byte-wise comparison of its repository-relative path written with `/` separators, feed

```
<relative path> '\n' <file bytes with every 0x0D removed>
```

into SHA-256. Stripping CR makes the digest identical on a CRLF checkout and an LF one.
`tests/support/SourceHash.cpp` is the implementation; from the repository root the same digest
comes out of:

```bash
find dsp/include dsp/src -type f | LC_ALL=C sort \
  | while IFS= read -r f; do printf '%s\n' "$f"; tr -d '\r' < "$f"; done \
  | sha256sum
```

**Why not a commit SHA.** Schema v1 recorded `generatorCommit`, resolved by CMake at configure
time from `git rev-parse HEAD`. That field could never be right: goldens are written *before* the
commit that contains them exists, so the recorded SHA always named that commit's parent, and no
checkout could verify the claim — the tree it named was, by construction, not the tree that
produced the bytes. A content hash has no such ordering problem, because the bytes it covers are
committed alongside the goldens. To verify any golden set, check out the commit that carries it
and run the command above: it must print the `dspSourceSha256` in the sidecars.

**What is and is not enforced.**

- `REGRESSION/A` requires every sidecar in the set to carry the *same* 64-hex digest, so a
  half-finished regeneration — some files rewritten, some left over from an older tree — fails
  instead of shipping a set no single checkout can account for.
- `REGRESSION/P` checks the mechanism itself: the digest is well-formed and deterministic, the
  SHA-256 primitive matches the FIPS 180-4 published vectors, and the recorded digest is printed
  next to the current checkout's so every CI log records whether they match.
- Neither asserts `recorded == current`. Whether the goldens are *stale* is what layers (a) and
  (b) decide, by re-rendering with the current code on every CI run. Asserting equality here
  would force a 60-file golden commit for any edit anywhere under `dsp/` — a comment, or a module
  like `OutputGain` this scenario never executes — which would make the `Regenerate-Goldens:`
  trailer a lie in exactly the cases where nothing changed.

## What consumes them

- **`REGRESSION/A: feature invariants`** re-renders each scenario and compares the *features* in the
  sidecar — partials within ±2 cents, per-band T60 within ±10 %, attack RMS within ±1.5 dB. These
  survive an interpolator or filter-topology swap, so they are the layer that keeps meaning across
  algorithm changes. It also checks the sidecar's recorded render configuration against the one the
  scenario actually uses.
- **`REGRESSION/B: float64 golden exactness`** compares sample-wise against the `.f64` at
  `atol = 1e-7`. This is effectively bit-stability of the float32 path and is toolchain-sensitive,
  so it runs on Windows/MSVC only (`docs/plan.md` section 4.9); the ubuntu portability job runs
  layer (a).
- **`REGRESSION/P: golden provenance is verifiable`** — see above.

## Regenerating

```
cmake --build build --config Release --target cnpg_regen_goldens
```

The target re-renders every scenario × variant × rate, overwrites the `.f64` files, refreshes the
sidecars, and prints a drift report (max absolute sample difference and the fundamental's cents
delta versus what was on disk). There is no configure step to remember any more: the sidecar's
provenance is read from the working tree at render time, not from the CMake cache.

Until `cnpg_render` lands (Task P1.11) the target drives the hidden Catch2 case
`REGEN: rewrite string_ir goldens` inside `cnpg_tests`, so goldens and the tests that gate them
always come from exactly the same renderer. The case is tagged `[.]`, so CTest never discovers it
and CI never runs it. It is deliberately *not* named `REGRESSION/…`: Catch2 only skips hidden cases
when the spec carries no filter at all, so a plausible name glob (`cnpg_tests "REGRESSION*"`) would
otherwise rewrite all 60 committed goldens as a side effect of running the tests that gate them.

Regenerate **after** every `dsp/` edit that the commit will carry, and commit the two together:
that ordering is what makes the recorded hash describe the committed tree.

## Commit rule (enforced)

**Any commit touching `tests/data/golden/` must carry a `Regenerate-Goldens: <reason>` git trailer**,
and the reason must state (1) which variants/rates changed, (2) the algorithm change motivating the
regeneration, and (3) that the layer-(a) invariants pass on the new renders.
`.github/scripts/check-golden-commit.sh` fails the Windows CI job otherwise.

Inspect the drift report and the diff before committing. Layer-(a) invariants are designed to stay
green across a legitimate regeneration; if they move, the change is a behaviour change, not a
refresh, and belongs in the commit message.

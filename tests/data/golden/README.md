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
time. `.json` is the sidecar carrying provenance (schema version, generator commit, generation
date, dsp state version), the exact render configuration (rate, variant, MIDI note, excitation
parameters, exciter noise seed, tap position, length, comparison `atol`) and the layer-(a)
reference features (partial frequencies 1–8, per-octave-band T60, attack-window RMS in dBFS).

## What consumes them

- **`REGRESSION/A: feature invariants`** re-renders each scenario and compares the *features* in the
  sidecar — partials within ±2 cents, per-band T60 within ±10 %, attack RMS within ±1.5 dB. These
  survive an interpolator or filter-topology swap, so they are the layer that keeps meaning across
  algorithm changes.
- **`REGRESSION/B: float64 golden exactness`** compares sample-wise against the `.f64` at
  `atol = 1e-7`. This is effectively bit-stability of the float32 path and is toolchain-sensitive,
  so it runs on Windows/MSVC only (`docs/plan.md` section 4.9); the ubuntu portability job runs
  layer (a).

## Regenerating

```
cmake --build build --config Release --target cnpg_regen_goldens
```

The target re-renders every scenario × variant × rate, overwrites the `.f64` files, refreshes the
sidecars, and prints a drift report (max absolute sample difference and the fundamental's cents
delta versus what was on disk). Re-run `cmake --preset windows-msvc-release` first if you want the
sidecar's `generatorCommit` to match HEAD — it is resolved at configure time.

Until `cnpg_render` lands (Task P1.11) the target drives the hidden Catch2 case
`REGRESSION/REGEN: rewrite string_ir goldens` inside `cnpg_tests`, so goldens and the tests that
gate them always come from exactly the same renderer. The case is tagged `[.]`, so CTest never
discovers it and CI never runs it.

## Commit rule (enforced)

**Any commit touching `tests/data/golden/` must carry a `Regenerate-Goldens: <reason>` git trailer**,
and the reason must state (1) which variants/rates changed, (2) the algorithm change motivating the
regeneration, and (3) that the layer-(a) invariants pass on the new renders.
`.github/scripts/check-golden-commit.sh` fails the Windows CI job otherwise.

Inspect the drift report and the diff before committing. Layer-(a) invariants are designed to stay
green across a legitimate regeneration; if they move, the change is a behaviour change, not a
refresh, and belongs in the commit message.

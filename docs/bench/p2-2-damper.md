# P2.2 damper -- what a permanently in-line junction costs

Task P2.2. Methodology, machine and settings are identical to `docs/bench/p2-1-scale-out.md`
(AMD Ryzen 9 7950X, MSVC 1944 Release, 48 kHz / 128-sample blocks / 2x oversampling / 60 s
renders, two runs of each command), so the rows below are directly comparable to that file's.

This document **records** numbers, it does not gate them. The 25-30% hard gate binds
`p2_default6` and lands with Task P2.9 in `docs/bench/p2-exit.md`.

```
cnpg_bench --config p2_default6 --samplerate 48000 --blocksize 128 --seconds 60
cnpg_bench --config p2_max8     --samplerate 48000 --blocksize 128 --seconds 60
```

## Numbers

| Configuration | strings | median (us) | median CPU | p99 (us) | p99 CPU | max (us) | max CPU |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `--config p2_default6` | 6 | 29.800 | 1.118% | 36.200 | 1.358% | 68.800 | 2.580% |
| `--config p2_default6` (run 2) | 6 | 30.700 | 1.151% | 36.700 | 1.376% | 86.500 | 3.244% |
| `--config p2_max8` | 8 | 38.300 | 1.436% | 46.000 | 1.725% | 680.000 | 25.500% |
| `--config p2_max8` (run 2) | 8 | 37.600 | 1.410% | 43.800 | 1.643% | 608.700 | 22.826% |

`nonFiniteSamples` is 0 in every run, and each configuration's `audioHash` is identical across
its two runs. The `max` column again varies by an order of magnitude between two runs of the
same command while the medians move by under a microsecond -- the OS-scheduling-preemption
pattern `docs/bench/p1-baseline.md` documents and the reason `max` is recorded but never gated.

**`p2_default6` sits at 1.12% of one core against a 30% budget -- about 27x headroom.**

## The delta from P2.1, and what caused it

| Configuration | P2.1 median | P2.2 median | change |
| --- | --- | --- | --- |
| `p2_default6` | 16.900 us | 29.800 us | +12.9 us, 1.76x |
| `p2_max8` | 20.800 us | 38.300 us | +17.5 us, 1.84x |

That is a real and expected cost, and it has two components worth separating because only one
of them is the junction.

**The seam, and why it is not conditional.** Every live string now runs
`readJunctionInputs` -> `scatter` -> `writeJunctionOutputs` on every sample: four fractional
rail interpolations and two fractional deposits, twelve rail slots touched, plus one divide for
the loss coefficient. That is comparable to the work `readTapAt` and `injectAt` were already
doing, so roughly doubling the per-string rail traffic is the right order of magnitude for what
was measured. It runs unconditionally -- there is deliberately no `if (engagement > 0)` around
it, because docs/plan.md Task P2.2 requires the junction to stay permanently in-line: a
compiled-out damper is a second topology, and the transparency claim is worth exactly as much
as the fact that it is measured on the shipping one. (The claim is strong: at engagement 0 the
seam deposits exactly `0.0f` and the render is bit-identical, so what this CPU buys is
*provability*, not audible transparency, which was free.)

**Strings staying live longer.** The bench workload plays and releases notes. Under P1's
release envelope a released string was cleared 460 ms after its note-off by a fixed countdown.
Under the real damper it leaves the loop when the silence watchdog observes it below
-100 dBFS -- measured at ~0.5 s for a mid note, and longer for low ones, because a point damper
at p = 0.15 has an exact node on partial 20 and cannot touch it at all, so that partial rides
the loop loss down on its own schedule. Released strings therefore occupy the per-sample loop
for longer than they used to, which raises the average number of strings the loop is actually
rendering. This is not overhead that can be optimised away without changing the physics; it is
the physics being honest about when a string has stopped ringing.

If a later phase needs the headroom back, the seam is the part with slack in it: the two reads in
`writeJunctionOutputs` re-derive values `readJunctionInputs` computed a few instructions earlier,
so roughly a third of the junction's rail traffic is redundant. Caching them across the pair would
be **bit-identical** — same inputs, same rail contents, nothing writes between the two calls — so
the transparency and passivity claims do not depend on the re-read, and nothing in this task's
gates would change. It is left as it is only because it would mean giving `WaveguideString` a
piece of per-string scratch state that lives *between* two calls and is silently wrong if a caller
ever interleaves them, which is a real interface change and belongs to a task that wants it (P2.3
rewrites both sides of this seam for the moving-junction crossfade and is the natural place). A
deferred optimisation, not a constraint.

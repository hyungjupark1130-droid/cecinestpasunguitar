# cecinestpasunguitar

A physical-modeling guitar-deconstruction synthesizer (VST3 / Standalone) — working title **cecinestpasunguitar**, vendor **Hyung Ju Park**. Licensed under the [GNU General Public License v3.0](LICENSE) (GPLv3).

## Architecture

The plugin is split into two domains. A sample-domain physics core (`dsp/`, `StringNetwork`: exciter → strings ⇄ bridge, per-sample loop, no inter-module block delay) feeds a block-domain unidirectional chain (pickup tap + RLC → Koren triode → monitoring filters), with the domain boundary crossed via per-block per-string tap buffers computed inside the sample loop. `dsp/` is a JUCE-free, headless-testable C++20 static library; `plugin/` is a thin JUCE 8 wrapper with APVTS as the single parameter source.

See [`docs/plan.md`](docs/plan.md) for the full implementation plan.

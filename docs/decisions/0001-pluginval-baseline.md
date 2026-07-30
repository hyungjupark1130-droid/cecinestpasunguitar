# 0001: pluginval baseline

## Context

Task P0.6 requires a validated `pluginval` gate against the built `cecinestpasunguitar` VST3 before P0
can close. `cmake/PluginvalPin.cmake` (added in P0.2/P0.3-era scaffolding) already pinned pluginval
`v1.0.4` and the Windows release asset URL, but its `CNPG_PLUGINVAL_WINDOWS_SHA256` field held the
placeholder `"UNVERIFIED-PENDING-P0.6"` — computing and recording that hash, installing the tool, and
running the gate is this task's job.

The task brief's literal acceptance-criterion command references `build\bin\Release\VST3\...`. That path
is stale: `CMAKE_RUNTIME_OUTPUT_DIRECTORY` only governs plain executables (`build/bin/<Config>/`), while
`juce_add_plugin()` writes VST3/Standalone artefacts to their own JUCE-managed artefact directory (per
`docs/plan.md` §1.2 item 4 and confirmed by the actual P0.4 build output). The command below uses the
real, adjudicated-correct artefact path.

## Install

- Version: pluginval **1.0.4** (matches `CNPG_PLUGINVAL_VERSION` in `cmake/PluginvalPin.cmake`).
- Source: official Tracktion GitHub release, Windows asset —
  `https://github.com/Tracktion/pluginval/releases/download/v1.0.4/pluginval_Windows.zip`.
- Downloaded and hashed via PowerShell:
  ```powershell
  Invoke-WebRequest -Uri $url -OutFile $dest -UseBasicParsing
  Get-FileHash -Path $dest -Algorithm SHA256
  ```
- Asset filename hashed: `pluginval_Windows.zip`.
- SHA-256: `C08E61CE3B96DB41636F8EC7E76F4C7E2C13EBDAC7FA1B5A1F52B4F32EC715AB` — now recorded in
  `cmake/PluginvalPin.cmake` (`CNPG_PLUGINVAL_WINDOWS_SHA256`), replacing the placeholder.
- Extracted `pluginval.exe` reports `pluginval - 1.0.4` via `--version`, confirming the archive matches
  the pinned version.
- Local install location (not committed — covered by the existing `build/` rule in `.gitignore`):
  `build/tools/pluginval/pluginval_Windows.zip` (archive) and
  `build/tools/pluginval/extracted/pluginval.exe` (extracted binary). CI (P0.7) will re-download and
  re-verify against this same pinned hash rather than reuse a machine-local copy.

## Command

Built artefact under test: `build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3`
(rebuilt immediately before the run via `cmake --build build --config Release --target cnpg_plugin`, so
the binary reflects the current P0.4 source, not a stale artefact).

```
pluginval --strictness-level 10 --validate build\plugin\cnpg_plugin_artefacts\Release\VST3\cecinestpasunguitar.vst3
```

(The brief's literal `build\bin\Release\VST3\...` path does not exist for this project's JUCE artefact
layout — see Context above.)

## Measurement

Run twice, back to back, to check for seed-dependent flakiness (pluginval's "Fuzz parameters" test uses a
random seed each run):

| Run | Random seed | Exit code | Result |
|---|---|---|---|
| 1 | `0x2e66bd5` | 0 | `SUCCESS` |
| 2 | (fresh seed, same invocation) | 0 | `SUCCESS` |

Both runs exercised the full strictness-10 suite: plugin scan/open (cold+warm), plugin info, programs,
editor (+ automation, + open-while-processing), audio processing and non-releasing audio processing at
{44100, 48000, 96000} Hz x {64, 128, 256, 512, 1024} samples, plugin state + state restoration,
parameter automation at the same sample-rate/block-size matrix, automatable parameters, background-thread
state, parameter thread safety, bus listing/enabling/disabling/restoring (1 stereo output, 0 input —
matches the P0.4 frozen bus layout), and fuzz parameters. No failing test, warning, or crash was reported
in either run. No audio-device or plugin process was left running after either run (verified via
`Get-Process`).

Full logs (not committed — `build/` is gitignored; archived as a CI artifact from P0.7 onward per the
task brief):
- `build/pluginval-logs/p0.6-strictness10.log` (run 1, primary)
- `build/pluginval-logs/p0.6-strictness10-verify.log` (run 2, confirmation)

## Decision

Gate passes at the required strictness level (10) against the current P0.4 build with pinned pluginval
1.0.4. No plugin-side fixes were needed. `cmake/PluginvalPin.cmake`'s SHA-256 placeholder is now filled
with a verified hash of the official release asset. This baseline is adopted as the P0.6 gate; P0.7 wires
the same command into CI using this same pin.

## Date

2026-07-30

## Appendix (reserved for P0.8)

Task P0.8 ("Manual acceptance — Ableton Live load check") records its pass/fail result and the Live
version used, here, per `docs/plan.md`. Not yet performed — out of scope for P0.6.

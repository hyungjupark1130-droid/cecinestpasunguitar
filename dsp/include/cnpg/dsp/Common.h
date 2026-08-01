#pragma once

// cnpg::dsp -- unified module contract, Sample aliases, and design-envelope constants.
// See docs/plan.md section 2.1 (this file is that draft, transcribed verbatim). Every
// public header under dsp/include/cnpg/dsp/ is 100% JUCE-free; this file has zero
// includes because it needs none.

namespace cnpg::dsp {

using Sample = float;    // realtime path (float32, locked)
using Sample64 = double; // offline references, goldens, energy accounting in tests

// Design-envelope constants; all preallocation in prepare() is sized against these.
inline constexpr int kMaxStrings = 8; // active count configurable 1..8, default 6
// Spatial pickup taps preallocated PER STRING on the sample->block domain boundary
// (docs/decisions/0004-phase2-vision-decisions.md, D1). Exactly one is active through P2.1; the
// capacity exists because a humbucker is a TRUE two-coil construction -- two spatial taps on the
// same string with real coil spacing, aperture and polarity, whose comb null at f = v/2d is then
// emergent -- and not a downstream voicing preset. Widening the boundary is cheap inside the task
// that is already rewriting that storage (P2.1) and expensive afterwards, which is the whole reason
// the capacity lands before the feature. Same shape as kMaxStrings: preallocate the maximum,
// run one, and let a later task raise the active count.
inline constexpr int kMaxTapsPerString = 4;
inline constexpr int kMinMidiNote = 21;             // A0
inline constexpr int kMaxMidiNote = 108;            // C8
inline constexpr double kMaxDesignRateHz = 96000.0; // 44.1-96 kHz guaranteed; 192 kHz best-effort
inline constexpr int kMaxOversampling = 8;          // Oversampler factor upper bound; valid factors {2, 4, 8} only
// Global pitch-bend range, semitones either side of centre. A design-envelope constant, not a
// taste one: WaveguideString sizes its delay rails for kMinMidiNote detuned this far DOWN, so
// widening it is a rail-sizing change (dsp/src/WaveguideString.cpp, kSizingLowestF0Hz) and not
// just a different number here. cnpg::dsp::pitchWheelToSemitones maps the MIDI pitch wheel onto
// it and StringNetworkParams::pitchBendSemitones carries it.
inline constexpr float kPitchBendRangeSemitones = 2.0f;

// Module lifecycle convention (informal concept; every module in dsp/ conforms):
//   void prepare(double sampleRate, int maxBlockSize);   // message thread, may allocate
//   void reset() noexcept;                                // realtime-safe, clears state
//   void setParams(const <Module>Params&) noexcept;       // realtime-safe, retargets smoothers
//   process(...) noexcept;                                 // realtime-safe, no alloc/locks
//
// prepare() sizes all state for the worst case (kMaxStrings, kMinMidiNote, maxBlockSize)
// against max(hostSampleRate, kMaxDesignRateHz), so a 192 kHz best-effort host never
// under-allocates. reset() is realtime-safe and clears transient state (delay-line
// contents, ramp/smoothing progress) -- the exact scope of "state" is per-module and
// documented on each module's own reset().
// process(...) never allocates, locks, throws, performs I/O, or traps on denormals
// (FTZ/DAZ is engaged by the caller's RAII guard in processBlock, not by this module).

} // namespace cnpg::dsp

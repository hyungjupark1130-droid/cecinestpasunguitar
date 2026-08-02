#pragma once

#include <array>
#include <cstdint>

// cnpg::dsp -- unified module contract, Sample aliases, and design-envelope constants.
// See docs/plan.md section 2.1 (this file is that draft, transcribed verbatim). Every
// public header under dsp/include/cnpg/dsp/ is 100% JUCE-free.

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

// Default open-string tuning. Slots 0..5 are EADGBE (docs/plan.md's {40, 45, 50, 55, 59, 64});
// slots 6 and 7 carry the low B and F# an extended-range 7- and 8-string instrument adds, so the
// full default set IS the standard 8-string tuning F#1 B1 E2 A2 D3 G3 B3 E4 -- written with the
// six-string spelling first, because slots 0..5 have to stay EADGBE for the shipped 6-string
// default.
//
// IT LIVES HERE, IN THE SHARED HEADER, BECAUSE TWO MODULES NEED THE SAME ANSWER (Task P2.7).
// NoteAllocator has always known the open tuning -- it is what fingering distance is measured from.
// StringNetwork now needs it too, for a different reason: a string nobody has played has to be
// tuned to SOMETHING, and through P2.6 that something was kMinMidiNote for every string. Six
// untouched strings all at A0 (27.5 Hz) is not an instrument, and it is worse than arbitrary:
// A0's harmonic series contains 55, 82.5, 110, 137.5, 165, 192.5 and 220 Hz, i.e. very nearly
// everything the other strings play, so an UNPLAYED string was a BETTER sympathetic resonator
// than a real open string. Measured consequence: the P2.6 sympathetic-truncation figure read
// 6.63 dB above P2.4's for exactly this reason (tests/dsp/RetriggerModeTests.cpp).
//
// A duplicated literal in the two headers would let the two answers drift apart silently, and
// "which tuning is the instrument at rest" is not a question that may have two answers.
inline constexpr std::array<std::uint8_t, kMaxStrings> kDefaultOpenStringMidiNote{40, 45, 50, 55, 59, 64, 35, 30};

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

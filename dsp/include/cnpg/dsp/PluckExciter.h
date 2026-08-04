#pragma once

#include <cstdint>
#include <type_traits>

// PluckExciter -- see docs/plan.md section 2.3. Task P1.1 landed PluckExciterParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft). Task P1.3 adds the
// PluckExciter class itself: one-shot pluck/pick excitation only through P2 (locked decision Q1)
// -- trigger() latches position and hardness for the life of the burst (exciter position is not
// modulatable while ringing, locked decision Q2); velocity maps to amplitude plus a mild hardness
// increase; a small seeded noise-burst component is mixed in, scaled by noiseAmount.
// renderSample() is called from inside the StringNetwork per-sample loop (StringNetwork itself
// lands in Task P1.5); output is injected into the string rails at latchedPosition01(). Per
// docs/plan.md section 2.1, every sample-domain class is template <typename SampleT> with
// explicit float/double instantiations compiled into cnpg_dsp -- the realtime path uses float,
// the tier-2 [energy] passivity tests (P2) run double. Zero JUCE includes.

namespace cnpg::dsp {

// ---- WHERE THE PICK LANDS, AND WHY IT IS NOT THE MIDDLE OF THE STRING -------------------------
//
// An ideal point excitation at a fraction d from a termination deposits energy in mode n in
// proportion to sin(n*pi*d), so it is EXACTLY BLIND to every partial with a node there: the comb's
// first null sits at partial 1/d and it repeats at every multiple. Written as a rational a/b in
// lowest terms, the null set is exactly {b, 2b, 3b, ...} -- ONSET b, DENSITY 1/b -- so
//
//     d = 1/2 is the single worst point on the whole slider: b = 2 is the smallest value b can
//     take, which is simultaneously the LOWEST possible onset (partial 2, the octave) and the
//     HIGHEST possible density (half of every partial the instrument produces).
//
// That is where this default sat through P2.9, and the pickup tap sat there too, so the two combs
// COINCIDED and every null was squared. Measured on the low open E through the shipping chain, the
// even partials 2/4/6/8/10 came out 48.9 / 43.1 / 48.1 / 43.5 / 63.9 dB below the loudest partial
// in the note, i.e. 39 to 49 dB below their own odd neighbours: an odd-harmonic-only spectrum,
// which is a clarinet, not a guitar. docs/listening/physical-plausibility-checklist.md item 7
// already names the pathology -- "near-middle plucks are hollow, with suppressed even harmonics" --
// and the shipped default WAS the near-middle pluck.
//
// THE CRITERION. Partials 2..6 are the ones that spell the intervals the instrument is asked to
// play: 2 and 4 are octaves, 3 and 6 are fifths (2 cents from tempered), 5 is a major third
// (14 cents flat). Partial 7 is the FIRST partial that names no tempered interval at all (31 cents
// flat of a minor seventh), so [2, 6] is where the harmonic series stops carrying chord identity
// and the band ends on a property of the series rather than on taste. Requiring no null inside it:
//
//     onset = 1/d >= 7    <=>    d <= 1/7 = 0.1429 of the string from a termination
//
// 1/7 is the cheapest value that satisfies it and it satisfies it with ZERO margin (partial 7 is
// nulled exactly). One step past it is 1/8 -- and 8 divides 16, which is the tap's own onset
// (StringNetwork.h), so a pluck there would put its null on top of the pickup's at partial 16 and
// re-create in miniature the very squaring that made the old default measure 49 dB deep instead of
// 25. 1/9 is the first value with margin AND a null set disjoint from the tap's through partial
// 143, and on a 25.5" scale it is 2.833" from the bridge -- an ordinary picking position, between
// the bridge pickup and the neck pickup where a player's hand actually is.
//
// THE COST, stated because it is real: coupling to the FUNDAMENTAL is sin(pi*d), so moving off the
// midpoint costs 20*log10(sin(pi/9)) = 9.4 dB of it. The midpoint is the unique position that
// maximises the fundamental, and it buys that by deleting every even partial. This trade is the
// whole change: the fundamental for the harmonic series.
//
// The value is written as a distance FROM THE BRIDGE and subtracted, because this codebase's
// convention is 0 = nut, 1 = bridge and a picking position is a bridge-referred quantity. sin^2 is
// symmetric about 0.5, so 1 - 1/9 and 1/9 are acoustically all but identical (the bridge is
// compliant and the nut is rigid, so not exactly), but only one of them makes the user-facing
// "Exciter Position" label read true.
inline constexpr float kDefaultPluckDistanceFromBridge01 = 1.0f / 9.0f;

struct PluckExciterParams {
    // 0..1 (0 = nut, 1 = bridge), used when the note event carries no explicit position.
    float defaultPosition = 1.0f - kDefaultPluckDistanceFromBridge01;
    float defaultHardness = 0.5f; // 0..1
    float noiseAmount = 0.0f;     // 0..1, small noise-burst component mixed into the pluck shape
};

// The criterion above, pinned on the shipped value rather than on a literal, so a future edit that
// walks the default back toward the midpoint fails here instead of in somebody's ears. It is not
// vacuous: at the old default d = 1/2 the left-hand side is 2.
static_assert(1.0f / kDefaultPluckDistanceFromBridge01 >= 7.0f,
              "The default pluck must not null a partial in [2, 6] -- the partials that spell the "
              "intervals the instrument plays. See the derivation above this struct.");

static_assert(std::is_trivially_copyable_v<PluckExciterParams>,
              "PluckExciterParams must stay trivially copyable for the realtime APVTS snapshot path.");

// One-shot pluck/pick excitation. trigger() starts a short raised-cosine (Hann-window) displacement
// burst: envelope is exactly 0 at the first and last rendered sample (click-free abutment with the
// silence before and after), peaking at 1 at the burst's midpoint. Burst duration is driven by the
// latched hardness (harder -> shorter -> spectrally brighter "tilt") and is always well under the
// 10 ms bound at 96 kHz, so no dynamic buffer is needed: renderSample() computes the shape
// analytically from a per-instance sample counter, which is what makes both prepare() and
// process() trivially alloc-free. A small noise-burst component (scaled by
// PluckExciterParams::noiseAmount, windowed by the same envelope so it never clicks at the
// boundaries) is drawn from an internal PRNG seeded with a fixed, non-time-based constant in
// prepare()/reset() -- required so cnpg_render's byte-identical renders (docs/plan.md section
// 4.8) only depend on the NoteEvent stream, never on wall-clock time.
template <typename SampleT> class PluckExciter {
  public:
    // Message thread; may allocate (nothing to allocate here -- see class comment). Calls reset().
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Deactivates any in-flight burst and reseeds the noise PRNG to its fixed
    // startup state, so a fresh prepared instance and any later reset() instance behave identically.
    void reset() noexcept;

    // Realtime-safe; only retargets params_ (a trivial copy) -- does not touch a burst already in
    // flight. noiseAmount is read live by renderSample(); defaultPosition/defaultHardness are for
    // callers deciding what to pass into trigger() when a NoteEvent carries no explicit value.
    void setParams(const PluckExciterParams& p) noexcept;

    // Starts a one-shot excitation; position01 and hardness01 are clamped to 0..1 and latched for
    // the life of the burst (unaffected by any setParams() call before the burst completes).
    // velocity (clamped 0..1) scales the burst's peak amplitude linearly and nudges the latched
    // hardness mildly brighter (docs/plan.md locked decision Q1).
    void trigger(float velocity, float position01, float hardness01) noexcept;

    // Realtime-safe; never allocates. Next excitation sample; exactly SampleT(0) once the burst has
    // completed (including every call before the first trigger()).
    SampleT renderSample() noexcept;

    // Injection point for StringNetwork; latched at the most recent trigger(), unaffected by
    // setParams() or by the burst completing.
    float latchedPosition01() const noexcept { return latchedPosition01_; }

    // True from trigger() until the burst's last sample has been rendered.
    bool isActive() const noexcept { return active_; }

  private:
    double sampleRate_ = 44100.0;
    PluckExciterParams params_{};

    float latchedPosition01_ = 0.5f;
    float latchedHardness01_ = 0.5f;
    SampleT peakAmplitude_ = SampleT(0);

    int burstLengthSamples_ = 0;
    int sampleIndex_ = 0;
    bool active_ = false;

    // xorshift32 state for the noise-burst component; never seeded from time or entropy (see
    // class comment). 0 is an absorbing state for xorshift32, so reset()/prepare() always restore
    // the fixed nonzero seed rather than merely clearing to 0.
    std::uint32_t noiseState_ = 0;

    float nextBipolarNoise() noexcept;
};

extern template class PluckExciter<float>;  // realtime path
extern template class PluckExciter<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

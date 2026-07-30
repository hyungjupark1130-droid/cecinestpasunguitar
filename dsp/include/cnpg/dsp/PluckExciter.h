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

struct PluckExciterParams {
    float defaultPosition = 0.5f; // 0..1, used when the note event carries no explicit position
    float defaultHardness = 0.5f; // 0..1
    float noiseAmount = 0.0f;     // 0..1, small noise-burst component mixed into the pluck shape
};

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

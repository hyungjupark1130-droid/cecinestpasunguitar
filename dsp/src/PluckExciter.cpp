#include "cnpg/dsp/PluckExciter.h"

#include <algorithm>
#include <cmath>

namespace cnpg::dsp {

namespace {

constexpr float kNominalPeakDb = -18.0f; // burst peak calibration target (docs/plan.md Task P1.3 AC)
constexpr float kMinBurstMs = 1.0f;      // hardness 1 -> shortest/brightest burst
constexpr float kMaxBurstMs = 8.0f;      // hardness 0 -> longest/darkest burst; both well under the 10 ms bound
constexpr float kVelocityHardnessCoupling = 0.15f; // "mild hardness increase" from velocity (locked decision Q1)

// xorshift32 (Marsaglia); a small, allocation-free, deterministic-given-its-seed PRNG. Requires a
// nonzero state (0 is an absorbing fixed point). Seed is a fixed constant chosen only to avoid the
// degenerate all-zero state -- never derived from time or entropy (see PluckExciter.h class comment).
constexpr std::uint32_t kNoiseSeed = 2463534242u;

std::uint32_t xorshift32(std::uint32_t& state) noexcept {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float dbToLinear(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

float clamp01(float v) noexcept { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

} // namespace

template <typename SampleT> void PluckExciter<SampleT>::prepare(double sampleRate, int /*maxBlockSize*/) {
    // Nothing to preallocate: burst duration is bounded (< 10 ms even at 96 kHz) and rendered
    // analytically from sampleIndex_/burstLengthSamples_, never from a buffer.
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    reset();
}

template <typename SampleT> void PluckExciter<SampleT>::reset() noexcept {
    active_ = false;
    sampleIndex_ = 0;
    burstLengthSamples_ = 0;
    peakAmplitude_ = SampleT(0);
    latchedPosition01_ = params_.defaultPosition;
    latchedHardness01_ = params_.defaultHardness;
    noiseState_ = kNoiseSeed; // fixed, non-time-based -- see PluckExciter.h class comment
}

template <typename SampleT> void PluckExciter<SampleT>::setParams(const PluckExciterParams& p) noexcept { params_ = p; }

template <typename SampleT>
void PluckExciter<SampleT>::trigger(float velocity, float position01, float hardness01) noexcept {
    const float clampedVelocity = clamp01(velocity);
    latchedPosition01_ = clamp01(position01);
    // Velocity nudges the effective hardness mildly brighter/shorter on top of the caller-supplied
    // hardness (docs/plan.md locked decision Q1: "velocity -> amplitude + mild hardness increase").
    latchedHardness01_ = clamp01(clamp01(hardness01) + kVelocityHardnessCoupling * clampedVelocity);

    peakAmplitude_ = static_cast<SampleT>(clampedVelocity * dbToLinear(kNominalPeakDb));

    // Harder -> shorter burst -> wider bandwidth -> brighter spectral tilt (docs/plan.md Task
    // P1.3 step 2). Duration alone drives the tilt; the envelope shape itself never changes.
    const float burstMs = kMaxBurstMs + (kMinBurstMs - kMaxBurstMs) * latchedHardness01_;
    const double burstSeconds = static_cast<double>(burstMs) / 1000.0;
    burstLengthSamples_ = std::max(1, static_cast<int>(burstSeconds * sampleRate_ + 0.5));

    sampleIndex_ = 0;
    active_ = true;
}

template <typename SampleT> SampleT PluckExciter<SampleT>::renderSample() noexcept {
    if (!active_)
        return SampleT(0);

    constexpr double kTwoPi = 6.283185307179586476925286766559;

    // Standard Hann-window formula w[n] = 0.5 * (1 - cos(2*pi*n/(N-1))), n = 0..N-1: exactly 0 at
    // both the first and last rendered sample (click-free abutment with the silence before and
    // after), peaking at exactly 1 at the midpoint. N == 1 is a degenerate single-sample burst;
    // render it at the envelope's peak rather than dividing by zero.
    const double t = (burstLengthSamples_ > 1)
                         ? static_cast<double>(sampleIndex_) / static_cast<double>(burstLengthSamples_ - 1)
                         : 0.5;
    const SampleT envelope = static_cast<SampleT>(0.5 * (1.0 - std::cos(kTwoPi * t)));

    SampleT sample = peakAmplitude_ * envelope;

    if (params_.noiseAmount > 0.0f) {
        // Windowed by the same envelope as the shape burst so the noise component fades in/out
        // with it instead of clicking at the burst boundaries.
        const float noise = nextBipolarNoise();
        sample += peakAmplitude_ * envelope * static_cast<SampleT>(params_.noiseAmount * noise);
    }

    ++sampleIndex_;
    if (sampleIndex_ >= burstLengthSamples_)
        active_ = false;

    return sample;
}

template <typename SampleT> float PluckExciter<SampleT>::nextBipolarNoise() noexcept {
    constexpr float kInvUint32Max = 1.0f / 4294967295.0f;                             // 2^32 - 1
    const float unit01 = static_cast<float>(xorshift32(noiseState_)) * kInvUint32Max; // [0, 1]
    return unit01 * 2.0f - 1.0f;                                                      // [-1, 1]
}

template class PluckExciter<float>;
template class PluckExciter<double>;

} // namespace cnpg::dsp

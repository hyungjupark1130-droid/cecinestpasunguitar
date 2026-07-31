#pragma once

#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"

// PickupTap -- see docs/plan.md section 2.8. Task P1.1 landed PickupTapParams only (already its
// final shape per the draft). This task (P1.6) adds the PickupTap class: a block-domain consumer
// of StringNetwork's per-string tap buffers (StringTapBuffers, fully declared in
// StringNetwork.h -- only forward-declared below, so this header stays light) that sums the
// active strings' fractional-position taps into mono, applies one linear RLC resonance biquad,
// then trims with outputGainDb toward the -18 dBFS per-string nominal structure. Magnetic
// nonlinearity is deliberately absent -- it is P4's. Block domain, NOT templated: it consumes
// only the float/Sample realtime tap instantiation. Zero JUCE includes.
//
// RLC model -- why a BANDPASS, and why that is not a modelling shortcut.
//
// taps.channel(i) carries the string's DISPLACEMENT, not velocity: WaveguideString.h's rails
// "carry the travelling-wave components of the string's displacement" and injectAt() "adds
// excitation into the string's displacement"; PluckExciter's burst is explicitly a displacement
// burst (PluckExciter.h). A magnetic pickup does not sense displacement -- by Faraday's law its
// induced EMF is proportional to the rate of change of flux, i.e. d(displacement)/dt. The
// transducer itself is a DIFFERENTIATOR, sitting in series before the coil's own R/L/C resonance
// ever enters the picture.
//
// So the full displacement -> pickup-output transfer function is (rate of change) x (loaded RLC
// tank response): s * w0^2 / (s^2 + (w0/Q)s + w0^2). That is a single DC zero (the
// differentiation) times a loaded pole pair -- algebraically a 2nd-order BANDPASS, exactly. The
// RBJ constant-skirt-gain bandpass biquad below is the bilinear discretization of precisely that
// analog prototype, with resonanceHz/q placing the pole pair as (w0, Q). The biquad's -6 dB/octave
// low-frequency skirt is therefore not an approximation error or a deliberate voicing tilt to
// paper over -- it IS the transducer's own differentiation, physically required, and belongs in
// the P1 signal path exactly as much as the resonant peak does.
//
// A resonant LOWPASS -- the shape "voltage across the load" suggests if the differentiation step
// is missed -- was tried first and is wrong on two independent counts, not one. (1) A damped
// 2-pole lowpass only produces a magnitude peak once q exceeds 1/sqrt(2) =~ 0.70711, so q = 0.7
// (the acceptance sweep's own lowest case) has no resonance to measure at all. (2) Even where a
// lowpass peak DOES exist, it sits at w0*sqrt(1 - 1/(2*Q^2)), not at w0: at q = 2 that is 6.5%
// below resonanceHz -- 6.5x past the +/-1% acceptance gate -- so the lowpass is structurally
// incompatible with the peak-frequency acceptance criterion at every q in the sweep, not only the
// q = 0.7 edge case. The bandpass's peak sits at exactly resonanceHz for every q > 0 (a property of
// that specific pole/zero placement, not a coincidence of q = 0.7 alone), which is what the
// q in {0.7, 2, 6} acceptance sweep actually needs.
//
// One further consequence of getting the transducer physics right rather than picking whichever
// topology passes the tests: any future revisit of this transducer (P4's magnetic-nonlinearity
// work is the one task that touches it again) must keep this bandpass shape, not "simplify" it
// back to a lowpass -- doing so would make the instrument 6 dB/octave too dark, silently, since
// nothing else in the P1-P3 chain would catch a tonal-balance regression like that.
//
// Smoothing. Coefficients and the post-filter output-gain trim are both smoothed with a
// per-block linear ramp, the convention OutputGain uses for its own gain: setParams() only
// retargets; the next process()/processMono() call ramps the last SETTLED coefficients/gain to
// the new targets across that call's samples, landing exactly on target at the last sample, so a
// ramp always completes within the block it starts in (never spilling into a later one).

namespace cnpg::dsp {

template <typename SampleT> struct StringTapBuffers; // full definition: StringNetwork.h

// -----------------------------------------------------------------------------------------------
// The -18 dBFS per-string calibration constant (Task P1.9 step 3).
// -----------------------------------------------------------------------------------------------
//
// docs/plan.md Task P1.9 requires "single string at velocity 1.0 peaks at -18 dBFS at the pickup
// sum". PluckExciter already lands its BURST peak at exactly -18 dBFS at velocity 1.0
// (dsp/src/PluckExciter.cpp, kNominalPeakDb) -- but that is the displacement injected INTO the
// string, not what comes out of the pickup. Between the two sit the string's own loop losses and
// the pickup's differentiating bandpass, whose skirt is far below its 2.5 kHz resonance at any
// note's fundamental. Measured end to end (StringNetwork -> PickupTap, every other parameter at its
// default, MIDI 45 / A2 at velocity 1.0, 2 s render), that path delivers:
//
//   44.1 kHz: -42.80 dBFS     48 kHz: -42.82 dBFS     96 kHz: -42.74 dBFS
//
// -- i.e. it is remarkably rate-independent (0.08 dB spread), so ONE constant calibrates every
// supported rate. kNominalPickupTrimDb is that constant, rounded to 0.1 dB, and it is the default
// of outputGainDb below; plugin/src/Parameters.cpp reads it straight out of PickupTapParams{} for
// the APVTS default and centres the knob's +/-24 dB range on it, so the plugin's default state IS
// the calibrated state and the two cannot drift apart.
//
// What the constant is NOT: a claim that every note lands at -18 dBFS. The same sweep across the
// range measures -18.0 dBFS from MIDI 21 through 52 (flat to 0.04 dB -- the peak there is the
// pluck transient passing the tap, which barely depends on pitch), rising to about -12.1 dBFS
// around MIDI 64 where the string's harmonics line up best with the 2.5 kHz pickup resonance, then
// falling away to -27 .. -30 dBFS at MIDI 108 as fewer and fewer harmonics survive the loop losses.
// -18 dBFS is the NOMINAL the rest of the chain is gain-staged against (TriodeStage.h's drive
// calibration, the +16 dB multi-string summing budget), and the calibration reference is the
// specific documented scenario above -- not an automatic gain control.
inline constexpr float kNominalPickupTrimDb = 24.8f;

struct PickupTapParams {
    float resonanceHz = 2500.0f;               // RLC resonant frequency
    float q = 2.0f;                            // resonance Q (loading)
    float outputGainDb = kNominalPickupTrimDb; // post-sum trim onto the -18 dBFS per-string
                                               // nominal structure; see the block comment above
};

static_assert(std::is_trivially_copyable_v<PickupTapParams>,
              "PickupTapParams must stay trivially copyable for the realtime APVTS snapshot path.");

class PickupTap {
  public:
    // Message thread; may allocate. Sizes the internal mono scratch buffer for maxBlockSize and
    // seeds both the settled and target coefficients/gain from PickupTapParams{}'s own defaults,
    // so a process() call before any setParams() runs a sane, already-settled filter rather than
    // a degenerate one. Calls reset().
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Clears the biquad's own memory (as if no signal had ever passed through it)
    // and collapses any pending coefficient/gain ramp onto its target immediately, so a reset
    // instance is indistinguishable from a freshly prepared one carrying the same parameters.
    void reset() noexcept;

    // Realtime-safe; only retargets the ramp. resonanceHz and q are clamped into a safe range
    // (away from 0 Hz and from Nyquist; q away from 0) before being placed as a pole pair, so a
    // malformed or extreme parameter value can move the response but never yields NaN or an
    // unstable filter.
    void setParams(const PickupTapParams& p) noexcept;

    // Realtime-safe; never allocates. Sums taps.channel(i) for every string reporting
    // taps.isActive(i) into an internal mono scratch buffer, then runs that sum through the same
    // stage processMono() runs. Clamped to [0, min(maxBlockSize, taps.numSamples())] -- a caller
    // asking for more samples than taps actually holds (a short last block, or a StringNetwork
    // prepared with a smaller maxBlockSize than this PickupTap's own) gets exactly
    // taps.numSamples() written; samples in `out` beyond that are left untouched rather than read
    // past each string's valid stride.
    void process(const StringTapBuffers<Sample>& taps, Sample* out, int numSamples) noexcept;

    // Test-only seam. docs/plan.md section 2.8 locks process(taps, out, numSamples) as the real
    // consumer entry point, but a StringTapBuffers can only be produced by a real StringNetwork
    // driving actual waveguide-string physics -- there is no way to hand it a synthetic sine for
    // the [contract] frequency-response measurement. processMono() runs the identical RLC-biquad
    // + output-gain stage process() uses, directly over an already-mono buffer, so that
    // measurement (and the parameter-step / no-NaN check) can drive a controlled signal through
    // the exact code process() itself runs. Realtime-safe; never allocates.
    void processMono(const Sample* in, Sample* out, int numSamples) noexcept;

  private:
    struct Coeffs {
        double b0 = 0.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
    };

    static Coeffs computeCoeffs(float resonanceHz, float q, double sampleRate) noexcept;
    double biquadTick(double x, const Coeffs& c) noexcept;
    void runBlock(const Sample* in, Sample* out, int numSamples) noexcept;

    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;

    Coeffs currentCoeffs_{}; // last settled coefficients
    Coeffs targetCoeffs_{};  // coefficients requested by the most recent setParams()
    float currentGainLinear_ = 1.0f;
    float targetGainLinear_ = 1.0f;

    // Direct-Form-I biquad memory, kept in double for headroom at high q.
    double x1_ = 0.0;
    double x2_ = 0.0;
    double y1_ = 0.0;
    double y2_ = 0.0;

    std::vector<Sample> scratchMono_; // sized maxBlockSize_ in prepare(); process()'s active sum
};

} // namespace cnpg::dsp

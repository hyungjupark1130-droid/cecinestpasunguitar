#include "cnpg/dsp/WaveguideString.h"

#include "cnpg/dsp/Common.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace cnpg::dsp {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// Shortest loop period the topology supports, in samples: two rail reads (>= 2 each after the
// fractional-delay solve) plus the four dispersion allpasses plus the loss filter. f0 is clamped
// so the loop never runs shorter than this. At 44.1 kHz it caps f0 at 4410 Hz -- above MIDI 108
// (4186.01 Hz), so the tuning gate range is never clamped; a +2-semitone bend from the very top
// note at 44.1 kHz is (documented) clamped.
constexpr double kMinLoopPeriodSamples = 10.0;

// Rails are sized for kMinMidiNote detuned kPitchBendRangeSemitones DOWN, i.e. MIDI 19. Widening
// the bend range means recomputing this literal (std::exp2 is not constexpr), which is why
// Common.h calls the range a design-envelope constant.
static_assert(kMinMidiNote == 21 && kPitchBendRangeSemitones == 2.0f,
              "kSizingLowestF0Hz below is 440 * 2^((kMinMidiNote - kPitchBendRangeSemitones - 69) / 12); "
              "recompute it if either constant moves.");
constexpr double kSizingLowestF0Hz = 24.499714748859330; // 440 * 2^((19 - 69) / 12)

constexpr double kSmoothingTimeSeconds = 0.008; // per-sample one-pole for f0, bend, and material

// Fractional-delay design ranges, and the centre the integer part of the rail read is chosen
// around. These are about REACHABILITY, not taste: the integer part steps by exactly 1, so the
// realizable spans only tile the reals without gaps if the interpolator's PHASE delay covers a
// full sample across its admissible D range.
//
// Lagrange-3: phase delay is exactly 1 at D = 1 and exactly 2 at D = 2 (the taps degenerate to a
// single unit tap at both ends), at every frequency -- so [1, 2] tiles perfectly and no headroom
// is needed. It is also the widest range available: |H(w)| exceeds 1 outside it (already 1.0013
// at Nyquist for D = 2.001), which would make the interpolator ACTIVE and break the tier-2
// energy gate.
//
// Thiran-1: phase delay is compressed relative to D and the compression grows with frequency --
// at the shortest supported loop (w0 = 2*pi/10) the span D in [0.5, 1.5] covers only 0.929
// samples of phase delay, leaving a 0.071-sample unreachable gap per rail. Left unhandled that
// gap is worth ~24 cents at MIDI 108. The range is therefore widened to [0.4, 1.8], which covers
// the required [0.488, 1.585] at that worst-case frequency with margin, while keeping |a| <= 0.43
// so the allpass pole stays far from the unit circle. Thiran-1 is allpass for every D > 0, so
// widening costs no passivity.
constexpr double kLagrangeDelayMin = 1.0;
constexpr double kLagrangeDelayMax = 2.0;
constexpr double kLagrangeDelayMid = 1.5;
constexpr double kThiranDelayMin = 0.4;
constexpr double kThiranDelayMax = 1.8;
constexpr double kThiranDelayMid = 1.0;

double clampd(double v, double lo, double hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

// Phase delay of the factor (1 + r z^-1) at w: -arg(1 + r e^-jw) / w.
double zeroPhaseDelay(double r, double w, double sinw, double cosw) noexcept {
    return std::atan2(r * sinw, 1.0 + r * cosw) / w;
}

// Phase delay of the first-order allpass (a + z^-1) / (1 + a z^-1) at w. Equals (1 - a)/(1 + a)
// as w -> 0 and 1 exactly for a == 0 (where the allpass degenerates to a unit delay).
double allpassPhaseDelay(double a, double w, double sinw, double cosw) noexcept {
    return 1.0 - 2.0 * zeroPhaseDelay(a, w, sinw, cosw);
}

// Cubic (order-3) Lagrange interpolator taps for design delay D: h[k] = prod_{j != k} (D-j)/(k-j).
void lagrangeCoefficients(double d, double h[4]) noexcept {
    h[0] = (d - 1.0) * (d - 2.0) * (d - 3.0) / -6.0;
    h[1] = d * (d - 2.0) * (d - 3.0) / 2.0;
    h[2] = d * (d - 1.0) * (d - 3.0) / -2.0;
    h[3] = d * (d - 1.0) * (d - 2.0) / 6.0;
}

double lagrangePhaseDelay(const double h[4], double w, double sinw, double cosw) noexcept {
    double re = 0.0;
    double im = 0.0;
    double ck = 1.0; // cos(k w)
    double sk = 0.0; // sin(k w)
    for (int k = 0; k < 4; ++k) {
        re += h[k] * ck;
        im += h[k] * sk;
        const double cn = ck * cosw - sk * sinw;
        const double sn = sk * cosw + ck * sinw;
        ck = cn;
        sk = sn;
    }
    return std::atan2(im, re) / w;
}

// Storage matrix of the Lagrange FIR's LOSSLESS ORTHOGONAL EMBEDDING (docs/plan.md section 4.2
// wants a "closed-form quadratic storage" of the interpolator states). No DIAGONAL storage
// exists for this filter: a diagonal V requires ||h||_1 <= 1, and ||h||_1 = 1.25 already at
// D = 1.5. The construction used here instead is exact and closed form:
//
//   1. R(w) = 1 - |H(w)|^2 >= 0 is a self-reciprocal Laurent polynomial of degree 3, written
//      from the autocorrelation r_m = delta_m0 - sum_k h_k h_{k+m}.
//   2. In x = z + 1/z it is a cubic. Lagrange interpolators satisfy sum_k h_k = 1 exactly, so
//      H(1) = 1, R has a double root at z = 1 and the cubic therefore has the known root x = 2.
//      Deflating it leaves a QUADRATIC -- solved by the quadratic formula, no iteration.
//   3. Each x root gives a reciprocal z pair (z^2 - x z + 1 = 0); taking one root per pair plus
//      z = 1 yields the spectral factor G with |H|^2 + |G|^2 = 1, i.e. the FIR's lossless
//      2-output embedding.
//   4. With A the state shift and C_e = [C; C_G], the storage is the observability Gramian
//      P = sum_{k=0..2} (A^k)^T C_e^T C_e A^k, which for this shift structure is exactly the
//      Hankel Gramian below. V(s) = s^T P s then satisfies V(next) - V(now) = u^2 - y^2 - w^2
//      <= u^2 - y^2 for every input: the interpolator is dissipative with this storage, with the
//      deficit w^2 being precisely the energy the interpolator's lowpass droop removes.
//
// `out` is the upper triangle packed as [p00, p01, p02, p11, p12, p22].
void lagrangeStorageMatrix(const double h[4], double out[6]) noexcept {
    double r[4];
    r[0] = 1.0 - (h[0] * h[0] + h[1] * h[1] + h[2] * h[2] + h[3] * h[3]);
    r[1] = -(h[0] * h[1] + h[1] * h[2] + h[2] * h[3]);
    r[2] = -(h[0] * h[2] + h[1] * h[3]);
    r[3] = -(h[0] * h[3]);

    double g[4] = {0.0, 0.0, 0.0, 0.0};

    if (r[0] > 1e-15) {
        // Cubic in x = z + 1/z, then deflate the known root x = 2.
        const double c3 = r[3];
        const double c2 = r[2];
        const double c1 = r[1] - 3.0 * r[3];
        const double q1 = c2 + 2.0 * c3;
        const double q0 = c1 + 2.0 * q1;

        std::complex<double> xRoots[2];
        int numX = 0;
        if (std::fabs(c3) > 1e-15) {
            const std::complex<double> disc = std::sqrt(std::complex<double>(q1 * q1 - 4.0 * c3 * q0, 0.0));
            xRoots[0] = (-q1 + disc) / (2.0 * c3);
            xRoots[1] = (-q1 - disc) / (2.0 * c3);
            numX = 2;
        } else if (std::fabs(q1) > 1e-15) {
            xRoots[0] = std::complex<double>(-q0 / q1, 0.0);
            numX = 1;
        }

        std::complex<double> zRoots[3];
        int numZ = 0;
        zRoots[numZ++] = std::complex<double>(1.0, 0.0); // the z = 1 root R always carries
        for (int i = 0; i < numX; ++i) {
            const std::complex<double> s = std::sqrt(xRoots[i] * xRoots[i] - 4.0);
            const std::complex<double> za = 0.5 * (xRoots[i] + s);
            const std::complex<double> zb = 0.5 * (xRoots[i] - s);
            zRoots[numZ++] = (std::abs(za) <= std::abs(zb)) ? za : zb;
        }

        // Expand prod_i (1 - z_i w). Roots are real or come in conjugate pairs, so the expanded
        // coefficients are real up to rounding.
        std::complex<double> poly[4] = {std::complex<double>(1.0, 0.0), {}, {}, {}};
        for (int i = 0; i < numZ; ++i)
            for (int k = i + 1; k >= 1; --k)
                poly[k] -= zRoots[i] * poly[k - 1];

        double norm = 0.0;
        double raw[4];
        for (int k = 0; k < 4; ++k) {
            raw[k] = poly[k].real();
            norm += raw[k] * raw[k];
        }
        // The root set fixes |G|^2 up to a constant, so matching lag 0 matches every lag.
        const double scale = (norm > 0.0) ? std::sqrt(r[0] / norm) : 0.0;
        for (int k = 0; k < 4; ++k)
            g[k] = raw[k] * scale;
    }

    // Hankel Gramian over the extended output: P_ij = sum_{k=0..2} (h_{i+k} h_{j+k} + g.. ),
    // 1-based state indices i, j in {1, 2, 3}.
    double p[3][3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double acc = 0.0;
            for (int k = 0; k < 3; ++k) {
                const int ii = i + 1 + k;
                const int jj = j + 1 + k;
                if (ii < 4 && jj < 4)
                    acc += h[ii] * h[jj] + g[ii] * g[jj];
            }
            p[i][j] = acc;
        }
    }
    out[0] = p[0][0];
    out[1] = p[0][1];
    out[2] = p[0][2];
    out[3] = p[1][1];
    out[4] = p[1][2];
    out[5] = p[2][2];
}

// Scalar storage V = p * s^2 for the one-pole-plus-zero loss filter realized as
// y = b0 x + s, s' = b1 x + a1 y, i.e. state-space (A, B, C, D) = (a1, a1*b0 + b1, 1, b0).
// The dissipation inequality p(As + Bx)^2 - p s^2 <= x^2 - (s + b0 x)^2 is a quadratic in p:
// f(p) = -B^2 p^2 + K p - 1 >= 0. The parabola's vertex maximizes the margin.
double lossStorageWeight(double b0, double b1, double a1) noexcept {
    const double bb = a1 * b0 + b1;
    const double oneMinusA2 = 1.0 - a1 * a1;
    const double oneMinusB2 = 1.0 - b0 * b0;
    if (oneMinusA2 <= 0.0 || oneMinusB2 <= 0.0)
        return 0.0; // degenerate / non-contractive; caller only uses this for diagnostics
    if (std::fabs(bb) < 1e-12)
        return 1.0 / (oneMinusA2 * oneMinusB2);
    const double k = oneMinusA2 * oneMinusB2 + bb * bb - 2.0 * a1 * b0 * bb;
    const double vertex = k / (2.0 * bb * bb);
    return std::max(vertex, 1.0 / oneMinusA2);
}

} // namespace

// ------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void WaveguideString<SampleT>::prepare(double sampleRate, int /*maxBlockSize*/, FractionalDelayKind kind) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    kind_ = kind;

    // docs/plan.md section 2.1: size against max(hostSampleRate, kMaxDesignRateHz) so a 192 kHz
    // best-effort host never under-allocates the rails.
    const double sizingRate = std::max(sampleRate_, kMaxDesignRateHz);
    const double longestLoop = sizingRate / kSizingLowestF0Hz;
    // Each rail carries half the loop; +8 covers the post-write frame offset, the interpolator's
    // 3-sample reach, and the tap read one sample past the bridge.
    const int needed = static_cast<int>(std::ceil(longestLoop * 0.5)) + 8;
    int length = 4;
    while (length < needed)
        length *= 2;

    up_.assign(static_cast<std::size_t>(length), SampleT(0));
    dn_.assign(static_cast<std::size_t>(length), SampleT(0));
    mask_ = length - 1;

    f0Min_ = kSizingLowestF0Hz;
    f0Max_ = sampleRate_ / kMinLoopPeriodSamples;

    smoothingCoeff_ = 1.0 - std::exp(-1.0 / (kSmoothingTimeSeconds * sampleRate_));

    // The position crossfade is specified as a DURATION and realized as a whole number of samples,
    // so the same gesture takes the same time at every rate: 128 samples at 48 kHz, 118 at 44.1,
    // 256 at 96. Floored at one sample, which is a degenerate but well-defined fade.
    positionCrossfadeSamples_ = std::max(1, static_cast<int>(std::lround(kPositionCrossfadeSeconds * sampleRate_)));
    positionFadeStep_ = 1.0 / static_cast<double>(positionCrossfadeSamples_);

    reset();
}

template <typename SampleT> void WaveguideString<SampleT>::reset() noexcept {
    std::fill(up_.begin(), up_.end(), SampleT(0));
    std::fill(dn_.begin(), dn_.end(), SampleT(0));
    upWrite_ = 0;
    dnWrite_ = 0;

    for (auto& ap : dispersion_)
        ap.state = SampleT(0);
    for (auto& s : fracState_)
        s.state = SampleT(0);
    lossState_ = SampleT(0);

    bridgeOutgoing_ = SampleT(0);
    bridgeAccepted_ = SampleT(0);
    bridgeAcceptPending_ = false;
    // The liveness counters are transient state, so reset() clears them: a test that renders,
    // resets and renders again must not inherit the first render's evidence. bridgePortDriven_ is
    // a TOPOLOGY declaration and deliberately survives, exactly as lossBypassed_ does.
    bridgeReflectionTicks_ = 0;
    internalReflectionTicks_ = 0;

    // DISARM every moving-position anchor rather than snap it onto a remembered target. Same
    // contract as the smoothers below -- a reset instance is indistinguishable from a freshly
    // prepared one -- reached the honest way: after reset() there is no previous read position for
    // the next one to be continuous WITH, so the first request snaps onto whatever it asks for
    // instead of crossfading in from wherever the last note left the pickup. Carrying a remembered
    // anchor across a reset would make a re-init at a new pitch glide the tap for 128 samples over
    // a string whose rails were just zeroed -- inaudible, and still a difference from a fresh
    // instance that nothing would be able to justify.
    for (auto& anchor : tapAnchor_)
        anchor = PositionAnchor{};
    junctionAnchor_ = PositionAnchor{};

    // Snap every smoother onto its target: a reset instance must be indistinguishable from a
    // freshly prepared one (docs/plan.md section 4.1, "reset is idempotent and complete"). A
    // retune ramp in flight is exactly such a difference, so it is abandoned here rather than
    // allowed to keep gliding a string whose rails were just zeroed.
    retuneStepsRemaining_ = 0;
    retuneRatio_ = 1.0;
    retargetSmoothers();
    f0Smoothed_ = f0Target_;
    lossLowSmoothed_ = lossLowTarget_;
    lossHighSmoothed_ = lossHighTarget_;
    dispersionSmoothed_ = dispersionTarget_;
    smoothersSettled_ = true;

    updateCoefficients();
}

template <typename SampleT> void WaveguideString<SampleT>::retargetSmoothers() noexcept {
    const double bent =
        static_cast<double>(params_.f0Hz) * std::exp2(static_cast<double>(params_.bendSemitones) / 12.0);
    f0Target_ = clampd(bent, f0Min_, f0Max_);

    const double low = clampd(static_cast<double>(params_.stringMaterial.lossGainLow), 0.0, 1.0);
    const double high = clampd(static_cast<double>(params_.stringMaterial.lossGainHigh), 0.0, 1.0);
    const double disp = clampd(static_cast<double>(params_.stringMaterial.dispersionAmount), 0.0, 1.0);

    lossLowTarget_ =
        static_cast<double>(kLossGainLowMin) + low * static_cast<double>(kLossGainLowMax - kLossGainLowMin);
    lossHighTarget_ =
        static_cast<double>(kLossGainHighMin) + high * static_cast<double>(kLossGainHighMax - kLossGainHighMin);
    // Negative coefficient: high frequencies travel FASTER than low ones, which is the stiff
    // (sharp-stretched) partial series real strings have. A positive coefficient would flatten
    // the partials instead.
    dispersionTarget_ = -disp * static_cast<double>(kDispersionMaxCoeff);

    // A target that moves while a ramp is in flight (a pitch bend arriving during a retrigger)
    // re-aims the remaining steps at it rather than stranding the glide short of the new note.
    refreshRetuneRatio();
}

template <typename SampleT> void WaveguideString<SampleT>::refreshRetuneRatio() noexcept {
    if (retuneStepsRemaining_ <= 0)
        return;
    // Constant cents per sample: the ratio that takes the value where it IS to where it is going in
    // exactly the steps that are left. Recomputed rather than accumulated, so a mid-ramp retarget
    // cannot leave the glide aimed at a pitch nobody asked for.
    const double from = (f0Smoothed_ > 0.0) ? f0Smoothed_ : f0Target_;
    const double to = (f0Target_ > 0.0) ? f0Target_ : from;
    retuneRatio_ = std::pow(to / from, 1.0 / static_cast<double>(retuneStepsRemaining_));
}

template <typename SampleT> void WaveguideString<SampleT>::beginRetuneRamp(double rampSeconds) noexcept {
    const double samples = rampSeconds * sampleRate_;
    // At least one step, so "begin a ramp" always has a defined landing sample even at a
    // degenerate duration -- a one-sample ramp is an immediate retune, which is a legitimate thing
    // for a caller to ask for and is what the negative control in the P2.6 click gate uses.
    retuneStepsRemaining_ = (samples >= 1.0) ? static_cast<int>(std::lround(samples)) : 1;
    refreshRetuneRatio();
    smoothersSettled_ = false;
}

template <typename SampleT> void WaveguideString<SampleT>::setParams(const WaveguideStringParams& p) noexcept {
    params_ = p;
    const double oldF0 = f0Target_;
    const double oldLow = lossLowTarget_;
    const double oldHigh = lossHighTarget_;
    const double oldDisp = dispersionTarget_;
    retargetSmoothers();
    if (f0Target_ != oldF0 || lossLowTarget_ != oldLow || lossHighTarget_ != oldHigh || dispersionTarget_ != oldDisp)
        smoothersSettled_ = false;
}

template <typename SampleT>
void WaveguideString<SampleT>::setAnalyticTuningCompensation(float delaySamplesCorrection) noexcept {
    analyticCorrectionSamples_ = static_cast<double>(delaySamplesCorrection);
    updateCoefficients();
}

template <typename SampleT>
void WaveguideString<SampleT>::loadCalibrationTable(const float* centsByMidiNote, int firstMidiNote, int count) {
    if (centsByMidiNote == nullptr || count <= 0) {
        calibrationCents_.clear();
        calibrationFirstMidiNote_ = 0;
        return;
    }
    calibrationCents_.assign(centsByMidiNote, centsByMidiNote + count);
    calibrationFirstMidiNote_ = firstMidiNote;
    // Deliberately does NOT select the table: the P1.4 brief specifies "loadCalibrationTable
    // stores the table and is a no-op source until P2 selects it" (Task P2.7).
}

template <typename SampleT> void WaveguideString<SampleT>::setLossBypassed(bool bypass) noexcept {
    if (bypass == lossBypassed_)
        return;
    lossBypassed_ = bypass;
    lossState_ = SampleT(0);
    updateCoefficients();
}

template <typename SampleT> void WaveguideString<SampleT>::setBridgePortDriven(bool driven) noexcept {
    if (driven == bridgePortDriven_)
        return;
    bridgePortDriven_ = driven;
    updateCoefficients(); // the loop is one sample longer (or shorter) from this call on
}

// ------------------------------------------------------------------------------------------
// loop-length solve
// ------------------------------------------------------------------------------------------

template <typename SampleT> void WaveguideString<SampleT>::advanceSmoothers() noexcept {
    auto approach = [this](double& value, double target) noexcept {
        const double delta = target - value;
        if (std::fabs(delta) <= std::fabs(target) * 1e-12 + 1e-15) {
            const bool moved = value != target;
            value = target;
            return moved;
        }
        value += smoothingCoeff_ * delta;
        return true;
    };

    bool moving = false;
    if (retuneStepsRemaining_ > 0) {
        // The retune ramp OWNS f0 while it is in flight (Task P2.6). Geometric step, and the last
        // one is written as the target itself rather than as the product of nine ratios -- so the
        // ramp LANDS, exactly, at a sample the caller can name, which is the whole reason it exists
        // beside the one-pole rather than instead of it.
        --retuneStepsRemaining_;
        f0Smoothed_ = (retuneStepsRemaining_ == 0) ? f0Target_ : f0Smoothed_ * retuneRatio_;
        moving = true;
    } else {
        moving |= approach(f0Smoothed_, f0Target_);
    }
    moving |= approach(lossLowSmoothed_, lossLowTarget_);
    moving |= approach(lossHighSmoothed_, lossHighTarget_);
    moving |= approach(dispersionSmoothed_, dispersionTarget_);
    smoothersSettled_ = !moving;
}

template <typename SampleT> void WaveguideString<SampleT>::updateCoefficients() noexcept {
    if (up_.empty())
        return; // not prepared yet; setParams/setLossBypassed before prepare() must stay inert

    const double f0 = clampd(f0Smoothed_, f0Min_, f0Max_);
    const double period = sampleRate_ / f0;
    const double w0 = kTwoPi * f0 / sampleRate_;
    const double sinw = std::sin(w0);
    const double cosw = std::cos(w0);

    // --- loop loss filter: one pole plus one zero, |H| <= max(gLow, gHigh) everywhere ---------
    double b0 = 1.0;
    double b1 = 0.0;
    double a1 = 0.0;
    double tauLoss = 0.0;
    if (!lossBypassed_) {
        a1 = static_cast<double>(kLossFilterPoleZ);
        b0 = 0.5 * (lossLowSmoothed_ * (1.0 - a1) + lossHighSmoothed_ * (1.0 + a1));
        b1 = 0.5 * (lossLowSmoothed_ * (1.0 - a1) - lossHighSmoothed_ * (1.0 + a1));
        tauLoss = zeroPhaseDelay(b1 / b0, w0, sinw, cosw) - zeroPhaseDelay(-a1, w0, sinw, cosw);
    }
    lossB0_ = static_cast<SampleT>(b0);
    lossB1_ = static_cast<SampleT>(b1);
    lossA1_ = static_cast<SampleT>(a1);

    // --- dispersion allpass chain, clamped so the loop always has room for both rail reads ----
    double disp = dispersionSmoothed_;
    {
        // Each rail read needs a realized delay of at least kMinRailSpan samples.
        const double budget = period - tauLoss - 2.0 * kMinRailSpan;
        const double tauMax = budget / static_cast<double>(kDispersionOrder);
        if (allpassPhaseDelay(disp, w0, sinw, cosw) > tauMax) {
            // Invert tau = 1 - 2 atan2(a sinw, 1 + a cosw)/w for the most negative admissible a.
            const double theta = 0.5 * w0 * (1.0 - tauMax);
            const double t = std::tan(clampd(theta, -1.5, 1.5));
            const double denom = sinw - t * cosw;
            if (std::fabs(denom) > 1e-12)
                disp = clampd(std::max(disp, t / denom), -0.95, 0.0);
            // else: the inversion is degenerate. Leaving `disp` alone costs a little tuning
            // accuracy at one note; snapping it to 0 would be an audible one-sample jump.
            // railSpan is separately floored at kMinRailSpan, so nothing can run out of rail.
        }
    }
    const double tauDispersion = static_cast<double>(kDispersionOrder) * allpassPhaseDelay(disp, w0, sinw, cosw);
    for (auto& ap : dispersion_)
        ap.coeff = static_cast<SampleT>(disp);

    // --- split the remainder evenly between the two rail reads --------------------------------
    // Clamped from ABOVE as well: analyticCorrectionSamples_ is caller-supplied and subtracted, so
    // a negative correction would otherwise grow the rail read past the allocation and alias
    // against fresh writes -- silently wrong output rather than a crash. The margin covers the
    // +1 frame offset, the interpolator's 3-sample reach and the tap that reads one past the
    // bridge.
    const double maxRailSpan = static_cast<double>(up_.size()) - 6.0;
    // The bridge seam's own sample is subtracted here, alongside the filters' phase delays and the
    // caller's probe offset, because it is exactly the same kind of quantity: loop delay that is
    // not rail. See setBridgePortDriven() -- StringNetwork's gather/scatter/accept ordering makes
    // the external reflection arrive one sample late, at every rate and every note.
    const double seamDelay = bridgePortDriven_ ? kBridgeSeamDelaySamples : 0.0;
    const double railSpan = clampd(0.5 * (period - tauDispersion - tauLoss - analyticCorrectionSamples_ - seamDelay),
                                   kMinRailSpan, maxRailSpan);

    const bool lagrange = (kind_ == FractionalDelayKind::Lagrange3);
    const double delayMin = lagrange ? kLagrangeDelayMin : kThiranDelayMin;
    const double delayMax = lagrange ? kLagrangeDelayMax : kThiranDelayMax;
    const double delayMid = lagrange ? kLagrangeDelayMid : kThiranDelayMid;

    int base = std::max(1, static_cast<int>(std::lround(railSpan - delayMid)));
    double solved =
        clampd(solveFractionalDelay(railSpan - static_cast<double>(base), w0, sinw, cosw, delayMin, delayMax), delayMin,
               delayMax);
    {
        // If the clamp bound, the requested span was outside this base's reachable window; try
        // the neighbouring base once and keep whichever lands closer. Deliberately NOT a loop:
        // re-solving until "in range" can ping-pong forever across an unreachable gap.
        const double residual = railSpan - (static_cast<double>(base) + interpolatorPhaseDelay(solved, w0, sinw, cosw));
        if (std::fabs(residual) > 1e-9) {
            const int alternative = (residual > 0.0) ? base + 1 : base - 1;
            if (alternative >= 1) {
                const double altSolved = clampd(solveFractionalDelay(railSpan - static_cast<double>(alternative), w0,
                                                                     sinw, cosw, delayMin, delayMax),
                                                delayMin, delayMax);
                const double altResidual =
                    railSpan - (static_cast<double>(alternative) + interpolatorPhaseDelay(altSolved, w0, sinw, cosw));
                if (std::fabs(altResidual) < std::fabs(residual)) {
                    base = alternative;
                    solved = altSolved;
                }
            }
        }
    }
    railBase_ = base;
    fractionalDelay_ = solved;

    if (lagrange) {
        double h[4];
        lagrangeCoefficients(fractionalDelay_, h);
        for (int k = 0; k < 4; ++k)
            lagrangeCoeff_[k] = static_cast<SampleT>(h[k]);
        realizedRailSpan_ = static_cast<double>(railBase_) + lagrangePhaseDelay(h, w0, sinw, cosw);
    } else {
        const double a = (1.0 - fractionalDelay_) / (1.0 + fractionalDelay_);
        for (auto& s : fracState_)
            s.coeff = static_cast<SampleT>(a);
        realizedRailSpan_ = static_cast<double>(railBase_) + allpassPhaseDelay(a, w0, sinw, cosw);
    }

    // See WaveguideString.h: Lagrange3's rail stays live out to railBase_ + 3, so the continuous
    // realized span is addressable; Thiran1's rail is consumed at railBase_, so anything past it
    // would be written or read after it has already left the loop.
    positionSpan_ = lagrange ? realizedRailSpan_ : std::max(0.0, static_cast<double>(railBase_ - 1));

    // The REALIZED total, seam included, so realizedLoopDelaySamples() still equals fs / f0 to
    // solver precision whether or not a bridge port is driving the string.
    realizedLoopDelay_ = 2.0 * realizedRailSpan_ + tauDispersion + tauLoss + seamDelay;
    energyCacheValid_ = false;
}

template <typename SampleT>
double WaveguideString<SampleT>::interpolatorPhaseDelay(double d, double w, double sinw, double cosw) const noexcept {
    if (kind_ == FractionalDelayKind::Lagrange3) {
        double h[4];
        lagrangeCoefficients(d, h);
        return lagrangePhaseDelay(h, w, sinw, cosw);
    }
    return allpassPhaseDelay((1.0 - d) / (1.0 + d), w, sinw, cosw);
}

template <typename SampleT>
double WaveguideString<SampleT>::solveFractionalDelay(double target, double w, double sinw, double cosw, double lo,
                                                      double hi) const noexcept {
    // Fixed point on d <- d + (target - phaseDelay(d)). d(phaseDelay)/dd is 1 to within a few
    // percent over the admissible ranges, so this contracts hard; 12 iterations with an early
    // exit reaches the double-precision floor.
    double d = clampd(target, lo - 0.25, hi + 0.25);
    for (int i = 0; i < 12; ++i) {
        const double step = target - interpolatorPhaseDelay(d, w, sinw, cosw);
        d += step;
        if (std::fabs(step) < 1e-13)
            break;
    }
    return d;
}

// ------------------------------------------------------------------------------------------
// rail access
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void WaveguideString<SampleT>::railDeposit(std::vector<SampleT>& rail, int writeIndex, double d,
                                           SampleT value) noexcept {
    const double floored = std::floor(d);
    const int i0 = static_cast<int>(floored);
    const auto frac = static_cast<SampleT>(d - floored);
    const int size = static_cast<int>(rail.size());
    // Amplitude-complementary linear weights (g1 + g2 = 1), per docs/plan.md section 2.4.
    rail[static_cast<std::size_t>((writeIndex - i0 + 2 * size) & mask_)] += value * (SampleT(1) - frac);
    rail[static_cast<std::size_t>((writeIndex - i0 - 1 + 2 * size) & mask_)] += value * frac;
}

template <typename SampleT>
SampleT WaveguideString<SampleT>::railInterpolate(const std::vector<SampleT>& rail, int writeIndex,
                                                  double d) const noexcept {
    const double floored = std::floor(d);
    const int i0 = static_cast<int>(floored);
    const auto frac = static_cast<SampleT>(d - floored);
    const int size = static_cast<int>(rail.size());
    const SampleT a = rail[static_cast<std::size_t>((writeIndex - i0 + 2 * size) & mask_)];
    const SampleT b = rail[static_cast<std::size_t>((writeIndex - i0 - 1 + 2 * size) & mask_)];
    return a * (SampleT(1) - frac) + b * frac;
}

template <typename SampleT>
SampleT WaveguideString<SampleT>::railFractionalRead(const std::vector<SampleT>& rail, int writeIndex) const noexcept {
    const int size = static_cast<int>(rail.size());
    if (kind_ == FractionalDelayKind::Lagrange3) {
        SampleT acc = SampleT(0);
        for (int k = 0; k < kLagrangeTaps; ++k)
            acc += lagrangeCoeff_[k] * rail[static_cast<std::size_t>((writeIndex - railBase_ - k + 2 * size) & mask_)];
        return acc;
    }
    return rail[static_cast<std::size_t>((writeIndex - railBase_ + 2 * size) & mask_)];
}

template <typename SampleT>
SampleT WaveguideString<SampleT>::railSampleAtDelay(bool upRail, int delaySamples) const noexcept {
    const std::vector<SampleT>& rail = upRail ? up_ : dn_;
    const int size = static_cast<int>(rail.size());
    // Delays at or past the rail length alias back onto live slots, so they are reported as 0
    // rather than as whatever the wrap lands on: a caller probing "is anything past the window"
    // must never be handed a live sample by an out-of-range query.
    if (size == 0 || delaySamples < 1 || delaySamples >= size)
        return SampleT(0);
    const int writeIndex = upRail ? upWrite_ : dnWrite_;
    return rail[static_cast<std::size_t>((writeIndex - delaySamples + 2 * size) & mask_)];
}

template <typename SampleT> void WaveguideString<SampleT>::injectAt(float position01, SampleT excitation) noexcept {
    // No anchor and no crossfade, deliberately: the exciter's position is latched at note-on and
    // never modulated, so there is no motion for the machinery below to make click-free.
    const double p = static_cast<double>(std::clamp(position01, 0.0f, 1.0f));
    const SampleT half = excitation * SampleT(0.5);
    railDeposit(up_, upWrite_, upDelayAt(p), half);
    railDeposit(dn_, dnWrite_, dnDelayAt(p), half);
}

// ------------------------------------------------------------------------------------------
// moving positions: the dual-anchor amplitude-complementary crossfade (see the header)
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void WaveguideString<SampleT>::retargetAnchor(PositionAnchor& anchor, double position01) noexcept {
    if (!anchor.armed) {
        // First read after reset(): there is no previous position to be continuous with, so this
        // one IS the anchor. Snapping here is what makes a freshly reset string read where it is
        // asked to rather than crossfading in from a default.
        anchor.anchor = position01;
        anchor.pending = position01;
        anchor.fade = 0.0;
        anchor.fading = false;
        anchor.armed = true;
        return;
    }
    if (anchor.fading)
        return; // `pending` is FROZEN for the length of the fade -- which is what makes this
                // function idempotent inside one sample, and therefore what lets the junction
                // seam's read and write agree on the weights without passing them between the two.
    if (std::fabs(position01 - anchor.anchor) > static_cast<double>(kPositionAnchorThreshold01)) {
        anchor.pending = position01;
        anchor.fade = 0.0; // g2 == 0: the fade's first sample reads exactly where the last one did
        anchor.fading = true;
    }
}

template <typename SampleT> void WaveguideString<SampleT>::advanceAnchor(PositionAnchor& anchor) noexcept {
    if (!anchor.fading)
        return;
    if (anchor.fade >= 1.0) {
        // The previous sample was read entirely at `pending`, so committing it now moves the
        // effective read position by exactly nothing. Committing on the sample that REACHES 1.0
        // instead would skip the g2 == 1 sample and leave the last step of the ramp uncrossfaded.
        anchor.anchor = anchor.pending;
        anchor.fade = 0.0;
        anchor.fading = false;
        return; // re-arms on the next retarget, which is how a fast sweep chains segments
    }
    anchor.fade = std::min(1.0, anchor.fade + positionFadeStep_);
}

template <typename SampleT>
typename WaveguideString<SampleT>::PositionCrossfade
WaveguideString<SampleT>::describeAnchor(const PositionAnchor& anchor) noexcept {
    PositionCrossfade out;
    out.anchor01 = static_cast<float>(anchor.anchor);
    out.pending01 = static_cast<float>(anchor.fading ? anchor.pending : anchor.anchor);
    out.crossfade01 = static_cast<float>(anchor.fading ? anchor.fade : 0.0);
    out.armed = anchor.armed;
    return out;
}

template <typename SampleT>
typename WaveguideString<SampleT>::PositionCrossfade
WaveguideString<SampleT>::tapCrossfade(int tapSlot) const noexcept {
    if (tapSlot < 0 || tapSlot >= kMaxTapsPerString)
        return PositionCrossfade{};
    return describeAnchor(tapAnchor_[static_cast<std::size_t>(tapSlot)]);
}

template <typename SampleT>
typename WaveguideString<SampleT>::PositionCrossfade WaveguideString<SampleT>::junctionCrossfade() const noexcept {
    return describeAnchor(junctionAnchor_);
}

template <typename SampleT> SampleT WaveguideString<SampleT>::tapAtPosition(double position01) const noexcept {
    return railInterpolate(up_, upWrite_, upDelayAt(position01)) +
           railInterpolate(dn_, dnWrite_, dnDelayAt(position01));
}

template <typename SampleT> SampleT WaveguideString<SampleT>::readTapAt(int tapSlot, float position01) noexcept {
    if (tapSlot < 0 || tapSlot >= kMaxTapsPerString)
        return SampleT(0);
    PositionAnchor& anchor = tapAnchor_[static_cast<std::size_t>(tapSlot)];
    retargetAnchor(anchor, static_cast<double>(std::clamp(position01, 0.0f, 1.0f)));
    // The not-fading path is the plain single-anchor read, not a two-anchor mix weighted (1, 0).
    // Both are exact in IEEE-754, and this one is what keeps a static-position render bit-identical
    // to the pre-P2.3 arithmetic -- which is what leaves the layer-(b) goldens unmoved.
    if (!anchor.fading)
        return tapAtPosition(anchor.anchor);
    const auto g2 = static_cast<SampleT>(anchor.fade);
    const SampleT g1 = SampleT(1) - g2;
    return g1 * tapAtPosition(anchor.anchor) + g2 * tapAtPosition(anchor.pending);
}

template <typename SampleT>
void WaveguideString<SampleT>::readJunctionInputs(float position01, SampleT& fromNut, SampleT& fromBridge) noexcept {
    retargetAnchor(junctionAnchor_, static_cast<double>(std::clamp(position01, 0.0f, 1.0f)));
    const PositionAnchor& anchor = junctionAnchor_;
    if (!anchor.fading) {
        fromNut = railInterpolate(up_, upWrite_, upDelayAt(anchor.anchor));
        fromBridge = railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.anchor));
        return;
    }
    const auto g2 = static_cast<SampleT>(anchor.fade);
    const SampleT g1 = SampleT(1) - g2;
    fromNut = g1 * railInterpolate(up_, upWrite_, upDelayAt(anchor.anchor)) +
              g2 * railInterpolate(up_, upWrite_, upDelayAt(anchor.pending));
    fromBridge = g1 * railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.anchor)) +
                 g2 * railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.pending));
}

template <typename SampleT>
void WaveguideString<SampleT>::writeJunctionOutputs(float position01, SampleT toBridge, SampleT toNut) noexcept {
    // Idempotent given the position the read was handed, so this recovers exactly the weights the
    // read used without either call having to carry them across.
    retargetAnchor(junctionAnchor_, static_cast<double>(std::clamp(position01, 0.0f, 1.0f)));
    const PositionAnchor& anchor = junctionAnchor_;

    // Deposit only the DIFFERENCE the junction introduces, re-reading the same points through the
    // same functional rather than caching them: a transparent junction is then bit-exactly a no-op,
    // mid-crossfade included, because the re-read reproduces the read's arithmetic exactly.
    // BOTH rails are re-read before EITHER is deposited into -- the two anchors of a fade can land
    // on overlapping slots, and depositing between the reads would let the up rail's deposit
    // contaminate the dn rail's read.
    if (!anchor.fading) {
        const SampleT incomingUp = railInterpolate(up_, upWrite_, upDelayAt(anchor.anchor));
        const SampleT incomingDn = railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.anchor));
        railDeposit(up_, upWrite_, upDelayAt(anchor.anchor), toBridge - incomingUp);
        railDeposit(dn_, dnWrite_, dnDelayAt(anchor.anchor), toNut - incomingDn);
        return;
    }
    const auto g2 = static_cast<SampleT>(anchor.fade);
    const SampleT g1 = SampleT(1) - g2;
    const SampleT incomingUp = g1 * railInterpolate(up_, upWrite_, upDelayAt(anchor.anchor)) +
                               g2 * railInterpolate(up_, upWrite_, upDelayAt(anchor.pending));
    const SampleT incomingDn = g1 * railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.anchor)) +
                               g2 * railInterpolate(dn_, dnWrite_, dnDelayAt(anchor.pending));
    // The EXACT TRANSPOSE of the read above: same two anchors, same two weights. That adjointness
    // is what carries the moving seam's passivity (see the header's energy identity), and it is the
    // whole reason the write may not simply deposit at the requested position.
    const SampleT differenceUp = toBridge - incomingUp;
    const SampleT differenceDn = toNut - incomingDn;
    railDeposit(up_, upWrite_, upDelayAt(anchor.anchor), g1 * differenceUp);
    railDeposit(up_, upWrite_, upDelayAt(anchor.pending), g2 * differenceUp);
    railDeposit(dn_, dnWrite_, dnDelayAt(anchor.anchor), g1 * differenceDn);
    railDeposit(dn_, dnWrite_, dnDelayAt(anchor.pending), g2 * differenceDn);
}

template <typename SampleT> void WaveguideString<SampleT>::railAcceptFromBridge(SampleT reflected) noexcept {
    bridgeAccepted_ = reflected;
    bridgeAcceptPending_ = true;
}

// ------------------------------------------------------------------------------------------
// per-sample loop
// ------------------------------------------------------------------------------------------

template <typename SampleT> SampleT WaveguideString<SampleT>::runLoopChain(SampleT x) noexcept {
    for (auto& ap : dispersion_)
        x = ap.process(x);
    if (!lossBypassed_) {
        const SampleT y = lossB0_ * x + lossState_;
        lossState_ = lossB1_ * x + lossA1_ * y;
        x = y;
    }
    return x;
}

template <typename SampleT> void WaveguideString<SampleT>::tick() noexcept {
    if (!smoothersSettled_) {
        advanceSmoothers();
        updateCoefficients();
    }

    // The ONE place a position crossfade advances. Putting it here rather than inside the reads is
    // what makes those reads idempotent within a sample -- so the junction seam's read and write
    // cannot disagree about the weights, and so a caller that probes a tap twice does not
    // accidentally run the fade at double speed.
    for (auto& anchor : tapAnchor_)
        advanceAnchor(anchor);
    advanceAnchor(junctionAnchor_);

    SampleT toBridge = railFractionalRead(up_, upWrite_);
    SampleT toNut = railFractionalRead(dn_, dnWrite_);
    if (kind_ == FractionalDelayKind::Thiran1) {
        toBridge = fracState_[0].process(toBridge);
        toNut = fracState_[1].process(toNut);
    }

    bridgeOutgoing_ = runLoopChain(toBridge);

    // Junction liveness (Task P2.4). The fallback branch is not an error here -- an isolated
    // WaveguideString is SUPPOSED to terminate itself, and P1 shipped exactly that -- but inside a
    // StringNetwork with a port attached it means the port did not tick, which degrades the
    // instrument to six uncoupled strings without failing anything. Counting both branches is what
    // makes that assertable from outside.
    const bool accepted = bridgeAcceptPending_;
    const SampleT bridgeIncoming = accepted ? bridgeAccepted_ : static_cast<SampleT>(-bridgeOutgoing_);
    bridgeAcceptPending_ = false;
    if (accepted)
        ++bridgeReflectionTicks_;
    else
        ++internalReflectionTicks_;

    up_[static_cast<std::size_t>(upWrite_)] = static_cast<SampleT>(-toNut);
    dn_[static_cast<std::size_t>(dnWrite_)] = bridgeIncoming;
    upWrite_ = (upWrite_ + 1) & mask_;
    dnWrite_ = (dnWrite_ + 1) & mask_;
}

template <typename SampleT> double WaveguideString<SampleT>::realizedLoopDelaySamples() const noexcept {
    return realizedLoopDelay_;
}

// ------------------------------------------------------------------------------------------
// energy storage functional
// ------------------------------------------------------------------------------------------

template <typename SampleT> void WaveguideString<SampleT>::refreshEnergyCache() const noexcept {
    if (kind_ == FractionalDelayKind::Lagrange3) {
        double h[4];
        for (int k = 0; k < kLagrangeTaps; ++k)
            h[k] = static_cast<double>(lagrangeCoeff_[k]);
        lagrangeStorageMatrix(h, lagrangeStorage_);
    }
    lossStorage_ = lossBypassed_ ? 0.0
                                 : lossStorageWeight(static_cast<double>(lossB0_), static_cast<double>(lossB1_),
                                                     static_cast<double>(lossA1_));
    energyCacheValid_ = true;
}

template <typename SampleT> double WaveguideString<SampleT>::energyEstimate() const noexcept {
    if (!energyCacheValid_)
        refreshEnergyCache();

    const int size = static_cast<int>(up_.size());

    // Rail window: delays 1..railBase_. The sample at delay railBase_ is the one the fractional
    // read consumes on the next tick; everything past it either belongs to the interpolator's
    // state (Lagrange) or has already left the loop.
    double railEnergy = 0.0;
    for (int d = 1; d <= railBase_; ++d) {
        const double u = static_cast<double>(up_[static_cast<std::size_t>((upWrite_ - d + 2 * size) & mask_)]);
        const double v = static_cast<double>(dn_[static_cast<std::size_t>((dnWrite_ - d + 2 * size) & mask_)]);
        railEnergy += u * u + v * v;
    }

    double stateEnergy = 0.0;
    if (kind_ == FractionalDelayKind::Lagrange3) {
        // Interpolator states are the rail samples at delays railBase_+1 .. railBase_+3.
        const int writeIndices[2] = {upWrite_, dnWrite_};
        const std::vector<SampleT>* rails[2] = {&up_, &dn_};
        for (int r = 0; r < 2; ++r) {
            double s[3];
            for (int k = 0; k < 3; ++k)
                s[k] = static_cast<double>(
                    (*rails[r])[static_cast<std::size_t>((writeIndices[r] - railBase_ - 1 - k + 2 * size) & mask_)]);
            stateEnergy += lagrangeStorage_[0] * s[0] * s[0] + lagrangeStorage_[3] * s[1] * s[1] +
                           lagrangeStorage_[5] * s[2] * s[2] +
                           2.0 * (lagrangeStorage_[1] * s[0] * s[1] + lagrangeStorage_[2] * s[0] * s[2] +
                                  lagrangeStorage_[4] * s[1] * s[2]);
        }
    } else {
        for (const auto& s : fracState_) {
            const double a = static_cast<double>(s.coeff);
            const double v = static_cast<double>(s.state);
            stateEnergy += v * v / (1.0 - a * a);
        }
    }

    for (const auto& ap : dispersion_) {
        const double a = static_cast<double>(ap.coeff);
        const double v = static_cast<double>(ap.state);
        stateEnergy += v * v / (1.0 - a * a);
    }

    const double ls = static_cast<double>(lossState_);
    stateEnergy += lossStorage_ * ls * ls;

    // THE SEAM REGISTER (Task P2.4). With an external bridge port driving the string, the wave the
    // loop chain produced on the last tick is sitting in bridgeOutgoing_ waiting to be scattered:
    // it has left the rails and has not yet arrived anywhere, so it is a state-bearing element of
    // exactly one sample and it carries exactly one sample's worth of energy. Left out, the tier-2
    // functional would fluctuate by whatever is in flight -- of order the signal itself, i.e.
    // ~1e-1 relative against a 1e-9 bound. With the INTERNAL termination it must NOT be counted:
    // tick() writes it into the down rail in the same call, where the rail sum above already has
    // it, and adding it here would double-count.
    if (bridgePortDriven_) {
        const double inFlight = static_cast<double>(bridgeOutgoing_);
        stateEnergy += inFlight * inFlight;
    }

    const double impedance = static_cast<double>(portImpedance());
    return (railEnergy + stateEnergy) / (2.0 * impedance);
}

template <typename SampleT>
typename WaveguideString<SampleT>::LossStorageProbe WaveguideString<SampleT>::lossStorageProbe() const noexcept {
    if (!energyCacheValid_)
        refreshEnergyCache();
    LossStorageProbe probe;
    probe.b0 = static_cast<double>(lossB0_);
    probe.b1 = static_cast<double>(lossB1_);
    probe.a1 = static_cast<double>(lossA1_);
    probe.storageWeight = lossStorage_;
    probe.bypassed = lossBypassed_;
    return probe;
}

template class WaveguideString<float>;  // realtime path
template class WaveguideString<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

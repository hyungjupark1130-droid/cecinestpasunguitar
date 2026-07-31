#include "cnpg/dsp/TriodeStage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

// -------------------------------------------------------------------------------------------
// Circuit constants. Rp and Rnext are LOCKED by docs/plan.md section 2.9 / the task brief
// ("100k plate load ... 1M next-stage load"). Vb (plate supply) and Rk (bypassed cathode bias
// resistor) are this module's own implementation choice, not pinned by the brief: 250 V / 1.5 k
// is the textbook classic-ECC83-gain-stage operating point (paired with the brief's own 100k
// plate load, this is the standard example bias point quoted for a 12AX7/ECC83 first gain stage
// in tube-amp design references, e.g. Blencowe, "Designing Valve Preamps for Guitar and Bass"),
// landing Ip0 in the ~1 mA / Vp0 in the ~100-150 V range typical of a real ECC83 preamp stage.
// kGridSourceResistanceOhms is likewise this module's own choice: the assumed Thevenin
// drive-source resistance the soft grid-current clamp (rgi) acts against (see TriodeStage.h's
// "grid-current clamp" section) -- 68k is a conventional grid-stopper/coupling-resistor value
// seen in real guitar-preamp input stages, deliberately distinct from kPlateLoadOhms so the two
// are never confusable in code review.
constexpr double kPlateSupplyVoltsDefault = 250.0;
constexpr double kPlateLoadOhms = 100000.0;      // LOCKED: brief's "100k plate load"
constexpr double kNextStageLoadOhms = 1000000.0; // LOCKED: brief's "1M next-stage load"
constexpr double kCathodeResistorOhms = 1500.0;  // bypassed cathode bias resistor
constexpr double kGridSourceResistanceOhms = 68000.0;

// vin (grid-referred input volts) -> table domain. Generous vs. any plausible drive*sample
// combination (see waveshapeOne()'s kGridVoltsPerFullScale): even drive at its clamp ceiling
// times a +12 dB-over-full-scale sample stays an order of magnitude inside this domain, so
// process() never rides the table's edge-clamp region in ordinary use -- the edges exist purely
// as a safety backstop against pathological input, not as part of the intended transfer shape.
constexpr double kDomainHalfWidthVolts = 60.0;
constexpr int kTableSize = 4097; // odd: (kTableSize-1)/2 is an exact-integer node at vin == 0

// vin -> grid-volts calibration for `drive`. A -18 dBFS single-string nominal sample (~0.1259)
// at drive=0.5 (unity pre-gain) produces a ~0.63 V grid swing -- comfortably inside the curve's
// gentle near-linear region, below the grid-conduction knee (Vgk_raw == 0 at vin == +Vk0, and
// Vk0 for the constants above lands in the ~1-2 V range). At drive=1.0 (2x pre-gain) the same
// nominal sample produces a ~1.26 V swing, close enough to that knee to show real, audible
// curvature growth; multi-string summing up to the +16 dB headroom budget (docs/plan.md section
// 2.9) pushes well past it. This is the one genuinely free "taste" constant in the whole chain --
// everything else in the vin -> normalized-output path is either a locked circuit value or is
// self-normalized by buildTransferTable()'s own unity-small-signal-gain step (see TriodeStage.h).
constexpr double kGridVoltsPerFullScale = 5.0;

constexpr float kMinDrive = 0.0f;
constexpr float kMaxDrive = 2.0f; // generous vs. the documented 0..1 nominal range; guards
                                  // against host automation overshoot without special-casing it
constexpr float kMinOutputTrimDb = -60.0f;
constexpr float kMaxOutputTrimDb = 24.0f;

float clampFinite(float v, float lo, float hi) noexcept {
    if (!(v == v)) // NaN guard: NaN compares unequal to itself
        return lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

float dbToLinear(float gainDb) noexcept { return std::pow(10.0f, gainDb / 20.0f); }

// softplus(x) = ln(1+exp(x)), guarded against exp() overflow. For x > 40, exp(x) alone would
// already dwarf 1.0 well past double's ~15-17 significant digits, so ln(1+exp(x)) == x to within
// less than 1e-17 relative error -- returning x directly avoids ever evaluating exp() on an
// argument that could overflow (double overflows exp() around x ~ 709), which matters here
// because buildTransferTable() probes grid voltages well outside any single "normal" operating
// range while searching the AC load line's bisection bracket.
double korenSoftplus(double x) noexcept {
    if (x > 40.0)
        return x;
    if (x < -40.0) // exp(x) is already <  4e-18; log1p of that underflows to x itself in double
        return std::exp(x);
    return std::log1p(std::exp(x));
}

double korenE1(double vp, double vgk, const KorenTriodeParams& k) noexcept {
    const double denom = std::sqrt(k.kvb + vp * vp);
    const double x = k.kp * (1.0 / k.mu + vgk / denom);
    return (vp / k.kp) * korenSoftplus(x);
}

// Koren plate current: Ip = (E1 > 0) ? 2*E1^ex/kg1 : 0 -- the factor of 2 (rather than the
// textbook "(1+sgn(E1))") folds the sgn() branch directly into the early-out, since sgn(E1) is
// always +1 on the branch that survives the E1 > 0 guard.
double korenIp(double vp, double vgk, const KorenTriodeParams& k) noexcept {
    const double e1 = korenE1(vp, vgk, k);
    if (!(e1 > 0.0))
        return 0.0;
    return 2.0 * std::pow(e1, k.ex) / k.kg1;
}

struct OperatingPoint {
    double vp0;
    double ip0;
    double vk0;
};

// Solves Ip0 == Ip(Vb - Ip0*Rp, -Ip0*Rk) by bisection. f(Ip0) = Ip(Vb-Ip0*Rp, -Ip0*Rk) - Ip0 is
// strictly decreasing in Ip0: increasing Ip0 both lowers Vp (which lowers the tube's own Ip,
// since Ip is increasing in Vp) and makes Vgk more negative via Rk (which independently lowers
// Ip, since Ip is increasing in Vg) -- so a single bracketed root exists and bisection is safe
// and allocation-free. 60 iterations halves the [0, Vb/Rp] bracket to roughly 1e-18 of its
// original width, far beyond double's own precision floor for these magnitudes.
OperatingPoint solveOperatingPoint(const KorenTriodeParams& k, double vb, double rp, double rk) noexcept {
    double lo = 0.0;
    double hi = vb / rp;
    auto residual = [&](double ip0) {
        const double vp0 = vb - ip0 * rp;
        const double vgk0 = -ip0 * rk;
        return korenIp(vp0, vgk0, k) - ip0;
    };
    double rLo = residual(lo);
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double rMid = residual(mid);
        if ((rLo >= 0.0) == (rMid >= 0.0)) {
            lo = mid;
            rLo = rMid;
        } else {
            hi = mid;
        }
    }
    const double ip0 = 0.5 * (lo + hi);
    const double vp0 = vb - ip0 * rp;
    const double vk0 = ip0 * rk;
    return OperatingPoint{vp0, ip0, vk0};
}

// Solves the AC load line's plate-voltage intersection at a fixed Vgk: Ip(Vp,Vgk) == Ip0 +
// (Vp0-Vp)/Rac. f(Vp) = Ip(Vp,Vgk) - (Ip0+(Vp0-Vp)/Rac) is strictly increasing in Vp (Ip is
// increasing in Vp; the load-line term is decreasing in Vp), so bisection over [0, Vb] is again
// safe with a single root.
double solvePlateVoltage(const KorenTriodeParams& k, double vgk, double vp0, double ip0, double rac,
                         double vb) noexcept {
    double lo = 0.0;
    double hi = vb;
    auto residual = [&](double vp) {
        const double ipLine = ip0 + (vp0 - vp) / rac;
        return korenIp(vp, vgk, k) - ipLine;
    };
    double rLo = residual(lo);
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double rMid = residual(mid);
        if ((rLo <= 0.0) == (rMid <= 0.0)) {
            lo = mid;
            rLo = rMid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

} // namespace

KorenTriodeParams TriodeStage::publishedEcc83() noexcept {
    // Published Koren ECC83/12AX7 SPICE parameters (Koren, N., "Improved Vacuum Tube Models for
    // SPICE Simulations," Glass Audio 8(5), 1996). This exact (mu, ex, kg1, kp, kvb, rgi) tuple
    // is the parameter set most widely circulated for the 12AX7 in tube-amp SPICE modelling
    // references (e.g. Duncan Amplification's published triode SPICE model library). Hardcoded
    // here as the single authoritative source -- never recomputed or approximated elsewhere.
    KorenTriodeParams p;
    p.mu = 100.0;
    p.ex = 1.4;
    p.kg1 = 1060.0;
    p.kp = 600.0;
    p.kvb = 300.0;
    p.rgi = 2000.0;
    return p;
}

void TriodeStage::prepare(double sampleRate, int maxBlockSize) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(1, maxBlockSize);
    koren_ = publishedEcc83();

    buildTransferTable();

    setParams(TriodeStageParams{});
    reset();
}

void TriodeStage::reset() noexcept {
    currentDriveLinear_ = targetDriveLinear_;
    currentOutputLinear_ = targetOutputLinear_;
}

void TriodeStage::setParams(const TriodeStageParams& p) noexcept {
    bypass_ = p.bypass;
    const float safeDrive = clampFinite(p.drive, kMinDrive, kMaxDrive);
    targetDriveLinear_ = 2.0f * safeDrive; // drive=0.5 -> unity pre-gain
    const float safeTrimDb = clampFinite(p.outputTrimDb, kMinOutputTrimDb, kMaxOutputTrimDb);
    targetOutputLinear_ = dbToLinear(safeTrimDb);
}

void TriodeStage::buildTransferTable() {
    const double vb = kPlateSupplyVoltsDefault;
    const double rp = kPlateLoadOhms;
    const double rk = kCathodeResistorOhms;
    const double rac = (kPlateLoadOhms * kNextStageLoadOhms) / (kPlateLoadOhms + kNextStageLoadOhms);
    const double rs = kGridSourceResistanceOhms;

    const OperatingPoint q = solveOperatingPoint(koren_, vb, rp, rk);

    constexpr int n = kTableSize;
    std::vector<double> raw(static_cast<std::size_t>(n));
    const double halfWidth = kDomainHalfWidthVolts;
    const double step = (2.0 * halfWidth) / static_cast<double>(n - 1);

    for (int i = 0; i < n; ++i) {
        const double vin = -halfWidth + step * static_cast<double>(i);
        const double vgkRaw = vin - q.vk0;
        // Soft grid-current clamp: once the grid swings positive of the (bypassed, fixed) cathode
        // voltage, a resistive divider between the assumed drive-source resistance and rgi softens
        // the effective grid drive -- see TriodeStage.h's "grid-current clamp" section.
        const double vgk = (vgkRaw > 0.0) ? vgkRaw * (koren_.rgi / (koren_.rgi + rs)) : vgkRaw;
        const double vp = solvePlateVoltage(koren_, vgk, q.vp0, q.ip0, rac, vb);
        // (vp - q.vp0) IS the inverted signal already -- do not re-negate it. A common-cathode
        // stage's own physics already inverts: vin > 0 -> Vgk more positive -> Ip rises -> the
        // Rac-loaded plate voltage FALLS (vp - vp0 < 0), so a positive grid swing already produces
        // a negative raw sample here, with no separate negation needed. Re-negating this value (as
        // an earlier revision did) cancels that physical inversion and also swaps which output half
        // is the compressed one: with the correct sign below, vin very negative drives the tube
        // toward cutoff (Ip -> 0, Vp -> Vb, a hard physical ceiling), so (vp - vp0) saturates
        // POSITIVE on that side -- the real ECC83 common-cathode stage's positive output half is
        // the one that compresses, not the negative one.
        raw[static_cast<std::size_t>(i)] = vp - q.vp0;
    }

    const std::size_t centerIndex = static_cast<std::size_t>(n - 1) / 2;
    // vin == 0 at i == centerIndex maps to vgkRaw == -q.vk0 (the exact Vgk the operating point
    // itself was solved at), so raw[centerIndex] is provably 0 already (up to bisection rounding,
    // negligible at this scale) -- forced to an exact 0.0 below regardless, so the table's own DC
    // removal is exact rather than merely "very close".

    // Small-signal gain at the origin, from the two nodes straddling the center: normalizes the
    // whole curve to unity small-signal gain so kGridVoltsPerFullScale is the only free gain
    // constant end to end (see TriodeStage.h's "Gain staging" section).
    const double slope = (raw[centerIndex + 1] - raw[centerIndex - 1]) / (2.0 * step);
    const double normalization = (std::fabs(slope) > 1e-12) ? (1.0 / std::fabs(slope)) : 1.0;

    table_.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        table_[static_cast<std::size_t>(i)] = static_cast<float>(raw[static_cast<std::size_t>(i)] * normalization);
    table_[centerIndex] = 0.0f; // exact DC removal at the one node that must be exactly zero

    invStep_ = 1.0 / step;
    centerIndexDouble_ = static_cast<double>(centerIndex);
}

float TriodeStage::interpolate(double fractionalIndex) const noexcept {
    const int n = static_cast<int>(table_.size());
    const double clamped = std::clamp(fractionalIndex, 0.0, static_cast<double>(n - 1));
    int i1 = static_cast<int>(std::floor(clamped));
    if (i1 >= n - 1)
        i1 = std::max(n - 2, 0);
    const double t = clamped - static_cast<double>(i1);

    const int i0 = std::max(i1 - 1, 0);
    const int i2 = std::min(i1 + 1, n - 1);
    const int i3 = std::min(i1 + 2, n - 1);

    const double p0 = table_[static_cast<std::size_t>(i0)];
    const double p1 = table_[static_cast<std::size_t>(i1)];
    const double p2 = table_[static_cast<std::size_t>(i2)];
    const double p3 = table_[static_cast<std::size_t>(i3)];

    // Uniform-knot Catmull-Rom cubic, evaluated in double via Horner's method. Exact node
    // reproduction at t == 0 (the innermost-to-outermost Horner steps reduce to plain a3 == p1)
    // is what makes waveshapeOne()'s "vin == 0 -> exact table node" index construction yield a
    // bit-exact zero output, not merely an approximate one.
    const double a0 = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
    const double a1 = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
    const double a2 = -0.5 * p0 + 0.5 * p2;
    const double a3 = p1;
    return static_cast<float>(((a0 * t + a1) * t + a2) * t + a3);
}

Sample TriodeStage::waveshapeOne(Sample x, float driveLinear, float outputLinear) const noexcept {
    double vin = static_cast<double>(x) * static_cast<double>(driveLinear) * kGridVoltsPerFullScale;
    // Guard a malformed audio SAMPLE (NaN or +/-Inf, as opposed to a malformed PARAMETER --
    // setParams()'s clampFinite() already handles those) before it reaches the table-index
    // arithmetic below. std::clamp does not reject NaN (every NaN comparison is false, so it falls
    // through to "return v" unclamped), so an un-guarded NaN vin would propagate into
    // interpolate()'s std::floor -> static_cast<int> of a NaN double -- undefined behavior in C++,
    // observed on this toolchain to produce an out-of-range table_[] index (a real
    // out-of-bounds/UB read, not just a wrong sample). TriodeStage is dsp/'s first module that
    // indexes a lookup table directly off an audio sample (as opposed to an internally-computed,
    // already-bounded position, the way WaveguideString's railInterpolate() does), so there is no
    // existing precedent to inherit the guard from. A non-finite sample is treated as silence
    // (matching clampFinite()'s own non-finite-parameter fallback), reusing the exact-zero path
    // proven bit-exact above.
    if (!std::isfinite(vin))
        vin = 0.0;
    // x == 0 -> vin == 0.0 exactly (0 * finite == 0.0) -> fractionalIndex == centerIndexDouble_
    // exactly (0*invStep_ == 0.0; x + 0.0 == x for finite x) -> interpolate() returns
    // table_[centerIndex] == 0.0f exactly -> the whole expression is exactly 0.0f regardless of
    // outputLinear. This is the "zero input -> DC-removed zero output" contract, held bit-exactly.
    const double fractionalIndex = vin * invStep_ + centerIndexDouble_;
    const float shaped = interpolate(fractionalIndex);
    return static_cast<Sample>(static_cast<double>(shaped) * static_cast<double>(outputLinear));
}

void TriodeStage::process(const Sample* in, Sample* out, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);
    if (count <= 0)
        return;

    if (bypass_) {
        for (int n = 0; n < count; ++n)
            out[n] = in[n];
        return;
    }

    const bool settled = (currentDriveLinear_ == targetDriveLinear_) && (currentOutputLinear_ == targetOutputLinear_);
    if (settled) {
        for (int n = 0; n < count; ++n)
            out[n] = waveshapeOne(in[n], currentDriveLinear_, currentOutputLinear_);
        currentDriveLinear_ = targetDriveLinear_;
        currentOutputLinear_ = targetOutputLinear_;
        return;
    }

    // Linear ramp from the settled drive/outputTrim gains to the newly targeted ones across this
    // block's samples (the OutputGain/PickupTap Direct-Form convention), landing exactly on target
    // at the last sample regardless of numSamples.
    const double invN = 1.0 / static_cast<double>(count);
    for (int n = 0; n < count; ++n) {
        const bool isLastSample = (n == count - 1);
        float driveG;
        float outG;
        if (isLastSample) {
            driveG = targetDriveLinear_;
            outG = targetOutputLinear_;
        } else {
            const double t = static_cast<double>(n + 1) * invN;
            driveG = static_cast<float>(
                static_cast<double>(currentDriveLinear_) +
                (static_cast<double>(targetDriveLinear_) - static_cast<double>(currentDriveLinear_)) * t);
            outG = static_cast<float>(
                static_cast<double>(currentOutputLinear_) +
                (static_cast<double>(targetOutputLinear_) - static_cast<double>(currentOutputLinear_)) * t);
        }
        out[n] = waveshapeOne(in[n], driveG, outG);
    }

    currentDriveLinear_ = targetDriveLinear_;
    currentOutputLinear_ = targetOutputLinear_;
}

bool TriodeStage::loadTransferTable(const TransferTableView& table) {
    // Validation-only through P2 (see TriodeStage.h's "Deferred hooks" section): no storage, no
    // wiring into process() -- the internally-solved Koren curve is always what process() runs.
    if (table.values == nullptr)
        return false;
    if (table.size < 2)
        return false;
    if (!std::isfinite(table.inputMin) || !std::isfinite(table.inputMax))
        return false;
    if (!(table.inputMin < table.inputMax)) // strictly monotone domain
        return false;
    for (int i = 0; i < table.size; ++i) {
        if (!std::isfinite(table.values[static_cast<std::size_t>(i)]))
            return false;
    }
    return true;
}

void TriodeStage::setSupplyVoltage(float plateSupplyVolts) noexcept {
    // Store-only through P2 -- see TriodeStage.h's "Deferred hooks" section. Never read by
    // prepare()/setParams()/process(); no audible effect.
    supplyVoltsOverride_ = plateSupplyVolts;
    supplyVoltsSet_ = true;
}

void TriodeStage::setHeaterVoltage(float heaterVolts) noexcept {
    // Store-only through P2 -- see TriodeStage.h's "Deferred hooks" section. Never read by
    // prepare()/setParams()/process(); no audible effect.
    heaterVoltsOverride_ = heaterVolts;
    heaterVoltsSet_ = true;
}

} // namespace cnpg::dsp

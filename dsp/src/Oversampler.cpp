#include "cnpg/dsp/Oversampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Per-stage design parameters. Only stage 0 has to be sharp -- see Oversampler.h's "halfband
// filters" section for why stages 1 and 2 are deliberately cheap.
constexpr Oversampler::HalfbandStageSpec kStageSpecs[Oversampler::kMaxStages] = {
    {10, 0.0125}, // base rate <-> 2x: passband edge 0.24375, stopband edge 0.25625, >= 91 dB
    {4, 0.10},    // 2x <-> 4x:        passband edge 0.20,    stopband edge 0.30,    >= 70 dB
    {4, 0.10},    // 4x <-> 8x:        same
};

double integerPower(double x, int n) noexcept {
    double result = 1.0;
    for (int i = 0; i < n; ++i)
        result *= x;
    return result;
}

// Theta-series numerator of sqrt(k)*sn(2*c*K/order, k): sum_{i>=0} (-1)^i q^(i(i+1)) sin((2i+1) pi
// c / order). The nome q of an elliptic halfband is small (< 0.1 for any usable transition width),
// so q^(i(i+1)) collapses past a handful of terms; the loop stops once a term is below the double
// noise floor of the running sum. The `i < 64` bound is a belt-and-braces guard against a
// degenerate q == 1 that the callers below already exclude, not a real iteration limit.
double thetaSeriesNumerator(double q, int order, int c) noexcept {
    int i = 0;
    int sign = 1;
    double accumulator = 0.0;
    double term = 0.0;
    do {
        term = integerPower(q, i * (i + 1)) * std::sin((i * 2 + 1) * kPi * c / order) * sign;
        accumulator += term;
        sign = -sign;
        ++i;
    } while (std::fabs(term) > 1e-100 && i < 64);
    return accumulator;
}

// Matching denominator: sum_{i>=1} (-1)^i q^(i^2) cos(2 i pi c / order). The caller adds the 0.5
// that makes this the half of theta_4's series (numerator and denominator are both used at half
// scale, so the factor cancels).
double thetaSeriesDenominator(double q, int order, int c) noexcept {
    int i = 1;
    int sign = -1;
    double accumulator = 0.0;
    double term = 0.0;
    do {
        term = integerPower(q, i * i) * std::cos(i * 2 * kPi * c / order) * sign;
        accumulator += term;
        sign = -sign;
        ++i;
    } while (std::fabs(term) > 1e-100 && i < 64);
    return accumulator;
}

// DC group delay of one first-order-in-z^-2 allpass section. (a + z^-1)/(1 + a z^-1) has group
// delay (1-a)/(1+a) at DC; substituting z^-2 for z^-1 doubles it.
double sectionDcGroupDelay(double a) noexcept { return 2.0 * (1.0 - a) / (1.0 + a); }

} // namespace

void Oversampler::AllpassBranch::configure(const std::vector<double>& coefs) {
    coefficients = coefs;
    lastInput.assign(coefs.size(), 0.0);
    lastOutput.assign(coefs.size(), 0.0);
}

void Oversampler::AllpassBranch::clear() noexcept {
    std::fill(lastInput.begin(), lastInput.end(), 0.0);
    std::fill(lastOutput.begin(), lastOutput.end(), 0.0);
}

double Oversampler::AllpassBranch::process(double x) noexcept {
    const std::size_t count = coefficients.size();
    for (std::size_t i = 0; i < count; ++i) {
        // One-multiply direct form of (a + z^-1)/(1 + a z^-1): y = a*(x - y[-1]) + x[-1].
        const double y = coefficients[i] * (x - lastOutput[i]) + lastInput[i];
        lastInput[i] = x;
        lastOutput[i] = y;
        x = y;
    }
    return x;
}

void Oversampler::Stage::configure(const std::vector<double>& coefs) {
    // Ascending coefficients dealt alternately: even indices to A0, odd to A1. See Oversampler.h.
    std::vector<double> a0;
    std::vector<double> a1;
    a0.reserve((coefs.size() + 1) / 2);
    a1.reserve(coefs.size() / 2);
    for (std::size_t i = 0; i < coefs.size(); ++i)
        ((i % 2) == 0 ? a0 : a1).push_back(coefs[i]);

    upEven.configure(a0);
    upOdd.configure(a1);
    downOdd.configure(a0);  // A0 filters the odd-indexed oversampled samples
    downEven.configure(a1); // A1 filters the even-indexed ones

    // Both branches carry the same DC group delay by construction (that is exactly what the
    // alternating deal buys), so either one names the halfband's own DC group delay. A0's is the
    // cheaper of the two to state: branch A1 additionally carries the structural z^-1.
    double delay = 0.0;
    for (double a : a0)
        delay += sectionDcGroupDelay(a);
    dcGroupDelay = delay;
}

void Oversampler::Stage::clear() noexcept {
    upEven.clear();
    upOdd.clear();
    downEven.clear();
    downOdd.clear();
}

Oversampler::HalfbandStageSpec Oversampler::stageSpec(int stageIndex) noexcept {
    const int index = std::clamp(stageIndex, 0, kMaxStages - 1);
    return kStageSpecs[index];
}

std::vector<double> Oversampler::designHalfbandCoefficients(int numCoefficients, double transitionBandwidth) {
    const int count = std::max(1, numCoefficients);
    // Guard the degenerate ends of the design space: t -> 0 is an infinitely sharp filter (k -> 1,
    // nome q -> 1, the series below stop converging) and t -> 0.5 collapses the passband.
    const double transition = std::clamp(transitionBandwidth, 1e-4, 0.49);

    const int order = count * 2 + 1;

    // Selectivity of the equivalent lowpass prototype. For a halfband the band edges are
    // symmetric about a quarter of the sample rate: passband edge fp = 0.25 - t/2, stopband edge
    // fs = 0.25 + t/2 = 0.5 - fp, hence k = tan(pi*fp)/tan(pi*fs) = tan^2(pi*fp).
    const double k = std::pow(std::tan((1.0 - transition * 2.0) * kPi / 4.0), 2.0);

    // Jacobi nome from the modulus, via the standard q(k) series (q = e + 2e^5 + 15e^9 + 150e^13).
    const double complementaryRoot = std::pow(1.0 - k * k, 0.25);
    const double e = 0.5 * (1.0 - complementaryRoot) / (1.0 + complementaryRoot);
    const double e2 = e * e;
    const double e4 = e2 * e2;
    const double q = e * (1.0 + e4 * (2.0 + e4 * (15.0 + 150.0 * e4)));

    std::vector<double> coefficients(static_cast<std::size_t>(count));
    for (int j = 0; j < count; ++j) {
        const int c = j + 1;
        // w = sqrt(k) * sn(2cK/order, k), evaluated as the theta-function ratio. The 2/2 scaling
        // between the two series and the 1/sqrt(k) of the textbook sn formula are both folded away
        // here on purpose: what the pole mapping below needs is exactly sqrt(k)*sn, not sn.
        const double w =
            thetaSeriesNumerator(q, order, c) * std::pow(q, 0.25) / (thetaSeriesDenominator(q, order, c) + 0.5);
        const double w2 = w * w;
        // Map the prototype's j-axis pole onto the halfband's own z-plane pole pair z = +/- j*sqrt(a).
        const double x = std::sqrt((1.0 - w2 * k) * (1.0 - w2 / k)) / (1.0 + w2);
        coefficients[static_cast<std::size_t>(j)] = (1.0 - x) / (1.0 + x);
    }
    return coefficients;
}

void Oversampler::prepare(double sampleRate, int maxBlockSize, int factor) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(1, maxBlockSize);

    // Snap DOWN to the largest valid power of two in [2, kMaxOversampling]: 1 -> 2, 3 -> 2,
    // 5/6/7 -> 4, >= 9 -> 8. See Oversampler.h for why a nonsense factor is corrected rather than
    // rejected.
    const int requested = std::clamp(factor, 2, kMaxOversampling);
    factor_ = (requested >= 8) ? 8 : ((requested >= 4) ? 4 : 2);

    numStages_ = (factor_ == 2) ? 1 : ((factor_ == 4) ? 2 : 3);
    oversampledCapacity_ = maxBlockSize_ * factor_;

    double baseRateDelay = 0.0;
    for (int s = 0; s < numStages_; ++s) {
        const HalfbandStageSpec spec = stageSpec(s);
        stages_[s].configure(designHalfbandCoefficients(spec.numCoefficients, spec.transitionBandwidth));
        // One up+down round trip through stage s costs (D_s - 0.5) samples at that stage's INPUT
        // rate, which is 2^s times the base rate. See Oversampler.h's "Latency" section.
        baseRateDelay += (stages_[s].dcGroupDelay - 0.5) / std::pow(2.0, s);
    }
    latencySamples_ = static_cast<int>(std::lround(baseRateDelay));

    workBuffer_.assign(static_cast<std::size_t>(oversampledCapacity_), Sample{0});
    scratchA_.assign(static_cast<std::size_t>(oversampledCapacity_), Sample{0});
    scratchB_.assign(static_cast<std::size_t>(oversampledCapacity_), Sample{0});

    reset();
}

void Oversampler::reset() noexcept {
    for (int s = 0; s < kMaxStages; ++s)
        stages_[s].clear();
}

int Oversampler::clampBlock(int numSamples) const noexcept { return std::clamp(numSamples, 0, maxBlockSize_); }

void Oversampler::upsampleStage(Stage& stage, const Sample* in, int numSamples, Sample* out) noexcept {
    for (int i = 0; i < numSamples; ++i) {
        const double x = static_cast<double>(in[i]);
        // The polyphase identity: with the input zero-stuffed, A0(z^2) only ever produces the even
        // output samples and z^-1 A1(z^2) only the odd ones, so each branch runs once per INPUT
        // sample at the low rate and the interpolation's 2x gain cancels H's own 0.5.
        out[2 * i] = static_cast<Sample>(stage.upEven.process(x));
        out[2 * i + 1] = static_cast<Sample>(stage.upOdd.process(x));
    }
}

void Oversampler::downsampleStage(Stage& stage, const Sample* in, int numOut, Sample* out) noexcept {
    for (int i = 0; i < numOut; ++i) {
        const double odd = stage.downOdd.process(static_cast<double>(in[2 * i + 1]));
        const double even = stage.downEven.process(static_cast<double>(in[2 * i]));
        // Dual of the upsampler: this is H applied to the full-rate stream and read at the odd
        // full-rate index 2i+1, which is where the (D - 1) half of the round-trip latency accounted
        // for in prepare() comes from.
        out[i] = static_cast<Sample>(0.5 * (odd + even));
    }
}

int Oversampler::upsample(const Sample* in, int numSamples, Sample* upBuffer) noexcept {
    const int count = clampBlock(numSamples);
    if (count <= 0)
        return 0;

    // Ping-pong so no stage ever reads and writes the same buffer (each stage doubles its sample
    // count in place, which would clobber its own unread input). The parity is chosen so the last
    // stage always lands in the caller's `upBuffer`.
    Sample* pingPong[2] = {upBuffer, scratchA_.data()};
    const int startIndex = (numStages_ % 2 == 0) ? 1 : 0;

    const Sample* source = in;
    int length = count;
    for (int s = 0; s < numStages_; ++s) {
        Sample* destination = pingPong[(startIndex + s) % 2];
        upsampleStage(stages_[s], source, length, destination);
        source = destination;
        length *= 2;
    }
    return length;
}

void Oversampler::downsample(const Sample* upBuffer, int numUpsampled, Sample* out) noexcept {
    const int available = std::clamp(numUpsampled, 0, oversampledCapacity_);
    const int count = clampBlock(available / factor_);
    if (count <= 0)
        return;

    // Same ping-pong, run backwards; `upBuffer` is const so the first stage must write elsewhere,
    // and the parity is chosen so the last stage lands in the caller's `out`.
    Sample* pingPong[2] = {scratchA_.data(), scratchB_.data()};

    const Sample* source = upBuffer;
    int length = count * factor_;
    for (int s = numStages_ - 1; s >= 0; --s) {
        length /= 2;
        Sample* destination = (s == 0) ? out : pingPong[s % 2];
        downsampleStage(stages_[s], source, length, destination);
        source = destination;
    }
}

} // namespace cnpg::dsp

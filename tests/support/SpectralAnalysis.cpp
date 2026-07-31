#include "support/SpectralAnalysis.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <map>

namespace cnpg::test {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

std::size_t largestPowerOfTwoAtMost(std::size_t n) {
    std::size_t p = 1;
    while ((p << 1) <= n)
        p <<= 1;
    return p;
}

// Twiddle tables are cached per FFT size: recomputing e^-2*pi*i*k/n by repeated complex
// multiplication accumulates enough rounding over a 2^21-point transform to move the
// interpolated peak, and calling std::polar inside the butterfly loop is far too slow.
const std::vector<std::complex<double>>& twiddleTable(std::size_t n) {
    static std::map<std::size_t, std::vector<std::complex<double>>> cache;
    auto it = cache.find(n);
    if (it != cache.end())
        return it->second;

    std::vector<std::complex<double>> table(n / 2);
    for (std::size_t k = 0; k < n / 2; ++k) {
        const double angle = -kTwoPi * static_cast<double>(k) / static_cast<double>(n);
        table[k] = std::complex<double>(std::cos(angle), std::sin(angle));
    }
    return cache.emplace(n, std::move(table)).first->second;
}

void fftInPlace(std::vector<std::complex<double>>& a) {
    const std::size_t n = a.size();
    if (n < 2)
        return;

    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; (j & bit) != 0; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }

    const std::vector<std::complex<double>>& tw = twiddleTable(n);
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const std::size_t half = len >> 1;
        const std::size_t stride = n / len;
        for (std::size_t i = 0; i < n; i += len) {
            for (std::size_t j = 0; j < half; ++j) {
                const std::complex<double> u = a[i + j];
                const std::complex<double> v = a[i + j + half] * tw[j * stride];
                a[i + j] = u + v;
                a[i + j + half] = u - v;
            }
        }
    }
}

// 4-term Blackman-Harris (docs/plan.md sections 4.3/4.4/4.5 all specify this window).
double blackmanHarris(std::size_t index, std::size_t length) {
    constexpr double a0 = 0.35875;
    constexpr double a1 = 0.48829;
    constexpr double a2 = 0.14128;
    constexpr double a3 = 0.01168;
    const double t = kTwoPi * static_cast<double>(index) / static_cast<double>(length - 1);
    return a0 - a1 * std::cos(t) + a2 * std::cos(2.0 * t) - a3 * std::cos(3.0 * t);
}

// RBJ constant-skirt bandpass with unity peak gain.
struct Biquad {
    double b0 = 0.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

    double process(double x) {
        const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
};

// FOURTH order (two cascaded RBJ sections), not second. A single biquad's -40 dB/decade skirts
// are nowhere near enough for this measurement: at MIDI 21 the fundamental sits 2.5 decades below
// the 8 kHz band centre but is ~70 dB stronger than the string's content up there, so a 2nd-order
// band "measures" the fundamental's decay in every band and reports one uniform T60 across the
// whole spectrum. Doubling the order (-80 dB/decade) puts the leakage well under the band's own
// content. Each section is given a 1.4-octave bandwidth so the cascade's -3 dB width lands back
// near one octave.
Biquad makeOctaveBandpassSection(double centreHz, double sampleRate) {
    Biquad bq;
    const double w0 = kTwoPi * centreHz / sampleRate;
    constexpr double bandwidthOctaves = 1.4;
    const double sinw = std::sin(w0);
    const double cosw = std::cos(w0);
    const double alpha = sinw * std::sinh(std::log(2.0) / 2.0 * bandwidthOctaves * (w0 / std::max(sinw, 1e-12)));
    const double a0 = 1.0 + alpha;
    bq.b0 = alpha / a0;
    bq.b1 = 0.0;
    bq.b2 = -alpha / a0;
    bq.a1 = -2.0 * cosw / a0;
    bq.a2 = (1.0 - alpha) / a0;
    return bq;
}

} // namespace

Spectrum computeSpectrum(const std::vector<double>& samples, double sampleRate, std::size_t analysisLength) {
    Spectrum spectrum;
    spectrum.sampleRate = sampleRate;
    if (samples.size() < 8)
        return spectrum;

    std::size_t length = (analysisLength > 0) ? analysisLength : largestPowerOfTwoAtMost(samples.size());
    length = std::min(length, samples.size());
    length = largestPowerOfTwoAtMost(length);

    const std::size_t fftSize = length * 4; // x4 zero padding, per docs/plan.md section 4.5
    std::vector<std::complex<double>> buffer(fftSize, std::complex<double>(0.0, 0.0));
    for (std::size_t i = 0; i < length; ++i)
        buffer[i] = std::complex<double>(samples[i] * blackmanHarris(i, length), 0.0);

    fftInPlace(buffer);

    spectrum.fftSize = fftSize;
    spectrum.magnitudeSquared.resize(fftSize / 2 + 1);
    for (std::size_t k = 0; k <= fftSize / 2; ++k)
        spectrum.magnitudeSquared[k] = std::norm(buffer[k]);
    return spectrum;
}

double findPeakHz(const Spectrum& spectrum, double targetHz, double searchCents) {
    if (spectrum.fftSize == 0 || targetHz <= 0.0)
        return 0.0;

    const double loHz = targetHz * std::exp2(-searchCents / 1200.0);
    const double hiHz = targetHz * std::exp2(searchCents / 1200.0);

    const auto lastBin = static_cast<std::ptrdiff_t>(spectrum.magnitudeSquared.size()) - 1;
    auto lo = static_cast<std::ptrdiff_t>(std::floor(spectrum.hzToBin(loHz)));
    auto hi = static_cast<std::ptrdiff_t>(std::ceil(spectrum.hzToBin(hiHz)));
    lo = std::max<std::ptrdiff_t>(lo, 1);
    hi = std::min<std::ptrdiff_t>(hi, lastBin - 1);
    if (hi < lo)
        return 0.0;

    std::ptrdiff_t peak = lo;
    for (std::ptrdiff_t k = lo; k <= hi; ++k)
        if (spectrum.magnitudeSquared[static_cast<std::size_t>(k)] >
            spectrum.magnitudeSquared[static_cast<std::size_t>(peak)])
            peak = k;

    const double centre = spectrum.magnitudeSquared[static_cast<std::size_t>(peak)];
    if (!(centre > 0.0))
        return 0.0;

    // Parabola through the LOG magnitudes of the peak and its neighbours. log(|X|^2) is
    // 2*log(|X|), and scaling a parabola does not move its vertex, so the squared magnitudes
    // can be used directly.
    const double left = std::log(std::max(spectrum.magnitudeSquared[static_cast<std::size_t>(peak - 1)], 1e-300));
    const double mid = std::log(centre);
    const double right = std::log(std::max(spectrum.magnitudeSquared[static_cast<std::size_t>(peak + 1)], 1e-300));

    const double denom = left - 2.0 * mid + right;
    double delta = 0.0;
    if (std::fabs(denom) > 1e-300)
        delta = 0.5 * (left - right) / denom;
    delta = std::clamp(delta, -0.5, 0.5);

    return spectrum.binToHz(static_cast<double>(peak) + delta);
}

double centsBetween(double measuredHz, double referenceHz) {
    if (measuredHz <= 0.0 || referenceHz <= 0.0)
        return 0.0;
    return 1200.0 * std::log2(measuredHz / referenceHz);
}

double midiNoteToHz(int midiNote) { return 440.0 * std::exp2((static_cast<double>(midiNote) - 69.0) / 12.0); }

double bandT60Seconds(const std::vector<double>& samples, double sampleRate, double centreHz) {
    if (samples.empty() || centreHz <= 0.0 || centreHz >= 0.45 * sampleRate)
        return -1.0;

    Biquad first = makeOctaveBandpassSection(centreHz, sampleRate);
    Biquad second = makeOctaveBandpassSection(centreHz, sampleRate);
    std::vector<double> banded(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
        banded[i] = second.process(first.process(samples[i]));

    // Schroeder backward integration.
    std::vector<double> schroeder(banded.size());
    double acc = 0.0;
    for (std::size_t i = banded.size(); i-- > 0;) {
        acc += banded[i] * banded[i];
        schroeder[i] = acc;
    }
    if (!(schroeder[0] > 0.0))
        return -1.0;

    const double reference = schroeder[0];
    // Fit the -5 dB .. -25 dB span (T20), then extrapolate to 60 dB.
    const double startLevel = reference * std::pow(10.0, -5.0 / 10.0);
    const double endLevel = reference * std::pow(10.0, -25.0 / 10.0);

    std::size_t startIndex = 0;
    while (startIndex < schroeder.size() && schroeder[startIndex] > startLevel)
        ++startIndex;
    std::size_t endIndex = startIndex;
    while (endIndex < schroeder.size() && schroeder[endIndex] > endLevel)
        ++endIndex;
    if (endIndex >= schroeder.size() || endIndex <= startIndex + 16)
        return -1.0;

    // Least-squares slope of 10*log10(schroeder) against time over the fitted span.
    double sumT = 0.0, sumY = 0.0, sumTT = 0.0, sumTY = 0.0;
    double n = 0.0;
    for (std::size_t i = startIndex; i <= endIndex; ++i) {
        if (!(schroeder[i] > 0.0))
            break;
        const double t = static_cast<double>(i) / sampleRate;
        const double y = 10.0 * std::log10(schroeder[i] / reference);
        sumT += t;
        sumY += y;
        sumTT += t * t;
        sumTY += t * y;
        n += 1.0;
    }
    if (n < 16.0)
        return -1.0;

    const double denom = n * sumTT - sumT * sumT;
    if (std::fabs(denom) < 1e-300)
        return -1.0;
    const double slope = (n * sumTY - sumT * sumY) / denom; // dB per second, negative
    if (slope >= -1e-6)
        return -1.0;
    return -60.0 / slope;
}

double rmsDbfs(const std::vector<double>& samples, double sampleRate, double windowSeconds) {
    const auto count = std::min(samples.size(), static_cast<std::size_t>(windowSeconds * sampleRate));
    if (count == 0)
        return -300.0;
    double acc = 0.0;
    for (std::size_t i = 0; i < count; ++i)
        acc += samples[i] * samples[i];
    const double rms = std::sqrt(acc / static_cast<double>(count));
    return (rms > 0.0) ? 20.0 * std::log10(rms) : -300.0;
}

} // namespace cnpg::test

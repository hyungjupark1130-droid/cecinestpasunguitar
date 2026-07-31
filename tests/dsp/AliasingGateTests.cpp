#include "cnpg/dsp/Oversampler.h"
#include "cnpg/dsp/TriodeStage.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using cnpg::dsp::Oversampler;
using cnpg::dsp::Sample;
using cnpg::dsp::TriodeStage;
using cnpg::dsp::TriodeStageParams;

// -------------------------------------------------------------------------------------------
// The [aliasing] gate -- docs/plan.md section 4.4, Task P1.8.
//
// Device under test: `Oversampler` wrapping `TriodeStage::process` via `processWrapped`, exactly
// as the P1 monitoring chain composes them, at a 48 kHz host rate.
//
// Stimulus (locked): fixed sines at 1244.5 Hz and 4186 Hz -- chosen so that neither one's harmonic
// series lands inside the stage-0 halfband's transition band, which is the one region of the
// spectrum an oversampler structurally cannot control (see Oversampler.h). Two drive settings are
// gated: nominal (default `TriodeStageParams::drive`, -18 dBFS peak input -- docs/plan.md section
// 1.9's per-string nominal) and maximum drive (drive 1.0, the top of the documented parameter
// range, same input level). A third condition, "headroom", is measured and printed but NOT gated:
// drive 1.0 with a -2 dBFS input, i.e. the nominal per-string level plus the whole +16 dB
// multi-string summing budget from docs/plan.md section 2.9. It is the hardest thing the shipped
// chain can actually be asked to do, and leaving it unmeasured would make the gate flattering.
//
// Measurement (locked): discard 8192 warm-up samples, capture 2^18, Blackman-Harris window, FFT
// >= 2^18 points (tests/support/SpectralAnalysis.cpp uses 2^18 analysis samples zero-padded x4 to
// a 2^20-point transform, which both satisfies the floor and puts a bin every 0.046 Hz).
//
// Bin classification (locked): enumerate predicted harmonics k*f0 until k*f0 exceeds
// 4 * (factor * 24 kHz). Harmonics with k*f0 <= 24 kHz are harmonic; every other k is classified by
// its folded image |k*f0 - round(k*f0 / 48000) * 48000|. That single formula covers BOTH fold
// mechanisms, and deliberately so: reflection folding about a multiple of the base rate is the
// composition of reflection folding about a multiple of the oversampled rate with the decimator's
// own fold, so aliasing generated inside the oversampled domain (which no decimation filter can
// remove) is measured by exactly the same rule as aliasing the decimation filter failed to reject.
//
// Two measurement radii, both stated in units of the ANALYSIS bin (48000 / 2^18 = 0.183 Hz):
//   - readout, 1 bin: the predicted frequencies are known exactly, so this only has to absorb the
//     x4-zero-padded grid's offset from the true peak. docs/plan.md section 4.4 allows +/-2.
//   - coincidence, 8 bins: a fold prediction closer than this to a harmonic prediction is dropped
//     as "coinciding with a harmonic bin". Eight bins is the Blackman-Harris main-lobe WIDTH -- the
//     smallest radius at which a harmonic's own main lobe can no longer be misread as a fold. The
//     plan's "+/-2 bins" is a peak-matching tolerance for an unknown peak; using it as a
//     coincidence radius for this window would report harmonic energy as aliasing.
// -------------------------------------------------------------------------------------------

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHostRate = 48000.0;
constexpr double kNyquist = kHostRate * 0.5;
constexpr int kBlockSize = 512;
constexpr int kWarmupSamples = 8192;
constexpr std::size_t kAnalysisLength = 1u << 18;

constexpr double kToneLow = 1244.5;  // D#6, docs/plan.md section 4.4
constexpr double kToneHigh = 4186.0; // C8
constexpr double kGateDbc = -60.0;

// -18 dBFS: the per-string nominal peak (docs/plan.md section 1.9). -2 dBFS is that plus the
// +16 dB multi-string summing headroom budget (docs/plan.md section 2.9).
constexpr double kNominalPeak = 0.12589254117941673;
constexpr double kHeadroomPeak = 0.7943282347242815;

struct DriveCondition {
    const char* name;
    float drive;
    double inputPeak;
    bool gated;
};

const DriveCondition kDriveConditions[] = {
    {"nominal", TriodeStageParams{}.drive, kNominalPeak, true},
    {"max", 1.0f, kNominalPeak, true},
    {"headroom", 1.0f, kHeadroomPeak, false},
};

// Renders a steady sine through Oversampler(TriodeStage) and returns `kAnalysisLength` settled
// samples as float64. The warm-up is discarded on a block boundary so the capture never straddles
// a partially settled filter state.
std::vector<double> renderOversampled(double freqHz, double amplitude, float drive, int factor) {
    Oversampler os;
    os.prepare(kHostRate, kBlockSize, factor);

    TriodeStage triode;
    // The wrapped stage runs at the OVERSAMPLED rate on OVERSAMPLED blocks -- the wiring P1.9 must
    // reproduce (see Oversampler.h's "Wiring note").
    triode.prepare(kHostRate * static_cast<double>(factor), kBlockSize * factor);
    TriodeStageParams params;
    params.drive = drive;
    triode.setParams(params);
    triode.reset();

    const int total = kWarmupSamples + static_cast<int>(kAnalysisLength);
    std::vector<double> captured;
    captured.reserve(kAnalysisLength);

    std::vector<Sample> in(static_cast<std::size_t>(kBlockSize));
    std::vector<Sample> out(static_cast<std::size_t>(kBlockSize));
    const double omega = 2.0 * kPi * freqHz / kHostRate;

    for (int offset = 0; offset < total; offset += kBlockSize) {
        for (int i = 0; i < kBlockSize; ++i)
            in[static_cast<std::size_t>(i)] =
                static_cast<Sample>(amplitude * std::sin(omega * static_cast<double>(offset + i)));
        os.processWrapped(in.data(), out.data(), kBlockSize,
                          [&triode](Sample* buffer, int count) { triode.process(buffer, buffer, count); });
        if (offset >= kWarmupSamples) {
            for (int i = 0; i < kBlockSize && captured.size() < kAnalysisLength; ++i)
                captured.push_back(static_cast<double>(out[static_cast<std::size_t>(i)]));
        }
    }
    return captured;
}

struct AliasingMeasurement {
    double worstFoldedDbc = -300.0;
    double worstFoldedHz = 0.0;
    double strongestHarmonicHz = 0.0;
    double noiseFloorDbc = -300.0;
    int numHarmonics = 0;
    int numFolds = 0;
};

double peakMagnitudeAround(const cnpg::test::Spectrum& spectrum, double hz, double radiusHz) {
    if (spectrum.fftSize == 0 || spectrum.magnitudeSquared.empty())
        return 0.0;
    const auto lastBin = static_cast<std::ptrdiff_t>(spectrum.magnitudeSquared.size()) - 1;
    auto lo = static_cast<std::ptrdiff_t>(std::floor(spectrum.hzToBin(hz - radiusHz)));
    auto hi = static_cast<std::ptrdiff_t>(std::ceil(spectrum.hzToBin(hz + radiusHz)));
    lo = std::max<std::ptrdiff_t>(lo, 0);
    hi = std::min<std::ptrdiff_t>(hi, lastBin);
    double best = 0.0;
    for (std::ptrdiff_t k = lo; k <= hi; ++k)
        best = std::max(best, spectrum.magnitudeSquared[static_cast<std::size_t>(k)]);
    return std::sqrt(best);
}

// Median bin magnitude between 100 Hz and 20 kHz: peaks are sparse enough that the median is the
// measurement's own noise floor. Reported alongside every row so a "-95 dBc worst fold" can be read
// as a real measurement rather than as the harness running out of dynamic range.
double medianMagnitude(const cnpg::test::Spectrum& spectrum) {
    if (spectrum.fftSize == 0)
        return 0.0;
    const auto lo = static_cast<std::size_t>(spectrum.hzToBin(100.0));
    const auto hi = std::min(static_cast<std::size_t>(spectrum.hzToBin(20000.0)), spectrum.magnitudeSquared.size() - 1);
    if (hi <= lo)
        return 0.0;
    std::vector<double> values(spectrum.magnitudeSquared.begin() + static_cast<std::ptrdiff_t>(lo),
                               spectrum.magnitudeSquared.begin() + static_cast<std::ptrdiff_t>(hi));
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return std::sqrt(*mid);
}

AliasingMeasurement measureAliasing(const std::vector<double>& signal, double f0, int factor) {
    AliasingMeasurement result;
    if (signal.size() < kAnalysisLength)
        return result;

    // The triode is asymmetric, so its output carries a real DC offset. Removing the mean keeps the
    // window's DC lobe from swamping folds that land at very low frequencies.
    double mean = 0.0;
    for (double v : signal)
        mean += v;
    mean /= static_cast<double>(signal.size());
    std::vector<double> centred(signal.size());
    for (std::size_t i = 0; i < signal.size(); ++i)
        centred[i] = signal[i] - mean;

    const cnpg::test::Spectrum spectrum = cnpg::test::computeSpectrum(centred, kHostRate, kAnalysisLength);
    const double analysisBinHz = kHostRate / static_cast<double>(kAnalysisLength);
    const double readoutRadiusHz = 1.0 * analysisBinHz;
    const double coincidenceRadiusHz = 8.0 * analysisBinHz;

    // Enumerate every predicted harmonic out to 4 x the oversampled Nyquist, per docs/plan.md
    // section 4.4. No amplitude-based early stop: the harness has no analytic prediction of the
    // Koren transfer's harmonic amplitudes, and enumerating MORE folds than necessary can only make
    // the gate stricter, never more flattering.
    const double enumerationLimitHz = 4.0 * (static_cast<double>(factor) * kNyquist);
    const int maxHarmonic = static_cast<int>(std::floor(enumerationLimitHz / f0));

    std::vector<double> harmonicHz;
    std::vector<double> foldHz;
    for (int k = 1; k <= maxHarmonic; ++k) {
        const double raw = static_cast<double>(k) * f0;
        if (raw <= kNyquist) {
            harmonicHz.push_back(raw);
            continue;
        }
        const double folded = std::fabs(raw - std::round(raw / kHostRate) * kHostRate);
        // A fold sitting on DC is not measurable under a windowed FFT; everything else in the band
        // counts, right up to Nyquist.
        if (folded > coincidenceRadiusHz && folded < kNyquist)
            foldHz.push_back(folded);
    }

    double strongestHarmonic = 0.0;
    for (double hz : harmonicHz) {
        const double magnitude = peakMagnitudeAround(spectrum, hz, readoutRadiusHz);
        if (magnitude > strongestHarmonic) {
            strongestHarmonic = magnitude;
            result.strongestHarmonicHz = hz;
        }
    }
    result.numHarmonics = static_cast<int>(harmonicHz.size());
    if (!(strongestHarmonic > 0.0))
        return result;

    double worstFold = 0.0;
    int counted = 0;
    for (double hz : foldHz) {
        bool coincides = false;
        for (double h : harmonicHz) {
            if (std::fabs(hz - h) <= coincidenceRadiusHz) {
                coincides = true;
                break;
            }
        }
        if (coincides)
            continue;
        ++counted;
        const double magnitude = peakMagnitudeAround(spectrum, hz, readoutRadiusHz);
        if (magnitude > worstFold) {
            worstFold = magnitude;
            result.worstFoldedHz = hz;
        }
    }
    result.numFolds = counted;
    result.worstFoldedDbc = (worstFold > 0.0) ? 20.0 * std::log10(worstFold / strongestHarmonic) : -300.0;
    const double floorMagnitude = medianMagnitude(spectrum);
    result.noiseFloorDbc = (floorMagnitude > 0.0) ? 20.0 * std::log10(floorMagnitude / strongestHarmonic) : -300.0;
    return result;
}

// `gated` is the row's real status, not the drive condition's: only the DEFAULT factor is gated
// (docs/plan.md section 4.4, "Non-default factors are report-only, never gated").
std::string formatRow(const char* path, int factor, int latency, double toneHz, const DriveCondition& drive,
                      const AliasingMeasurement& m, bool gated) {
    char line[256];
    std::snprintf(line, sizeof(line), "  %-8s %5d %8d %10.1f %-9s %11.2f %11.1f %10.2f %7d %s", path, factor, latency,
                  toneHz, drive.name, m.worstFoldedDbc, m.worstFoldedHz, m.noiseFloorDbc, m.numFolds,
                  gated ? "GATED" : "report");
    return std::string(line);
}

void printHeader() {
    std::cout << "  path     factor  latency       tone drive           worst dBc    at Hz     noise "
                 "floor   folds status\n";
}

} // namespace

TEST_CASE("ALIASING: triode folded components under -60 dBc", "[aliasing]") {
    std::cout << "[aliasing] Oversampler(TriodeStage) folded-component table, 48 kHz host rate,\n"
                 "[aliasing] 2^18-sample Blackman-Harris analysis after an 8192-sample warm-up.\n";
    printHeader();

    double worstGatedDbc = -300.0;
    std::string worstGatedRow;

    for (int factor : {2, 4, 8}) {
        Oversampler probe;
        probe.prepare(kHostRate, kBlockSize, factor);
        const int latency = probe.latencySamples();

        for (double tone : {kToneLow, kToneHigh}) {
            for (const DriveCondition& drive : kDriveConditions) {
                const std::vector<double> rendered = renderOversampled(tone, drive.inputPeak, drive.drive, factor);
                const AliasingMeasurement m = measureAliasing(rendered, tone, factor);
                const bool gated = (factor == Oversampler::kDefaultFactor) && drive.gated;
                const std::string row = formatRow("OS", factor, latency, tone, drive, m, gated);
                std::cout << row << "\n";

                // Guards against a VACUOUS pass. A gate whose only assertion is "the worst thing I
                // found is quiet enough" also passes when the harness found nothing at all, so every
                // row must first prove it measured something: fold predictions were enumerated and
                // classified, a real level came back for the worst of them, and the measurement's own
                // noise floor sits far enough below the limit that aliasing could not be hiding in it.
                INFO(row);
                REQUIRE(m.numFolds > 0);
                REQUIRE(m.numHarmonics > 0);
                REQUIRE(m.worstFoldedDbc > -300.0);
                REQUIRE(m.noiseFloorDbc < kGateDbc - 20.0);

                if (gated && m.worstFoldedDbc > worstGatedDbc) {
                    worstGatedDbc = m.worstFoldedDbc;
                    worstGatedRow = row;
                }
            }
        }
    }

    std::cout << "[aliasing] worst GATED folded component at the default factor " << Oversampler::kDefaultFactor
              << "x: " << worstGatedDbc << " dBc (limit " << kGateDbc << " dBc)\n"
              << "[aliasing] worst row:\n"
              << worstGatedRow << "\n";

    // The gate itself: default factor, both tones, both gated drive settings.
    REQUIRE(worstGatedDbc > -300.0); // four gated rows really did run
    CHECK(worstGatedDbc <= kGateDbc);
}

// -------------------------------------------------------------------------------------------
// ADAA spike (Task P1.8 step 3). Test-only, hidden from CTest ("[.]"), regenerates the tables in
// docs/decisions/0003-adaa-vs-oversampling.md. Follows the same house pattern as
// "SPIKE: fractional-delay comparison (report generator)".
//
// First-order antiderivative antialiasing on the SHIPPED transfer curve: the curve is recovered
// from `TriodeStage` through its own public `process()` on a dense uniform grid (so the spike can
// never drift away from what the plugin actually ships), linearly interpolated between grid points,
// and integrated in closed form per segment -- a piecewise-linear f has an exactly-integrable
// piecewise-quadratic antiderivative F, which is all first-order ADAA needs:
//
//   y[n] = (F(x[n]) - F(x[n-1])) / (x[n] - x[n-1]),   falling back to f((x[n]+x[n-1])/2) when the
//                                                     difference is too small to divide by.
// -------------------------------------------------------------------------------------------

namespace {

constexpr double kAdaaGridHalfWidth = 8.0; // input-sample units; covers drive 1.0 x a +18 dBFS input
constexpr int kAdaaGridPoints = 65537;     // odd, so x == 0 is an exact grid node

struct AdaaTransfer {
    std::vector<double> value;          // f at each grid node
    std::vector<double> antiderivative; // F at each grid node, F(0) == 0
    double step = 0.0;
    double centreIndex = 0.0;

    double clampedIndex(double x) const {
        const double idx = x / step + centreIndex;
        return std::clamp(idx, 0.0, static_cast<double>(value.size() - 1));
    }

    double f(double x) const {
        const double idx = clampedIndex(x);
        const auto i = static_cast<std::size_t>(idx);
        const std::size_t j = std::min(i + 1, value.size() - 1);
        const double t = idx - static_cast<double>(i);
        return value[i] + (value[j] - value[i]) * t;
    }

    // F(x) = F(node i) + integral of the linear segment from node i to x. Exact for the
    // piecewise-linear f above.
    double antiderivativeAt(double x) const {
        const double idx = clampedIndex(x);
        const auto i = static_cast<std::size_t>(idx);
        const std::size_t j = std::min(i + 1, value.size() - 1);
        const double t = idx - static_cast<double>(i);
        const double here = value[i] + (value[j] - value[i]) * t;
        return antiderivative[i] + 0.5 * (value[i] + here) * t * step;
    }
};

// Samples the shipped TriodeStage transfer at the given drive and builds its antiderivative.
AdaaTransfer buildAdaaTransfer(float drive) {
    AdaaTransfer transfer;
    const int n = kAdaaGridPoints;
    transfer.step = 2.0 * kAdaaGridHalfWidth / static_cast<double>(n - 1);
    transfer.centreIndex = static_cast<double>((n - 1) / 2);

    TriodeStage stage;
    stage.prepare(kHostRate, n);
    TriodeStageParams params;
    params.drive = drive;
    stage.setParams(params);
    stage.reset();

    std::vector<Sample> in(static_cast<std::size_t>(n));
    std::vector<Sample> out(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        in[static_cast<std::size_t>(i)] =
            static_cast<Sample>(-kAdaaGridHalfWidth + transfer.step * static_cast<double>(i));
    stage.process(in.data(), out.data(), n);

    transfer.value.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        transfer.value[static_cast<std::size_t>(i)] = static_cast<double>(out[static_cast<std::size_t>(i)]);

    transfer.antiderivative.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 1; i < n; ++i)
        transfer.antiderivative[static_cast<std::size_t>(i)] =
            transfer.antiderivative[static_cast<std::size_t>(i - 1)] +
            0.5 * (transfer.value[static_cast<std::size_t>(i - 1)] + transfer.value[static_cast<std::size_t>(i)]) *
                transfer.step;
    // Re-zero at x == 0 so F carries no arbitrary constant (irrelevant to the difference quotient,
    // but it keeps the magnitudes small and the numbers readable).
    const double offset = transfer.antiderivative[static_cast<std::size_t>(transfer.centreIndex)];
    for (double& v : transfer.antiderivative)
        v -= offset;
    return transfer;
}

// First-order ADAA. `previous` carries x[n-1] across blocks.
void processAdaa(const AdaaTransfer& transfer, const double* in, double* out, int count, double& previous) {
    constexpr double kEpsilon = 1e-6;
    for (int i = 0; i < count; ++i) {
        const double x = in[i];
        const double dx = x - previous;
        if (std::fabs(dx) < kEpsilon)
            out[i] = transfer.f(0.5 * (x + previous));
        else
            out[i] = (transfer.antiderivativeAt(x) - transfer.antiderivativeAt(previous)) / dx;
        previous = x;
    }
}

std::vector<double> renderAdaa(double freqHz, double amplitude, float drive) {
    const AdaaTransfer transfer = buildAdaaTransfer(drive);
    const int total = kWarmupSamples + static_cast<int>(kAnalysisLength);
    std::vector<double> in(static_cast<std::size_t>(total));
    std::vector<double> out(static_cast<std::size_t>(total));
    const double omega = 2.0 * kPi * freqHz / kHostRate;
    for (int i = 0; i < total; ++i)
        in[static_cast<std::size_t>(i)] = amplitude * std::sin(omega * static_cast<double>(i));

    double previous = 0.0;
    processAdaa(transfer, in.data(), out.data(), total, previous);

    return std::vector<double>(out.begin() + kWarmupSamples, out.end());
}

// Nanoseconds per BASE-rate sample. Both paths process the same number of base-rate samples, so the
// oversampled path's per-sample cost already includes its factor-x nonlinearity evaluations plus
// both halfband legs.
double timeOversampledPath(int factor, float drive) {
    Oversampler os;
    os.prepare(kHostRate, kBlockSize, factor);
    TriodeStage triode;
    triode.prepare(kHostRate * static_cast<double>(factor), kBlockSize * factor);
    TriodeStageParams params;
    params.drive = drive;
    triode.setParams(params);
    triode.reset();

    std::vector<Sample> in(static_cast<std::size_t>(kBlockSize));
    std::vector<Sample> out(static_cast<std::size_t>(kBlockSize));
    for (int i = 0; i < kBlockSize; ++i)
        in[static_cast<std::size_t>(i)] =
            static_cast<Sample>(0.3 * std::sin(2.0 * kPi * 440.0 * static_cast<double>(i) / kHostRate));

    constexpr int kBlocks = 4000;
    for (int b = 0; b < 64; ++b) // warm the caches / branch predictors
        os.processWrapped(in.data(), out.data(), kBlockSize, [&](Sample* buf, int n) { triode.process(buf, buf, n); });

    const auto start = std::chrono::steady_clock::now();
    for (int b = 0; b < kBlocks; ++b)
        os.processWrapped(in.data(), out.data(), kBlockSize, [&](Sample* buf, int n) { triode.process(buf, buf, n); });
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double ns = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
    return ns / static_cast<double>(kBlocks * kBlockSize);
}

double timeNaivePath(float drive) {
    TriodeStage stage;
    stage.prepare(kHostRate, kBlockSize);
    TriodeStageParams params;
    params.drive = drive;
    stage.setParams(params);
    stage.reset();

    std::vector<Sample> in(static_cast<std::size_t>(kBlockSize));
    std::vector<Sample> out(static_cast<std::size_t>(kBlockSize));
    for (int i = 0; i < kBlockSize; ++i)
        in[static_cast<std::size_t>(i)] =
            static_cast<Sample>(0.3 * std::sin(2.0 * kPi * 440.0 * static_cast<double>(i) / kHostRate));

    constexpr int kBlocks = 4000;
    for (int b = 0; b < 64; ++b)
        stage.process(in.data(), out.data(), kBlockSize);

    const auto start = std::chrono::steady_clock::now();
    for (int b = 0; b < kBlocks; ++b)
        stage.process(in.data(), out.data(), kBlockSize);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double ns = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
    return ns / static_cast<double>(kBlocks * kBlockSize);
}

double timeAdaaPath(float drive) {
    const AdaaTransfer transfer = buildAdaaTransfer(drive);
    std::vector<double> in(static_cast<std::size_t>(kBlockSize));
    std::vector<double> out(static_cast<std::size_t>(kBlockSize));
    for (int i = 0; i < kBlockSize; ++i)
        in[static_cast<std::size_t>(i)] = 0.3 * std::sin(2.0 * kPi * 440.0 * static_cast<double>(i) / kHostRate);

    constexpr int kBlocks = 4000;
    double previous = 0.0;
    for (int b = 0; b < 64; ++b)
        processAdaa(transfer, in.data(), out.data(), kBlockSize, previous);

    const auto start = std::chrono::steady_clock::now();
    for (int b = 0; b < kBlocks; ++b)
        processAdaa(transfer, in.data(), out.data(), kBlockSize, previous);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double ns = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
    return ns / static_cast<double>(kBlocks * kBlockSize);
}

// Amplitude of the component at `freqHz` in a settled buffer, by direct correlation over as close
// to a whole number of cycles as the sample grid allows. The residual partial-cycle leakage scales
// as 1/(2*pi*cycles); with 2^18 samples that is below 1e-5 relative at every probe frequency used
// here, i.e. four orders of magnitude under the effects being measured.
double componentAmplitude(const std::vector<double>& signal, double freqHz) {
    const double samplesPerCycle = kHostRate / freqHz;
    const auto usable =
        static_cast<std::size_t>(std::floor(static_cast<double>(signal.size()) / samplesPerCycle) * samplesPerCycle);
    if (usable < 16)
        return 0.0;
    double re = 0.0;
    double im = 0.0;
    const double omega = 2.0 * kPi * freqHz / kHostRate;
    for (std::size_t i = 0; i < usable; ++i) {
        re += signal[i] * std::cos(omega * static_cast<double>(i));
        im += signal[i] * std::sin(omega * static_cast<double>(i));
    }
    return 2.0 * std::hypot(re, im) / static_cast<double>(usable);
}

// Step-response smearing: 10-90% rise time, in base-rate samples, of the response to a step from
// -peak to +peak. The static (unwrapped) stage is the zero-smearing reference.
double stepRiseSamples(const std::vector<double>& response) {
    const double first = response.front();
    const double last = response.back();
    const double span = last - first;
    if (std::fabs(span) < 1e-12)
        return 0.0;
    const double low = first + 0.1 * span;
    const double high = first + 0.9 * span;
    std::size_t lowIndex = 0;
    std::size_t highIndex = 0;
    for (std::size_t i = 0; i < response.size(); ++i) {
        const double v = response[i];
        const bool reachedLow = (span > 0.0) ? (v >= low) : (v <= low);
        const bool reachedHigh = (span > 0.0) ? (v >= high) : (v <= high);
        if (lowIndex == 0 && reachedLow)
            lowIndex = i;
        if (reachedHigh) {
            highIndex = i;
            break;
        }
    }
    return static_cast<double>(highIndex) - static_cast<double>(lowIndex);
}

} // namespace

TEST_CASE("SPIKE: ADAA vs oversampling comparison (report generator)", "[.][report]") {
    std::cout << "[spike] ADAA (first order, shipped Koren transfer) vs Oversampler(TriodeStage).\n"
                 "[spike] Same stimulus, same classifier, same 48 kHz host rate as the [aliasing] gate.\n";
    printHeader();

    for (double tone : {kToneLow, kToneHigh}) {
        for (const DriveCondition& drive : kDriveConditions) {
            const std::vector<double> adaa = renderAdaa(tone, drive.inputPeak, drive.drive);
            const AliasingMeasurement m = measureAliasing(adaa, tone, 1);
            std::cout << formatRow("ADAA1", 1, 0, tone, drive, m, false) << "\n";
        }
    }
    // Naive (no antialiasing at all) reference, so the ADAA and oversampling numbers can be read
    // against what the raw stage does.
    for (double tone : {kToneLow, kToneHigh}) {
        for (const DriveCondition& drive : kDriveConditions) {
            TriodeStage stage;
            stage.prepare(kHostRate, kBlockSize);
            TriodeStageParams params;
            params.drive = drive.drive;
            stage.setParams(params);
            stage.reset();
            const int total = kWarmupSamples + static_cast<int>(kAnalysisLength);
            std::vector<double> captured;
            captured.reserve(kAnalysisLength);
            std::vector<Sample> in(static_cast<std::size_t>(kBlockSize));
            std::vector<Sample> out(static_cast<std::size_t>(kBlockSize));
            const double omega = 2.0 * kPi * tone / kHostRate;
            for (int offset = 0; offset < total; offset += kBlockSize) {
                for (int i = 0; i < kBlockSize; ++i)
                    in[static_cast<std::size_t>(i)] =
                        static_cast<Sample>(drive.inputPeak * std::sin(omega * static_cast<double>(offset + i)));
                stage.process(in.data(), out.data(), kBlockSize);
                if (offset >= kWarmupSamples)
                    for (int i = 0; i < kBlockSize && captured.size() < kAnalysisLength; ++i)
                        captured.push_back(static_cast<double>(out[static_cast<std::size_t>(i)]));
            }
            const AliasingMeasurement m = measureAliasing(captured, tone, 1);
            std::cout << formatRow("naive", 1, 0, tone, drive, m, false) << "\n";
        }
    }

    std::cout << "[spike] CPU, nanoseconds per BASE-rate sample (dev machine, Release):\n";
    std::printf("  %-14s %12s\n", "path", "ns/sample");
    std::printf("  %-14s %12.4f\n", "naive 1x", timeNaivePath(1.0f));
    std::printf("  %-14s %12.4f\n", "ADAA1 1x", timeAdaaPath(1.0f));
    std::printf("  %-14s %12.4f\n", "OS 2x", timeOversampledPath(2, 1.0f));
    std::printf("  %-14s %12.4f\n", "OS 4x", timeOversampledPath(4, 1.0f));
    std::printf("  %-14s %12.4f\n", "OS 8x", timeOversampledPath(8, 1.0f));

    // Passband transparency. In the small-signal limit ADAA1's difference quotient collapses to
    // f((x[n]+x[n-1])/2), i.e. the waveshaper behind the two-tap FIR (1 + z^-1)/2, whose magnitude
    // response is |cos(w/2)| -- -2.0 dB at 10 kHz and -11.7 dB at 20 kHz at a 48 kHz host rate.
    // Oversampling has no such term. Measured at the default drive, where the stage is near-linear
    // and the fundamental's own amplitude is therefore a clean readout of the path's gain.
    std::cout << "[spike] passband gain vs the static stage, drive 0.5, -18 dBFS (dB):\n";
    std::printf("  %10s %12s %12s\n", "tone", "ADAA1", "OS 2x");
    for (double tone : {1000.0, 4186.0, 10000.0, 15000.0, 20000.0}) {
        TriodeStage stage;
        stage.prepare(kHostRate, kBlockSize);
        stage.setParams(TriodeStageParams{});
        stage.reset();
        const int total = kWarmupSamples + static_cast<int>(kAnalysisLength);
        std::vector<double> reference;
        reference.reserve(kAnalysisLength);
        std::vector<Sample> in(static_cast<std::size_t>(kBlockSize));
        std::vector<Sample> out(static_cast<std::size_t>(kBlockSize));
        const double omega = 2.0 * kPi * tone / kHostRate;
        for (int offset = 0; offset < total; offset += kBlockSize) {
            for (int i = 0; i < kBlockSize; ++i)
                in[static_cast<std::size_t>(i)] =
                    static_cast<Sample>(kNominalPeak * std::sin(omega * static_cast<double>(offset + i)));
            stage.process(in.data(), out.data(), kBlockSize);
            if (offset >= kWarmupSamples)
                for (int i = 0; i < kBlockSize && reference.size() < kAnalysisLength; ++i)
                    reference.push_back(static_cast<double>(out[static_cast<std::size_t>(i)]));
        }
        const double staticAmp = componentAmplitude(reference, tone);
        const double adaaAmp = componentAmplitude(renderAdaa(tone, kNominalPeak, TriodeStageParams{}.drive), tone);
        const double osAmp =
            componentAmplitude(renderOversampled(tone, kNominalPeak, TriodeStageParams{}.drive, 2), tone);
        std::printf("  %10.1f %12.4f %12.4f\n", tone, 20.0 * std::log10(adaaAmp / staticAmp),
                    20.0 * std::log10(osAmp / staticAmp));
    }

    std::cout << "[spike] step-response smearing (10-90% rise, base-rate samples):\n";
    {
        constexpr int kLength = 256;
        constexpr int kStepAt = 64;
        const double peak = kNominalPeak;

        // Static reference.
        TriodeStage stage;
        stage.prepare(kHostRate, kLength);
        TriodeStageParams params;
        params.drive = 1.0f;
        stage.setParams(params);
        stage.reset();
        std::vector<Sample> in(static_cast<std::size_t>(kLength));
        for (int i = 0; i < kLength; ++i)
            in[static_cast<std::size_t>(i)] = static_cast<Sample>((i < kStepAt) ? -peak : peak);
        std::vector<Sample> out(static_cast<std::size_t>(kLength));
        stage.process(in.data(), out.data(), kLength);
        std::vector<double> staticResponse(static_cast<std::size_t>(kLength));
        for (int i = 0; i < kLength; ++i)
            staticResponse[static_cast<std::size_t>(i)] = static_cast<double>(out[static_cast<std::size_t>(i)]);

        const AdaaTransfer transfer = buildAdaaTransfer(1.0f);
        std::vector<double> adaaIn(static_cast<std::size_t>(kLength));
        for (int i = 0; i < kLength; ++i)
            adaaIn[static_cast<std::size_t>(i)] = (i < kStepAt) ? -peak : peak;
        std::vector<double> adaaOut(static_cast<std::size_t>(kLength));
        double previous = -peak;
        processAdaa(transfer, adaaIn.data(), adaaOut.data(), kLength, previous);

        std::printf("  %-14s %12.3f\n", "static 1x", stepRiseSamples(staticResponse));
        std::printf("  %-14s %12.3f\n", "ADAA1 1x", stepRiseSamples(adaaOut));

        for (int factor : {2, 4}) {
            Oversampler os;
            os.prepare(kHostRate, kLength, factor);
            TriodeStage wrapped;
            wrapped.prepare(kHostRate * static_cast<double>(factor), kLength * factor);
            wrapped.setParams(params);
            wrapped.reset();
            std::vector<Sample> osOut(static_cast<std::size_t>(kLength));
            os.processWrapped(in.data(), osOut.data(), kLength,
                              [&wrapped](Sample* buf, int n) { wrapped.process(buf, buf, n); });
            std::vector<double> osResponse(static_cast<std::size_t>(kLength));
            for (int i = 0; i < kLength; ++i)
                osResponse[static_cast<std::size_t>(i)] = static_cast<double>(osOut[static_cast<std::size_t>(i)]);
            char name[32];
            std::snprintf(name, sizeof(name), "OS %dx", factor);
            std::printf("  %-14s %12.3f\n", name, stepRiseSamples(osResponse));
        }
    }
}

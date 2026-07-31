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
//
// -------------------------------------------------------------------------------------------
// The local leakage floor, and why the coincidence radius is not enough on its own.
// -------------------------------------------------------------------------------------------
//
// Excluding a harmonic's MAIN LOBE does nothing about its SKIRT. A Blackman-Harris window leaks
// roughly -125 dBc some 80 analysis bins from a peak and roughly -146 dBc a thousand bins out --
// both orders of magnitude above any global-median noise estimate of this spectrum. So a fold
// prediction that happens to land in the neighbourhood of a strong line reads the WINDOW rather
// than the device, and does so reproducibly: a pure sine with no nonlinearity at all reproduces
// such a reading to within a fraction of a dB. Two tells in the raw tables: a "worst folded
// component" that does not move when the drive is raised, and one that matches the pure-sine
// control.
//
// Every reading therefore carries a LOCAL floor -- the level this exact readout reports at nearby
// frequencies where no line is predicted. `localFloorMagnitude` slides the same
// max-over-readout-radius operator across a neighbourhood of the target, keeps only the positions
// whose entire readout window is clear of every predicted line (harmonics AND folds), and takes the
// median of those. That is the matched null distribution for this statistic, not an approximation
// of one. A reading within `kFloorResolutionDb` of its local floor is FLOOR-LIMITED: an upper bound
// on the device, never a measurement of it, and it is printed with an `FL` marker instead of being
// quoted. Each row additionally reports the worst fold that does stand clear of its own floor.
//
// The GATE still compares the raw reading against -60 dBc, unchanged. That stays correct precisely
// because leakage can only inflate a reading and never deflate one, so a floor-limited row is a
// conservative pass, not a hidden failure.
//
// -------------------------------------------------------------------------------------------
// Why this case cannot pass vacuously.
// -------------------------------------------------------------------------------------------
//
// "The worst thing I found is quiet enough" also passes when nothing was found. Counting predicted
// frequencies does not fix that -- the prediction lists are built from f0 and the factor, never
// from the signal, so those counts are the same whatever the device does. The invariant that
// actually binds is the DEVICE's own second and third harmonics standing clear of the local floor
// by `kMinHarmonicHeadroomDb`: no nonlinearity, no harmonics, no pass. "ALIASING: engagement
// invariant rejects a linear device (negative control)" at the bottom of this file is the standing
// red-verification -- it swaps TriodeStage for the identity map and shows that invariant, and only
// that invariant, going red while everything else still looks healthy.
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

// Spectrum geometry. tests/support/SpectralAnalysis.cpp zero-pads x4, so one ANALYSIS bin spans
// four FFT bins; every radius below is expressed in FFT bins so the readout used for a reading and
// the readout used for its floor are literally the same operator.
constexpr double kAnalysisBinHz = kHostRate / static_cast<double>(kAnalysisLength);
constexpr std::ptrdiff_t kZeroPadFactor = 4;
constexpr std::ptrdiff_t kReadoutRadiusBins = 1 * kZeroPadFactor;     // +/-1 analysis bin
constexpr std::ptrdiff_t kCoincidenceRadiusBins = 8 * kZeroPadFactor; // +/-1 BH main-lobe width
constexpr double kCoincidenceRadiusHz = 8.0 * kAnalysisBinHz;

// Local-floor estimation. The neighbourhood starts at +/-64 analysis bins (~11.7 Hz) and widens
// x4 twice if the predicted-line mask leaves too few clear positions to take a stable median over.
constexpr std::ptrdiff_t kFloorHalfWidthBins = 256;
constexpr std::size_t kMinFloorSamples = 64;

// A reading this close to its own local floor is reported as floor-limited rather than quoted.
constexpr double kFloorResolutionDb = 6.0;

// The engagement invariant (see the file-level comment). H2 and H3 must clear their own local
// floors by this much for a row's aliasing reading to be evidence about the device at all.
// Measured: the shipped stage clears it by 115.8 to 121.7 dB on every row of the table, and the
// identity map lands at -0.01 dB (its "harmonics" ARE the floor). The threshold sits ~96 dB under
// the tightest real row and 20 dB over the negative control, so no plausible value in between
// changes any verdict -- 20 dB is chosen as a round number in the middle of a 116 dB gap, not
// fitted to either side.
constexpr double kMinHarmonicHeadroomDb = 20.0;

// Which nonlinearity the harness wraps. `Identity` exists only for the negative control.
enum class DeviceUnderTest { Triode, Identity };

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
std::vector<double> renderOversampled(double freqHz, double amplitude, float drive, int factor,
                                      DeviceUnderTest device = DeviceUnderTest::Triode) {
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
        os.processWrapped(in.data(), out.data(), kBlockSize, [&](Sample* buffer, int count) {
            if (device == DeviceUnderTest::Triode)
                triode.process(buffer, buffer, count);
        });
        if (offset >= kWarmupSamples) {
            for (int i = 0; i < kBlockSize && captured.size() < kAnalysisLength; ++i)
                captured.push_back(static_cast<double>(out[static_cast<std::size_t>(i)]));
        }
    }
    return captured;
}

struct AliasingMeasurement {
    // Conservative upper bound on the worst fold. This is what the gate compares, and it may be
    // floor-limited (leakage can only inflate it, so a floor-limited row is a conservative pass).
    double worstFoldedDbc = -300.0;
    double worstFoldedHz = 0.0;
    double worstFoldedFloorDbc = -300.0; // local leakage floor at that same frequency
    bool worstFoldedFloorLimited = false;
    // Worst fold that stands clear of its OWN local floor -- i.e. the strongest thing here that is
    // genuinely a measurement of the device. -300 means "nothing resolved above the window".
    double worstResolvedDbc = -300.0;
    double worstResolvedHz = 0.0;
    // min over H2, H3 of (level - local floor). The engagement invariant.
    double harmonicHeadroomDb = -300.0;
    double strongestHarmonicHz = 0.0;
    int numFolds = 0;
};

std::ptrdiff_t binOf(const cnpg::test::Spectrum& spectrum, double hz) {
    return static_cast<std::ptrdiff_t>(std::llround(spectrum.hzToBin(hz)));
}

// THE readout: max magnitude over +/-kReadoutRadiusBins FFT bins. Used both for predicted lines and,
// unchanged, for the local floor -- so a floor is always the level this same operator reports where
// nothing is predicted, and the two are directly comparable.
double readoutAtBin(const cnpg::test::Spectrum& spectrum, std::ptrdiff_t centre) {
    const auto lastBin = static_cast<std::ptrdiff_t>(spectrum.magnitudeSquared.size()) - 1;
    const std::ptrdiff_t lo = std::max<std::ptrdiff_t>(centre - kReadoutRadiusBins, 0);
    const std::ptrdiff_t hi = std::min<std::ptrdiff_t>(centre + kReadoutRadiusBins, lastBin);
    double best = 0.0;
    for (std::ptrdiff_t k = lo; k <= hi; ++k)
        best = std::max(best, spectrum.magnitudeSquared[static_cast<std::size_t>(k)]);
    return std::sqrt(best);
}

double peakMagnitudeAround(const cnpg::test::Spectrum& spectrum, double hz) {
    if (spectrum.fftSize == 0 || spectrum.magnitudeSquared.empty())
        return 0.0;
    return readoutAtBin(spectrum, binOf(spectrum, hz));
}

// Marks +/-kCoincidenceRadiusBins (one Blackman-Harris main-lobe width) around every predicted line,
// harmonic and fold alike, so no local floor can be contaminated by the very peak it is the floor
// for -- nor by any other predicted product sitting nearby.
std::vector<char> buildLineMask(const cnpg::test::Spectrum& spectrum, const std::vector<double>& lines) {
    std::vector<char> masked(spectrum.magnitudeSquared.size(), static_cast<char>(0));
    if (masked.empty())
        return masked;
    const auto lastBin = static_cast<std::ptrdiff_t>(masked.size()) - 1;
    for (double hz : lines) {
        const std::ptrdiff_t centre = binOf(spectrum, hz);
        const std::ptrdiff_t lo = std::max<std::ptrdiff_t>(centre - kCoincidenceRadiusBins, 0);
        const std::ptrdiff_t hi = std::min<std::ptrdiff_t>(centre + kCoincidenceRadiusBins, lastBin);
        for (std::ptrdiff_t k = lo; k <= hi; ++k)
            masked[static_cast<std::size_t>(k)] = static_cast<char>(1);
    }
    return masked;
}

// The local leakage floor at `hz`: median of the SAME readout evaluated at every nearby position
// whose whole readout window is clear of predicted lines. Returns 0 if the neighbourhood never
// yields enough clear positions even after widening (callers then treat the reading as unresolved
// rather than silently trusting it).
double localFloorMagnitude(const cnpg::test::Spectrum& spectrum, const std::vector<char>& masked, double hz) {
    if (spectrum.fftSize == 0 || spectrum.magnitudeSquared.empty())
        return 0.0;
    const auto lastBin = static_cast<std::ptrdiff_t>(spectrum.magnitudeSquared.size()) - 1;
    const std::ptrdiff_t centre = binOf(spectrum, hz);

    std::vector<double> readouts;
    readouts.reserve(static_cast<std::size_t>(2 * (kFloorHalfWidthBins << 4) + 1));
    for (int widen = 0; widen < 3; ++widen) {
        const std::ptrdiff_t halfWidth = kFloorHalfWidthBins << (2 * widen);
        readouts.clear();
        const std::ptrdiff_t lo = std::max<std::ptrdiff_t>(centre - halfWidth, kReadoutRadiusBins);
        const std::ptrdiff_t hi = std::min<std::ptrdiff_t>(centre + halfWidth, lastBin - kReadoutRadiusBins);
        for (std::ptrdiff_t c = lo; c <= hi; ++c) {
            bool clear = true;
            double best = 0.0;
            for (std::ptrdiff_t k = c - kReadoutRadiusBins; k <= c + kReadoutRadiusBins; ++k) {
                if (masked[static_cast<std::size_t>(k)] != 0) {
                    clear = false;
                    break;
                }
                best = std::max(best, spectrum.magnitudeSquared[static_cast<std::size_t>(k)]);
            }
            if (clear)
                readouts.push_back(best);
        }
        if (readouts.size() >= kMinFloorSamples)
            break;
    }
    if (readouts.size() < kMinFloorSamples)
        return 0.0;
    const auto mid = readouts.begin() + static_cast<std::ptrdiff_t>(readouts.size() / 2);
    std::nth_element(readouts.begin(), mid, readouts.end());
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
        if (folded > kCoincidenceRadiusHz && folded < kNyquist)
            foldHz.push_back(folded);
    }

    // Every predicted line masked before any floor is estimated.
    std::vector<double> allLines = harmonicHz;
    allLines.insert(allLines.end(), foldHz.begin(), foldHz.end());
    const std::vector<char> masked = buildLineMask(spectrum, allLines);

    double strongestHarmonic = 0.0;
    for (double hz : harmonicHz) {
        const double magnitude = peakMagnitudeAround(spectrum, hz);
        if (magnitude > strongestHarmonic) {
            strongestHarmonic = magnitude;
            result.strongestHarmonicHz = hz;
        }
    }
    if (!(strongestHarmonic > 0.0))
        return result;

    // Engagement: how far the device's own 2nd and 3rd harmonics stand above the window's local
    // leakage. A linear device has none, so this collapses to ~0 dB -- which is the whole point.
    double headroom = 1e300;
    for (int k : {2, 3}) {
        const double hz = static_cast<double>(k) * f0;
        if (hz >= kNyquist)
            continue;
        const double level = peakMagnitudeAround(spectrum, hz);
        const double floorMagnitude = localFloorMagnitude(spectrum, masked, hz);
        const double db = (level > 0.0 && floorMagnitude > 0.0) ? 20.0 * std::log10(level / floorMagnitude) : -300.0;
        headroom = std::min(headroom, db);
    }
    result.harmonicHeadroomDb = (headroom < 1e299) ? headroom : -300.0;

    double worstFold = 0.0;
    double worstFoldFloor = 0.0;
    double worstResolved = 0.0;
    const double resolutionRatio = std::pow(10.0, kFloorResolutionDb / 20.0);
    int counted = 0;
    for (double hz : foldHz) {
        bool coincides = false;
        for (double h : harmonicHz) {
            if (std::fabs(hz - h) <= kCoincidenceRadiusHz) {
                coincides = true;
                break;
            }
        }
        if (coincides)
            continue;
        ++counted;
        const double magnitude = peakMagnitudeAround(spectrum, hz);
        const double floorMagnitude = localFloorMagnitude(spectrum, masked, hz);
        if (magnitude > worstFold) {
            worstFold = magnitude;
            worstFoldFloor = floorMagnitude;
            result.worstFoldedHz = hz;
        }
        // Resolved = genuinely above this frequency's own leakage, so a device measurement rather
        // than a reading of the analysis window.
        if (floorMagnitude > 0.0 && magnitude > floorMagnitude * resolutionRatio && magnitude > worstResolved) {
            worstResolved = magnitude;
            result.worstResolvedHz = hz;
        }
    }
    result.numFolds = counted;
    result.worstFoldedDbc = (worstFold > 0.0) ? 20.0 * std::log10(worstFold / strongestHarmonic) : -300.0;
    result.worstFoldedFloorDbc =
        (worstFoldFloor > 0.0) ? 20.0 * std::log10(worstFoldFloor / strongestHarmonic) : -300.0;
    result.worstFoldedFloorLimited =
        (worstFoldFloor > 0.0) && (result.worstFoldedDbc <= result.worstFoldedFloorDbc + kFloorResolutionDb);
    result.worstResolvedDbc = (worstResolved > 0.0) ? 20.0 * std::log10(worstResolved / strongestHarmonic) : -300.0;
    return result;
}

// `gated` is the row's real status, not the drive condition's: only the DEFAULT factor is gated
// (docs/plan.md section 4.4, "Non-default factors are report-only, never gated").
std::string formatRow(const char* path, int factor, int latency, double toneHz, const DriveCondition& drive,
                      const AliasingMeasurement& m, bool gated) {
    char resolved[48];
    if (m.worstResolvedDbc > -300.0)
        std::snprintf(resolved, sizeof(resolved), "%9.2f @%8.1f", m.worstResolvedDbc, m.worstResolvedHz);
    else
        std::snprintf(resolved, sizeof(resolved), "%9s %9s", "none", "");

    char line[320];
    std::snprintf(line, sizeof(line), "  %-8s %3d %3d %8.1f %-8s %9.2f %-2s %9.2f %9.1f  %s %7.1f %5d %s", path, factor,
                  latency, toneHz, drive.name, m.worstFoldedDbc, m.worstFoldedFloorLimited ? "FL" : "  ",
                  m.worstFoldedFloorDbc, m.worstFoldedHz, resolved, m.harmonicHeadroomDb, m.numFolds,
                  gated ? "GATED" : "report");
    return std::string(line);
}

void printHeader() {
    std::cout << "  path     fac lat     tone drive     worst dBc FL     floor     at Hz   worst resolved "
                 "dBc @    Hz  H2/H3 folds status\n"
                 "  (FL = floor-limited: the reading sits within "
              << kFloorResolutionDb
              << " dB of the local window leakage and is an UPPER BOUND, not a measurement.\n"
                 "   H2/H3 = dB by which the device's own 2nd/3rd harmonics clear that same local floor.)\n";
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

                INFO(row);
                // STRUCTURAL only -- these catch a truncated or silent render, nothing more. They
                // are deliberately NOT presented as evidence: numFolds counts PREDICTED frequencies
                // derived from f0 and the factor, so it reads the same whatever the device does.
                REQUIRE(m.numFolds > 0);
                REQUIRE(m.worstFoldedDbc > -300.0);

                // EVIDENTIARY, and the invariant that actually makes this gate non-vacuous: the
                // device's own 2nd and 3rd harmonics must stand clear of the local window leakage.
                // Only a real nonlinearity can satisfy it. "ALIASING: engagement invariant rejects a
                // linear device (negative control)" below is the standing red-verification.
                REQUIRE(m.harmonicHeadroomDb >= kMinHarmonicHeadroomDb);

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

TEST_CASE("ALIASING: engagement invariant rejects a linear device (negative control)", "[aliasing]") {
    // Standing red-verification for the gate above. Substituting the identity map for
    // TriodeStage::process leaves the harness structurally intact -- same prediction lists, same
    // fold count, same finite worst-fold reading -- so every counting-style invariant still passes
    // and the gate's own -60 dBc comparison reports a comfortable PASS while measuring nothing but
    // the analysis window. Exactly one invariant notices, and this case pins that.
    //
    // If the first CHECK below ever goes the other way, the gate above has stopped being evidence
    // about the device and this file needs rereading before its numbers are quoted anywhere.
    const std::vector<double> linear =
        renderOversampled(kToneHigh, kNominalPeak, 1.0f, Oversampler::kDefaultFactor, DeviceUnderTest::Identity);
    const AliasingMeasurement m = measureAliasing(linear, kToneHigh, Oversampler::kDefaultFactor);

    Oversampler probe;
    probe.prepare(kHostRate, kBlockSize, Oversampler::kDefaultFactor);

    std::cout << "[aliasing] negative control -- identity nonlinearity through the same wrapper:\n";
    printHeader();
    const std::string row = formatRow("identity", Oversampler::kDefaultFactor, probe.latencySamples(), kToneHigh,
                                      kDriveConditions[1], m, false);
    std::cout << row << "\n";
    INFO(row);

    // THE point: a linear device generates no harmonics, so H2/H3 sit on the leakage floor and the
    // gate's evidentiary invariant fails.
    CHECK(m.harmonicHeadroomDb < kMinHarmonicHeadroomDb);

    // ... while everything a counting-style invariant can see still looks perfectly healthy. These
    // three CHECKs are the vacuous pass itself, pinned so it cannot quietly come back.
    CHECK(m.numFolds > 0);
    CHECK(m.worstFoldedDbc > -300.0);
    CHECK(m.worstFoldedDbc <= kGateDbc);

    // And the reading it would have published is floor-limited, i.e. a measurement of the window.
    CHECK(m.worstFoldedFloorLimited);
    CHECK(m.worstResolvedDbc == -300.0); // nothing at all resolved above the leakage

    std::cout << "[aliasing] negative control: H2/H3 headroom " << m.harmonicHeadroomDb
              << " dB (gate requires >= " << kMinHarmonicHeadroomDb << " dB), worst folded reading " << m.worstFoldedDbc
              << " dBc -- floor-limited, and " << (kGateDbc - m.worstFoldedDbc)
              << " dB inside the -60 dBc limit it would have 'passed'.\n";
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

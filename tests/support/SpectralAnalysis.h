#pragma once

#include <cstddef>
#include <vector>

// SpectralAnalysis -- the canonical frequency-domain measurement kit shared by the [tuning] and
// [regression] suites. docs/plan.md section 4.5 pins the estimator exactly ("Windowed FFT +
// quadratic (parabolic) interpolation: take 2^18 analysis samples at 44.1/48 kHz (2^19 at
// 96 kHz ...), Blackman-Harris window, zero-pad x4, FFT, locate the fundamental partial's peak
// bin (search restricted to +/-80 cents around the target f0 ...), then fit a parabola through
// the log-magnitude of the peak and its neighbors to refine the frequency"), and section 4.3
// layer (a) reuses the same estimator for partials 1-8 plus a per-octave-band T60. Both live
// here so there is exactly one implementation of each measurement in the repo.

namespace cnpg::test {

// docs/plan.md section 4.5: the tuning sweep's peak search is restricted to +/-80 cents around
// the nominal so a dispersion-sharpened upper partial can never be picked up as the fundamental.
inline constexpr double kTuningSearchCents = 80.0;

// Power spectrum of a real signal: Blackman-Harris window over the largest power-of-two prefix
// of `samples`, x4 zero padding, radix-2 FFT. `magnitudeSquared` holds bins 0..fftSize/2.
struct Spectrum {
    std::vector<double> magnitudeSquared;
    double sampleRate = 0.0;
    std::size_t fftSize = 0;

    double binToHz(double bin) const { return bin * sampleRate / static_cast<double>(fftSize); }
    double hzToBin(double hz) const { return hz * static_cast<double>(fftSize) / sampleRate; }
};

// `analysisLength` 0 means "largest power of two <= samples.size()".
Spectrum computeSpectrum(const std::vector<double>& samples, double sampleRate, std::size_t analysisLength = 0);

// Peak frequency within +/-searchCents of targetHz, refined by a parabola fitted through the
// log-magnitude of the peak bin and its two neighbours. Returns 0.0 if the search window holds
// no usable peak (e.g. the band is empty at this note).
double findPeakHz(const Spectrum& spectrum, double targetHz, double searchCents);

// Cents difference measured against `referenceHz`; positive means sharp.
double centsBetween(double measuredHz, double referenceHz);

// Equal temperament, A4 = 440 Hz.
double midiNoteToHz(int midiNote);

// Per-octave-band T60 in seconds: a 4th-order octave bandpass (two cascaded RBJ sections -- a
// single biquad's skirts leak a low note's fundamental into every band, see the .cpp) plus
// Schroeder backward integration, fitted over the -5 dB .. -25 dB span of the decay curve and
// extrapolated (T20 * 3). Returns a negative value when the band carries too little energy for
// a valid fit -- callers treat that as "band not present at this note" rather than a failure.
double bandT60Seconds(const std::vector<double>& samples, double sampleRate, double centreHz);

// RMS in dBFS over the first `windowSeconds` of the signal.
double rmsDbfs(const std::vector<double>& samples, double sampleRate, double windowSeconds);

} // namespace cnpg::test

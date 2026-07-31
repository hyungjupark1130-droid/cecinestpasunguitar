#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::PluckExciter;
using cnpg::dsp::PluckExciterParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

// docs/plan.md section 4.5 render recipe: NoteOn velocity 0.8, pluckPosition 0.28, hardness 0.5,
// noiseAmount 0; render 7 s of the tap channel; discard the first 0.5 s of attack.
constexpr double kRenderSeconds = 7.0;
constexpr double kDiscardSeconds = 0.5;
constexpr float kVelocity = 0.8f;
constexpr float kPluckPosition = 0.28f;
constexpr float kHardness = 0.5f;
constexpr float kTapPosition = 0.87f;

// docs/plan.md section 4.5 gates MIDI 33-96 in P1 and emits 21-32 / 97-108 report-only.
constexpr int kGateLowMidi = 33;
constexpr int kGateHighMidi = 96;
constexpr double kGateCents = 2.0;

// SUSTAIN SETTING FOR THE SWEEP -- deliberate, documented, and NOT a loosening of the gate.
// The mandated estimator analyses 2^18 samples (2^19 at 96 kHz) starting 0.5 s after the pluck.
// At the DEFAULT material the loop loss gives MIDI 108 a T60 of ~65 ms, so that window would
// contain digital silence and the measurement would be of nothing at all. The sweep therefore
// runs the material knobs at their sustain end (1.0), which is the same filters, the same code
// path and the same analytic compensation -- only a parameter value inside the shipping range.
// `TUNING: analytic compensation is material-independent` below re-measures at the DEFAULT
// material everywhere the default's own decay still leaves a tone to measure, and holds the same
// +/-2 cents, which is what proves the sustain setting is not doing any of the work.
constexpr float kSweepLossKnob = 1.0f;

std::size_t analysisLengthFor(double sampleRate) {
    // docs/plan.md section 4.5: 2^18 analysis samples at 44.1/48 kHz, 2^19 at 96 kHz (keeping bin
    // spacing ~0.18 Hz).
    return (sampleRate > 60000.0) ? (std::size_t{1} << 19) : (std::size_t{1} << 18);
}

std::vector<double> renderTapChannel(FractionalDelayKind kind, double sampleRate, double f0Hz, float bendSemitones,
                                     float lossKnob, float dispersionKnob) {
    WaveguideString<float> string;
    string.prepare(sampleRate, 512, kind);

    WaveguideStringParams params;
    params.f0Hz = static_cast<float>(f0Hz);
    params.bendSemitones = bendSemitones;
    params.material.lossGainLow = lossKnob;
    params.material.lossGainHigh = lossKnob;
    params.material.dispersionAmount = dispersionKnob;
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset(); // snaps the smoothers, so f0 is exact from the first rendered sample

    PluckExciter<float> exciter;
    exciter.prepare(sampleRate, 512);
    PluckExciterParams exciterParams;
    exciterParams.noiseAmount = 0.0f;
    exciter.setParams(exciterParams);
    exciter.trigger(kVelocity, kPluckPosition, kHardness);

    const auto total = static_cast<std::size_t>(kRenderSeconds * sampleRate);
    const auto discard = static_cast<std::size_t>(kDiscardSeconds * sampleRate);
    const std::size_t wanted = analysisLengthFor(sampleRate);

    std::vector<double> out;
    out.reserve(wanted);
    for (std::size_t n = 0; n < total; ++n) {
        const float excitation = exciter.renderSample();
        if (excitation != 0.0f)
            string.injectAt(exciter.latchedPosition01(), excitation);
        const float tap = string.readTapAt(kTapPosition);
        if (n >= discard && out.size() < wanted)
            out.push_back(static_cast<double>(tap));
        string.tick();
    }
    return out;
}

// The canonical estimator of docs/plan.md section 4.5, start to finish.
double measureF0Hz(const std::vector<double>& samples, double sampleRate, double targetHz) {
    const cnpg::test::Spectrum spectrum =
        cnpg::test::computeSpectrum(samples, sampleRate, analysisLengthFor(sampleRate));
    return cnpg::test::findPeakHz(spectrum, targetHz, cnpg::test::kTuningSearchCents);
}

double measureCentsError(FractionalDelayKind kind, double sampleRate, int midiNote, float lossKnob = kSweepLossKnob,
                         float dispersionKnob = 0.0f) {
    const double target = cnpg::test::midiNoteToHz(midiNote);
    const std::vector<double> tap = renderTapChannel(kind, sampleRate, target, 0.0f, lossKnob, dispersionKnob);
    const double measured = measureF0Hz(tap, sampleRate, target);
    if (measured <= 0.0)
        return 1e9; // no usable peak -- fails every gate loudly
    return cnpg::test::centsBetween(measured, target);
}

std::string formatRow(int midiNote, double sampleRate, double cents) {
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer), "  MIDI %3d  %6.0f Hz  %9.1f Hz  %+8.3f cents", midiNote, sampleRate,
                  cnpg::test::midiNoteToHz(midiNote), cents);
    return std::string(buffer);
}

const double kRates[3] = {44100.0, 48000.0, 96000.0};

// The shipping fractional-delay default (docs/decisions/0002-fractional-delay.md). The gate binds
// this kind; the loser stays compiled and golden-covered.
constexpr FractionalDelayKind kShippingKind = FractionalDelayKind::Lagrange3;

} // namespace

// ---------------------------------------------------------------------------------------------
// mandatory estimator self-calibration (docs/plan.md section 4.5)
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: estimator self-calibration", "[tuning]") {
    // Synthetic exponentially decaying sinusoids with low-level harmonics at known frequencies
    // spanning 27.5 Hz - 4186 Hz; docs/plan.md section 4.5 requires estimator error <= 0.2 cents
    // everywhere -- a 10x guard band under the 2-cent gate. The sweep case below is invalid if
    // this fails.
    constexpr double kSelfCalCents = 0.2;
    const double frequencies[] = {27.5,    32.7032, 55.0,    82.4069, 110.0,   164.814, 220.0,
                                  329.628, 440.0,   659.255, 880.0,   1318.51, 2093.0,  4186.01};

    double worst = 0.0;
    for (double sampleRate : kRates) {
        const std::size_t length = analysisLengthFor(sampleRate);
        for (double f0 : frequencies) {
            if (f0 >= 0.45 * sampleRate)
                continue;

            std::vector<double> signal(length);
            // Decay time constant deliberately in the same ballpark as a ringing string, so the
            // estimator is calibrated on the shape of signal it actually has to measure.
            const double tau = 1.5;
            const double amplitudes[4] = {1.0, 0.35, 0.18, 0.09};
            const double phases[4] = {0.31, 1.17, 2.44, 0.83};
            for (std::size_t n = 0; n < length; ++n) {
                const double t = static_cast<double>(n) / sampleRate;
                double v = 0.0;
                for (int k = 0; k < 4; ++k) {
                    const double fk = f0 * static_cast<double>(k + 1);
                    if (fk >= 0.5 * sampleRate)
                        break;
                    v += amplitudes[k] * std::sin(6.283185307179586 * fk * t + phases[k]);
                }
                signal[n] = v * std::exp(-t / tau);
            }

            const double measured = measureF0Hz(signal, sampleRate, f0);
            REQUIRE(measured > 0.0);
            const double cents = cnpg::test::centsBetween(measured, f0);
            INFO("rate " << sampleRate << " f0 " << f0 << " measured " << measured << " error " << cents << " cents");
            REQUIRE(std::fabs(cents) <= kSelfCalCents);
            worst = std::max(worst, std::fabs(cents));
        }
    }

    std::cout << "[tuning] estimator self-calibration: worst error " << worst << " cents (limit " << kSelfCalCents
              << ")\n";
}

// ---------------------------------------------------------------------------------------------
// P1 analytic compensation sweep
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: P1 analytic compensation sweep", "[tuning]") {
    double worstGated = 0.0;
    int worstGatedNote = 0;
    double worstGatedRate = 0.0;
    std::vector<std::string> reportOnly;

    for (double sampleRate : kRates) {
        for (int midiNote = cnpg::dsp::kMinMidiNote; midiNote <= cnpg::dsp::kMaxMidiNote; ++midiNote) {
            const double cents = measureCentsError(kShippingKind, sampleRate, midiNote);
            const bool gated = (midiNote >= kGateLowMidi && midiNote <= kGateHighMidi);

            if (gated) {
                INFO("gated note " << midiNote << " at " << sampleRate << " Hz: " << cents << " cents");
                REQUIRE(std::fabs(cents) <= kGateCents);
                if (std::fabs(cents) > worstGated) {
                    worstGated = std::fabs(cents);
                    worstGatedNote = midiNote;
                    worstGatedRate = sampleRate;
                }
            } else {
                // docs/plan.md section 4.5: MIDI 21-32 and 97-108 are report-only in P1; the full
                // 88-note assertion binds only the P2 calibration-table case.
                reportOnly.push_back(formatRow(midiNote, sampleRate, cents));
            }
        }
    }

    std::cout << "[tuning] P1 analytic compensation sweep (report-only bands, MIDI 21-32 and 97-108):\n";
    for (const std::string& row : reportOnly)
        std::cout << row << "\n";
    std::cout << "[tuning] gated band MIDI " << kGateLowMidi << "-" << kGateHighMidi << ": worst |error| " << worstGated
              << " cents at MIDI " << worstGatedNote << " / " << worstGatedRate << " Hz (limit " << kGateCents << ")\n";
}

// ---------------------------------------------------------------------------------------------
// material independence -- the guard on the sweep's sustain setting
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: analytic compensation is material-independent", "[tuning]") {
    // The sweep runs the material at its sustain end so the mandated analysis window contains a
    // tone at all. This case re-measures at the DEFAULT material (and at full dispersion) over
    // the range where the default's own decay still leaves something to analyse, and holds the
    // same +/-2 cents -- so the sustain setting buys measurability, never accuracy.
    constexpr double kSampleRate = 48000.0;
    double worstDefault = 0.0;
    double worstDispersed = 0.0;

    for (int midiNote = 33; midiNote <= 72; midiNote += 3) {
        const cnpg::dsp::StringMaterialParams defaults{};
        const double defaultCents =
            measureCentsError(kShippingKind, kSampleRate, midiNote, defaults.lossGainLow, defaults.dispersionAmount);
        INFO("default material, MIDI " << midiNote << ": " << defaultCents << " cents");
        REQUIRE(std::fabs(defaultCents) <= kGateCents);
        worstDefault = std::max(worstDefault, std::fabs(defaultCents));

        // R4 (docs/plan.md section 5) watches exactly this: analytic compensation drifting as
        // dispersionAmount rises.
        const double dispersedCents = measureCentsError(kShippingKind, kSampleRate, midiNote, kSweepLossKnob, 1.0f);
        INFO("full dispersion, MIDI " << midiNote << ": " << dispersedCents << " cents");
        REQUIRE(std::fabs(dispersedCents) <= kGateCents);
        worstDispersed = std::max(worstDispersed, std::fabs(dispersedCents));
    }

    std::cout << "[tuning] material independence at 48 kHz: worst |error| " << worstDefault
              << " cents (default material), " << worstDispersed << " cents (dispersionAmount = 1)\n";
}

// ---------------------------------------------------------------------------------------------
// static bend accuracy (docs/plan.md section 4.5 third named case)
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: static bend accuracy", "[tuning]") {
    constexpr int kMidiNote = 40;
    for (double sampleRate : kRates) {
        for (float bend : {-2.0f, 2.0f}) {
            const double base = cnpg::test::midiNoteToHz(kMidiNote);
            const double target = base * std::exp2(static_cast<double>(bend) / 12.0);
            const std::vector<double> tap =
                renderTapChannel(kShippingKind, sampleRate, base, bend, kSweepLossKnob, 0.0f);
            const double measured = measureF0Hz(tap, sampleRate, target);
            REQUIRE(measured > 0.0);
            const double cents = cnpg::test::centsBetween(measured, target);
            INFO("bend " << bend << " semitones at " << sampleRate << " Hz: " << cents << " cents");
            REQUIRE(std::fabs(cents) <= kGateCents);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// hidden: full 88-note x 3-rate x 2-kind table for docs/decisions/0002-fractional-delay.md
// ---------------------------------------------------------------------------------------------

// Deliberately NOT tagged [tuning]: the plan documents running tag filters straight against the
// binary (`cnpg_tests.exe "[tuning]"`), and hidden cases still run when a filter matches one of
// their tags -- this would add a minute to every such run. Select it by name or by "[report]".
TEST_CASE("TUNING: full comparison table (report generator)", "[.][report]") {
    std::cout << "kind,sampleRate,midiNote,targetHz,cents\n";
    for (FractionalDelayKind kind : {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1}) {
        const char* name = (kind == FractionalDelayKind::Lagrange3) ? "lagrange3" : "thiran1";
        for (double sampleRate : kRates) {
            for (int midiNote = cnpg::dsp::kMinMidiNote; midiNote <= cnpg::dsp::kMaxMidiNote; ++midiNote) {
                const double cents = measureCentsError(kind, sampleRate, midiNote);
                char buffer[512];
                std::snprintf(buffer, sizeof(buffer), "%s,%.0f,%d,%.4f,%.6f", name, sampleRate, midiNote,
                              cnpg::test::midiNoteToHz(midiNote), cents);
                std::cout << buffer << "\n";
            }
        }
    }
    SUCCEED();
}

#include "cnpg/dsp/BridgeTuning.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
// THE +/-2 CENT CRITERION. docs/plan.md section 4.5 binds it to the P2 CALIBRATION-TABLE case --
// "the full 88-note (21-108) x 3-rate +/-2-cent assertion binds only the P2 calibration-table case"
// -- and Task P2.4 is where that distinction stopped being academic. See kAnalyticSanityCents.
constexpr double kGateCents = 2.0;

// *** THE +/-2 CENT LINE IS BACK ON THE SHIPPING TOPOLOGY (Task P2.7). ***
//
// The history, because the number below moved twice and each move has to be readable.
//
// P1: the analytic compensation is a closed-form solve over the loop's own filters -- rails,
// fractional interpolator, dispersion chain, loop loss -- derived for a string terminated by a rigid
// -1, and EXACT for one (0.00028 cents worst over the gated band).
//
// P2.4 gave the shipping topology a loaded bridge, and a bridge with a resonance pulls the partials
// near it. The compensation could not absorb a load it was derived without, the measured worst went
// to 4.90 cents, and this file's gated bound became a documented +/-12 cent SANITY bound with the
// +/-2 cent obligation recorded as a binding entry condition on Task P2.7.
//
// P2.7 DISCHARGES IT. The bridge port's own self-reflectance now contributes its PHASE delay to the
// loop-length solve (dsp/include/cnpg/dsp/BridgeTuning.h, ADR 0007 D1), so the compensation is
// derived WITH the load rather than without it. Measured on the sweep below: worst 0.0598 cents over
// MIDI 33-96 at all three rates, against the 4.90 cents P2.4 recorded for the identical render.
//
// So the bound here is the CRITERION again, not a sanity bound, and it is kGateCents. What is kept
// from P2.4 is the attribution structure -- the decoupled control below -- because that is what says
// a future failure is the bridge term and not the P1 solve underneath it.
constexpr double kAnalyticSanityCents = kGateCents;

// The measured worst on this sweep is 0.0598 cents. This is what the sweep must stay UNDER for the
// gate to be measuring a working compensation rather than merely a passing one: the +/-2 cent line
// would be cleared by a compensation that had silently lost 97% of its effect. Set with ~4x headroom
// over the measurement, at all three rates and both bands.
constexpr double kCompensatedWorstCents = 0.25;

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

// THE SHIPPING TOPOLOGY, and from Task P2.4 that means something it did not mean before.
//
// docs/plan.md section 4.5's render recipe is "StringNetwork with 1 active string, damper
// transparent, DEFAULT BRIDGE ADMITTANCE ATTACHED (tuning is accepted against the shipping topology,
// which is also what cnpg_calibrate measures in P2)". Through P2.3 this function rendered an
// ISOLATED WaveguideString instead, and that was harmless only by accident: couplingStrength
// defaulted to 0.0, where BridgeJunction reduces bit-exactly to the rigid termination an isolated
// string applies internally, so the two topologies were the same object. Task P2.4's nonzero default
// (docs/decisions/0006) ends that equivalence, and the deviation stopped being harmless -- the
// isolated render reads 0.00028 cents where the shipping one reads several cents.
//
// Rendering the isolated string would now be measuring a topology that ships nowhere. Since what
// section 4.5 is FOR is accepting the instrument's tuning, it renders the instrument.
std::vector<double> renderTapChannel(FractionalDelayKind kind, double sampleRate, double f0Hz, float bendSemitones,
                                     float lossKnob, float dispersionKnob,
                                     float coupling = cnpg::dsp::BridgeAdmittanceParams{}.couplingStrength) {
    // `f0Hz` is the note's UNBENT nominal and `bendSemitones` is applied on top of it by the
    // network's own parameter path -- exactly as the isolated-string version set params.f0Hz and
    // params.bendSemitones separately. Subtracting the bend here would play a different note and
    // then bend it back to the nominal, which is not what "static bend accuracy" measures. (It also
    // read 57.7 cents when it was wrong, which is how this was caught.)
    const int midiNote = static_cast<int>(std::lround(69.0 + 12.0 * std::log2(f0Hz / 440.0)));

    cnpg::dsp::StringNetworkParams params;
    params.pickupPosition01 = kTapPosition;
    params.pitchBendSemitones = bendSemitones;
    params.stringMaterial.lossGainLow = lossKnob;
    params.stringMaterial.lossGainHigh = lossKnob;
    params.stringMaterial.dispersionAmount = dispersionKnob;
    params.exciter.noiseAmount = 0.0f;
    // Defaults to the struct's own couplingStrength ON PURPOSE: that IS "default bridge admittance
    // attached", and it is the whole point of re-pointing this render. The parameter exists so one
    // case can render the DECOUPLED control and attribute the residual (see the sweep below).
    params.bridge.couplingStrength = coupling;

    cnpg::dsp::StringNetwork<float> network;
    network.prepare(sampleRate, 512, kind);
    network.setNumStrings(1);
    network.setParams(params);
    network.reset(); // snaps the smoothers, so f0 is exact from the first rendered sample

    cnpg::dsp::NoteEvent noteOn{};
    noteOn.type = cnpg::dsp::NoteEventType::NoteOn;
    noteOn.sampleOffset = 0;
    noteOn.stringIndex = 0;
    noteOn.channel = 0;
    noteOn.midiNote = static_cast<std::uint8_t>(std::clamp(midiNote, cnpg::dsp::kMinMidiNote, cnpg::dsp::kMaxMidiNote));
    noteOn.velocity = kVelocity;
    noteOn.pluckPosition = kPluckPosition;
    noteOn.hardness = kHardness;
    cnpg::dsp::BlockEventQueue events;
    events.push(noteOn);

    const auto total = static_cast<std::size_t>(kRenderSeconds * sampleRate);
    const auto discard = static_cast<std::size_t>(kDiscardSeconds * sampleRate);
    const std::size_t wanted = analysisLengthFor(sampleRate);

    std::vector<double> out;
    out.reserve(wanted);
    std::size_t rendered = 0;
    while (rendered < total && out.size() < wanted) {
        network.process(events, 512);
        const float* channel = network.tapBuffers().channel(0, 0);
        for (int n = 0; n < 512 && out.size() < wanted; ++n, ++rendered)
            if (rendered >= discard)
                out.push_back(static_cast<double>(channel[n]));
    }
    // The port really drove every tick: without this the case could silently be measuring the
    // internal rigid termination again, which is exactly the deviation being corrected.
    REQUIRE(network.unbridgedTicks() == 0);
    return out;
}

// The canonical estimator of docs/plan.md section 4.5, start to finish.
double measureF0Hz(const std::vector<double>& samples, double sampleRate, double targetHz) {
    const cnpg::test::Spectrum spectrum =
        cnpg::test::computeSpectrum(samples, sampleRate, analysisLengthFor(sampleRate));
    return cnpg::test::findPeakHz(spectrum, targetHz, cnpg::test::kTuningSearchCents);
}

double measureCentsError(FractionalDelayKind kind, double sampleRate, int midiNote, float lossKnob = kSweepLossKnob,
                         float dispersionKnob = 0.0f,
                         float coupling = cnpg::dsp::BridgeAdmittanceParams{}.couplingStrength) {
    const double target = cnpg::test::midiNoteToHz(midiNote);
    const std::vector<double> tap =
        renderTapChannel(kind, sampleRate, target, 0.0f, lossKnob, dispersionKnob, coupling);
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
                REQUIRE(std::fabs(cents) <= kAnalyticSanityCents);
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
    std::cout << "[tuning] band MIDI " << kGateLowMidi << "-" << kGateHighMidi
              << " IN THE SHIPPING COUPLED TOPOLOGY: worst |error| " << worstGated << " cents at MIDI "
              << worstGatedNote << " / " << worstGatedRate << " Hz (gate +/-" << kGateCents
              << "; P2.4 measured 4.90 cents on this identical render before the bridge phase-delay compensation)\n";

    // NOT MERELY UNDER THE GATE -- under the number a WORKING compensation produces. The +/-2 cent
    // criterion would be cleared by a compensation that had lost 97% of its effect, so the bound
    // that actually guards this is kCompensatedWorstCents. See its declaration.
    REQUIRE(worstGated < kCompensatedWorstCents);

    // THE RENDER REALLY WENT THROUGH A LOADED BRIDGE, asserted on the state rather than inferred
    // from the residual. Through P2.4 this was the job of `worstGated > 0.5` -- "if the error is
    // small the bridge must have fallen out of the render" -- and that reasoning is exactly what
    // P2.7 invalidates: the error is small now BECAUSE the bridge is in the render and compensated.
    // Replacing an inference with the direct observation is the P2.1 ruling applied to this file.
    {
        cnpg::dsp::StringNetworkParams params;
        params.stringMaterial.lossGainLow = kSweepLossKnob;
        params.stringMaterial.lossGainHigh = kSweepLossKnob;
        cnpg::dsp::StringNetwork<float> probe;
        probe.prepare(48000.0, 512, kShippingKind);
        probe.setNumStrings(1);
        probe.setParams(params);
        probe.reset();
        cnpg::dsp::NoteEvent on{};
        on.type = cnpg::dsp::NoteEventType::NoteOn;
        on.stringIndex = 0;
        on.midiNote = 45;
        on.velocity = kVelocity;
        on.pluckPosition = kPluckPosition;
        on.hardness = kHardness;
        cnpg::dsp::BlockEventQueue events;
        events.push(on);
        probe.process(events, 512);
        // The shipping default admittance contributes a real, nonzero phase delay, and the string is
        // tuned with EXACTLY that -- asserted against a freshly solved value, never against a
        // magnitude.
        //
        // *** THIS DELIBERATELY DOES NOT ENCODE THE NUMBER, AND THAT IS THE WHOLE POINT. *** The
        // first cut of P2.7 asserted `|compensation| > 1.0` here, which is a test of the PROVISIONAL
        // couplingStrength default rather than of the compensation. The phase delay is exactly linear
        // in coupling -- measured at MIDI 45 / 48 kHz, res 180 / zeta 0.5: 0.173569 / 0.347140 /
        // 0.694288 / 1.041452 / 1.215043 samples at c = 0.05 / 0.10 / 0.20 / 0.30 / 0.35, i.e.
        // 3.4714 samples per unit coupling to five digits -- so |tau| crosses 1.0 sample at
        // c = 0.288, and that bound would have FAILED for any default below it. ADR 0007 D4 leaves
        // the default provisional and schedules P2.8 to compare LOWER values, with its own evidence
        // (beat depth 10.08 dB at 0.1 against 3.59 at 0.35) pointing at 0.1-0.2. A gate that breaks
        // when a provisional default moves in the direction its own ADR predicts is a gate on the
        // wrong quantity.
        //
        // Both halves of the original claim survive without the magnitude: the compensation IN FORCE
        // must be what the solve produced (the string is tuned with the answer, not with something
        // else), and that answer must be NONZERO -- a zero means either the coupling default went
        // back to 0 or the compensation stopped being pushed, and either way the sweep above would
        // still pass while measuring something else.
        const cnpg::dsp::IBridgePort<float>* attached = probe.attachedBridgePort();
        REQUIRE(attached != nullptr);
        // The very arguments StringNetwork::applyStringParams() solves with: the string's own bent
        // target (bend 0 here, and f0Hz is a float, so the round-trip through float is part of the
        // number), port index 0, one loading port.
        const double solvedTargetHz = static_cast<double>(static_cast<float>(cnpg::test::midiNoteToHz(45)));
        const cnpg::dsp::BridgeTuningSolution solved =
            cnpg::dsp::solveBridgeTuning(*attached, 0, solvedTargetHz, 48000.0, 1);
        INFO("bridge compensation in force at MIDI 45 / 48 kHz: "
             << probe.bridgeCompensationSamples(0) << " samples; freshly solved " << solved.phaseDelaySamples);
        REQUIRE(solved.phaseDelaySamples != 0.0);
        // Exact equality is available and is therefore what is asserted: the 8 ms bridge ramp is 384
        // samples at 48 kHz and LANDS (it is not the one-pole the other smoothers use), so 512
        // samples of render puts the smoother exactly on its target, and the only transformation
        // between the two sides is setBridgePhaseDelaySamples()'s float parameter.
        REQUIRE(probe.bridgeCompensationSamples(0) ==
                static_cast<double>(static_cast<float>(solved.phaseDelaySamples)));
        REQUIRE(probe.bridgeTuningConverged(0));
        // ...and no solve in this render landed on the fallback, i.e. the instrument spent none of it
        // outside its own tuning guarantee. A whole-render statement, which is why the diagnostic is
        // a running count rather than a flag (StringNetwork.h). Driven NON-ZERO in
        // tests/dsp/TuningAccuracyTests.cpp, so this is not a bound nothing can violate.
        REQUIRE(probe.bridgeTuningFallbacks() == 0);
        REQUIRE(probe.unbridgedTicks() == 0);
    }

    // THE ATTRIBUTION CONTROL, kept from P2.4 and still doing its job. Rendering the identical sweep
    // with the bridge DECOUPLED isolates the P1 solve from the P2.7 bridge term: decoupled, the
    // analytic compensation is the exact closed form it was derived as, and the bridge contributes
    // nothing. If a future change breaks the P1 solve, BOTH arms fail; if it breaks only the bridge
    // term, only the coupled arm does. That is the whole reason to keep measuring a topology that
    // ships nowhere.
    double worstDecoupled = 0.0;
    int worstDecoupledNote = 0;
    for (double sampleRate : kRates) {
        for (int midiNote : {33, 45, 57, 69, 81, 96}) {
            const double cents = measureCentsError(kShippingKind, sampleRate, midiNote, kSweepLossKnob, 0.0f, 0.0f);
            INFO("decoupled control, MIDI " << midiNote << " at " << sampleRate << " Hz: " << cents << " cents");
            REQUIRE(std::fabs(cents) <= kGateCents);
            if (std::fabs(cents) > worstDecoupled) {
                worstDecoupled = std::fabs(cents);
                worstDecoupledNote = midiNote;
            }
        }
    }
    std::cout << "[tuning] DECOUPLED control (couplingStrength 0, same render otherwise): worst |error| "
              << worstDecoupled << " cents at MIDI " << worstDecoupledNote << " (limit " << kGateCents
              << ") -- the P1 solve alone, with no load to correct for\n";
    // The decoupled arm is the exact closed form, so it must remain an order of magnitude tighter
    // than the compensated coupled arm even now that the coupled arm is itself under a tenth of a
    // cent. `< 0.1 * worstGated` was the P2.4 form of this and it still holds; stated against the
    // absolute figure too, so it cannot be satisfied by the coupled arm degrading.
    REQUIRE(worstDecoupled < 0.1 * worstGated);
    REQUIRE(worstDecoupled < 0.01);
}

// ---------------------------------------------------------------------------------------------
// material independence -- the guard on the sweep's sustain setting
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: analytic compensation is material-independent", "[tuning]") {
    // The sweep runs the material at its sustain end so the mandated analysis window contains a
    // tone at all. This case re-measures at the DEFAULT material (and at full dispersion) over
    // the range where the default's own decay still leaves something to analyse, and holds the
    // same bound -- so the sustain setting buys measurability, never accuracy. (The bound is the
    // shipping-topology sanity bound from Task P2.4, not +/-2 cents; see kAnalyticSanityCents.)
    constexpr double kSampleRate = 48000.0;
    double worstDefault = 0.0;
    double worstDispersed = 0.0;

    for (int midiNote = 33; midiNote <= 72; midiNote += 3) {
        const cnpg::dsp::StringMaterialParams defaults{};
        const double defaultCents =
            measureCentsError(kShippingKind, kSampleRate, midiNote, defaults.lossGainLow, defaults.dispersionAmount);
        INFO("default material, MIDI " << midiNote << ": " << defaultCents << " cents");
        REQUIRE(std::fabs(defaultCents) <= kAnalyticSanityCents);
        worstDefault = std::max(worstDefault, std::fabs(defaultCents));

        // R4 (docs/plan.md section 5) watches exactly this: analytic compensation drifting as
        // dispersionAmount rises.
        const double dispersedCents = measureCentsError(kShippingKind, kSampleRate, midiNote, kSweepLossKnob, 1.0f);
        INFO("full dispersion, MIDI " << midiNote << ": " << dispersedCents << " cents");
        REQUIRE(std::fabs(dispersedCents) <= kAnalyticSanityCents);
        worstDispersed = std::max(worstDispersed, std::fabs(dispersedCents));
    }

    std::cout << "[tuning] material independence at 48 kHz: worst |error| " << worstDefault
              << " cents (default material), " << worstDispersed << " cents (dispersionAmount = 1)\n";
}

// ---------------------------------------------------------------------------------------------
// static bend accuracy (docs/plan.md section 4.5 third named case)
// ---------------------------------------------------------------------------------------------

TEST_CASE("TUNING: static bend accuracy", "[tuning]") {
    // Section 4.5's third named case, and from Task P2.4 it renders the SHIPPING topology like the
    // other two (renderTapChannel above). Its bound moves with them, and for the same reason: the
    // bridge load's pull is not something a bend parameter can be held responsible for. What this
    // case still measures exactly is that the bend ARITHMETIC lands where it should -- the residual
    // at +/-2 semitones must not exceed the residual the unbent sweep already reports.
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
            std::cout << "[tuning] static bend " << bend << " st at " << sampleRate
                      << " Hz (shipping topology): " << cents << " cents (sanity bound " << kAnalyticSanityCents
                      << ")\n";
            REQUIRE(std::fabs(cents) <= kAnalyticSanityCents);
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

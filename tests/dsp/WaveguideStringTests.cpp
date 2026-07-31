#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/WaveguideString.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <iostream>
#include <random>
#include <vector>

using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::PluckExciter;
using cnpg::dsp::PluckExciterParams;
using cnpg::dsp::StringMaterialParams;
using cnpg::dsp::WaveguideString;
using cnpg::dsp::WaveguideStringParams;

namespace {

constexpr double kRates[3] = {44100.0, 48000.0, 96000.0};
constexpr FractionalDelayKind kKinds[2] = {FractionalDelayKind::Lagrange3, FractionalDelayKind::Thiran1};

const char* kindName(FractionalDelayKind kind) {
    return kind == FractionalDelayKind::Lagrange3 ? "Lagrange3" : "Thiran1";
}

double midiToHz(int midiNote) { return 440.0 * std::exp2((static_cast<double>(midiNote) - 69.0) / 12.0); }

template <typename SampleT>
void configure(WaveguideString<SampleT>& string, double sampleRate, FractionalDelayKind kind, double f0Hz,
               const StringMaterialParams& material = StringMaterialParams{}) {
    string.prepare(sampleRate, 512, kind);
    WaveguideStringParams params;
    params.f0Hz = static_cast<float>(f0Hz);
    params.material = material;
    string.setParams(params);
    string.setAnalyticTuningCompensation(0.0f);
    string.reset();
}

// One pluck rendered through the string, returning the tap channel.
template <typename SampleT>
std::vector<SampleT> pluckAndRender(WaveguideString<SampleT>& string, double sampleRate, std::size_t sampleCount,
                                    float noiseAmount = 0.0f) {
    PluckExciter<SampleT> exciter;
    exciter.prepare(sampleRate, 512);
    PluckExciterParams exciterParams;
    exciterParams.noiseAmount = noiseAmount;
    exciter.setParams(exciterParams);
    exciter.trigger(0.8f, 0.28f, 0.5f);

    std::vector<SampleT> out(sampleCount);
    for (std::size_t n = 0; n < sampleCount; ++n) {
        const SampleT excitation = exciter.renderSample();
        if (excitation != SampleT(0))
            string.injectAt(exciter.latchedPosition01(), excitation);
        out[n] = string.readTapAt(0.87f);
        string.tick();
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// lifecycle contract
// ---------------------------------------------------------------------------------------------

TEMPLATE_TEST_CASE("CONTRACT: WaveguideString prepare is re-entrant", "[contract]", float, double) {
    for (FractionalDelayKind kind : kKinds) {
        WaveguideString<TestType> reused;
        configure(reused, 44100.0, kind, 220.0);
        pluckAndRender(reused, 44100.0, 4096);

        // Re-prepare at a different rate/block size; must be indistinguishable from fresh.
        configure(reused, 96000.0, kind, 220.0);

        WaveguideString<TestType> fresh;
        configure(fresh, 96000.0, kind, 220.0);

        const auto a = pluckAndRender(reused, 96000.0, 8192);
        const auto b = pluckAndRender(fresh, 96000.0, 8192);
        REQUIRE(a.size() == b.size());
        for (std::size_t i = 0; i < a.size(); ++i)
            REQUIRE(a[i] == b[i]);
    }
}

TEMPLATE_TEST_CASE("CONTRACT: WaveguideString reset is idempotent and complete", "[contract]", float, double) {
    for (FractionalDelayKind kind : kKinds) {
        WaveguideString<TestType> excited;
        configure(excited, 48000.0, kind, 130.81);
        pluckAndRender(excited, 48000.0, 48000); // 1 s of ringing
        excited.reset();
        excited.reset(); // twice must equal once

        WaveguideString<TestType> fresh;
        configure(fresh, 48000.0, kind, 130.81);

        for (int i = 0; i < 2048; ++i) {
            REQUIRE(excited.readTapAt(0.87f) == fresh.readTapAt(0.87f));
            excited.tick();
            fresh.tick();
        }
        REQUIRE(excited.energyEstimate() == 0.0);
    }
}

TEMPLATE_TEST_CASE("CONTRACT: WaveguideString is silent before excitation", "[contract]", float, double) {
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kRates) {
            WaveguideString<TestType> string;
            configure(string, sampleRate, kind, 82.41);
            for (int i = 0; i < 4096; ++i) {
                REQUIRE(string.readTapAt(0.87f) == TestType(0));
                REQUIRE(string.railOutgoingAtBridge() == TestType(0));
                string.tick();
            }
            REQUIRE(string.energyEstimate() == 0.0);
        }
    }
}

TEST_CASE("CONTRACT: WaveguideString per-sample loop allocates nothing", "[contract]") {
    WaveguideString<float> string;
    configure(string, 48000.0, FractionalDelayKind::Lagrange3, 110.0);

    PluckExciter<float> exciter;
    exciter.prepare(48000.0, 512);
    exciter.trigger(0.8f, 0.28f, 0.5f);

    WaveguideStringParams bending;
    bending.f0Hz = 110.0f;

    cnpg::test::resetAllocationCount();
    for (int n = 0; n < 200000; ++n) {
        // Sweep the bend continuously so the per-sample retune path (which re-solves the loop
        // length every sample while f0 moves) is the one under the allocation guard.
        bending.bendSemitones = 2.0f * std::sin(static_cast<float>(n) * 1.0e-4f);
        string.setParams(bending);
        const float excitation = exciter.renderSample();
        if (excitation != 0.0f)
            string.injectAt(exciter.latchedPosition01(), excitation);
        (void)string.readTapAt(0.87f);
        float fromNut = 0.0f;
        float fromBridge = 0.0f;
        string.readJunctionInputs(0.5f, fromNut, fromBridge);
        string.writeJunctionOutputs(0.5f, fromNut, fromBridge);
        string.tick();
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

TEST_CASE("CONTRACT: WaveguideString stays finite under a continuous bend fuzz", "[contract]") {
    for (FractionalDelayKind kind : kKinds) {
        WaveguideString<float> string;
        configure(string, 48000.0, kind, 82.41);

        PluckExciter<float> exciter;
        exciter.prepare(48000.0, 512);

        std::mt19937 rng(20260731u);
        std::uniform_real_distribution<float> unit01(0.0f, 1.0f);

        WaveguideStringParams params;
        bool allFinite = true;
        for (int n = 0; n < 400000; ++n) {
            if (n % 20000 == 0)
                exciter.trigger(unit01(rng), unit01(rng), unit01(rng));
            params.f0Hz = 82.41f;
            params.bendSemitones = -2.0f + 4.0f * unit01(rng);
            params.material.lossGainLow = unit01(rng);
            params.material.lossGainHigh = unit01(rng);
            params.material.dispersionAmount = unit01(rng);
            string.setParams(params);

            const float excitation = exciter.renderSample();
            if (excitation != 0.0f)
                string.injectAt(exciter.latchedPosition01(), excitation);
            allFinite &= std::isfinite(string.readTapAt(0.87f));
            string.tick();
        }
        INFO("kind " << kindName(kind));
        REQUIRE(allFinite);
    }
}

// ---------------------------------------------------------------------------------------------
// tuning solve, seams, and the loss filter's passivity
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: WaveguideString realizes the requested loop period exactly", "[contract]") {
    // The analytic compensation's whole job: the realized round-trip PHASE delay at f0 must equal
    // fs / f0. Anything the solver leaves on the table shows up in the [tuning] gate as cents.
    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kRates) {
            for (int midiNote = cnpg::dsp::kMinMidiNote; midiNote <= cnpg::dsp::kMaxMidiNote; ++midiNote) {
                for (float dispersion : {0.0f, 0.5f, 1.0f}) {
                    StringMaterialParams material;
                    material.dispersionAmount = dispersion;
                    WaveguideString<double> string;
                    configure(string, sampleRate, kind, midiToHz(midiNote), material);

                    const double expected = sampleRate / string.currentF0Hz();
                    const double realized = string.realizedLoopDelaySamples();
                    const double centsError = 1200.0 * std::log2(expected / realized);
                    INFO(kindName(kind) << " rate " << sampleRate << " MIDI " << midiNote << " dispersion "
                                        << dispersion << " expected " << expected << " realized " << realized);
                    REQUIRE(std::fabs(centsError) < 1e-6);
                }
            }
        }
    }
}

TEMPLATE_TEST_CASE("CONTRACT: WaveguideString junction seam is transparent when the junction is", "[contract]", float,
                   double) {
    for (FractionalDelayKind kind : kKinds) {
        WaveguideString<TestType> plain;
        WaveguideString<TestType> seamed;
        configure(plain, 48000.0, kind, 146.83);
        configure(seamed, 48000.0, kind, 146.83);

        PluckExciter<TestType> exciterA;
        PluckExciter<TestType> exciterB;
        exciterA.prepare(48000.0, 512);
        exciterB.prepare(48000.0, 512);
        exciterA.trigger(0.9f, 0.31f, 0.6f);
        exciterB.trigger(0.9f, 0.31f, 0.6f);

        for (int n = 0; n < 20000; ++n) {
            const TestType ea = exciterA.renderSample();
            if (ea != TestType(0))
                plain.injectAt(exciterA.latchedPosition01(), ea);
            const TestType eb = exciterB.renderSample();
            if (eb != TestType(0))
                seamed.injectAt(exciterB.latchedPosition01(), eb);

            // A pass-through junction at p = 0.4: outputs equal inputs, so nothing may change.
            TestType fromNut = TestType(0);
            TestType fromBridge = TestType(0);
            seamed.readJunctionInputs(0.4f, fromNut, fromBridge);
            seamed.writeJunctionOutputs(0.4f, fromNut, fromBridge);

            REQUIRE(plain.readTapAt(0.87f) == seamed.readTapAt(0.87f));
            plain.tick();
            seamed.tick();
        }
    }
}

TEST_CASE("CONTRACT: WaveguideString bridge port overrides the internal termination", "[contract]") {
    // The P2 seam: railOutgoingAtBridge() publishes the loop chain's output and
    // railAcceptFromBridge() replaces the internal rigid -1 reflection for the next tick. Driving
    // the port with 0 is a perfectly absorbing bridge (the string must bleed out); driving it
    // with -outgoing is the rigid bridge again (the string must keep ringing).
    auto runPortDriven = [](bool absorbing) {
        WaveguideString<double> string;
        configure(string, 48000.0, FractionalDelayKind::Lagrange3, 196.0);
        string.setLossBypassed(true);

        PluckExciter<double> exciter;
        exciter.prepare(48000.0, 512);
        exciter.trigger(0.7f, 0.28f, 0.5f);

        // Peak, not a fixed sample index: an absorbing bridge drains a 196 Hz string in about one
        // loop period (245 samples at 48 kHz), so any "after the attack" index is already empty.
        double peakEnergy = 0.0;
        for (int n = 0; n < 48000; ++n) {
            const double excitation = exciter.renderSample();
            if (excitation != 0.0)
                string.injectAt(exciter.latchedPosition01(), excitation);
            string.railAcceptFromBridge(absorbing ? 0.0 : -string.railOutgoingAtBridge());
            string.tick();
            if ((n % 64) == 0)
                peakEnergy = std::max(peakEnergy, string.energyEstimate());
        }
        return std::pair<double, double>{peakEnergy, string.energyEstimate()};
    };

    const auto absorbed = runPortDriven(true);
    const auto reflected = runPortDriven(false);

    REQUIRE(absorbed.first > 0.0);
    REQUIRE(absorbed.second < 1e-6 * absorbed.first);  // absorbing bridge drains the string
    REQUIRE(reflected.second > 0.1 * reflected.first); // rigid bridge keeps it ringing

    WaveguideString<double> string;
    configure(string, 48000.0, FractionalDelayKind::Lagrange3, 196.0);
    REQUIRE(string.portImpedance() == 1.0f);
}

TEST_CASE("CONTRACT: WaveguideString loop loss filter is passive over the whole knob range", "[contract]") {
    // |H(w)|^2 is a Mobius function of cos(w), so its extrema over w sit at DC and Nyquist; the
    // design therefore guarantees |H| <= max(gLow, gHigh) <= 1 everywhere. Checked directly here
    // because tier-3 [energy] (P2.4) depends on it and a sign slip in the b0/b1 solve would not
    // show up in the lossless tier-2 case.
    constexpr double kPoleZ = static_cast<double>(cnpg::dsp::kLossFilterPoleZ);
    for (int lowStep = 0; lowStep <= 10; ++lowStep) {
        for (int highStep = 0; highStep <= 10; ++highStep) {
            const double gLow =
                static_cast<double>(cnpg::dsp::kLossGainLowMin) +
                0.1 * lowStep * static_cast<double>(cnpg::dsp::kLossGainLowMax - cnpg::dsp::kLossGainLowMin);
            const double gHigh =
                static_cast<double>(cnpg::dsp::kLossGainHighMin) +
                0.1 * highStep * static_cast<double>(cnpg::dsp::kLossGainHighMax - cnpg::dsp::kLossGainHighMin);
            const double b0 = 0.5 * (gLow * (1.0 - kPoleZ) + gHigh * (1.0 + kPoleZ));
            const double b1 = 0.5 * (gLow * (1.0 - kPoleZ) - gHigh * (1.0 + kPoleZ));

            const double bound = std::max(gLow, gHigh);
            for (int i = 0; i <= 512; ++i) {
                const double w = 3.14159265358979323846 * static_cast<double>(i) / 512.0;
                const std::complex<double> z = std::polar(1.0, -w);
                const std::complex<double> h = (b0 + b1 * z) / (1.0 - kPoleZ * z);
                REQUIRE(std::abs(h) <= bound + 1e-12);
            }
        }
    }
}

TEST_CASE("CONTRACT: WaveguideString loadCalibrationTable stores without becoming the active source", "[contract]") {
    // docs/plan.md section 2.4 / the P1.4 brief: the table is stored and is a no-op source until
    // Task P2.7 selects it. Loading an absurd table must not move a single sample in P1.
    WaveguideString<double> plain;
    WaveguideString<double> tabled;
    configure(plain, 48000.0, FractionalDelayKind::Lagrange3, 220.0);
    configure(tabled, 48000.0, FractionalDelayKind::Lagrange3, 220.0);

    std::vector<float> cents(88, 250.0f);
    tabled.loadCalibrationTable(cents.data(), cnpg::dsp::kMinMidiNote, static_cast<int>(cents.size()));

    const auto a = pluckAndRender(plain, 48000.0, 16384);
    const auto b = pluckAndRender(tabled, 48000.0, 16384);
    for (std::size_t i = 0; i < a.size(); ++i)
        REQUIRE(a[i] == b[i]);

    tabled.loadCalibrationTable(nullptr, 0, 0); // clearing is legal and equally inert
    SUCCEED();
}

// ---------------------------------------------------------------------------------------------
// energy: early tier-2 check on the isolated string (P1.4 acceptance criterion 4)
// ---------------------------------------------------------------------------------------------

TEST_CASE("ENERGY/T2: lossless isolated string never gains storage-function energy", "[energy]") {
    // P1.4 acceptance criterion: setLossBypassed(true), pluck on the DOUBLE instantiation, and
    // energyEstimate() -- impedance-weighted rail energy plus the closed-form quadratic storage
    // of the dispersion-allpass, loss-filter and fractional-delay interpolator states -- must not
    // increase per block over 10 s, within 1e-9 relative tolerance.
    constexpr double kSeconds = 10.0;
    constexpr int kBlockSize = 128;
    constexpr double kTolerance = 1e-9;

    for (FractionalDelayKind kind : kKinds) {
        for (double sampleRate : kRates) {
            for (float dispersion : {0.0f, 1.0f}) {
                StringMaterialParams material;
                material.dispersionAmount = dispersion;

                WaveguideString<double> string;
                configure(string, sampleRate, kind, midiToHz(45), material);
                string.setLossBypassed(true);

                PluckExciter<double> exciter;
                exciter.prepare(sampleRate, kBlockSize);
                exciter.trigger(0.8f, 0.28f, 0.5f);

                const auto totalBlocks = static_cast<int>(kSeconds * sampleRate / kBlockSize);
                double previous = -1.0;
                double first = -1.0;
                double worstGrowth = 0.0;
                for (int block = 0; block < totalBlocks; ++block) {
                    for (int n = 0; n < kBlockSize; ++n) {
                        const double excitation = exciter.renderSample();
                        if (excitation != 0.0)
                            string.injectAt(exciter.latchedPosition01(), excitation);
                        string.tick();
                    }
                    if (exciter.isActive())
                        continue; // still injecting energy this block

                    const double energy = string.energyEstimate();
                    REQUIRE(std::isfinite(energy));
                    // Non-triviality guard: a silent string would satisfy "never grows" for free.
                    REQUIRE(energy > 0.0);
                    if (first < 0.0)
                        first = energy;
                    if (previous >= 0.0) {
                        const double growth = (energy - previous) / std::max(previous, 1e-300);
                        worstGrowth = std::max(worstGrowth, growth);
                        INFO(kindName(kind)
                             << " rate " << sampleRate << " dispersion " << dispersion << " block " << block
                             << " energy " << energy << " previous " << previous << " growth " << growth);
                        REQUIRE(growth <= kTolerance);
                    }
                    previous = energy;
                }
                std::cout << "[energy] " << kindName(kind) << " @ " << sampleRate << " Hz, dispersion " << dispersion
                          << ": worst per-block relative growth " << worstGrowth << " (limit " << kTolerance
                          << "), energy " << first << " -> " << previous << "\n";
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// hidden: the P1 fractional-delay spike's measurements, for docs/decisions/0002-fractional-delay.md
// ---------------------------------------------------------------------------------------------

TEST_CASE("SPIKE: fractional-delay comparison (report generator)", "[.][report]") {
    constexpr double kSampleRate = 48000.0;

    std::cout << "kind,staticNsPerSample,bendingNsPerSample,bendEnergyGrowth,bendTransientRatio\n";
    for (FractionalDelayKind kind : kKinds) {
        // ---- cost per sample -----------------------------------------------------------------
        auto timeTicks = [&](bool bending) {
            WaveguideString<float> string;
            configure(string, kSampleRate, kind, 110.0);
            WaveguideStringParams params;
            params.f0Hz = 110.0f;
            string.injectAt(0.28f, 1.0f);

            constexpr int kTicks = 2000000;
            const auto start = std::chrono::steady_clock::now();
            for (int n = 0; n < kTicks; ++n) {
                if (bending) {
                    params.bendSemitones = 2.0f * std::sin(static_cast<float>(n) * 3.0e-5f);
                    string.setParams(params);
                }
                (void)string.readTapAt(0.87f);
                string.tick();
            }
            const auto elapsed = std::chrono::steady_clock::now() - start;
            return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()) /
                   static_cast<double>(kTicks);
        };
        const double staticNs = timeTicks(false);
        const double bendingNs = timeTicks(true);

        // ---- decay-tail smoothness under continuous retune -------------------------------------
        // Two views of the same thing. (1) storage-function energy: a stateful interpolator whose
        // coefficients jump when the integer part of the rail read steps injects or destroys
        // energy, and the lossless double instantiation makes that visible directly. (2) the tap
        // signal's peak second difference relative to its RMS, which is what a step in the read
        // position sounds like; reported as a ratio against the same measurement on a STATIC
        // note, so a value of 1 means the bend added no impulsive content at all.
        auto measureBend = [&](bool bend) {
            WaveguideString<double> string;
            configure(string, kSampleRate, kind, 110.0);
            string.setLossBypassed(true);

            PluckExciter<double> exciter;
            exciter.prepare(kSampleRate, 128);
            exciter.trigger(0.8f, 0.28f, 0.5f);

            WaveguideStringParams params;
            params.f0Hz = 110.0f;

            const int total = static_cast<int>(6.0 * kSampleRate);
            const int settle = static_cast<int>(0.5 * kSampleRate);
            std::vector<double> tap;
            tap.reserve(static_cast<std::size_t>(total));

            double worstGrowth = 0.0;
            double previousEnergy = -1.0;
            for (int n = 0; n < total; ++n) {
                if (bend) {
                    // A full +/-2-semitone glide, slow enough to be a musical bend: the integer
                    // part of each rail read crosses dozens of samples on the way.
                    params.bendSemitones = 2.0f * std::sin(static_cast<float>(n) / static_cast<float>(kSampleRate));
                    string.setParams(params);
                }
                const double excitation = exciter.renderSample();
                if (excitation != 0.0)
                    string.injectAt(exciter.latchedPosition01(), excitation);
                tap.push_back(string.readTapAt(0.87f));
                string.tick();

                if (n > settle && (n % 128) == 0) {
                    const double energy = string.energyEstimate();
                    if (previousEnergy > 0.0)
                        worstGrowth = std::max(worstGrowth, (energy - previousEnergy) / previousEnergy);
                    previousEnergy = energy;
                }
            }

            double sumSquares = 0.0;
            double peakSecondDiff = 0.0;
            for (std::size_t i = static_cast<std::size_t>(settle) + 2; i < tap.size(); ++i) {
                sumSquares += tap[i] * tap[i];
                peakSecondDiff = std::max(peakSecondDiff, std::fabs(tap[i] - 2.0 * tap[i - 1] + tap[i - 2]));
            }
            const double rms =
                std::sqrt(sumSquares / static_cast<double>(tap.size() - static_cast<std::size_t>(settle) - 2));
            return std::pair<double, double>{worstGrowth, peakSecondDiff / std::max(rms, 1e-300)};
        };

        const auto bent = measureBend(true);
        const auto still = measureBend(false);

        char line[192];
        std::snprintf(line, sizeof(line), "%s,%.2f,%.2f,%.3e,%.3f", kindName(kind), staticNs, bendingNs, bent.first,
                      bent.second / std::max(still.second, 1e-300));
        std::cout << line << "\n";
    }
    SUCCEED();
}

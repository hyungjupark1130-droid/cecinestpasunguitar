#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/IBridgePort.h"

#include "support/AllocationGuard.h"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>

// BridgeEnergyTierOneTests -- Task P2.4's tier-1 [energy] gate: is the BRIDGE'S SCATTERING ALGEBRA
// passive, pointwise in parameter space, with no filters and no delay lines involved?
//
// docs/plan.md section 4.2 states the tier separation and the canonical grid; this file references
// both and restates neither as new numbers. A failure here is a math error in the junction design,
// which is precisely what makes it worth separating from tier 2 (the assembled network) and tier 3
// (the intentional losses).
//
// TWO MEASUREMENTS, and the second is the one with teeth:
//
//   1. ||S||_2 <= 1 + 1e-12 over the grid, from copyScatteringMatrix(), via a one-sided Jacobi SVD
//      in double. This is what the plan names. On its own it is WEAK, and saying so is better than
//      pretending otherwise: the string block of the scattering matrix is
//      2 sqrt(Z_i Z_j)/sigma_total - delta_ij, whose largest singular value is
//      max(|2 sigma/sigma_total - 1|, 1) = 1 for ANY positive sigma_total, so it stays passive even
//      if the element impedances are computed completely wrongly -- a wrong sigma_total is still
//      some sigma_total.
//   2. THE SAMPLE-LEVEL ENERGY BALANCE. scatter() is driven with random incident vectors and the
//      full account is checked every sample: energy handed in must be >= energy handed back plus
//      the change in the junction's OWN stored energy. That covers the element ports, the mass and
//      spring recurrences, the storage functional itself, and the coefficient smoother -- none of
//      which the matrix norm can see. With the loss bypassed it must balance to the arithmetic
//      floor (the junction is then exactly lossless); with the dashpot in it must only ever lose.

using cnpg::dsp::BridgeAdmittanceParams;
using cnpg::dsp::BridgeJunction;

namespace {

constexpr int kBlock = 128;

// docs/plan.md section 4.2, tier 1: "for every port count 1..8 with representative impedance sets
// (equal impedances, and a 4:1 spread)", "resonanceHz in {80, 400, 2000, 8000} x damping in
// {0 (exercises the positive-real clamp), 0.1, 1, 10} x couplingStrength in {0, 0.5, 1}".
// Referenced from this one place so the grid is stated once.
constexpr std::array<float, 4> kGridResonanceHz{80.0f, 400.0f, 2000.0f, 8000.0f};
constexpr std::array<float, 4> kGridDamping{0.0f, 0.1f, 1.0f, 10.0f};
constexpr std::array<float, 3> kGridCoupling{0.0f, 0.5f, 1.0f};
constexpr std::array<double, 3> kGridRates{44100.0, 48000.0, 96000.0};

// The two impedance sets the plan names. The 4:1 spread is the one that matters: with equal
// impedances the power-normalized matrix and the raw-port matrix coincide, so an implementation
// that forgot to normalize at all would pass the equal-impedance half of the grid unchanged.
std::vector<float> impedanceSet(int numPorts, bool spread) {
    std::vector<float> z(static_cast<std::size_t>(numPorts), 1.0f);
    if (!spread || numPorts < 2)
        return z;
    for (int p = 0; p < numPorts; ++p) {
        const double t = static_cast<double>(p) / static_cast<double>(numPorts - 1);
        z[static_cast<std::size_t>(p)] = static_cast<float>(1.0 + 3.0 * t); // 1.0 .. 4.0
    }
    return z;
}

// Largest singular value of an n x n matrix by ONE-SIDED JACOBI, in double. Deliberately a general
// SVD rather than "S is symmetric, take the largest |eigenvalue|": the derivation in
// BridgeJunction.h licenses the shortcut, and the test's whole job is to measure the matrix it is
// handed rather than to re-assume the structure it is supposed to be checking. (Same reasoning as
// the closed-form 2x2 SVD in tests/dsp/DamperEnergyTests.cpp, generalized.)
double spectralNorm(const double* rowMajor, int n, int stride) {
    std::vector<double> a(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            a[static_cast<std::size_t>(i) * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)] =
                rowMajor[static_cast<std::size_t>(i) * static_cast<std::size_t>(stride) + static_cast<std::size_t>(j)];

    auto col = [&](int j, int i) -> double& {
        return a[static_cast<std::size_t>(i) * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)];
    };

    for (int sweep = 0; sweep < 60; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double alpha = 0.0;
                double beta = 0.0;
                double gamma = 0.0;
                for (int i = 0; i < n; ++i) {
                    alpha += col(p, i) * col(p, i);
                    beta += col(q, i) * col(q, i);
                    gamma += col(p, i) * col(q, i);
                }
                off = std::max(off, std::fabs(gamma) / std::sqrt(std::max(alpha * beta, 1e-300)));
                if (std::fabs(gamma) <= 1e-16 * std::sqrt(alpha * beta))
                    continue;
                const double zeta = (beta - alpha) / (2.0 * gamma);
                const double t = (zeta >= 0.0 ? 1.0 : -1.0) / (std::fabs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = c * t;
                for (int i = 0; i < n; ++i) {
                    const double ap = col(p, i);
                    const double aq = col(q, i);
                    col(p, i) = c * ap - s * aq;
                    col(q, i) = s * ap + c * aq;
                }
            }
        }
        if (off <= 1e-15)
            break;
    }

    double largest = 0.0;
    for (int j = 0; j < n; ++j) {
        double norm = 0.0;
        for (int i = 0; i < n; ++i)
            norm += col(j, i) * col(j, i);
        largest = std::max(largest, std::sqrt(norm));
    }
    return largest;
}

// xorshift32, so the random probe below is deterministic across platforms and runs.
struct Rng {
    unsigned state = 2463534242u;
    double next() noexcept {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return 2.0 * (static_cast<double>(state) / 4294967296.0) - 1.0;
    }
};

template <typename SampleT>
BridgeJunction<SampleT> makeJunction(double sampleRate, const std::vector<float>& impedances,
                                     const BridgeAdmittanceParams& admittance, bool lossBypassed) {
    BridgeJunction<SampleT> junction;
    junction.setLossBypassed(lossBypassed);
    junction.prepare(sampleRate, kBlock, static_cast<int>(impedances.size()), impedances.data());
    junction.setAdmittance(admittance);
    junction.reset();
    return junction;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// tier 1(a): the scattering matrix is passive over the canonical grid
// ---------------------------------------------------------------------------------------------

TEMPLATE_TEST_CASE("ENERGY/T1: BridgeJunction scattering matrix is passive", "[energy]", float, double) {
    constexpr double kNormLimit = 1.0 + 1.0e-12;

    double worstNorm = 0.0;
    int worstPorts = 0;
    float worstAt[3] = {0.0f, 0.0f, 0.0f};
    int gridPoints = 0;
    double worstRigidDeviation = 0.0;

    for (double sampleRate : kGridRates) {
        for (int ports = 1; ports <= cnpg::dsp::kMaxStrings; ++ports) {
            for (bool spread : {false, true}) {
                const std::vector<float> impedances = impedanceSet(ports, spread);
                for (float resonanceHz : kGridResonanceHz) {
                    // 8 kHz is above the Nyquist margin at no rate in the grid, but the clamp
                    // ceiling scales with the rate, so a point can be legal at 96 kHz and clamped
                    // at 44.1 -- which is exactly the behaviour the illegal-input case below
                    // gates, and here it just means the grid is honest about what it swept.
                    for (float damping : kGridDamping) {
                        for (float coupling : kGridCoupling) {
                            BridgeAdmittanceParams admittance;
                            admittance.resonanceHz = resonanceHz;
                            admittance.damping = damping;
                            admittance.couplingStrength = coupling;

                            BridgeJunction<TestType> junction =
                                makeJunction<TestType>(sampleRate, impedances, admittance, true);

                            INFO("rate " << sampleRate << " ports " << ports << (spread ? " (4:1 spread)" : " (equal)")
                                         << " resonance " << resonanceHz << " damping " << damping << " coupling "
                                         << coupling);

                            // The grid point really IS the grid point, after validation. Without
                            // this the sweep could be measuring one clamped state 2304 times and
                            // nothing would say so.
                            REQUIRE(junction.currentCouplingStrength() == std::clamp(coupling, 0.0f, 1.0f));
                            REQUIRE(junction.currentDamping() >= cnpg::dsp::kBridgeMinDamping);
                            REQUIRE(junction.currentDamping() <= cnpg::dsp::kBridgeMaxDamping);
                            REQUIRE(junction.currentResonanceHz() >= cnpg::dsp::kBridgeMinResonanceHz);
                            REQUIRE(static_cast<double>(junction.currentResonanceHz()) <=
                                    static_cast<double>(cnpg::dsp::kBridgeResonanceNyquistFraction) * sampleRate);
                            if (damping >= cnpg::dsp::kBridgeMinDamping)
                                REQUIRE(junction.currentDamping() == damping);

                            std::array<double, cnpg::dsp::kMaxStrings * cnpg::dsp::kMaxStrings> matrix{};
                            junction.copyScatteringMatrix(matrix.data(), cnpg::dsp::kMaxStrings);
                            for (int i = 0; i < ports; ++i)
                                for (int j = 0; j < ports; ++j)
                                    REQUIRE(std::isfinite(
                                        matrix[static_cast<std::size_t>(i * cnpg::dsp::kMaxStrings + j)]));

                            const double norm = spectralNorm(matrix.data(), ports, cnpg::dsp::kMaxStrings);
                            REQUIRE(std::isfinite(norm));
                            REQUIRE(norm <= kNormLimit);
                            ++gridPoints;
                            if (norm > worstNorm) {
                                worstNorm = norm;
                                worstPorts = ports;
                                worstAt[0] = resonanceHz;
                                worstAt[1] = damping;
                                worstAt[2] = coupling;
                            }

                            // NON-VACUITY, both ends of the couplingStrength axis:
                            //   coupling 0 must be EXACTLY the rigid termination (S = -I), which is
                            //     the contract docs/plan.md section 2.6 gives it and what every
                            //     later body feature depends on not being the default; and
                            //   coupling > 0 must NOT be -- a junction that never couples anything
                            //     is trivially passive and would pass every line above.
                            double offDiagonal = 0.0;
                            for (int i = 0; i < ports; ++i)
                                for (int j = 0; j < ports; ++j)
                                    if (i != j)
                                        offDiagonal = std::max(
                                            offDiagonal,
                                            std::fabs(
                                                matrix[static_cast<std::size_t>(i * cnpg::dsp::kMaxStrings + j)]));
                            if (coupling == 0.0f) {
                                for (int i = 0; i < ports; ++i)
                                    for (int j = 0; j < ports; ++j) {
                                        const double expected = (i == j) ? -1.0 : 0.0;
                                        const double got =
                                            matrix[static_cast<std::size_t>(i * cnpg::dsp::kMaxStrings + j)];
                                        worstRigidDeviation = std::max(worstRigidDeviation, std::fabs(got - expected));
                                        REQUIRE(got == expected);
                                    }
                                REQUIRE(junction.instantaneousMobility() == 0.0);
                            } else if (ports >= 2) {
                                REQUIRE(offDiagonal > 0.0);
                                REQUIRE(junction.instantaneousMobility() > 0.0);
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "[energy] T1 BridgeJunction |S|_2: worst " << worstNorm << " over " << gridPoints
              << " section-4.2 grid points x 3 rates (limit " << kNormLimit << "), at " << worstPorts
              << " ports, resonance " << worstAt[0] << " Hz, damping " << worstAt[1] << ", coupling " << worstAt[2]
              << "; worst deviation from S = -I at coupling 0: " << worstRigidDeviation << "\n";

    // The junction is lossless-or-dissipative everywhere and ACTIVE nowhere: the worst norm over
    // the whole grid is the unit gain of the mode the load cannot touch. (S~ is an orthogonal
    // reflection -- eigenvalues +1 on span(sqrt(Z)) and -1 elsewhere -- so 1 is not slack in the
    // bound, it is the answer.) Asserted as "1 to within the Jacobi SVD's own arithmetic" rather
    // than as an exact ==: the 2x2 case in tests/dsp/DamperEnergyTests.cpp can use a closed form
    // and lands exactly on 1, an 8x8 iterative SVD lands a few ulps away and that is a property of
    // the MEASUREMENT, not of the matrix. The upper half of the window is the plan's own bound.
    REQUIRE(worstNorm >= 1.0 - 1.0e-12);
    REQUIRE(worstNorm <= kNormLimit);
}

TEST_CASE("ENERGY/T1: BridgeJunction clamps illegal admittance inputs into a passive load", "[energy]") {
    // Task P2.4 acceptance: "deliberately illegal setAdmittance inputs (negative damping, resonance
    // >= Nyquist) are clamped and still yield a passive S". Each row states what the clamp must
    // produce, so the case asserts the VALIDATION and not merely that nothing exploded.
    constexpr double kRate = 48000.0;
    const std::vector<float> impedances = impedanceSet(6, true);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    struct Case {
        const char* what;
        BridgeAdmittanceParams in;
    };
    const Case cases[] = {
        {"negative damping", {180.0f, -1.0f, 0.5f}},          {"zero damping", {180.0f, 0.0f, 0.5f}},
        {"damping past the ceiling", {180.0f, 1.0e9f, 0.5f}}, {"resonance at Nyquist", {24000.0f, 0.5f, 0.5f}},
        {"resonance past Nyquist", {1.0e6f, 0.5f, 1.0f}},     {"resonance at DC", {0.0f, 0.5f, 1.0f}},
        {"negative resonance", {-400.0f, 0.5f, 1.0f}},        {"coupling past 1", {180.0f, 0.5f, 17.0f}},
        {"negative coupling", {180.0f, 0.5f, -3.0f}},         {"NaN everywhere", {nan, nan, nan}},
        {"infinities everywhere", {inf, inf, inf}},
    };

    for (const Case& testCase : cases) {
        BridgeJunction<float> junction = makeJunction<float>(kRate, impedances, testCase.in, true);
        INFO(testCase.what);

        // The clamps, asserted directly. damping >= a STRICTLY positive floor is the positive-real
        // constraint itself; resonance under the Nyquist margin is what keeps the prewarped analog
        // prototype finite.
        REQUIRE(junction.currentDamping() >= cnpg::dsp::kBridgeMinDamping);
        REQUIRE(junction.currentDamping() <= cnpg::dsp::kBridgeMaxDamping);
        REQUIRE(junction.currentResonanceHz() >= cnpg::dsp::kBridgeMinResonanceHz);
        REQUIRE(static_cast<double>(junction.currentResonanceHz()) <=
                static_cast<double>(cnpg::dsp::kBridgeResonanceNyquistFraction) * kRate);
        REQUIRE(junction.currentCouplingStrength() >= 0.0f);
        REQUIRE(junction.currentCouplingStrength() <= 1.0f);
        REQUIRE(std::isfinite(junction.instantaneousMobility()));

        std::array<double, cnpg::dsp::kMaxStrings * cnpg::dsp::kMaxStrings> matrix{};
        junction.copyScatteringMatrix(matrix.data(), cnpg::dsp::kMaxStrings);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                REQUIRE(std::isfinite(matrix[static_cast<std::size_t>(i * cnpg::dsp::kMaxStrings + j)]));
        REQUIRE(spectralNorm(matrix.data(), 6, cnpg::dsp::kMaxStrings) <= 1.0 + 1.0e-12);

        // ...and the audio path survives them too, which the matrix alone does not show.
        std::array<float, 6> incident{0.3f, -0.7f, 0.1f, 0.9f, -0.2f, 0.05f};
        std::array<float, 6> outgoing{};
        for (int n = 0; n < 4096; ++n) {
            junction.scatter(incident.data(), outgoing.data(), 6);
            for (float value : outgoing)
                REQUIRE(std::isfinite(value));
            REQUIRE(std::isfinite(junction.bridgeOutput()));
        }
        REQUIRE(std::isfinite(junction.storageEnergy()));
    }
}

// ---------------------------------------------------------------------------------------------
// tier 1(b): the sample-level energy balance -- the measurement the matrix norm cannot make
// ---------------------------------------------------------------------------------------------

TEST_CASE("ENERGY/T1: BridgeJunction never returns more energy than it is given", "[energy]") {
    // WHAT THE MATRIX NORM CANNOT SEE, and why this case exists. The string block of the
    // scattering matrix is 2 sqrt(Z_i Z_j)/sigma_total - delta_ij; its largest singular value is
    // max(|2 sigma / sigma_total - 1|, 1), which is 1 for ANY positive sigma_total. So the tier-1
    // norm above stays at exactly 1 even if the mass and spring impedances are computed wrongly,
    // even if the element recurrences have the wrong sign, and even if storageEnergy() reports a
    // number unrelated to what is stored. This case closes all three by measuring the whole
    // account, every sample, in the junction's own units (a wave x on a port of impedance Z carries
    // Z x^2, which is what the adaptor conserves):
    //
    //     sum_i Z_i a_i^2   >=   sum_i Z_i b_i^2   +   (E_bridge[n] - E_bridge[n-1])
    //
    // with equality to the arithmetic floor when the dashpot is bypassed.
    constexpr double kBalanceTolerance = 1.0e-11; // relative to the energy handed in

    double worstLosslessImbalance = 0.0;
    double worstLossyGain = 0.0;
    double worstDissipation = 0.0;
    const char* worstAt = "";

    for (double sampleRate : kGridRates) {
        for (int ports : {1, 2, 6, 8}) {
            for (bool spread : {false, true}) {
                const std::vector<float> impedances = impedanceSet(ports, spread);
                for (float resonanceHz : kGridResonanceHz) {
                    for (float damping : kGridDamping) {
                        for (float coupling : kGridCoupling) {
                            if (coupling == 0.0f)
                                continue; // the rigid limit is exactly S = -I and is gated above
                            for (bool bypassLoss : {true, false}) {
                                BridgeAdmittanceParams admittance;
                                admittance.resonanceHz = resonanceHz;
                                admittance.damping = damping;
                                admittance.couplingStrength = coupling;

                                BridgeJunction<double> junction =
                                    makeJunction<double>(sampleRate, impedances, admittance, bypassLoss);
                                const double zRefSquared =
                                    2.0 * junction.referenceImpedance() * junction.referenceImpedance();

                                Rng rng;
                                std::array<double, cnpg::dsp::kMaxStrings> incident{};
                                std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
                                double stored = junction.storageEnergy() * zRefSquared;
                                REQUIRE(stored == 0.0); // reset really did clear the states

                                // The per-sample REQUIREs live OUTSIDE the sample loop: the sweep
                                // is ~1500 configurations of 6000 samples, and a Catch2 assertion
                                // per sample would dominate the suite's runtime while measuring
                                // exactly the same worst case.
                                double totalIn = 0.0;
                                double totalOut = 0.0;
                                double configGain = 0.0;
                                double configImbalance = 0.0;
                                double configDissipation = 0.0;
                                bool allFinite = true;
                                for (int n = 0; n < 6000; ++n) {
                                    double handedIn = 0.0;
                                    for (int p = 0; p < ports; ++p) {
                                        // Excite for a while, then let it ring, so the balance is
                                        // checked both while driven and while decaying.
                                        incident[static_cast<std::size_t>(p)] = (n < 2000) ? rng.next() : 0.0;
                                        handedIn += static_cast<double>(impedances[static_cast<std::size_t>(p)]) *
                                                    incident[static_cast<std::size_t>(p)] *
                                                    incident[static_cast<std::size_t>(p)];
                                    }
                                    junction.scatter(incident.data(), outgoing.data(), ports);
                                    double handedBack = 0.0;
                                    for (int p = 0; p < ports; ++p) {
                                        allFinite &= std::isfinite(outgoing[static_cast<std::size_t>(p)]);
                                        handedBack += static_cast<double>(impedances[static_cast<std::size_t>(p)]) *
                                                      outgoing[static_cast<std::size_t>(p)] *
                                                      outgoing[static_cast<std::size_t>(p)];
                                    }
                                    const double now = junction.storageEnergy() * zRefSquared;
                                    const double residual = handedIn - handedBack - (now - stored);
                                    const double scale = std::max({handedIn, handedBack, now, stored, 1.0e-30});

                                    configGain = std::max(configGain, -residual / scale);
                                    configImbalance = std::max(configImbalance, std::fabs(residual) / scale);
                                    configDissipation = std::max(configDissipation, residual / scale);
                                    stored = now;
                                    totalIn += handedIn;
                                    totalOut += handedBack;
                                }

                                INFO("rate " << sampleRate << " ports " << ports << (spread ? " spread" : " equal")
                                             << " resonance " << resonanceHz << " damping " << damping << " coupling "
                                             << coupling << (bypassLoss ? " lossless" : " lossy") << ": worst gain "
                                             << configGain << " worst |imbalance| " << configImbalance);
                                REQUIRE(allFinite);
                                // THE passivity statement: the junction never creates energy.
                                REQUIRE(configGain <= kBalanceTolerance);
                                if (bypassLoss) {
                                    // ...and with the dashpot removed it never destroys any either,
                                    // which is what makes it a legal element of a tier-2 LOSSLESS
                                    // network rather than merely a safe one.
                                    REQUIRE(configImbalance <= kBalanceTolerance);
                                    worstLosslessImbalance = std::max(worstLosslessImbalance, configImbalance);
                                } else {
                                    worstDissipation = std::max(worstDissipation, configDissipation);
                                }
                                if (configGain > worstLossyGain) {
                                    worstLossyGain = configGain;
                                    worstAt = bypassLoss ? "lossless" : "lossy";
                                }
                                // Non-vacuous: the probe really pushed energy through the junction.
                                REQUIRE(totalIn > 0.0);
                                REQUIRE(totalOut > 0.0);
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "[energy] T1 BridgeJunction sample-level balance: worst |imbalance| with the dashpot bypassed "
              << worstLosslessImbalance << " (limit " << kBalanceTolerance
              << ", relative); worst ENERGY GAIN over the whole sweep " << worstLossyGain << " (" << worstAt
              << "); worst single-sample dissipation with the dashpot in " << worstDissipation << "\n";

    // Non-vacuity on the lossy side: a dashpot that never dissipated would pass every inequality
    // above, and would also be a bridge that cannot damp a string.
    REQUIRE(worstDissipation > 0.0);
}

TEST_CASE("CONTRACT: BridgeJunction scatter allocates nothing", "[contract]") {
    const std::vector<float> impedances = impedanceSet(8, true);
    BridgeAdmittanceParams admittance;
    admittance.couplingStrength = 0.5f;
    BridgeJunction<float> junction = makeJunction<float>(48000.0, impedances, admittance, false);

    cnpg::test::resetAllocationCount();
    std::array<float, cnpg::dsp::kMaxStrings> incident{};
    std::array<float, cnpg::dsp::kMaxStrings> outgoing{};
    for (int n = 0; n < 100000; ++n) {
        if ((n % 4096) == 0) {
            admittance.resonanceHz = (n % 8192 == 0) ? 120.0f : 900.0f;
            admittance.couplingStrength = (n % 8192 == 0) ? 0.2f : 0.8f;
            junction.setAdmittance(admittance); // retargets the smoothers from the audio thread
        }
        for (int p = 0; p < cnpg::dsp::kMaxStrings; ++p)
            incident[static_cast<std::size_t>(p)] = static_cast<float>(std::sin(0.01 * n + 0.3 * p));
        junction.scatter(incident.data(), outgoing.data(), cnpg::dsp::kMaxStrings);
    }
    REQUIRE(cnpg::test::allocationCount() == 0);
}

TEST_CASE("ENERGY/T1: BridgeJunction stays passive while its admittance is being changed", "[energy]") {
    // The corner a pointwise grid cannot reach. setAdmittance() is automatable (the plugin exposes
    // Bridge Coupling / Resonance / Damping), so the coefficients MOVE while waves are passing
    // through the junction, and "passive at every parameter value" is not the same statement as
    // "passive along every path between them" -- a smoother that let sigma_total and the
    // sqrt(Z) weights disagree for even one sample would break the reflection's norm without ever
    // visiting an illegal parameter value.
    //
    // The construction is what makes this hold: sigma_total is rebuilt every sample from the SAME
    // smoothed roots the reflection multiplies by, so the adaptor is a legal positive-impedance
    // parallel junction at every instant of the glide. This case measures that claim.
    constexpr double kRate = 48000.0;
    const std::vector<float> impedances = impedanceSet(6, true);

    BridgeAdmittanceParams admittance;
    admittance.resonanceHz = 90.0f;
    admittance.damping = 0.05f;
    admittance.couplingStrength = 0.02f;
    BridgeJunction<double> junction = makeJunction<double>(kRate, impedances, admittance, true);
    const double zRefSquared = 2.0 * junction.referenceImpedance() * junction.referenceImpedance();

    Rng rng;
    std::array<double, cnpg::dsp::kMaxStrings> incident{};
    std::array<double, cnpg::dsp::kMaxStrings> outgoing{};
    double stored = 0.0;
    double worstImbalance = 0.0;
    double worstGain = 0.0;
    int retargets = 0;
    double mobilityLow = 1.0e300;
    double mobilityHigh = 0.0;

    for (int n = 0; n < 200000; ++n) {
        if ((n % 997) == 0) {
            // Deliberately violent: a full traverse of the parameter space every ~20 ms, which is
            // faster than the 8 ms smoother can ever settle, so the junction spends essentially the
            // whole render mid-glide rather than at a grid point.
            const double phase = static_cast<double>(n) / 200000.0;
            admittance.resonanceHz = static_cast<float>(60.0 + 7000.0 * phase);
            admittance.damping = static_cast<float>(0.02 + 4.0 * (1.0 - phase));
            admittance.couplingStrength = static_cast<float>(((n / 997) % 2 == 0) ? 1.0 : 0.01);
            junction.setAdmittance(admittance);
            ++retargets;
        }
        double handedIn = 0.0;
        for (int p = 0; p < 6; ++p) {
            incident[static_cast<std::size_t>(p)] = (n < 100000) ? rng.next() : 0.0;
            handedIn += static_cast<double>(impedances[static_cast<std::size_t>(p)]) *
                        incident[static_cast<std::size_t>(p)] * incident[static_cast<std::size_t>(p)];
        }
        junction.scatter(incident.data(), outgoing.data(), 6);
        double handedBack = 0.0;
        for (int p = 0; p < 6; ++p)
            handedBack += static_cast<double>(impedances[static_cast<std::size_t>(p)]) *
                          outgoing[static_cast<std::size_t>(p)] * outgoing[static_cast<std::size_t>(p)];
        const double now = junction.storageEnergy() * zRefSquared;
        const double residual = handedIn - handedBack - (now - stored);
        const double scale = std::max({handedIn, handedBack, now, stored, 1.0e-30});
        worstImbalance = std::max(worstImbalance, std::fabs(residual) / scale);
        worstGain = std::max(worstGain, -residual / scale);
        stored = now;
        mobilityLow = std::min(mobilityLow, junction.instantaneousMobility());
        mobilityHigh = std::max(mobilityHigh, junction.instantaneousMobility());
    }

    std::cout << "[energy] T1 BridgeJunction under " << retargets << " admittance retargets: worst energy GAIN "
              << worstGain << ", worst |imbalance| " << worstImbalance
              << " (dashpot bypassed, so this should be arithmetic only); instantaneous mobility ranged " << mobilityLow
              << " .. " << mobilityHigh << "\n";

    // IN the state this case claims to test: the parameters really moved, and they really moved the
    // load. A retarget count alone would be satisfied by a setAdmittance() that did nothing.
    REQUIRE(retargets > 100);
    REQUIRE(mobilityHigh > 10.0 * mobilityLow);
    REQUIRE(worstGain <= 1.0e-11);
}

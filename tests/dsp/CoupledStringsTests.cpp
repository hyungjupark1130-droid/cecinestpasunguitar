#include "cnpg/dsp/BridgeJunction.h"
#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

// CoupledStringsTests -- Task P2.4's [contract] gates for the thing the whole task exists to make:
// six strings that are one instrument. Sympathetic response, beating, Weinreich's two-stage decay,
// and the direct assertion that a string nobody plucked really does accumulate state.
//
// ---------------------------------------------------------------------------------------------
// WHY "IT GOT LOUDER" IS NOT ALLOWED TO BE THE MEASUREMENT HERE
// ---------------------------------------------------------------------------------------------
// The P2.1 review's standing ruling (a state change covered only by a level metric is not tested;
// a test must assert it is in the state it claims to exercise) bites hardest in exactly this file.
// "String 1's tap rose above -60 dBFS" is satisfied by: summing the wrong channel, a reference
// render that was never silent, numerical leakage through a shared buffer, and by raising
// couplingStrength until noise gets through. So every case below pairs its level measurement with
//   (a) a DIRECT state observation -- stringEnergyEstimate(k), which is that string's own share of
//       the storage functional and cannot be reached by anything except energy actually being in
//       that string's rails and filters; and
//   (b) a NEGATIVE CONTROL at couplingStrength 0, where the same render must produce EXACTLY zero
//       on the same channel -- not "quieter", bit-exactly zero, because a rigid bridge is
//       structurally incapable of moving energy between strings.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::FractionalDelayKind;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::dsp::StringNetwork;
using cnpg::dsp::StringNetworkParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;
constexpr double kTwoPi = 6.283185307179586;

NoteEvent noteOn(int sampleOffset, int midiNote, int stringIndex, float velocity = 0.8f) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = velocity;
    event.pluckPosition = 0.28f;
    event.hardness = 0.5f;
    return event;
}

double dbfs(double amplitude) { return 20.0 * std::log10(std::max(amplitude, 1.0e-30)); }

// Block-RMS envelope at `hop` samples, which is what an "amplitude modulation" claim is actually
// about. Returned with its own sample rate so the beat search below is in Hz and not in bins.
std::vector<double> rmsEnvelope(const std::vector<double>& signal, int hop) {
    std::vector<double> out;
    out.reserve(signal.size() / static_cast<std::size_t>(hop) + 1);
    for (std::size_t k = 0; k + static_cast<std::size_t>(hop) <= signal.size(); k += static_cast<std::size_t>(hop)) {
        double sum = 0.0;
        for (int n = 0; n < hop; ++n)
            sum += signal[k + static_cast<std::size_t>(n)] * signal[k + static_cast<std::size_t>(n)];
        out.push_back(std::sqrt(sum / static_cast<double>(hop)));
    }
    return out;
}

// Dominant modulation rate of an amplitude envelope, in Hz, searched inside [loHz, hiHz].
//
// The envelope of a decaying pair of strings is (beat) x (decay), so the log envelope is
// (periodic) + (a straight line). Taking the log and removing a least-squares line therefore
// removes the decay exactly and leaves the beat, which is why this is done in dB and not on the
// raw envelope -- an un-detrended DFT of a decaying envelope has all its energy at DC and the beat
// disappears under the skirt.
double dominantModulationHz(const std::vector<double>& envelope, double envelopeRate, double loHz, double hiHz) {
    const std::size_t count = envelope.size();
    if (count < 16)
        return 0.0;
    std::vector<double> logEnvelope(count);
    for (std::size_t k = 0; k < count; ++k)
        logEnvelope[k] = 20.0 * std::log10(std::max(envelope[k], 1.0e-30));

    // Least-squares line removal.
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    for (std::size_t k = 0; k < count; ++k) {
        const double x = static_cast<double>(k);
        sumX += x;
        sumY += logEnvelope[k];
        sumXX += x * x;
        sumXY += x * logEnvelope[k];
    }
    const double n = static_cast<double>(count);
    const double denom = n * sumXX - sumX * sumX;
    const double slope = (denom != 0.0) ? (n * sumXY - sumX * sumY) / denom : 0.0;
    const double intercept = (sumY - slope * sumX) / n;
    for (std::size_t k = 0; k < count; ++k)
        logEnvelope[k] -= intercept + slope * static_cast<double>(k);

    // Hann window, then a direct DFT over a fine frequency grid -- the search band is a handful of
    // hertz wide, so a dense grid is both cheaper and more accurate here than an FFT plus
    // interpolation.
    for (std::size_t k = 0; k < count; ++k)
        logEnvelope[k] *= 0.5 - 0.5 * std::cos(kTwoPi * static_cast<double>(k) / (n - 1.0));

    double bestHz = 0.0;
    double bestPower = -1.0;
    const int steps = 4000;
    for (int step = 0; step <= steps; ++step) {
        const double hz = loHz + (hiHz - loHz) * static_cast<double>(step) / static_cast<double>(steps);
        const double omega = kTwoPi * hz / envelopeRate;
        double re = 0.0;
        double im = 0.0;
        for (std::size_t k = 0; k < count; ++k) {
            const double phase = omega * static_cast<double>(k);
            re += logEnvelope[k] * std::cos(phase);
            im -= logEnvelope[k] * std::sin(phase);
        }
        const double power = re * re + im * im;
        if (power > bestPower) {
            bestPower = power;
            bestHz = hz;
        }
    }
    return bestHz;
}

// Peak-to-trough spread, in dB, of an envelope after its exponential decay has been removed by a
// least-squares line fit in the log domain -- i.e. how much MODULATION is left once the decay is
// taken out. Printed beside a measured beat rate so that "the search returned its low boundary"
// can be read as "there is no beat left to find" rather than as a suspiciously small number.
double detrendedLogSpreadDb(const std::vector<double>& envelope) {
    const std::size_t count = envelope.size();
    if (count < 16)
        return 0.0;
    std::vector<double> logEnvelope(count);
    for (std::size_t k = 0; k < count; ++k)
        logEnvelope[k] = 20.0 * std::log10(std::max(envelope[k], 1.0e-30));
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    for (std::size_t k = 0; k < count; ++k) {
        const double x = static_cast<double>(k);
        sumX += x;
        sumY += logEnvelope[k];
        sumXX += x * x;
        sumXY += x * logEnvelope[k];
    }
    const double n = static_cast<double>(count);
    const double denom = n * sumXX - sumX * sumX;
    const double slope = (denom != 0.0) ? (n * sumXY - sumX * sumY) / denom : 0.0;
    const double intercept = (sumY - slope * sumX) / n;
    double lo = 1.0e300;
    double hi = -1.0e300;
    // The first and last few points carry the fit's own edge effects; trim them.
    const std::size_t trim = count / 20 + 1;
    for (std::size_t k = trim; k + trim < count; ++k) {
        const double residual = logEnvelope[k] - (intercept + slope * static_cast<double>(k));
        lo = std::min(lo, residual);
        hi = std::max(hi, residual);
    }
    return (hi > lo) ? (hi - lo) : 0.0;
}

// Schroeder backward integration of `signal`, in dB relative to its own start, decimated onto
// `points` samples with the time of each point in seconds.
struct SchroederCurve {
    std::vector<double> seconds;
    std::vector<double> linear; // normalized to 1.0 at t = 0
};

// Restricts a Schroeder curve to a stated dynamic span, exactly as bandT60Seconds() already does
// for T60 (-5 .. -25 dB). NOT cosmetic: an envelope's decay curve flattens onto a floor -- the
// heterodyne partial filter's leakage from neighbouring partials, and eventually the render's own
// end -- and a fit scored over the WHOLE curve is rewarded for modelling that floor with a second,
// almost-zero-rate exponential. Measured on a single string that bought 21.8 dB of "improvement"
// from a second term, which is the two-stage effect this case exists to detect, appearing where
// there is provably only one stage. Restricting the span is what makes the comparison a comparison.
SchroederCurve trimCurve(const SchroederCurve& curve, double fromDb, double toDb) {
    SchroederCurve out;
    for (std::size_t k = 0; k < curve.linear.size(); ++k) {
        const double db = 10.0 * std::log10(curve.linear[k]);
        if (db > fromDb)
            continue;
        if (db < toDb)
            break;
        out.seconds.push_back(curve.seconds[k] - (out.seconds.empty() ? curve.seconds[k] : 0.0));
        out.linear.push_back(curve.linear[k]);
    }
    // Re-zero time and re-normalize level, so both fits see the same well-conditioned problem.
    if (!out.linear.empty()) {
        const double t0 = curve.seconds[curve.linear.size() - out.linear.size()];
        const double norm = out.linear.front();
        for (std::size_t k = 0; k < out.linear.size(); ++k) {
            out.linear[k] /= norm;
            out.seconds[k] = curve.seconds[curve.linear.size() - out.linear.size() + k] - t0;
        }
    }
    return out;
}

SchroederCurve schroeder(const std::vector<double>& signal, double sampleRate, int points) {
    std::vector<double> backward(signal.size() + 1, 0.0);
    for (std::size_t k = signal.size(); k-- > 0;)
        backward[k] = backward[k + 1] + signal[k] * signal[k];
    SchroederCurve curve;
    if (backward.front() <= 0.0)
        return curve;
    const double norm = backward.front();
    curve.seconds.reserve(static_cast<std::size_t>(points));
    curve.linear.reserve(static_cast<std::size_t>(points));
    for (int p = 0; p < points; ++p) {
        const auto index = static_cast<std::size_t>(static_cast<double>(p) / static_cast<double>(points) *
                                                    static_cast<double>(signal.size()));
        const double value = backward[index] / norm;
        if (value <= 0.0)
            break;
        curve.seconds.push_back(static_cast<double>(index) / sampleRate);
        curve.linear.push_back(value);
    }
    return curve;
}

// Best-fit residual, in dB RMS, of a sum of `terms` decaying exponentials against a Schroeder
// curve. The amplitudes enter LINEARLY, so the search is a grid over the decay rates with an
// exact (weighted) linear least squares inside -- no gradient descent, no starting guess, and the
// same residual metric for both models so "beats the single-exponential fit by 6 dB" compares
// like with like.
//
// The weighting is 1/S^2, i.e. the fit minimizes RELATIVE error, because a decay curve spans
// 60 dB and an unweighted fit would be a fit of its first 3 dB.
struct ExponentialFit {
    double residualDb = 1.0e30; // RMS of the dB error over the fitted span
    double peakDb = 0.0;        // largest |dB error| anywhere on it
    double rateA = 0.0;         // s^-1, energy decay rate (S ~ exp(-rate * t))
    double rateB = 0.0;
    double weightA = 0.0;
    double weightB = 0.0;
};

ExponentialFit fitExponentials(const SchroederCurve& curve, int terms) {
    ExponentialFit best;
    const std::size_t count = curve.seconds.size();
    if (count < 8)
        return best;

    constexpr int kGrid = 96;
    constexpr double kRateLo = 0.2;
    constexpr double kRateHi = 400.0;
    std::vector<double> rates(static_cast<std::size_t>(kGrid));
    for (int g = 0; g < kGrid; ++g)
        rates[static_cast<std::size_t>(g)] =
            kRateLo * std::pow(kRateHi / kRateLo, static_cast<double>(g) / static_cast<double>(kGrid - 1));

    auto residualFor = [&](double rateA, double rateB, int termCount) {
        // Weighted normal equations for c in min sum_k ((c . e_k - S_k)/S_k)^2.
        double m00 = 0.0;
        double m01 = 0.0;
        double m11 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        for (std::size_t k = 0; k < count; ++k) {
            const double t = curve.seconds[k];
            const double s = curve.linear[k];
            const double w = 1.0 / (s * s);
            const double a = std::exp(-rateA * t);
            const double b = (termCount == 2) ? std::exp(-rateB * t) : 0.0;
            m00 += w * a * a;
            m01 += w * a * b;
            m11 += w * b * b;
            v0 += w * a * s;
            v1 += w * b * s;
        }
        double cA = 0.0;
        double cB = 0.0;
        if (termCount == 1) {
            if (m00 <= 0.0)
                return ExponentialFit{};
            cA = v0 / m00;
        } else {
            const double det = m00 * m11 - m01 * m01;
            if (std::fabs(det) < 1.0e-30)
                return ExponentialFit{};
            cA = (v0 * m11 - v1 * m01) / det;
            cB = (v1 * m00 - v0 * m01) / det;
            if (cA <= 0.0 || cB <= 0.0)
                return ExponentialFit{}; // a negative amplitude is not a decay
        }
        double sumSquares = 0.0;
        double peak = 0.0;
        for (std::size_t k = 0; k < count; ++k) {
            const double t = curve.seconds[k];
            const double model = cA * std::exp(-rateA * t) + ((termCount == 2) ? cB * std::exp(-rateB * t) : 0.0);
            if (model <= 0.0)
                return ExponentialFit{};
            const double db = 10.0 * std::log10(model / curve.linear[k]);
            sumSquares += db * db;
            peak = std::max(peak, std::fabs(db));
        }
        ExponentialFit fit;
        fit.residualDb = std::sqrt(sumSquares / static_cast<double>(count));
        fit.peakDb = peak;
        fit.rateA = rateA;
        fit.rateB = (termCount == 2) ? rateB : 0.0;
        fit.weightA = cA;
        fit.weightB = cB;
        return fit;
    };

    if (terms == 1) {
        for (double rate : rates) {
            const ExponentialFit fit = residualFor(rate, 0.0, 1);
            if (fit.residualDb < best.residualDb)
                best = fit;
        }
        return best;
    }

    for (int a = 0; a < kGrid; ++a)
        for (int b = a + 1; b < kGrid; ++b) {
            const ExponentialFit fit =
                residualFor(rates[static_cast<std::size_t>(a)], rates[static_cast<std::size_t>(b)], 2);
            if (fit.residualDb < best.residualDb)
                best = fit;
        }
    return best;
}

struct CoupledRender {
    std::vector<double> bridge;
    std::vector<std::vector<double>> taps; // per string
    std::vector<double> energyOfString;    // final stringEnergyEstimate per string
    double energyOfSilentStringAfterFirstBlock = 0.0;
    unsigned long long unbridgedTicks = 0;
    bool silentStringReportedActive = false;
    float couplingInForce = -1.0f;
};

struct RenderSpec {
    float coupling = 0.5f;
    int strings = 2;
    int midiNote = 53;
    float detuneCentsOnString1 = 0.0f;
    bool pluckString1 = false;
    double seconds = 4.0;
    float lossKnob = 0.5f;
    float resonanceHz = 180.0f;
    float damping = 0.5f;
    double sampleRate = kRate;

    // EVERY STRING IN THESE RENDERS IS TUNED TO `midiNote`, INCLUDING THE ONES NOBODY PLUCKS.
    //
    // *** THIS FIELD EXISTS BECAUSE ITS ABSENCE MADE THE UNISON-PAIR CASE MEASURE SOMETHING ELSE
    // ENTIRELY, AND THE SUBSTITUTION WAS INVISIBLE UNTIL P2.7 FIXED THE THING IT LEANED ON. ***
    //
    // Weinreich's two-stage decay is a property of a UNISON PAIR: two strings at the SAME pitch
    // whose symmetric and antisymmetric normal modes decay at different rates. Through P2.6 an
    // unplucked string was left at kMinMidiNote -- A0, 27.5 Hz -- because StringNetwork::prepare()
    // had nowhere else to put it, so the case's "second string of the unison pair" was in fact an
    // A0 string, and A0 at 27.5 Hz has a mode every 27.5 Hz. That dense comb is a strong absorber
    // near anything, so a two-stage decay appeared and the case passed at 10.14 dB -- for the wrong
    // reason. Giving strings a real rest pitch (P2.7) put string 1 at its open A2 instead, whose
    // nearest mode to F3 is 35% away, and the measured improvement collapsed to 0.06 dB.
    //
    // The effect had never been measured. Setting the rest pitch to the played note is what makes
    // these renders the unison pair the plan's criterion names, so the number below is about
    // coupling and not about an accidental mode comb.
    int restMidiNote = -1; // < 0 means "the same note the render plays"
};

CoupledRender renderCoupled(const RenderSpec& spec) {
    StringNetworkParams params;
    params.pickupPosition01 = 0.87f;
    params.exciter.noiseAmount = 0.0f;
    params.stringMaterial.lossGainLow = spec.lossKnob;
    params.stringMaterial.lossGainHigh = spec.lossKnob;
    params.bridge.couplingStrength = spec.coupling;
    params.bridge.resonanceHz = spec.resonanceHz;
    params.bridge.damping = spec.damping;
    params.perString[1].tuningOffsetCents = spec.detuneCentsOnString1;
    // See RenderSpec::restMidiNote. A string nobody plucks still has a pitch, and in a unison-pair
    // measurement it has to be the pair's pitch.
    {
        const int rest = (spec.restMidiNote >= 0) ? spec.restMidiNote : spec.midiNote;
        for (auto& perString : params.perString)
            perString.restMidiNote = static_cast<std::uint8_t>(rest);
    }

    StringNetwork<float> network;
    network.prepare(spec.sampleRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(spec.strings);
    network.setParams(params);
    network.reset();

    BlockEventQueue events;
    events.push(noteOn(0, spec.midiNote, 0));
    if (spec.pluckString1)
        events.push(noteOn(1, spec.midiNote, 1));

    CoupledRender out;
    out.couplingInForce = network.internalBridgeJunction().currentCouplingStrength();
    out.taps.assign(static_cast<std::size_t>(spec.strings), {});
    const auto total = static_cast<std::size_t>(spec.seconds * spec.sampleRate);
    out.bridge.reserve(total);
    bool firstBlockDone = false;
    while (out.bridge.size() < total) {
        network.process(events, kBlock);
        for (int s = 0; s < spec.strings; ++s) {
            const float* channel = network.tapBuffers().channel(s, 0);
            for (int n = 0; n < kBlock; ++n)
                out.taps[static_cast<std::size_t>(s)].push_back(channel != nullptr ? static_cast<double>(channel[n])
                                                                                   : 0.0);
        }
        for (int n = 0; n < kBlock; ++n)
            out.bridge.push_back(static_cast<double>(network.bridgeOutputBuffer()[n]));
        if (!firstBlockDone) {
            out.energyOfSilentStringAfterFirstBlock = network.stringEnergyEstimate(1);
            firstBlockDone = true;
        }
        if (network.tapBuffers().isActive(1))
            out.silentStringReportedActive = true;
    }
    for (int s = 0; s < spec.strings; ++s)
        out.energyOfString.push_back(network.stringEnergyEstimate(s));
    out.unbridgedTicks = network.unbridgedTicks();
    return out;
}

double peakOf(const std::vector<double>& v, std::size_t from = 0) {
    double peak = 0.0;
    for (std::size_t k = from; k < v.size(); ++k)
        peak = std::max(peak, std::fabs(v[k]));
    return peak;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// carry-forward B1: the idle-string skip predicate, fixed and asserted DIRECTLY
// ---------------------------------------------------------------------------------------------

TEST_CASE("CoupledStrings: a quiescent string accumulates state from the bridge alone", "[contract]") {
    // THE LANDMINE (carry-forward B1, written into StringNetwork.cpp by Task P2.1 naming this
    // task). The per-sample loop skipped any string with no state of its own. Under bidirectional
    // coupling that predicate deletes sympathetic resonance in complete silence: the string
    // receiving bridge energy reports no state, is skipped, is never ticked, and the energy
    // vanishes -- with no test failing and nothing to hear, because the "before" is also silence.
    //
    // This case is the direct assertion the ruling asks for, and it deliberately does NOT look at
    // a level. stringEnergyEstimate(1) is string 1's own share of the storage functional -- its
    // rails, its dispersion allpasses, its loss filter, its interpolator states. Nothing except
    // energy genuinely being IN string 1 can move it.
    constexpr int kBlocks = 400;
    StringNetworkParams params;
    params.bridge.couplingStrength = 0.5f;
    params.exciter.noiseAmount = 0.0f;

    StringNetwork<float> network;
    network.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    network.setNumStrings(2);
    network.setParams(params);
    network.reset();

    // PRECONDITIONS, asserted rather than assumed: string 1 holds nothing, is not sounding, and
    // the bridge is genuinely loaded.
    REQUIRE(network.stringEnergyEstimate(0) == 0.0);
    REQUIRE(network.stringEnergyEstimate(1) == 0.0);
    REQUIRE(network.internalBridgeJunction().currentCouplingStrength() == 0.5f);
    REQUIRE(network.internalBridgeJunction().instantaneousMobility() > 0.0);

    BlockEventQueue events;
    events.push(noteOn(0, 53, 0)); // string 0 only; string 1 never receives an event of any kind

    double firstNonZeroAtSeconds = -1.0;
    double energyOfSilentString = 0.0;
    bool silentStringActive = false;
    for (int b = 0; b < kBlocks; ++b) {
        network.process(events, kBlock);
        energyOfSilentString = network.stringEnergyEstimate(1);
        if (firstNonZeroAtSeconds < 0.0 && energyOfSilentString > 0.0)
            firstNonZeroAtSeconds = static_cast<double>((b + 1) * kBlock) / kRate;
        if (network.tapBuffers().isActive(1))
            silentStringActive = true;
    }

    std::cout << "[contract] B1 idle-string skip: string 1 was never played; its own storage functional first became "
              << "non-zero at " << (firstNonZeroAtSeconds * 1000.0) << " ms and reached " << energyOfSilentString
              << " after " << (static_cast<double>(kBlocks * kBlock) / kRate)
              << " s (string 0: " << network.stringEnergyEstimate(0) << ")\n";

    REQUIRE(firstNonZeroAtSeconds >= 0.0);
    REQUIRE(energyOfSilentString > 0.0);
    // The domain boundary must agree, or PickupTap drops the channel and the feature is silence
    // again one level up -- which is the SAME defect, moved.
    REQUIRE(silentStringActive);
    REQUIRE(network.unbridgedTicks() == 0);

    // THE NEGATIVE CONTROL. A rigid bridge cannot move energy between strings, so the same render
    // must leave string 1 at EXACTLY zero -- not smaller, zero -- which is what says the number
    // above came from the coupling and not from anything else in the loop.
    StringNetworkParams decoupled = params;
    decoupled.bridge.couplingStrength = 0.0f;
    StringNetwork<float> control;
    control.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
    control.setNumStrings(2);
    control.setParams(decoupled);
    control.reset();
    BlockEventQueue controlEvents;
    controlEvents.push(noteOn(0, 53, 0));
    for (int b = 0; b < kBlocks; ++b) {
        control.process(controlEvents, kBlock);
        REQUIRE(control.stringEnergyEstimate(1) == 0.0);
    }
    REQUIRE(control.stringEnergyEstimate(0) > 0.0); // ...and string 0 really did ring in the control
}

// ---------------------------------------------------------------------------------------------
// acceptance: sympathetic response
// ---------------------------------------------------------------------------------------------

TEST_CASE("CoupledStrings: an unplucked unison string sings above -60 dBFS within a second", "[contract]") {
    // docs/plan.md Task P2.4 acceptance: "strings 0 and 1 in unison, couplingStrength 0.5, pluck
    // string 0 only: string 1's tap channel rises above -60 dBFS within 1 s."
    constexpr double kThresholdDb = -60.0;
    RenderSpec spec;
    spec.coupling = 0.5f;
    spec.midiNote = 53; // F3, 174.6 Hz -- a unison pair sitting on the default 180 Hz bridge mode
    spec.seconds = 2.0;
    const CoupledRender coupled = renderCoupled(spec);

    REQUIRE(coupled.couplingInForce == 0.5f); // IN the state the criterion names
    REQUIRE(coupled.unbridgedTicks == 0);
    REQUIRE(coupled.silentStringReportedActive);

    const auto oneSecond = static_cast<std::size_t>(1.0 * kRate);
    REQUIRE(coupled.taps[1].size() > oneSecond);
    std::vector<double> firstSecond(coupled.taps[1].begin(),
                                    coupled.taps[1].begin() + static_cast<std::ptrdiff_t>(oneSecond));
    const double peakWithinOneSecond = peakOf(firstSecond);
    const double peakOverall = peakOf(coupled.taps[1]);
    const double plucked = peakOf(coupled.taps[0]);

    // When it crosses, not just that it does.
    double crossedAtSeconds = -1.0;
    for (std::size_t k = 0; k < coupled.taps[1].size(); ++k)
        if (std::fabs(coupled.taps[1][k]) > std::pow(10.0, kThresholdDb / 20.0)) {
            crossedAtSeconds = static_cast<double>(k) / kRate;
            break;
        }

    // The negative control: same render, rigid bridge. Not "quieter" -- bit-exactly silent.
    RenderSpec decoupled = spec;
    decoupled.coupling = 0.0f;
    const CoupledRender control = renderCoupled(decoupled);
    const double controlPeak = peakOf(control.taps[1]);

    std::cout << "[contract] sympathetic response at couplingStrength 0.5, unison MIDI 53: unplucked string peaks "
              << dbfs(peakWithinOneSecond) << " dBFS within 1 s (" << dbfs(peakOverall) << " dBFS over 2 s), crossing "
              << kThresholdDb << " dBFS at " << (crossedAtSeconds * 1000.0) << " ms; plucked string peaks "
              << dbfs(plucked) << " dBFS; decoupled control peaks " << controlPeak << " (exactly)\n";

    REQUIRE(dbfs(peakWithinOneSecond) > kThresholdDb);
    REQUIRE(crossedAtSeconds >= 0.0);
    REQUIRE(crossedAtSeconds <= 1.0);
    REQUIRE(controlPeak == 0.0);
    // ...and the sympathetic string is genuinely quieter than the plucked one, so this is coupling
    // and not the two channels having been wired to the same string.
    REQUIRE(peakOverall < plucked);
    REQUIRE(coupled.energyOfString[1] > 0.0);
    REQUIRE(control.energyOfString[1] == 0.0);
}

// ---------------------------------------------------------------------------------------------
// acceptance: beating at the coupling where the uncoupled prediction holds
// ---------------------------------------------------------------------------------------------

TEST_CASE("CoupledStrings: a 4-cent detuned pair beats at the predicted rate", "[contract]") {
    // docs/plan.md Task P2.4 acceptance: "string 1 detuned +4 cents, both plucked, couplingStrength
    // 0.1 (where the uncoupled beat prediction holds; at higher coupling, normal-mode splitting per
    // Weinreich exceeds the naive prediction): bridge-output envelope shows amplitude modulation at
    // the predicted beat rate f0 * (2^(4/1200) - 1) within +/-20%."
    //
    // The gate is at 0.1 and ONLY at 0.1. The higher couplings below are reported, never asserted,
    // because a deviation there is Weinreich's normal-mode splitting doing exactly what the plan
    // says it does -- and "fix" it by widening the tolerance and the case stops measuring anything.
    constexpr float kDetuneCents = 4.0f;
    constexpr int kMidiNote = 64; // E4, 329.6 Hz: ~9 beat periods inside a 12 s render
    constexpr double kSeconds = 12.0;
    constexpr double kTolerance = 0.20;

    const double f0 = cnpg::test::midiNoteToHz(kMidiNote);
    const double predictedHz = f0 * (std::pow(2.0, static_cast<double>(kDetuneCents) / 1200.0) - 1.0);

    // WHICH ENVELOPE, and why it is not the raw broadband one. The predicted rate is
    // f0 * (2^(4/1200) - 1): it is a statement about the FUNDAMENTAL. A detune of k cents detunes
    // every partial by k cents, so partial n of the pair beats at n times that rate, and the raw
    // bridge output is the sum of all of them. Measured that way this case reads 1.548 Hz against a
    // 0.762 Hz prediction -- which is not a failure, it is partial 2 beating at exactly 2x and
    // carrying more of the bridge output than partial 1 does (the load is a bandpass, and at MIDI 64
    // its skirt does not favour the fundamental). So the gate is measured on the fundamental's own
    // envelope, via the heterodyne partial filter the [regression] suite already uses, and the
    // broadband figure is printed beside it as the cross-check it is: it must land on a MULTIPLE of
    // the prediction, not on something unrelated.
    constexpr double kPartialBandwidthHz = 30.0; // << the 329.6 Hz partial spacing, >> the 0.76 Hz beat
    constexpr int kHop = 256;

    struct BeatMeasurement {
        double fundamentalHz = 0.0;
        double broadbandHz = 0.0;
        double depthDb = 0.0; // peak-to-trough of the detrended log envelope: how much beat there is
    };

    auto measure = [&](float coupling) {
        RenderSpec spec;
        spec.coupling = coupling;
        spec.midiNote = kMidiNote;
        spec.detuneCentsOnString1 = kDetuneCents;
        spec.pluckString1 = true;
        spec.seconds = kSeconds;
        spec.lossKnob = 1.0f; // sustain end of the shipping range, so 12 s of tail exists to measure
        const CoupledRender render = renderCoupled(spec);
        REQUIRE(render.unbridgedTicks == 0);
        REQUIRE(render.couplingInForce == coupling);
        // BOTH strings really were plucked: a beat needs two sources, and a render where the second
        // NoteOn was dropped would produce no modulation and fail for the wrong reason.
        REQUIRE(render.energyOfString[0] > 0.0);
        REQUIRE(render.energyOfString[1] > 0.0);
        REQUIRE(peakOf(render.taps[0]) > 0.0);
        REQUIRE(peakOf(render.taps[1]) > 0.0);

        BeatMeasurement out;
        const std::vector<double> broadband = rmsEnvelope(render.bridge, kHop);
        out.broadbandHz = dominantModulationHz(broadband, kRate / static_cast<double>(kHop), 0.15, 6.0);

        const std::vector<double> partial = cnpg::test::partialEnvelope(render.bridge, kRate, f0, kPartialBandwidthHz);
        std::vector<double> decimated;
        decimated.reserve(partial.size() / static_cast<std::size_t>(kHop) + 1);
        for (std::size_t k = 0; k < partial.size(); k += static_cast<std::size_t>(kHop))
            decimated.push_back(partial[k]);
        out.fundamentalHz = dominantModulationHz(decimated, kRate / static_cast<double>(kHop), 0.15, 4.0);
        out.depthDb = detrendedLogSpreadDb(decimated);
        return out;
    };

    const BeatMeasurement atGate = measure(0.1f);
    const double error = (atGate.fundamentalHz - predictedHz) / predictedHz;

    std::cout << "[contract] beating, MIDI " << kMidiNote << " with +" << kDetuneCents
              << " cents on string 1: predicted " << predictedHz << " Hz\n"
              << "  couplingStrength 0.1 (GATED): fundamental-partial envelope " << atGate.fundamentalHz
              << " Hz, error " << (100.0 * error) << "%; broadband bridge envelope " << atGate.broadbandHz << " Hz ("
              << (atGate.broadbandHz / predictedHz) << "x the prediction -- partial "
              << std::llround(atGate.broadbandHz / predictedHz) << " beating)\n";

    // The detune itself, asserted: the whole prediction is a function of it.
    {
        StringNetworkParams params;
        params.perString[1].tuningOffsetCents = kDetuneCents;
        StringNetwork<float> probe;
        probe.prepare(kRate, kBlock, FractionalDelayKind::Lagrange3);
        probe.setNumStrings(2);
        probe.setParams(params);
        probe.reset();
        BlockEventQueue events;
        events.push(noteOn(0, kMidiNote, 0));
        events.push(noteOn(0, kMidiNote, 1));
        probe.process(events, kBlock);
        // (there is no per-string f0 accessor on the network; the detune is a parameter of the
        // string's own smoother, so the check that it is in force is the measured beat itself --
        // stated here rather than pretended otherwise.)
        REQUIRE(probe.stringEnergyEstimate(0) > 0.0);
        REQUIRE(probe.stringEnergyEstimate(1) > 0.0);
    }

    REQUIRE(atGate.fundamentalHz > 0.0);
    REQUIRE(std::fabs(error) <= kTolerance);
    // The broadband reading is a harmonic of the prediction, which is what says the two
    // measurements agree about the physics and differ only about which partial they are watching.
    const double broadbandRatio = atGate.broadbandHz / predictedHz;
    REQUIRE(std::fabs(broadbandRatio - std::round(broadbandRatio)) <= 0.15);

    // REPORT ONLY, per the criterion's own parenthesis, and the reason it says what it says. Above
    // this coupling the two strings stop beating at the detuning rate at all: once the coupling
    // exceeds the detuning, the normal modes PULL INTO each other (frequency locking -- the other
    // half of Weinreich's result), the fundamental's envelope stops modulating, and the search
    // returns whatever sits at its low boundary. `modulationDepthDb` is printed beside the rate so
    // a boundary hit reads as "there is no beat left to find" rather than as a suspiciously low
    // number -- and it is why the gate above is at couplingStrength 0.1 and nowhere else.
    for (float coupling : {0.35f, 0.5f, 1.0f}) {
        const BeatMeasurement measured = measure(coupling);
        std::cout << "  couplingStrength " << coupling << " (report only): fundamental-partial envelope "
                  << measured.fundamentalHz << " Hz (" << (100.0 * (measured.fundamentalHz - predictedHz) / predictedHz)
                  << "% from the UNCOUPLED prediction), modulation depth " << measured.depthDb << " dB vs "
                  << atGate.depthDb << " dB at the gated coupling"
                  << (measured.fundamentalHz <= 0.16 ? "  [at the search floor: no beat resolvable]" : "") << "\n";
    }
}

// ---------------------------------------------------------------------------------------------
// acceptance: Weinreich's two-stage decay
// ---------------------------------------------------------------------------------------------

TEST_CASE("CoupledStrings: a coupled unison pair decays in two stages", "[contract]") {
    // docs/plan.md Task P2.4 acceptance: "unison pair, pluck one string; a two-exponential fit to
    // the Schroeder-integrated bridge-output decay envelope beats a single-exponential fit by >= 6
    // dB residual and the two fitted decay rates differ by >= 2x."
    //
    // THE PHYSICS, so the numbers below mean something. Two identical strings on one bridge have
    // two normal modes: the SYMMETRIC one moves the bridge and therefore loses energy into it
    // quickly, and the ANTISYMMETRIC one puts a node on the bridge, cannot drive it, and decays
    // only on the strings' own loop loss. Plucking ONE string excites both equally, so the sound
    // is a fast stage followed by a long one -- the effect Weinreich (JASA 1977) described for
    // piano unisons, and the reason a two-exponential fit must beat a one-exponential fit here and
    // must NOT on a single string.
    //
    // MEASURED PER PARTIAL, and that is not a convenience. The normal-mode pair is a property of
    // ONE partial: partial n of the two strings splits into its own fast and slow mode. A broadband
    // Schroeder curve of the bridge output instead superimposes every partial's decay, and since a
    // plucked string's partials already have wildly different T60s, a two-exponential fit improves
    // on a one-exponential fit there whether the strings are coupled or not. Measured that way this
    // case read 2.5 dB of improvement for the coupled pair and 4.1 dB for a SINGLE STRING -- i.e.
    // the broadband measurement was dominated by multi-partial decay and was not measuring coupling
    // at all. The fundamental's own envelope, extracted with the heterodyne partial filter the
    // [regression] suite already uses, is the quantity Weinreich's result is about.
    constexpr double kResidualImprovementDb = 6.0;
    constexpr double kRateSeparation = 2.0;
    constexpr double kPartialBandwidthHz = 25.0;

    // *** THE CHANNEL, AND WHY IT IS NO LONGER THE BRIDGE OUTPUT (Task P2.7, with a derivation) ***
    //
    // docs/plan.md's P2.4 criterion names "the Schroeder-integrated BRIDGE-OUTPUT decay envelope".
    // For an EXACT unison pair that recipe is structurally incapable of showing the effect, and the
    // reason is the same sentence that defines the effect: the SLOW stage is the ANTISYMMETRIC
    // normal mode, whose defining property is a NODE AT THE BRIDGE. A mode that does not move the
    // bridge does not appear in bridgeOutput(), which is the bridge's velocity. So the bridge output
    // of a perfect unison contains the symmetric mode alone -- one exponential, by construction.
    //
    // Measured, on this instrument, fundamental envelope in dB at t = 0.05 / 0.2 / 0.8 / 3.2 / 6.4 s
    // (2 strings, MIDI 53, couplingStrength 0.5, sustain material):
    //
    //   string-0 tap    -31.7  -35.3  -35.7  -35.7  -35.8    <- fast stage, then the slow mode, flat
    //   bridge output   -58.4  -80.8 -167.9 -218.2 -220.3    <- one exponential, all the way down
    //   bridge, +2 cent -58.4  -78.7  -83.7  -84.2  -84.8    <- the slow mode reappears once the
    //                                                           pair is imperfect and its node
    //                                                           stops sitting exactly on the bridge
    //
    // The third row is the real-instrument case and is why the effect is audible at all: no two
    // strings are ever exactly in unison, so the slow mode always radiates a little. But the gate
    // must not depend on how imperfect the pair happens to be, so it measures the PLUCKED STRING'S
    // OWN TAP, where both modes are present at full amplitude for any tuning whatsoever.
    //
    // *** THIS CASE PASSED THROUGH P2.6 WITHOUT MEASURING A UNISON PAIR AT ALL. *** Until P2.7 gave
    // strings a rest pitch, the unplucked "second string of the pair" sat at kMinMidiNote -- A0,
    // 27.5 Hz, a mode every 27.5 Hz -- and that dense comb absorbed and re-emitted energy near F3,
    // producing a bend in the bridge-output curve that a two-exponential fit duly beat by 10.14 dB.
    // With a real unison pair the same measurement reads 0.11 dB. Recorded here rather than quietly
    // repaired, because "the gate measured the wrong device" is this project's recurring failure and
    // the only defence is writing down each occurrence.
    // `useBridgeChannel` selects the WITHDRAWN recipe (the bridge output) instead of the plucked
    // string's tap. It exists so the repair can be shown to fail on the defect it exists to catch --
    // see "THE CONSTRUCTED FAILURES" below -- rather than only asserted to pass.
    auto analyse = [](const RenderSpec& spec, double spanDb = -30.0, bool useBridgeChannel = false) {
        const CoupledRender render = renderCoupled(spec);
        REQUIRE(render.unbridgedTicks == 0);
        const double f0 = cnpg::test::midiNoteToHz(spec.midiNote);
        // Envelope of the fundamental, then its own energy decay curve. Schroeder-integrating the
        // SQUARED envelope (rather than the raw signal) is what makes the normal-mode beat -- which
        // a coupled unison pair has by construction, since coupling SPLITS the two modes in
        // frequency -- integrate out instead of rippling the fit.
        const std::vector<double> envelope = cnpg::test::partialEnvelope(
            useBridgeChannel ? render.bridge : render.taps[0], kRate, f0, kPartialBandwidthHz);
        // Skip the attack and the heterodyne filter's own settling, so the curve being fitted is a
        // decay and not a transient.
        const auto skip = static_cast<std::size_t>(0.15 * kRate);
        REQUIRE(envelope.size() > skip);
        const std::vector<double> tail(envelope.begin() + static_cast<std::ptrdiff_t>(skip), envelope.end());
        const SchroederCurve curve = trimCurve(schroeder(tail, kRate, 20000), -1.0, spanDb);
        REQUIRE(curve.seconds.size() > 100);
        const ExponentialFit single = fitExponentials(curve, 1);
        const ExponentialFit twin = fitExponentials(curve, 2);
        return std::pair<ExponentialFit, ExponentialFit>{single, twin};
    };

    RenderSpec spec;
    spec.coupling = 0.5f;
    spec.strings = 2;
    spec.midiNote = 53; // on the default bridge resonance, where the coupling is strongest
    spec.seconds = 10.0;
    spec.lossKnob = 1.0f; // sustain end: the antisymmetric mode must have somewhere to survive TO
    const auto pair = analyse(spec);
    const ExponentialFit& single = pair.first;
    const ExponentialFit& twin = pair.second;
    const double difference = single.residualDb - twin.residualDb;
    const double ratioDb = 20.0 * std::log10(single.residualDb / std::max(twin.residualDb, 1.0e-12));
    const double separation = (twin.rateA > 0.0) ? std::max(twin.rateB / twin.rateA, twin.rateA / twin.rateB) : 0.0;

    // THE CONTROL: one string, same everything. A single string has one decay, so a
    // two-exponential fit must NOT buy 6 dB there -- otherwise the fit is just absorbing curvature
    // and the case would pass on any decaying signal at all.
    RenderSpec solo = spec;
    solo.strings = 1;
    const auto soloPair = analyse(solo);
    const double soloImprovement = soloPair.first.residualDb - soloPair.second.residualDb;
    const double soloRatioDb =
        20.0 * std::log10(soloPair.first.residualDb / std::max(soloPair.second.residualDb, 1.0e-12));
    const double soloSeparation =
        (soloPair.second.rateA > 0.0)
            ? std::max(soloPair.second.rateB / soloPair.second.rateA, soloPair.second.rateA / soloPair.second.rateB)
            : 0.0;

    // ---------------------------------------------------------------------------------------
    // *** THE CONSTRUCTED FAILURES. *** The repair above is a claim about WHY this case used to
    // pass; a claim like that is worth exactly as much as the measurement that reproduces the old
    // reading. Both defects are rebuilt here, run, and their numbers printed.
    // ---------------------------------------------------------------------------------------
    //
    // DEFECT 1 -- THE WITHDRAWN CHANNEL. Measure the SAME genuine unison pair on the bridge output,
    // which is the recipe docs/plan.md's P2.4 criterion names. It must NOT show the effect, because
    // the slow stage is the antisymmetric mode and that mode has a node at the bridge: this is the
    // derivation above, asserted rather than narrated. If a future change ever made the bridge
    // output show two stages for an exact unison, the derivation would be wrong and the channel
    // choice would have to be re-argued.
    const auto bridgeChannelPair = analyse(spec, -30.0, /*useBridgeChannel=*/true);
    const double bridgeChannelPeakDb = bridgeChannelPair.first.peakDb;

    // DEFECT 2 -- THE A0 REST PITCH. Put the unplucked string back where P2.6 left it (kMinMidiNote,
    // A0, a mode every 27.5 Hz) and measure the bridge output, i.e. reproduce the configuration this
    // case actually ran under through P2.6. It DOES show two stages -- which is what says the old
    // 10.14 dB came from an accidental mode comb and not from Weinreich's normal modes.
    RenderSpec restedAtA0 = spec;
    restedAtA0.restMidiNote = cnpg::dsp::kMinMidiNote;
    const auto a0Pair = analyse(restedAtA0, -30.0, /*useBridgeChannel=*/true);
    const double a0PeakDb = a0Pair.first.peakDb;
    const double a0Separation = (a0Pair.second.rateA > 0.0) ? std::max(a0Pair.second.rateB / a0Pair.second.rateA,
                                                                       a0Pair.second.rateA / a0Pair.second.rateB)
                                                            : 0.0;

    // THE SPAN SWEEP behind the refusal recorded below: the single-exponential residual SATURATES.
    // Widening the fitted span does not make a one-exponential fit worse in RMS-dB terms, because
    // past the knee the curve is a straight line again and the extra points are fitted well.
    std::cout << "[contract] Weinreich two-stage decay, unison pair at couplingStrength " << spec.coupling << ", MIDI "
              << spec.midiNote << ":\n";
    for (double span : {-20.0, -30.0, -40.0, -50.0, -60.0}) {
        const auto probe = analyse(spec, span);
        std::cout << "  span -1 .. " << span << " dB: single-exponential residual " << probe.first.residualDb
                  << " dB RMS (peak " << probe.first.peakDb << " dB), two-exponential " << probe.second.residualDb
                  << " dB RMS (peak " << probe.second.peakDb << " dB); difference "
                  << (probe.first.residualDb - probe.second.residualDb) << " dB, ratio "
                  << (20.0 * std::log10(probe.first.residualDb / std::max(probe.second.residualDb, 1e-12))) << " dB\n";
    }
    std::cout << "  GATED span -1 .. -30 dB -- coupled pair: single-exponential residual " << single.residualDb
              << " dB RMS, peak error " << single.peakDb << " dB (rate " << single.rateA
              << " /s); two-exponential residual " << twin.residualDb << " dB RMS, peak error " << twin.peakDb
              << " dB (rates " << twin.rateA << " and " << twin.rateB << " /s, ratio " << separation << "); difference "
              << difference << " dB, RATIO " << ratioDb << " dB\n"
              << "  single-string control: single " << soloPair.first.residualDb << " dB RMS, PEAK error "
              << soloPair.first.peakDb << " dB, two-exponential " << soloPair.second.residualDb
              << " dB RMS, difference " << soloImprovement << " dB, ratio " << soloRatioDb << " dB, rate ratio "
              << soloSeparation << "\n";

    // ---------------------------------------------------------------------------------------
    // REFUSAL, WITH THE DERIVATION (carry-forward A3: physics governs over plan text)
    // ---------------------------------------------------------------------------------------
    // The criterion as written is "a two-exponential fit beats a single-exponential fit by >= 6 dB
    // residual". Read as an ARITHMETIC DIFFERENCE of dB-RMS residuals it is UNREACHABLE for this
    // system, and not because the effect is weak -- because of what the quantity is:
    //
    //   difference = residual(single) - residual(twin) >= 6 dB
    //
    // requires residual(single) >= 6 dB, since residual(twin) >= 0. residual(single) is the RMS
    // dB error of the BEST straight line through a log-decay curve that bends once, and that is
    // bounded BY THE SPAN: the worst case is a curve that is two straight segments, whose best
    // single line has RMS error 0.1875 x (span in dB). At the -1 .. -30 dB span this case fits, the
    // ceiling is 5.44 dB -- strictly under 6, so the difference reading cannot be met here at all.
    // The measured sweep printed above agrees and is well under the ceiling (~3.5 dB), saturating
    // and then FALLING as the span widens, because past the knee the slow mode is a straight line
    // again and the extra points are fitted well.
    //
    // The claim is scoped to the span DELIBERATELY. 0.1875 x span reaches 6 dB at a 32 dB span, so
    // "unreachable for any two-stage decay" would be over-stated; what is true is that it is
    // unreachable at any span this project's conventions license -- bandT60Seconds fits -5 .. -25,
    // this case fits -1 .. -30, and section 4.3's layer (a) uses the same family. (Corrected after
    // the P2.4 review derived the ceiling independently.)
    //
    // SUBSTITUTED, and strictly stronger than what the difference reading would have gated:
    //   (1) [WITHDRAWN AT P2.7 -- see below] the residual RATIO, 20*log10(single/twin) >= 6 dB.
    //   (2) the single-exponential fit must be wrong by >= 6 dB SOMEWHERE (peak error), which is the
    //       plain-language claim "a single exponential does not describe this decay" as an ABSOLUTE
    //       statement rather than a relative one. Measured 10.92 dB, against 0.03 dB for the control.
    //   (3) the rate separation the criterion also names, unchanged at >= 2x. Measured 2.23x,
    //       against 1.08x for the control.
    //
    // *** CONJUNCT (1) IS WITHDRAWN AS A GATE AT TASK P2.7, BECAUSE ON A GENUINE UNISON PAIR IT
    // POINTS THE WRONG WAY. *** Measured on the real pair: coupled -0.06 dB, single-string control
    // +21.94 dB. A gate written on it would PASS the control and FAIL the effect.
    //
    // The reason is the physics, not the arithmetic. The slow stage of a unison pair is the
    // antisymmetric mode, which has a node at the bridge and therefore loses energy only to the
    // strings' own loop loss -- at the sustain material this case uses, that is very nearly nothing,
    // so over the analysis window the slow stage is not a decaying exponential at all. Its Schroeder
    // curve is 10*log10(1 - t/T), the shape a CONSTANT envelope integrates to, and neither a one-
    // nor a two-exponential fit represents it: both misfit it equally (4.81 vs 4.84 dB RMS) and
    // their ratio is a comparison of two equally wrong numbers.
    //
    // This was invisible while the case was measuring the A0 artifact, where both stages really were
    // exponentials, and P2.4's own commentary had already recorded that the ratio is uninformative
    // for a well-fitted curve (the control clears it at 21.8 dB "because a scale-free ratio does not
    // care that BOTH of its residuals are 0.01 dB"). The measurement above is that observation
    // becoming decisive. What replaces it is the SEPARATION between the effect and its control on
    // the conjunct that does discriminate -- a factor of 330 on peak error -- which is the claim the
    // ratio was reaching for, made against the control instead of against the fit.
    //
    // CONJUNCT (2) IS SPAN-SENSITIVE, and the span is therefore a load-bearing constant rather than
    // a formatting choice: at -1 .. -20 dB the same coupled pair reads a peak error of 0.97 dB and
    // this gate would FAIL, because 20 dB of curve is not enough to contain the knee. The span is
    // -1 .. -30 dB for that reason and for one more -- it is the shallowest span that does contain
    // it, and going deeper (see the printed sweep) buys nothing but noise-floor. It is deliberately
    // NOT bandT60Seconds' -5 .. -25 dB: that window is chosen to measure ONE slope robustly, which
    // is the opposite of what this case needs.
    // The three MUST be a conjunction, and the single-string control is what proves it: the control
    // clears (1) at 21.8 dB, because a scale-free ratio does not care that BOTH of its residuals
    // are 0.01 dB -- a very well fitted curve fitted slightly better still improves by a large
    // ratio. It is (2) and (3) that discriminate, and they do so by three orders of magnitude. A
    // gate written on the ratio alone would have been a gate on nothing, which is precisely the
    // failure mode this project keeps finding, so it is recorded here rather than quietly patched.
    REQUIRE(single.peakDb >= kResidualImprovementDb);
    REQUIRE(separation >= kRateSeparation);
    // The control must NOT show the effect, asserted on the two ABSOLUTE quantities (see above).
    REQUIRE(soloPair.first.peakDb < kResidualImprovementDb);
    REQUIRE(soloSeparation < kRateSeparation);
    // ...and the two must be SEPARATED, not merely on opposite sides of a threshold. This is what
    // replaces the withdrawn ratio conjunct: it is the same "beats it by a wide margin" claim, made
    // between the effect and its control rather than between two fits of the same curve. Measured
    // 10.92 dB against 0.033 dB, a factor of 330, so the 10x bound has two decades of headroom and
    // still fails immediately if the coupled render ever stops showing two stages.
    REQUIRE(single.peakDb > 10.0 * soloPair.first.peakDb);

    // ---- and the two constructed failures, asserted -------------------------------------------
    std::cout << "  CONSTRUCTED FAILURES (the two defects this case was repaired from, rebuilt and measured):\n"
              << "    (1) the WITHDRAWN CHANNEL -- the same genuine unison pair on the BRIDGE OUTPUT, which is the "
                 "recipe P2.4's criterion names: peak error "
              << bridgeChannelPeakDb << " dB against the " << kResidualImprovementDb
              << " dB criterion. The bridge cannot see the slow mode, because the slow mode's node is ON it.\n"
              << "    (2) the A0 REST PITCH -- the unplucked string put back at kMinMidiNote and measured on the "
                 "bridge output, i.e. the configuration this case ran under through P2.6: peak error "
              << a0PeakDb << " dB, rate separation " << a0Separation
              << ". THAT is where the old 10.14 dB came from -- a mode every 27.5 Hz absorbing near F3, not "
                 "Weinreich's normal modes.\n";

    // (1) must FAIL the criterion: an exact unison's slow mode is invisible at the bridge.
    INFO("withdrawn channel (bridge output, genuine unison pair): peak error " << bridgeChannelPeakDb << " dB");
    REQUIRE(bridgeChannelPeakDb < kResidualImprovementDb);
    // ...and it must fail by a wide margin, not merely land under the line -- otherwise the channel
    // choice is a coin toss rather than a derivation.
    REQUIRE(bridgeChannelPeakDb < 0.25 * single.peakDb);

    // (2) must PASS the criterion it should never have been passing: the A0 comb reproduces the old
    // reading. This is the assertion that makes "the gate was measuring the wrong device" a
    // measurement. If a future change stopped it passing, the explanation for the P2.6 number would
    // be wrong and this case's repair would need re-deriving.
    INFO("A0 rest pitch (the P2.6 configuration): peak error " << a0PeakDb << " dB, separation " << a0Separation);
    REQUIRE(a0PeakDb >= kResidualImprovementDb);

    (void)ratioDb;
    (void)difference;
    (void)soloRatioDb;
    (void)soloImprovement;
}

// ---------------------------------------------------------------------------------------------
// the evidence behind the shipping couplingStrength default (carry-forward B2 / ADR 0004 am. 3)
// ---------------------------------------------------------------------------------------------

TEST_CASE("CoupledStrings: the couplingStrength default is a measured choice", "[contract]") {
    // carry-forward B2: "couplingStrength still defaults to 0.0, and 0.0 is a silent kill switch.
    // You must decide and record a nonzero shipping default, justify it against the
    // sympathetic-response and beating criteria, and record it in the report and in an ADR."
    //
    // This case is the measurement that justification stands on, and it is a gate as well as a
    // table: whatever the shipped default is, it must clear the sympathetic-response criterion by
    // itself, and it must not cost more than a stated fraction of the uncoupled sustain.
    constexpr double kSympatheticThresholdDb = -60.0;
    constexpr double kMaxSustainCost = 0.60; // the coupled T60 must stay above 40% of the uncoupled one

    const float shipped = StringNetworkParams{}.bridge.couplingStrength;
    std::cout << "[contract] couplingStrength evidence table (shipped default " << shipped << "):\n"
              << "  coupling | sympathetic peak (dBFS, 1 s) | bridge-output peak | plucked-string T60 (s)\n";

    double shippedSympatheticDb = -300.0;
    double uncoupledT60 = -1.0;
    double shippedT60 = -1.0;

    for (float coupling : {0.0f, 0.05f, 0.1f, 0.2f, shipped, 0.5f, 1.0f}) {
        RenderSpec spec;
        spec.coupling = coupling;
        spec.midiNote = 53;
        spec.seconds = 3.0;
        const CoupledRender render = renderCoupled(spec);
        REQUIRE(render.couplingInForce == coupling);

        const auto oneSecond = static_cast<std::size_t>(1.0 * kRate);
        std::vector<double> firstSecond(render.taps[1].begin(),
                                        render.taps[1].begin() + static_cast<std::ptrdiff_t>(oneSecond));
        const double sympatheticDb = dbfs(peakOf(firstSecond));
        const double bridgePeak = peakOf(render.bridge);
        const double t60 = cnpg::test::bandT60Seconds(render.taps[0], kRate, 250.0);

        std::cout << "  " << coupling << "      | " << sympatheticDb << " | " << bridgePeak << " | " << t60 << "\n";

        if (coupling == 0.0f) {
            uncoupledT60 = t60;
            REQUIRE(peakOf(render.taps[1]) == 0.0); // the kill switch really does kill
            REQUIRE(bridgePeak == 0.0);             // ...and takes bridgeOutput() with it, per section 2.6
        }
        if (coupling == shipped) {
            shippedSympatheticDb = sympatheticDb;
            shippedT60 = t60;
        }
    }

    std::cout << "  shipped default " << shipped << ": sympathetic peak " << shippedSympatheticDb
              << " dBFS within 1 s (criterion " << kSympatheticThresholdDb << "), 250 Hz-band T60 " << shippedT60
              << " s vs " << uncoupledT60 << " s uncoupled (" << (100.0 * shippedT60 / uncoupledT60) << "% of it)\n";

    REQUIRE(shipped > 0.0f); // ADR 0004 amendment 3, closed
    REQUIRE(shippedSympatheticDb > kSympatheticThresholdDb);
    REQUIRE(uncoupledT60 > 0.0);
    REQUIRE(shippedT60 > 0.0);
    REQUIRE(shippedT60 >= (1.0 - kMaxSustainCost) * uncoupledT60);
}

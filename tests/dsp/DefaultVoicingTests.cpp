#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/StringNetwork.h"

#include "support/P1Chain.h"
#include "support/SpectralAnalysis.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

// DefaultVoicingTests -- THE ASSERTION NOBODY WROTE, part two: the default GEOMETRY.
//
// tests/dsp/DamperReleaseSpectrumTests.cpp gates that a note-off leaves a musical decay, after the
// author heard that it did not. The same physics -- a point contact couples to mode n through
// sin(n*pi*p), so it is exactly blind to every partial with a node at p -- also governs where the
// pick lands and where the coil sits, and there both defaults sat on p = 1/2.
//
// p = 1/2 is not merely a bad choice; it is provably the WORST point on the whole slider. Written
// as a rational a/b in lowest terms, a comb at a/b nulls exactly the partials {b, 2b, 3b, ...}:
// ONSET b, DENSITY 1/b. b = 2 is the smallest value b can take, so p = 1/2 simultaneously has the
// lowest possible onset (partial 2, the OCTAVE) and the highest possible density (half of every
// partial the instrument produces). And the exciter and the tap were at the SAME p, so the two
// combs coincided and every null was squared.
//
// Measured through the shipping chain at the low open E, dB below the loudest partial:
//
//   partial     1      2      3      4      5      6      7      8
//   old       0.0  -48.9   -3.4  -43.1   -0.6  -48.1  -16.4  -43.5
//
// Half the harmonic series was not attenuated, it was absent. docs/listening/physical-plausibility-
// checklist.md item 7 already names the pathology in words -- "near-middle plucks are hollow, with
// suppressed even harmonics" -- and the shipped default WAS the near-middle pluck; item 7 is also
// the item whose pluck half P2.8's review found had NO corpus evidence at all, because every render
// left Exciter Position at 0.5.
//
// Three [contract] cases, and EACH IS SHOWN RED AT THE OLD DEFAULT IN ITS OWN BODY, because a gate
// that only ever passes is not evidence:
//
//   1. THE MEASUREMENT. No partial in the identity band may sit in a hole, measured on the rendered
//      spectrum of the lowest open string.
//   2. THE DERIVATION. The closed form both defaults were chosen from, asserted on the defaults
//      themselves, with no render and no geometry-dependent estimator in it.
//   3. THE GAIN STAGING. A six-string open chord at the default velocity keeps real headroom under
//      the limiter -- the promise TriodeStage.h makes about the +16 dB summing budget and which
//      docs/listening/P2-20260803.md records as never having been measured.
//
// Everything here is reproducible: build\bin\Release\cnpg_tests.exe "[report]" runs the hidden
// sweeps every table in
// .superpowers/sdd/2026-07-30-pm-guitar-synth-p0-p2-plan/task-default-voicing.md came from, and
// -- since 2026-08-05 -- the realised-onset sweep that task-default-diffs.md's REFUSAL of a
// proposed tap at 1 - 1/7 rests on.

using cnpg::dsp::BlockEventQueue;
using cnpg::dsp::NoteEvent;
using cnpg::dsp::NoteEventType;
using cnpg::test::P1Chain;
using cnpg::test::P1ChainParams;

namespace {

constexpr double kRate = 48000.0;
constexpr int kBlock = 128;

NoteEvent noteOn(int sampleOffset, int stringIndex, int midiNote, float velocity, float pluckPosition) {
    NoteEvent event{};
    event.type = NoteEventType::NoteOn;
    event.sampleOffset = sampleOffset;
    event.stringIndex = static_cast<std::uint8_t>(stringIndex);
    event.channel = 0;
    event.midiNote = static_cast<std::uint8_t>(midiNote);
    event.velocity = velocity;
    event.pluckPosition = pluckPosition;
    event.hardness = cnpg::dsp::kUnspecifiedNoteParam;
    return event;
}

struct Note {
    int sampleOffset = 0;
    int stringIndex = 0;
    int midiNote = 48;
    float velocity = 0.8f;
};

struct Rendered {
    std::vector<double> out; // chain output (post limiter)
    std::vector<double> tap; // string 0's raw displacement tap, pre-pickup
};

Rendered render(const P1ChainParams& params, const std::vector<Note>& notes, double seconds, int numStrings) {
    P1ChainParams p = params;
    p.numStrings = numStrings;

    P1Chain chain;
    chain.prepare(kRate, kBlock, numStrings, cnpg::dsp::Oversampler::kDefaultFactor);
    // Settle every ramp onto its target BEFORE the first note, exactly as
    // MonitoringChainTests::ChainHarness does ("settle every ramp so the ceiling bounds the very
    // first sample"). TriodeStageParams' own outputTrimDb default is 0 dB while the assembled chain
    // runs it at kUnityGainOutputTrimDb = -13.98 dB, so a note struck in the very first block after
    // prepare() rides a trim ramp that has not arrived yet and reads up to 14 dB hot. Applying the
    // parameters once and then reset()ing collapses the ramp; without it every level in this file
    // would be a measurement of the harness.
    {
        BlockEventQueue warmup;
        chain.processBlock(p, warmup, kBlock);
    }
    chain.reset();

    const auto totalBlocks = static_cast<int>(seconds * kRate / kBlock);
    Rendered r;
    r.out.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));
    r.tap.reserve(static_cast<std::size_t>(totalBlocks) * static_cast<std::size_t>(kBlock));

    BlockEventQueue events;
    for (int b = 0; b < totalBlocks; ++b) {
        const int blockStart = b * kBlock;
        for (const Note& n : notes) {
            if (n.sampleOffset >= blockStart && n.sampleOffset < blockStart + kBlock)
                events.push(noteOn(n.sampleOffset - blockStart, n.stringIndex, n.midiNote, n.velocity,
                                   cnpg::dsp::kUnspecifiedNoteParam));
        }
        chain.processBlock(p, events, kBlock);
        const float* tap = chain.network.tapBuffers().channel(0, 0);
        for (int i = 0; i < kBlock; ++i) {
            r.out.push_back(static_cast<double>(chain.mono[static_cast<std::size_t>(i)]));
            r.tap.push_back(tap != nullptr ? static_cast<double>(tap[i]) : 0.0);
        }
    }
    return r;
}

// Peak power within +/- searchCents of n*f0, in dB. -300 when the band holds nothing.
double partialDb(const cnpg::test::Spectrum& s, double hz, double searchCents = 45.0) {
    const double lo = hz * std::pow(2.0, -searchCents / 1200.0);
    const double hi = hz * std::pow(2.0, searchCents / 1200.0);
    const auto loBin = static_cast<std::size_t>(std::max(1.0, std::floor(s.hzToBin(lo))));
    const auto hiBin = static_cast<std::size_t>(std::ceil(s.hzToBin(hi)));
    double peak = 0.0;
    for (std::size_t b = loBin; b <= hiBin && b < s.magnitudeSquared.size(); ++b)
        peak = std::max(peak, s.magnitudeSquared[b]);
    return peak > 0.0 ? 10.0 * std::log10(peak) : -300.0;
}

std::vector<double> partialSpectrumDb(const std::vector<double>& samples, double f0, int maxPartial,
                                      double skipSeconds = 0.20, std::size_t analysisLength = 65536) {
    const auto skip = static_cast<std::size_t>(skipSeconds * kRate);
    std::vector<double> window(samples.begin() + static_cast<std::ptrdiff_t>(skip), samples.end());
    const cnpg::test::Spectrum s = cnpg::test::computeSpectrum(window, kRate, analysisLength);
    std::vector<double> db;
    db.reserve(static_cast<std::size_t>(maxPartial));
    for (int n = 1; n <= maxPartial; ++n)
        db.push_back(partialDb(s, f0 * n));
    return db;
}

double peakDbfs(const std::vector<double>& x) {
    double peak = 0.0;
    for (double v : x)
        peak = std::max(peak, std::fabs(v));
    return peak > 0.0 ? 20.0 * std::log10(peak) : -300.0;
}

double rmsDbfs(const std::vector<double>& x, std::size_t from, std::size_t to) {
    double sum = 0.0;
    to = std::min(to, x.size());
    if (to <= from)
        return -300.0;
    for (std::size_t i = from; i < to; ++i)
        sum += x[i] * x[i];
    const double rms = std::sqrt(sum / static_cast<double>(to - from));
    return rms > 0.0 ? 20.0 * std::log10(rms) : -300.0;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// 1. the measurement
// ---------------------------------------------------------------------------------------------

namespace {

// The six default open strings (Common.h's kDefaultOpenStringMidiNote), i.e. what an untouched
// instrument plays when somebody strums it -- which is the whole question a shipping default
// answers. Gated at 48 kHz; the rate-independent half of the claim is the closed form in case 2.
constexpr std::array<int, 6> kOpenStrings{40, 45, 50, 55, 59, 64};

// THE STATISTIC. For each EVEN partial in the identity band, how far it sits below the mean of its
// two ODD neighbours. Even partials are where p = 1/2's nulls are, and comparing each against its
// immediate neighbours rather than against the fundamental makes the reading independent of the
// note's overall spectral tilt -- which is exactly the thing the new geometry deliberately changes,
// so a gate that measured tilt would be measuring the fix instead of the defect.
double worstEvenPartialDeficitDb(const std::vector<double>& out, double f0) {
    const std::vector<double> db = partialSpectrumDb(out, f0, 8);
    double worst = -300.0;
    for (int n = 2; n <= 6; n += 2)
        worst = std::max(worst, 0.5 * (db[static_cast<std::size_t>(n - 2)] + db[static_cast<std::size_t>(n)]) -
                                    db[static_cast<std::size_t>(n - 1)]);
    return worst;
}

// THE LIMIT, bracketed by measurement in both directions (both readings are reproduced by
// "REPORT: voicing -- bracketing the comb-hole limit" below, re-measured at couplingStrength 0.20):
//
//   healthy -- at the shipping geometry the six open strings read -3.02 .. +5.78 dB, and widening
//              to MIDI 36 and 72 only reaches +10.68;
//   defect  -- at the geometry this replaced, the same six read +37.47 .. +46.25 dB.
//
// 18 dB therefore sits 12.2 dB above the worst healthy reading over the gated set and 19.5 dB below
// the weakest reading the defect produces. It is a ratio between partials of one note, so nothing
// in it depends on level calibration or on kNominalPickupTrimDb. It DOES move a little with
// couplingStrength -- the readings above shifted by up to 2.6 dB when that default went 0.35 ->
// 0.20 on 2026-08-05 -- so the numbers quoted here name the coupling they were taken at, and the
// gate's own RED arm re-measures the defect on every run rather than quoting one. The readings
// above moved by up to 4.5 dB when couplingStrength went 0.35 -> 0.20 on 2026-08-05.
//
// *** THIS GATE IS THE ONE THAT CAUGHT dp = 1/7. *** It is not decorative and it is not merely a
// slower restatement of the closed form in case 2: the closed form is a continuous-string identity
// and the onset this waveguide REALISES is lower than 1/d by about one sample of tap distance
// (StringNetwork.h). A tap at the zero-margin 1/7 passes case 2 and reads 21.72 dB here, on the
// open B string. See "REPORT: voicing -- the realised comb onset is lower than 1/d".
constexpr double kMaxEvenPartialDeficitDb = 18.0;

} // namespace

TEST_CASE("CONTRACT: DefaultVoicing -- the default geometry does not delete the even harmonics", "[contract]") {
    const P1ChainParams shipping = cnpg::test::makeDefaultP1ChainParams();

    // The geometry really is the shipped one and not something this file assembled.
    REQUIRE(shipping.network.exciter.defaultPosition == cnpg::dsp::PluckExciterParams{}.defaultPosition);
    REQUIRE(shipping.network.pickupPosition01 == cnpg::dsp::StringNetworkParams{}.pickupPosition01);

    // ---- THE DEFECT, so the gate is shown to fail on what it exists to catch ------------------
    P1ChainParams defect = shipping;
    defect.network.exciter.defaultPosition = 0.5f;
    defect.network.pickupPosition01 = 0.5f;

    double worstShipping = -300.0;
    double weakestDefect = 300.0;
    int worstShippingNote = 0;
    int weakestDefectNote = 0;
    for (int midi : kOpenStrings) {
        const double f0 = cnpg::test::midiNoteToHz(midi);

        const Rendered good = render(shipping, {{0, 0, midi, 0.8f}}, 3.0, 1);
        // NON-VACUITY: the note really sounded, so "no deep hole" is not "no signal".
        REQUIRE(peakDbfs(good.out) > -40.0);
        const double goodDeficit = worstEvenPartialDeficitDb(good.out, f0);
        if (goodDeficit > worstShipping) {
            worstShipping = goodDeficit;
            worstShippingNote = midi;
        }

        const Rendered bad = render(defect, {{0, 0, midi, 0.8f}}, 3.0, 1);
        REQUIRE(peakDbfs(bad.out) > -40.0);
        const double badDeficit = worstEvenPartialDeficitDb(bad.out, f0);
        if (badDeficit < weakestDefect) {
            weakestDefect = badDeficit;
            weakestDefectNote = midi;
        }
    }

    std::cout << "[contract] default voicing, six open strings at 48 kHz: worst EVEN-partial deficit at the shipping "
              << "geometry (pluck " << shipping.network.exciter.defaultPosition << ", tap "
              << shipping.network.pickupPosition01 << ") is " << worstShipping << " dB at MIDI " << worstShippingNote
              << "; at the geometry it replaced (0.5 / 0.5) the WEAKEST reading is " << weakestDefect << " dB at MIDI "
              << weakestDefectNote << "; limit " << kMaxEvenPartialDeficitDb << " dB\n";

    INFO("shipping worst " << worstShipping << " dB at MIDI " << worstShippingNote << "; defect weakest "
                           << weakestDefect << " dB at MIDI " << weakestDefectNote);

    // THE GATE.
    REQUIRE(worstShipping <= kMaxEvenPartialDeficitDb);
    // ...and THE RED it is calibrated against. Not one open string, EVERY one of the six: the
    // weakest reading the old geometry produces is still 12 dB past the limit.
    REQUIRE(weakestDefect > kMaxEvenPartialDeficitDb + 12.0);
}

// ---------------------------------------------------------------------------------------------
// 2. the derivation
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DefaultVoicing -- both default positions clear the identity band in closed form", "[contract]") {
    // WHAT THIS COVERS THAT THE RENDER ABOVE CANNOT. That case measures one rate, one velocity and
    // one estimator, and it can only see a hole a render puts in front of it. This one is the
    // closed form both defaults were chosen from: it has no rate, no note and no estimator in it,
    // so it is a statement about the instrument rather than about a measurement of it.
    //
    //     a point contact couples to mode n through sin(n*pi*d), d = distance from a termination
    //     => the null set of a comb at a rational d = a/b (lowest terms) is exactly {b, 2b, 3b,...}
    //     => onset b, density 1/b, and b = 2 is the smallest b can be
    //
    // The identity band is [2, 6]: partials 2 and 4 are octaves, 3 and 6 are fifths (2 cents from
    // tempered), 5 is a major third (14 cents flat), and partial 7 is the FIRST partial naming no
    // tempered interval at all (31 cents flat of a minor seventh). Requiring no null inside it is
    // onset >= 7, i.e. d <= 1/7.
    constexpr double kPi = 3.14159265358979323846;
    const double pluck = static_cast<double>(cnpg::dsp::PluckExciterParams{}.defaultPosition);
    const double tap = static_cast<double>(cnpg::dsp::StringNetworkParams{}.pickupPosition01);

    // (0) Both belong at the BRIDGE end. sin^2 is symmetric about 0.5, so the mirror of each value
    // is acoustically near-identical -- but these two parameters are user-facing and labelled
    // "Exciter Position" and "Pickup Position", and a pickup at 0.0625 would be a pickup mounted at
    // the NUT. The label has to read true, so the side is pinned, not left to the symmetry.
    REQUIRE(pluck > 0.5);
    REQUIRE(tap > 0.5);

    const double pluckDistance = 1.0 - pluck; // from the bridge, which is the nearer termination
    const double tapDistance = 1.0 - tap;

    // (1) Neither comb nulls a partial in [2, 6]. The onsets below are PRINTED; the criterion is
    // GATED in the form d <= 1/7 rather than 1/d >= 7, and the difference is not pedantry.
    //
    // The two are the same criterion over the reals and are NOT the same predicate in floating
    // point AT THE BOUNDARY -- which is where a proposed tap value sat on 2026-08-05 (1 - 1/7; the
    // move was refused for a different and larger reason, see the realised-onset report below).
    // float(1/7) rounds UP to 0.14285714924..., so its float reciprocal is 6.99999952f: the
    // reciprocal form REJECTS the exact boundary value. In double,
    // 1.0/(1.0 - double(1.0f - 1.0f/7.0f)) lands on 7.000000417 and therefore ACCEPTS it, by 4e-7
    // of rounding rather than by any property of the instrument. A gate whose verdict at its own
    // boundary is decided by which way a rounding went is not measuring the physics, so the clause
    // below compares the shipped DISTANCE against the same expression that defines the limit, in
    // the type the header declares it in, where equality is exact.
    // dsp/include/cnpg/dsp/StringNetwork.h's static_assert carries the same correction -- in its
    // previous form it would have FAILED TO COMPILE at d = 1/7.
    const double pluckOnset = 1.0 / pluckDistance;
    const double tapOnset = 1.0 / tapDistance;

    // ...and the parameters really are derived from those constants rather than restating them, so
    // gating the constants gates what the instrument uses.
    REQUIRE(cnpg::dsp::PluckExciterParams{}.defaultPosition == 1.0f - cnpg::dsp::kDefaultPluckDistanceFromBridge01);
    REQUIRE(cnpg::dsp::StringNetworkParams{}.pickupPosition01 == 1.0f - cnpg::dsp::kDefaultTapDistanceFromBridge01);

    // (2) The two combs are not COINCIDENT, and their null sets do not meet inside the band the
    // render above speaks for. Coincidence is the mechanism, not a detail: it is why the old
    // default measured a 47 dB hole where one comb alone would have given about 25.
    // Written as a function of the two distances rather than inline, so the RED arm below runs the
    // SAME code on the old geometry instead of re-deriving the answer from literals. A RED arm that
    // recomputes "the first even number" by hand would assert nothing about this gate.
    auto firstSharedNullAtOrBelow = [](double de, double dp, int band) {
        const int ePeriod = static_cast<int>(std::lround(1.0 / de));
        const int pPeriod = static_cast<int>(std::lround(1.0 / dp));
        for (int n = 2; n <= band; ++n)
            if (n % ePeriod == 0 && n % pPeriod == 0)
                return n;
        return 0;
    };
    const int firstSharedNull = firstSharedNullAtOrBelow(pluckDistance, tapDistance, 24);

    // (3) The product comb, which is what a partial actually arrives through, keeps every partial
    // in the identity band. 1.0 would be a partial read at a comb maximum at both ends.
    auto productComb = [&](double de, double dp, int n) {
        return std::fabs(std::sin(kPi * n * de) * std::sin(kPi * n * dp));
    };
    double worstInBand = 1.0e300;
    int worstPartial = 0;
    for (int n = 2; n <= 6; ++n) {
        const double w = productComb(pluckDistance, tapDistance, n);
        if (w < worstInBand) {
            worstInBand = w;
            worstPartial = n;
        }
    }

    std::cout << "[contract] default voicing geometry: pluck " << pluck << " (1/" << pluckOnset
              << " of the string from the bridge, " << (25.5 / pluckOnset) << "\" on a 25.5\" scale, first null at "
              << "partial " << pluckOnset << "), tap " << tap << " (1/" << tapOnset << ", " << (25.5 / tapOnset)
              << "\", first null at partial " << tapOnset << "); null sets share nothing at or below partial 24; "
              << "worst product comb over [2, 6] is n = " << worstPartial << " at " << worstInBand
              << "; at 0.5 / 0.5 the same reading is " << productComb(0.5, 0.5, 2) << " at n = 2\n";

    INFO("pluck onset " << pluckOnset << ", tap onset " << tapOnset << ", worst in-band product " << worstInBand);

    // THE GATE.
    REQUIRE(cnpg::dsp::kDefaultPluckDistanceFromBridge01 <= 1.0f / 7.0f);
    REQUIRE(cnpg::dsp::kDefaultTapDistanceFromBridge01 <= 1.0f / 7.0f);
    REQUIRE(firstSharedNull == 0);
    REQUIRE(worstInBand > 0.15);

    // THE RED, EVALUATED BY THE SAME EXPRESSIONS ON THE GEOMETRY THIS REPLACED. Every clause of the
    // gate fails there, and the distance clause fails at the extreme value the criterion can take
    // rather than marginally: 1/2 is the largest distance from a termination any position can have,
    // i.e. onset 2, the lowest onset the slider admits.
    constexpr double kOldPosition = 0.5;
    const double oldDistance = 1.0 - kOldPosition;
    const double oldOnset = 1.0 / oldDistance;
    REQUIRE(oldOnset == 2.0);
    REQUIRE_FALSE(static_cast<float>(oldDistance) <= 1.0f / 7.0f);
    // Coincident combs, so the shared-null search finds the OCTAVE -- through the same lambda the
    // gate itself uses, not a hand-rolled restatement of the answer.
    REQUIRE(firstSharedNullAtOrBelow(oldDistance, oldDistance, 24) == 2);
    // ...and the product comb is not merely small at partials 2, 4 and 6, it is zero to machine
    // precision, because a coincident null is squared.
    double oldWorstInBand = 1.0e300;
    for (int n = 2; n <= 6; ++n)
        oldWorstInBand = std::min(oldWorstInBand, productComb(oldDistance, oldDistance, n));
    REQUIRE(oldWorstInBand < 1.0e-12);
    REQUIRE_FALSE(oldWorstInBand > 0.15);
}

// ---------------------------------------------------------------------------------------------
// 3. the gain staging
// ---------------------------------------------------------------------------------------------

TEST_CASE("CONTRACT: DefaultVoicing -- a six-string chord at full velocity keeps real headroom", "[contract]") {
    // TriodeStage.h promises that "the whole ~+16 dB multi-string summing budget still fits under
    // the SoftClipLimiter ceiling instead of being spent before the first chord", and
    // docs/listening/P2-20260803.md records that the six-string level was INFERRED from arithmetic
    // and never measured ("Record it as an inference if you record it at all"). This measures it,
    // on the assembled chain, at the shipping defaults -- and it is the gate that would have caught
    // the geometry change if the geometry change had cost level, which it did not.
    //
    // docs/bench/p2-exit.md 8.3 item 6 records the OTHER level gate as undischargeable, because
    // SoftClipLimiter is hard-wired last and the ceiling it is compared against is the asymptote
    // that limiter enforces -- so "peak <= ceiling" can never fail. This case asks the opposite
    // question, which can: not "did the limiter hold" but "is the instrument sitting ON it".
    constexpr double kMinHeadroomDb = 6.0;
    constexpr double kMinPeakDbfs = -20.0;

    const P1ChainParams shipping = cnpg::test::makeDefaultP1ChainParams();
    const double ceiling = static_cast<double>(shipping.limiter.ceilingDb);

    std::vector<Note> chord;
    for (int s = 0; s < 6; ++s)
        chord.push_back({0, s, kOpenStrings[static_cast<std::size_t>(s)], 1.0f});

    const Rendered r = render(shipping, chord, 3.0, 6);
    const double peak = peakDbfs(r.out);
    const double headroom = ceiling - peak;

    // The two RED arms, each on the same two readings, each failing a different clause: too hot
    // (the chord pinned against the limiter) and too quiet (headroom bought by being inaudible).
    P1ChainParams hot = shipping;
    hot.outputGain.gainDb = 24.0f;
    const double hotPeak = peakDbfs(render(hot, chord, 3.0, 6).out);

    P1ChainParams quiet = shipping;
    quiet.outputGain.gainDb = -24.0f;
    const double quietPeak = peakDbfs(render(quiet, chord, 3.0, 6).out);

    std::cout << "[contract] six-string open chord (EADGBE) at velocity 1.0 through the shipping chain: peak " << peak
              << " dBFS, " << headroom << " dB under the " << ceiling
              << " dB ceiling (limits: headroom >= " << kMinHeadroomDb << ", peak >= " << kMinPeakDbfs
              << "); RED arms read " << hotPeak << " dBFS (+24 dB "
              << "output gain) and " << quietPeak << " dBFS (-24 dB)\n";

    INFO("peak " << peak << " dBFS, headroom " << headroom << " dB");

    // THE GATE, both directions.
    REQUIRE(headroom >= kMinHeadroomDb);
    REQUIRE(peak >= kMinPeakDbfs);

    // THE RED. +24 dB of output gain drives the chord onto the ceiling, so the headroom clause
    // fails; -24 dB buys headroom by making the instrument inaudible, so the level clause fails.
    REQUIRE((ceiling - hotPeak) < kMinHeadroomDb);
    REQUIRE(quietPeak < kMinPeakDbfs);
}

// ---------------------------------------------------------------------------------------------
// the numbers the defaults were chosen from, kept re-derivable
// ---------------------------------------------------------------------------------------------

TEST_CASE("REPORT: voicing -- bracketing the comb-hole limit", "[.][report]") {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nworst EVEN-partial deficit over n in {2,4,6}: how far partial n sits below the mean of its\n"
                 "two ODD neighbours, chain output, one string, velocity 0.8\n";
    std::cout << "geometry             ";
    for (int midi : {40, 45, 50, 55, 59, 64, 36, 72})
        std::cout << "  MIDI " << midi;
    std::cout << "\n";
    struct Geom {
        const char* label;
        float exciter;
        float pickup;
    };
    for (const Geom& g : std::vector<Geom>{{"P2.9   0.5 / 0.5   ", 0.5f, 0.5f},
                                           {"SHIPPING 1-1/9 / 1-1/16", 1.0f - 1.0f / 9.0f, 1.0f - 1.0f / 16.0f},
                                           {"REFUSED  1-1/9 / 1-1/7 ", 1.0f - 1.0f / 9.0f, 1.0f - 1.0f / 7.0f}}) {
        std::cout << g.label;
        for (int midi : {40, 45, 50, 55, 59, 64, 36, 72}) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.network.exciter.defaultPosition = g.exciter;
            p.network.pickupPosition01 = g.pickup;
            const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);
            const std::vector<double> db = partialSpectrumDb(r.out, cnpg::test::midiNoteToHz(midi), 8);
            double worst = -300.0;
            for (int n = 2; n <= 6; n += 2)
                worst = std::max(worst, 0.5 * (db[static_cast<std::size_t>(n - 2)] + db[static_cast<std::size_t>(n)]) -
                                            db[static_cast<std::size_t>(n - 1)]);
            std::cout << std::setw(9) << worst;
        }
        std::cout << "\n";
    }

    std::cout << "\nsix-string open chord at the shipping defaults: peak dBFS and headroom to the ceiling\n";
    std::array<int, 6> open{40, 45, 50, 55, 59, 64};
    for (float velocity : {0.5f, 0.8f, 1.0f}) {
        for (float outputGain : {0.0f, 24.0f, -24.0f}) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.outputGain.gainDb = outputGain;
            std::vector<Note> chord;
            for (int s = 0; s < 6; ++s)
                chord.push_back({0, s, open[static_cast<std::size_t>(s)], velocity});
            const Rendered r = render(p, chord, 3.0, 6);
            const double peak = peakDbfs(r.out);
            std::cout << "  velocity " << std::setw(4) << velocity << "  outputGain " << std::setw(6) << outputGain
                      << " dB   peak " << std::setw(9) << peak << "   headroom " << std::setw(8) << (-0.3 - peak)
                      << "\n";
        }
    }
}

TEST_CASE("REPORT: voicing -- what the criterion costs in loudness", "[.][report]") {
    // The criterion (no null in [2, 6] at either end) forces a fundamental loss: coupling to the
    // fundamental is sin(pi*d) at BOTH ends, so onset >= 7 at both means at least
    // 2 * 20*log10(sin(pi/7)) = -14.6 dB of it, against 0 dB at the midpoint. This measures what
    // that actually does to a six-string chord's spectrum.
    std::cout << std::fixed << std::setprecision(2);
    struct Geom {
        const char* label;
        double de;
        double dp;
    };
    const std::vector<Geom> grid{
        {"P2.9  de 1/2  dp 1/2  (midpoint)  ", 0.5, 0.5},
        {"      de 1/8  dp 1/7  (min loss)  ", 1.0 / 8, 1.0 / 7},
        {"REFD  de 1/9  dp 1/7  (middle)    ", 1.0 / 9, 1.0 / 7},
        {"      de 1/9  dp 1/10             ", 1.0 / 9, 1.0 / 10},
        {"SHIP  de 1/9  dp 1/16 (bridge)    ", 1.0 / 9, 1.0 / 16},
    };
    std::array<int, 6> open{40, 45, 50, 55, 59, 64};
    std::cout << "six-string open chord, velocity 0.8, band energy over 0.3-3.0 s of the sustain (dB)\n";
    std::cout << "geometry                            fund.loss   80-200  200-500  500-1.5k  1.5k-4k   peak dBFS  "
                 "worst even deficit (MIDI 40)\n";
    for (const Geom& g : grid) {
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        p.network.exciter.defaultPosition = static_cast<float>(1.0 - g.de);
        p.network.pickupPosition01 = static_cast<float>(1.0 - g.dp);
        std::vector<Note> chord;
        for (int s = 0; s < 6; ++s)
            chord.push_back({0, s, open[static_cast<std::size_t>(s)], 0.8f});
        const Rendered r = render(p, chord, 4.0, 6);
        std::vector<double> sustain(r.out.begin() + static_cast<std::ptrdiff_t>(0.3 * kRate),
                                    r.out.begin() + static_cast<std::ptrdiff_t>(3.0 * kRate));
        const cnpg::test::Spectrum s = cnpg::test::computeSpectrum(sustain, kRate, 65536);
        auto band = [&](double lo, double hi) {
            double sum = 0.0;
            for (auto b = static_cast<std::size_t>(s.hzToBin(lo));
                 b <= static_cast<std::size_t>(s.hzToBin(hi)) && b < s.magnitudeSquared.size(); ++b)
                sum += s.magnitudeSquared[b];
            return sum > 0.0 ? 10.0 * std::log10(sum) : -300.0;
        };
        const double fundLoss =
            20.0 * std::log10(std::sin(3.14159265358979323846 * g.de) * std::sin(3.14159265358979323846 * g.dp));
        const Rendered one = render(p, {{0, 0, 40, 0.8f}}, 3.0, 1);
        std::cout << g.label << std::setw(10) << fundLoss << std::setw(9) << band(80.0, 200.0) << std::setw(9)
                  << band(200.0, 500.0) << std::setw(10) << band(500.0, 1500.0) << std::setw(9) << band(1500.0, 4000.0)
                  << std::setw(12) << peakDbfs(r.out) << std::setw(14)
                  << worstEvenPartialDeficitDb(one.out, cnpg::test::midiNoteToHz(40)) << "\n";
    }
}

TEST_CASE("REPORT: voicing -- the geometry comb over the exploration grid", "[.][report]") {
    // The grid the shortlist was cut from; the first row is the geometry that shipped through P2.9.
    std::cout << std::fixed;

    struct Geometry {
        const char* label;
        float exciter;
        float pickup;
    };
    const std::vector<Geometry> geometries{
        {"OLD DEFAULT 0.500 / 0.500", 0.5f, 0.5f},        {"exciter 0.500, tap 0.9375", 0.5f, 0.9375f},
        {"exciter 0.8375, tap 0.9375", 0.8375f, 0.9375f}, {"exciter 0.8375, tap 0.750 (neck)", 0.8375f, 0.750f},
        {"exciter 0.8750, tap 0.9375", 0.875f, 0.9375f},  {"exciter 0.9000, tap 0.9375", 0.900f, 0.9375f},
    };

    for (int midi : {40, 48, 55}) {
        const double f0 = cnpg::test::midiNoteToHz(midi);
        std::cout << "\n=== MIDI " << midi << " (" << std::setprecision(2) << f0
                  << " Hz), one string, raw tap, dB re partial 1 ===\n";
        std::cout << std::setprecision(1);
        std::cout << "geometry                            ";
        for (int n = 1; n <= 16; ++n)
            std::cout << std::setw(6) << n;
        std::cout << "\n";
        for (const Geometry& g : geometries) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.network.exciter.defaultPosition = g.exciter;
            p.network.pickupPosition01 = g.pickup;
            const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);
            const std::vector<double> db = partialSpectrumDb(r.tap, f0, 16);
            std::cout << std::setw(34) << std::left << g.label << std::right << "  ";
            for (int n = 1; n <= 16; ++n)
                std::cout << std::setw(6) << (db[static_cast<std::size_t>(n - 1)] - db[0]);
            std::cout << "\n";
        }
    }
}

TEST_CASE("REPORT: voicing -- the closed-form comb over the candidate grid", "[.][report]") {
    constexpr double kPi = 3.14159265358979323846;
    constexpr int kBand = 24;
    auto W = [&](double de, double dp, int n) {
        return std::fabs(std::sin(kPi * static_cast<double>(n) * de) * std::sin(kPi * static_cast<double>(n) * dp));
    };

    // The null set of a comb at a RATIONAL distance a/b (lowest terms) is exactly {b, 2b, 3b, ...}:
    // onset b, density 1/b. b = 2 is the smallest possible, so d = 1/2 is the unique worst point on
    // the whole slider -- and it is where BOTH defaults sit, which squares every null.
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n=== single comb: onset and density by distance from the nearer termination ===\n";
    std::cout << "  d        1/d   position01   inches @25.5\"   onset   density\n";
    for (int b : {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 20}) {
        const double d = 1.0 / static_cast<double>(b);
        std::cout << std::setprecision(4) << std::setw(7) << d << std::setprecision(1) << std::setw(7)
                  << static_cast<double>(b) << std::setprecision(4) << std::setw(13) << (1.0 - d)
                  << std::setprecision(2) << std::setw(15) << (25.5 * d) << std::setw(8) << b << std::setw(9)
                  << (100.0 / static_cast<double>(b)) << "%\n";
    }

    const std::vector<double> exciterD{0.5, 1.0 / 10, 1.0 / 9, 1.0 / 8, 1.0 / 7, 1.0 / 6, 1.0 / 5};
    const std::vector<double> pickupD{0.5, 1.0 / 16, 1.0 / 14, 1.0 / 11, 1.0 / 8, 1.0 / 7, 1.0 / 6, 1.0 / 4};

    std::cout << "\n=== the product comb |sin(n*pi*de) sin(n*pi*dp)| over n in [1, " << kBand << "] ===\n";
    std::cout << "de       dp       lowest hole (<-20dB)   holes in [2,24]   tilt W(peak<=12)/W(1) dB\n";
    for (double de : exciterD) {
        for (double dp : pickupD) {
            int lowest = 0;
            int holes = 0;
            double peak = 0.0;
            for (int n = 2; n <= kBand; ++n) {
                const double w = W(de, dp, n);
                if (w < 0.1) {
                    holes++;
                    if (lowest == 0)
                        lowest = n;
                }
                if (n <= 12)
                    peak = std::max(peak, w);
            }
            peak = std::max(peak, W(de, dp, 1));
            const double tilt = 20.0 * std::log10(peak / W(de, dp, 1));
            std::cout << std::setprecision(4) << std::setw(7) << de << std::setw(9) << dp << std::setw(18) << lowest
                      << std::setw(19) << holes << std::setprecision(1) << std::setw(22) << tilt << "\n";
        }
    }
}

TEST_CASE("REPORT: voicing -- candidate geometries measured through the chain", "[.][report]") {
    std::cout << std::fixed << std::setprecision(1);

    struct Candidate {
        double de;
        double dp;
    };
    const std::vector<Candidate> candidates{
        {0.5, 0.5},      // SHIPPING
        {0.5, 1.0 / 16}, // pickup fixed only
        {1.0 / 10, 0.5}, // exciter fixed only
        {1.0 / 10, 1.0 / 16}, {1.0 / 10, 1.0 / 14}, {1.0 / 9, 1.0 / 16}, {1.0 / 9, 1.0 / 14},
        {1.0 / 8, 1.0 / 16},  {1.0 / 7, 1.0 / 16},  {1.0 / 6, 1.0 / 16}, {1.0 / 10, 1.0 / 7},
        {1.0 / 9, 1.0 / 7},   {1.0 / 6, 1.0 / 11},  {1.0 / 10, 1.0 / 4},
    };

    // Band energies of the chain output over the sustain, as a share of total, plus level.
    auto bandDb = [](const cnpg::test::Spectrum& s, double loHz, double hiHz) {
        double sum = 0.0;
        const auto lo = static_cast<std::size_t>(std::max(1.0, s.hzToBin(loHz)));
        const auto hi = static_cast<std::size_t>(s.hzToBin(hiHz));
        for (std::size_t b = lo; b <= hi && b < s.magnitudeSquared.size(); ++b)
            sum += s.magnitudeSquared[b];
        return sum;
    };

    for (int midi : {40, 48}) {
        const double f0 = cnpg::test::midiNoteToHz(midi);
        std::cout << "\n=== MIDI " << midi << ", one string, vel 0.8, chain output ===\n";
        std::cout << "  de       dp     peak dBFS  RMS dBFS   worst notch  at n   band share %: 80-250 250-800 "
                     "800-2.5k 2.5k-8k\n";
        for (const Candidate& c : candidates) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.network.exciter.defaultPosition = static_cast<float>(1.0 - c.de);
            p.network.pickupPosition01 = static_cast<float>(1.0 - c.dp);
            const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);

            const std::vector<double> db = partialSpectrumDb(r.out, f0, 14);
            // Worst notch: how far partial n sits below the mean of its two neighbours.
            double worstNotch = 0.0;
            int atN = 0;
            for (int n = 2; n <= 13; ++n) {
                const double here = db[static_cast<std::size_t>(n - 1)];
                const double neighbours = 0.5 * (db[static_cast<std::size_t>(n - 2)] + db[static_cast<std::size_t>(n)]);
                if (neighbours - here > worstNotch) {
                    worstNotch = neighbours - here;
                    atN = n;
                }
            }

            std::vector<double> window(r.out.begin() + static_cast<std::ptrdiff_t>(0.2 * kRate), r.out.end());
            const cnpg::test::Spectrum s = cnpg::test::computeSpectrum(window, kRate, 65536);
            const double b1 = bandDb(s, 80.0, 250.0);
            const double b2 = bandDb(s, 250.0, 800.0);
            const double b3 = bandDb(s, 800.0, 2500.0);
            const double b4 = bandDb(s, 2500.0, 8000.0);
            const double tot = b1 + b2 + b3 + b4;

            std::cout << std::setprecision(4) << std::setw(8) << c.de << std::setw(9) << c.dp << std::setprecision(1)
                      << std::setw(10) << peakDbfs(r.out) << std::setw(10)
                      << rmsDbfs(r.out, static_cast<std::size_t>(0.2 * kRate), static_cast<std::size_t>(1.2 * kRate))
                      << std::setw(12) << worstNotch << std::setw(7) << atN << std::setw(12) << (100.0 * b1 / tot)
                      << std::setw(8) << (100.0 * b2 / tot) << std::setw(9) << (100.0 * b3 / tot) << std::setw(9)
                      << (100.0 * b4 / tot) << "\n";
        }
    }
}

TEST_CASE("REPORT: voicing -- the shortlist partial by partial", "[.][report]") {
    std::cout << std::fixed << std::setprecision(1);
    struct Candidate {
        const char* label;
        double de;
        double dp;
    };
    const std::vector<Candidate> shortlist{
        {"P2.9           de 1/2    dp 1/2   ", 0.5, 0.5},
        {"SHIPPING       de 1/9    dp 1/16  ", 1.0 / 9, 1.0 / 16},
        {"bridge pickup  de 1/10   dp 1/16  ", 1.0 / 10, 1.0 / 16},
        {"REFUSED        de 1/9    dp 1/7   ", 1.0 / 9, 1.0 / 7},
        {"middle-ish     de 1/9    dp 1/6   ", 1.0 / 9, 1.0 / 6},
        {"neck pickup    de 1/9    dp 1/4   ", 1.0 / 9, 1.0 / 4},
    };

    // MIDI 59 is here because it is where the REFUSED dp = 1/7 row breaks: its realised comb null
    // lands on partial 6 at this note, 21.72 dB below the mean of its odd neighbours, while every
    // other open string reads 0.6 to 9.6 dB. A table without the binding note is a table about the
    // easy cases. See "REPORT: voicing -- the realised comb onset is lower than 1/d".
    for (int midi : {40, 48, 52, 59}) {
        const double f0 = cnpg::test::midiNoteToHz(midi);
        std::cout << "\n=== MIDI " << midi << " chain output, dB re the LOUDEST partial ===\n";
        std::cout << "                                     ";
        for (int n = 1; n <= 14; ++n)
            std::cout << std::setw(6) << n;
        std::cout << "\n";
        for (const Candidate& c : shortlist) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.network.exciter.defaultPosition = static_cast<float>(1.0 - c.de);
            p.network.pickupPosition01 = static_cast<float>(1.0 - c.dp);
            const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);
            const std::vector<double> db = partialSpectrumDb(r.out, f0, 14);
            double best = -300.0;
            for (double v : db)
                best = std::max(best, v);
            std::cout << c.label << " ";
            for (int n = 1; n <= 14; ++n)
                std::cout << std::setw(6) << (db[static_cast<std::size_t>(n - 1)] - best);
            std::cout << "\n";
        }
    }

    std::cout << "\n=== level: single string MIDI 45 vel 1.0, and the six-string chord vel 0.8 ===\n";
    std::cout << "geometry                             1-string peak   chord peak   chord headroom to -0.3 dB\n";
    std::array<int, 6> open{40, 45, 50, 55, 59, 64};
    for (const Candidate& c : shortlist) {
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        p.network.exciter.defaultPosition = static_cast<float>(1.0 - c.de);
        p.network.pickupPosition01 = static_cast<float>(1.0 - c.dp);
        const Rendered one = render(p, {{0, 0, 45, 1.0f}}, 2.5, 1);
        std::vector<Note> chord;
        for (int s = 0; s < 6; ++s)
            chord.push_back({0, s, open[static_cast<std::size_t>(s)], 0.8f});
        const Rendered six = render(p, chord, 3.0, 6);
        const double chordPeak = peakDbfs(six.out);
        std::cout << c.label << std::setw(12) << peakDbfs(one.out) << std::setw(14) << chordPeak << std::setw(20)
                  << (-0.3 - chordPeak) << "\n";
    }
}

TEST_CASE("REPORT: voicing -- re-deriving the pickup trim at the new geometry", "[.][report]") {
    // PickupTap.h's kNominalPickupTrimDb is DEFINED by one scenario -- StringNetwork -> PickupTap,
    // every other parameter at its default, MIDI 45 at velocity 1.0, 2 s render -- and the shipping
    // geometry is one of those "other parameters". Moving the tap moves the measurement, so the
    // constant has to be re-derived rather than inherited.
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n rate     raw peak (trim 0)   trim needed for -18 dBFS\n";
    for (double rate : {44100.0, 48000.0, 96000.0}) {
        constexpr int kB = 128;
        cnpg::dsp::StringNetwork<float> network;
        network.prepare(rate, kB, cnpg::dsp::FractionalDelayKind::Lagrange3);
        network.setNumStrings(1);
        network.setParams(cnpg::dsp::StringNetworkParams{});
        network.reset();

        cnpg::dsp::PickupTap pickup;
        pickup.prepare(rate, kB);
        cnpg::dsp::PickupTapParams pp;
        pp.outputGainDb = 0.0f;
        pickup.setParams(pp);
        pickup.reset();

        BlockEventQueue events;
        events.push(noteOn(0, 0, 45, 1.0f, cnpg::dsp::kUnspecifiedNoteParam));

        std::vector<cnpg::dsp::Sample> out(static_cast<std::size_t>(kB), 0.0f);
        double peak = 0.0;
        for (int rendered = 0; rendered < static_cast<int>(2.0 * rate); rendered += kB) {
            network.process(events, kB);
            pickup.process(network.tapBuffers(), out.data(), kB);
            for (int n = 0; n < kB; ++n)
                peak = std::max(peak, std::fabs(static_cast<double>(out[static_cast<std::size_t>(n)])));
        }
        const double rawDb = 20.0 * std::log10(peak);
        std::cout << std::setw(7) << rate << std::setw(18) << rawDb << std::setw(26) << (-18.0 - rawDb) << "\n";
    }
    std::cout << "  (shipping constant: " << cnpg::dsp::kNominalPickupTrimDb << " dB)\n";
}

TEST_CASE("REPORT: voicing -- string material decay across the register", "[.][report]") {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nT60 of the fundamental at the shipping material (lossLow 0.5 lossHigh 0.5 dispersion 0),\n"
                 "one string, no note-off, 48 kHz -- and what a real electric guitar does.\n";
    std::cout << " MIDI    f0 Hz    T60 s\n";
    for (int midi : {28, 40, 45, 52, 60, 72, 84, 96}) {
        const double f0 = cnpg::test::midiNoteToHz(midi);
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 12.0, 1);
        const std::vector<double> env = cnpg::test::partialEnvelope(r.tap, kRate, f0, 8.0);
        const double t60 = cnpg::test::partialT60Seconds(env, kRate, static_cast<std::size_t>(0.05 * kRate));
        std::cout << std::setw(5) << midi << std::setw(10) << f0 << std::setw(9) << t60 << "\n";
    }
}

TEST_CASE("REPORT: voicing -- coupling and the unison stack", "[.][report]") {
    // ADR 0007 D7.0/D7.1 measure criterion (4) -- "near-unison strings ~25 cents apart do not
    // involuntarily mode-lock". This re-measures it AT BOTH GEOMETRIES using the protocol
    // tests/dsp/StringNetworkScaleTests.cpp already uses, so the only thing differing between the
    // two arms is the geometry: sustain material (lossLow = lossHigh = 1.0, because the estimator
    // analyses 2^18 samples starting 0.5 s after the pluck and the DEFAULT material has nothing
    // left in that window), two strings at MIDI 45, string 1 offset +25 cents, both plucked, 7 s,
    // raw taps, peak search +/-80 cents around each string's OWN nominal.
    std::cout << std::fixed << std::setprecision(3);

    struct Geom {
        const char* label;
        float exciter;
        float pickup;
    };
    const std::vector<Geom> geometries{
        {"P2.9     0.5 / 0.5   ", 0.5f, 0.5f},
        {"SHIPPING 1-1/9 / 1-1/16", 1.0f - 1.0f / 9.0f, 1.0f - 1.0f / 16.0f},
        {"REFUSED  1-1/9 / 1-1/7 ", 1.0f - 1.0f / 9.0f, 1.0f - 1.0f / 7.0f},
    };

    for (bool sustain : {true, false}) {
        std::cout << "\n=== " << (sustain ? "SUSTAIN material (D7.0's protocol)" : "DEFAULT material")
                  << ": two strings 25 cents apart ===\n";
        std::cout << "geometry              coupling   string 0 Hz   string 1 Hz   separation   pull on s0\n";
        for (const Geom& g : geometries) {
            // Finer than D7.0's ladder between 0.25 and 0.30, because that interval is where the
            // isolated pair's criterion-(4) boundary lives and the shipping default now sits below
            // it: a bracket two steps wide is not evidence about where the edge is.
            for (float coupling : {0.0f, 0.10f, 0.20f, 0.22f, 0.25f, 0.27f, 0.28f, 0.30f, 0.32f, 0.35f}) {
                cnpg::dsp::StringNetworkParams params;
                params.exciter.defaultPosition = g.exciter;
                params.pickupPosition01 = g.pickup;
                params.exciter.noiseAmount = 0.0f;
                if (sustain) {
                    params.stringMaterial.lossGainLow = 1.0f;
                    params.stringMaterial.lossGainHigh = 1.0f;
                }
                params.perString[1].tuningOffsetCents = 25.0f;
                params.bridge.couplingStrength = coupling;

                cnpg::dsp::StringNetwork<float> network;
                network.prepare(kRate, kBlock, cnpg::dsp::FractionalDelayKind::Lagrange3);
                network.setNumStrings(2);
                network.setParams(params);
                network.reset();

                BlockEventQueue events;
                events.push(noteOn(0, 0, 45, 0.8f, cnpg::dsp::kUnspecifiedNoteParam));
                events.push(noteOn(0, 1, 45, 0.8f, cnpg::dsp::kUnspecifiedNoteParam));

                constexpr std::size_t kLen = std::size_t{1} << 18;
                const auto discard = static_cast<std::size_t>(0.5 * kRate);
                std::vector<double> a;
                std::vector<double> b;
                a.reserve(kLen);
                b.reserve(kLen);
                std::size_t rendered = 0;
                while (rendered < static_cast<std::size_t>(7.0 * kRate) && a.size() < kLen) {
                    network.process(events, kBlock);
                    const float* ca = network.tapBuffers().channel(0, 0);
                    const float* cb = network.tapBuffers().channel(1, 0);
                    for (int n = 0; n < kBlock; ++n, ++rendered) {
                        if (rendered < discard || a.size() >= kLen)
                            continue;
                        a.push_back(ca != nullptr ? static_cast<double>(ca[n]) : 0.0);
                        b.push_back(cb != nullptr ? static_cast<double>(cb[n]) : 0.0);
                    }
                }
                const double nom0 = cnpg::test::midiNoteToHz(45);
                const double nom1 = nom0 * std::exp2(25.0 / 1200.0);
                const double f0 = cnpg::test::findPeakHz(cnpg::test::computeSpectrum(a, kRate, kLen), nom0,
                                                         cnpg::test::kTuningSearchCents);
                const double f1 = cnpg::test::findPeakHz(cnpg::test::computeSpectrum(b, kRate, kLen), nom1,
                                                         cnpg::test::kTuningSearchCents);
                std::cout << g.label << std::setw(10) << coupling << std::setw(14) << f0 << std::setw(14) << f1
                          << std::setw(13) << cnpg::test::centsBetween(f1, f0) << std::setw(13)
                          << cnpg::test::centsBetween(f0, nom0) << "\n";
            }
        }
    }

    std::cout << "\n=== the author's own case: SIX simultaneous C3s on the six-string instrument ===\n";
    std::cout << " coupling   peak dBFS   RMS 0-1s   RMS 1-3s   headroom to -0.3 dB\n";
    for (float coupling : {0.0f, 0.10f, 0.20f, 0.30f, 0.35f}) {
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        p.network.bridge.couplingStrength = coupling;
        std::vector<Note> stack;
        for (int s = 0; s < 6; ++s)
            stack.push_back({static_cast<int>(0.003 * kRate) * s, s, 48, 0.8f});
        const Rendered r = render(p, stack, 4.0, 6);
        const double peak = peakDbfs(r.out);
        std::cout << std::setw(9) << coupling << std::setw(12) << peak << std::setw(11)
                  << rmsDbfs(r.out, 0, static_cast<std::size_t>(1.0 * kRate)) << std::setw(11)
                  << rmsDbfs(r.out, static_cast<std::size_t>(1.0 * kRate), static_cast<std::size_t>(3.0 * kRate))
                  << std::setw(22) << (-0.3 - peak) << "\n";
    }
}

TEST_CASE("REPORT: voicing -- the chain tilt at a null-free geometry", "[.][report]") {
    std::cout << std::fixed << std::setprecision(1);
    // A geometry with no null at or below partial 24, so what is measured is the CHAIN's rolloff
    // (pluck 1/n^2, loop filter, pickup bandpass, cab) and not the comb.
    for (int midi : {40, 48, 55, 64}) {
        const double f0 = cnpg::test::midiNoteToHz(midi);
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        p.network.exciter.defaultPosition = 1.0f - 1.0f / 29.0f;
        p.network.pickupPosition01 = 1.0f - 1.0f / 31.0f;
        const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);
        const std::vector<double> tapDb = partialSpectrumDb(r.tap, f0, 24);
        const std::vector<double> outDb = partialSpectrumDb(r.out, f0, 24);
        std::cout << "\nMIDI " << midi << " f0 " << std::setprecision(2) << f0 << " Hz\n" << std::setprecision(1);
        std::cout << "  n            ";
        for (int n = 1; n <= 24; ++n)
            std::cout << std::setw(6) << n;
        std::cout << "\n  tap  dB re 1";
        for (int n = 1; n <= 24; ++n)
            std::cout << std::setw(6) << (tapDb[static_cast<std::size_t>(n - 1)] - tapDb[0]);
        std::cout << "\n  out  dB re 1";
        for (int n = 1; n <= 24; ++n)
            std::cout << std::setw(6) << (outDb[static_cast<std::size_t>(n - 1)] - outDb[0]);
        std::cout << "\n";
        // Where the chain output falls 20 dB below the fundamental, and where below its own max.
        double best = -300.0;
        for (double v : outDb)
            best = std::max(best, v);
        int n20 = 0;
        for (int n = 2; n <= 24; ++n)
            if (outDb[static_cast<std::size_t>(n - 1)] >= best - 20.0)
                n20 = n;
        std::cout << "  highest partial within 20 dB of the loudest: n = " << n20 << "\n";
    }
}

TEST_CASE("REPORT: voicing -- gain staging one string to six", "[.][report]") {
    std::cout << std::fixed << std::setprecision(2);
    const P1ChainParams base = cnpg::test::makeDefaultP1ChainParams();

    struct Case {
        const char* label;
        std::vector<Note> notes;
        int numStrings;
    };
    // EADGBE, struck together, and struck as a strum (12 ms apart, downstroke).
    std::vector<Note> chord;
    std::vector<Note> strum;
    const std::array<int, 6> open{40, 45, 50, 55, 59, 64};
    for (int s = 0; s < 6; ++s) {
        chord.push_back({0, s, open[static_cast<std::size_t>(s)], 0.8f});
        strum.push_back({static_cast<int>(0.012 * kRate) * s, s, open[static_cast<std::size_t>(s)], 0.8f});
    }

    const std::vector<Case> cases{
        {"one string, MIDI 45, vel 0.8", {{0, 0, 45, 0.8f}}, 1},
        {"one string, MIDI 45, vel 1.0", {{0, 0, 45, 1.0f}}, 1},
        {"six-string chord, vel 0.8", chord, 6},
        {"six-string chord, vel 1.0",
         [&] {
             std::vector<Note> c = chord;
             for (Note& n : c)
                 n.velocity = 1.0f;
             return c;
         }(),
         6},
        {"six-string strum, vel 0.8", strum, 6},
    };

    std::cout << "                                     peak dBFS   RMS(0-1s)   RMS(1-3s)   ceiling headroom\n";
    for (const Case& c : cases) {
        const Rendered r = render(base, c.notes, 4.0, c.numStrings);
        const double peak = peakDbfs(r.out);
        std::cout << std::setw(36) << std::left << c.label << std::right << std::setw(9) << peak << std::setw(12)
                  << rmsDbfs(r.out, 0, static_cast<std::size_t>(1.0 * kRate)) << std::setw(12)
                  << rmsDbfs(r.out, static_cast<std::size_t>(1.0 * kRate), static_cast<std::size_t>(3.0 * kRate))
                  << std::setw(14) << (base.limiter.ceilingDb - peak) << "\n";
    }
}

TEST_CASE("REPORT: voicing -- exciter noise", "[.][report]") {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "noise   peak dBFS   attack RMS(0-20ms)   body RMS(0.2-1.2s)   attack-to-body dB\n";
    for (float noise : {0.0f, 0.05f, 0.10f, 0.15f, 0.25f, 0.50f}) {
        P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
        p.network.exciter.noiseAmount = noise;
        const Rendered r = render(p, {{0, 0, 45, 0.8f}}, 2.0, 1);
        const double attack = rmsDbfs(r.out, 0, static_cast<std::size_t>(0.020 * kRate));
        const double body =
            rmsDbfs(r.out, static_cast<std::size_t>(0.2 * kRate), static_cast<std::size_t>(1.2 * kRate));
        std::cout << std::setw(5) << noise << std::setw(12) << peakDbfs(r.out) << std::setw(20) << attack
                  << std::setw(21) << body << std::setw(18) << (attack - body) << "\n";
    }

    // Determinism: two renders of the same events must be bit-identical.
    P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
    p.network.exciter.noiseAmount = 0.15f;
    const Rendered a = render(p, {{0, 0, 45, 0.8f}, {24000, 0, 52, 0.7f}}, 2.0, 1);
    const Rendered b = render(p, {{0, 0, 45, 0.8f}, {24000, 0, 52, 0.7f}}, 2.0, 1);
    double worst = 0.0;
    for (std::size_t i = 0; i < a.out.size(); ++i)
        worst = std::max(worst, std::fabs(a.out[i] - b.out[i]));
    std::cout << "determinism at noise 0.15: worst |diff| between two renders = " << worst << "\n";

    // What the noise actually BUYS: how much of the string's upper series the excitation reaches.
    // Measured over the note's first 200 ms -- the attack, which is where a pick's broadband
    // content lives -- at MIDI 45, as dB re partial 1.
    std::cout << "\nattack spectrum (first 200 ms, MIDI 45, chain output), dB re partial 1\n";
    std::cout << "noise ";
    for (int n = 1; n <= 16; ++n)
        std::cout << std::setw(7) << n;
    std::cout << "\n";
    for (float noise : {0.0f, 0.05f, 0.10f, 0.25f}) {
        P1ChainParams np = cnpg::test::makeDefaultP1ChainParams();
        np.network.exciter.noiseAmount = noise;
        const Rendered r = render(np, {{0, 0, 45, 0.8f}}, 1.0, 1);
        std::vector<double> attack(r.out.begin(), r.out.begin() + static_cast<std::ptrdiff_t>(0.200 * kRate));
        const cnpg::test::Spectrum s = cnpg::test::computeSpectrum(attack, kRate, 8192);
        const double f0 = cnpg::test::midiNoteToHz(45);
        const double ref = partialDb(s, f0, 60.0);
        std::cout << std::setw(5) << noise << " ";
        for (int n = 1; n <= 16; ++n)
            std::cout << std::setw(7) << (partialDb(s, f0 * n, 60.0) - ref);
        std::cout << "\n";
    }
}

// ---------------------------------------------------------------------------------------------
// THE REALISED ONSET: why d <= 1/7 is NECESSARY BUT NOT SUFFICIENT in this waveguide
// ---------------------------------------------------------------------------------------------

TEST_CASE("REPORT: voicing -- the realised comb onset is lower than 1/d, and by how much", "[.][report]") {
    // *** THIS CASE IS THE EVIDENCE FOR A REFUSAL (2026-08-05). *** The tap was proposed to move
    // from 1 - 1/16 to 1 - 1/7 on the strength of this file's own closed form, which says a tap
    // 1/7 of the string from the bridge has its first null at partial 7 and therefore clears the
    // identity band [2, 6] -- with, in the previous task's own words, ZERO MARGIN.
    //
    // THE CLOSED FORM IS A CONTINUOUS-STRING IDENTITY AND THIS INSTRUMENT IS A DISCRETE WAVEGUIDE.
    // WaveguideString reads a tap at delay `1.0 + position01 * positionSpan_`
    // (dsp/include/cnpg/dsp/WaveguideString.h), and `positionSpan_` is ONE RAIL's realized span --
    // it excludes the loss, dispersion, seam and bridge phase delays, which are part of the
    // acoustic loop but not of the rail. So the tap's acoustic distance from the bridge is
    //
    //     (1 + (1 - p) * S) samples   out of a half-loop of   (S + tau/2) samples
    //
    // rather than (1 - p) of it, and the realised onset is therefore LOWER than 1/d by an amount
    // that is an ABSOLUTE offset of about one sample. An absolute offset is a bigger fraction of a
    // shorter loop, so the error grows toward the top of the register and toward the LOWEST sample
    // rate -- which is exactly what the second table below measures, and it is what says the effect
    // is the discretisation and not the comb (the closed form contains no sample rate at all).
    //
    // MEASURED CONSEQUENCE. At 1/7 the realised null lands ON PARTIAL 6 -- inside the identity band
    // -- on the open B string (MIDI 59), reading 21.72 dB of even-partial deficit through the chain
    // against this file's 18 dB gate, and 24.77 dB on the raw tap at 44.1 kHz. At 96 kHz, where the
    // loop is twice as long and the one-sample offset is half the fraction, the same note reads
    // 5.58 dB. A geometry whose in-band spectrum depends on the sample rate that much is not a
    // geometry that satisfies the criterion.
    //
    // WHAT THE MEASUREMENT SUPPORTS INSTEAD: 1/8. It is the largest tap distance that is clean at
    // all three supported rates (6.23 / 5.54 / 5.91 dB), where 1/7.5 already reads 18.76 dB at
    // 44.1 kHz. The margin the closed form needs here is about ONE PARTIAL, not zero -- which is
    // the same conclusion PluckExciter.h reached for the PLUCK when it declined the zero-margin 1/7
    // in favour of 1/9, and the reason the pluck default is unaffected by any of this.
    std::cout << std::fixed << std::setprecision(2);
    const std::vector<double> taps{1.0 / 6.0, 1.0 / 7.0,  1.0 / 7.5,  1.0 / 8.0,
                                   1.0 / 9.0, 1.0 / 10.0, 1.0 / 12.0, 1.0 / 16.0};

    std::cout << "\nCHAIN, 48 kHz, six open strings, pluck at its shipping 1/9: worst even-partial deficit\n"
                 "(this is the statistic the [contract] gate above uses, and its limit is 18 dB)\n";
    for (double dp : taps) {
        double worst = -300.0;
        int worstNote = 0;
        for (int midi : kOpenStrings) {
            P1ChainParams p = cnpg::test::makeDefaultP1ChainParams();
            p.network.pickupPosition01 = static_cast<float>(1.0 - dp);
            const Rendered r = render(p, {{0, 0, midi, 0.8f}}, 3.0, 1);
            const double d = worstEvenPartialDeficitDb(r.out, cnpg::test::midiNoteToHz(midi));
            if (d > worst) {
                worst = d;
                worstNote = midi;
            }
        }
        std::cout << "  tap 1/" << std::setw(6) << (1.0 / dp) << "   worst " << std::setw(8) << worst << " dB at MIDI "
                  << worstNote << "\n";
    }

    std::cout << "\nRAW TAP (no chain, so no pickup resonance and no triode in it), six open strings,\n"
                 "worst even-partial deficit at each supported rate -- the rate dependence IS the finding\n"
                 "    tap        44.1 kHz   48 kHz    96 kHz\n";
    for (double dp : taps) {
        std::cout << "  tap 1/" << std::setw(6) << (1.0 / dp);
        for (double rate : {44100.0, 48000.0, 96000.0}) {
            double worst = -300.0;
            for (int midi : kOpenStrings) {
                cnpg::dsp::StringNetwork<float> network;
                network.prepare(rate, kBlock, cnpg::dsp::FractionalDelayKind::Lagrange3);
                network.setNumStrings(1);
                cnpg::dsp::StringNetworkParams np;
                np.pickupPosition01 = static_cast<float>(1.0 - dp);
                network.setParams(np);
                network.reset();
                BlockEventQueue events;
                events.push(noteOn(0, 0, midi, 0.8f, cnpg::dsp::kUnspecifiedNoteParam));
                std::vector<double> tap;
                for (int k = 0; k < static_cast<int>(3.0 * rate); k += kBlock) {
                    network.process(events, kBlock);
                    const float* c = network.tapBuffers().channel(0, 0);
                    for (int n = 0; n < kBlock; ++n)
                        tap.push_back(c != nullptr ? static_cast<double>(c[n]) : 0.0);
                }
                const double f0 = cnpg::test::midiNoteToHz(midi);
                const auto skip = static_cast<std::size_t>(0.20 * rate);
                std::vector<double> w(tap.begin() + static_cast<std::ptrdiff_t>(skip), tap.end());
                const cnpg::test::Spectrum spec = cnpg::test::computeSpectrum(w, rate, 65536);
                double db[8];
                for (int n = 1; n <= 8; ++n)
                    db[static_cast<std::size_t>(n - 1)] = partialDb(spec, f0 * n);
                for (int n = 2; n <= 6; n += 2)
                    worst =
                        std::max(worst, 0.5 * (db[static_cast<std::size_t>(n - 2)] + db[static_cast<std::size_t>(n)]) -
                                            db[static_cast<std::size_t>(n - 1)]);
            }
            std::cout << std::setw(10) << worst;
        }
        std::cout << "\n";
    }
}

#pragma once

#include <type_traits>

#include "cnpg/dsp/Common.h"

// IBridgePort -- see docs/plan.md section 2.6. The single port-level seam shared by the primary
// bidirectional bridge (BridgeJunction, Task P2.4) and the fallback bus (SympatheticResonatorBus,
// Task P2.5 only if the passivity timebox triggers): StringNetwork holds exactly one
// IBridgePort<SampleT>& and cannot tell them apart. Wave variables are power-normalized against
// the per-port impedances supplied at prepare (WaveguideString::portImpedance()).
//
// This header also carries the trivial passive reflective termination StringNetwork runs against
// until BridgeJunction lands, exactly as the section-2.6 header split specifies
// ("IBridgePort.h (also carrying the trivial passive reflective termination)"). BridgeJunction.h
// and SympatheticResonatorBus.h are separate headers and arrive with their tasks.
//
// BridgeAdmittanceParams lives here rather than in BridgeJunction.h because StringNetworkParams
// carries one (docs/plan.md section 2.7) and StringNetwork.h must therefore see the type in P1,
// before BridgeJunction.h exists. Zero JUCE includes.

namespace cnpg::dsp {

struct BridgeAdmittanceParams { // tunable positive-real 2nd-order load (P2.4)
    // Load resonance. Validated into [kBridgeMinResonanceHz, kBridgeResonanceNyquistFraction * fs]
    // by BridgeJunction::setAdmittance. 180 Hz is a plausible low body/bridge resonance for a
    // guitar-shaped thing and sits where the instrument's fundamentals actually are, which is what
    // makes the coupling audible rather than theoretical.
    float resonanceHz = 180.0f;
    // Damping RATIO zeta of that resonance, validated onto a STRICTLY POSITIVE floor
    // (kBridgeMinDamping) -- the positive-real constraint itself, since zeta = 0 puts the poles on
    // the imaginary axis and makes Re Y identically zero. 0.5 is Q = 1, i.e. deliberately broad:
    // a real bridge has many modes and P2 ships one, so a narrow one would couple nothing except
    // the handful of partials that happened to land on it.
    float damping = 0.5f;
    // 0..1; scales the FULL load admittance (conductance AND susceptance): 0 = strings fully
    // decoupled AND bridgeOutput() == 0 (the P3 body feed requires > 0).
    //
    // NONZERO BY DECISION, Task P2.4 (docs/decisions/0004-phase2-vision-decisions.md amendment 3,
    // closed by docs/decisions/0006-p2-bridge-passivity-fallback.md). Through P1 this was 0
    // because P1's termination was rigid and there was no load to couple into. Leaving it at 0
    // once BridgeJunction exists would be a silent kill switch: the strings would stay
    // independent, bridgeOutput() would stay identically zero, and every later body/chamber/pickup
    // -feed feature would be disabled by a default rather than by a decision. The value is
    // measured, not guessed -- see the ADR for the sympathetic-response, beat-rate, T60 and
    // tuning-shift numbers behind it.
    //
    // ---- 0.35 -> 0.20, AND HOW THIS WAS SETTLED ------------------------------------------------
    //
    // *** SETTLED BY AUTHOR DELEGATION ON 2026-08-05. NOT SETTLED BY EAR. *** ADR 0007 D4 reserves
    // this value for a recorded listening sign-off and reserved it across three separate rulings.
    // THAT SIGN-OFF WAS NEVER PERFORMED: docs/listening/P2-20260803.md is still marked NOT
    // PERFORMED with every verdict field blank, and nothing here fills it. D4's condition was
    // WAIVED, not met, and the distinction is the point of this paragraph -- a later reader must
    // not be able to mistake this for a value an ear confirmed.
    //
    // WHAT THE MEASUREMENT SAYS, which is why 0.20 and not some other waiver. ADR 0007 D5's
    // criterion (4) is that near-unison strings ~25 cents apart must not involuntarily mode-lock,
    // and it is measured on two topologies:
    //
    //   isolated pair, MIDI 45, +25 cents on one string (D7.0): separation 25.045 cents at 0.20 and
    //     25.099 at 0.25, then 0.003 cents at 0.30 and 0.35 -- a total collapse, with the whole
    //     25 cents landing as a +25.0-cent pull on the string NOBODY detuned;
    //   shipping six strings on one bridge (D7.1): the detuned partner is absorbed progressively
    //     -- -0.3 dB at 0.00, -1.8 at 0.10, -6.8 at 0.20 -- and is GONE at 0.30 and above.
    //     Separation 23.612 cents at 0.20; ONE PEAK at 0.30.
    //
    // 0.20 is the LARGEST MEASURED POINT at which criterion (4) holds on the topology that ships.
    // It is not an interpolation: the boundary lies somewhere in (0.20, 0.30) and no point inside
    // that interval has been measured on six strings, so the value is placed at the last reading
    // that passes rather than at a guessed edge. The margin is therefore ZERO in the only direction
    // that matters, and D7.1's own question -- whether a partner 6.8 dB down is still the chord
    // that was played -- is still a listening question and is still unanswered.
    //
    // WHAT IT COSTS. Coupling IS the mechanism of sympathetic resonance: less of it means less of
    // one string in the others, and it is the feed ADR 0004's later body/chamber rides on. What it
    // buys back, measured: on a six-C3 unison stack the 1-3 s tail is 8.3 dB LOUDER at 0.20 than at
    // 0.35 (-80.4 vs -88.7 dBFS), and ADR 0006's beat depth runs the same way (10.08 dB at 0.10
    // against 3.59 at 0.35), so on the evidence that exists the sympathetic character does not
    // obviously degrade here. That is one measurement of one phenomenon and it is not the ear.
    //
    // THE DECLARED NORMAL RANGE IS NOT MOVED BY THIS. kBridgeNormalCouplingMax stays at 0.35: it is
    // the criterion-(1) (+/-2 cent tuning) boundary, it is still measured to hold there, and ADR
    // 0007 D7/D7.0 now record what this default's move does and does not change about it.
    float couplingStrength = 0.20f;
};

static_assert(std::is_trivially_copyable_v<BridgeAdmittanceParams>,
              "BridgeAdmittanceParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Per docs/plan.md section 2.1, every sample-domain type is template <typename SampleT>: the port
// seam must match the SampleT of the StringNetwork instantiation that holds it, so the realtime
// float path and the tier-2 [energy] double path each get their own port.
template <typename SampleT> class IBridgePort {
  public:
    virtual ~IBridgePort() = default;

    // Message thread; may allocate. portImpedances: one reference impedance per port (per
    // string), length numPorts.
    virtual void prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) = 0;

    // Realtime-safe; clears every state-bearing element.
    virtual void reset() noexcept = 0;

    // Per-sample exchange, called from inside the StringNetwork loop: one incident wave in, one
    // outgoing wave out, per port. Realtime-safe; never allocates.
    virtual void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept = 0;

    // Mono bridge signal for the pickup/body chain; identically 0 when the load is fully
    // decoupled (couplingStrength == 0), which includes a rigid termination.
    virtual SampleT bridgeOutput() const noexcept = 0;

    // Tier-2 energy test support: makes the load lossless.
    virtual void setLossBypassed(bool bypass) noexcept = 0;

    // The load's tuning surface. Hoisted here from the two implementations (docs/plan.md section
    // 2.6 declares an identical setAdmittance on BridgeJunction and on SympatheticResonatorBus,
    // and calls the latter's "same tuning surface as BridgeJunction") because StringNetworkParams
    // CARRIES a BridgeAdmittanceParams (section 2.7) while StringNetwork holds only this base and
    // "cannot tell implementations apart". Without it on the interface the parameter has no route
    // from the network's surface to the port, and every caller would have to know which
    // implementation it attached -- exactly the fail-open wiring this seam is supposed to remove.
    // Realtime-safe; implementations clamp into their own passive region at set time.
    virtual void setAdmittance(const BridgeAdmittanceParams& p) noexcept = 0;

    // True when the port stores no energy, so zero incident waves imply zero outgoing waves for
    // ever. StringNetwork's idle-string skip predicate needs it: once the port's reflected waves
    // are routed back into the strings, a string with no state of its own can still be driven
    // through the bridge, and "there is nothing to do" stops being a purely per-string question.
    virtual bool isQuiescent() const noexcept = 0;

    // The port's own contribution to the discrete Lyapunov storage functional, in the same units
    // as WaveguideString::energyEstimate(). Summed into StringNetwork::energyEstimate(), which is
    // what the tier-2 [energy] bound is asserted on: a junction whose stored energy is left out of
    // the functional makes the functional fluctuate by however much energy is sloshing in and out
    // of it, which is orders of magnitude above the 1e-9 bound.
    virtual Sample64 storageEnergy() const noexcept = 0;

    // ---- THE TUNING SURFACE (Task P2.7, docs/decisions/0007) -----------------------------------
    //
    // The PHASE delay, in samples, that this port's own self-reflectance adds to the round-trip
    // loop of the string on `portIndex`, evaluated at `frequencyHz`, when `numPorts` string ports
    // are loading the junction. Sign convention: POSITIVE lengthens the loop, i.e. the string sings
    // FLAT by that many samples of loop period.
    //
    // PHASE delay, never group delay -- the P1.4 ruling (dsp/include/cnpg/dsp/WaveguideString.h's
    // "TUNING" header), applied one element further down the loop. A resonance of the loop is where
    // the round-trip PHASE is a multiple of 2*pi, so the quantity that decides pitch is
    // -arg(H(e^jw))/w and nothing else; group delay is a different number and using it misses the
    // gate. This is what replaces P2.7's originally planned per-MIDI-note calibration table, which
    // ADR 0007 withdrew: the residual is a function of three LIVE parameters and reverses sign
    // across the resonance, and a one-dimensional note-indexed table structurally cannot carry that.
    //
    // WHY IT IS ON THE INTERFACE AND NOT ON BridgeJunction. StringNetwork holds exactly one
    // IBridgePort& and "cannot tell implementations apart" -- the same argument that put
    // setAdmittance here. A network that had to know which implementation it was holding in order
    // to tune its strings would be the fail-open wiring this seam exists to remove, and the
    // fallback bus of Q17 would silently ship a detuned instrument.
    //
    // NOT realtime-path work. StringNetwork calls this on parameter change and on note change --
    // O(1) per string per event -- never per sample, per ADR 0007 D6 ("the fixed point runs outside
    // the audio path"). It must be noexcept, must not allocate, and must return a value that is
    // finite or the solver's fallback takes over (see BridgeTuning.h).
    //
    // Returning 0 is the honest answer for any port whose reflectance is a real constant (a rigid
    // -1, or any frequency-independent scaling of it): a real reflectance has zero phase.
    virtual double reflectionPhaseDelaySamples(int portIndex, double frequencyHz, int numPorts) const noexcept = 0;
};

// The trivial passive reflective termination P1 runs against: a rigid, lossless, inverting
// bridge. Every port reflects its own incident wave with r = -1 and no port sees any other, so
// the strings are fully decoupled -- the couplingStrength == 0 limit of BridgeJunction, and the
// same reflection WaveguideString::tick() applies internally when no external wave is accepted.
//
// P1 therefore has no audible bridge load: bridgeOutput() is identically 0 (a rigid termination
// stores and dissipates nothing, so there is no load velocity to feed a body node). Task P2.4
// replaces this with the loaded N-port BridgeJunction, at which point the port's outgoing waves
// become the strings' actual bridge reflection and bridgeOutput() becomes a real signal.
//
// Header-only: there is nothing here worth a translation unit, and no explicit instantiation is
// needed (unlike BridgeJunction, whose bodies live in its own .cpp).
template <typename SampleT> class RigidBridgeTermination final : public IBridgePort<SampleT> {
  public:
    void prepare(double sampleRate, int maxBlockSize, int numPorts, const float* portImpedances) override {
        // Nothing to allocate and nothing impedance-dependent: r = -1 holds at every impedance,
        // which is exactly what makes this the trivial termination.
        (void)sampleRate;
        (void)maxBlockSize;
        (void)numPorts;
        (void)portImpedances;
    }

    void reset() noexcept override {} // stateless

    void scatter(const SampleT* incident, SampleT* outgoing, int numPorts) noexcept override {
        for (int port = 0; port < numPorts; ++port)
            outgoing[port] = -incident[port];
    }

    SampleT bridgeOutput() const noexcept override { return SampleT(0); }

    void setLossBypassed(bool bypass) noexcept override {
        (void)bypass; // already lossless: |r| == 1 at every port
    }

    // A rigid termination IS the Y == 0 limit of every admittance, so there is nothing to tune:
    // BridgeJunction with couplingStrength == 0 reduces to exactly this object, sample for sample.
    void setAdmittance(const BridgeAdmittanceParams& p) noexcept override { (void)p; }

    // Memoryless: it stores nothing, ever.
    bool isQuiescent() const noexcept override { return true; }
    Sample64 storageEnergy() const noexcept override { return 0.0; }

    // r = -1 is a REAL reflectance, so its phase is 0 at every frequency and it costs the loop no
    // tuning at all. That is not an approximation and not a stub: it is the exact closed form for
    // this termination, and it is why P1's analytic compensation was exact (0.00028 cents) before a
    // load existed. Every argument is unused for exactly that reason.
    double reflectionPhaseDelaySamples(int portIndex, double frequencyHz, int numPorts) const noexcept override {
        (void)portIndex;
        (void)frequencyHz;
        (void)numPorts;
        return 0.0;
    }
};

} // namespace cnpg::dsp

#pragma once

#include <type_traits>

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
    float resonanceHz = 180.0f; // load resonance
    float damping = 0.5f;       // >= 0; positive-real constraint enforced at set time (P2.4)
    // 0..1; scales the FULL load admittance (conductance AND susceptance): 0 = strings fully
    // decoupled AND bridgeOutput() == 0 (the P3 body feed requires > 0). The P1 default is 0
    // because P1's termination is rigid: there is no load to couple into, and P1 ships no
    // parameter that could move this (docs/plan.md Task P2.4 adds the APVTS surface).
    float couplingStrength = 0.0f;
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
};

} // namespace cnpg::dsp

#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/EventQueue.h"
#include "cnpg/dsp/IBridgePort.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/WaveguideString.h"

// StringNetwork -- see docs/plan.md section 2.7. The sample-domain physics core: it owns the
// strings and their exciters plus exactly one IBridgePort&, and runs the whole bidirectional
// section as a per-sample loop, so nothing inside it costs a block of delay. process() consumes
// the sample-accurate event queue at exact offsets and produces the two domain-boundary
// artifacts the block domain consumes: the per-block per-string tap buffers (smoothed fractional
// pickup taps computed PER SAMPLE inside the loop) and the bridge output buffer.
//
// Task P1.1 landed RetriggerMode and the APVTS-wired subset of StringNetworkParams. Task P1.5
// (this file's current state) adds StringTapBuffers, the StringNetwork class, and the remaining
// StringNetworkParams fields except `damper`, whose DamperJunctionParams type ships in
// DamperJunction.h with Task P2.2 -- adding it here would put a P2.2-owned type in a P1 header.
// Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// P1 SCOPE (what is real here, and what is deliberately not yet)
// ---------------------------------------------------------------------------------------------
// P1 is the single-string vertical slice: prepare() preallocates all kMaxStrings strings, but
// setNumStrings(1) is what P1 runs and what NoteAllocator targets. Specifically:
//   - Storage. Everything StringNetwork itself owns is already per-field (structure-of-arrays)
//     across strings, including the tap buffers, so the P2.1 scale-out changes the loop's trip
//     count rather than its shape. The per-string physics objects are still one WaveguideString
//     and one PluckExciter per string; P2.1 flattens THEIR rails, filter states and smoothers
//     into per-field arrays, which is the part that needs the SIMD-friendly layout.
//   - Retrigger. A same-pitch retrigger plucks over the ringing state; a pitch-changing retrigger
//     re-initializes the string at the new pitch. retriggerMode is accepted and stored, and the
//     full Physical (damper choke -> retune ramp -> re-excite) and Synth (<= 5 ms fade) semantics
//     complete in Task P2.6, which is also where the fade that belongs in front of that
//     re-initialization lands -- both modes need DamperJunction, which is P2.2.
//   - NoteOff. A fixed fast release on the string's tap contribution, followed by a state clear
//     once it is inaudible. It is an envelope, not physics: the real felt damper is
//     DamperJunction (P2.2), and damperPosition01/`damper` are stored for it.
//   - Bridge. The port is driven every sample (it sees the strings' outgoing waves and publishes
//     bridgeOutput()), but its reflected waves are NOT fed back into the strings in P1 -- see
//     setBridgePort() for why that is a deliberate P1 boundary rather than an omission.

namespace cnpg::dsp {

enum class RetriggerMode : std::uint8_t {
    Physical, // same pitch: pluck over ringing state; new pitch: damper choke -> retune ramp -> re-excite
    Synth     // fast fade, full state reset, instant re-init at new pitch
};

struct StringNetworkParams {
    RetriggerMode retriggerMode = RetriggerMode::Physical;
    float pitchBendSemitones = 0.0f; // global bend, +/-kPitchBendRangeSemitones (Common.h); not an APVTS
                                     // parameter -- the plugin drives it from the MIDI pitch
                                     // wheel via pitchWheelToSemitones() (MidiTranslation.h)
    float pickupPosition01 = 0.5f;   // tap position; continuously modulatable while ringing
    float damperPosition01 = 0.15f;  // junction position; consumed by DamperJunction (P2.2)
    StringMaterialParams material;   // one global shared physics set
    BridgeAdmittanceParams bridge;   // consumed by BridgeJunction (P2.4)
    PluckExciterParams exciter;
    // DamperJunctionParams damper;  // P2.2, with DamperJunction.h (see the file comment above)

    struct PerString {
        float tuningOffsetCents = 0.0f; // additive cents inside the same f0 smoother as the bend
        bool enabled = true;            // mute/enable
    };
    std::array<PerString, kMaxStrings> perString{};
};

static_assert(std::is_trivially_copyable_v<StringNetworkParams>,
              "StringNetworkParams must stay trivially copyable for the realtime APVTS snapshot path.");

template <typename SampleT> class StringNetwork;

// View over the per-block per-string tap buffers filled by StringNetwork::process. Storage is
// SoA: one contiguous SampleT run per string, owned by StringNetwork and valid until that
// network's next process() call. PickupTap (Task P1.6) is the block-domain consumer.
template <typename SampleT> struct StringTapBuffers {
    // numSamples() contiguous samples for `stringIndex`; nullptr if the index is out of range or
    // no block has been processed yet.
    const SampleT* channel(int stringIndex) const noexcept;

    // String enabled and ringing at some point during the block just processed. A string that is
    // silent for the whole block reports false and its channel is all zeros, so a consumer may
    // skip it entirely rather than summing silence.
    bool isActive(int stringIndex) const noexcept;

    int numStrings() const noexcept;
    int numSamples() const noexcept;

  private:
    friend class StringNetwork<SampleT>;

    const SampleT* base_ = nullptr;
    int stride_ = 0; // samples between channel starts; the prepared maxBlockSize, not numSamples_
    int numStrings_ = 0;
    int numSamples_ = 0;
    std::array<bool, kMaxStrings> active_{};
};

extern template struct StringTapBuffers<float>;  // realtime path
extern template struct StringTapBuffers<double>; // tier-2 [energy] tests

template <typename SampleT> class StringNetwork {
  public:
    // Message thread; allocates. Preallocates for kMaxStrings strings at kMinMidiNote against
    // max(sampleRate, kMaxDesignRateHz) REGARDLESS of the active count, so neither a later
    // setNumStrings() nor a 192 kHz best-effort host can ever under-allocate. `kind` is fixed for
    // the life of the prepared instance. Attaches the internal rigid termination (see
    // setBridgePort) and calls reset().
    void prepare(double sampleRate, int maxBlockSize, FractionalDelayKind kind);

    // Realtime-safe. Clears every string, exciter and buffer, releases all sounding notes, snaps
    // the pickup-position smoother onto its target, and resets the attached port, so a reset
    // instance is indistinguishable from a freshly prepared one carrying the same parameters.
    void reset() noexcept;

    // Realtime-safe; never allocates (capacity is preallocated for kMaxStrings). Clamped to
    // 1..kMaxStrings. A count increase takes effect immediately and the new string starts silent.
    // A count REDUCTION is immediate here as well; routing it through the per-string enable ramp
    // (so the removed string ramps silent first and leaves the loop on a later block) lands with
    // the P2.1 scale-out, together with the ramp itself.
    void setNumStrings(int count) noexcept;
    int numStrings() const noexcept { return numStrings_; }

    // Realtime-safe; only retargets. Never touches string state or a burst already in flight.
    void setParams(const StringNetworkParams& p) noexcept;

    // Message thread. StringNetwork holds exactly one port and cannot tell implementations apart.
    // The port is prepared for kMaxStrings ports against the impedances the strings publish.
    //
    // P1 DRIVES the port every sample -- it is handed the strings' outgoing bridge waves and its
    // bridgeOutput() fills bridgeOutputBuffer() -- but does NOT feed its reflected waves back
    // into the strings, because routing the bridge through an external junction inserts one extra
    // sample into each string's loop, and the loop-length solve that subtracts it again is
    // BridgeJunction's work (see WaveguideString::railAcceptFromBridge). Doing it now would move
    // every note off pitch by that sample -- at MIDI 108 / 44.1 kHz, one sample of a 10.5-sample
    // loop, which is ~160 cents. With P1's rigid termination the reflection the strings apply
    // internally is bit-identical to what the port returns anyway -- "CONTRACT: StringNetwork
    // drives the bridge port but keeps P1's internal termination" asserts both halves of that: the
    // rigid termination's scatter() is exactly -incident, and attaching a port that reflects
    // nonsense does not move one output sample. So nothing audible is being dropped; P2.4 turns
    // the feedback on together with the loop-length term that pays for it.
    void setBridgePort(IBridgePort<SampleT>& port) noexcept;

    // Per-block entry point. Realtime-safe: never allocates, locks, throws or performs I/O.
    // Consumes (pops) every event in `events` inside the per-sample loop at the event's own
    // sampleOffset; offsets are clamped into [0, numSamples - 1], so an offset past the block
    // lands on its last sample rather than being lost. Fills the tap buffers and the bridge
    // output buffer for exactly `numSamples` samples (clamped to the prepared maxBlockSize).
    void process(BlockEventQueue& events, int numSamples) noexcept;

    // Valid until the next process() call.
    const StringTapBuffers<SampleT>& tapBuffers() const noexcept { return tapView_; }

    // numSamples of mono bridge signal (the body/pickup feed). Identically 0 through P1: the
    // rigid termination carries no load (see IBridgePort.h).
    const SampleT* bridgeOutputBuffer() const noexcept { return bridgeBuffer_.data(); }

    // P4 feedback-bus seam, declared now and implemented in P4: the power-amp output re-excites
    // the strings, the block delay being physically the speaker-to-string air path. A no-op in P1
    // -- calling it may not perturb one output sample ("CONTRACT: StringNetwork injectFeedback is
    // audibly inert in P1"). It deliberately records nothing either: state nothing can observe is
    // state that reads as live and guards nothing, and P4 brings its own.
    void injectFeedback(const SampleT* buffer, int numSamples, float airDelayMs, float gain) noexcept;

    // Energy-test hooks (tiers 2 and 3). Forwards to every string and to the attached port,
    // including strings outside the current count, so a later setNumStrings() inherits the mode.
    void setLosslessTestMode(bool lossless) noexcept;

    // Discrete Lyapunov storage function, NOT rail energy alone: impedance-weighted rail energy
    // PLUS the closed-form quadratic storage of every state-bearing element (dispersion allpass
    // states, loss-filter states, fractional-delay interpolator states -- and, from P2.4, the
    // bridge admittance biquad states). Summed over enabled strings. NOT realtime-safe: a
    // string's first call after a coefficient change may run a closed-form factorization. Tier-2
    // [energy] tests assert the 1e-9 per-block non-increase on the double instantiation.
    Sample64 energyEstimate() const noexcept;

  private:
    void handleEvent(const NoteEvent& event) noexcept;
    void applyStringParams(int stringIndex) noexcept;

    // P1 fixed fast release, standing in for DamperJunction (P2.2): a one-pole decay applied to
    // the string's tap contribution, after which the string's state is cleared. The time constant
    // is stated as a TIME CONSTANT, and set to the centre of the 20..100 ms felt-time-constant
    // window DamperJunction validates in P2.2, so replacing this envelope with the real damper is
    // not also a change of speed. It reaches -60 dB in 276 ms and the clear-out floor in 460 ms.
    static constexpr double kReleaseTimeConstantSeconds = 0.040;
    static constexpr float kReleaseFloor = 1.0e-5f; // -100 dB: below this the tail is cleared

    // Per-sample smoothing time for pickupPosition01, matching WaveguideString's own smoothers.
    static constexpr double kPositionSmoothingSeconds = 0.008;

    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;
    int numStrings_ = 1; // P1 vertical slice; P2.1 scales to 1..kMaxStrings
    StringNetworkParams params_{};
    bool lossless_ = false;

    // Physics objects, one per string (see the P1 SCOPE note: P2.1 flattens their internals).
    std::vector<WaveguideString<SampleT>> strings_;
    std::vector<PluckExciter<SampleT>> exciters_;

    // Per-string state, one contiguous array per field (SoA).
    std::array<std::uint8_t, kMaxStrings> midiNote_{};
    std::array<bool, kMaxStrings> sounding_{};
    std::array<bool, kMaxStrings> releasing_{};
    std::array<float, kMaxStrings> releaseGain_{};
    std::array<float, kMaxStrings> portImpedance_{};
    std::array<SampleT, kMaxStrings> portIncident_{};
    std::array<SampleT, kMaxStrings> portOutgoing_{};

    // Domain-boundary buffers: kMaxStrings contiguous runs of maxBlockSize_ samples, plus the
    // bridge feed.
    std::vector<SampleT> tapStorage_;
    std::vector<SampleT> bridgeBuffer_;
    StringTapBuffers<SampleT> tapView_{};

    double positionSmoothingCoeff_ = 0.0;
    double pickupTarget_ = 0.5;
    double pickupSmoothed_ = 0.5;
    float releaseCoeff_ = 0.0f;

    RigidBridgeTermination<SampleT> internalPort_; // the P1 termination; see setBridgePort()
    IBridgePort<SampleT>* port_ = nullptr;
};

extern template class StringNetwork<float>;  // realtime path
extern template class StringNetwork<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

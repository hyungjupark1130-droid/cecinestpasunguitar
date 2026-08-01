#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"
#include "cnpg/dsp/DamperJunction.h"
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
// added StringTapBuffers, the StringNetwork class, and the remaining StringNetworkParams fields
// except `damper`, whose DamperJunctionParams type ships in DamperJunction.h. Task P2.1 scaled the
// network out to N = 1..8 strings and widened the domain boundary to (string, tap). Task P2.2
// (this file's current state) gives every string a real DamperJunction and deletes the P1 release
// envelope. Zero JUCE includes.
//
// ---------------------------------------------------------------------------------------------
// SCOPE (what is real here, and what is deliberately not yet)
// ---------------------------------------------------------------------------------------------
//   - Storage (P2.1). Every piece of per-string state StringNetwork itself owns is one contiguous
//     array per field spanning all strings -- note/sounding/releasing/release gain/enable gain,
//     the port's incident and outgoing waves, the per-(string, tap) position smoothers, and the
//     tap buffers. What is deliberately NOT flattened is the interior of WaveguideString and
//     PluckExciter: those are separate, shipped, gated modules (the [tuning] +/-2-cent sweep and
//     the float64 golden IRs both measure WaveguideString directly), and dissolving their rails
//     and filter states into arrays here would rewrite that arithmetic -- which is exactly what
//     the goldens exist to forbid. The measured headroom does not ask for it either: the P1
//     baseline is ~0.26% of one core per string against a 30% budget. If a later phase does need
//     field-wise physics, it is its own task with its own golden regeneration, not a side effect
//     of a scale-out.
//   - Tap arity (P2.1, docs/decisions/0004-phase2-vision-decisions.md D1). The boundary is
//     (string, tap) with kMaxTapsPerString preallocated and exactly ONE active. Every tap of a
//     string currently reads the same position, so the widening does not move one output sample;
//     it exists because a humbucker is a true two-coil construction -- two spatial taps with real
//     spacing, aperture and polarity -- and because widening the storage is cheap inside the task
//     that is already rewriting it.
//   - Retrigger. A same-pitch retrigger plucks over the ringing state; a pitch-changing retrigger
//     re-initializes the string at the new pitch. retriggerMode is accepted and stored, and the
//     full Physical (damper choke -> retune ramp -> re-excite) and Synth (<= 5 ms fade) semantics
//     complete in Task P2.6, which is also where the fade that belongs in front of that
//     re-initialization lands.
//   - NoteOff (P2.2). PHYSICS, not an envelope any more: the note-off engages the string's
//     DamperJunction with the felt time constant and the string is damped by a real resistive
//     two-port at damperPosition01. Nothing multiplies the tap. The P1 stand-in -- a one-pole
//     gain on the tap contribution -- is gone, and with it the two things that were wrong with
//     it: it damped every partial identically (so no palm mute, no node behaviour, no surviving
//     octave) and it attenuated the OUTPUT while leaving the string's stored energy untouched, so
//     a note-off was inaudible to energyEstimate() until the state clear caught up with it.
//   - Silence watchdog (P2.2). A damped string still has to LEAVE the loop eventually, and the
//     damper cannot promise that: a point contact at p has exact nodes (p = 0.15 puts one on
//     partial 20) that it can never touch, so those partials decay on loop loss alone. The
//     watchdog is therefore a level observation, not a timer -- see kSilenceWindowSeconds below.
//   - Bridge. The port is driven every sample (it sees the strings' outgoing waves and publishes
//     bridgeOutput()), but its reflected waves are NOT fed back into the strings before P2.4 --
//     see setBridgePort() for why that is a deliberate boundary rather than an omission.
//   - Allocation. Which string a host note lands on is NoteAllocator's decision, and its
//     multi-string assignment modes are Task P2.6. StringNetwork addresses strings by the
//     NoteEvent's own stringIndex and does not care where it came from.

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

    // Junction position, 0 = nut, 1 = bridge. LIVE from Task P2.2 but NOT YET SMOOTHED: a change
    // takes effect on the next block boundary, and the click-free machinery (per-sample smoothing
    // here, dual-anchor amplitude-complementary crossfade inside WaveguideString's junction seam)
    // is Task P2.3. What that costs today is bounded and worth stating rather than discovering: at
    // engagement 0 the junction is bit-exactly transparent AT EVERY POSITION, so moving this while
    // no damper is engaged cannot produce a click at all. The exposure is only audible while a
    // damper IS engaged -- during a note-off tail, or with a partial maxLoss held down -- and that
    // is precisely the case P2.3's click test gates ("damperPosition01 swept 0.1->0.9 at 2 Hz with
    // engagement held at 0.5").
    float damperPosition01 = 0.15f;
    StringMaterialParams stringMaterial; // one global shared physics set
    BridgeAdmittanceParams bridge;       // consumed by BridgeJunction (P2.4)
    PluckExciterParams exciter;

    // Shared damper behaviour (Task P2.2). `damper.position01` is MIRRORED from damperPosition01
    // above on the way into each DamperJunction, exactly as docs/plan.md section 2.7 specifies --
    // so on this struct's surface damperPosition01 is the single source of truth and whatever a
    // caller leaves in damper.position01 is ignored. The duplication exists because
    // DamperJunctionParams is the module's own complete parameter set (it is what
    // DamperJunction::setParams takes) while damperPosition01 is what the APVTS automates.
    DamperJunctionParams damper;

    struct PerString {
        float tuningOffsetCents = 0.0f; // additive cents inside the same f0 smoother as the bend

        // RESERVED, and inert through P2.1 -- nothing reads it, and a [contract] test pins that it
        // cannot move an output sample. It is declared now because of what the Envelope module
        // needs (docs/decisions/0004-phase2-vision-decisions.md, D2): in Physical mode the envelope
        // drives LOOP LOSS and damper engagement, and loop loss lives in the SHARED
        // stringMaterial set above. Strings are independently triggered, so an envelope written
        // against that shared set would let one string's note-on alter every other ringing
        // string's decay -- audible, wrong, and structurally impossible to fix without a
        // per-string scalar sitting exactly here. Reserving the seam in the task that is already
        // rewriting this struct costs nothing; retrofitting it after the Envelope module is
        // written costs the Envelope module.
        float envelopeScale = 1.0f;

        bool enabled = true; // mute/enable; a runtime change routes through the enable ramp
    };
    std::array<PerString, kMaxStrings> perString{};
};

static_assert(std::is_trivially_copyable_v<StringNetworkParams>,
              "StringNetworkParams must stay trivially copyable for the realtime APVTS snapshot path.");

// StringNetwork seeds its per-(string, tap) position smoothers with this same value, so a
// prepare() before the first setParams() starts the taps where the parameter says they are rather
// than gliding in from 0. Pinned here so the two cannot drift apart silently.
static_assert(StringNetworkParams{}.pickupPosition01 == 0.5f,
              "StringNetwork::kDefaultTapPosition01 must track StringNetworkParams::pickupPosition01's default.");

template <typename SampleT> class StringNetwork;

// View over the per-block tap buffers filled by StringNetwork::process. This is THE sample->block
// domain boundary, and from Task P2.1 it is addressed by (string, tap) rather than by string alone
// (docs/decisions/0004-phase2-vision-decisions.md, D1). Storage is SoA: one contiguous SampleT run
// per (string, tap), laid out string-major so a string's taps are neighbours, owned by StringNetwork
// and valid until that network's next process() call. PickupTap (Task P1.6) is the block-domain
// consumer.
//
// numTaps() is 1 through P2.1 and the capacity is kMaxTapsPerString. A consumer must loop
// `for t in [0, numTaps())` rather than hardcoding tap 0: that is the whole point of the widening,
// and a consumer that reads only tap 0 will silently drop the second coil of a humbucker the day
// one exists.
template <typename SampleT> struct StringTapBuffers {
    // numSamples() contiguous samples for (stringIndex, tapIndex); nullptr if either index is out
    // of range or no block has been processed yet. The tap index is deliberately NOT defaulted:
    // an implicit tap 0 is exactly the silent-drop this widening exists to prevent.
    const SampleT* channel(int stringIndex, int tapIndex) const noexcept;

    // String enabled and ringing at some point during the block just processed. A string that is
    // silent for the whole block reports false and every one of its channels is all zeros, so a
    // consumer may skip it entirely rather than summing silence.
    //
    // Per STRING, not per (string, tap), and that asymmetry is deliberate: activity means "this
    // string has energy in it", and a tap cannot ring independently of the string it reads.
    bool isActive(int stringIndex) const noexcept;

    // The loop trip count of the block just processed. NOTE: during a setNumStrings() REDUCTION
    // this is larger than StringNetwork::numStrings() -- a removed string keeps its channel until
    // its enable ramp has taken it to silence, because dropping a still-ringing channel from the
    // consumer's view IS the click the ramp exists to prevent.
    int numStrings() const noexcept;

    // Active taps per string; 1 through P2.1, capacity kMaxTapsPerString.
    int numTaps() const noexcept;

    int numSamples() const noexcept;

  private:
    friend class StringNetwork<SampleT>;

    const SampleT* base_ = nullptr;
    int stride_ = 0; // samples between channel starts; the prepared maxBlockSize, not numSamples_
    int numStrings_ = 0;
    int numTaps_ = 0;
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
    // every per-(string, tap) position smoother and every enable ramp onto its target, and resets
    // the attached port, so a reset instance is indistinguishable from a freshly prepared one
    // carrying the same parameters. Also collapses any pending setNumStrings() reduction: after
    // reset() the loop trip count IS numStrings().
    void reset() noexcept;

    // Realtime-safe; never allocates (capacity is preallocated for kMaxStrings). Clamped to
    // 1..kMaxStrings.
    //
    // An INCREASE takes effect immediately. The new string starts silent -- its state was already
    // clear, so its enable gain is snapped rather than ramped (0 * silence and 1 * silence are the
    // same silence, and ramping instead would fade in the attack of a note plucked on it in that
    // same block). Its position smoothers are snapped onto the current target too, so it reads
    // where the pickup IS rather than gliding in from wherever the count last left it.
    //
    // A REDUCTION is deferred. The removed strings are retargeted to silence through the per-string
    // enable ramp and stay in the loop -- and in tapBuffers() -- until that ramp completes, at which
    // point their state is cleared and the trip count drops on a later block. numStrings() reports
    // the count you asked for from the moment you ask for it; tapBuffers().numStrings() reports the
    // trip count, which is what lags. Dropping a still-ringing string from the consumer's view in
    // the same block would be precisely the click the ramp exists to prevent.
    void setNumStrings(int count) noexcept;
    int numStrings() const noexcept { return numStrings_; }

    // Spatial taps per string on the domain boundary, 1..kMaxTapsPerString (D1). ONE through P2.1,
    // and this setter is the seam a later coil-count feature turns up, not a shipped control: every
    // tap currently reads the same smoothed position, so a second tap today is an exact duplicate
    // of the first and summing both would simply double the level. Real per-coil spacing, aperture
    // and polarity -- the things that make a second tap mean something, and that make the comb null
    // at f = v/2d emerge instead of being dialled in -- belong to the task that adds them.
    //
    // Realtime-safe; never allocates (storage is preallocated for kMaxStrings * kMaxTapsPerString).
    // A newly activated tap starts snapped onto the current position target.
    void setNumTapsPerString(int count) noexcept;
    int numTapsPerString() const noexcept { return numTapsPerString_; }

    // Realtime-safe; only retargets. Never touches string state or a burst already in flight.
    // A change to perString[i].enabled routes through the same per-string enable ramp a count
    // reduction uses.
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
    // bridge admittance biquad states). Summed over EVERY string, including strings outside the
    // active count and strings whose enable ramp has muted them: a muted string that is still
    // ringing genuinely stores that energy, and reporting 0 for it would make this function agree
    // with the output rather than with the physics. NOT realtime-safe: a string's first call after
    // a coefficient change may run a closed-form factorization. Tier-2 [energy] tests assert the
    // 1e-9 per-block non-increase on the double instantiation.
    Sample64 energyEstimate() const noexcept;

    // Diagnostics (tests, and later cnpg_calibrate). The position tap (stringIndex, tapIndex) is
    // reading RIGHT NOW, i.e. the per-(string, tap) smoother's current value -- the same role
    // WaveguideString::currentF0Hz() plays for pitch. Returns 0 for an out-of-range index.
    float tapPosition01(int stringIndex, int tapIndex) const noexcept;

    // The damper state of `stringIndex` right now (Task P2.2) -- ALL of it, which is the point:
    // engagement is the felt ramp's current value 0..1, lossDepth is the smoothed maxLoss, and
    // position is where the junction sits on the string. Between them they are the whole of what
    // DamperJunction stores, and every state change any of them undergoes has to be gated by a
    // DIRECT assertion on the state itself rather than only by a click metric on the rendered
    // audio (the P2.1 review's ruling). lossDepth is here for a concrete reason: a smoother nobody
    // can observe is a smoother nobody can gate, and the first bug in this seam was exactly that
    // -- clearStringState() snapped the engagement and left the loss depth gliding.
    // All three return 0 for an out-of-range index.
    float damperEngagement(int stringIndex) const noexcept;
    float damperLossDepth(int stringIndex) const noexcept;
    float damperPosition01(int stringIndex) const noexcept;

  private:
    void handleEvent(const NoteEvent& event) noexcept;
    void applyStringParams(int stringIndex) noexcept;
    void refreshEnableTargets() noexcept;
    void snapTapPositions(int stringIndex) noexcept;
    void updateLoopStringCount() noexcept;
    bool stringHasState(int stringIndex) const noexcept;

    // The one place a string's physical state is thrown away. Clears the rails, snaps the damper
    // back to released, and re-arms the silence watchdog -- always together, because the damper's
    // engagement is the only DISCONTINUOUS thing about it and it is safe exactly when the rails it
    // scatters are zeros. Keeping the three in one function is what makes that invariant checkable
    // rather than a rule three call sites have to remember.
    void clearStringState(int stringIndex) noexcept;

    static constexpr int tapSlot(int stringIndex, int tapIndex) noexcept {
        return stringIndex * kMaxTapsPerString + tapIndex;
    }

    static constexpr int kTapSlots = kMaxStrings * kMaxTapsPerString;

    // The per-(string, tap) position arrays below are filled with the PARAMETER's own default
    // rather than left value-initialized to 0. That is not cosmetic: prepare() snaps the smoothers
    // onto their targets, and a caller that prepares before its first setParams() -- which is the
    // documented order, and what every headless executable and the plugin itself do -- would
    // otherwise start every tap at position 0 (the nut, where the two rails cancel to near-silence)
    // and glide it to 0.5 over the first 8 ms of the very first block. The single smoother this
    // array replaced carried the same default for the same reason.
    static constexpr std::array<double, kTapSlots> filledTapSlots(double value) noexcept {
        std::array<double, kTapSlots> slots{};
        for (double& slot : slots)
            slot = value;
        return slots;
    }
    static constexpr double kDefaultTapPosition01 = 0.5;

    // The silence watchdog that replaced P1's release envelope (Task P2.2). A released string is
    // damped by physics now, so nothing counts it down: it leaves the loop when it is OBSERVED
    // silent, which is the only honest criterion once a point damper is doing the damping. A
    // damper at p has exact nodes at every partial n = k/p (p = 0.15 puts one on partial 20) and
    // is blind to them by construction, so those partials ride the loop loss down on their own
    // schedule and a fixed release time would either cut them off audibly or hold every released
    // string in the trip count for the worst case.
    //
    // Measured as a WINDOWED PEAK rather than an instantaneous level or a one-pole follower.
    // Instantaneous fails at every zero crossing; a follower needs a seed value, and any seed is
    // either a floor on how fast a quiet string can leave (too high) or the same zero-crossing bug
    // (too low). One window's peak needs no seed and cannot be fooled by a zero crossing, provided
    // the window spans a full period of the lowest note the instrument has -- 27.5 Hz, 36.4 ms.
    static constexpr double kSilenceWindowSeconds = 0.050;
    static constexpr float kSilenceFloor = 1.0e-5f; // -100 dBFS: below this the tail is cleared

    // Per-sample smoothing time for pickupPosition01, matching WaveguideString's own smoothers.
    static constexpr double kPositionSmoothingSeconds = 0.008;

    // Per-string enable ramp: the fade a string takes to or from silence when perString[i].enabled
    // moves, or when a setNumStrings() reduction removes it. LINEAR, not one-pole, and that is the
    // point -- a one-pole fade never reaches zero, so "the string is silent, drop it from the trip
    // count and clear its rails" would need an arbitrary audibility floor to ever become true. A
    // linear ramp lands on exactly 0 (and exactly 1) at a known sample. 10 ms is far longer than
    // the ~0.02 ms of one sample and far shorter than the release envelope, so what a listener
    // hears is a mute, not a fade-out.
    static constexpr double kEnableRampSeconds = 0.010;

    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;
    int numStrings_ = 1;       // the REQUESTED active count, 1..kMaxStrings
    int loopStrings_ = 1;      // the per-sample loop's trip count; >= numStrings_ while a
                               // reduction's removed strings are still ramping out
    int numTapsPerString_ = 1; // active taps per string; capacity is kMaxTapsPerString
    StringNetworkParams params_{};
    bool lossless_ = false;

    // Physics objects, one per string. Deliberately NOT flattened into per-field arrays -- see the
    // SCOPE note at the top of this file for why dissolving WaveguideString's rails and filter
    // states here would rewrite arithmetic the goldens and the [tuning] gate exist to pin.
    std::vector<WaveguideString<SampleT>> strings_;
    std::vector<PluckExciter<SampleT>> exciters_;
    std::vector<DamperJunction<SampleT>> dampers_; // one per string, permanently in-line (P2.2)

    // Per-string state, one contiguous array per field (SoA).
    std::array<std::uint8_t, kMaxStrings> midiNote_{};
    std::array<bool, kMaxStrings> sounding_{};
    std::array<bool, kMaxStrings> releasing_{};
    std::array<float, kMaxStrings> silencePeak_{};  // windowed peak of the watchdog, while releasing
    std::array<int, kMaxStrings> silenceCount_{};   // samples into the current watchdog window
    std::array<float, kMaxStrings> enableGain_{};   // 0..1, the ramp's current value
    std::array<float, kMaxStrings> enableTarget_{}; // 0 or 1
    std::array<float, kMaxStrings> portImpedance_{};
    std::array<SampleT, kMaxStrings> portIncident_{};
    std::array<SampleT, kMaxStrings> portOutgoing_{};

    // Domain-boundary buffers: kMaxStrings * kMaxTapsPerString contiguous runs of maxBlockSize_
    // samples laid out (string, tap, sample), plus the bridge feed.
    std::vector<SampleT> tapStorage_;
    std::vector<SampleT> bridgeBuffer_;
    StringTapBuffers<SampleT> tapView_{};

    // One position smoother per (string, tap), SoA (D1/A2). Every slot currently carries the same
    // target -- the global pickupPosition01 -- so every slot holds the same value and the split
    // moves no output sample; it exists so a per-coil offset has somewhere to land.
    double positionSmoothingCoeff_ = 0.0;
    std::array<double, kTapSlots> tapTarget_ = filledTapSlots(kDefaultTapPosition01);
    std::array<double, kTapSlots> tapSmoothed_ = filledTapSlots(kDefaultTapPosition01);
    int silenceWindowSamples_ = 1;
    float enableRampStep_ = 1.0f;

    RigidBridgeTermination<SampleT> internalPort_; // the P1 termination; see setBridgePort()
    IBridgePort<SampleT>* port_ = nullptr;
};

extern template class StringNetwork<float>;  // realtime path
extern template class StringNetwork<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

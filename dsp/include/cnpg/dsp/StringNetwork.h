#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "cnpg/dsp/BridgeJunction.h"
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
//   - Retrigger (P2.6). Both RetriggerMode states are real, and both act only on a string that
//     OWNS A NOTE -- which means SOUNDING, i.e. a note the player has not released. Physical: a
//     same-pitch restrike plucks over the ringing state; a pitch change retargets f0 and glides it
//     with WaveguideString's one-shot retune ramp while the RAILS ARE KEPT, which is what makes the
//     old note continue into the new one -- "emergent legato" is the preserved state, not a slide
//     effect. Synth: a kSynthFadeSeconds fade to silence, a full state clear, then re-init and
//     re-excite at the new pitch, so nothing of the old note survives. A NoteOn on a string that
//     owns NO note is not a retrigger at all and takes neither path -- see the note-ownership
//     paragraph in handleEvent(), which also records what fixes wave 2 measured when this predicate
//     read "sounding or releasing" instead (4.88 dB of corpus RMS on the note-after-note path).
//     The plan's "damper choke" clause is REFUSED with a derivation; see handleEvent().
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
//   - Bridge (P2.4). BIDIRECTIONAL. The network owns a BridgeJunction and runs it as its default
//     port: every sample it gathers each string's outgoing bridge wave, scatters them through the
//     junction's loaded N-port, and hands each string back its reflection. That closes the loop
//     that makes six strings one instrument -- sympathetic resonance, two-stage decay, dead spots
//     -- and it is what setBridgePort() replaces when P2.5's fallback bus is substituted. Three
//     consequences worth stating where they bite: the seam adds exactly one sample to every
//     string's loop (WaveguideString::setBridgePortDriven subtracts it from the tuning solve), a
//     string with NO state of its own can now be driven purely through the bridge (which is why
//     the idle-string skip predicate in process() had to grow a bridge term), and the junction's
//     stored energy is part of energyEstimate()'s storage functional.
//   - Allocation. Which string a host note lands on is NoteAllocator's decision, and its
//     multi-string assignment modes are Task P2.6. StringNetwork addresses strings by the
//     NoteEvent's own stringIndex and does not care where it came from.

namespace cnpg::dsp {

enum class RetriggerMode : std::uint8_t {
    Physical, // same pitch: pluck over ringing state; new pitch: retune ramp, rails kept (legato)
    Synth     // fast fade, full state reset, instant re-init at new pitch
};

// The Physical retrigger's retune ramp and the Synth retrigger's fade, both named here so tests
// reference them rather than restating numbers, and both DEFAULTS rather than constants: the ramp
// length is a legato-speed voicing choice (StringNetwork::setRetuneRampSeconds) and the fade is
// what the [contract] gate measures.
//
// 30 ms: the plan's own figure. A retune ramp does not remove motion, it turns a step into a glide,
// and a glide of a full rail has motion of its own -- while the rails shorten the read position
// sweeps through the buffer faster than one sample per sample, which is the pitch change and which
// reads on a peak-|dx| metric as motion a fresh pluck does not have. Measured against a fresh pluck
// at MIDI 45 -> 51 (tests/dsp/RetriggerModeTests.cpp, printed every run, 19 points, with the
// re-strike LEVEL-PLACED on the loudest sample of the old note's cycle -- see below):
//
//     ramp     0.02 ms  2 ms   8 ms   16 ms  17 ms  18 ms  19 ms  20 ms  21 ms  22 ms
//     excess   10.37    18.70  10.11  6.04   4.85   2.62   2.22   2.30   3.39   1.18
//     ramp     23 ms    24 ms  25 ms  26 ms  28 ms  29 ms  30 ms  31 ms  32 ms
//     excess   1.83     1.47   1.47   1.20   3.59   0.61   0.90   0.98   1.00
//
// 30 ms clears the 3 dB criterion, AND THAT IS ALL THIS STATISTIC SAYS. It does not say 30 ms is the
// shortest ramp that clears it -- 22 ms clears it at 1.18 dB and eight other sub-30 ms points do too
// -- and it cannot order ramp lengths at all: 28 ms FAILS at 3.59 dB sitting between two passing
// neighbours, and the sequence inverts at seven of the sampled points. An earlier revision of this
// comment claimed "at nothing shorter" from a five-point sweep too coarse to see any of that.
//
// THE TABLE MOVED AT FIXES WAVE 3 AND THE CONCLUSIONS DID NOT. Until then the re-strike was struck
// at a round block boundary -- an arbitrary phase of the 110 Hz note being replaced -- and the whole
// table is a function of that phase: recomputing the gate at ten phases across one period swung its
// median from 0.111 to 3.199 dB, across the criterion, on identical code. The re-strike is now
// placed on the loudest sample of the old note's cycle (tests/support/ClickMetric.h states the rule
// for controls and perturbations alike), which makes the phase a property of the signal instead of
// of the block size. Every number above is the level-placed one.
//
// The statistic is also STRUCTURALLY BIASED against legato and cannot choose this constant even in
// principle: its reference is a fresh pluck, which contains no glide, so every millisecond of glide
// is excess by construction and a longer ramp always reads better. Its minimum is at "no legato".
// What it is for is refusing a ramp so short that the retune is a STEP -- 8 ms was tried first, on
// the argument that 30 ms of glide is the audible-slide mode Q3 defers, and is refused by this gate
// at 10.11 dB. Whether 30 ms of glide reads as a hammer-on or as a slide is an ear question, it is
// docs/listening/physical-plausibility-checklist.md item 17, it is UNANSWERED, and it is what
// actually decides this number. The knob to answer it with is StringNetwork::setRetuneRampSeconds.
inline constexpr double kRetuneRampSeconds = 0.030;

// 2 ms, under the plan's "<= 5 ms fade" by 2.5x. The fade is the only thing standing between a
// full rail and a state clear, so it exists to make that clear click-free and nothing more; its
// length is therefore pure latency (the re-excitation waits for it) and the shortest duration that
// does the job is the right one. 96 samples at 48 kHz, and only on a retrigger over a string that
// is already sounding -- a fresh note on an idle string is not delayed at all, and neither is one
// arriving over a string whose note has been RELEASED: a released note is over, so that is a fresh
// note in either mode and the two modes render it bit-identically (tests/dsp/RetriggerModeTests.cpp).
inline constexpr double kSynthFadeSeconds = 0.002;

struct StringNetworkParams {
    RetriggerMode retriggerMode = RetriggerMode::Physical;
    float pitchBendSemitones = 0.0f; // global bend, +/-kPitchBendRangeSemitones (Common.h); not an APVTS
                                     // parameter -- the plugin drives it from the MIDI pitch
                                     // wheel via pitchWheelToSemitones() (MidiTranslation.h)
    float pickupPosition01 = 0.5f;   // tap position; continuously modulatable while ringing

    // Junction position, 0 = nut, 1 = bridge. Continuously modulatable while a note rings from
    // Task P2.3: this value is a block-snapshotted TARGET, per-sample smoothed inside process()
    // exactly as pickupPosition01 is, and the smoothed result reaches the rails through
    // WaveguideString's dual-anchor amplitude-complementary crossfade. Through P2.2 it was applied
    // raw, so a change stepped at the block boundary; with a damper engaged that was an audible
    // click, measured at 5.50 dB of click-metric excess against a 3 dB criterion (the P2.3 RED
    // reading, tests/dsp/MovingPositionClickTests.cpp).
    //
    // damperPosition01(stringIndex) reports the SMOOTHED value actually in force;
    // DamperJunction::currentPosition01() reports the validated target it is gliding toward. There
    // is one validation point (the junction's own setParams clamp) and the smoother sits
    // downstream of it, so the two cannot disagree about anything but the glide.
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
    // The port is prepared for kMaxStrings ports against the impedances the strings publish, given
    // the current StringNetworkParams::bridge, and reset.
    //
    // The DEFAULT port is the network's own BridgeJunction, so the shipping topology is coupled
    // without any caller having to remember to attach anything. That is deliberate: a bridge port
    // that is absent or not ticked fails OPEN -- the strings fall back to their internal rigid
    // termination and the instrument still makes sound, just an uncoupled one, with nothing to
    // hear except the absence of something that was never there. Owning the default removes the
    // whole failure mode for every caller (plugin, cnpg_bench, cnpg_render, the goldens, the
    // [tuning] sweep) at once; bridgeDrivenTicks()/unbridgedTicks() below are what assert it for
    // the case where a caller DOES substitute a port.
    //
    // This entry point exists for that substitution: docs/plan.md Q17 designs for the P2.5
    // fallback by making BridgeJunction and SympatheticResonatorBus share IBridgePort, and this is
    // the seam the swap happens at.
    void setBridgePort(IBridgePort<SampleT>& port) noexcept;

    // Per-block entry point. Realtime-safe: never allocates, locks, throws or performs I/O.
    // Consumes (pops) every event in `events` inside the per-sample loop at the event's own
    // sampleOffset; offsets are clamped into [0, numSamples - 1], so an offset past the block
    // lands on its last sample rather than being lost. Fills the tap buffers and the bridge
    // output buffer for exactly `numSamples` samples (clamped to the prepared maxBlockSize).
    void process(BlockEventQueue& events, int numSamples) noexcept;

    // Valid until the next process() call.
    const StringTapBuffers<SampleT>& tapBuffers() const noexcept { return tapView_; }

    // numSamples of mono bridge signal (the body/pickup feed) -- the bridge point's VELOCITY, which
    // is what a body node is driven by. Identically 0 only when the load is fully decoupled
    // (BridgeAdmittanceParams::couplingStrength == 0), which is exactly why that is no longer the
    // shipping default (docs/decisions/0006). Its LEVEL is physical rather than normalized: a
    // lightly-coupled bridge moves very little, so this signal sits far below the tap channels and
    // whatever consumes it is expected to scale it.
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
    //
    // damperPosition01 is StringNetwork's OWN per-(string) smoother (Task P2.3), not a forward to
    // the junction: DamperJunction::currentPosition01() is the validated target, this is the value
    // in force right now, and the difference between them is exactly the glide. All three return 0
    // for an out-of-range index.
    float damperEngagement(int stringIndex) const noexcept;
    float damperLossDepth(int stringIndex) const noexcept;
    float damperPosition01(int stringIndex) const noexcept;

    // ---- retrigger diagnostics (Task P2.6) -----------------------------------------------------
    // The whole of what a retrigger changes, observable directly rather than inferred from audio.
    // The P2.1 ruling in one line: a state change covered only by a click test is not tested.

    // The per-sample smoothed fundamental string `stringIndex` is synthesizing right now -- the
    // quantity the "f0 reaches the new pitch within 30 ms" criterion is about. 0 for an
    // out-of-range index or an unprepared network.
    float stringF0Hz(int stringIndex) const noexcept;

    // Samples left in that string's Physical retune ramp, 0 when none is in flight. Exposed so a
    // test asserts the ramp IS running at the moment it claims to measure it, and asserts the exact
    // landing sample instead of sampling a tolerance around an asymptote.
    int retuneRampSamplesRemaining(int stringIndex) const noexcept;

    // The Synth retrigger fade's gain for `stringIndex`: 1 when no fade is in flight, gliding
    // linearly to exactly 0 across kSynthFadeSeconds and back to 1 on the sample the state is
    // cleared and the pending note fires. It multiplies the tap AND the string's bridge incident
    // wave, so a string fading out under a retrigger stops driving its neighbours too.
    float retriggerFadeGain(int stringIndex) const noexcept;
    bool retriggerFadeActive(int stringIndex) const noexcept;

    // The Physical retune ramp's length in seconds, default kRetuneRampSeconds. A LEGATO SPEED, so
    // it is a voicing control rather than a test hook -- how long a fretted pitch change takes to
    // arrive is exactly the kind of question the P2.8 listening pass exists to answer, and it is
    // also what lets a [contract] case measure what the ramp buys by shortening it to one sample.
    // Realtime-safe; clamped to at least one sample by WaveguideString::beginRetuneRamp. Applies to
    // ramps STARTED after the call; one already in flight keeps the length it was given.
    void setRetuneRampSeconds(double seconds) noexcept;
    double retuneRampSeconds() const noexcept { return retuneRampSeconds_; }

    // ---- bridge diagnostics (Task P2.4) --------------------------------------------------------

    // ONE string's contribution to the storage functional. The whole point of bidirectional
    // coupling is that a string nobody plucked ends up holding energy, and "the tap got louder" is
    // not evidence of that -- it is satisfied by leakage, by summing the wrong channel, and by a
    // reference render that was never silent. This is the direct state observation that says the
    // energy is IN string k. Returns 0 for an out-of-range index. NOT realtime-safe (see
    // energyEstimate()).
    Sample64 stringEnergyEstimate(int stringIndex) const noexcept;

    // JUNCTION LIVENESS (carry-forward B4). Summed over every string: ticks whose bridge reflection
    // came from the attached port, and ticks that fell back to WaveguideString's internal rigid
    // -1 because no reflection had been supplied. In this network the second number must be ZERO
    // for every rendered sample -- a nonzero count means the port was mis-wired or did not tick,
    // which degrades the instrument to uncoupled strings SILENTLY. Cleared by reset().
    unsigned long long bridgeDrivenTicks() const noexcept;
    unsigned long long unbridgedTicks() const noexcept;

    // The network's own default BridgeJunction, for tests and for the plugin's parameter surface.
    // Returns nullptr semantics are avoided deliberately: the network always owns one, whether or
    // not setBridgePort() has substituted something else for the audio path.
    BridgeJunction<SampleT>& internalBridgeJunction() noexcept { return internalBridge_; }
    const BridgeJunction<SampleT>& internalBridgeJunction() const noexcept { return internalBridge_; }

    // The port actually in the loop right now (the internal junction unless setBridgePort()
    // substituted one). Diagnostics only.
    const IBridgePort<SampleT>* attachedBridgePort() const noexcept { return port_; }

  private:
    void handleEvent(const NoteEvent& event) noexcept;
    void excite(int stringIndex, const NoteEvent& event) noexcept;
    void landSynthFade(int stringIndex) noexcept;
    void applyStringParams(int stringIndex) noexcept;
    void refreshEnableTargets() noexcept;
    void snapPositionSmoothers(int stringIndex) noexcept;
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

    // Same argument as filledTapSlots above, for the Synth retrigger fade: an array of zeros would
    // mean "every string is fully faded out" on an instance nobody has reset yet, and the diagnostic
    // accessor would report it.
    static constexpr std::array<float, kMaxStrings> filledStrings(float value) noexcept {
        std::array<float, kMaxStrings> values{};
        for (float& slot : values)
            slot = value;
        return values;
    }

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

    // Per-sample smoothing time for pickupPosition01 AND damperPosition01, matching WaveguideString's
    // own smoothers. Both positions are block-snapshotted targets smoothed per sample inside
    // process(); the smoothed value is what WaveguideString's dual-anchor crossfade is handed.
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
    // "The player is holding a note on this string." THE ownership predicate, and the one
    // handleEvent() reads to decide whether a NoteOn is a retrigger -- NoteAllocator::owned_ asks
    // the same question of the same two events (see that header). Set by the note-on this class
    // consumes, cleared by the note-off it consumes and by the enable ramp landing on zero.
    std::array<bool, kMaxStrings> sounding_{};
    // "A note was released here and its tail has not died yet." NOT ownership: a released note is
    // over, and a NoteOn arriving over one is a fresh note, not a retrigger. Set by the note-off,
    // cleared by the silence watchdog -- which can be seconds later, and reading this as ownership
    // is what made the two levels disagree for the whole of that time (fixes wave 2).
    std::array<bool, kMaxStrings> releasing_{};
    // "This string carries motion", independent of whether anyone played it (Task P2.4). Under
    // bidirectional coupling a string can be ringing with no note of its own -- that IS sympathetic
    // resonance -- and every piece of bookkeeping that used to read (sounding || releasing) as
    // "has state" would otherwise be wrong about it: the enable-ramp snap, the position-smoother
    // snap, the loop's own skip predicate and the silence watchdog. Latched by any rendered sample
    // whose outgoing bridge wave is non-zero, cleared with the string's state by the watchdog.
    std::array<bool, kMaxStrings> ringing_{};
    std::array<float, kMaxStrings> silencePeak_{};  // windowed peak of the watchdog, while releasing
    std::array<int, kMaxStrings> silenceCount_{};   // samples into the current watchdog window
    std::array<float, kMaxStrings> enableGain_{};   // 0..1, the ramp's current value
    std::array<float, kMaxStrings> enableTarget_{}; // 0 or 1
    std::array<float, kMaxStrings> portImpedance_{};
    std::array<SampleT, kMaxStrings> portIncident_{};
    std::array<SampleT, kMaxStrings> portOutgoing_{};

    // The Synth retrigger's fade (Task P2.6), one per string. `fadeGain_` multiplies the tap AND
    // the bridge incident wave; `fadeSteps_` counts down to the sample the state is cleared and
    // `pendingNote_` fires. `pendingNoteOff_` covers the case that would otherwise be a stuck note:
    // a host sending a note-off inside the 2 ms fade, for the note that has not started yet.
    std::array<float, kMaxStrings> fadeGain_ = filledStrings(1.0f);
    std::array<int, kMaxStrings> fadeSteps_{};
    std::array<bool, kMaxStrings> fadeActive_{};
    std::array<bool, kMaxStrings> pendingNoteOff_{};
    std::array<NoteEvent, kMaxStrings> pendingNote_{};
    int synthFadeSamples_ = 1;
    float synthFadeStep_ = 1.0f;
    double retuneRampSeconds_ = kRetuneRampSeconds;

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

    // One damper-position smoother per STRING (Task P2.3). Per string rather than global because
    // every place that snaps a smoother -- reset(), a setNumStrings() increase over a silent string
    // -- is per string, and a global smoother could not be snapped for one string without stepping
    // the junction of every other one that is still ringing. The target is read back from the
    // string's own DamperJunction after setParams, so the clamp/NaN validation happens in exactly
    // one place and this smoother is downstream of it.
    std::array<double, kMaxStrings> damperTarget_{};
    std::array<double, kMaxStrings> damperSmoothed_{};
    int silenceWindowSamples_ = 1;
    float enableRampStep_ = 1.0f;

    // Which strings the loop rendered on the PREVIOUS sample. A string that rendered fed the
    // junction, so on the next sample the junction may hand energy to any string -- including one
    // that has nothing of its own. This is the cheap, exact form of "the bridge node is moving"
    // that the skip predicate needs; the junction's own isQuiescent() covers the case where every
    // string has already fallen silent but the bridge resonator has not.
    std::uint32_t previousRenderedMask_ = 0;

    BridgeJunction<SampleT> internalBridge_; // the shipping default port; see setBridgePort()
    IBridgePort<SampleT>* port_ = nullptr;
};

extern template class StringNetwork<float>;  // realtime path
extern template class StringNetwork<double>; // tier-2 [energy] tests

} // namespace cnpg::dsp

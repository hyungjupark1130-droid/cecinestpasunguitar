#include "cnpg/dsp/StringNetwork.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

constexpr double kCentsPerSemitone = 100.0;

double clampd(double v, double lo, double hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

float clampf(float v, float lo, float hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

// docs/plan.md section 2.3: the exciter defaults are "used when the note event carries no explicit
// position". An in-range event value is per-note data and wins; anything else --
// kUnspecifiedNoteParam, or a NaN from a malformed caller -- falls back to the parameter.
float resolveNoteParam(float eventValue, float exciterDefault) noexcept {
    if (eventValue >= 0.0f && eventValue <= 1.0f)
        return eventValue;
    return clampf(exciterDefault, 0.0f, 1.0f);
}

// Equal temperament, A4 = 440 Hz -- the same expression the [tuning] and [regression] harnesses
// use, so a note rendered through the network lands on exactly the frequency they measure.
double midiNoteToHz(int midiNote) noexcept { return 440.0 * std::exp2((static_cast<double>(midiNote) - 69.0) / 12.0); }

} // namespace

// ------------------------------------------------------------------------------------------
// StringTapBuffers
// ------------------------------------------------------------------------------------------

template <typename SampleT>
const SampleT* StringTapBuffers<SampleT>::channel(int stringIndex, int tapIndex) const noexcept {
    if (base_ == nullptr || stringIndex < 0 || stringIndex >= numStrings_ || tapIndex < 0 || tapIndex >= numTaps_)
        return nullptr;
    const auto slot = static_cast<std::ptrdiff_t>(stringIndex) * static_cast<std::ptrdiff_t>(kMaxTapsPerString) +
                      static_cast<std::ptrdiff_t>(tapIndex);
    return base_ + slot * static_cast<std::ptrdiff_t>(stride_);
}

template <typename SampleT> bool StringTapBuffers<SampleT>::isActive(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= numStrings_)
        return false;
    return active_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> int StringTapBuffers<SampleT>::numStrings() const noexcept { return numStrings_; }

template <typename SampleT> int StringTapBuffers<SampleT>::numTaps() const noexcept { return numTaps_; }

template <typename SampleT> int StringTapBuffers<SampleT>::numSamples() const noexcept { return numSamples_; }

// ------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void StringNetwork<SampleT>::prepare(double sampleRate, int maxBlockSize, FractionalDelayKind kind) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(1, maxBlockSize);

    // Every string is prepared, not just the active ones: WaveguideString::prepare sizes its
    // rails for kMinMidiNote (bend included) against max(sampleRate, kMaxDesignRateHz), and
    // setNumStrings() must never allocate afterwards.
    strings_.resize(static_cast<std::size_t>(kMaxStrings));
    exciters_.resize(static_cast<std::size_t>(kMaxStrings));
    dampers_.resize(static_cast<std::size_t>(kMaxStrings));
    for (int s = 0; s < kMaxStrings; ++s) {
        strings_[static_cast<std::size_t>(s)].prepare(sampleRate_, maxBlockSize_, kind);
        strings_[static_cast<std::size_t>(s)].setAnalyticTuningCompensation(0.0f);
        exciters_[static_cast<std::size_t>(s)].prepare(sampleRate_, maxBlockSize_);
        dampers_[static_cast<std::size_t>(s)].prepare(sampleRate_, maxBlockSize_);
        dampers_[static_cast<std::size_t>(s)].setLossBypassed(lossless_);
        midiNote_[static_cast<std::size_t>(s)] = static_cast<std::uint8_t>(kMinMidiNote);
        portImpedance_[static_cast<std::size_t>(s)] = strings_[static_cast<std::size_t>(s)].portImpedance();
    }

    // (string, tap, sample): kMaxStrings * kMaxTapsPerString runs of maxBlockSize_, all of them
    // preallocated regardless of how many strings or taps are active, for the same reason the rails
    // are -- setNumStrings() and setNumTapsPerString() are realtime-safe and must never allocate.
    tapStorage_.assign(static_cast<std::size_t>(kMaxStrings) * static_cast<std::size_t>(kMaxTapsPerString) *
                           static_cast<std::size_t>(maxBlockSize_),
                       SampleT(0));
    bridgeBuffer_.assign(static_cast<std::size_t>(maxBlockSize_), SampleT(0));

    positionSmoothingCoeff_ = 1.0 - std::exp(-1.0 / (kPositionSmoothingSeconds * sampleRate_));
    silenceWindowSamples_ = std::max(1, static_cast<int>(std::lround(kSilenceWindowSeconds * sampleRate_)));
    enableRampStep_ = static_cast<float>(1.0 / std::max(1.0, kEnableRampSeconds * sampleRate_));

    if (port_ == nullptr)
        port_ = &internalPort_;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);

    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);

    reset();
}

template <typename SampleT> void StringNetwork<SampleT>::reset() noexcept {
    // reset() before prepare() must stay inert rather than index empty vectors: the module
    // lifecycle allows it (docs/plan.md section 2.1) and prepare() itself calls reset() last.
    const bool prepared = !strings_.empty();
    for (int s = 0; s < kMaxStrings; ++s) {
        if (prepared) {
            // clearStringState covers the string, the damper and the watchdog together -- see its
            // declaration for why those three always move as one.
            clearStringState(s);
            exciters_[static_cast<std::size_t>(s)].reset();
        } else {
            silencePeak_[static_cast<std::size_t>(s)] = 0.0f;
            silenceCount_[static_cast<std::size_t>(s)] = 0;
        }
        sounding_[static_cast<std::size_t>(s)] = false;
        releasing_[static_cast<std::size_t>(s)] = false;
        portIncident_[static_cast<std::size_t>(s)] = SampleT(0);
        portOutgoing_[static_cast<std::size_t>(s)] = SampleT(0);
    }

    std::fill(tapStorage_.begin(), tapStorage_.end(), SampleT(0));
    std::fill(bridgeBuffer_.begin(), bridgeBuffer_.end(), SampleT(0));
    tapView_ = StringTapBuffers<SampleT>{};

    // Snap every position smoother and every enable ramp onto its target, matching
    // WaveguideString::reset(): a reset instance must be indistinguishable from a freshly prepared
    // one. That includes collapsing a pending count reduction -- nothing is ringing for a ramp to
    // protect any more, so the trip count IS the requested count.
    refreshEnableTargets();
    for (int s = 0; s < kMaxStrings; ++s) {
        enableGain_[static_cast<std::size_t>(s)] = enableTarget_[static_cast<std::size_t>(s)];
        snapPositionSmoothers(s);
    }
    loopStrings_ = numStrings_;

    if (port_ != nullptr)
        port_->reset();
}

template <typename SampleT> void StringNetwork<SampleT>::setNumStrings(int count) noexcept {
    const int clamped = std::clamp(count, 1, kMaxStrings);
    const int previous = numStrings_;
    numStrings_ = clamped;

    if (clamped > previous) {
        // Immediate: the readmitted strings rejoin the loop on the next block.
        loopStrings_ = std::max(loopStrings_, clamped);

        // Their POSITION smoothers -- pickup taps and the damper junction alike -- are snapped only
        // if the string is genuinely silent. "Outside the count" does not imply "carries no state":
        // a reduction leaves its removed strings ringing
        // for the length of their enable ramp -- that deferral is the entire reason
        // updateLoopStringCount() exists -- so an increase arriving inside that window readmits
        // strings that are still sounding. Snapping one of those would jump its tap read by however
        // far the position smoother still had to glide, on a string with a live waveform under the
        // tap: a genuine discontinuity in readTapAt(), and the exact defect class the smoother is
        // there to prevent. A silent string has no such waveform, and snapping it is what stops a
        // note plucked on it from gliding in from wherever the count last left the pickup.
        //
        // Same predicate, same reason, as the enable-gain snap in refreshEnableTargets() below.
        for (int s = previous; s < clamped; ++s)
            if (!stringHasState(s))
                snapPositionSmoothers(s);
    }

    refreshEnableTargets();
}

template <typename SampleT> void StringNetwork<SampleT>::setNumTapsPerString(int count) noexcept {
    const int clamped = std::clamp(count, 1, kMaxTapsPerString);
    if (clamped > numTapsPerString_) {
        // A newly activated tap has no history to glide from: snap it onto the current target so
        // its first sample reads where the pickup is.
        for (int s = 0; s < kMaxStrings; ++s)
            for (int t = numTapsPerString_; t < clamped; ++t) {
                const auto slot = static_cast<std::size_t>(tapSlot(s, t));
                tapSmoothed_[slot] = tapTarget_[slot];
            }
    }
    numTapsPerString_ = clamped;
}

template <typename SampleT> void StringNetwork<SampleT>::setParams(const StringNetworkParams& p) noexcept {
    params_ = p;
    // One target per (string, tap). Every slot carries the same global pickupPosition01 today; the
    // array exists so a per-coil offset has somewhere to land without another rewrite of the loop.
    const double target = clampd(static_cast<double>(params_.pickupPosition01), 0.0, 1.0);
    for (double& slot : tapTarget_)
        slot = target;
    refreshEnableTargets();
    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);
}

template <typename SampleT> void StringNetwork<SampleT>::setBridgePort(IBridgePort<SampleT>& port) noexcept {
    port_ = &port;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);
    port_->reset();
}

template <typename SampleT> bool StringNetwork<SampleT>::stringHasState(int stringIndex) const noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);
    if (sounding_[index] || releasing_[index])
        return true;
    return !exciters_.empty() && exciters_[index].isActive();
}

template <typename SampleT> void StringNetwork<SampleT>::refreshEnableTargets() noexcept {
    for (int s = 0; s < kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        const bool wanted = (s < numStrings_) && params_.perString[index].enabled;
        enableTarget_[index] = wanted ? 1.0f : 0.0f;

        // A string with no state at all is snapped in EITHER direction rather than ramped, and the
        // argument is the same both ways: 0 * silence and 1 * silence are the same silence, so
        // there is no discontinuity available to produce. Going on, a ramp would attenuate the
        // front of whatever gets plucked next; going off, it would hold a string that has nothing
        // to say in the loop's trip count for ten pointless milliseconds. What is NEVER snapped is
        // a string that still holds a tail -- including one coming back on mid-ramp-out, where a
        // jump in gain would step that tail.
        if (enableTarget_[index] != enableGain_[index] && !stringHasState(s))
            enableGain_[index] = enableTarget_[index];
    }
}

template <typename SampleT> void StringNetwork<SampleT>::snapPositionSmoothers(int stringIndex) noexcept {
    for (int t = 0; t < kMaxTapsPerString; ++t) {
        const auto slot = static_cast<std::size_t>(tapSlot(stringIndex, t));
        tapSmoothed_[slot] = tapTarget_[slot];
    }
    // The junction position snaps with the taps, and for the same reason: both are only ever
    // snapped where the string carries no state for the snap to step. WaveguideString's own anchors
    // are disarmed by its reset(), so a cleared string re-anchors on wherever it is next asked to
    // read rather than crossfading in from where the last note left the damper.
    damperSmoothed_[static_cast<std::size_t>(stringIndex)] = damperTarget_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> void StringNetwork<SampleT>::updateLoopStringCount() noexcept {
    // The trip count is the requested count, extended upward over any removed string whose enable
    // ramp has not finished taking it to silence. Once a ramp lands on 0 the per-sample loop clears
    // that string, so `enableGain_ > 0` is exactly "still has something to say".
    int required = numStrings_;
    for (int s = numStrings_; s < kMaxStrings; ++s)
        if (enableGain_[static_cast<std::size_t>(s)] > 0.0f)
            required = s + 1;
    loopStrings_ = std::clamp(required, 1, kMaxStrings);
}

template <typename SampleT> void StringNetwork<SampleT>::clearStringState(int stringIndex) noexcept {
    if (strings_.empty())
        return;
    const auto index = static_cast<std::size_t>(stringIndex);
    strings_[index].reset();
    // reset(), not setEngagementImmediate(0). Both open the damper, but the junction has a SECOND
    // smoother -- the loss depth -- and only reset() snaps that one onto its parameter too. Using
    // the narrower call left a maxLoss automation move gliding across a state clear, so a string
    // cleared mid-glide came back carrying the old depth for another 8 ms. That is inaudible today
    // (the engagement is 0, so the coefficient is 0 whatever the depth is) and it still had to go:
    // it contradicts this junction's own documented reset contract, and "inaudible today" is a
    // property of the current call graph rather than of the code.
    //
    // Snapping the engagement is a DISCONTINUITY in the junction's scattering coefficients, and
    // the only reason it is inaudible is that the line above just made every wave the junction
    // scatters a zero. That is the whole invariant, and it is why this pairing is a function
    // rather than a convention.
    dampers_[index].reset();
    silencePeak_[index] = 0.0f;
    silenceCount_[index] = 0;
}

template <typename SampleT> void StringNetwork<SampleT>::applyStringParams(int stringIndex) noexcept {
    if (strings_.empty())
        return;

    const auto index = static_cast<std::size_t>(stringIndex);

    // The damper's own parameter set, with position01 mirrored from the network's
    // damperPosition01 (docs/plan.md section 2.7) -- so the network's surface has exactly one
    // position field and DamperJunctionParams::position01 is a module-level detail.
    DamperJunctionParams damperParams = params_.damper;
    damperParams.position01 = params_.damperPosition01;
    dampers_[index].setParams(damperParams);
    // Read the target back OUT of the junction rather than from params_: setParams is where
    // position01 is clamped into 0..1 and where a NaN resolves, so taking the value from there
    // keeps one validation point and puts this smoother strictly downstream of it. Only the target
    // moves; the smoothed value glides toward it inside process().
    damperTarget_[index] = static_cast<double>(dampers_[index].currentPosition01());

    WaveguideStringParams p;
    p.f0Hz = static_cast<float>(midiNoteToHz(static_cast<int>(midiNote_[index])));
    // Bend and the per-string tuning offset compose additively in semitones and reach the string
    // through its own per-sample f0 smoother, which is what keeps both click-free while ringing.
    const float bend = clampf(params_.pitchBendSemitones, -kPitchBendRangeSemitones, kPitchBendRangeSemitones);
    p.bendSemitones =
        bend + static_cast<float>(static_cast<double>(params_.perString[index].tuningOffsetCents) / kCentsPerSemitone);
    p.stringMaterial = params_.stringMaterial;
    strings_[index].setParams(p);
}

template <typename SampleT> float StringNetwork<SampleT>::tapPosition01(int stringIndex, int tapIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings || tapIndex < 0 || tapIndex >= kMaxTapsPerString)
        return 0.0f;
    return static_cast<float>(tapSmoothed_[static_cast<std::size_t>(tapSlot(stringIndex, tapIndex))]);
}

template <typename SampleT> float StringNetwork<SampleT>::damperEngagement(int stringIndex) const noexcept {
    if (dampers_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0f;
    return dampers_[static_cast<std::size_t>(stringIndex)].currentEngagement();
}

template <typename SampleT> float StringNetwork<SampleT>::damperLossDepth(int stringIndex) const noexcept {
    if (dampers_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0f;
    return dampers_[static_cast<std::size_t>(stringIndex)].currentLossDepth();
}

template <typename SampleT> float StringNetwork<SampleT>::damperPosition01(int stringIndex) const noexcept {
    if (dampers_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0f;
    // The SMOOTHED value, i.e. where the junction seam is being read and written right now -- the
    // same role tapPosition01() plays for the pickup. The junction's own currentPosition01() is the
    // validated target this is gliding toward.
    return static_cast<float>(damperSmoothed_[static_cast<std::size_t>(stringIndex)]);
}

// ------------------------------------------------------------------------------------------
// event consumption
// ------------------------------------------------------------------------------------------

template <typename SampleT> void StringNetwork<SampleT>::handleEvent(const NoteEvent& event) noexcept {
    const int stringIndex = static_cast<int>(event.stringIndex);
    // Addressed against the REQUESTED count, not the trip count: a string that is only still in the
    // loop because it is ramping out has been removed as far as a caller is concerned, and handing
    // it a fresh note would resurrect it underneath its own fade.
    if (stringIndex < 0 || stringIndex >= numStrings_)
        return;
    const auto index = static_cast<std::size_t>(stringIndex);
    if (!params_.perString[index].enabled)
        return;

    if (event.type == NoteEventType::NoteOff) {
        if (!sounding_[index])
            return; // already released, or never sounded: nothing to damp
        sounding_[index] = false;
        releasing_[index] = true;
        // THE note-off (Task P2.2). The felt comes down on the string with its own time constant;
        // nothing touches the output. engage() only retargets a one-pole ramp that is currently at
        // 0, and DamperJunction advances that ramp AFTER the sample it scatters, so this very
        // sample is still bit-exactly the undamped one and the damping starts on the next.
        dampers_[index].engage();
        // The watchdog starts its first window here rather than carrying whatever a previous
        // release left behind.
        silencePeak_[index] = 0.0f;
        silenceCount_[index] = 0;
        return;
    }

    const int note = std::clamp(static_cast<int>(event.midiNote), kMinMidiNote, kMaxMidiNote);

    // Retrigger semantics (see the StringNetwork.h scope note): same pitch on a ringing string
    // plucks over the existing state; anything else re-initializes the string at the new pitch.
    // A string mid-release counts as "anything else" -- its tail is already attenuated, so
    // clearing it is inaudible, whereas restoring the release gain to 1 over a still-ringing tail
    // would step it back up.
    const bool pluckOverRinging =
        sounding_[index] && !releasing_[index] && midiNote_[index] == static_cast<std::uint8_t>(note);
    if (!pluckOverRinging) {
        midiNote_[index] = static_cast<std::uint8_t>(note);
        applyStringParams(stringIndex); // retarget f0 first...
        clearStringState(stringIndex);  // ...so reset() snaps the smoothers onto the NEW pitch
    } else {
        // Plucking over a ringing string: the finger comes OFF, so the damper ramps away with the
        // same felt time it arrived with. Ramped, not snapped -- the rails under this junction are
        // full, and a coefficient that jumps while a waveform is passing through it is precisely
        // the click clearStringState() is allowed to make and this path is not. Today the
        // engagement here is always already 0 (only a NoteOff engages it, and a NoteOff clears
        // `sounding_`, which this branch requires), so release() is a no-op that becomes load-
        // bearing the moment P2.6 adds a path that plucks over a damped string.
        dampers_[index].release();
    }

    sounding_[index] = true;
    releasing_[index] = false;
    silencePeak_[index] = 0.0f;
    silenceCount_[index] = 0;

    exciters_[index].setParams(params_.exciter);
    exciters_[index].trigger(event.velocity, resolveNoteParam(event.pluckPosition, params_.exciter.defaultPosition),
                             resolveNoteParam(event.hardness, params_.exciter.defaultHardness));
    tapView_.active_[index] = true;
}

// ------------------------------------------------------------------------------------------
// per-sample loop
// ------------------------------------------------------------------------------------------

template <typename SampleT> void StringNetwork<SampleT>::process(BlockEventQueue& events, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);

    // Resolved once per block, before anything is published: a removed string leaves the trip count
    // only after its ramp has already taken it to silence, so the boundary the block-domain
    // consumer sees never loses a channel that still carries signal.
    updateLoopStringCount();

    const auto stride = static_cast<std::size_t>(maxBlockSize_);
    tapView_.base_ = tapStorage_.data();
    tapView_.stride_ = maxBlockSize_;
    tapView_.numStrings_ = loopStrings_;
    tapView_.numTaps_ = numTapsPerString_;
    tapView_.numSamples_ = count;
    for (int s = 0; s < kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        // "Enabled and ringing at some point during this block": seeded from the state the block
        // starts in, then latched true by any NoteOn the loop consumes. A string whose enable ramp
        // is on its way UP counts too -- its gain can be exactly 0 at the block boundary and
        // non-zero one sample later, and marking it inactive would drop that whole block.
        tapView_.active_[index] = s < loopStrings_ && (sounding_[index] || releasing_[index]) &&
                                  (enableGain_[index] > 0.0f || enableTarget_[index] > 0.0f);
    }

    if (count == 0)
        return; // events stay queued for the next block rather than firing at no sample at all

    for (int s = 0; s < loopStrings_; ++s)
        exciters_[static_cast<std::size_t>(s)].setParams(params_.exciter);

    // Ports past the trip count present no incident wave. Written once per block rather than once
    // per sample: scatter() is only ever handed loopStrings_ ports, so these slots exist to keep
    // the array wholly defined, not to be read.
    for (int s = loopStrings_; s < kMaxStrings; ++s)
        portIncident_[static_cast<std::size_t>(s)] = SampleT(0);

    // Which strings the loop actually ran this sample, as a bitmask, so the tick pass below does
    // not have to recompute the predicate. process() is only ever entered with count > 0 on a
    // prepared instance, so indexing strings_/exciters_ inside the loop needs no emptiness guard.
    std::uint32_t renderedMask = 0;

    // Block-invariant loads hoisted out of the per-sample loop. handleEvent() is called from inside
    // that loop and the compiler must assume a non-inlined member call can touch any member, so
    // every one of these would otherwise be re-loaded from `this` on every sample of every string.
    // None of them is written by handleEvent(): the trip count, the tap count and the three
    // coefficients are all set outside process(), and the vectors are never resized after prepare().
    const int loopStrings = loopStrings_;
    const int numTaps = numTapsPerString_;
    const double smoothingCoeff = positionSmoothingCoeff_;
    const float rampStep = enableRampStep_;
    const int silenceWindow = silenceWindowSamples_;
    SampleT* const tapBase = tapStorage_.data();
    WaveguideString<SampleT>* const strings = strings_.data();
    PluckExciter<SampleT>* const exciters = exciters_.data();
    DamperJunction<SampleT>* const dampers = dampers_.data();

    for (int n = 0; n < count; ++n) {
        // Sample-accurate consumption: every event whose (clamped) offset has been reached fires
        // BEFORE this sample is rendered, so an event at offset k first shows up in sample k.
        while (const NoteEvent* event = events.peek()) {
            const int offset = std::clamp(static_cast<int>(event->sampleOffset), 0, count - 1);
            if (offset > n)
                break;
            handleEvent(*event);
            events.pop();
        }
        renderedMask = 0;

        // Per-sample smoothed fractional pickup taps (docs/plan.md Task P1.5 step 3), now one
        // smoother per (string, tap), plus the damper junction's position (Task P2.3). Both
        // positions move continuously INSIDE the loop, not once per block, which is the first half
        // of what makes them click-free; the second half is WaveguideString's dual-anchor crossfade,
        // which is what the smoothed values are then handed to. Kept as its own tight, branch-free
        // pass rather than folded into the string body below: every slot is an independent one-pole
        // recurrence over two contiguous arrays, which is the one part of this loop a compiler can
        // actually vectorise. Advanced for every string in the trip count whether or not that string
        // is currently sounding, so a string plucked after a position change reads where the pickup
        // and the damper ARE rather than gliding in from where they were when that string last rang.
        for (int s = 0; s < loopStrings; ++s) {
            const auto index = static_cast<std::size_t>(s);
            const auto base = index * static_cast<std::size_t>(kMaxTapsPerString);
            for (int t = 0; t < numTaps; ++t) {
                const std::size_t slot = base + static_cast<std::size_t>(t);
                tapSmoothed_[slot] += smoothingCoeff * (tapTarget_[slot] - tapSmoothed_[slot]);
            }
            damperSmoothed_[index] += smoothingCoeff * (damperTarget_[index] - damperSmoothed_[index]);
        }

        for (int s = 0; s < loopStrings; ++s) {
            const auto index = static_cast<std::size_t>(s);
            const auto base = static_cast<std::size_t>(s) * static_cast<std::size_t>(kMaxTapsPerString);

            // Enable ramp. Linear, one step per sample, landing exactly on the target.
            float gain = enableGain_[index];
            const float gainTarget = enableTarget_[index];
            if (gain != gainTarget) {
                gain =
                    (gain < gainTarget) ? std::min(gainTarget, gain + rampStep) : std::max(gainTarget, gain - rampStep);
                enableGain_[index] = gain;
                if (gain == 0.0f) {
                    // The fade has landed on silence: clear the string rather than leave a muted
                    // tail ringing forever underneath a gain of zero. This is what lets the trip
                    // count drop on a later block, and what makes energyEstimate() tell the truth.
                    clearStringState(s);
                    sounding_[index] = false;
                    releasing_[index] = false;
                }
            }

            // Nothing to render: either the string is fully muted with its state already cleared,
            // or it is enabled but idle (no note ringing, no release tail, no burst in flight). An
            // idle string's rails are zero, so ticking it would compute zeros -- and re-solve its
            // loop length while doing it. Skipping is bit-identical, and it is what keeps eight
            // preallocated strings from costing eight strings' CPU when two are being played.
            //
            // ---- PRECONDITION, and the task that invalidates it -------------------------------
            // This predicate is complete ONLY because nothing outside this loop can put energy into
            // a string's rails. Today that holds: injectFeedback() is a documented no-op until P4,
            // and railAcceptFromBridge() is declared but never called -- the port is driven, its
            // reflected waves are discarded (see setBridgePort()).
            //
            // Task P2.4 ENDS THAT. Wiring the port's reflected waves back into the strings is
            // exactly what makes body coupling bidirectional, and bidirectional coupling means a
            // string that is not sounding can receive energy through the bridge from one that is.
            // That is not an edge case -- it IS sympathetic resonance, the whole point of the
            // feature. On that day this predicate silently kills it: the string receiving bridge
            // energy reports no state, gets skipped, is never ticked, and the energy vanishes with
            // no test failing and no sound to notice, because the "before" is also silence.
            //
            // So P2.4 must extend `live` to include pending incident energy at the bridge port
            // (and P4 the same for injectFeedback), not merely remember to. Written here rather
            // than only in a plan document because here is where it breaks.
            const bool muted = (gain == 0.0f && gainTarget == 0.0f);
            const bool live = !muted && (sounding_[index] || releasing_[index] || exciters[index].isActive());
            if (!live) {
                for (int t = 0; t < numTaps; ++t)
                    tapBase[(base + static_cast<std::size_t>(t)) * stride + static_cast<std::size_t>(n)] = SampleT(0);
                portIncident_[index] = SampleT(0); // presents no incident wave at its bridge slot
                continue;
            }

            renderedMask |= (1u << static_cast<unsigned>(s));

            const SampleT excitation = exciters[index].renderSample();
            if (excitation != SampleT(0))
                strings[index].injectAt(exciters[index].latchedPosition01(), excitation);

            // THE DAMPER, permanently in-line (Task P2.2). Read the two waves arriving at the
            // junction, scatter them through the linear passive two-port, write the difference
            // back. At engagement 0 the scatter is a bit-exact pass-through, so the difference is
            // exactly 0.0 and these three calls cost the ringing string nothing at all -- which is
            // why "CONTRACT: StringNetwork renders the isolated string bit-exactly" still holds
            // with the junction in the loop. There is deliberately NO engagement test around this:
            // a compiled-out damper is a second topology, and the transparency claim is worth
            // exactly as much as the fact that it is measured on the shipping one.
            //
            // The position handed to the seam is the SMOOTHED one, not the junction's own
            // (validated, block-snapshotted) target: through P2.2 the target went in raw and
            // stepped at every block boundary, which with the felt down measured 5.50 dB of
            // click-metric excess against a 3 dB criterion. What the seam does with the smoothed
            // value -- hold it until it has drifted a thirty-second of the string, then
            // amplitude-complementary crossfade to it -- is WaveguideString's business.
            const auto damperPosition = static_cast<float>(damperSmoothed_[index]);
            SampleT fromNut = SampleT(0);
            SampleT fromBridge = SampleT(0);
            strings[index].readJunctionInputs(damperPosition, fromNut, fromBridge);
            SampleT toBridge = SampleT(0);
            SampleT toNut = SampleT(0);
            dampers[index].scatter(fromNut, fromBridge, toBridge, toNut);
            strings[index].writeJunctionOutputs(damperPosition, toBridge, toNut);

            // Multiplying by an exactly-1.0f gain is exact in IEEE-754, so a string that is not
            // ramping is bit-identical to one carrying no envelope at all. Nothing else multiplies
            // the tap any more: a note-off is the damper's work, not a gain's.
            const auto envelope = static_cast<SampleT>(gain);
            for (int t = 0; t < numTaps; ++t) {
                const std::size_t slot = base + static_cast<std::size_t>(t);
                // The tap INDEX is passed, not just the position: each (string, tap) owns its own
                // crossfade anchor pair inside WaveguideString, so a second coil at a different
                // offset gets its own staircase rather than sharing -- and fighting over -- the
                // first one's. Every slot carries the same target today, so this moves no sample.
                const SampleT tap = strings[index].readTapAt(t, static_cast<float>(tapSmoothed_[slot])) * envelope;
                tapBase[slot * stride + static_cast<std::size_t>(n)] = tap;
            }
            const SampleT outgoing = strings[index].railOutgoingAtBridge();
            portIncident_[index] = outgoing * static_cast<SampleT>(gain);

            if (releasing_[index]) {
                // Silence watchdog. Measured on the wave leaving the string at the bridge, not on
                // the tap: the tap can sit on a node of whatever partial is still ringing and
                // report silence that is not there, whereas every mode circulates through the
                // bridge by construction. The peak is taken over a whole window before it is
                // judged, so a zero crossing cannot end a note early -- see kSilenceWindowSeconds.
                silencePeak_[index] = std::max(silencePeak_[index], std::fabs(static_cast<float>(outgoing)));
                if (++silenceCount_[index] >= silenceWindow) {
                    const bool silent = silencePeak_[index] < kSilenceFloor;
                    silencePeak_[index] = 0.0f;
                    silenceCount_[index] = 0;
                    if (silent) {
                        // Inaudible: clear the string rather than tick a dead one forever, so
                        // energyEstimate() tells the truth and the tail costs nothing. The exciter
                        // is deliberately NOT reset -- reseeding its PRNG would make a render
                        // depend on note history.
                        releasing_[index] = false;
                        clearStringState(s);
                    }
                }
            }
        }

        // The port sees every string's outgoing bridge wave and publishes the mono bridge signal.
        // Its reflected waves are not routed back into the strings before P2.4 -- see
        // setBridgePort().
        port_->scatter(portIncident_.data(), portOutgoing_.data(), loopStrings);
        bridgeBuffer_[static_cast<std::size_t>(n)] = port_->bridgeOutput();

        for (int s = 0; s < loopStrings; ++s)
            if ((renderedMask & (1u << static_cast<unsigned>(s))) != 0u)
                strings[static_cast<std::size_t>(s)].tick();
    }
}

// ------------------------------------------------------------------------------------------
// seams and diagnostics
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void StringNetwork<SampleT>::injectFeedback(const SampleT* buffer, int numSamples, float airDelayMs,
                                            float gain) noexcept {
    // P4 seam: nothing happens, and nothing is recorded either -- see the declaration.
    (void)buffer;
    (void)numSamples;
    (void)airDelayMs;
    (void)gain;
}

template <typename SampleT> void StringNetwork<SampleT>::setLosslessTestMode(bool lossless) noexcept {
    lossless_ = lossless;
    for (auto& string : strings_)
        string.setLossBypassed(lossless);
    // docs/plan.md section 2.7: "forwards to strings/dampers/bridge". The damper's resistive
    // junction loss is its only intentional loss, so bypassing it makes the junction transparent
    // -- note that this is a convenience, not a requirement of the tier-2 bound: a dissipative
    // element can never violate a per-block NON-INCREASE, and DamperJunction's passivity is
    // structural (see its header), which the tier-1 grid gates independently.
    for (auto& damper : dampers_)
        damper.setLossBypassed(lossless);
    if (port_ != nullptr)
        port_->setLossBypassed(lossless);
}

template <typename SampleT> Sample64 StringNetwork<SampleT>::energyEstimate() const noexcept {
    if (strings_.empty())
        return 0.0;
    Sample64 total = 0.0;
    // Every string, including strings outside the active count and strings the enable ramp has
    // muted: a muted string that is still ringing genuinely stores that energy, and reporting 0 for
    // it would make this function agree with the output rather than with the physics. A string that
    // has never been excited, or whose ramp or release has completed, contributes exactly 0 because
    // its state was cleared -- so this is honest without being noisy.
    for (int s = 0; s < kMaxStrings; ++s)
        total += strings_[static_cast<std::size_t>(s)].energyEstimate();
    // The rigid termination is memoryless, so it stores nothing; the bridge admittance biquad's
    // storage term joins this sum with BridgeJunction (P2.4).
    return total;
}

template struct StringTapBuffers<float>;  // realtime path
template struct StringTapBuffers<double>; // tier-2 [energy] tests
template class StringNetwork<float>;      // realtime path
template class StringNetwork<double>;     // tier-2 [energy] tests

} // namespace cnpg::dsp

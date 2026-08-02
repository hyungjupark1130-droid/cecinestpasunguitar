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
        // Every string in this network is terminated by the bridge port, not by its own internal
        // rigid reflection, so it must solve its loop one sample shorter and count the sample in
        // flight at the seam (Task P2.4). Declared here, once, for all kMaxStrings strings --
        // including ones outside the active count, so a later setNumStrings() cannot readmit a
        // string that is silently a semitone-and-a-bit sharp.
        strings_[static_cast<std::size_t>(s)].setBridgePortDriven(true);
        // THE STRING'S REST PITCH (Task P2.7), not kMinMidiNote. Six untouched strings all at A0 is
        // not an instrument, and A0's harmonic series contains very nearly everything the other
        // strings play -- see StringNetworkParams::PerString::restMidiNote for the measurement that
        // says how much that cost.
        midiNote_[static_cast<std::size_t>(s)] = static_cast<std::uint8_t>(std::clamp(
            static_cast<int>(params_.perString[static_cast<std::size_t>(s)].restMidiNote), kMinMidiNote, kMaxMidiNote));
        portImpedance_[static_cast<std::size_t>(s)] = strings_[static_cast<std::size_t>(s)].portImpedance();
        lastSolveValid_[static_cast<std::size_t>(s)] = false;
        bridgeTuningConverged_[static_cast<std::size_t>(s)] = true;
        bridgeTuningIterations_[static_cast<std::size_t>(s)] = 0;
        bridgeTuningResidual_[static_cast<std::size_t>(s)] = 0.0;
    }
    bridgeTuningFallbacks_ = 0;
    // A different sample rate makes every phase delay a different number, so no solve cached before
    // this call may be reused.
    ++admittanceGeneration_;

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

    // The Synth retrigger fade is a whole number of samples so the same gesture takes the same time
    // at every rate, and it reaches EXACTLY zero on its last faded sample -- linear for the same
    // reason the enable ramp is (a one-pole never arrives, and "the string is silent, clear it" has
    // to become true at a sample somebody can name).
    synthFadeSamples_ = std::max(1, static_cast<int>(std::lround(kSynthFadeSeconds * sampleRate_)));
    synthFadeStep_ = 1.0f / static_cast<float>(synthFadeSamples_);

    if (port_ == nullptr)
        port_ = &internalBridge_;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);
    port_->setAdmittance(params_.bridge);

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
        ringing_[static_cast<std::size_t>(s)] = false;
        portIncident_[static_cast<std::size_t>(s)] = SampleT(0);
        portOutgoing_[static_cast<std::size_t>(s)] = SampleT(0);
        // A Synth retrigger fade in flight is abandoned, pending note and all: a reset instance
        // must be indistinguishable from a freshly prepared one, and a note that fires 2 ms after a
        // reset is exactly such a difference. The event that scheduled it is already gone from the
        // queue, so there is nothing to re-deliver and nothing that could tell.
        fadeGain_[static_cast<std::size_t>(s)] = 1.0f;
        fadeSteps_[static_cast<std::size_t>(s)] = 0;
        fadeActive_[static_cast<std::size_t>(s)] = false;
        pendingNoteOff_[static_cast<std::size_t>(s)] = false;
        pendingNote_[static_cast<std::size_t>(s)] = NoteEvent{};
    }
    previousRenderedMask_ = 0;
    // Cleared AFTER the loop above, not before: clearStringState() re-solves each string's bridge
    // tuning on its way back to the rest pitch, so anything counted during a reset belongs to the
    // instance being discarded and not to the one that comes out of it. Same contract as
    // WaveguideString's reflection-tick counters -- a reset instance is indistinguishable from a
    // freshly prepared one, and a nonzero fallback count is exactly such a difference.
    bridgeTuningFallbacks_ = 0;

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

    // The count enters the junction's sigma, so it enters every string's tuning (Task P2.7). A
    // small term -- one unit port in a sigma of hundreds -- and re-solving is cheap, so it is done
    // rather than argued about.
    if (clamped != previous)
        for (int s = 0; s < kMaxStrings; ++s)
            applyStringParams(s);

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
    // The admittance generation is bumped BEFORE params_ is overwritten, and only when the value
    // really differs -- this is the whole of what makes the per-block setParams cascade cheap again
    // (see lastSolvedTargetHz_). Field-wise rather than memcmp: BridgeAdmittanceParams is trivially
    // copyable but padding bytes are not required to be equal, so a memcmp could report a change that
    // did not happen and re-solve every block anyway.
    if (p.bridge.couplingStrength != params_.bridge.couplingStrength ||
        p.bridge.resonanceHz != params_.bridge.resonanceHz || p.bridge.damping != params_.bridge.damping)
        ++admittanceGeneration_;
    params_ = p;
    // One target per (string, tap). Every slot carries the same global pickupPosition01 today; the
    // array exists so a per-coil offset has somewhere to land without another rewrite of the loop.
    const double target = clampd(static_cast<double>(params_.pickupPosition01), 0.0, 1.0);
    for (double& slot : tapTarget_)
        slot = target;
    refreshEnableTargets();
    // The bridge admittance travels on StringNetworkParams (docs/plan.md section 2.7) and reaches
    // whatever port is attached through IBridgePort::setAdmittance, which is why that method is on
    // the interface rather than only on BridgeJunction: this call site cannot know which
    // implementation it is holding, and must not have to.
    //
    // IT GOES FIRST, and the order is load-bearing from Task P2.7. applyStringParams() below asks
    // the port what phase delay it contributes, and the answer has to be the one for the admittance
    // the caller just set -- otherwise every parameter change would tune the strings for the
    // PREVIOUS load and the compensation would run one setParams() behind the instrument for ever.
    if (port_ != nullptr)
        port_->setAdmittance(params_.bridge);
    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);
}

template <typename SampleT> void StringNetwork<SampleT>::setBridgePort(IBridgePort<SampleT>& port) noexcept {
    port_ = &port;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);
    port_->setAdmittance(params_.bridge);
    port_->reset();
    // A substituted port is a different LOAD, so it is a different tuning (Task P2.7), and no solve
    // cached against the port it replaced may be reused. Re-solving here is what keeps Q17's
    // "StringNetwork must not be able to tell the difference" true of pitch as well as of scattering
    // -- a fallback bus swapped in silently at the same seam would otherwise inherit whatever
    // compensation the junction it replaced had asked for.
    ++admittanceGeneration_;
    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);
}

template <typename SampleT> bool StringNetwork<SampleT>::stringHasState(int stringIndex) const noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);
    // ringing_ is here for Task P2.4's sake: a string driven purely through the bridge is neither
    // sounding nor releasing and its exciter is idle, yet it is emphatically not silent, and every
    // caller of this helper (the enable-ramp snap, the position-smoother snap) is asking exactly
    // "would a discontinuity here be audible?".
    if (sounding_[index] || releasing_[index] || ringing_[index])
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
    // THE STRING GOES BACK TO ITS REST PITCH (Task P2.7) unless the caller names another one. On a
    // real instrument, releasing a fretted note returns the string to open; a string left tuned to
    // the last note anyone happened to play it would keep resonating sympathetically at that pitch
    // for the rest of the session, which is not what a guitar does.
    clearStringState(stringIndex,
                     static_cast<int>(params_.perString[static_cast<std::size_t>(stringIndex)].restMidiNote));
}

template <typename SampleT>
void StringNetwork<SampleT>::clearStringState(int stringIndex, int leaveTunedToMidiNote) noexcept {
    if (strings_.empty())
        return;
    const auto index = static_cast<std::size_t>(stringIndex);

    // THE PITCH IS PART OF WHAT IS BEING CLEARED, and it moves here rather than at the call sites
    // because it obeys the same invariant they do: retuning a string is free exactly when the waves
    // it scatters are zeros, which is the property this function establishes. A fresh note-on names
    // the note it is about to pluck (so the snap below lands on it instead of gliding to it from
    // the open string); everything else -- reset(), the enable ramp landing, the silence watchdog --
    // takes the default and gets the rest pitch.
    //
    // BEFORE the reset, not after: applyStringParams() retargets the f0 smoother (and the bridge
    // tuning solve that hangs off it), and WaveguideString::reset() is what SNAPS every smoother
    // onto its target. In the other order the string would glide over 8 ms from a set of rails that
    // were just zeroed -- inaudible, and a difference from a freshly prepared instance that nothing
    // could justify.
    midiNote_[index] = static_cast<std::uint8_t>(std::clamp(leaveTunedToMidiNote, kMinMidiNote, kMaxMidiNote));
    applyStringParams(stringIndex);

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
    // The rails are zero now, so the string carries no motion -- and a stale `ringing_` would keep
    // it in the loop for ever and keep the whole network from ever reaching quiescence.
    ringing_[index] = false;
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

    // ---- THE BRIDGE TUNING SOLVE (Task P2.7, ADR 0007 D1/D6) ----------------------------------
    //
    // Here, and only here: this function is the single place a string's pitch target or its load can
    // change, so it is the single place the compensation can go stale. It runs on a parameter change
    // and on a note event -- O(1) per string, a handful of trig calls -- and NEVER per sample, which
    // is what ADR 0007 D6's "offline at parameter-change time, never on the audio path" asks for.
    // The result is handed to the string as a smoother TARGET, so the change itself is a glide.
    //
    // Evaluated at the string's own BENT target, not at the note's nominal: a bend is a pitch and
    // the port's phase delay is a function of pitch. The plugin calls setParams() every block, so a
    // wheel ride re-solves at block rate and the compensation tracks the bend.
    //
    // numStrings_ rather than loopStrings_ is what sigma is formed over. The two differ only while a
    // count REDUCTION's removed strings are still ramping out, and the difference is one unit port
    // in a sigma of hundreds to thousands -- under 0.01 cents at the shipping impedances. What
    // numStrings_ buys is that the answer is a function of the configuration the caller asked for
    // rather than of how far a 10 ms ramp has got, which is what makes it reproducible.
    if (port_ != nullptr) {
        const double targetHz = static_cast<double>(p.f0Hz) * std::exp2(static_cast<double>(p.bendSemitones) / 12.0);
        // SKIPPED WHEN NOTHING IT DEPENDS ON HAS MOVED -- see lastSolvedTargetHz_ for why this is the
        // ADR's claim rather than a shortcut. The comparison is exact equality on purpose: these are
        // the very doubles the previous solve consumed, so "unchanged" means bitwise unchanged and a
        // tolerance would only introduce a threshold nobody could justify.
        const bool unchanged = lastSolveValid_[index] && lastSolvedTargetHz_[index] == targetHz &&
                               lastSolvedAdmittanceGen_[index] == admittanceGeneration_ &&
                               lastSolvedPortCount_[index] == numStrings_;
        if (!unchanged) {
            const BridgeTuningSolution solved =
                solveBridgeTuning(*port_, stringIndex, targetHz, sampleRate_, numStrings_);
            strings_[index].setBridgePhaseDelaySamples(static_cast<float>(solved.phaseDelaySamples));
            bridgeTuningConverged_[index] = solved.converged;
            bridgeTuningIterations_[index] = solved.iterations;
            bridgeTuningResidual_[index] = solved.residualCents;
            if (!solved.converged)
                ++bridgeTuningFallbacks_;
            lastSolvedTargetHz_[index] = targetHz;
            lastSolvedAdmittanceGen_[index] = admittanceGeneration_;
            lastSolvedPortCount_[index] = numStrings_;
            lastSolveValid_[index] = true;
        }
    }
}

template <typename SampleT> double StringNetwork<SampleT>::bridgeCompensationSamples(int stringIndex) const noexcept {
    if (strings_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0;
    return strings_[static_cast<std::size_t>(stringIndex)].currentBridgePhaseDelaySamples();
}

template <typename SampleT> bool StringNetwork<SampleT>::bridgeTuningConverged(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return true;
    return bridgeTuningConverged_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> int StringNetwork<SampleT>::bridgeTuningIterations(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0;
    return bridgeTuningIterations_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> double StringNetwork<SampleT>::bridgeTuningResidualCents(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0;
    return bridgeTuningResidual_[static_cast<std::size_t>(stringIndex)];
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

template <typename SampleT> float StringNetwork<SampleT>::stringF0Hz(int stringIndex) const noexcept {
    if (strings_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0f;
    return strings_[static_cast<std::size_t>(stringIndex)].currentF0Hz();
}

template <typename SampleT> int StringNetwork<SampleT>::retuneRampSamplesRemaining(int stringIndex) const noexcept {
    if (strings_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0;
    return strings_[static_cast<std::size_t>(stringIndex)].retuneRampSamplesRemaining();
}

template <typename SampleT> float StringNetwork<SampleT>::retriggerFadeGain(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return 1.0f;
    return fadeGain_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> bool StringNetwork<SampleT>::retriggerFadeActive(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= kMaxStrings)
        return false;
    return fadeActive_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> void StringNetwork<SampleT>::setRetuneRampSeconds(double seconds) noexcept {
    retuneRampSeconds_ = (seconds > 0.0) ? seconds : 0.0;
}

// ------------------------------------------------------------------------------------------
// event consumption
// ------------------------------------------------------------------------------------------

// The excitation itself, with no decision in it: whatever the retrigger paths above decided about
// the rails, this is what puts the new note into them.
template <typename SampleT> void StringNetwork<SampleT>::excite(int stringIndex, const NoteEvent& event) noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);

    sounding_[index] = true;
    releasing_[index] = false;
    silencePeak_[index] = 0.0f;
    silenceCount_[index] = 0;

    exciters_[index].setParams(params_.exciter);
    exciters_[index].trigger(event.velocity, resolveNoteParam(event.pluckPosition, params_.exciter.defaultPosition),
                             resolveNoteParam(event.hardness, params_.exciter.defaultHardness));
    tapView_.active_[index] = true;
}

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
        // A note-off for the note a Synth fade has not started yet. Without this the fade lands, the
        // pending NoteOn fires into a string nobody will ever release, and the note is STUCK: the
        // allocator has already moved ownership to the pending note, so this is the only note-off
        // that note will ever get. Deferred to the landing rather than applied now, where it would
        // damp the old state that is about to be discarded anyway.
        if (fadeActive_[index]) {
            pendingNoteOff_[index] = true;
            return;
        }
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

    // A NoteOn arriving inside a Synth fade REPLACES what the fade is going to land on rather than
    // restarting the fade -- restarting would extend the silence a third note could extend again,
    // and the string is already on its way to zero either way.
    if (fadeActive_[index]) {
        pendingNote_[index] = event;
        pendingNoteOff_[index] = false;
        return;
    }

    const int note = std::clamp(static_cast<int>(event.midiNote), kMinMidiNote, kMaxMidiNote);

    // -------------------------------------------------------------------------------------------
    // "OWNS A NOTE" IS WHAT MAKES A NoteOn A RETRIGGER (Task P2.4's entry condition for P2.6)
    // -------------------------------------------------------------------------------------------
    // Since the bridge became bidirectional a string can be RINGING with no note of its own -- that
    // is sympathetic resonance, and it is true of every string on the instrument a moment after the
    // first chord. So "has state" and "is playing something" came apart, and the retrigger paths
    // below are about the second one:
    //
    //   - A string that is SOUNDING owns a note. It is being RE-struck: there is a live note to be
    //     continuous with (Physical) or to be replaced cleanly (Synth), and that is what
    //     RetriggerMode decides between.
    //   - A string that owns none is being struck for the FIRST time, whatever else it may be
    //     carrying -- a release tail, a neighbour's sympathetic drive, both. There is no live note.
    //     Retuning that motion from whatever pitch the string was last left at -- kMinMidiNote for
    //     a string nobody has played -- would glide inaudible content across two and a half octaves
    //     to make a point no listener can hear, and the same fresh attack would then sound
    //     different depending on whether a neighbour happened to be ringing. The state is cleared,
    //     exactly as it was through P2.3.
    //
    // A RELEASED NOTE IS OVER, AND THAT IS WHY THIS READS `sounding_` ALONE. It shipped as
    // `sounding_ || releasing_`, which made the two levels disagree for the whole span between a
    // note-off and its tail dying -- the most common state in ordinary playing, and hundreds of
    // milliseconds to seconds long, because `releasing_` is cleared only by the silence watchdog
    // (below in process(): the outgoing bridge wave must stay under kSilenceFloor for a whole
    // kSilenceWindowSeconds window). NoteAllocator releases its ownership the instant it EMITS the
    // note-off, so through that whole span the allocator handed out a string it believed was fresh
    // and this function took the retrigger path -- plucking into a felt still most of the way
    // closed, which then ramped open on the damper's own time constant. Measured on the P1 corpus:
    // phrase 01 (note after note, nothing overlapping) lost 4.88 dB of RMS against the pre-P2.6
    // parent, reaching 10.99 dB in individual 2 s windows, and isolating the two halves attributed
    // -5.33 dB of that to the ramped-off felt against +0.45 dB gained by keeping the rails. The
    // retained tail was worth 0.45 dB and sat about 39 dB down; the price was an uncontrolled,
    // articulation-dependent level swing on the commonest gesture there is.
    //
    // NoteAllocator asks the SAME question of the same event stream -- `owned_[i]`, set by the
    // note-on it emits and cleared by the note-off it emits (dsp/include/cnpg/dsp/NoteAllocator.h).
    // The two can therefore differ only where an event was DROPPED between them, and every such
    // drop is counted: queueOverflowCount() (the event never reached this queue) and
    // unaddressableNoteOffCount() (the string left the active count or was muted, so this function
    // would have discarded the event at its own early returns above). There is no path that lets
    // them disagree silently, which is exactly the property the shipped `|| releasing_` destroyed.
    //
    // WHAT THIS COSTS, MEASURED NOT ASSUMED, because it is a real cost and not a free win: a
    // re-strike now CLEARS a string that is still audibly releasing, and the earlier the re-strike
    // the louder the content thrown away. tests/dsp/RetriggerModeTests.cpp sweeps note-off age from
    // 5 ms to 500 ms and reports the discarded level and the click excess at each, against a
    // genuine A/B control and a level-placed hard cut. Clearing a sympathetically ringing string is
    // quantified in the same file against the note that replaces it.
    const bool ownsNote = sounding_[index];

    if (!ownsNote) {
        // The state goes and the string comes back tuned to the note about to be plucked -- one
        // call, because the retune and the clear are the same operation and doing them separately
        // is what let the pitch and the rails disagree (Task P2.7). The NOTE is named here rather
        // than defaulted, so the snap inside lands on the new pitch instead of gliding to it from
        // the open string.
        clearStringState(stringIndex, note);
        excite(stringIndex, event);
        return;
    }

    if (params_.retriggerMode == RetriggerMode::Synth) {
        // Fade first, clear at the bottom of it, then re-init and re-excite -- see landSynthFade().
        pendingNote_[index] = event;
        pendingNoteOff_[index] = false;
        fadeActive_[index] = true;
        fadeSteps_[index] = synthFadeSamples_;
        return;
    }

    // -------------------------------------------------------------------------------------------
    // PHYSICAL. The finger comes off the felt, and the pitch (if it moved) glides on kept rails.
    // -------------------------------------------------------------------------------------------
    // release(), ramped and not snapped: the rails under this junction are full, and a scattering
    // coefficient that jumps while a waveform is passing through it is precisely the click
    // clearStringState() is allowed to make and this path is not.
    //
    // IT IS A NO-OP TODAY, AND SAYING SO IS THE POINT. Reaching here means `sounding_[index]` is
    // true, and the only thing that engages a damper is the note-off branch above -- which clears
    // `sounding_` on the same line -- or landSynthFade()'s pending-note-off, which does the same.
    // So the engagement is provably already 0 whenever this runs, and the call moves nothing. It
    // shipped in fixes wave 1 with a comment claiming it had "become load-bearing, because a
    // restrike on a RELEASING string reaches here instead of clearing it"; the wave-2 predicate
    // above sends exactly that restrike down the fresh path instead, so the claim is retracted and
    // the call is back to being what its P2.2 comment originally called it -- the correct thing to
    // do the moment any future path plucks over an engaged damper, kept because the alternative is
    // a silent dependency on a two-hop argument about which flags can be true together.
    // tests/dsp/RetriggerModeTests.cpp asserts the engagement is 0 across the restrike rather than
    // leaving that argument unchecked.
    dampers_[index].release();

    if (midiNote_[index] != static_cast<std::uint8_t>(note)) {
        // ***** REFUSED, WITH A DERIVATION: the plan's "damper choke (fast engage())" *****
        //
        // docs/plan.md P2.6 specifies "pitch change performs damper choke (fast engage()), a retune
        // ramp completing within 30 ms, then re-excitation". The retune ramp and the kept rails
        // ship. The choke does not, because it cannot do the job it is named for.
        //
        // A damper is a LINEAR two-port (DamperJunction.h derives its 2x2 S from Kirchhoff; there is
        // no nonlinearity anywhere in this loop). The string's output after the note-on is, by
        // superposition, the free response of the state that was already circulating plus the
        // response to the exciter's injection -- superposition gives ADDITIVITY, and that is all it
        // gives. Equal attenuation does not follow from it, and is not exactly true here: a damper
        // attenuates per PASS through its junction, and during the first round trip (9.1 ms at
        // 110 Hz) the fresh injection has passed fewer times than the content already circulating,
        // so the balance does shift for that long. TO FIRST ORDER, once a round trip has elapsed,
        // both are distributed around the same loop passing the same junction at the same rate and
        // the attenuation they see is the same. So a choke applied at or after the re-excitation
        // cannot make the old content quieter RELATIVE to the new note in any lasting way, which is
        // the only thing "choke" could mean here; all it can do is make the whole re-attack quieter.
        // The transient is real, which is why this paragraph is not the whole refusal. The only
        // asymmetry available outright is TIME -- choke first, excite afterwards -- and the plan's
        // own "then" reads that way, at a cost of up to 30 ms of latency on every legato note, which
        // is not a playable instrument.
        //
        // *** THE REFUSAL RESTS ON COST AND LATENCY, and its evidence is thinner than it was. ***
        // The original form measured both halves of the trade on one pair of renders: the felt took
        // 1.4925 dB (amplitude) out of the old note and charged the re-attack 1.22328 dB for it,
        // netting 0.269221 dB on the balance -- the only quantity a "choke" could be for. Wave 2's
        // ownership ruling makes a released note OVER, so the arms of that comparison stopped
        // differing only in the felt; the attack-cost and net-balance figures were taken at 0eb52b9
        // and are NOT reproducible in this tree. What is still measured, in
        // tests/dsp/RetriggerModeTests.cpp ("a retrigger damper choke has no state left to act on,
        // and buying one costs elapsed time"), is the suppression half -- 10.67 ms of felt takes
        // -1.4925 dB out of a ringing string, reproduced exactly -- which is what the plan would
        // have to spend latency on every legato note to obtain. That, plus the latency, is the
        // refusal. The same case asserts the engagement is 0 across this branch, which is a
        // REGRESSION GUARD (a build inserting the choke fails on the sample it was inserted) and
        // NOT an argument against the clause: the clause proposes to add the engagement, so
        // observing that there is none today restates this control flow rather than refuting it.
        // Fixes wave 2 characterised that observation as "stronger than refusing it on cost"; fixes
        // wave 3 withdraws the characterisation and leaves the guard.
        //
        // What the choke was reaching for is real -- a fast retune of a full rail sweeps the old
        // content -- and it is handled where it belongs: by the ramp being short and landing, and by
        // the click gate over exactly that transition.
        midiNote_[index] = static_cast<std::uint8_t>(note);
        applyStringParams(stringIndex); // retarget f0; the RAILS ARE NOT TOUCHED
        strings_[index].beginRetuneRamp(retuneRampSeconds_);
    }

    excite(stringIndex, event);
}

// The bottom of a Synth retrigger fade: the string is at exactly zero gain, so clearing it is free
// (the same invariant clearStringState() documents -- coefficients may jump while zeros travel),
// and the note that has been waiting fires on this sample.
template <typename SampleT> void StringNetwork<SampleT>::landSynthFade(int stringIndex) noexcept {
    const auto index = static_cast<std::size_t>(stringIndex);
    const NoteEvent pending = pendingNote_[index];

    fadeActive_[index] = false;
    fadeSteps_[index] = 0;
    fadeGain_[index] = 1.0f;

    // Same one call as the fresh-note path above, and for the same reason: the clear and the retune
    // onto the pending note are one operation (Task P2.7).
    clearStringState(stringIndex, static_cast<int>(pending.midiNote));
    excite(stringIndex, pending);

    if (pendingNoteOff_[index]) {
        // The host released the note before it started. Applied here rather than dropped, so a
        // zero-length note is a zero-length note and not a stuck one.
        pendingNoteOff_[index] = false;
        sounding_[index] = false;
        releasing_[index] = true;
        dampers_[index].engage();
        silencePeak_[index] = 0.0f;
        silenceCount_[index] = 0;
    }
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
        // starts in, then latched true by any NoteOn the loop consumes and by any non-zero tap
        // sample the loop writes (see the end of process(), and why that latch is load-bearing
        // under bidirectional coupling). A string whose enable ramp is on its way UP counts too:
        // its gain can be exactly 0 at the block boundary and non-zero one sample later, and
        // marking it inactive would drop that whole block.
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
    // Strings that produced a NON-ZERO tap sample somewhere in this block. This -- not "was
    // rendered" -- is what the domain boundary's isActive() reports, and the distinction is the
    // whole of StringTapBuffers' documented contract: "a string that is silent for the whole block
    // reports false and every one of its channels is all zeros, so a consumer may skip it entirely
    // rather than summing silence". Under bidirectional coupling the loop legitimately RENDERS
    // strings that are still silent (it has to, or they could never start), so the two questions
    // came apart at Task P2.4 and the boundary must answer the one its consumers ask.
    std::uint32_t blockAudibleMask = 0;

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

        // CAN THE BRIDGE HAND ENERGY TO A STRING THAT HAS NONE OF ITS OWN THIS SAMPLE? Two ways,
        // and both have to be asked or the idle-string skip below silently deletes sympathetic
        // resonance (see the PRECONDITION note there):
        //   - some string rendered on the PREVIOUS sample, so it fed the junction and the junction
        //     will scatter that energy across every port this sample; or
        //   - every string has already fallen silent but the junction's own resonator has not, so
        //     it is still driving them out of its own store.
        // Written as a network-level question rather than a per-string one because the bridge node
        // is a single point: if it is moving, it is moving under all of them.
        const bool bridgeMayDrive = (previousRenderedMask_ != 0u) || !port_->isQuiescent();

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

            // The Synth retrigger fade (Task P2.6). Linear, one step per sample, reaching EXACTLY
            // zero on its last faded sample -- and the sample AFTER that is where the state is
            // cleared and the pending note fires, because a clear is only free while the waves it
            // scatters are zeros and the fade is what makes them so.
            if (fadeActive_[index]) {
                if (fadeSteps_[index] > 0) {
                    --fadeSteps_[index];
                    fadeGain_[index] = static_cast<float>(fadeSteps_[index]) * synthFadeStep_;
                } else {
                    landSynthFade(s);
                }
            }

            // Nothing to render: either the string is fully muted with its state already cleared,
            // or it is enabled but idle (no note ringing, no release tail, no burst in flight). An
            // idle string's rails are zero, so ticking it would compute zeros -- and re-solve its
            // loop length while doing it. Skipping is bit-identical, and it is what keeps eight
            // preallocated strings from costing eight strings' CPU when two are being played.
            //
            // ---- THE PRECONDITION THAT TASK P2.4 PAID -----------------------------------------
            // Through P2.3 this predicate read only the string's own state, and it was complete
            // ONLY because nothing outside this loop could put energy into a string's rails: the
            // port was driven but its reflected waves were discarded, and injectFeedback() is a
            // documented no-op until P4. P2.1 wrote the warning here, naming this task.
            //
            // Wiring the port's reflected waves back in is what makes body coupling bidirectional,
            // and bidirectional coupling means a string that is not sounding can receive energy
            // through the bridge from one that is. That is not an edge case -- it IS sympathetic
            // resonance, the whole point of the feature. Left as it was, this predicate would have
            // killed it in silence: the string receiving bridge energy reports no state of its own,
            // gets skipped, is never ticked, and the energy vanishes with no test failing and
            // nothing to hear, because the "before" is also silence. The tempting fix from the
            // outside would then have been to raise couplingStrength until something leaked
            // through.
            //
            // So `live` now has two more terms. `ringing_` is the string's own bridge-driven
            // motion, latched below and cleared with its state by the silence watchdog.
            // `bridgeMayDrive` is the network-level question computed once per sample above: it is
            // what lets a string with EXACTLY zero state be ticked, which is the only way it can
            // ever acquire any. tests/dsp/CoupledStringsTests.cpp asserts that directly -- a
            // quiescent string's own stringEnergyEstimate() must rise from exactly 0 with no note
            // of its own -- rather than inferring it from a level that got louder.
            // (P4 owes injectFeedback() the same treatment, for the same reason.)
            const bool muted = (gain == 0.0f && gainTarget == 0.0f);
            const bool live = !muted && (sounding_[index] || releasing_[index] || ringing_[index] ||
                                         fadeActive_[index] || exciters[index].isActive() || bridgeMayDrive);
            if (!live) {
                // M2 (P2.4 review), stated where it happens: a string that is in the trip count but
                // not live presents a ZERO incident wave while still occupying a port, and a port
                // presenting zero is a PERFECT ABSORBER -- the junction hands it b = v and that
                // energy is discarded, because the string is not ticked and never accepts it. So
                // the bridge's effective damping depends on how many strings are currently idle or
                // disabled: six live strings load it differently from one live and five muted.
                // That is passive (energy only ever leaves) and it is not a defect, but it IS an
                // audible coupling between the string count and the instrument's decay, and P2.6
                // and the P2.8 listening pass should both know it is there.
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

            // Multiplying by an exactly-1.0f gain is exact in IEEE-754, so a string that is neither
            // ramping nor mid-retrigger-fade is bit-identical to one carrying no envelope at all --
            // which is why adding the Synth fade's factor here moves no sample of any render that
            // does not use it. Nothing else multiplies the tap: a note-off is the damper's work.
            const auto envelope = static_cast<SampleT>(gain * fadeGain_[index]);
            for (int t = 0; t < numTaps; ++t) {
                const std::size_t slot = base + static_cast<std::size_t>(t);
                // The tap INDEX is passed, not just the position: each (string, tap) owns its own
                // crossfade anchor pair inside WaveguideString, so a second coil at a different
                // offset gets its own staircase rather than sharing -- and fighting over -- the
                // first one's. Every slot carries the same target today, so this moves no sample.
                const SampleT tap = strings[index].readTapAt(t, static_cast<float>(tapSmoothed_[slot])) * envelope;
                tapBase[slot * stride + static_cast<std::size_t>(n)] = tap;
                if (tap != SampleT(0))
                    blockAudibleMask |= (1u << static_cast<unsigned>(s));
            }
            const SampleT outgoing = strings[index].railOutgoingAtBridge();
            // B5 (P2.1): a disabled or ramping string presents its bridge incident wave SCALED by
            // the enable gain, so it reaches exactly zero only once fully muted. The reflection is
            // handed back UNSCALED -- see the accept pass below for why one factor of gain is both
            // sufficient for passivity and the right amount. (This comment claimed the opposite
            // until the review caught it: it still described the two-factor version that measurement
            // rejected.) The Synth retrigger fade rides the same factor, so a string on its way to
            // a state clear stops driving its NEIGHBOURS through the bridge as it goes quiet, rather
            // than feeding them right up to the sample its rails are zeroed.
            portIncident_[index] = outgoing * envelope;

            // "This string carries motion", whether or not anyone played it. Latched here because
            // this is the one place that knows: `outgoing` is the wave leaving the string at the
            // bridge, and every mode of the string passes through it by construction (which is
            // also why the watchdog measures it rather than the tap).
            if (outgoing != SampleT(0))
                ringing_[index] = true;

            // The silence watchdog now runs for ANY non-sounding string that is in the loop, not
            // just a releasing one: a sympathetically driven string has to be able to leave the
            // loop too, or the network never reaches quiescence, energyEstimate() never returns to
            // zero, and eight strings tick for ever after one note.
            if (!sounding_[index]) {
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

        // THE COUPLING (Task P2.4). Every string's outgoing bridge wave in, one shared bridge
        // velocity and N reflections out. This is the single point at which six strings stop being
        // six instruments -- and the reason the whole task is one commit rather than six.
        port_->scatter(portIncident_.data(), portOutgoing_.data(), loopStrings);
        bridgeBuffer_[static_cast<std::size_t>(n)] = port_->bridgeOutput();

        for (int s = 0; s < loopStrings; ++s)
            if ((renderedMask & (1u << static_cast<unsigned>(s))) != 0u) {
                const auto index = static_cast<std::size_t>(s);
                // Handed back UNSCALED, and that is the deliberate choice rather than the obvious
                // one. B5 says a ramping string "presents its bridge incident wave scaled by the
                // enable gain" -- one scaling, on the way in -- and one is all passivity needs: the
                // junction is a contraction on whatever it is given, so ||b|| <= ||g a|| <= ||a||
                // already. Scaling the reflection as well would square the factor and damp the
                // string's LOOP twice as hard as its output during a 10 ms mute, which costs level
                // the string never gets back when a count reduction is reversed mid-ramp. Measured
                // on P2.1's count-churn click case against its 3 dB gate: 3.74 dB with the
                // reflection scaled too, 2.42 dB with it unscaled -- and the DIRECT tap-continuity
                // assertion in that same case was clean in both, i.e. it was lost level and not a
                // discontinuity. One factor of g per round trip is unavoidable and correct: a
                // string that presents less to the bridge gets less back.
                //
                // Then tick(): the accept is consumed by the very next tick, and a tick that finds
                // none falls back to the internal rigid -1, which is the silent failure
                // WaveguideString's liveness counters exist to make loud.
                strings[index].railAcceptFromBridge(portOutgoing_[index]);
                strings[index].tick();
            }

        previousRenderedMask_ = renderedMask;
    }

    // Any string that put a non-zero sample on the boundary was ringing during this block, whatever
    // its note flags say. This latch is what makes sympathetic resonance survive the boundary: a
    // bridge-driven string is neither sounding nor releasing, so the seed above reports false for
    // it, and PickupTap -- which honours isActive() -- would drop its channel and turn the feature
    // into silence that looks like a tuning problem. tapBuffers() is only read after process()
    // returns, so writing it here is the same boundary the seed was.
    for (int s = 0; s < kMaxStrings; ++s)
        if ((blockAudibleMask & (1u << static_cast<unsigned>(s))) != 0u)
            tapView_.active_[static_cast<std::size_t>(s)] = true;
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
    // ...and the bridge. NOTE the asymmetry with the damper, because it is what makes the tier-2
    // network case non-vacuous where the damper-motion one was not (docs/plan.md section 4.2,
    // P2.3 amendment): bypassing the DAMPER's loss makes it transparent, so the element under test
    // disappears, whereas bypassing the BRIDGE's loss removes only the dashpot and leaves a
    // lossless mass-spring resonator that still couples every string to every other one. The
    // coupling -- the thing tier 2 is there to gate -- is fully present in lossless mode.
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
    // ...and the junction's own store (Task P2.4). A bridge admittance is a mass and a spring, and
    // a mass and a spring hold energy: leaving them out would make this functional fluctuate by
    // however much is currently in the bridge -- which for a resonator sitting where the
    // instrument's fundamentals are is not a rounding term. It is also the reason IBridgePort has a
    // storageEnergy(): the network cannot know which implementation it is holding, and a
    // fallback bus (P2.5) would store energy in different states with a different closed form.
    if (port_ != nullptr)
        total += port_->storageEnergy();
    return total;
}

template <typename SampleT> Sample64 StringNetwork<SampleT>::stringEnergyEstimate(int stringIndex) const noexcept {
    if (strings_.empty() || stringIndex < 0 || stringIndex >= kMaxStrings)
        return 0.0;
    return strings_[static_cast<std::size_t>(stringIndex)].energyEstimate();
}

template <typename SampleT> unsigned long long StringNetwork<SampleT>::bridgeDrivenTicks() const noexcept {
    unsigned long long total = 0;
    for (const auto& string : strings_)
        total += string.bridgeReflectionTicks();
    return total;
}

template <typename SampleT> unsigned long long StringNetwork<SampleT>::unbridgedTicks() const noexcept {
    unsigned long long total = 0;
    for (const auto& string : strings_)
        total += string.internalReflectionTicks();
    return total;
}

template struct StringTapBuffers<float>;  // realtime path
template struct StringTapBuffers<double>; // tier-2 [energy] tests
template class StringNetwork<float>;      // realtime path
template class StringNetwork<double>;     // tier-2 [energy] tests

} // namespace cnpg::dsp

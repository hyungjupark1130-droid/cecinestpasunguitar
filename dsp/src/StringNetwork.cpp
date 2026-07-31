#include "cnpg/dsp/StringNetwork.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::dsp {

namespace {

// -60 dB expressed as the number of time constants of a one-pole decay, so kReleaseSeconds is
// read as "time to inaudibility" rather than as a time constant.
constexpr double kMinus60dBTimeConstants = 6.907755278982137; // ln(1000)

constexpr double kCentsPerSemitone = 100.0;

double clampd(double v, double lo, double hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

float clampf(float v, float lo, float hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

// Equal temperament, A4 = 440 Hz -- the same expression the [tuning] and [regression] harnesses
// use, so a note rendered through the network lands on exactly the frequency they measure.
double midiNoteToHz(int midiNote) noexcept { return 440.0 * std::exp2((static_cast<double>(midiNote) - 69.0) / 12.0); }

} // namespace

// ------------------------------------------------------------------------------------------
// StringTapBuffers
// ------------------------------------------------------------------------------------------

template <typename SampleT> const SampleT* StringTapBuffers<SampleT>::channel(int stringIndex) const noexcept {
    if (base_ == nullptr || stringIndex < 0 || stringIndex >= numStrings_)
        return nullptr;
    return base_ + static_cast<std::ptrdiff_t>(stringIndex) * static_cast<std::ptrdiff_t>(stride_);
}

template <typename SampleT> bool StringTapBuffers<SampleT>::isActive(int stringIndex) const noexcept {
    if (stringIndex < 0 || stringIndex >= numStrings_)
        return false;
    return active_[static_cast<std::size_t>(stringIndex)];
}

template <typename SampleT> int StringTapBuffers<SampleT>::numStrings() const noexcept { return numStrings_; }

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
    for (int s = 0; s < kMaxStrings; ++s) {
        strings_[static_cast<std::size_t>(s)].prepare(sampleRate_, maxBlockSize_, kind);
        strings_[static_cast<std::size_t>(s)].setAnalyticTuningCompensation(0.0f);
        exciters_[static_cast<std::size_t>(s)].prepare(sampleRate_, maxBlockSize_);
        midiNote_[static_cast<std::size_t>(s)] = static_cast<std::uint8_t>(kMinMidiNote);
        portImpedance_[static_cast<std::size_t>(s)] = strings_[static_cast<std::size_t>(s)].portImpedance();
    }

    tapStorage_.assign(static_cast<std::size_t>(kMaxStrings) * static_cast<std::size_t>(maxBlockSize_), SampleT(0));
    bridgeBuffer_.assign(static_cast<std::size_t>(maxBlockSize_), SampleT(0));

    positionSmoothingCoeff_ = 1.0 - std::exp(-1.0 / (kPositionSmoothingSeconds * sampleRate_));
    releaseCoeff_ = static_cast<float>(std::exp(-kMinus60dBTimeConstants / (kReleaseSeconds * sampleRate_)));

    if (port_ == nullptr)
        port_ = &internalPort_;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);

    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);

    reset();
}

template <typename SampleT> void StringNetwork<SampleT>::reset() noexcept {
    for (int s = 0; s < kMaxStrings; ++s) {
        if (!strings_.empty())
            strings_[static_cast<std::size_t>(s)].reset();
        if (!exciters_.empty())
            exciters_[static_cast<std::size_t>(s)].reset();
        sounding_[static_cast<std::size_t>(s)] = false;
        releasing_[static_cast<std::size_t>(s)] = false;
        releaseGain_[static_cast<std::size_t>(s)] = 1.0f;
        portIncident_[static_cast<std::size_t>(s)] = SampleT(0);
        portOutgoing_[static_cast<std::size_t>(s)] = SampleT(0);
    }

    std::fill(tapStorage_.begin(), tapStorage_.end(), SampleT(0));
    std::fill(bridgeBuffer_.begin(), bridgeBuffer_.end(), SampleT(0));
    tapView_ = StringTapBuffers<SampleT>{};

    // Snap the position smoother onto its target, matching WaveguideString::reset(): a reset
    // instance must be indistinguishable from a freshly prepared one.
    pickupSmoothed_ = pickupTarget_;

    if (port_ != nullptr)
        port_->reset();
}

template <typename SampleT> void StringNetwork<SampleT>::setNumStrings(int count) noexcept {
    numStrings_ = std::clamp(count, 1, kMaxStrings);
}

template <typename SampleT> void StringNetwork<SampleT>::setParams(const StringNetworkParams& p) noexcept {
    params_ = p;
    pickupTarget_ = clampd(static_cast<double>(params_.pickupPosition01), 0.0, 1.0);
    for (int s = 0; s < kMaxStrings; ++s)
        applyStringParams(s);
}

template <typename SampleT> void StringNetwork<SampleT>::setBridgePort(IBridgePort<SampleT>& port) noexcept {
    port_ = &port;
    port_->prepare(sampleRate_, maxBlockSize_, kMaxStrings, portImpedance_.data());
    port_->setLossBypassed(lossless_);
    port_->reset();
}

template <typename SampleT> void StringNetwork<SampleT>::applyStringParams(int stringIndex) noexcept {
    if (strings_.empty())
        return;

    const auto index = static_cast<std::size_t>(stringIndex);
    WaveguideStringParams p;
    p.f0Hz = static_cast<float>(midiNoteToHz(static_cast<int>(midiNote_[index])));
    // Bend and the per-string tuning offset compose additively in semitones and reach the string
    // through its own per-sample f0 smoother, which is what keeps both click-free while ringing.
    const float bend = clampf(params_.pitchBendSemitones, -kPitchBendRangeSemitones, kPitchBendRangeSemitones);
    p.bendSemitones =
        bend + static_cast<float>(static_cast<double>(params_.perString[index].tuningOffsetCents) / kCentsPerSemitone);
    p.material = params_.material;
    strings_[index].setParams(p);
}

// ------------------------------------------------------------------------------------------
// event consumption
// ------------------------------------------------------------------------------------------

template <typename SampleT>
float StringNetwork<SampleT>::resolveNoteParam(float eventValue, float exciterDefault) const noexcept {
    // docs/plan.md section 2.3: the exciter defaults are "used when the note event carries no
    // explicit position". An in-range event value is per-note data and wins; anything else --
    // kUnspecifiedNoteParam, or a NaN from a malformed caller -- falls back to the parameter.
    if (eventValue >= 0.0f && eventValue <= 1.0f)
        return eventValue;
    return clampf(exciterDefault, 0.0f, 1.0f);
}

template <typename SampleT> void StringNetwork<SampleT>::handleEvent(const NoteEvent& event) noexcept {
    const int stringIndex = static_cast<int>(event.stringIndex);
    if (stringIndex < 0 || stringIndex >= numStrings_)
        return; // addressed to a string this network is not running
    const auto index = static_cast<std::size_t>(stringIndex);
    if (!params_.perString[index].enabled)
        return;

    if (event.type == NoteEventType::NoteOff) {
        if (!sounding_[index])
            return; // already released, or never sounded: nothing to damp
        sounding_[index] = false;
        releasing_[index] = true; // the release ramps on from wherever the gain currently is
        return;
    }

    const int note = std::clamp(static_cast<int>(event.midiNote), kMinMidiNote, kMaxMidiNote);

    // P1 retrigger semantics (see the StringNetwork.h scope note): same pitch on a ringing string
    // plucks over the existing state; anything else re-initializes the string at the new pitch.
    // A string mid-release counts as "anything else" -- its tail is already attenuated, so
    // clearing it is inaudible, whereas restoring the release gain to 1 over a still-ringing tail
    // would step it back up.
    const bool pluckOverRinging =
        sounding_[index] && !releasing_[index] && midiNote_[index] == static_cast<std::uint8_t>(note);
    if (!pluckOverRinging) {
        midiNote_[index] = static_cast<std::uint8_t>(note);
        applyStringParams(stringIndex); // retarget f0 first...
        strings_[index].reset();        // ...so reset() snaps the smoothers onto the NEW pitch
    }

    sounding_[index] = true;
    releasing_[index] = false;
    releaseGain_[index] = 1.0f;

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

    tapView_.base_ = tapStorage_.data();
    tapView_.stride_ = maxBlockSize_;
    tapView_.numStrings_ = numStrings_;
    tapView_.numSamples_ = count;
    for (int s = 0; s < kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        // "Enabled and ringing at some point during this block": seeded from the state the block
        // starts in, then latched true by any NoteOn the loop consumes.
        tapView_.active_[index] =
            params_.perString[index].enabled && (sounding_[index] || releasing_[index]) && s < numStrings_;
    }

    if (count == 0)
        return; // events stay queued for the next block rather than firing at no sample at all

    for (int s = 0; s < numStrings_; ++s)
        exciters_[static_cast<std::size_t>(s)].setParams(params_.exciter);

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

        // Per-sample smoothed fractional pickup tap (docs/plan.md Task P1.5 step 3): the position
        // moves continuously inside the loop, not once per block, which is what makes a moving
        // pickup click-free (P2.3 sweeps it).
        pickupSmoothed_ += positionSmoothingCoeff_ * (pickupTarget_ - pickupSmoothed_);
        const auto pickup = static_cast<float>(pickupSmoothed_);

        for (int s = 0; s < numStrings_; ++s) {
            const auto index = static_cast<std::size_t>(s);
            if (!params_.perString[index].enabled) {
                tapStorage_[index * static_cast<std::size_t>(maxBlockSize_) + static_cast<std::size_t>(n)] = SampleT(0);
                portIncident_[index] = SampleT(0); // a disabled string presents no incident wave
                continue;
            }

            const SampleT excitation = exciters_[index].renderSample();
            if (excitation != SampleT(0))
                strings_[index].injectAt(exciters_[index].latchedPosition01(), excitation);

            // Multiplying by an exactly-1.0f gain is exact in IEEE-754, so a string that is not
            // releasing is bit-identical to one with no release envelope at all.
            const SampleT tap = strings_[index].readTapAt(pickup) * static_cast<SampleT>(releaseGain_[index]);
            tapStorage_[index * static_cast<std::size_t>(maxBlockSize_) + static_cast<std::size_t>(n)] = tap;
            portIncident_[index] = strings_[index].railOutgoingAtBridge();

            if (releasing_[index]) {
                releaseGain_[index] *= releaseCoeff_;
                if (releaseGain_[index] <= kReleaseFloor) {
                    // Inaudible: clear the string rather than leave it ringing under a vanishing
                    // gain, so energyEstimate() tells the truth and the decayed tail costs
                    // nothing. The exciter is deliberately NOT reset -- reseeding its PRNG would
                    // make a render depend on note history.
                    releaseGain_[index] = 0.0f;
                    releasing_[index] = false;
                    strings_[index].reset();
                }
            }
        }

        for (int s = numStrings_; s < kMaxStrings; ++s)
            portIncident_[static_cast<std::size_t>(s)] = SampleT(0);

        // The port sees every string's outgoing bridge wave and publishes the mono bridge signal.
        // Its reflected waves are not routed back into the strings in P1 -- see setBridgePort().
        port_->scatter(portIncident_.data(), portOutgoing_.data(), numStrings_);
        bridgeBuffer_[static_cast<std::size_t>(n)] = port_->bridgeOutput();

        for (int s = 0; s < numStrings_; ++s)
            if (params_.perString[static_cast<std::size_t>(s)].enabled)
                strings_[static_cast<std::size_t>(s)].tick();
    }
}

// ------------------------------------------------------------------------------------------
// seams and diagnostics
// ------------------------------------------------------------------------------------------

template <typename SampleT>
void StringNetwork<SampleT>::injectFeedback(const SampleT* buffer, int numSamples, float airDelayMs,
                                            float gain) noexcept {
    // P4 seam: the request is recorded so a caller can see it arrived, and nothing else happens.
    // Implemented for real in P4, where the speaker-to-string air path becomes the block delay.
    (void)buffer;
    (void)numSamples;
    feedbackAirDelayMs_ = airDelayMs;
    feedbackGain_ = gain;
}

template <typename SampleT> void StringNetwork<SampleT>::setLosslessTestMode(bool lossless) noexcept {
    lossless_ = lossless;
    for (auto& string : strings_)
        string.setLossBypassed(lossless);
    if (port_ != nullptr)
        port_->setLossBypassed(lossless);
}

template <typename SampleT> Sample64 StringNetwork<SampleT>::energyEstimate() const noexcept {
    Sample64 total = 0.0;
    for (int s = 0; s < numStrings_; ++s) {
        const auto index = static_cast<std::size_t>(s);
        if (!params_.perString[index].enabled)
            continue;
        total += strings_[index].energyEstimate();
    }
    // P1's termination is rigid and memoryless, so it stores nothing; the bridge admittance
    // biquad's storage term joins this sum with BridgeJunction (P2.4).
    return total;
}

template struct StringTapBuffers<float>;  // realtime path
template struct StringTapBuffers<double>; // tier-2 [energy] tests
template class StringNetwork<float>;      // realtime path
template class StringNetwork<double>;     // tier-2 [energy] tests

} // namespace cnpg::dsp

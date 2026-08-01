#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include <array>
#include <atomic>
#include <type_traits>

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/OutputGain.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"

// Parameters -- the APVTS parameter layout (docs/plan.md Task P1.1: "Create the APVTS as the
// single parameter source") and the once-per-block snapshot that turns its atomics into the
// plain, trivially-copyable cnpg::dsp param structs the audio thread hands to dsp/ modules.
// This is the plugin-side "thin adapter" (docs/plan.md Architecture): the parameter ID string
// literals below are the naming registry for this project's automatable parameters, and are
// spelled out here only.

namespace cnpg::params {

// Parameter ID string literals, grouped to match the P1 surface in docs/plan.md Task P1.1
// step 1. AudioUnit version-hint convention: every parameter uses hint 1 (none have shipped
// yet, so there is no compatibility history to preserve).
namespace ID {

// Exciter (cnpg::dsp::PluckExciterParams, nested under StringNetworkParams::exciter)
inline constexpr const char* exciterDefaultPosition = "exciterDefaultPosition";
inline constexpr const char* exciterDefaultHardness = "exciterDefaultHardness";
inline constexpr const char* exciterNoiseAmount = "exciterNoiseAmount";

// String material (cnpg::dsp::StringMaterialParams, nested under
// StringNetworkParams::stringMaterial). Renamed from material* at Task P2.1 per ADR 0004: these
// three are the STRING's loop loss and dispersion, and Material/Wood is reserved for the body
// module a later phase adds. Done in the task that already bumps the state version, because a
// parameter ID persists into saved state and renaming it later would be a migration.

inline constexpr const char* stringMaterialLossGainLow = "stringMaterialLossGainLow";
inline constexpr const char* stringMaterialLossGainHigh = "stringMaterialLossGainHigh";
inline constexpr const char* stringMaterialDispersionAmount = "stringMaterialDispersionAmount";

// Pickup (cnpg::dsp::PickupTapParams, plus pickupPosition01 on StringNetworkParams itself --
// see docs/plan.md section 2.7, pickupPosition01 is a StringNetwork-owned tap position, not a
// PickupTapParams field)
inline constexpr const char* pickupResonanceHz = "pickupResonanceHz";
inline constexpr const char* pickupQ = "pickupQ";
inline constexpr const char* pickupOutputGainDb = "pickupOutputGainDb";
inline constexpr const char* pickupPosition01 = "pickupPosition01";

// Triode (cnpg::dsp::TriodeStageParams)
inline constexpr const char* triodeDrive = "triodeDrive";
inline constexpr const char* triodeOutputTrimDb = "triodeOutputTrimDb";
inline constexpr const char* triodeBypass = "triodeBypass";

// Monitoring chain (cnpg::dsp::CabFilterParams, SoftClipLimiterParams, OutputGainParams)
inline constexpr const char* cabBypass = "cabBypass";
inline constexpr const char* limiterCeilingDb = "limiterCeilingDb";
inline constexpr const char* outputGainDb = "outputGainDb";

// Damper (cnpg::dsp::DamperJunctionParams, nested under StringNetworkParams::damper, plus
// damperPosition01 on StringNetworkParams itself -- exactly the split pickupPosition01 has, and for
// the same reason: the POSITION is a StringNetwork-owned junction position, continuously
// modulatable while a note rings from Task P2.3, while depth and felt time are the junction module's
// own behaviour. Task P2.2 wired it; Task P2.3 made automating it click-free (per-sample smoothing
// in StringNetwork, dual-anchor crossfade on WaveguideString's seam), which is what this parameter
// being automatable had been promising since P2.2 and not delivering.
// The state version stays 2 (it stays 2 through all of P2, per Task P2.1).
inline constexpr const char* damperPosition01 = "damperPosition01";
inline constexpr const char* damperMaxLoss = "damperMaxLoss";
inline constexpr const char* damperFeltTimeMs = "damperFeltTimeMs";

// Bridge (cnpg::dsp::BridgeAdmittanceParams, nested under StringNetworkParams::bridge). Task P2.4.
// These three are the load every string terminates on, so they are the controls of how much of one
// string reaches the others -- not an effect on top of the strings but a property of the instrument
// they are all attached to. `bridgeCoupling` is the continuum ADR 0004 asks the later "chamber" to
// ride on; its shipping default is nonzero and measured (docs/decisions/0006). Smoothed per sample
// inside BridgeJunction, which is what makes them automatable without the exposure Task P2.2
// created for damperPosition01 and Task P2.3 had to close.
// The state version stays 2 (it stays 2 through all of P2, per Task P2.1).
inline constexpr const char* bridgeCoupling = "bridgeCoupling";
inline constexpr const char* bridgeResonanceHz = "bridgeResonanceHz";
inline constexpr const char* bridgeDamping = "bridgeDamping";

// Global (cnpg::dsp::RetriggerMode, nested under StringNetworkParams::retriggerMode)
inline constexpr const char* retriggerMode = "retriggerMode";

// Active string count, 1..kMaxStrings (Task P2.1). Not a StringNetworkParams field: the count is
// set through StringNetwork::setNumStrings(), which is deliberately its own entry point because a
// reduction is not a plain retarget -- it routes the removed strings through their enable ramp and
// only drops the loop's trip count on a later block.
inline constexpr const char* numStrings = "numStrings";

// Per-string block (cnpg::dsp::StringNetworkParams::PerString), one entry per slot for all
// kMaxStrings slots regardless of the active count -- automation lanes and saved state have to
// exist for a string before it is switched on, or turning the count up would silently reset the
// slots it reveals. Spelled as indexed tables rather than 16 named constants because the surface is
// a fixed-size family that createParameterLayout() builds in a loop; the ID STRINGS themselves are
// still literal, so this stays a complete grep-able registry of every shipped parameter ID.
inline constexpr const char* stringTuningOffsetCents[cnpg::dsp::kMaxStrings] = {
    "stringTuningOffsetCents0", "stringTuningOffsetCents1", "stringTuningOffsetCents2", "stringTuningOffsetCents3",
    "stringTuningOffsetCents4", "stringTuningOffsetCents5", "stringTuningOffsetCents6", "stringTuningOffsetCents7"};
inline constexpr const char* stringEnabled[cnpg::dsp::kMaxStrings] = {
    "stringEnabled0", "stringEnabled1", "stringEnabled2", "stringEnabled3",
    "stringEnabled4", "stringEnabled5", "stringEnabled6", "stringEnabled7"};

} // namespace ID

// Task P2.1's per-string tuning offset range, in cents either side of the note. Half a semitone is
// a deliberate ceiling rather than a round number: the offset composes ADDITIVELY with the pitch
// wheel inside the same f0 smoother, and WaveguideString's rails are sized for kMinMidiNote detuned
// by exactly kPitchBendRangeSemitones. Anything past that is clamped by the string rather than
// realised, so a wider knob would buy nothing at the bottom of the range except a control that
// silently stops responding. +/-50 cents covers what the control is for -- unison spread, a
// deliberately sour course, per-string compensation -- with the clamp only reachable by combining a
// full-scale bend with a full-scale offset on the lowest note the instrument has.
inline constexpr float kStringTuningOffsetRangeCents = 50.0f;

// Shipped default active string count (cnpg/dsp/Common.h: "active count configurable 1..8,
// default 6"). NOTE for the reader wondering why six strings sound like one today: NoteAllocator's
// multi-string assignment modes are Task P2.6, so every host note still lands on string 0 and the
// other five idle. They cost almost nothing while idle (StringNetwork skips a string with no state
// rather than ticking zeros through it), and shipping the designed count now means the parameter's
// default does not move again after saved state starts carrying it.
inline constexpr int kDefaultNumStrings = 6;

// Builds the full P1 APVTS parameter layout (docs/plan.md Task P1.1 step 1). Message-thread
// only; called once from PluginProcessor's member-initializer list.
juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

// One cached std::atomic<float>* per APVTS parameter, captured once (message thread, after
// APVTS construction) via apvts.getRawParameterValue(). Reading through these pointers on the
// audio thread is a plain atomic load -- no string lookup, no allocation, no locking -- which
// is what makes snapshotParameters() below "a trivial atomic-read adapter" (docs/plan.md Task
// P1.1 acceptance criteria).
struct RawParameterPointers {
    std::atomic<float>* exciterDefaultPosition = nullptr;
    std::atomic<float>* exciterDefaultHardness = nullptr;
    std::atomic<float>* exciterNoiseAmount = nullptr;

    std::atomic<float>* stringMaterialLossGainLow = nullptr;
    std::atomic<float>* stringMaterialLossGainHigh = nullptr;
    std::atomic<float>* stringMaterialDispersionAmount = nullptr;

    std::atomic<float>* pickupResonanceHz = nullptr;
    std::atomic<float>* pickupQ = nullptr;
    std::atomic<float>* pickupOutputGainDb = nullptr;
    std::atomic<float>* pickupPosition01 = nullptr;

    std::atomic<float>* triodeDrive = nullptr;
    std::atomic<float>* triodeOutputTrimDb = nullptr;
    std::atomic<float>* triodeBypass = nullptr;

    std::atomic<float>* cabBypass = nullptr;
    std::atomic<float>* limiterCeilingDb = nullptr;
    std::atomic<float>* outputGainDb = nullptr;

    std::atomic<float>* damperPosition01 = nullptr;
    std::atomic<float>* damperMaxLoss = nullptr;
    std::atomic<float>* damperFeltTimeMs = nullptr;

    std::atomic<float>* bridgeCoupling = nullptr;
    std::atomic<float>* bridgeResonanceHz = nullptr;
    std::atomic<float>* bridgeDamping = nullptr;

    std::atomic<float>* retriggerMode = nullptr;

    std::atomic<float>* numStrings = nullptr;
    std::array<std::atomic<float>*, cnpg::dsp::kMaxStrings> stringTuningOffsetCents{};
    std::array<std::atomic<float>*, cnpg::dsp::kMaxStrings> stringEnabled{};
};

// Message-thread only; must run after the APVTS (and therefore every parameter in
// createParameterLayout()) exists. Every pointer returned by
// AudioProcessorValueTreeState::getRawParameterValue() remains valid for the APVTS's lifetime
// (docs/plan.md: APVTS "must have the same lifetime as the processor"), so caching them once is
// sound for as long as PluginProcessor owns both.
RawParameterPointers collectRawParameterPointers(const juce::AudioProcessorValueTreeState& apvts);

// Aggregate of every dsp/ param struct the P1 surface currently drives, one field per module
// (docs/plan.md Task P1.1 step 2). Trivially copyable: every member type is itself
// static_assert'd trivially copyable in its own dsp/ header, so the audio thread may copy a
// whole Snapshot by value with no allocation or locking.
struct Snapshot {
    cnpg::dsp::StringNetworkParams stringNetwork; // includes the nested exciter + material params

    // The active string count travels beside StringNetworkParams rather than inside it, mirroring
    // the dsp/ split: setParams() retargets, setNumStrings() changes the loop's shape. The audio
    // thread applies it with the same once-per-block cascade as everything else, and
    // setNumStrings() is idempotent, so re-applying an unchanged count every block costs a compare.
    int numStrings = kDefaultNumStrings;

    cnpg::dsp::PickupTapParams pickup;
    cnpg::dsp::TriodeStageParams triode;
    cnpg::dsp::CabFilterParams cab;
    cnpg::dsp::SoftClipLimiterParams limiter;
    cnpg::dsp::OutputGainParams outputGain;
};

static_assert(std::is_trivially_copyable_v<Snapshot>,
              "Snapshot must stay trivially copyable for the realtime APVTS snapshot path.");

// Realtime-safe: reads every APVTS atomic exactly once through the cached pointers, writing the
// results into plain structs. No allocation, no locking. Called once per block from
// PluginProcessor::processBlock (docs/plan.md Task P1.1 step 2).
Snapshot snapshotParameters(const RawParameterPointers& params) noexcept;

} // namespace cnpg::params

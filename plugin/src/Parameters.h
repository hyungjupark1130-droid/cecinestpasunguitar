#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

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

// Material (cnpg::dsp::StringMaterialParams, nested under StringNetworkParams::material)
inline constexpr const char* materialLossGainLow = "materialLossGainLow";
inline constexpr const char* materialLossGainHigh = "materialLossGainHigh";
inline constexpr const char* materialDispersionAmount = "materialDispersionAmount";

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

// Global (cnpg::dsp::RetriggerMode, nested under StringNetworkParams::retriggerMode)
inline constexpr const char* retriggerMode = "retriggerMode";

} // namespace ID

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

    std::atomic<float>* materialLossGainLow = nullptr;
    std::atomic<float>* materialLossGainHigh = nullptr;
    std::atomic<float>* materialDispersionAmount = nullptr;

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

    std::atomic<float>* retriggerMode = nullptr;
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

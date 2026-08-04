// Task P1.1: compiles every dsp/ param struct landed so far for a module whose class hasn't
// arrived yet (PluckExciter.h, StringNetwork.h, TriodeStage.h, CabFilter.h, SoftClipLimiter.h --
// OutputGain.h and PickupTap.h already have a full module and their own *Tests.cpp).
// Each header's own static_assert(std::is_trivially_copyable_v<...>) already gates the
// no-alloc-assurance acceptance criterion at compile time; the checks below additionally prove
// aggregate initialization, default values, and plain-copy semantics behave as documented,
// under both the windows-msvc-release and linux-dsp-only (GCC + Clang) presets.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/DamperJunction.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>

using namespace cnpg::dsp;

TEST_CASE("PluckExciterParams: default-constructs to its documented defaults and copies by value", "[contract]") {
    PluckExciterParams params;
    REQUIRE(params.defaultPosition == 0.5f);
    REQUIRE(params.defaultHardness == 0.5f);
    REQUIRE(params.noiseAmount == 0.0f);

    params.noiseAmount = 0.25f;
    const PluckExciterParams copy = params; // plain-copy, not a reference: proves value semantics
    REQUIRE(copy.noiseAmount == 0.25f);
}

TEST_CASE("StringNetworkParams: nests StringMaterialParams and PluckExciterParams with documented defaults",
          "[contract]") {
    StringNetworkParams params;
    REQUIRE(params.retriggerMode == RetriggerMode::Physical);
    REQUIRE(params.pickupPosition01 == 0.5f);
    REQUIRE(params.stringMaterial.lossGainLow == 0.5f);
    REQUIRE(params.stringMaterial.lossGainHigh == 0.5f);
    REQUIRE(params.stringMaterial.dispersionAmount == 0.0f);
    REQUIRE(params.exciter.defaultPosition == 0.5f);

    // The per-string block (Task P2.1), one entry per kMaxStrings slot. Every slot defaults to "in
    // tune, on, and carrying no envelope scaling", so a StringNetworkParams built from nothing is a
    // playable instrument rather than a silent one.
    REQUIRE(params.perString.size() == static_cast<std::size_t>(cnpg::dsp::kMaxStrings));
    for (const auto& perString : params.perString) {
        REQUIRE(perString.tuningOffsetCents == 0.0f);
        REQUIRE(perString.envelopeScale == 1.0f); // reserved for the Envelope module (ADR 0004 D2)
        REQUIRE(perString.enabled);
    }

    params.retriggerMode = RetriggerMode::Synth;
    const StringNetworkParams copy = params;
    REQUIRE(copy.retriggerMode == RetriggerMode::Synth);
}

TEST_CASE("DamperJunctionParams: default-constructs to its documented defaults", "[contract]") {
    constexpr DamperJunctionParams params;
    // 1/25 -- the derivation lives on StringNetworkParams::damperPosition01, which this field
    // mirrors, and it is gated by tests/dsp/DamperReleaseSpectrumTests.cpp rather than by this
    // equality. Revised from 0.15 after the P2.9 exit on a listening finding.
    STATIC_REQUIRE(params.position01 == 0.04f);
    STATIC_REQUIRE(params.maxLoss == 1.0f);
    // The centre of the validated 20..100 ms window, and deliberately the same 40 ms the P1
    // placeholder release envelope used, so Task P2.2 replacing that envelope with a real damper
    // was not also a change of speed.
    STATIC_REQUIRE(params.feltTimeConstantMs == 40.0f);
    STATIC_REQUIRE(params.feltTimeConstantMs >= kFeltTimeConstantMinMs);
    STATIC_REQUIRE(params.feltTimeConstantMs <= kFeltTimeConstantMaxMs);

    // StringNetworkParams nests it, and carries its own damperPosition01 that is MIRRORED into
    // DamperJunctionParams::position01 on the way to each junction (docs/plan.md section 2.7). The
    // two defaults must agree, or a caller that never touches either would find the junction
    // somewhere other than where the network's own surface says it is.
    constexpr StringNetworkParams network;
    STATIC_REQUIRE(network.damper.maxLoss == params.maxLoss);
    STATIC_REQUIRE(network.damper.feltTimeConstantMs == params.feltTimeConstantMs);
    STATIC_REQUIRE(network.damperPosition01 == params.position01);
}

TEST_CASE("PickupTapParams: default-constructs to its documented defaults", "[contract]") {
    constexpr PickupTapParams params;
    STATIC_REQUIRE(params.resonanceHz == 2500.0f);
    STATIC_REQUIRE(params.q == 2.0f);
    // Task P1.9 gain staging: the trim's default is the measured -18 dBFS per-string calibration
    // constant, not 0 dB. plugin/src/Parameters.cpp reads this same value for the APVTS default,
    // and tests/dsp/MonitoringChainTests.cpp is what actually gates the -18 dBFS +/- 1 dB level.
    STATIC_REQUIRE(params.outputGainDb == kNominalPickupTrimDb);
}

TEST_CASE("TriodeStageParams: default-constructs to its documented defaults", "[contract]") {
    constexpr TriodeStageParams params;
    STATIC_REQUIRE(params.drive == 0.5f);
    STATIC_REQUIRE(params.outputTrimDb == 0.0f);
    STATIC_REQUIRE(params.bypass == false);
}

TEST_CASE("CabFilterParams and SoftClipLimiterParams: default-construct to their documented defaults", "[contract]") {
    constexpr CabFilterParams cab;
    STATIC_REQUIRE(cab.bypass == false);

    constexpr SoftClipLimiterParams limiter;
    STATIC_REQUIRE(limiter.ceilingDb == -0.3f);
}

// Task P1.1: compiles every dsp/ param struct landed so far for a module whose class hasn't
// arrived yet (PluckExciter.h, StringNetwork.h, TriodeStage.h, CabFilter.h, SoftClipLimiter.h --
// OutputGain.h and PickupTap.h already have a full module and their own *Tests.cpp).
// Each header's own static_assert(std::is_trivially_copyable_v<...>) already gates the
// no-alloc-assurance acceptance criterion at compile time; the checks below additionally prove
// aggregate initialization, default values, and plain-copy semantics behave as documented,
// under both the windows-msvc-release and linux-dsp-only (GCC + Clang) presets.

#include "cnpg/dsp/CabFilter.h"
#include "cnpg/dsp/PickupTap.h"
#include "cnpg/dsp/PluckExciter.h"
#include "cnpg/dsp/SoftClipLimiter.h"
#include "cnpg/dsp/StringNetwork.h"
#include "cnpg/dsp/TriodeStage.h"

#include <catch2/catch_test_macros.hpp>

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
    REQUIRE(params.material.lossGainLow == 0.5f);
    REQUIRE(params.material.lossGainHigh == 0.5f);
    REQUIRE(params.material.dispersionAmount == 0.0f);
    REQUIRE(params.exciter.defaultPosition == 0.5f);

    params.retriggerMode = RetriggerMode::Synth;
    const StringNetworkParams copy = params;
    REQUIRE(copy.retriggerMode == RetriggerMode::Synth);
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

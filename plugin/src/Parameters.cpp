#include "Parameters.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cnpg::params {

namespace {

// AudioUnit parameter-ordering version hint (docs/plan.md Task P1.1: "parameter IDs"); every
// P1 parameter ships in the same release, so every one uses the same hint.
constexpr int kParameterVersionHint = 1;

juce::ParameterID makeParameterID(const char* id) { return juce::ParameterID(id, kParameterVersionHint); }

} // namespace

juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout() {
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    const juce::NormalisableRange<float> unitRange(0.0f, 1.0f);
    const juce::NormalisableRange<float> trimDbRange(-24.0f, 24.0f);

    // Task P1.9 gain staging. The pickup trim is the stage that lands a single string on the
    // -18 dBFS per-string nominal (docs/plan.md Task P1.9 step 3), and the calibrated value is
    // measured, not chosen: cnpg::dsp::kNominalPickupTrimDb (see PickupTap.h for the measurement
    // and the exact reference scenario). Both the default AND the range's centre are read from
    // that one constant, so the plugin's default state IS the calibrated state, the user keeps a
    // symmetric +/-24 dB of trim around it, and the dsp-side gate test
    // (tests/dsp/MonitoringChainTests.cpp, which drives PickupTapParams{} directly) cannot drift
    // apart from what the plugin actually ships.
    const float pickupTrimDefaultDb = cnpg::dsp::PickupTapParams{}.outputGainDb;
    const juce::NormalisableRange<float> pickupTrimDbRange(pickupTrimDefaultDb - 24.0f, pickupTrimDefaultDb + 24.0f);

    // Exciter. The position default is READ from cnpg::dsp::PluckExciterParams rather than retyped
    // here, exactly as the damper's is: the derivation, the criterion it satisfies and the cost all
    // live at the field (PluckExciter.h), and a plugin that restated the number could ship a
    // geometry the dsp side never agreed to. It was 0.5 through P2.9 -- the midpoint, which nulls
    // every even partial at the pick as well as at the tap.
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterDefaultPosition),
                                                           "Exciter Position", unitRange,
                                                           cnpg::dsp::PluckExciterParams{}.defaultPosition));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterDefaultHardness),
                                                           "Exciter Hardness", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::exciterNoiseAmount), "Exciter Noise",
                                                           unitRange, 0.0f));

    // String material (ADR 0004 amendment 1: these are the STRING's frequency-dependent loop loss
    // and its dispersion, i.e. bending stiffness. "Material"/"Wood" is reserved for the BODY, which
    // is a separate module in a later phase; shipping both under "Material" on one GUI tab would be
    // a permanent naming collision, and parameter IDs persist into saved state.)
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::stringMaterialLossGainLow),
                                                           "String Loss Low", unitRange, 0.5f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::stringMaterialLossGainHigh),
                                                           "String Loss High", unitRange, 0.5f));
    // Dispersion IS bending stiffness: a stiff string's partials stretch sharp. "String Stiffness"
    // is the physical name for the knob; dispersionAmount stays the code-side spelling.
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::stringMaterialDispersionAmount),
                                                           "String Stiffness", unitRange, 0.0f));

    // Pickup
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupResonanceHz), "Pickup Resonance",
                                                           juce::NormalisableRange<float>(100.0f, 8000.0f, 1.0f, 0.4f),
                                                           2500.0f,
                                                           juce::AudioParameterFloatAttributes().withLabel("Hz")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupQ), "Pickup Q",
                                                           juce::NormalisableRange<float>(0.1f, 10.0f), 2.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupOutputGainDb),
                                                           "Pickup Output Gain", pickupTrimDbRange, pickupTrimDefaultDb,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));
    // Same rule as the exciter position above and the damper position below: the default is read
    // from the dsp-side struct, never retyped. See StringNetwork.h for why the tap sits 1/16 of the
    // string from the bridge instead of on the midpoint it occupied through P2.9, and why the
    // proposed move to 1/7 was refused on measurement.
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::pickupPosition01), "Pickup Position",
                                                           unitRange,
                                                           cnpg::dsp::StringNetworkParams{}.pickupPosition01));

    // Damper (Task P2.2). Position and depth are the two controls that make a palm mute and a
    // natural harmonic playable rather than emergent-only: p = 0.5 leaves a released note ringing
    // an octave up (the 2nd harmonic has a node there and the damper cannot touch it), and a
    // partial depth is a palm mute rather than a note-off. Felt time is exposed over exactly the
    // window DamperJunction validates, so the knob cannot ask for a setting the module refuses.
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::damperPosition01), "Damper Position",
                                                           unitRange,
                                                           cnpg::dsp::StringNetworkParams{}.damperPosition01));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::damperMaxLoss), "Damper Depth",
                                                           unitRange, cnpg::dsp::DamperJunctionParams{}.maxLoss));
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        makeParameterID(ID::damperFeltTimeMs), "Damper Felt Time",
        juce::NormalisableRange<float>(cnpg::dsp::kFeltTimeConstantMinMs, cnpg::dsp::kFeltTimeConstantMaxMs),
        cnpg::dsp::DamperJunctionParams{}.feltTimeConstantMs, juce::AudioParameterFloatAttributes().withLabel("ms")));

    // Bridge (Task P2.4). The three controls of the load every string terminates on, and therefore
    // of how much of ONE string reaches the others: coupling is the continuum
    // docs/decisions/0004-phase2-vision-decisions.md asks the later "chamber" to ride on rather
    // than a toggle, and its default is measured rather than chosen (docs/decisions/0006). Each is
    // exposed over exactly the window BridgeJunction validates, so the knob cannot ask for a
    // setting the module refuses -- and the resonance range is expressed against the shipping
    // design envelope rather than against the Nyquist ceiling, which moves with the sample rate.
    //
    // ---- THE NORMAL RANGE, AND WHERE THESE SLIDERS LEAVE IT (Task P2.7; ADR 0007 D7) ------------
    //
    // The +/-2 cent tuning guarantee binds over the PROVISIONAL NORMAL RANGE, which is narrower than
    // every one of the three ranges below:
    //
    //     Bridge Coupling    guaranteed 0.00 .. kBridgeNormalCouplingMax    (slider 0 .. 1)
    //     Bridge Resonance   guaranteed  kBridgeNormalResonanceMinHz .. kBridgeNormalResonanceMaxHz
    //                                                                       (slider 20 .. 2000 Hz)
    //     Bridge Damping     guaranteed  kBridgeNormalDampingMin .. kBridgeNormalDampingMax
    //                                                                       (slider 0.01 .. 4.0)
    //
    // Outside it is the **Extended (Effect) range**: every setting stays reachable and stays passive,
    // and the instrument simply stops promising to be in tune there -- a radically compliant or
    // radically sharp bridge SHOULD pull pitch (ADR 0007 D3). The ranges below are DELIBERATELY NOT
    // narrowed to the guarantee: narrowing them would delete the effect range rather than declare it,
    // and ADR 0004 ships the chamber as a continuum. What D5 requires is that the boundary be
    // declared rather than discovered, so it is named here, on the parameter surface, in the
    // constants dsp/include/cnpg/dsp/BridgeJunction.h derives and carries -- and a UI that draws a
    // guaranteed-range marker on these three sliders has exactly one place to read it from.
    //
    // *** WHAT THE GUARANTEE IS AND IS NOT. *** It is the +/-2 cent TUNING bound -- ADR 0007 D5's
    // criterion (1) -- and nothing else. D5 defines the Normal range as the region where all FIVE of
    // its criteria hold at once, and the box above is STILL not that region: criterion (4),
    // "near-unison strings ~25 cents apart do not involuntarily mode-lock", is measured to FAIL at
    // kBridgeNormalCouplingMax. Two strings 25 cents apart collapse to a 0.003-cent separation there.
    // See ADR 0007 D7.0. A UI marker drawn from these constants would therefore be marking the
    // in-tune sub-range, not a "safe" one.
    //
    // *** WHAT CHANGED ON 2026-08-05: THE DEFAULT LEFT THE FAILING REGION; THE CEILING DID NOT MOVE.
    // *** This slider's default WAS kBridgeNormalCouplingMax itself, so the shipped instrument sat on
    // the one point of the box where criterion (4) is known to fail. It is now 0.20, which is the
    // largest value at which criterion (4) is measured to HOLD on the shipping six-string topology
    // (ADR 0007 D7.1: separation 23.612 cents, against a single locked peak at 0.30). So the default
    // is inside the criterion-complete sub-region and the declared box still is not -- the gap
    // between 0.20 and the ceiling is real and is what a user crosses by dragging this slider up.
    //
    // The default was settled by AUTHOR DELEGATION on 2026-08-05, NOT by the listening pass ADR 0007
    // D4 reserves: docs/listening/P2-20260803.md is still marked not performed. D4's condition was
    // waived, not met. The RANGE remains provisional and criterion (5) -- "the bridge still behaves
    // as an instrument component rather than an overt resonant effect" -- has still never been judged.
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::bridgeCoupling), "Bridge Coupling",
                                                           unitRange,
                                                           cnpg::dsp::BridgeAdmittanceParams{}.couplingStrength));
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        makeParameterID(ID::bridgeResonanceHz), "Bridge Resonance",
        juce::NormalisableRange<float>(cnpg::dsp::kBridgeMinResonanceHz, 2000.0f, 0.0f, 0.35f),
        cnpg::dsp::BridgeAdmittanceParams{}.resonanceHz, juce::AudioParameterFloatAttributes().withLabel("Hz")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        makeParameterID(ID::bridgeDamping), "Bridge Damping",
        juce::NormalisableRange<float>(cnpg::dsp::kBridgeMinDamping, 4.0f, 0.0f, 0.5f),
        cnpg::dsp::BridgeAdmittanceParams{}.damping));

    // Triode
    layout.add(
        std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::triodeDrive), "Triode Drive", unitRange, 0.5f));
    // Task P1.9 gain staging, part two. TriodeStage's small-signal gain in the SAMPLE domain is
    // kGridVoltsPerFullScale at the default drive (+13.98 dB) -- see the block comment above
    // cnpg::dsp::kUnityGainOutputTrimDb in TriodeStage.h. This trim takes exactly that back out,
    // so a single string arriving at the -18 dBFS per-string nominal leaves the triode at
    // -18 dBFS too and the whole ~+16 dB summing budget still fits under the limiter ceiling.
    // Derived from the circuit constant, not measured and rounded, and centred on like the pickup
    // trim so the user keeps a symmetric +/-24 dB either side of the calibrated value.
    const juce::NormalisableRange<float> triodeTrimDbRange(cnpg::dsp::kUnityGainOutputTrimDb - 24.0f,
                                                           cnpg::dsp::kUnityGainOutputTrimDb + 24.0f);
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        makeParameterID(ID::triodeOutputTrimDb), "Triode Output Trim", triodeTrimDbRange,
        cnpg::dsp::kUnityGainOutputTrimDb, juce::AudioParameterFloatAttributes().withLabel("dB")));
    layout.add(std::make_unique<juce::AudioParameterBool>(makeParameterID(ID::triodeBypass), "Triode Bypass", false));

    // Monitoring chain
    layout.add(std::make_unique<juce::AudioParameterBool>(makeParameterID(ID::cabBypass), "Cab Bypass", false));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::limiterCeilingDb), "Limiter Ceiling",
                                                           juce::NormalisableRange<float>(-12.0f, 0.0f), -0.3f,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));
    layout.add(std::make_unique<juce::AudioParameterFloat>(makeParameterID(ID::outputGainDb), "Output Gain",
                                                           trimDbRange, 0.0f,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));

    // Global
    layout.add(std::make_unique<juce::AudioParameterChoice>(makeParameterID(ID::retriggerMode), "Retrigger Mode",
                                                            juce::StringArray{"Physical", "Synth"}, 0));

    // Allocation (Task P2.6). GuitarFingering is index 0 and the shipped default: it is the mode
    // that makes six strings behave like an instrument rather than like six independent zones, and
    // it is the one whose default table (EADGBE, below) is a tuning somebody actually plays.
    layout.add(std::make_unique<juce::AudioParameterChoice>(makeParameterID(ID::allocationMode), "Allocation Mode",
                                                            juce::StringArray{"Guitar Fingering", "Free Zones"}, 0));

    // Strings (Task P2.1). The count is an INT parameter, not a float: it is a structural choice
    // with kMaxStrings discrete states, and a host that shows it as a continuous 0..1 knob would
    // make "5.4 strings" a thing a user can automate toward.
    layout.add(std::make_unique<juce::AudioParameterInt>(makeParameterID(ID::numStrings), "Strings", 1,
                                                         cnpg::dsp::kMaxStrings, kDefaultNumStrings));

    // One tuning-offset and one enable per slot, for ALL kMaxStrings slots regardless of the active
    // count -- see the ID table's comment for why the inactive slots still get parameters.
    const juce::NormalisableRange<float> tuningOffsetRange(-kStringTuningOffsetRangeCents,
                                                           kStringTuningOffsetRangeCents);
    const cnpg::dsp::NoteAllocatorParams allocatorDefaults;
    for (int s = 0; s < cnpg::dsp::kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        const juce::String suffix(s + 1); // 1-based in the UI; 0-based in the ID, matching the code
        layout.add(std::make_unique<juce::AudioParameterFloat>(
            makeParameterID(ID::stringTuningOffsetCents[index]), "String " + suffix + " Tune", tuningOffsetRange, 0.0f,
            juce::AudioParameterFloatAttributes().withLabel("cents")));
        layout.add(std::make_unique<juce::AudioParameterBool>(makeParameterID(ID::stringEnabled[index]),
                                                              "String " + suffix + " On", true));

        // Allocation tables (Task P2.6). Every default is READ FROM cnpg::dsp::NoteAllocatorParams
        // rather than retyped here, so the plugin's default state IS the dsp-side default and the
        // headless [contract] cases cannot drift apart from what the plugin ships. The open note is
        // restricted to the design envelope (kMinMidiNote..kMaxMidiNote) because a string tuned
        // outside it could never sound; the zone bounds span the whole of MIDI, because a zone is a
        // routing decision and a user is entitled to draw one that catches notes the strings will
        // then reject on their own terms.
        layout.add(std::make_unique<juce::AudioParameterInt>(
            makeParameterID(ID::stringOpenNote[index]), "String " + suffix + " Open Note", cnpg::dsp::kMinMidiNote,
            cnpg::dsp::kMaxMidiNote, static_cast<int>(allocatorDefaults.openStringMidiNote[index])));
        layout.add(std::make_unique<juce::AudioParameterInt>(makeParameterID(ID::stringZoneLowNote[index]),
                                                             "String " + suffix + " Zone Low", 0, 127,
                                                             static_cast<int>(allocatorDefaults.zones[index].lowNote)));
        layout.add(std::make_unique<juce::AudioParameterInt>(
            makeParameterID(ID::stringZoneHighNote[index]), "String " + suffix + " Zone High", 0, 127,
            static_cast<int>(allocatorDefaults.zones[index].highNote)));
    }

    return layout;
}

RawParameterPointers collectRawParameterPointers(const juce::AudioProcessorValueTreeState& apvts) {
    RawParameterPointers params;

    params.exciterDefaultPosition = apvts.getRawParameterValue(ID::exciterDefaultPosition);
    params.exciterDefaultHardness = apvts.getRawParameterValue(ID::exciterDefaultHardness);
    params.exciterNoiseAmount = apvts.getRawParameterValue(ID::exciterNoiseAmount);

    params.stringMaterialLossGainLow = apvts.getRawParameterValue(ID::stringMaterialLossGainLow);
    params.stringMaterialLossGainHigh = apvts.getRawParameterValue(ID::stringMaterialLossGainHigh);
    params.stringMaterialDispersionAmount = apvts.getRawParameterValue(ID::stringMaterialDispersionAmount);

    params.pickupResonanceHz = apvts.getRawParameterValue(ID::pickupResonanceHz);
    params.pickupQ = apvts.getRawParameterValue(ID::pickupQ);
    params.pickupOutputGainDb = apvts.getRawParameterValue(ID::pickupOutputGainDb);
    params.pickupPosition01 = apvts.getRawParameterValue(ID::pickupPosition01);

    params.damperPosition01 = apvts.getRawParameterValue(ID::damperPosition01);
    params.damperMaxLoss = apvts.getRawParameterValue(ID::damperMaxLoss);
    params.damperFeltTimeMs = apvts.getRawParameterValue(ID::damperFeltTimeMs);

    params.bridgeCoupling = apvts.getRawParameterValue(ID::bridgeCoupling);
    params.bridgeResonanceHz = apvts.getRawParameterValue(ID::bridgeResonanceHz);
    params.bridgeDamping = apvts.getRawParameterValue(ID::bridgeDamping);

    params.triodeDrive = apvts.getRawParameterValue(ID::triodeDrive);
    params.triodeOutputTrimDb = apvts.getRawParameterValue(ID::triodeOutputTrimDb);
    params.triodeBypass = apvts.getRawParameterValue(ID::triodeBypass);

    params.cabBypass = apvts.getRawParameterValue(ID::cabBypass);
    params.limiterCeilingDb = apvts.getRawParameterValue(ID::limiterCeilingDb);
    params.outputGainDb = apvts.getRawParameterValue(ID::outputGainDb);

    params.retriggerMode = apvts.getRawParameterValue(ID::retriggerMode);
    params.allocationMode = apvts.getRawParameterValue(ID::allocationMode);

    params.numStrings = apvts.getRawParameterValue(ID::numStrings);
    for (int s = 0; s < cnpg::dsp::kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        params.stringTuningOffsetCents[index] = apvts.getRawParameterValue(ID::stringTuningOffsetCents[index]);
        params.stringEnabled[index] = apvts.getRawParameterValue(ID::stringEnabled[index]);
        params.stringOpenNote[index] = apvts.getRawParameterValue(ID::stringOpenNote[index]);
        params.stringZoneLowNote[index] = apvts.getRawParameterValue(ID::stringZoneLowNote[index]);
        params.stringZoneHighNote[index] = apvts.getRawParameterValue(ID::stringZoneHighNote[index]);
    }

    return params;
}

Snapshot snapshotParameters(const RawParameterPointers& params) noexcept {
    Snapshot snapshot;

    snapshot.stringNetwork.exciter.defaultPosition = params.exciterDefaultPosition->load();
    snapshot.stringNetwork.exciter.defaultHardness = params.exciterDefaultHardness->load();
    snapshot.stringNetwork.exciter.noiseAmount = params.exciterNoiseAmount->load();

    snapshot.stringNetwork.stringMaterial.lossGainLow = params.stringMaterialLossGainLow->load();
    snapshot.stringNetwork.stringMaterial.lossGainHigh = params.stringMaterialLossGainHigh->load();
    snapshot.stringNetwork.stringMaterial.dispersionAmount = params.stringMaterialDispersionAmount->load();

    snapshot.stringNetwork.pickupPosition01 = params.pickupPosition01->load();

    // damperPosition01 is the single source of truth on this surface; StringNetwork mirrors it into
    // each DamperJunction's own position01 field (docs/plan.md section 2.7), so nothing writes
    // snapshot.stringNetwork.damper.position01 here.
    snapshot.stringNetwork.damperPosition01 = params.damperPosition01->load();
    snapshot.stringNetwork.damper.maxLoss = params.damperMaxLoss->load();
    snapshot.stringNetwork.damper.feltTimeConstantMs = params.damperFeltTimeMs->load();

    // The bridge admittance travels on StringNetworkParams and reaches whatever IBridgePort is
    // attached through StringNetwork::setParams -- the plugin never touches BridgeJunction
    // directly, which is what keeps the P2.5 fallback substitutable at the port seam.
    snapshot.stringNetwork.bridge.couplingStrength = params.bridgeCoupling->load();
    snapshot.stringNetwork.bridge.resonanceHz = params.bridgeResonanceHz->load();
    snapshot.stringNetwork.bridge.damping = params.bridgeDamping->load();
    // AudioParameterChoice reports its selected index as a float via getRawParameterValue();
    // index 0 -> Physical, 1 -> Synth (matches the choices list in createParameterLayout()).
    snapshot.stringNetwork.retriggerMode =
        params.retriggerMode->load() >= 0.5f ? cnpg::dsp::RetriggerMode::Synth : cnpg::dsp::RetriggerMode::Physical;

    snapshot.pickup.resonanceHz = params.pickupResonanceHz->load();
    snapshot.pickup.q = params.pickupQ->load();
    snapshot.pickup.outputGainDb = params.pickupOutputGainDb->load();

    snapshot.triode.drive = params.triodeDrive->load();
    snapshot.triode.outputTrimDb = params.triodeOutputTrimDb->load();
    // AudioParameterBool reports 0.0f/1.0f via getRawParameterValue().
    snapshot.triode.bypass = params.triodeBypass->load() >= 0.5f;

    snapshot.cab.bypass = params.cabBypass->load() >= 0.5f;

    snapshot.limiter.ceilingDb = params.limiterCeilingDb->load();

    snapshot.outputGain.gainDb = params.outputGainDb->load();

    // AudioParameterInt reports its DENORMALISED integer value through getRawParameterValue(), so
    // this is a plain round rather than a 0..1 remap. Clamped anyway: StringNetwork clamps too, but
    // a Snapshot that carries an out-of-range count would put the burden on every future consumer.
    snapshot.numStrings =
        std::clamp(static_cast<int>(std::lround(params.numStrings->load())), 1, cnpg::dsp::kMaxStrings);

    // Allocation (Task P2.6). The mode and the two tables; the count and the per-string mute are
    // MIRRORED from the same values StringNetwork gets, in the loop below, so the allocator and the
    // network cannot disagree about which strings exist.
    snapshot.noteAllocator.mode = params.allocationMode->load() >= 0.5f ? cnpg::dsp::AllocationMode::FreeZones
                                                                        : cnpg::dsp::AllocationMode::GuitarFingering;
    snapshot.noteAllocator.activeStringCount = snapshot.numStrings;

    for (int s = 0; s < cnpg::dsp::kMaxStrings; ++s) {
        const auto index = static_cast<std::size_t>(s);
        snapshot.stringNetwork.perString[index].tuningOffsetCents = params.stringTuningOffsetCents[index]->load();
        // AudioParameterBool reports 0.0f/1.0f via getRawParameterValue().
        snapshot.stringNetwork.perString[index].enabled = params.stringEnabled[index]->load() >= 0.5f;
        snapshot.noteAllocator.stringEnabled[index] = snapshot.stringNetwork.perString[index].enabled;

        // AudioParameterInt reports its DENORMALISED integer value, so these are plain rounds.
        // Clamped into 0..127 anyway: a Snapshot carrying an out-of-range MIDI note would push the
        // burden onto every future consumer, and NoteAllocatorParams stores them as std::uint8_t.
        const auto toNote = [](float raw) {
            return static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(raw)), 0, 127));
        };
        snapshot.noteAllocator.openStringMidiNote[index] = toNote(params.stringOpenNote[index]->load());
        snapshot.noteAllocator.zones[index].lowNote = toNote(params.stringZoneLowNote[index]->load());
        snapshot.noteAllocator.zones[index].highNote = toNote(params.stringZoneHighNote[index]->load());
    }

    return snapshot;
}

} // namespace cnpg::params

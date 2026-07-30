# cnpg_plugin -- JUCE 8 VST3 + Standalone wrapper around cnpg_dsp.
#
# Permanent identity codes (pinned Task P0.4; see docs/plan.md section 1.4 -- changing either
# of these breaks host-side plugin identity/caching for every future version, so they must
# never change once released):
#   PLUGIN_MANUFACTURER_CODE  Hjpk
#   PLUGIN_CODE               Cnpg
juce_add_plugin(cnpg_plugin
    PRODUCT_NAME              "cecinestpasunguitar"
    COMPANY_NAME              "Hyung Ju Park"
    BUNDLE_ID                 com.hyungjupark.cecinestpasunguitar
    PLUGIN_MANUFACTURER_CODE  Hjpk
    PLUGIN_CODE               Cnpg
    FORMATS                   VST3 Standalone
    IS_SYNTH                  TRUE
    NEEDS_MIDI_INPUT          TRUE
    NEEDS_MIDI_OUTPUT         FALSE
    VST3_CATEGORIES           Instrument Synth
    COPY_PLUGIN_AFTER_BUILD   FALSE)

target_sources(cnpg_plugin PRIVATE
    src/MidiConverter.cpp
    src/Parameters.cpp
    src/PluginProcessor.cpp
)

# JUCE_DISPLAY_SPLASH_SCREEN=0 is legitimate under the GPLv3 JUCE license path this project
# builds under (docs/plan.md section 1.3/1.4). JUCE_WEB_BROWSER/JUCE_USE_CURL are off because
# nothing in this plugin needs a webview or network access.
target_compile_definitions(cnpg_plugin PRIVATE
    JUCE_DISPLAY_SPLASH_SCREEN=0
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0)

# Links cnpg_dsp per the target graph (docs/plan.md section 1.4) even though P0.4's sine-tone
# scaffold doesn't call into it yet -- real dsp wiring (APVTS, PluckExciter, StringNetwork, ...)
# starts landing in P1.1. juce_audio_utils pulls in juce_audio_processors (AudioProcessor,
# GenericAudioProcessorEditor), juce_audio_devices (Standalone's device manager), and their
# transitive dependencies.
target_link_libraries(cnpg_plugin
    PRIVATE
        cnpg_dsp
        juce::juce_audio_utils
)

cnpg_set_warnings(cnpg_plugin)

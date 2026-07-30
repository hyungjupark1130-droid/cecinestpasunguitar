# cnpg_plugin -- JUCE 8 VST3 + Standalone wrapper around cnpg_dsp.
#
# juce_add_plugin() target definition (FORMATS VST3 Standalone, product/vendor identity,
# frozen bus layout) lands in Task P0.4. Intentionally empty for now: this file only needs
# to exist so the top-level CMakeLists.txt's add_subdirectory(plugin) succeeds. Only reached
# when CNPG_BUILD_PLUGIN=ON -- the linux-dsp-only preset never adds this subdirectory.

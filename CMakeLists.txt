cmake_minimum_required(VERSION 3.24)

# LANGUAGES includes C: JUCE's own CMakeLists.txt (FetchContent'd below) requires a C
# compiler to be enabled on the project even though cnpg_* targets are C++-only.
project(cecinestpasunguitar VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Every executable lands in build/bin/<Config>/ (multi-config generators append the
# per-config subdirectory automatically). The plugin's VST3/Standalone artefacts land under
# their own juce_add_plugin() artefact directory instead (wired in P0.4).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(CNPG_BUILD_PLUGIN "Build cnpg_plugin (JUCE VST3 + Standalone); OFF for the dsp-only portability job" ON)
option(CNPG_BUILD_TESTS "Build cnpg_tests and the headless dsp/ executables" ON)
option(CNPG_WARNINGS_AS_ERRORS "Treat cnpg target warnings as errors (/WX, -Werror); escape hatch for dependency-upgrade triage only, never OFF in CI" ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/PluginvalPin.cmake)
include(cmake/Dependencies.cmake)

add_subdirectory(dsp) # cnpg_dsp -- target defined starting Task P0.3

if(CNPG_BUILD_PLUGIN)
    add_subdirectory(plugin) # cnpg_plugin -- target defined starting Task P0.4
endif()

if(CNPG_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests) # cnpg_tests / cnpg_bench / cnpg_render / cnpg_calibrate -- defined starting Task P0.3
endif()

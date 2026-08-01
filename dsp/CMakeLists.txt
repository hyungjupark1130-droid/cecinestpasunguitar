# cnpg_dsp -- JUCE-free physics + block DSP static library.
#
# STATIC library, public include dir dsp/include, sources from dsp/src. Links nothing
# external (see docs/plan.md section 2: every dsp/ type is 100% JUCE-free and headless).
# cnpg_set_warnings() is defined by cmake/CompilerWarnings.cmake, included by the
# top-level CMakeLists.txt before add_subdirectory(dsp).

add_library(cnpg_dsp STATIC
    src/BridgeJunction.cpp
    src/CabFilter.cpp
    src/DamperJunction.cpp
    src/ModuleGraph.cpp
    src/NoteAllocator.cpp
    src/OutputGain.cpp
    src/Oversampler.cpp
    src/PickupTap.cpp
    src/PluckExciter.cpp
    src/SoftClipLimiter.cpp
    src/StringNetwork.cpp
    src/TriodeStage.cpp
    src/WaveguideString.cpp
)

target_include_directories(cnpg_dsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

cnpg_set_warnings(cnpg_dsp)

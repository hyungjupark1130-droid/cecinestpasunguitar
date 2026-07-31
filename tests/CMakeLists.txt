# cnpg_tests / cnpg_bench / cnpg_render / cnpg_calibrate -- headless verification executables.
#
# cnpg_tests (Catch2 v3 + CTest, catch_discover_tests) lands here starting Task P0.3
# alongside the first [contract] test. Links cnpg_dsp + Catch2::Catch2WithMain only --
# no JUCE, per the naming registry. cnpg_bench and cnpg_render land in P1, cnpg_calibrate
# in P2.

add_executable(cnpg_tests
    dsp/EventQueueTests.cpp
    dsp/MidiTranslationTests.cpp
    dsp/NoteAllocatorTests.cpp
    dsp/OutputGainTests.cpp
    dsp/ParamStructsTests.cpp
    dsp/PluckExciterTests.cpp
    dsp/ScopedFtzDazGuardTests.cpp
    dsp/WaveguideStringTests.cpp
    dsp/WaveguideStringRegressionTests.cpp
    dsp/WaveguideStringTuningTests.cpp
    support/AllocationGuard.cpp
    support/GoldenIo.cpp
    support/SpectralAnalysis.cpp
    support/StringIrScenarios.cpp
)

# Lets any test include shared test-support headers as "support/<Header>.h" (e.g.
# support/AllocationGuard.h), regardless of which tests/<subdir> the including file lives in.
target_include_directories(cnpg_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Commit the goldens were generated from, for the sidecar's provenance field (docs/plan.md
# section 4.3). Resolved at configure time; re-run CMake before regenerating goldens so the
# recorded SHA matches the tree the render came from.
find_package(Git QUIET)
set(CNPG_GIT_COMMIT "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE CNPG_GIT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(NOT CNPG_GIT_COMMIT)
        set(CNPG_GIT_COMMIT "unknown")
    endif()
endif()

target_compile_definitions(cnpg_tests PRIVATE
    CNPG_GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/data/golden"
    CNPG_GIT_COMMIT="${CNPG_GIT_COMMIT}"
)

target_link_libraries(cnpg_tests PRIVATE
    cnpg_dsp
    Catch2::Catch2WithMain
)

cnpg_set_warnings(cnpg_tests)

include(Catch)
# ADD_TAGS_AS_LABELS mirrors every Catch2 tag (e.g. [contract]) to a CTest label, so
# `ctest -L contract` selects by tag exactly as docs/plan.md section 1.5's conventions
# describe.
catch_discover_tests(cnpg_tests ADD_TAGS_AS_LABELS)

# Regenerate-goldens target (docs/plan.md section 1.6). The section-1.6 draft drives this from
# `cnpg_render --regenerate-goldens`; cnpg_render lands in Task P1.11, so until then the
# regeneration entry point is the hidden ("[.]"-tagged, therefore never discovered by CTest and
# never run in CI) Catch2 case "REGEN: rewrite string_ir goldens" inside cnpg_tests. That keeps
# a single renderer shared by the goldens and the tests they gate, which is the property that
# actually matters: a golden can never be written by different code than the one that later
# compares against it.
add_custom_target(cnpg_regen_goldens
    COMMAND $<TARGET_FILE:cnpg_tests> "REGEN: rewrite string_ir goldens"
    DEPENDS cnpg_tests
    USES_TERMINAL
    COMMENT "Rewrites float64 golden IRs for every scenario/algorithm variant at 44.1/48/96 kHz")

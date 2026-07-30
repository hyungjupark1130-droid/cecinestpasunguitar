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
    support/AllocationGuard.cpp
)

# Lets any test include shared test-support headers as "support/<Header>.h" (e.g.
# support/AllocationGuard.h), regardless of which tests/<subdir> the including file lives in.
target_include_directories(cnpg_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
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

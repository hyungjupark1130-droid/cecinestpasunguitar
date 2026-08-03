# cnpg_tests / cnpg_bench / cnpg_render / cnpg_calibrate -- headless verification executables.
#
# cnpg_tests (Catch2 v3 + CTest, catch_discover_tests) lands here starting Task P0.3
# alongside the first [contract] test. Links cnpg_dsp + Catch2::Catch2WithMain only --
# no JUCE, per the naming registry. cnpg_bench (P1.10), cnpg_render (P1.11) and
# cnpg_calibrate (P2.7) have all landed.
#
# cnpg_bench, cnpg_render and cnpg_calibrate live in their own subdirectories with their own
# CMakeLists.txt (the naming registry's file tree, docs/plan.md section 2 line 177-178) since, unlike
# cnpg_tests, they link cnpg_dsp only -- no Catch2 -- and each has its own target-level configuration.

add_subdirectory(bench)     # cnpg_bench    -- Task P1.10
add_subdirectory(render)    # cnpg_render   -- Task P1.11
add_subdirectory(calibrate) # cnpg_calibrate -- Task P2.7

add_executable(cnpg_tests
    dsp/AliasingGateTests.cpp
    dsp/BridgeEnergyTierOneTests.cpp
    dsp/BridgePortContractTests.cpp
    dsp/BridgeTuningClickTests.cpp
    dsp/CorpusSweepTests.cpp
    dsp/CoupledStringsTests.cpp
    dsp/DamperEnergyTests.cpp
    dsp/DamperFeltTimeTests.cpp
    dsp/DamperNodeSuppressionTests.cpp
    dsp/DenormalTests.cpp
    dsp/EventQueueTests.cpp
    dsp/MidiTranslationTests.cpp
    dsp/ModuleGraphTests.cpp
    dsp/MonitoringChainTests.cpp
    dsp/MovingJunctionEnergyTests.cpp
    dsp/MovingPositionClickTests.cpp
    dsp/NetworkEnergyTierThreeTests.cpp
    dsp/NetworkEnergyTierTwoTests.cpp
    dsp/NoteAllocatorFingeringTests.cpp
    dsp/NoteAllocatorTests.cpp
    dsp/NoteAllocatorZonesTests.cpp
    dsp/OutputGainTests.cpp
    dsp/OversamplerTests.cpp
    dsp/ParamStructsTests.cpp
    dsp/PickupTapTests.cpp
    dsp/PitchBendClickTests.cpp
    dsp/PluckExciterTests.cpp
    dsp/RenderTests.cpp
    dsp/RetriggerModeTests.cpp
    dsp/ScopedFtzDazGuardTests.cpp
    dsp/SustainPedalTests.cpp
    dsp/StringNetworkTests.cpp
    dsp/StringNetworkScaleTests.cpp
    dsp/TriodeStageTests.cpp
    dsp/TuningAccuracyTests.cpp
    dsp/WaveguideStringTests.cpp
    dsp/WaveguideStringRegressionTests.cpp
    dsp/WaveguideStringTuningTests.cpp
    support/AllocationGuard.cpp
    support/ClickMetric.cpp
    support/GoldenIo.cpp
    support/SourceHash.cpp
    support/SpectralAnalysis.cpp
    support/StringIrScenarios.cpp
)

# Lets any test include shared test-support headers as "support/<Header>.h" (e.g.
# support/AllocationGuard.h), regardless of which tests/<subdir> the including file lives in.
target_include_directories(cnpg_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# The golden sidecars' provenance field (docs/plan.md section 4.3) is a content hash of
# dsp/include + dsp/src computed at RENDER time from the checkout below -- see
# tests/support/SourceHash.h for why a configure-time `git rev-parse HEAD` could never be correct
# here (it names the parent of the commit that carries the goldens, always) and
# tests/data/golden/README.md for how to verify a recorded hash against any checkout.
target_compile_definitions(cnpg_tests PRIVATE
    CNPG_GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/data/golden"
    CNPG_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    # tests/dsp/RenderTests.cpp drives the real cnpg_render executable rather than a second,
    # hand-duplicated copy of its render loop (Task P1.11; docs/plan.md section 4.8's
    # "RENDER: determinism smoke ... once out-of-process"), so it needs both the corpus location and
    # the binary's own path. $<TARGET_FILE:...> resolves per-config, so the Release and Debug test
    # runs each invoke their own build of the tool rather than whichever one happened to be built
    # last. CMake emits forward slashes here on every platform, so the value is a valid C string
    # literal on Windows too.
    CNPG_CORPUS_DIR="${CMAKE_SOURCE_DIR}/tests/corpus"
    CNPG_RENDER_EXE="$<TARGET_FILE:cnpg_render>"
)

# ...and cnpg_render must actually exist before cnpg_tests runs. This is a build-order dependency
# only (cnpg_tests links nothing from it), which is exactly what add_dependencies expresses.
add_dependencies(cnpg_tests cnpg_render)

target_link_libraries(cnpg_tests PRIVATE
    cnpg_dsp
    Catch2::Catch2WithMain
)

cnpg_set_warnings(cnpg_tests)

include(Catch)
# ADD_TAGS_AS_LABELS mirrors every Catch2 tag (e.g. [contract]) to a CTest label, so
# `ctest -L contract` selects by tag exactly as docs/plan.md section 1.5's conventions
# describe.
#
# DISCOVERY_MODE PRE_TEST is what makes `ctest -C <cfg>` actually honour <cfg> under the
# multi-config VS generator the windows-msvc-* presets use (both presets share binaryDir `build`,
# so Release and Debug are two configurations of ONE build tree). In the default POST_BUILD mode
# Catch2 discovers tests from a post-build command whose output path carries no $<CONFIG>, so both
# configurations write the same `cnpg_tests-<hash>_tests.cmake`, every add_test() in it hard-codes
# an absolute path to whichever configuration was built last, and `cnpg_tests-<hash>_include.cmake`
# includes that one file unconditionally -- leaving ctest nothing to select between. Building Debug
# last therefore made `ctest -C Release` run bin/Debug/cnpg_tests.exe while still reporting itself
# as a Release run, silently voiding the Release half of this project's dual-config verification
# standard (and inflating ctest wall times, which is what made the P2.7 grid gate look ~10x slower
# through ctest than in-process). PRE_TEST instead emits one `..._tests-<CONFIG>.cmake` per
# configuration -- the executable path resolved through $<TARGET_FILE:...> -- plus an include file
# that dispatches on ${CTEST_CONFIGURATION_TYPE}, so the binary that runs always matches -C. It
# also fails loudly (a single cnpg_tests_NOT_BUILT test) when the requested configuration has not
# been built, instead of quietly running the other one.
#
# The cost is that discovery moves from build time to just before the first ctest run, spending one
# `--list-tests --reporter json` invocation per configuration whenever the tests file is older than
# the binary. The ubuntu CI job is unaffected: its linux-dsp-only preset uses Ninja, and for a
# single-config generator PRE_TEST emits one unconditional include file, so
# `ctest --test-dir build-dsp` keeps working with no -C flag. Conversely, under a multi-config
# generator PRE_TEST requires -C: a bare `ctest --test-dir build` now prints "No configuration for
# testing specified" and discovers nothing, which is correct -- there is no longer a "last built"
# configuration for it to guess at.
catch_discover_tests(cnpg_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)

# Regenerate-goldens target (docs/plan.md section 1.6). The section-1.6 draft drives this from
# `cnpg_render --regenerate-goldens`. cnpg_render has now landed (Task P1.11) and this target
# deliberately still does NOT use it: the regeneration entry point stays the hidden ("[.]"-tagged,
# therefore never discovered by CTest and never run in CI) Catch2 case "REGEN: rewrite string_ir
# goldens" inside cnpg_tests, because that keeps a single renderer shared by the goldens and the
# tests they gate -- the property that actually matters, since a golden can never then be written by
# different code than the one that later compares against it. Moving regeneration into cnpg_render
# would mean migrating tests/support/StringIrScenarios (the float64 reference path the goldens ARE)
# out of cnpg_tests and re-establishing that shared-renderer guarantee across a process boundary,
# which is a change to the golden pipeline and not part of Task P1.11's corpus/listening work.
add_custom_target(cnpg_regen_goldens
    COMMAND $<TARGET_FILE:cnpg_tests> "REGEN: rewrite string_ir goldens"
    DEPENDS cnpg_tests
    USES_TERMINAL
    COMMENT "Rewrites float64 golden IRs for every scenario/algorithm variant at 44.1/48/96 kHz")

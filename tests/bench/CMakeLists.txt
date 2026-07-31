# cnpg_bench -- headless dsp/-only benchmark executable (Task P1.10; docs/plan.md section 4.7).
# Links cnpg_dsp ONLY -- no JUCE, no Catch2 -- so it builds under the linux-dsp-only preset
# exactly like cnpg_dsp/cnpg_tests do (the naming registry, docs/plan.md section 1.5).

add_executable(cnpg_bench
    BenchMain.cpp
)

target_link_libraries(cnpg_bench PRIVATE cnpg_dsp)

cnpg_set_warnings(cnpg_bench)

# Best-effort short git commit, baked in at configure time purely for the JSON report's
# provenance field (docs/plan.md section 4.7: "every run emits JSON containing ... git commit").
# Unlike the golden sidecars' content hash (tests/support/SourceHash.h's file-level comment
# explains why a configure-time `git rev-parse HEAD` is structurally wrong for THAT use), a
# benchmark run naming its own commit approximately is the ordinary convention -- every profiler
# does this -- and carries none of the goldens' self-consistency requirement. "unknown" is a
# perfectly fine fallback (a source tarball with no .git directory, or git missing from PATH) and
# never fails the configure step.
find_package(Git QUIET)
set(CNPG_BENCH_GIT_COMMIT "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE CNPG_BENCH_GIT_COMMIT_RESOLVED
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE CNPG_BENCH_GIT_RESULT
        ERROR_QUIET
    )
    if(CNPG_BENCH_GIT_RESULT EQUAL 0 AND NOT CNPG_BENCH_GIT_COMMIT_RESOLVED STREQUAL "")
        set(CNPG_BENCH_GIT_COMMIT "${CNPG_BENCH_GIT_COMMIT_RESOLVED}")
    endif()
endif()

# Build type and build flags, baked in at configure time for the JSON report's "build type AND
# flags" field (docs/plan.md section 4.7's locked schema). CMAKE_BUILD_TYPE is NOT reliable here:
# the windows-msvc-release preset uses the multi-config Visual Studio generator, where the actual
# config is chosen at BUILD time (`cmake --build build --config Release`), not configure time, so
# CMAKE_BUILD_TYPE stays empty for that preset the whole way through. The `$<CONFIG>` generator
# expression is what resolves correctly for both the multi-config VS generator and the
# single-config Ninja generator linux-dsp-only uses (CMAKE_BUILD_TYPE=Release set explicitly
# there) -- CMake substitutes it per build, not per configure, so this stays correct across
# `cmake --build build --config Release` and `cmake --build build --config Debug` from the SAME
# configure.
#
# The flags string mirrors cnpg_set_warnings() (cmake/CompilerWarnings.cmake) exactly -- it has to
# be kept in sync with that function by hand, since a plain string here cannot introspect what a
# function call elsewhere actually applied; there is no single generator expression that reads
# back another target's already-applied compile options portably across the VS and Ninja
# generators this project's two presets use.
if(MSVC)
    set(CNPG_BENCH_BUILD_FLAGS_BASE "/permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc")
    set(CNPG_BENCH_BUILD_FLAGS_WERROR "/WX")
else()
    set(CNPG_BENCH_BUILD_FLAGS_BASE "-Wall -Wextra -Wpedantic -Wshadow -Wconversion")
    set(CNPG_BENCH_BUILD_FLAGS_WERROR "-Werror")
endif()
if(CNPG_WARNINGS_AS_ERRORS)
    set(CNPG_BENCH_BUILD_FLAGS "${CNPG_BENCH_BUILD_FLAGS_BASE} ${CNPG_BENCH_BUILD_FLAGS_WERROR}")
else()
    set(CNPG_BENCH_BUILD_FLAGS "${CNPG_BENCH_BUILD_FLAGS_BASE}")
endif()

target_compile_definitions(cnpg_bench PRIVATE
    CNPG_BENCH_GIT_COMMIT="${CNPG_BENCH_GIT_COMMIT}"
    CNPG_BENCH_BUILD_TYPE="$<CONFIG>"
    CNPG_BENCH_BUILD_FLAGS="${CNPG_BENCH_BUILD_FLAGS}"
)

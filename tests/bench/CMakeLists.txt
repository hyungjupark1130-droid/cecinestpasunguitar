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

target_compile_definitions(cnpg_bench PRIVATE
    CNPG_BENCH_GIT_COMMIT="${CNPG_BENCH_GIT_COMMIT}"
)

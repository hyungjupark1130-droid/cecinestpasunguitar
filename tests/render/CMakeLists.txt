# cnpg_render -- headless MIDI-to-WAV renderer over the P1 dsp/ chain (Task P1.11; docs/plan.md
# section 4.8). Links cnpg_dsp ONLY -- no JUCE, no Catch2 -- so it builds under the linux-dsp-only
# preset exactly like cnpg_dsp/cnpg_tests/cnpg_bench do (the naming registry, docs/plan.md
# section 1.5). The SMF parser and WAV writer next to it are minimal, JUCE-free implementations for
# the same reason (docs/plan.md section 1.5: "Support code the headless executables need (SMF
# parsing, WAV writing) lives in tests/render/ as minimal JUCE-free implementations rather than
# pulling JUCE into the test domain").

add_executable(cnpg_render
    MidiFileReader.cpp
    RenderMain.cpp
    WavWriter.cpp
)

target_link_libraries(cnpg_render PRIVATE cnpg_dsp)

# Lets RenderMain.cpp include the shared P1 chain assembly as "support/P1Chain.h" -- the same
# spelling cnpg_tests and cnpg_bench use. That header is what guarantees cnpg_render renders the
# chain cnpg_bench measures and the plugin plays, rather than a third hand-maintained copy of it.
target_include_directories(cnpg_render PRIVATE ${CMAKE_SOURCE_DIR}/tests)

cnpg_set_warnings(cnpg_render)

# Best-effort short git commit, baked in at configure time. Unlike cnpg_bench's (where it is purely
# a provenance field in a JSON report), this one is load-bearing: docs/plan.md section 4.8 requires
# "Render filenames embed corpus version + git hash so listening notes are attributable", and this
# is where the hash half of that comes from. It carries the same limitation cnpg_bench's does -- a
# configure-time `git rev-parse HEAD` necessarily names the PARENT of the commit that lands the
# binary, never the commit itself (tests/support/SourceHash.h's file comment explains why that
# limitation is fatal for the golden sidecars and merely approximate here) -- which is fine for
# attributing a listening note to a build. "unknown" is an acceptable fallback (a source tarball
# with no .git directory, or git missing from PATH) and never fails the configure step.
find_package(Git QUIET)
set(CNPG_RENDER_GIT_COMMIT "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE CNPG_RENDER_GIT_COMMIT_RESOLVED
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE CNPG_RENDER_GIT_RESULT
        ERROR_QUIET
    )
    if(CNPG_RENDER_GIT_RESULT EQUAL 0 AND NOT CNPG_RENDER_GIT_COMMIT_RESOLVED STREQUAL "")
        set(CNPG_RENDER_GIT_COMMIT "${CNPG_RENDER_GIT_COMMIT_RESOLVED}")
    endif()
endif()

target_compile_definitions(cnpg_render PRIVATE
    CNPG_RENDER_GIT_COMMIT="${CNPG_RENDER_GIT_COMMIT}"
)

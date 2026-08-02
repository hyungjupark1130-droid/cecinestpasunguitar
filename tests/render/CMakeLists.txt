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

# THE PROVENANCE FIELD IS A RENDER-TIME CONTENT HASH, NOT A CONFIGURE-TIME GIT COMMIT (Task P2.7).
#
# This block used to resolve `git rev-parse --short HEAD` here and bake it in, with a comment
# claiming it "necessarily names the PARENT of the commit that lands the binary". That claim was
# false in the direction that matters: a configure-time value is resolved when CMake LAST RAN, so a
# build/ tree configured at 77b0430 produced renders from the code at 1ccfcb1 and filed them as
# `..._cv1_g77b0430.wav` -- three commits stale, and two different code states writing identical
# filenames. docs/plan.md section 4.8 requires those filenames to make listening notes attributable,
# and P2.8 is the listening pass, so it is fixed here rather than lived with.
#
# cnpg_render now links tests/support/SourceHash.cpp and computes the digest at RENDER time over the
# bytes a render is actually a function of (see tests/support/SourceHash.h::renderSourceHash). That
# file has no Catch2 dependency, so this stays a cnpg_dsp-only executable and still builds under the
# linux-dsp-only preset.
target_sources(cnpg_render PRIVATE ${CMAKE_SOURCE_DIR}/tests/support/SourceHash.cpp)

target_compile_definitions(cnpg_render PRIVATE
    CNPG_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

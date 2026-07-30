include(FetchContent)

# JUCE 8 -- latest stable 8-series tag as of pin date 2026-07-30: 8.0.15.
# JUCE 9.0.0 exists upstream but JUCE 8 is the locked major for P0-P2 (see docs/plan.md).
# GPL path, no splash screen (JUCE_DISPLAY_SPLASH_SCREEN=0 is set in plugin/CMakeLists.txt,
# landing in P0.4).
# JUCE's 8.x release tags are lightweight (not annotated) upstream -- verified via
# `git ls-remote --tags https://github.com/juce-framework/JUCE.git` on 2026-07-30, which
# shows no dereferenced (^{}) entry for any 6.1.0+ tag. A lightweight tag ref points directly
# at the commit, so the SHA below is both the tag ref and the resolved commit.
# Resolved commit SHA (git ls-remote, pinned 2026-07-30): 91ad83ae34a81e0833b1a2b0866f54846370ae53
# Consumed by .github/scripts/verify-pins.cmake (P0.7) to detect tag drift.
FetchContent_Declare(JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG        8.0.15
    GIT_SHALLOW    TRUE)

# Catch2 v3 -- latest v3.x tag as of pin date 2026-07-30: v3.15.3.
# This is an annotated tag -- verified via `git ls-remote --tags
# https://github.com/catchorg/Catch2.git` on 2026-07-30, which shows a dereferenced (^{})
# entry for it.
# Tag object SHA:        95d8a61b089317bec800c7cc4c64064cbcb3802d
# Resolved commit SHA (dereferenced, pinned 2026-07-30): 8b08d4d79514f45f7e4ce2a607ac9c94e920d1bb
# Consumed by .github/scripts/verify-pins.cmake (P0.7) to detect tag drift.
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.15.3
    GIT_SHALLOW    TRUE)

# JUCE is only fetched for the full build. The ubuntu dsp-only CI job (and the
# linux-dsp-only preset) sets CNPG_BUILD_PLUGIN=OFF, so JUCE's large checkout never happens
# there.
if(CNPG_BUILD_PLUGIN)
    FetchContent_MakeAvailable(JUCE)
endif()

# Catch2 is always fetched: dsp/ tests need it headless, even under linux-dsp-only.
# CNPG_BUILD_TESTS defaults ON and is never toggled off by any current preset.
if(CNPG_BUILD_TESTS)
    FetchContent_MakeAvailable(Catch2)
endif()

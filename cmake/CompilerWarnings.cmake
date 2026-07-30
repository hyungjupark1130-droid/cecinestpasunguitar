# cnpg_set_warnings(<target>)
#
# Applies the project's shared compiler-warning policy to <target>. Only cnpg-authored
# targets (cnpg_dsp, cnpg_plugin, cnpg_tests, cnpg_bench, cnpg_render, cnpg_calibrate) call
# this. FetchContent'd dependencies (JUCE, Catch2) build under their own settings and are
# never touched here -- CNPG_WARNINGS_AS_ERRORS must never leak into third-party code.
#
# MSVC is the primary dev-machine toolchain; GCC/Clang are exercised by the ubuntu CI job's
# linux-dsp-only build as a portability guard. Both must stay warning-clean.
function(cnpg_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /permissive- /W4 /Zc:__cplusplus /utf-8)
        if(CNPG_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
        if(CNPG_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

# cnpg_set_warnings(<target>)
#
# Applies the project's shared compiler-warning policy to <target>. Only cnpg-authored
# targets (cnpg_dsp, cnpg_plugin, cnpg_tests, cnpg_bench, cnpg_render, cnpg_calibrate) call
# this. FetchContent'd dependencies (JUCE, Catch2) build under their own settings and are
# never touched here -- CNPG_WARNINGS_AS_ERRORS must never leak into third-party code.
#
# MSVC is the primary dev-machine toolchain; GCC/Clang are exercised by the ubuntu CI job's
# linux-dsp-only build as a portability guard. Both must stay warning-clean.
#
# /EHsc (standard C++ exception-handling model, synchronous, extern "C" assumed nothrow) is
# included here because it is generator-dependent whether it is on by default: CMake's
# Visual Studio generator bakes ExceptionHandling=Sync into every .vcxproj, but the Ninja
# generator (used by the linux-dsp-only preset even when the compiler resolved on PATH is
# cl.exe, as happens when exercising that preset locally on this Windows dev machine) does
# not add any default -- omitting it left <chrono>'s (and other STL headers') internal
# try/catch usage tripping MSVC warning C4530, fatal once CNPG_WARNINGS_AS_ERRORS promotes it
# via /WX. Setting it explicitly here makes both generators behave identically.
function(cnpg_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /permissive- /W4 /Zc:__cplusplus /utf-8 /EHsc)
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

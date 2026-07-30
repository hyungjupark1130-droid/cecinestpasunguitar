# .github/scripts/verify-pins.cmake -- P0 pin-immutability guard (Task P0.7).
#
# cmake/Dependencies.cmake pins JUCE and Catch2 by a human-readable GIT_TAG with GIT_SHALLOW
# TRUE (a real commit-SHA GIT_TAG and GIT_SHALLOW don't mix). Immutability is enforced here
# instead, per docs/plan.md section 1.3: this script re-resolves each tag against its live
# remote via `git ls-remote` and compares the resolved commit SHA against the SHA recorded as a
# "Resolved commit SHA...:" comment immediately above that dependency's FetchContent_Declare(...)
# call in cmake/Dependencies.cmake. A mismatch means the tag has moved (or drifted) since that
# comment was written; CI fails so any legitimate dependency bump becomes a deliberate, reviewed
# commit touching Dependencies.cmake instead of silently changing what gets built.
#
# Usage: cmake -P .github/scripts/verify-pins.cmake
# Side effects: read-only `git ls-remote` network calls against the pinned dependency
# repositories only (no local clone, no writes, no state touched in this repository).

cmake_minimum_required(VERSION 3.24)

get_filename_component(CNPG_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(CNPG_DEPENDENCIES_FILE "${CNPG_REPO_ROOT}/cmake/Dependencies.cmake")

if(NOT EXISTS "${CNPG_DEPENDENCIES_FILE}")
    message(FATAL_ERROR "verify-pins.cmake: cannot find ${CNPG_DEPENDENCIES_FILE}")
endif()

find_program(CNPG_GIT_EXECUTABLE git)
if(NOT CNPG_GIT_EXECUTABLE)
    message(FATAL_ERROR "verify-pins.cmake: 'git' executable not found on PATH")
endif()

file(STRINGS "${CNPG_DEPENDENCIES_FILE}" CNPG_DEP_LINES)

# Parser state.
set(CNPG_LAST_SEEN_SHA "")
set(CNPG_IN_DECLARE FALSE)
set(CNPG_CURRENT_NAME "")
set(CNPG_CURRENT_REPO "")
set(CNPG_CURRENT_TAG "")
set(CNPG_CURRENT_EXPECTED_SHA "")
set(CNPG_PIN_NAMES "")

foreach(line IN LISTS CNPG_DEP_LINES)
    # Track the most recently seen "Resolved commit SHA...: <hex-sha>" comment. Each pin's
    # block carries exactly one of these immediately above its FetchContent_Declare(...) call
    # (see cmake/Dependencies.cmake); a "Tag object SHA:" comment (Catch2's annotated-tag object,
    # as opposed to the commit it points at) is deliberately NOT matched here.
    if(line MATCHES "Resolved commit SHA.*:[ \t]*([0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]+)[ \t]*$")
        set(CNPG_LAST_SEEN_SHA "${CMAKE_MATCH_1}")
    endif()

    if(NOT CNPG_IN_DECLARE)
        if(line MATCHES "^[ \t]*FetchContent_Declare\\(([A-Za-z0-9_]+)")
            set(CNPG_IN_DECLARE TRUE)
            set(CNPG_CURRENT_NAME "${CMAKE_MATCH_1}")
            set(CNPG_CURRENT_REPO "")
            set(CNPG_CURRENT_TAG "")
            set(CNPG_CURRENT_EXPECTED_SHA "${CNPG_LAST_SEEN_SHA}")
        endif()
    else()
        if(line MATCHES "GIT_REPOSITORY[ \t]+([^\t )]+)")
            set(CNPG_CURRENT_REPO "${CMAKE_MATCH_1}")
        elseif(line MATCHES "GIT_TAG[ \t]+([^\t )]+)")
            set(CNPG_CURRENT_TAG "${CMAKE_MATCH_1}")
        endif()

        if(line MATCHES "\\)[ \t]*$")
            set(CNPG_IN_DECLARE FALSE)
            if(CNPG_CURRENT_REPO AND CNPG_CURRENT_TAG)
                list(APPEND CNPG_PIN_NAMES "${CNPG_CURRENT_NAME}")
                set(CNPG_REPO_${CNPG_CURRENT_NAME} "${CNPG_CURRENT_REPO}")
                set(CNPG_TAG_${CNPG_CURRENT_NAME} "${CNPG_CURRENT_TAG}")
                set(CNPG_SHA_${CNPG_CURRENT_NAME} "${CNPG_CURRENT_EXPECTED_SHA}")
            endif()
        endif()
    endif()
endforeach()

list(LENGTH CNPG_PIN_NAMES CNPG_PIN_COUNT)
if(CNPG_PIN_COUNT EQUAL 0)
    message(FATAL_ERROR
        "verify-pins.cmake: parsed zero FetchContent_Declare(...) pins out of "
        "${CNPG_DEPENDENCIES_FILE} -- the parser or the file's format changed; fix this script.")
endif()

message(STATUS "verify-pins.cmake: checking ${CNPG_PIN_COUNT} pin(s): ${CNPG_PIN_NAMES}")

set(CNPG_ANY_MISMATCH FALSE)

foreach(name IN LISTS CNPG_PIN_NAMES)
    set(repo "${CNPG_REPO_${name}}")
    set(tag "${CNPG_TAG_${name}}")
    set(expected_sha "${CNPG_SHA_${name}}")

    if(NOT expected_sha)
        message(FATAL_ERROR
            "verify-pins.cmake: no 'Resolved commit SHA' comment found above the ${name} "
            "FetchContent_Declare(...) block -- every pin must record one; see "
            "cmake/Dependencies.cmake.")
    endif()

    # Request the exact tag ref AND its peeled ("^{}") form as two explicit refspecs. This is
    # deliberately NOT `git ls-remote --tags <repo> <tag>`: passing the bare tag name as a
    # ls-remote pattern only matches that literal ref name, which excludes the peeled
    # "refs/tags/<tag>^{}" ref for annotated tags (verified empirically -- `--tags <repo> <tag>`
    # silently drops the ^{} line even though it exists upstream, which would make this script
    # compare against an annotated tag's TAG OBJECT sha instead of the commit it points at, and
    # false-positive "drift" forever). Asking for both exact refs by full path sidesteps that.
    # A lightweight tag (e.g. JUCE's) simply has no matching "^{}" ref and only the first line
    # comes back, which the fallback below handles.
    execute_process(
        COMMAND "${CNPG_GIT_EXECUTABLE}" ls-remote "${repo}" "refs/tags/${tag}" "refs/tags/${tag}^{}"
        OUTPUT_VARIABLE CNPG_LS_REMOTE_OUTPUT
        ERROR_VARIABLE CNPG_LS_REMOTE_ERROR
        RESULT_VARIABLE CNPG_LS_REMOTE_RESULT
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT CNPG_LS_REMOTE_RESULT EQUAL 0)
        message(FATAL_ERROR
            "verify-pins.cmake: 'git ls-remote ${repo} refs/tags/${tag} refs/tags/${tag}^{}' "
            "failed (exit ${CNPG_LS_REMOTE_RESULT}): ${CNPG_LS_REMOTE_ERROR}")
    endif()

    if(CNPG_LS_REMOTE_OUTPUT STREQUAL "")
        message(FATAL_ERROR
            "verify-pins.cmake: 'git ls-remote ${repo} refs/tags/${tag} refs/tags/${tag}^{}' "
            "returned no matching ref -- tag renamed, deleted upstream, or repository unreachable.")
    endif()

    # ls-remote prints one line per matching ref. An annotated tag additionally prints a
    # dereferenced "<tag>^{}" line whose SHA is the actual commit the tag points at (the plain
    # "<tag>" line for an annotated tag is the tag OBJECT's SHA, not the commit). A lightweight
    # tag only ever prints the plain line, which already is the commit SHA. Prefer the
    # dereferenced line when present.
    set(CNPG_RESOLVED_SHA "")
    string(REPLACE "\n" ";" CNPG_LS_REMOTE_LINES "${CNPG_LS_REMOTE_OUTPUT}")
    foreach(refline IN LISTS CNPG_LS_REMOTE_LINES)
        if(refline MATCHES "^([0-9a-fA-F]+)[ \t]+refs/tags/.*\\^\\{\\}$")
            set(CNPG_RESOLVED_SHA "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    if(CNPG_RESOLVED_SHA STREQUAL "")
        foreach(refline IN LISTS CNPG_LS_REMOTE_LINES)
            if(refline MATCHES "^([0-9a-fA-F]+)[ \t]+refs/tags/")
                set(CNPG_RESOLVED_SHA "${CMAKE_MATCH_1}")
            endif()
        endforeach()
    endif()

    if(CNPG_RESOLVED_SHA STREQUAL "")
        message(FATAL_ERROR
            "verify-pins.cmake: could not parse a commit SHA out of git ls-remote output for "
            "${name}:\n${CNPG_LS_REMOTE_OUTPUT}")
    endif()

    string(TOLOWER "${CNPG_RESOLVED_SHA}" CNPG_RESOLVED_SHA_LOWER)
    string(TOLOWER "${expected_sha}" CNPG_EXPECTED_SHA_LOWER)

    if(NOT CNPG_RESOLVED_SHA_LOWER STREQUAL CNPG_EXPECTED_SHA_LOWER)
        message(WARNING
            "verify-pins.cmake: PIN DRIFT on ${name} (tag ${tag}): recorded SHA ${expected_sha} "
            "in cmake/Dependencies.cmake does not match the live tag's resolved commit SHA "
            "${CNPG_RESOLVED_SHA} from ${repo}.")
        set(CNPG_ANY_MISMATCH TRUE)
    else()
        message(STATUS "verify-pins.cmake: OK  ${name} ${tag} -> ${CNPG_RESOLVED_SHA}")
    endif()
endforeach()

if(CNPG_ANY_MISMATCH)
    message(FATAL_ERROR
        "verify-pins.cmake: one or more dependency tags have drifted from the commit SHA "
        "recorded for them in cmake/Dependencies.cmake. If this is a deliberate dependency "
        "bump, update GIT_TAG and its 'Resolved commit SHA' comment together in a dedicated, "
        "reviewed commit (see docs/plan.md section 1.3). If it is not deliberate, investigate "
        "before merging.")
endif()

message(STATUS "verify-pins.cmake: all pins immutable, no drift detected.")

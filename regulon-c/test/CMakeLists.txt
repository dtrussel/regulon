# =============================================================================
# test/CMakeLists.txt — Test build for the Regulon C11 library
#
# Builds the Unity framework, one executable per test suite, and registers
# each executable with CTest.
#
# RON-TP-001 §2.3 | RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

# ---------------------------------------------------------------------------
# Unity test framework (vendored, MIT)
# ---------------------------------------------------------------------------
add_library(unity STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/framework/unity/unity.c
)
target_include_directories(unity
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/framework/unity
)
target_compile_options(unity PRIVATE
    $<$<C_COMPILER_ID:GNU>:  -Wall -Wextra -Wno-unused-parameter>
    $<$<C_COMPILER_ID:Clang>:-Wall -Wextra -Wno-unused-parameter>
)

# ---------------------------------------------------------------------------
# Helper macro: add one test suite executable and register it with CTest
# ---------------------------------------------------------------------------
#
# Usage:
#   ron_add_test(test_name source_file [extra_source ...])
#
# Creates an executable named <test_name>, links it against the regulon
# library and Unity, then registers it with add_test() so that 'ctest' picks
# it up automatically.
# ---------------------------------------------------------------------------
macro(ron_add_test test_name)
    add_executable(${test_name} ${ARGN})
    target_link_libraries(${test_name} PRIVATE
        regulon
        unity
        $<$<AND:$<NOT:$<C_COMPILER_ID:MSVC>>,$<NOT:$<PLATFORM_ID:Windows>>>:m>
    )
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/framework/unity
    )
    target_compile_options(${test_name} PRIVATE
        $<$<C_COMPILER_ID:GNU>:
            -Wall -Wextra -Wpedantic -Werror
            -Wconversion -Wshadow
        >
        $<$<C_COMPILER_ID:Clang>:
            -Wall -Wextra -Wpedantic -Werror
            -Wconversion -Wshadow
        >
        $<$<C_COMPILER_ID:MSVC>:
            /W4 /WX /wd4127
        >
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
    # Propagate sanitiser flags from the parent CMake invocation
    set_tests_properties(${test_name} PROPERTIES
        ENVIRONMENT "ASAN_OPTIONS=detect_leaks=1"
    )
endmacro()

# ---------------------------------------------------------------------------
# Unit test suites
# ---------------------------------------------------------------------------
# Sprint 1 — Type definitions and compile-time size assertions
ron_add_test(test_ron_pid_types  unit/test_ron_pid_types.c)

# Sprint 2 — PID config validation and core algorithm (TC-PID-001..014)
ron_add_test(test_ron_pid_core   unit/test_ron_pid_core.c)

# Sprint 3 — PID API, fault detection, anti-windup (TC-PID-015..039)
ron_add_test(test_ron_pid_api    unit/test_ron_pid_api.c)

# Sprint 4 — Signal conditioning filters (TC-FILT-001..017)
ron_add_test(test_ron_filter     unit/test_ron_filter.c)

# Sprint 5 - Feed-forward PID extension (TC-FF-001..009)
ron_add_test(test_ron_feedforward unit/test_ron_feedforward.c)

# Sprint 6 - Gain scheduling (TC-GS-001..008)
ron_add_test(test_ron_gain_sched unit/test_ron_gain_sched.c)

# Sprint 7 - Trajectory generators (TC-TRAJ-001..008)
ron_add_test(test_ron_trajectory unit/test_ron_trajectory.c)

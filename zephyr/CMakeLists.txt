# =============================================================================
# Zephyr build glue for the Regulon control systems library.
#
# Compiles the library into a Zephyr application using the same source
# selection as the standalone CMake build (regulon-c/CMakeLists.txt), driven
# by the Kconfig options in zephyr/Kconfig instead of RON_ENABLE_* cache
# variables.
#
# The generated ron/ron_modules.h is produced here from the same template the
# standalone build uses, so the aggregate ron/ron.h includes exactly the
# headers whose implementations were compiled in - regardless of which build
# system produced the library.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

if(NOT CONFIG_REGULON)
    return()
endif()

set(RON_ROOT ${CMAKE_CURRENT_LIST_DIR}/../regulon-c)

zephyr_library_named(regulon)

# ---------------------------------------------------------------------------
# Generated module-availability header (ron/ron_modules.h)
#
# The template expects RON_HAVE_<MODULE> to be 1 or 0; Kconfig gives us "y"
# or an empty string, so translate rather than substituting directly.
# ---------------------------------------------------------------------------
foreach(_mod FILTER GAIN_SCHED TRAJECTORY CASCADE KALMAN STATESPACE LQR LQG AUTOTUNE HEALTH METRICS)
    if(CONFIG_REGULON_${_mod})
        set(RON_HAVE_${_mod} 1)
    else()
        set(RON_HAVE_${_mod} 0)
    endif()
endforeach()

set(RON_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
configure_file(
    ${RON_ROOT}/cmake/ron_modules.h.in
    ${RON_GENERATED_INCLUDE_DIR}/ron/ron_modules.h
    @ONLY
)

# Public to the whole application: consumers include <ron/ron.h> directly.
zephyr_include_directories(
    ${RON_ROOT}/include
    ${RON_GENERATED_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# Sources: mandatory baseline plus the selected modules
# ---------------------------------------------------------------------------
zephyr_library_sources(
    ${RON_ROOT}/src/ron_pid_api.c
    ${RON_ROOT}/src/ron_pid_config.c
    ${RON_ROOT}/src/ron_pid_core.c
    ${RON_ROOT}/src/ron_pid_fault.c
    ${RON_ROOT}/src/ron_feedforward.c
)

zephyr_library_sources_ifdef(CONFIG_REGULON_FILTER     ${RON_ROOT}/src/ron_filter.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_GAIN_SCHED ${RON_ROOT}/src/ron_gain_sched.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_CASCADE    ${RON_ROOT}/src/ron_cascade.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_AUTOTUNE   ${RON_ROOT}/src/ron_autotune.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_HEALTH     ${RON_ROOT}/src/ron_health.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_METRICS    ${RON_ROOT}/src/ron_metrics.c)

zephyr_library_sources_ifdef(CONFIG_REGULON_TRAJECTORY
    ${RON_ROOT}/src/ron_trajectory_trap.c
    ${RON_ROOT}/src/ron_trajectory_scurve.c
)

# Matrix / state-estimation modules share the internal ron_matrix helper.
# Kconfig `select` guarantees STATESPACE/LQR/LQG imply KALMAN, so testing
# KALMAN alone is sufficient here.
zephyr_library_sources_ifdef(CONFIG_REGULON_KALMAN
    ${RON_ROOT}/src/ron_matrix.c
    ${RON_ROOT}/src/ron_kalman.c
)

zephyr_library_sources_ifdef(CONFIG_REGULON_STATESPACE
    ${RON_ROOT}/src/ron_statespace.c
    ${RON_ROOT}/src/ron_observer.c
)

zephyr_library_sources_ifdef(CONFIG_REGULON_LQR ${RON_ROOT}/src/ron_lqr.c)
zephyr_library_sources_ifdef(CONFIG_REGULON_LQG ${RON_ROOT}/src/ron_lqg.c)

# ---------------------------------------------------------------------------
# Compile-time options
#
# Precision is part of the ABI: ron_float_t must be the same type in the
# library and in every translation unit that includes its headers, so this is
# defined application-wide rather than only on the library target.
# ---------------------------------------------------------------------------
if(CONFIG_REGULON_DOUBLE_PRECISION)
    zephyr_compile_definitions(RON_USE_DOUBLE=1)
endif()

if(CONFIG_REGULON_ASSERT)
    zephyr_library_compile_definitions(
        "RON_ASSERT(cond)=do { if(!(cond)) { __builtin_trap(); } } while(0)"
    )
endif()

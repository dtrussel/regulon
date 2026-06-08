# =============================================================================
# regulon-c/zephyr/CMakeLists.txt — Zephyr module build glue
#
# Compiles the Regulon library into a zephyr_library() when CONFIG_REGULON is
# set, gating each optional module on its CONFIG_REGULON_<MODULE> symbol. The
# library production sources live in ../src and are reused verbatim; the
# generated ron/ron_modules.h is produced from the shared cmake/ron_modules.h.in
# template (the same single source of truth used by the standalone build).
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

if(CONFIG_REGULON)

  set(RON_ROOT ${ZEPHYR_CURRENT_MODULE_DIR}/regulon-c)

  # --- Generated module-availability header (driven by Kconfig) -------------
  foreach(_mod FILTER GAIN_SCHED TRAJECTORY CASCADE KALMAN STATESPACE
               AUTOTUNE HEALTH METRICS)
    if(CONFIG_REGULON_${_mod})
      set(RON_HAVE_${_mod} 1)
    else()
      set(RON_HAVE_${_mod} 0)
    endif()
  endforeach()

  set(RON_ZEPHYR_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/regulon_generated/include)
  configure_file(
    ${RON_ROOT}/cmake/ron_modules.h.in
    ${RON_ZEPHYR_GEN_DIR}/ron/ron_modules.h
    @ONLY)

  zephyr_library()
  zephyr_include_directories(${RON_ROOT}/include ${RON_ZEPHYR_GEN_DIR})

  # --- Mandatory baseline: PID core + integrated feed-forward ---------------
  zephyr_library_sources(
    ${RON_ROOT}/src/ron_pid_api.c
    ${RON_ROOT}/src/ron_pid_config.c
    ${RON_ROOT}/src/ron_pid_core.c
    ${RON_ROOT}/src/ron_pid_fault.c
    ${RON_ROOT}/src/ron_feedforward.c)

  # --- Optional standalone / PID-dependent modules --------------------------
  zephyr_library_sources_ifdef(CONFIG_REGULON_FILTER
    ${RON_ROOT}/src/ron_filter.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_GAIN_SCHED
    ${RON_ROOT}/src/ron_gain_sched.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_TRAJECTORY
    ${RON_ROOT}/src/ron_trajectory_trap.c
    ${RON_ROOT}/src/ron_trajectory_scurve.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_CASCADE
    ${RON_ROOT}/src/ron_cascade.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_AUTOTUNE
    ${RON_ROOT}/src/ron_autotune.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_HEALTH
    ${RON_ROOT}/src/ron_health.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_METRICS
    ${RON_ROOT}/src/ron_metrics.c)

  # --- Matrix / state-estimation modules ------------------------------------
  # STATESPACE selects KALMAN in Kconfig, so the shared ron_matrix helper is
  # added exactly once via the KALMAN branch.
  zephyr_library_sources_ifdef(CONFIG_REGULON_KALMAN
    ${RON_ROOT}/src/ron_matrix.c
    ${RON_ROOT}/src/ron_kalman.c)
  zephyr_library_sources_ifdef(CONFIG_REGULON_STATESPACE
    ${RON_ROOT}/src/ron_statespace.c
    ${RON_ROOT}/src/ron_observer.c)

  # --- Compile-time options -------------------------------------------------
  zephyr_compile_definitions_ifdef(CONFIG_REGULON_USE_DOUBLE RON_USE_DOUBLE=1)
  zephyr_compile_definitions_ifdef(CONFIG_REGULON_ENABLE_ASSERT
    "RON_ASSERT(cond)=do { if (!(cond)) { __builtin_trap(); } } while (0)")

endif() # CONFIG_REGULON

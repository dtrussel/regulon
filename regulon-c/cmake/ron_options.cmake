# =============================================================================
# ron_options.cmake — Build options for the Regulon C11 library
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

# ---------------------------------------------------------------------------
# Global build options
# ---------------------------------------------------------------------------
option(RON_USE_DOUBLE     "Use 64-bit double precision (default: 32-bit float)"  OFF)
option(RON_BUILD_TESTS    "Build unit and integration tests"                      ON)
option(RON_BUILD_EXAMPLES "Build host example programs (host only)"               OFF)
option(RON_BUILD_BENCHMARKS "Build host timing benchmarks (host only)"             OFF)
option(RON_ENABLE_ASSERT  "Enable runtime RON_ASSERT checks (uses __builtin_trap)" OFF)

# ---------------------------------------------------------------------------
# Per-module selection (RON-IS-001 §8.1)
#
# The PID core (ron_pid_*.c) and its integrated feed-forward path
# (ron_feedforward.c, required by ron_pid_config_validate) form the mandatory
# baseline and are always built.  The options below let constrained targets
# compile only the modules they need; each defaults ON so the default build is
# the complete library.  Dependencies are resolved in the top-level
# CMakeLists.txt (e.g. RON_ENABLE_STATESPACE forces RON_ENABLE_KALMAN).
# ---------------------------------------------------------------------------
option(RON_ENABLE_FILTER     "Build signal-conditioning filters (ron_filter)"      ON)
option(RON_ENABLE_GAIN_SCHED "Build gain scheduling (ron_gain_sched)"              ON)
option(RON_ENABLE_TRAJECTORY "Build trajectory generators (ron_trajectory)"        ON)
option(RON_ENABLE_CASCADE    "Build cascade controller (ron_cascade)"              ON)
option(RON_ENABLE_KALMAN     "Build discrete Kalman filter (ron_kalman)"           ON)
option(RON_ENABLE_STATESPACE "Build state-space controller + observer (forces KALMAN)" ON)
option(RON_ENABLE_LQR        "Build LQR optimal state-feedback controller (forces KALMAN+STATESPACE)" ON)
option(RON_ENABLE_LQG        "Build LQG controller (forces KALMAN+LQR)"             ON)
option(RON_ENABLE_AUTOTUNE   "Build relay-feedback auto-tuner (ron_autotune)"      ON)
option(RON_ENABLE_HEALTH     "Build control-loop health monitor (ron_health)"      ON)
option(RON_ENABLE_METRICS    "Build runtime performance metrics (ron_metrics)"     ON)

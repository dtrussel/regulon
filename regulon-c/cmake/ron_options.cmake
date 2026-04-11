# =============================================================================
# ron_options.cmake — Build options for the Regulon C11 library
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

option(RON_USE_DOUBLE    "Use 64-bit double precision (default: 32-bit float)"  OFF)
option(RON_BUILD_TESTS   "Build unit and integration tests"                      ON)
option(RON_ENABLE_ASSERT "Enable runtime RON_ASSERT checks (uses __builtin_trap)" OFF)

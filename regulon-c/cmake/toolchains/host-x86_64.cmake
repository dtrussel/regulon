# =============================================================================
# host-x86_64.cmake — Host toolchain for unit testing on x86-64
#
# This is the default when no toolchain file is specified.
# Included for completeness and to document the expected host environment.
#
# RON-IS-001 §8.1 | RON-TP-001 ENV-HOST
# SPDX-License-Identifier: MIT
# =============================================================================

set(CMAKE_SYSTEM_NAME    ${CMAKE_HOST_SYSTEM_NAME})
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Use the default system compilers; prefer Clang if available.
# Override with -DCMAKE_C_COMPILER=gcc or -DCMAKE_C_COMPILER=clang.
find_program(CLANG_FOUND clang)
if(CLANG_FOUND AND NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER clang)
endif()

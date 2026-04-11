# =============================================================================
# arm-none-eabi.cmake — Cross-compile toolchain for ARM Cortex-M targets
#
# Usage:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
#         -DCMAKE_C_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb" \
#         -B build_arm -S regulon-c/
#
# RON-IS-001 §8.1 | RON-DC-005
# SPDX-License-Identifier: MIT
# =============================================================================

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)

set(CMAKE_C_COMPILER        arm-none-eabi-gcc)
set(CMAKE_AR                arm-none-eabi-ar)
set(CMAKE_RANLIB             arm-none-eabi-ranlib)
set(CMAKE_SIZE               arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXE_LINKER_FLAGS_INIT   "-nostartfiles -nostdlib")

# Prevent CMake from searching host system paths for libraries/includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

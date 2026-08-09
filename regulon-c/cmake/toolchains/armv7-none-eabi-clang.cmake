# =============================================================================
# armv7-none-eabi-clang.cmake - Cross-compile toolchain for ARMv7 bare-metal
# targets using LLVM/Clang.
#
# Usage:
#   cmake -G Ninja \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/armv7-none-eabi-clang.cmake \
#         -B build_armv7_clang -S regulon-c/
#
# Optional cache variables:
#   RON_ARM_CLANG_TARGET  LLVM target triple, defaults to armv7-none-eabi.
#   RON_ARM_CLANG_SYSROOT Optional C library/sysroot. The library itself is
#                         freestanding and needs none; this is only useful if
#                         you build the tests or examples for the target.
#   RON_ARM_CLANG_CPU     Optional -mcpu value.
#   RON_ARM_CLANG_ARCH    Optional -march value.
#   RON_ARM_CLANG_THUMB   Enable Thumb code generation, defaults ON.
#
# RON-IS-001 Section 8.1 | RON-DC-005
# SPDX-License-Identifier: MIT
# =============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR armv7)

set(RON_ARM_CLANG_TARGET
    "armv7-none-eabi"
    CACHE STRING "LLVM target triple for the ARMv7 bare-metal C build")
set(RON_ARM_CLANG_SYSROOT
    ""
    CACHE PATH "Optional sysroot containing ARM bare-metal C headers/libraries")
set(RON_ARM_CLANG_CPU
    ""
    CACHE STRING "Optional ARM CPU passed to Clang with -mcpu")
set(RON_ARM_CLANG_ARCH
    ""
    CACHE STRING "Optional ARM architecture passed to Clang with -march")
set(RON_ARM_CLANG_THUMB
    ON
    CACHE BOOL "Build ARMv7 objects in Thumb mode")

if(NOT DEFINED CMAKE_C_COMPILER)
    find_program(RON_ARM_CLANG_C_COMPILER NAMES clang REQUIRED)
else()
    set(RON_ARM_CLANG_C_COMPILER "${CMAKE_C_COMPILER}")
endif()
find_program(RON_ARM_CLANG_AR NAMES llvm-ar REQUIRED)
find_program(RON_ARM_CLANG_RANLIB NAMES llvm-ranlib REQUIRED)
find_program(RON_ARM_CLANG_SIZE NAMES llvm-size)

set(CMAKE_C_COMPILER
    "${RON_ARM_CLANG_C_COMPILER}"
    CACHE FILEPATH "Clang C compiler for ARMv7 bare-metal" FORCE)
set(CMAKE_C_COMPILER_TARGET
    "${RON_ARM_CLANG_TARGET}"
    CACHE STRING "Clang C target triple" FORCE)
set(CMAKE_AR
    "${RON_ARM_CLANG_AR}"
    CACHE FILEPATH "LLVM archiver for ARMv7 bare-metal" FORCE)
set(CMAKE_RANLIB
    "${RON_ARM_CLANG_RANLIB}"
    CACHE FILEPATH "LLVM ranlib for ARMv7 bare-metal" FORCE)
if(RON_ARM_CLANG_SIZE)
    set(CMAKE_SIZE
        "${RON_ARM_CLANG_SIZE}"
        CACHE FILEPATH "LLVM size tool for ARMv7 bare-metal" FORCE)
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -nostdlib")

# The library is freestanding (RON-DC-002): the only headers it includes are
# <stdint.h>, <stdbool.h>, <float.h> and <stddef.h>, all of which Clang
# provides itself. No libc, no sysroot, and no -lm are needed to build it.
set(RON_ARM_CLANG_INITIAL_FLAGS "-ffreestanding -fno-builtin")
if(RON_ARM_CLANG_THUMB)
    string(APPEND RON_ARM_CLANG_INITIAL_FLAGS " -mthumb")
endif()
if(NOT "${RON_ARM_CLANG_CPU}" STREQUAL "")
    string(APPEND RON_ARM_CLANG_INITIAL_FLAGS " -mcpu=${RON_ARM_CLANG_CPU}")
endif()
if(NOT "${RON_ARM_CLANG_ARCH}" STREQUAL "")
    string(APPEND RON_ARM_CLANG_INITIAL_FLAGS " -march=${RON_ARM_CLANG_ARCH}")
endif()
if(NOT "${RON_ARM_CLANG_SYSROOT}" STREQUAL "")
    set(CMAKE_SYSROOT "${RON_ARM_CLANG_SYSROOT}" CACHE PATH "ARMv7 bare-metal sysroot" FORCE)
endif()
set(CMAKE_C_FLAGS_INIT "${RON_ARM_CLANG_INITIAL_FLAGS}")

# Prevent CMake from searching host system paths for target libraries/includes.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

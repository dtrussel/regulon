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
#   RON_ARM_CLANG_SYSROOT Optional C library/sysroot for bare-metal headers.
#   RON_ARM_CLANG_NEWLIB_INCLUDE Optional Newlib include directory.
#   RON_ARM_CLANG_ALLOW_HEADER_SHIM Allow declaration-only local fallback.
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
set(RON_ARM_CLANG_NEWLIB_INCLUDE
    ""
    CACHE PATH "Optional Newlib include directory for ARM bare-metal headers")
set(RON_ARM_CLANG_ALLOW_HEADER_SHIM
    ON
    CACHE BOOL "Allow declaration-only header fallback when Newlib is unavailable")
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

set(RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES "")
if(NOT "${RON_ARM_CLANG_NEWLIB_INCLUDE}" STREQUAL "")
    list(APPEND RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES "${RON_ARM_CLANG_NEWLIB_INCLUDE}")
endif()
if(NOT "${RON_ARM_CLANG_SYSROOT}" STREQUAL "")
    list(APPEND RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES
         "${RON_ARM_CLANG_SYSROOT}/include"
         "${RON_ARM_CLANG_SYSROOT}/arm-none-eabi/include")
endif()
if(WIN32)
    file(GLOB RON_ARM_CLANG_WINDOWS_NEWLIB_INCLUDES
         LIST_DIRECTORIES true
         "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/arm-none-eabi/include"
         "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/arm-none-eabi/include"
         "C:/Program Files/GNU Arm Embedded Toolchain/*/arm-none-eabi/include"
         "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/arm-none-eabi/include"
         "$ENV{USERPROFILE}/AppData/Roaming/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/*/.content/arm-none-eabi/include"
         "$ENV{USERPROFILE}/scoop/apps/gcc-arm-none-eabi/current/arm-none-eabi/include"
         "C:/ProgramData/chocolatey/lib/gcc-arm-embedded/tools/*/arm-none-eabi/include")
    list(APPEND RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES
         ${RON_ARM_CLANG_WINDOWS_NEWLIB_INCLUDES})
else()
    list(APPEND RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES
         "/usr/include/newlib"
         "/usr/arm-none-eabi/include"
         "/usr/lib/arm-none-eabi/include")
endif()

set(RON_ARM_CLANG_RESOLVED_NEWLIB_INCLUDE "")
foreach(RON_ARM_CLANG_INCLUDE_CANDIDATE IN LISTS RON_ARM_CLANG_NEWLIB_INCLUDE_CANDIDATES)
    if((NOT "${RON_ARM_CLANG_INCLUDE_CANDIDATE}" STREQUAL "") AND
       EXISTS "${RON_ARM_CLANG_INCLUDE_CANDIDATE}/math.h")
        set(RON_ARM_CLANG_RESOLVED_NEWLIB_INCLUDE "${RON_ARM_CLANG_INCLUDE_CANDIDATE}")
        break()
    endif()
endforeach()

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
if(NOT "${RON_ARM_CLANG_RESOLVED_NEWLIB_INCLUDE}" STREQUAL "")
    message(STATUS "Using ARMv7 Clang Newlib headers: ${RON_ARM_CLANG_RESOLVED_NEWLIB_INCLUDE}")
    string(APPEND RON_ARM_CLANG_INITIAL_FLAGS " -isystem \"${RON_ARM_CLANG_RESOLVED_NEWLIB_INCLUDE}\"")
elseif(RON_ARM_CLANG_ALLOW_HEADER_SHIM)
    message(WARNING
        "ARMv7 Clang Newlib headers were not found; using declaration-only "
        "freestanding header fallback for static-library smoke builds. "
        "Install Newlib and set RON_ARM_CLANG_NEWLIB_INCLUDE for target-library evidence.")
    set(RON_ARM_CLANG_FREESTANDING_INCLUDE
        "${CMAKE_CURRENT_LIST_DIR}/../freestanding/armv7-none-eabi/include")
    string(APPEND RON_ARM_CLANG_INITIAL_FLAGS " -isystem \"${RON_ARM_CLANG_FREESTANDING_INCLUDE}\"")
else()
    message(FATAL_ERROR
        "ARMv7 Clang Newlib headers were not found. Set "
        "RON_ARM_CLANG_NEWLIB_INCLUDE to the directory containing math.h, "
        "or enable RON_ARM_CLANG_ALLOW_HEADER_SHIM for an object-only smoke build.")
endif()

set(CMAKE_C_FLAGS_INIT "${RON_ARM_CLANG_INITIAL_FLAGS}")

# Prevent CMake from searching host system paths for target libraries/includes.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

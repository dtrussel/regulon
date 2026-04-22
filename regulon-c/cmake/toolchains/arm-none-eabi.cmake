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

set(RON_ARM_GCC_NEWLIB_INCLUDE
    ""
    CACHE PATH "Optional Newlib include directory for ARM bare-metal headers")
set(RON_ARM_GCC_ALLOW_HEADER_SHIM
    ON
    CACHE BOOL "Allow declaration-only header fallback when Newlib is unavailable")

set(RON_ARM_GCC_NEWLIB_INCLUDE_CANDIDATES "")
if(NOT "${RON_ARM_GCC_NEWLIB_INCLUDE}" STREQUAL "")
    list(APPEND RON_ARM_GCC_NEWLIB_INCLUDE_CANDIDATES "${RON_ARM_GCC_NEWLIB_INCLUDE}")
endif()
if(WIN32)
    file(GLOB RON_ARM_GCC_WINDOWS_NEWLIB_INCLUDES
         LIST_DIRECTORIES true
         "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/arm-none-eabi/include"
         "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/arm-none-eabi/include"
         "C:/Program Files/GNU Arm Embedded Toolchain/*/arm-none-eabi/include"
         "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/arm-none-eabi/include"
         "$ENV{USERPROFILE}/AppData/Roaming/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/*/.content/arm-none-eabi/include"
         "$ENV{USERPROFILE}/scoop/apps/gcc-arm-none-eabi/current/arm-none-eabi/include"
         "C:/ProgramData/chocolatey/lib/gcc-arm-embedded/tools/*/arm-none-eabi/include")
    list(APPEND RON_ARM_GCC_NEWLIB_INCLUDE_CANDIDATES
         ${RON_ARM_GCC_WINDOWS_NEWLIB_INCLUDES})
else()
    list(APPEND RON_ARM_GCC_NEWLIB_INCLUDE_CANDIDATES
         "/usr/include/newlib"
         "/usr/arm-none-eabi/include"
         "/usr/lib/arm-none-eabi/include")
endif()

set(RON_ARM_GCC_RESOLVED_NEWLIB_INCLUDE "")
foreach(RON_ARM_GCC_INCLUDE_CANDIDATE IN LISTS RON_ARM_GCC_NEWLIB_INCLUDE_CANDIDATES)
    if((NOT "${RON_ARM_GCC_INCLUDE_CANDIDATE}" STREQUAL "") AND
       EXISTS "${RON_ARM_GCC_INCLUDE_CANDIDATE}/math.h")
        set(RON_ARM_GCC_RESOLVED_NEWLIB_INCLUDE "${RON_ARM_GCC_INCLUDE_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT "${RON_ARM_GCC_RESOLVED_NEWLIB_INCLUDE}" STREQUAL "")
    message(STATUS "Using ARM GCC Newlib headers: ${RON_ARM_GCC_RESOLVED_NEWLIB_INCLUDE}")
    list(APPEND CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
         "${RON_ARM_GCC_RESOLVED_NEWLIB_INCLUDE}")
elseif(RON_ARM_GCC_ALLOW_HEADER_SHIM)
    message(WARNING
        "ARM GCC Newlib headers were not found; using declaration-only "
        "freestanding header fallback for static-library smoke builds. "
        "Install Newlib and set RON_ARM_GCC_NEWLIB_INCLUDE for target-library evidence.")
    set(RON_ARM_GCC_FREESTANDING_INCLUDE
        "${CMAKE_CURRENT_LIST_DIR}/../freestanding/armv7-none-eabi/include")
    list(APPEND CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
         "${RON_ARM_GCC_FREESTANDING_INCLUDE}")
else()
    message(FATAL_ERROR
        "ARM GCC Newlib headers were not found. Set "
        "RON_ARM_GCC_NEWLIB_INCLUDE to the directory containing math.h, "
        "or enable RON_ARM_GCC_ALLOW_HEADER_SHIM for an object-only smoke build.")
endif()
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
    "${CMAKE_C_STANDARD_INCLUDE_DIRECTORIES}"
    CACHE STRING "ARM GCC target C standard include directories" FORCE)

# Prevent CMake from searching host system paths for libraries/includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

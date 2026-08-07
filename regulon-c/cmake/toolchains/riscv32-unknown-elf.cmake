# =============================================================================
# riscv32-unknown-elf.cmake — Cross-compile toolchain for RISC-V 32-bit targets
#
# Usage:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/riscv32-unknown-elf.cmake \
#         -DCMAKE_C_FLAGS="-march=rv32imc -mabi=ilp32" \
#         -B build_rv32 -S regulon-c/
#
# The Ubuntu/Debian `gcc-riscv64-unknown-elf` package installs a multilib
# compiler binary named `riscv64-unknown-elf-gcc` that targets rv32 (or rv64)
# depending on the `-march`/`-mabi` flags passed at compile time; there is no
# separate `riscv32-unknown-elf-gcc` binary to look for.
#
# RON-IS-001 §8.1 | RON-DC-005
# SPDX-License-Identifier: MIT
# =============================================================================

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  riscv)

set(CMAKE_C_COMPILER        riscv64-unknown-elf-gcc)
set(CMAKE_AR                riscv64-unknown-elf-ar)
set(CMAKE_RANLIB            riscv64-unknown-elf-ranlib)
set(CMAKE_SIZE              riscv64-unknown-elf-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXE_LINKER_FLAGS_INIT   "-nostartfiles -nostdlib")

set(RON_RISCV_GCC_LIBC_INCLUDE
    ""
    CACHE PATH "Optional RISC-V bare-metal libc include directory (Newlib/picolibc math.h)")
set(RON_RISCV_GCC_ALLOW_HEADER_SHIM
    ON
    CACHE BOOL "Allow declaration-only header fallback when no libc headers are available")

set(RON_RISCV_GCC_LIBC_INCLUDE_CANDIDATES "")
if(NOT "${RON_RISCV_GCC_LIBC_INCLUDE}" STREQUAL "")
    list(APPEND RON_RISCV_GCC_LIBC_INCLUDE_CANDIDATES "${RON_RISCV_GCC_LIBC_INCLUDE}")
endif()
if(WIN32)
    file(GLOB RON_RISCV_GCC_WINDOWS_LIBC_INCLUDES
         LIST_DIRECTORIES true
         "$ENV{USERPROFILE}/AppData/Roaming/xPacks/@xpack-dev-tools/riscv-none-elf-gcc/*/.content/riscv-none-elf/include")
    list(APPEND RON_RISCV_GCC_LIBC_INCLUDE_CANDIDATES
         ${RON_RISCV_GCC_WINDOWS_LIBC_INCLUDES})
else()
    list(APPEND RON_RISCV_GCC_LIBC_INCLUDE_CANDIDATES
         "/usr/lib/picolibc/riscv64-unknown-elf/include"
         "/usr/riscv64-unknown-elf/include"
         "/usr/riscv32-unknown-elf/include"
         "/usr/lib/riscv64-unknown-elf/include")
endif()

set(RON_RISCV_GCC_RESOLVED_LIBC_INCLUDE "")
foreach(RON_RISCV_GCC_INCLUDE_CANDIDATE IN LISTS RON_RISCV_GCC_LIBC_INCLUDE_CANDIDATES)
    if((NOT "${RON_RISCV_GCC_INCLUDE_CANDIDATE}" STREQUAL "") AND
       EXISTS "${RON_RISCV_GCC_INCLUDE_CANDIDATE}/math.h")
        set(RON_RISCV_GCC_RESOLVED_LIBC_INCLUDE "${RON_RISCV_GCC_INCLUDE_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT "${RON_RISCV_GCC_RESOLVED_LIBC_INCLUDE}" STREQUAL "")
    message(STATUS "Using RISC-V libc headers: ${RON_RISCV_GCC_RESOLVED_LIBC_INCLUDE}")
    list(APPEND CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
         "${RON_RISCV_GCC_RESOLVED_LIBC_INCLUDE}")
elseif(RON_RISCV_GCC_ALLOW_HEADER_SHIM)
    message(WARNING
        "RISC-V bare-metal libc headers were not found; using declaration-only "
        "freestanding header fallback for static-library smoke builds. Install "
        "picolibc/Newlib for this target and set RON_RISCV_GCC_LIBC_INCLUDE for "
        "target-library evidence.")
    set(RON_RISCV_GCC_FREESTANDING_INCLUDE
        "${CMAKE_CURRENT_LIST_DIR}/../freestanding/riscv32-unknown-elf/include")
    list(APPEND CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
         "${RON_RISCV_GCC_FREESTANDING_INCLUDE}")
else()
    message(FATAL_ERROR
        "RISC-V bare-metal libc headers were not found. Set "
        "RON_RISCV_GCC_LIBC_INCLUDE to the directory containing math.h, or "
        "enable RON_RISCV_GCC_ALLOW_HEADER_SHIM for an object-only smoke build.")
endif()
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
    "${CMAKE_C_STANDARD_INCLUDE_DIRECTORIES}"
    CACHE STRING "RISC-V GCC target C standard include directories" FORCE)

# Prevent CMake from searching host system paths for libraries/includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

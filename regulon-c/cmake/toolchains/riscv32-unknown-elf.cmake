# =============================================================================
# riscv32-unknown-elf.cmake — Cross-compile toolchain for RISC-V 32-bit targets
#
# Usage:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/riscv32-unknown-elf.cmake \
#         -DCMAKE_C_FLAGS="-march=rv32imc -mabi=ilp32" \
#         -B build_rv32 -S regulon-c/
#
# RON-IS-001 §8.1 | RON-DC-005
# SPDX-License-Identifier: MIT
# =============================================================================

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  riscv)

set(CMAKE_C_COMPILER        riscv32-unknown-elf-gcc)
set(CMAKE_AR                riscv32-unknown-elf-ar)
set(CMAKE_RANLIB             riscv32-unknown-elf-ranlib)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXE_LINKER_FLAGS_INIT   "-nostartfiles -nostdlib")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

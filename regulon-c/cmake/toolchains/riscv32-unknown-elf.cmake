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

# The library is freestanding (RON-DC-002): the only headers it includes are
# <stdint.h>, <stdbool.h>, <float.h> and <stddef.h>, all of which GCC provides
# itself. No libc, no sysroot, and no -lm are needed to build it, so there is
# nothing here to locate or configure.
set(CMAKE_C_FLAGS_INIT "-ffreestanding")

# Prevent CMake from searching host system paths for libraries/includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

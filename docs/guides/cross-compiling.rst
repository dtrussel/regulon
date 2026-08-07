Cross-Compiling
===============

Regulon is built for embedded targets, and its cross-compiled builds are
exercised on every push rather than assumed to work. Toolchain files live in
``regulon-c/cmake/toolchains/``.

Tests never cross-compile: they need a host to run on, so
``RON_BUILD_TESTS`` is ignored when ``CMAKE_CROSSCOMPILING`` is set. A cross
build is a compile-and-link check that the library is clean for the target.

ARM Cortex-M
------------

.. code-block:: bash

   cmake -B build_arm -S regulon-c \
         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
         -DCMAKE_C_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb" \
         -DRON_BUILD_TESTS=OFF
   cmake --build build_arm

The toolchain file selects ``arm-none-eabi-gcc``; a Clang variant is
provided as ``armv7-none-eabi-clang.cmake``. CPU, FPU and ABI flags are
yours to set, since they follow from the part rather than the library.

On a Cortex-M4F or M7 with a single-precision FPU, leave ``RON_USE_DOUBLE``
off. Every ``double`` operation would otherwise be emulated in software,
which costs both cycles and flash.

RISC-V
------

.. code-block:: bash

   cmake -B build_riscv -S regulon-c \
         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/riscv32-unknown-elf.cmake \
         -DRON_BUILD_TESTS=OFF
   cmake --build build_riscv

The toolchain targets ``rv32imc``/``ilp32`` using the ``riscv64-unknown-elf``
GCC that Debian and Ubuntu package, which supports 32-bit targets through
``-march``/``-mabi``.

Freestanding targets without a full libc
----------------------------------------

Regulon needs very little from the C library — chiefly ``<math.h>``
declarations for a handful of functions. On a bare toolchain with no libc
headers at all, both toolchain files can fall back to a declaration-only
shim under ``regulon-c/cmake/freestanding/``:

.. code-block:: bash

   cmake -B build_arm -S regulon-c \
         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
         -DRON_ARM_GCC_ALLOW_HEADER_SHIM=ON \
         -DRON_BUILD_TESTS=OFF

The RISC-V equivalent is ``RON_RISCV_GCC_ALLOW_HEADER_SHIM``. Both toolchains
also accept an explicit libc include directory
(``RON_ARM_GCC_LIBC_INCLUDE`` / ``RON_RISCV_GCC_LIBC_INCLUDE``) when your
sysroot is somewhere the toolchain file does not look.

The shim only declares functions; it does not implement them. You still need
to link an implementation, whether that is newlib, picolibc, or your own.

Writing a toolchain file for another target
-------------------------------------------

Nothing in the library is architecture-specific, so a new target usually
needs only a conventional CMake toolchain file:

.. code-block:: cmake

   set(CMAKE_SYSTEM_NAME      Generic)
   set(CMAKE_SYSTEM_PROCESSOR my_arch)

   set(CMAKE_C_COMPILER my-arch-gcc)
   set(CMAKE_AR         my-arch-ar)
   set(CMAKE_RANLIB     my-arch-ranlib)

   # The library is freestanding; skip the compiler's link test, which
   # needs a full runtime that a bare cross toolchain may not have.
   set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

The two shipped toolchain files are worth reading as worked examples,
particularly for how they locate libc headers.

Integer and floating-point requirements
---------------------------------------

The library assumes ``<stdint.h>``, ``<stdbool.h>`` and IEEE-754 arithmetic
in the selected precision. It does not require an FPU — soft-float targets
work — but a control loop without hardware floating point will be
substantially slower, and the timing figures in :doc:`verification` will not
transfer.

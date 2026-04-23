# AGENTS.md - Regulon C11 Implementation

This file applies to `regulon-c/`. Repository-wide requirements and traceability rules remain in `../AGENTS.md`.

## Layout
```
include/ron/       <- public C headers (ron_*.h)
src/               <- C sources (ron_*.c)
test/unit/         <- Unity tests (test_ron_*.c)
test/formal/       <- CBMC harnesses (*_proof.c)
```

## Active Scope
- Keep the active C11 implementation PID-only until the PID slice is accepted closed.
- Do not add public headers or default-build sources for future modules unless the active iteration explicitly asks for them.
- Active public PID surface is `include/ron/ron_platform.h`, `include/ron/ron_pid_types.h`, and `include/ron/ron_pid.h`.

## C11 Rules
- **File header** (mandatory on every `.c`/`.h`): `@file`, `@brief`, `@doc`, `@req`, `SPDX-License-Identifier: MIT`.
- **Production C headers/sources** use `@doc RON-IS-001`; formal harnesses and test sources may use `@doc RON-TP-001`.
- **Every function** must have `/* Satisfies: RON-FR-xxx | Test: RON-TC-xxx-NNN */` above it.
- **Naming**: public `ron_<module>_<verb>`, internal `static <module>_<verb>`, types `ron_<noun>_t`, macros `RON_<SCREAMING>`.
- **Permitted production headers**: `<stdint.h>`, `<stdbool.h>`, `<float.h>`, `<stddef.h>`. `<math.h>` only in coefficient helpers.
- **Error pattern**: null-check -> init-check -> fault-latch -> input validation -> computation (in that order, every public function).
- **Coding standard**: MISRA C:2023. Run `cppcheck --addon=misra.py --error-exitcode=1 regulon-c/src/` from the repository root.

## C Test IDs
C unit function name: `test_ron_tc_pid_015`.
C formal function name: `<harness_basename>` for `regulon-c/test/formal/<harness_basename>.c`.

## Build
```bash
# Host tests with sanitizers
cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON -DCMAKE_C_FLAGS="-fsanitize=address,undefined"
cmake --build regulon-c/build
ctest --test-dir regulon-c/build --output-on-failure --output-junit results.xml

# Windows/MSVC multi-config generators
cmake -B regulon-c/build_msvc -S regulon-c -DRON_BUILD_TESTS=ON
cmake --build regulon-c/build_msvc --config Debug
ctest --test-dir regulon-c/build_msvc -C Debug --output-on-failure

# Cross-compile C (ARM example)
cmake -B regulon-c/build/arm -S regulon-c -DCMAKE_TOOLCHAIN_FILE=regulon-c/cmake/toolchains/arm-none-eabi.cmake
```

## Analysis & Coverage
```bash
# MISRA, complexity, formatting for the active PID slice
cppcheck --addon=misra.py --error-exitcode=1 \
  -I regulon-c/include \
  regulon-c/src/ron_pid_api.c \
  regulon-c/src/ron_pid_config.c \
  regulon-c/src/ron_pid_core.c \
  regulon-c/src/ron_pid_fault.c

lizard -C 10 \
  regulon-c/src/ron_pid_api.c \
  regulon-c/src/ron_pid_config.c \
  regulon-c/src/ron_pid_core.c \
  regulon-c/src/ron_pid_fault.c

clang-format --dry-run --Werror \
  regulon-c/src/ron_pid_api.c \
  regulon-c/src/ron_pid_config.c \
  regulon-c/src/ron_pid_core.c \
  regulon-c/src/ron_pid_fault.c \
  regulon-c/src/ron_pid_internal.h \
  regulon-c/include/ron/ron_platform.h \
  regulon-c/include/ron/ron_pid_types.h \
  regulon-c/include/ron/ron_pid.h

# Coverage must reach 100% statement + branch for the active PID slice.
# Use branch coverage explicitly and tolerate intentionally unused defensive filters.
cmake -B regulon-c/build_cov -S regulon-c -DRON_BUILD_TESTS=ON \
  -DCMAKE_C_COMPILER=gcc -DCMAKE_C_FLAGS="-O0 -g --coverage"
cmake --build regulon-c/build_cov --parallel
ctest --test-dir regulon-c/build_cov --output-on-failure
lcov --rc branch_coverage=1 --capture --directory regulon-c/build_cov --output-file coverage.info
lcov --rc branch_coverage=1 --remove coverage.info '/usr/*' '*/test/*' '*/unity/*' \
  --ignore-errors unused --output-file coverage_filtered.info
lcov --rc branch_coverage=1 --summary coverage_filtered.info
genhtml coverage_filtered.info --branch-coverage --output-directory coverage_html
```

## Formal Verification
```bash
# Each harness basename must match its proof function name.
for harness in regulon-c/test/formal/*_proof.c; do
  entry=$(basename "$harness" .c)
  cbmc --function "$entry" --unwind 65 --unwinding-assertions \
    --bounds-check --pointer-check \
    "$harness" \
    regulon-c/src/ron_pid_api.c \
    regulon-c/src/ron_pid_config.c \
    regulon-c/src/ron_pid_core.c \
    regulon-c/src/ron_pid_fault.c \
    -I regulon-c/include
done
```

## Formal Proof Guidance
- Normal-operation proofs must constrain nondeterministic inputs to the SRS operating assumptions, especially `RON-ASM-02` and `RON-ASM-03`: bounded sample periods and bounded finite process/setpoint values.
- Do not claim "all finite inputs" when arithmetic can overflow before saturation. Unbounded finite overflow belongs in fault-detection proofs, where `RON-SR-010` expects a latched fault and safe output.
- Keep proof harnesses loop bounds compatible with the CI unwind limit (`--unwind 65`) and use `--unwinding-assertions` for soundness.
- If a proof uses a bounded environment assumption, record the bound in `docs/specs/TP_ControlLib.rst`.

## C-Specific Prohibitions
`malloc/free` | recursion | VLAs | `goto/setjmp` | global mutable state | magic numbers | `int`/`long` without width | unbounded loops | implicit numeric casts

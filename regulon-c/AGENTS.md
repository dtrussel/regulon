# AGENTS.md - Regulon C11 Implementation

This file applies to `regulon-c/`. Repository-wide requirements and traceability rules remain in `../AGENTS.md`.

## Layout
```
include/ron/       <- public C headers (ron_*.h)
src/               <- C sources (ron_*.c) and internal headers (*_internal.h)
test/unit/         <- Unity tests (test_ron_*.c)
test/integration/  <- cross-module tests through ron/ron.h
test/formal/       <- CBMC harnesses (*_proof.c)
examples/, bench/  <- host example programs and the timing benchmark
scripts/           <- source manifests and CI helper scripts
```

## Source Manifests
- `scripts/lib_sources.txt` lists every production `.c` file; `scripts/format_files.txt` every file under `clang-format`.
- The cppcheck, lizard, coverage and CBMC gates read these lists, so a new source or header **must** be registered there. `bash regulon-c/scripts/check_manifest.sh` fails on drift.
- A new optional module also needs a `RON_ENABLE_<MODULE>` option (`cmake/ron_options.cmake`), a `RON_HAVE_<MODULE>` entry (`cmake/ron_modules.h.in`), and the matching `CONFIG_REGULON_<MODULE>` in `../zephyr/Kconfig` with its source added to `../zephyr/CMakeLists.txt`.

## C11 Rules
- **File header** (mandatory on every `.c`/`.h`): `@file`, `@brief`, `@doc`, `@req`, `SPDX-License-Identifier: MIT`.
- **Production C headers/sources** use `@doc RON-IS-001`; formal harnesses and test sources may use `@doc RON-TP-001`.
- **Every function** must have `/* Satisfies: RON-FR-xxx | Test: RON-TC-xxx-NNN */` above it.
- **Naming**: public `ron_<module>_<verb>`, internal `static <module>_<verb>`, types `ron_<noun>_t`, macros `RON_<SCREAMING>`.
- **Permitted production headers**: `<stdint.h>`, `<stdbool.h>`, `<float.h>`, `<stddef.h>` — all freestanding, so the library builds with no libc at all. `<math.h>` is **not** permitted (RON-DC-002): the biquad coefficient helpers use an internal bounded sin/cos, and nothing else needs it.
- **Error pattern**: null-check -> init-check -> fault-latch -> input validation -> computation (in that order, every public function).
- **Coding standard**: MISRA C:2023, enforced by the cppcheck gate below; every suppression must have a record in `docs/deviations/MISRA_C_deviations.rst`.

## C Test IDs
C unit function name: `test_ron_tc_pid_015`.
C formal function name: `<harness_basename>` for `regulon-c/test/formal/<harness_basename>.c`.

## Gates
`.github/workflows/ci_c.yml` is authoritative; the commands below mirror it. On Windows, `scripts/verify.ps1` runs the same gates. Run from the repository root.

```bash
# File lists shared by every gate
mapfile -t SRC < <(grep -vE '^[[:space:]]*(#.*)?$' regulon-c/scripts/lib_sources.txt)
mapfile -t FMT < <(grep -vE '^[[:space:]]*(#.*)?$' regulon-c/scripts/format_files.txt)

# Host tests with sanitizers (also run with -DRON_USE_DOUBLE=ON and with clang)
cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON \
      -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all"
cmake --build regulon-c/build --parallel
ctest --test-dir regulon-c/build --output-on-failure

# Format, complexity, manifest drift, stack budget
clang-format --dry-run --Werror "${FMT[@]}"
python3 -m lizard -C 10 "${SRC[@]}"
bash regulon-c/scripts/check_manifest.sh
python3 regulon-c/scripts/check_traceability.py
bash regulon-c/scripts/check_stack_usage.sh <gcc-build-dir> 768

# MISRA C:2023 (suppressions are the deviations in docs/deviations/)
cppcheck --addon=misra.py --check-level=exhaustive --enable=style --error-exitcode=1 \
  --suppress=missingIncludeSystem --suppress=misra-c2012-2.3 --suppress=misra-c2012-2.4 \
  --suppress=misra-c2012-2.5 --suppress=misra-c2012-8.7 --suppress=misra-c2012-15.5 \
  --suppress=misra-c2012-15.7 --suppress=misra-c2012-20.9 --suppress=misra-c2012-20.10 \
  -I regulon-c/include -I regulon-c/src "${SRC[@]}"

# CBMC: each harness basename is its entry function
for harness in regulon-c/test/formal/*_proof.c; do
  cbmc --function "$(basename "$harness" .c)" --unwind 65 --unwinding-assertions \
       --bounds-check --pointer-check "$harness" "${SRC[@]}" \
       -I regulon-c/include -I regulon-c/src -I regulon-c/test/formal
done
```

Coverage must be 100% statement and branch over `lib_sources.txt`, measured with LLVM source-based coverage (`-fprofile-instr-generate -fcoverage-mapping`, then `llvm-profdata merge` and `llvm-cov export -summary-only`); see the `coverage` job in `ci_c.yml` for the exact invocation.

Cross builds (`cmake/toolchains/`) must also pass `scripts/check_no_libm.sh <nm> libregulon.a`.

## Formal Proof Guidance
- Normal-operation proofs must constrain nondeterministic inputs to the SRS operating assumptions, especially `RON-ASM-02` and `RON-ASM-03`: bounded sample periods and bounded finite process/setpoint values.
- Do not claim "all finite inputs" when arithmetic can overflow before saturation. Unbounded finite overflow belongs in fault-detection proofs, where `RON-SR-010` expects a latched fault and safe output.
- Keep proof harnesses loop bounds compatible with the CI unwind limit (`--unwind 65`) and use `--unwinding-assertions` for soundness.
- If a proof uses a bounded environment assumption, record the bound in `docs/specs/TP_ControlLib.rst`.

## C-Specific Prohibitions
`malloc/free` | recursion | VLAs | `goto/setjmp` | global mutable state | magic numbers | `int`/`long` without width | unbounded loops | implicit numeric casts

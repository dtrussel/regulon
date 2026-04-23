# C11 PID Rollout Status

Date: 2026-04-22

## Scope

`regulon-c` is currently narrowed to the PID deliverable only. The active public surface is:

- `regulon-c/include/ron/ron_platform.h`
- `regulon-c/include/ron/ron_pid_types.h`
- `regulon-c/include/ron/ron_pid.h`

All non-PID modules remain present in the repository but stay out of the active CMake build until the PID slice is closed with traceability and quality evidence.

## Implemented In This Kickoff

- PID public API implemented in `ron_pid_api.c`, `ron_pid_core.c`, `ron_pid_fault.c`, and `ron_pid_internal.h`.
- Configuration validation expanded for runtime updates, enum validation, normalisation ranges, and threshold checks.
- Build narrowed to the PID slice.
- Windows/MSVC host test build fixed by making test math-library linkage conditional.
- Local verification entrypoint added at `regulon-c/scripts/verify_pid.ps1` for tool probing, PID-only host builds, formatting, static analysis, and optional cross/formal steps.
- Traceable Unity suites added for:
  - `RON-TC-PID-001` to `RON-TC-PID-014`
  - `RON-TC-PID-015` to `RON-TC-PID-039`
  - host safety checks for `RON-TC-SAFE-001`, `RON-TC-SAFE-006`, `RON-TC-SAFE-007`, `RON-TC-SAFE-008`, `RON-TC-SAFE-009`, `RON-TC-SAFE-010`, and `RON-TC-SAFE-011`
  - deterministic reproducibility check `RON-TC-QUAL-017`
- Initial CBMC harness set added for:
  - `RON-TC-PID-015-FV`
  - `RON-TC-PID-022-FV`
  - `RON-TC-PID-026-FV`
  - `RON-TC-PID-036-FV`
  - `RON-TC-SAFE-006-FV`

## Implemented In Verification Closure

- `regulon-c/scripts/verify_pid.ps1` now includes an LLVM `coverage` step, dynamic CBMC harness discovery, `--unwinding-assertions`, and clean skip reporting for missing coverage, cross, formal, and Clang toolchain components.
- The Clang verification step now uses standalone Clang with Ninja when available and only uses Visual Studio `ClangCL` when the VS Platform Toolset is installed.
- C formal harness coverage was expanded for:
  - `RON-TC-SAFE-001-FV`
  - `RON-TC-SAFE-002-FV`
  - `RON-TC-SAFE-003-FV`
  - `RON-TC-SAFE-004-FV`
  - `RON-TC-SAFE-005-FV`
  - `RON-TC-SAFE-007-FV`
  - `RON-TC-SAFE-011-FV`
  - `RON-TC-SAFE-013-FV`
  - `RON-TC-PERF-002-FV`
  - `RON-TC-PERF-004-FV`
  - `RON-TC-QUAL-016-FV`
- `.github/workflows/ci_c.yml` now enforces 100% statement and branch coverage with Clang/LLVM coverage, uploads raw and rendered coverage artifacts, runs the ARM Cortex-M cross-compile smoke build, and uploads CBMC proof logs.

## Local Evidence

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
- `cmake --build regulon-c/build --config Debug`: passes
- `ctest --test-dir regulon-c/build --output-on-failure -C Debug`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps probe,format,complexity,cppcheck`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps msvc,double`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps probe,clang`: passes with a clean `clang` skip because this host has standalone LLVM and `clang-cl`, but no Ninja or Visual Studio `ClangCL` Platform Toolset.
- `regulon-c/scripts/verify_pid.ps1 -Steps coverage,cross-arm,cbmc`: reports the current local tool gaps without execution failure.
- `clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over all `regulon-c/test/formal/*_proof.c`: passes.

## Remaining Gaps

- Phase 0 PID closure is accepted for opening the first non-PID module. Local
  MSVC, double-precision, standalone Clang, format, complexity, cppcheck/MISRA,
  and LLVM coverage gates now run through the repo script for the active source
  set.
- Local LLVM coverage now reaches 100% statement and branch coverage for the
  active C11 source set.
- Local Clang host evidence is available through standalone Clang plus Ninja.
- No local embedded cross-compile build has been rerun because
  `arm-none-eabi-gcc` is not installed on this machine; CI contains the ARM
  Cortex-M cross-compile smoke build.
- No local CBMC execution evidence has been produced yet because `cbmc` is not
  installed on this machine; the full harness inventory is discovered
  dynamically by the local script and CI.
- The RAM budget checks in `ron_pid_types.h` and `test_ron_pid_types.c` are now explicitly scoped to the single-precision configuration, matching `RON-PR-021` as written in the SRS; the double-precision regression build now executes successfully.
- The anti-windup host test has been narrowed to an implementation-level open-loop recovery contrast. Closed-loop settling-time benefit still requires plant-coupled verification beyond the current host unit slice.

## Phase 1 Filter Opening Evidence

- `ron_filter.h` and `ron_filter.c` are now active in the default C11 build.
- `test_ron_filter.c` covers `RON-TC-FILT-001` through `RON-TC-FILT-017`.
- Filter-focused CBMC harnesses were added for null-pointer, bounded execution,
  bounded array access, and no-heap evidence; local CBMC remains unavailable on
  this host.
- Local evidence after enabling filters:
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`: passes with 100% statement and branch coverage

## Phase 2 Feed-Forward Opening Evidence

- `ron_feedforward.h` and `ron_feedforward.c` are now active in the default
  C11 build.
- `test_ron_feedforward.c` covers `RON-TC-FF-001` through `RON-TC-FF-009`,
  including static, velocity, acceleration, external, disabled-path,
  output-limit, and diagnostic scenarios.
- The existing `ron_pid_step()` signature is preserved. External
  feed-forward input is exposed through `ron_pid_step_feedforward()`.
- Local evidence after enabling feed-forward:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
  - `cmake --build regulon-c/build --config Debug`: passes
  - `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`: passes with 100% statement and branch coverage
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`: passes with local `cross-arm-clang` evidence, `cross-arm` skipped because `arm-none-eabi-gcc` is unavailable, and `cbmc` skipped because CBMC is unavailable

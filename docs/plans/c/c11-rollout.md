# C11 PID Rollout Status

Date: 2026-04-12

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

## Local Evidence

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
- `cmake --build regulon-c/build --config Debug`: passes
- `ctest --test-dir regulon-c/build --output-on-failure -C Debug`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps probe,format,complexity,cppcheck`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps msvc,double`: passes
- `regulon-c/scripts/verify_pid.ps1 -Steps probe,clang,cross-arm,cbmc`: reports the current local tool gaps without execution failure

## Remaining Gaps

- No local coverage report has been generated yet, so the `RON-QR-022` target is still unproven.
- The local Windows environment now has `cppcheck`, `clang-format`, Python, and `lizard`, but still lacks `clang-cl`, `arm-none-eabi-gcc`, and `cbmc` on `PATH`; those remain explicit environment gaps for full closure.
- No embedded cross-compile build has been rerun against the narrowed PID-only target yet because the local ARM toolchain is not installed.
- No local CBMC execution evidence has been produced yet because `cbmc` is not installed on this machine.
- The RAM budget checks in `ron_pid_types.h` and `test_ron_pid_types.c` are now explicitly scoped to the single-precision configuration, matching `RON-PR-021` as written in the SRS; the double-precision regression build now executes successfully.
- The anti-windup host test has been narrowed to an implementation-level open-loop recovery contrast. Closed-loop settling-time benefit still requires plant-coupled verification beyond the current host unit slice.

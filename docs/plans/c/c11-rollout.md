# C11 PID Rollout Status

Date: 2026-04-23

## Scope

`regulon-c` now includes the accepted PID baseline plus the completed Phase 1,
Phase 2, Phase 3, and Phase 4 slices. The active public surface is:

- `regulon-c/include/ron/ron_platform.h`
- `regulon-c/include/ron/ron_pid_types.h`
- `regulon-c/include/ron/ron_pid.h`
- `regulon-c/include/ron/ron_filter.h`
- `regulon-c/include/ron/ron_feedforward.h`
- `regulon-c/include/ron/ron_gain_sched.h`
- `regulon-c/include/ron/ron_trajectory.h`

Later roadmap modules remain present in the repository but stay out of the
active CMake build until their phase is explicitly opened and closed with the
same traceability and quality evidence bar.

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
- Local ARM and ARMv7 Clang cross-compile smoke builds now run through the
  verification script. This host still lacks Newlib target headers, so both
  cross-builds use the existing declaration-only freestanding header fallback.
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

## Phase 3 Gain Scheduling Opening Evidence

- `ron_gain_sched.h` and `ron_gain_sched.c` are now active in the default C11
  build.
- `ron_pid_set_config()` is now part of the public PID runtime API and is used
  as the atomic full-config update path for gain scheduling and the existing
  PID runtime setters.
- `test_ron_gain_sched.c` covers `RON-TC-GS-001` through `RON-TC-GS-008`,
  including hard-switch selection, interpolation, atomic no-change-on-invalid
  update, next-step behaviour, integral reset policy, and table validation.
- `test_ron_pid_api.c` now directly covers the null, uninitialised,
  successful-update, and invalid-candidate paths for `ron_pid_set_config()`
  under `RON-TC-PID-033`.
- Gain scheduling uses the conservative interpolation rule documented in the
  Phase 3 plan: interpolate `Kp`, `Ki`, and `Kd` only, and reject interpolation
  tables that vary other `ron_pid_config_t` fields across adjacent entries.
- Local evidence after enabling gain scheduling:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
  - `cmake --build regulon-c/build --config Debug`: passes
  - `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`: passes with 100% statement and branch coverage
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`: passes with local `cross-arm-clang` evidence, `cross-arm` skipped because `arm-none-eabi-gcc` is unavailable, and `cbmc` skipped because CBMC is unavailable

## Phase 4 Trajectory Generators Opening Evidence

- `ron_trajectory.h`, `ron_trajectory_trap.c`, and
  `ron_trajectory_scurve.c` are now active in the default C11 build.
- `test_ron_trajectory.c` covers `RON-TC-TRAJ-001` through
  `RON-TC-TRAJ-008`, including trapezoidal convergence, short moves,
  reverse motion, velocity-continuous retargeting, hold/resume, defensive
  validation, S-curve seven-phase planning, zero-duration phases, bounded
  velocity/acceleration/jerk, and completion snapping.
- `docs/specs/IS_ControlLib.rst` now records explicit trajectory state fields
  for target tracking, direction/timing, completion, hold, fault latching, and
  initialisation guards.
- `docs/specs/TP_ControlLib.rst` now includes detailed descriptions for
  `RON-TC-TRAJ-003` and `RON-TC-TRAJ-005` through `RON-TC-TRAJ-008`.
- Trajectory generation remains standalone from PID internals and only reuses
  shared C platform/fault/status conventions.
- Local evidence after enabling trajectory generators:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
  - `cmake --build regulon-c/build --config Debug`: passes
  - `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`: passes
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`: passes with 100% statement and branch coverage
  - `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`: passes with local `cross-arm` and `cross-arm-clang` evidence using freestanding header fallback; `cbmc` skipped because CBMC is unavailable
  - `git diff --check`: passes

## Phase 6 Kalman Filter Opening Evidence

- `ron_kalman.h` and `ron_kalman.c` are now active in the default C11 build.
  The public API matches the IS specification (`ron_kf_t`, `ron_kf_config_t`,
  `ron_kf_state_t`, and the `ron_kf_init` / `ron_kf_reset` / `ron_kf_predict`
  / `ron_kf_update` / `ron_kf_get_state` lifecycle); the header includes
  `ron/ron_pid_types.h` rather than `ron/ron_platform.h` directly so it
  inherits the shared `ron_fault_t` / `RON_FAULT_*` conventions used by every
  other active C11 module.
- `test_ron_kalman.c` covers `RON-TC-KF-001` through `RON-TC-KF-008`,
  including scalar convergence, full parameter/dimension validation
  including positive- and negative-infinity rejection, hand-checked
  predict/update reference cases for both `n == 1` and `n == 2` instances,
  the diagonal and non-diagonal Cholesky paths and the non-positive-
  definite `S` rejection for `m > 1`, the degenerate scalar `S` guard for
  `m == 1`, Joseph-form parity with the standard update including
  symmetry, measurement dropout (with `z = NULL`), steady-state fixed-gain
  mode, maximum-dimension storage exercising all `RON_KF_MAX_*` bounds, and
  all defensive null / uninitialised / non-finite input paths plus the
  `RON_FAULT_OUTPUT_NAN` numeric-overflow detection in both
  `ron_kf_predict` and `ron_kf_update`.
- `regulon-c/test/formal/kalman_no_heap_proof.c` adds the
  `RON-TC-KF-008-FV` CBMC harness for the Kalman lifecycle; the harness
  is discovered automatically by the dynamic `*_proof.c` enumeration in
  both the local verify script and CI.
- `docs/specs/TP_ControlLib.rst` now records detailed entries for
  `RON-TC-KF-002` through `RON-TC-KF-005`, `RON-TC-KF-007`,
  `RON-TC-KF-008`, and `RON-TC-KF-008-FV`; the previously existing
  `RON-TC-KF-001` and `RON-TC-KF-006` entries are unchanged.
- `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml` now
  list `ron_kalman.c` and `ron_kalman.h` in the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source/header sets.  The same edit closes
  a pre-existing gap by also adding `ron_cascade.c` / `ron_cascade.h` to
  the verify script, bringing it into line with CI.
- The library `add_library(regulon STATIC ...)` source list and the
  `regulon-c/test/CMakeLists.txt` test enumeration now include
  `ron_kalman.c` and `test_ron_kalman` respectively.
- Local evidence after enabling the Kalman slice:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
  - `cmake --build regulon-c/build --config Debug`: passes
  - `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: 9/9
    suites pass, including the new `test_ron_kalman`.
  - Single- and double-precision GCC builds (`RON_USE_DOUBLE=ON` and the
    default), standalone Clang / Ninja build, and GCC
    `-fsanitize=address,undefined -fno-sanitize-recover=all` build: all
    9 suites pass on each configuration.
  - `clang-format --dry-run --Werror` over the new `ron_kalman.c`,
    `ron_kalman.h`, `test_ron_kalman.c`, and `kalman_no_heap_proof.c`:
    passes.
  - `clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over the new
    `kalman_no_heap_proof.c`: passes.
  - `gcov -b -c` instrumentation of the active C source set including
    `ron_kalman.c`: 100% line coverage and 100% branch coverage
    (both directions taken) on `ron_kalman.c`.
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 6)

- The Linux verification host used for this slice lacks
  `libclang_rt.profile-x86_64.a` and `libclang_rt.asan-x86_64.a`, so the
  clang LLVM source-based coverage and clang ASan/UBSan builds remain a
  CI responsibility.  GCC `--coverage` (gcov) is used locally and reports
  100% line and 100% branch coverage on `ron_kalman.c`; the equivalent
  LLVM `llvm-cov` gate continues to be enforced by `ci_c.yml`.
- `cppcheck` and `lizard` are not installed on this verification host;
  static-analysis (MISRA) and complexity gates remain a CI responsibility.
- `cbmc` and `arm-none-eabi-gcc` are not installed on this verification
  host; the formal Kalman no-heap proof and the ARM GCC cross-compile
  smoke build remain CI responsibilities.  The dynamic harness discovery
  in the verify script and CI picks up `kalman_no_heap_proof.c`
  automatically once `cbmc` is available.

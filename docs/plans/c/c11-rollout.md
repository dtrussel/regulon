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

## Phase 7 State-Space / Observer Opening Evidence

- `ron_statespace.h` / `ron_statespace.c` and `ron_observer.h` /
  `ron_observer.c` are now active in the default C11 build.  The
  state-feedback controller computes `u = -K x_hat + Kr r` with three
  selectable estimate sources (external vector, embedded Luenberger
  observer, embedded Kalman filter), optional integral augmentation, and
  PID-equivalent output saturation and rate limiting; the Luenberger
  observer implements `x_hat(k+1) = A x_hat + B u + L (y - C x_hat)`.  The
  public API matches the IS specification added for both modules.
- The bounded fixed-size matrix / vector primitives (load/store, mat-vec,
  mat-mat, mat-mat^T, add, Cholesky factor/solve, bounded Newton sqrt) were
  factored out of `ron_kalman.c` into the new internal, non-public
  `regulon-c/src/ron_matrix.{c,h}` helper (uniform `RON_MAT_MAX_DIM`
  stride), now shared by the Kalman, state-space, and observer modules.
  This is a pure extraction: the Kalman public API, numerics, and full
  `RON-TC-KF-*` suite are unchanged.
- `test_ron_observer.c` covers `RON-TC-SS-006` through `RON-TC-SS-009`
  (hand-checked two-step recursion, scalar convergence fixture, per-matrix
  non-finite config rejection, full defensive null / uninitialised /
  non-finite paths, maximum-dimension storage, and `RON_FAULT_OUTPUT_NAN`
  overflow detection).  `test_ron_statespace.c` covers `RON-TC-SS-001`
  through `RON-TC-SS-005` plus `RON-TC-SS-009` (reference state feedback,
  all three estimate sources with embedded-estimator advance and
  cross-source rejection, integral accumulation / clamp / reset,
  saturation and bidirectional rate limiting matching the PID pipeline,
  runtime gain update, and the full configuration-validation and defensive
  surface).
- `regulon-c/test/formal/statespace_sat_proof.c` adds the
  `RON-TC-SS-004-FV` CBMC harness proving the output stays within
  `[u_min, u_max]` and that the state-space path performs no heap
  allocation; it is discovered automatically by the dynamic `*_proof.c`
  enumeration in both the verify script and CI.
- `docs/specs/IS_ControlLib.rst` now contains the `ron_observer.h` and
  `ron_statespace.h` API blocks; `docs/specs/TP_ControlLib.rst` records
  detailed entries for `RON-TC-SS-001` through `RON-TC-SS-009` and
  `RON-TC-SS-004-FV`.
- `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml` now
  list `ron_matrix.c`, `ron_observer.c`, `ron_statespace.c` (and the new
  public headers / internal helper header) in the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source sets, with `-I regulon-c/src` added
  to the cppcheck and CBMC invocations so the shared internal header
  resolves.  `RON_SS_MAX_*` minimum-bound static asserts and the
  `RON_MAT_MAX_DIM` definition / coverage asserts were added to
  `ron_platform.h`.
- Local evidence after enabling the Phase 7 slice:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
    `cmake --build regulon-c/build`: pass with no warnings under the strict
    flag set.
  - `ctest --test-dir regulon-c/build --output-on-failure`: 11/11 suites
    pass, including the new `test_ron_observer` and `test_ron_statespace`
    and the unchanged `test_ron_kalman`.
  - Default and double-precision (`RON_USE_DOUBLE=ON`) GCC builds,
    standalone Clang / Ninja build, and GCC
    `-fsanitize=address,undefined -fno-sanitize-recover=all` build: all
    11 suites pass on each configuration.
  - `clang-format --dry-run --Werror` over every new source / header and
    `clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
    `statespace_sat_proof.c`: pass.
  - `gcov -b` over the active C source set: 100% line and 100% branch
    coverage (both directions taken) on `ron_matrix.c`, `ron_observer.c`,
    `ron_statespace.c`, and `ron_kalman.c`.
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 7)

- As in Phase 6, this verification host lacks the clang LLVM coverage /
  ASan runtime libraries, `cppcheck`, `lizard`, `cbmc`, and
  `arm-none-eabi-gcc`; the LLVM `llvm-cov` 100% gate, MISRA static
  analysis, lizard complexity (CCN <= 10), the `RON-TC-SS-004-FV` CBMC
  proof, and the ARM GCC cross-compile smoke build remain CI
  responsibilities.  The dynamic harness discovery picks up
  `statespace_sat_proof.c` automatically once `cbmc` is available.

## Phase 8 Relay Feedback Auto-Tuning Opening Evidence

- `ron_autotune.h` / `ron_autotune.c` are now active in the default C11
  build, replacing the previous `ron_autotune.c` stub.  The module
  implements the relay excitation lifecycle
  (`RON-FR-800`), configurable relay amplitude / hysteresis / minimum cycles
  / timeout (`RON-FR-801`), zero-crossing `Ku = 4d/(pi*A)` and `Tu`
  estimation (`RON-FR-802`), the Ziegler-Nichols, Tyreus-Luyben,
  some-overshoot, and no-overshoot tuning rules (`RON-FR-803`), apply-only
  gain commit (`RON-FR-804`), raw `Ku` / `Tu` exposure (`RON-FR-805`), a
  relay output bounded to `[u_bias - d, u_bias + d]` (`RON-FR-806`), and
  abort-with-restore on caller abort or timeout (`RON-FR-807`).

- The module is standalone scalar math (SADS DD-15 — zero-crossing counting,
  no FFT, no buffers) and carries no `ron_matrix` dependency.  It references
  the target PID only through the existing atomic APIs: `ron_autotune_start`
  snapshots the gains / mode and parks the PID in manual via
  `ron_pid_set_mode`; `ron_autotune_apply` commits the tuned gains via
  `ron_pid_set_gains`; `ron_autotune_abort` and the timeout path restore the
  snapshot.

- `test_ron_autotune.c` covers `RON-TC-AT-001` through `RON-TC-AT-008`
  (closed-loop first-order-plant excitation, configuration validation,
  Ku/Tu accuracy within 10% on a synthetic reference oscillation, all four
  tuning rules, apply-only gain commit, raw Ku/Tu exposure, relay-bound and
  step guards, and abort / timeout restore), plus an insufficient-excitation
  abort case.

- `regulon-c/test/formal/autotune_relay_bound_proof.c` adds the
  `RON-TC-AT-007-FV` CBMC harness proving the relay output stays within
  `[u_bias - d, u_bias + d]` for any bounded finite step and that the
  auto-tuner path performs no heap allocation; it is picked up by the
  dynamic `*_proof.c` discovery used by the verify script and CI.

- `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml` now
  list `ron_autotune.c` and `ron_autotune.h` in the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source sets.

- Local evidence after enabling the Phase 8 slice:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
    `cmake --build regulon-c/build`: pass with no warnings under the strict
    flag set.
  - `ctest --test-dir regulon-c/build`: 12/12 suites pass, including the new
    `test_ron_autotune`.
  - Default and double-precision (`RON_USE_DOUBLE=ON`) GCC builds,
    standalone Clang build, and GCC
    `-fsanitize=address,undefined -fno-sanitize-recover=all` build: all
    12 suites pass on each configuration.
  - `clang-format --dry-run --Werror` over every new source / header / test /
    harness and `gcc -fsyntax-only` over `autotune_relay_bound_proof.c`: pass.
  - `gcov -b` on `ron_autotune.c`: 100% line and 100% branch coverage (both
    directions taken) and 100% call coverage.
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 8)

- As in Phases 6 and 7, this verification host lacks the clang LLVM coverage /
  ASan runtime libraries, `cppcheck`, `lizard`, `cbmc`, and
  `arm-none-eabi-gcc`; the LLVM `llvm-cov` 100% gate, MISRA static analysis,
  lizard complexity (CCN <= 10), the `RON-TC-AT-007-FV` CBMC proof, and the
  ARM GCC cross-compile smoke build remain CI responsibilities.  The dynamic
  harness discovery picks up `autotune_relay_bound_proof.c` automatically once
  `cbmc` is available.

## Phase 9 Health Monitor Opening Evidence

- `ron_health.h` / `ron_health.c` are now active in the default C11 build,
  replacing the previous `ron_health.c` stub.  The module is a passive
  control-loop health monitor: it attaches to any controller and evaluates loop
  health each step from `(r, y, u, dt)` (`RON-FR-900`), reporting five
  conditions through a latched `ron_health_status_t` bitmask — output-stuck,
  diverging, oscillating, sensor-dropout, and setpoint-unreachable
  (`RON-FR-901`) — each with its own threshold and time constant
  (`RON-FR-902`).  It never modifies the controller (`RON-FR-903`, SADS DD-16),
  fires the optional `ron_health_cb_t` on each condition's first activation
  (`RON-FR-904`), and latches every condition until `ron_health_clear`
  (`RON-FR-905`).  The header includes `ron/ron_pid_types.h` for the shared
  `ron_float_t` / `ron_fault_t` conventions and carries no PID or `ron_matrix`
  dependency.

- The detectors implement the SADS comparators directly: a saturation /
  dropout / settling duration counter per condition (rounded to the nearest
  sample so time-based thresholds land deterministically under single
  precision), a fixed `RON_HEALTH_OSC_WINDOW` ring of error signs for the
  oscillation count, and a growing-magnitude test for divergence.  Output-stuck
  is realised as an unchanged output for `t_sat_max`, the only reading
  consistent with the IS configuration (which exposes no `u_min` / `u_max`);
  the previous setpoint for step detection is recovered from the stored
  `e_prev + y_prev`, and two opaque state fields (`u_prev`, `prev_valid`)
  extend the IS-enumerated `ron_health_state_t` for the stuck reference and the
  first-step guard.  No new `ron_fault_t` bits were added — misuse reuses
  `RON_FAULT_NULL_POINTER` / `RON_FAULT_CONFIG_INVALID`.

- `test_ron_health.c` covers `RON-TC-HLTH-001` through `RON-TC-HLTH-010`,
  including the init / attach lifecycle and full per-field configuration and
  defensive validation, the threshold-boundary output-stuck case (not set at
  step 49, set at step 50, callback once), each of the other four condition
  detectors with isolated deterministic stimuli, independent per-condition
  thresholds, a passivity check that compares a 200-step PID loop with and
  without the monitor attached and asserts bit-identical controller output,
  callback-once-per-first-activation across two conditions plus a NULL-callback
  path, and the latch / clear / re-detect cycle.

- `regulon-c/test/formal/health_no_heap_proof.c` adds the `RON-TC-HLTH-008-FV`
  CBMC harness proving that one bounded finite step performs no heap allocation
  (`RON-SR-003`) and only ever sets status bits (monotonic latch, `RON-FR-905`,
  supporting the `RON-FR-903` passive property); it is discovered automatically
  by the dynamic `*_proof.c` enumeration in the verify script and CI.

- `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml` now list
  `ron_health.c` and `ron_health.h` in the format, cppcheck/MISRA, complexity,
  coverage, and CBMC source sets.  The library `add_library(regulon STATIC ...)`
  source list and the `regulon-c/test/CMakeLists.txt` test enumeration now
  include `ron_health.c` and `test_ron_health` respectively.

- Local evidence after enabling the Phase 9 slice:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
    `cmake --build regulon-c/build`: pass, warning-free under
    `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
  - `ctest --test-dir regulon-c/build`: 13/13 suites pass, including the new
    `test_ron_health`.
  - Default and double-precision (`RON_USE_DOUBLE=ON`) GCC builds, standalone
    Clang build, and GCC `-fsanitize=address,undefined -fno-sanitize-recover=all`
    build: all 13 suites pass on each configuration.
  - `clang-format --dry-run --Werror` over the new source / header / test /
    harness and `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
    `health_no_heap_proof.c`: clean.
  - `gcov -b` on `ron_health.c`: 100% line and 100% branch coverage (both
    directions taken).
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 9)

- As in Phases 6 through 8, this verification host lacks the clang LLVM coverage
  / ASan runtime libraries, `cppcheck`, `lizard`, `cbmc`, and
  `arm-none-eabi-gcc`; the LLVM `llvm-cov` 100% gate, MISRA static analysis,
  lizard complexity (CCN <= 10), the `RON-TC-HLTH-008-FV` CBMC proof, and the
  ARM GCC cross-compile smoke build remain CI responsibilities.  The dynamic
  harness discovery picks up `health_no_heap_proof.c` automatically once `cbmc`
  is available.

## Phase 10 Runtime Metrics Opening Evidence

- `ron_metrics.h` / `ron_metrics.c` are now active in the default C11 build,
  replacing the previous `ron_metrics.c` stub.  The module is a passive runtime
  performance metrics accumulator: it attaches to any controller and quantifies
  closed-loop quality each step from `(r, y, dt)` (`RON-FR-950`), computing the
  error integrals IAE, ISE, ITAE and the step-response transients peak
  overshoot, rise time, and settling time (`RON-FR-951`).  It supports
  cumulative and windowed accumulation (`RON-FR-952`), is enable/disable at
  runtime and disabled by default with a zero-overhead disabled path
  (`RON-FR-953`), and auto-restarts the transient metrics when it detects a
  setpoint step `|Δr| >= step_thresh` (`RON-FR-954`).  The header includes
  `ron/ron_pid_types.h` for the shared `ron_float_t` / `ron_fault_t`
  conventions and carries no PID, health, or `ron_matrix` dependency.

- The per-step update implements the SADS `ron_metrics` pseudocode directly:
  the error integrals always accumulate (ITAE weighted by the elapsed time
  since the last step / window restart), while the transient metrics are
  evaluated only when the captured `|step_size|` exceeds `RON_METRICS_MIN_STEP`
  so the divide by the step is never taken on a zero step.  Rise time is the
  10 %→90 % fraction crossing, overshoot is the sign-normalised peak beyond the
  target (a refinement of the SADS positive-step form that also handles
  downward steps), and settling uses the same nearest-sample duration rounding
  as the health monitor so the band dwell lands deterministically under single
  precision.  Windowed mode rolls every `window_steps` samples by restarting the
  integrals and timers while keeping the step reference frame.  No new
  `ron_fault_t` bits were added — misuse reuses `RON_FAULT_NULL_POINTER` /
  `RON_FAULT_CONFIG_INVALID`.

- `test_ron_metrics.c` covers `RON-TC-MET-001` through `RON-TC-MET-007`,
  including the init / enable / reset / get / step lifecycle and full per-field
  configuration and defensive validation, the closed-form IAE/ISE/ITAE
  reference (constant error 0.5 over 100 steps → 0.5 / 0.25 / 0.2525), peak
  overshoot, rise and settling tracking against analytic step responses,
  windowed vs cumulative accumulation, the disabled zero-overhead / no-state
  change path alongside a 200-step PID loop whose output is bit-identical with
  metrics enabled or disabled, and setpoint-step detection restarting the
  transient frame.

- `regulon-c/test/formal/metrics_no_heap_proof.c` adds the `RON-TC-MET-001-FV`
  CBMC harness proving that one bounded finite step performs no heap allocation
  (`RON-SR-003`) and only ever adds non-negative contributions to the IAE / ISE
  integrals (`RON-FR-951`, supporting the `RON-FR-953` passive property); it is
  discovered automatically by the dynamic `*_proof.c` enumeration in the verify
  script and CI.

- `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml` now list
  `ron_metrics.c` and `ron_metrics.h` in the format, cppcheck/MISRA, complexity,
  coverage, and CBMC source sets.  The library `add_library(regulon STATIC ...)`
  source list and the `regulon-c/test/CMakeLists.txt` test enumeration now
  include `ron_metrics.c` and `test_ron_metrics` respectively.

- Local evidence after enabling the Phase 10 slice:
  - `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
    `cmake --build regulon-c/build`: pass, warning-free under
    `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
  - `ctest --test-dir regulon-c/build`: 14/14 suites pass, including the new
    `test_ron_metrics`.
  - Default and double-precision (`RON_USE_DOUBLE=ON`) GCC builds, standalone
    Clang build, and GCC `-fsanitize=address,undefined -fno-sanitize-recover=all`
    build: all 14 suites pass on each configuration.
  - `clang-format --dry-run --Werror` over the new source / header / test /
    harness and `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
    `metrics_no_heap_proof.c`: clean.
  - `gcov -b` on `ron_metrics.c`: 100% line and 100% branch coverage (both
    directions taken).
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 10)

- As in Phases 6 through 9, this verification host lacks the clang LLVM coverage
  / ASan runtime libraries, `cppcheck`, `lizard`, `cbmc`, and
  `arm-none-eabi-gcc`; the LLVM `llvm-cov` 100% gate, MISRA static analysis,
  lizard complexity (CCN <= 10), the `RON-TC-MET-001-FV` CBMC proof, and the
  ARM GCC cross-compile smoke build remain CI responsibilities.  The dynamic
  harness discovery picks up `metrics_no_heap_proof.c` automatically once `cbmc`
  is available.

## Phase 11 Full-Library Integration And Release Hardening Closure Evidence

- Phase 11 hardens the library for release rather than adding a module. The
  active public surface is now the complete set: `ron_platform.h`,
  `ron_pid_types.h`, `ron_pid.h`, `ron_feedforward.h`, `ron_filter.h`,
  `ron_gain_sched.h`, `ron_cascade.h`, `ron_trajectory.h`, `ron_kalman.h`,
  `ron_statespace.h`, `ron_observer.h`, `ron_autotune.h`, `ron_health.h`,
  `ron_metrics.h`, the aggregate `ron.h`, and the generated `ron_modules.h`.

- Added the aggregate header `ron/ron.h` (guarded by `RON_HAVE_<MODULE>` macros
  from the CMake-generated `ron/ron_modules.h`); per-module `RON_ENABLE_*`
  options with dependency resolution (state-space forces Kalman; PID core +
  feed-forward are the mandatory baseline); the `RON-TC-INT-001`..`INT-005`
  integration suite; host examples behind `RON_BUILD_EXAMPLES`; and centralized
  CI source manifests (`scripts/lib_sources.txt`, `scripts/format_files.txt`)
  with a drift-check (`scripts/check_manifest.sh`).

- The drift check and CBMC inventory audit closed real gaps: the CI format list
  was missing `ron_matrix_internal.h`; `verify_pid.ps1` omitted four sources
  from its CBMC set; and `RON-TC-CASC-004-FV` has no dedicated harness yet
  (recorded as OI-TP-06). The specs were reconciled: TP gained the `INT` module,
  the five `RON-TC-INT-*` blocks, both matrices, and the CBMC harness inventory;
  IS and SADS gained the aggregate header and module-selection options.

- Local evidence after completing the Phase 11 slice:
  - Full default build (GCC) `ctest`: 15/15 suites pass, including
    `test_ron_integration` (`RON-TC-INT-001`..`005`).
  - Clang full build, double-precision (`RON_USE_DOUBLE=ON`), and GCC
    `-fsanitize=address,undefined -fno-sanitize-recover=all`: 15/15 each.
  - Minimal-subset build (all `RON_ENABLE_*` OFF): PID-only library compiles and
    links. Partial subset (filter+health+metrics): 7/7 suites; integration suite
    correctly not registered. `RON_ENABLE_STATESPACE` force-enables Kalman.
  - Single-include compile of `ron/ron.h` under the strict flag set: clean.
  - `RON_BUILD_EXAMPLES=ON` (GCC): both examples build and run.
  - `clang-format --dry-run --Werror` over the format manifest (incl. `ron.h`):
    clean. `bash regulon-c/scripts/check_manifest.sh`: manifests in sync.
  - `git diff --check`: passes.

### Residual Tool Gaps (Phase 11)

- As in Phases 6 through 10, this verification host lacks the clang LLVM coverage
  / ASan runtime libraries, `cppcheck`, `lizard`, `cbmc`, and
  `arm-none-eabi-gcc`; the LLVM `llvm-cov` 100% gate, MISRA static analysis,
  lizard complexity (CCN <= 10), the CBMC proofs, and the ARM cross-compile smoke
  builds remain CI responsibilities. The new `manifest-check`, `build-subset`,
  and `build-examples` CI jobs run on stock Ubuntu runners with no extra tooling.

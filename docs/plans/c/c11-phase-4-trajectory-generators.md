# C11 Phase 4 Trajectory Generators Plan

Date: 2026-05-08
Status: Complete

## Objective

Implement the C11 trajectory-generator slice for `regulon-c` so Phase 4 of
`docs/plans/c11-roadmap.md` is vertically complete and can be closed with the
same evidence bar used for PID, filters, feed-forward, and gain scheduling.

This file is the Phase 4 living record. It was created before implementation
and updated after completion with final evidence, residual gaps, and design
choices.

## Scope

Requirement scope:

- `RON-FR-500` through `RON-FR-503`
- `RON-FR-510` through `RON-FR-513`

Primary test scope:

- `RON-TC-TRAJ-001` through `RON-TC-TRAJ-008`

Out of scope:

- Cascade, Kalman, state-space, auto-tune, health, metrics, or any later module.
- PID integration beyond exposing feed-forward-compatible kinematic outputs.
- New formal test IDs unless `docs/specs/TP_ControlLib.rst` is explicitly
  expanded first.

## Implemented Shape

- Added `regulon-c/include/ron/ron_trajectory.h` with explicit trapezoidal and
  S-curve phase enums, config/state/instance types, and lifecycle/step/hold
  APIs.
- Replaced `regulon-c/src/ron_trajectory_trap.c` and
  `regulon-c/src/ron_trajectory_scurve.c` stubs with bounded implementations.
- Added `regulon-c/test/unit/test_ron_trajectory.c` covering all active
  `RON-TC-TRAJ-*` IDs.
- Updated `docs/specs/IS_ControlLib.rst` and `docs/specs/TP_ControlLib.rst`
  before implementation where the current trajectory state and test details are
  incomplete.
- Enabled the module in CMake and extended local/CI verification source lists
  after the trajectory tests are active.

## Design Constraints

- Keep `ron_trajectory` standalone from PID internals.
- Use the shared `ron_float_t`, `ron_fault_t`, and `ron_status_t` conventions.
- Do not use dynamic allocation, recursion, VLAs, global mutable state,
  unbounded loops, hidden OS calls, or future-module artifacts.
- Avoid `<math.h>` in trajectory production sources; use bounded internal root
  helpers for the short-move and S-curve planning calculations.
- Preserve current kinematic state on online replanning. Exact acceleration
  continuity for arbitrary S-curve replans is not claimed in this phase.

## Verification Evidence

Local evidence collected on 2026-05-08:

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes.
- `cmake --build regulon-c/build --config Debug`: passes.
- `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: 7/7 tests
  passed.
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`:
  passed.
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`:
  passed for MSVC Debug, double-precision MSVC, and standalone Clang/Ninja.
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`:
  passed with 100% statement and branch coverage across the active source set.
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`:
  probe passed, `cross-arm` passed, `cross-arm-clang` passed, and `cbmc` skipped
  because CBMC is not installed on this host.
- `git diff --check`: passed.

## Implemented Files

- `regulon-c/include/ron/ron_trajectory.h`
- `regulon-c/src/ron_trajectory_trap.c`
- `regulon-c/src/ron_trajectory_scurve.c`
- `regulon-c/test/unit/test_ron_trajectory.c`
- `regulon-c/CMakeLists.txt`
- `regulon-c/test/CMakeLists.txt`
- `regulon-c/scripts/verify_pid.ps1`
- `.github/workflows/ci_c.yml`
- `docs/specs/IS_ControlLib.rst`
- `docs/specs/TP_ControlLib.rst`
- `docs/plans/c/c11-rollout.md`
- `docs/plans/c11-roadmap.md`
- `CHANGELOG.rst`

## Residual Tool Gaps

- CBMC was not executed locally because `cbmc` is not installed. The verification
  script reports this as a clean skip.
- ARM and ARMv7 Clang cross-builds succeeded using the existing declaration-only
  freestanding header fallback because Newlib headers are not installed.

## Deliberate Design Choices

- Trajectory generation is standalone from PID internals and only reuses shared
  platform, fault, and status conventions.
- Trapezoidal replanning preserves velocity continuity by retaining current
  kinematic state and recomputing the signed target plan.
- S-curve planning uses a symmetric bounded seven-phase jerk-limited plan at
  `set_target()` time. Zero-duration phases are explicit and skipped by bounded
  iteration.
- Internal square-root and cube-root helpers use fixed iteration counts instead
  of `<math.h>` to preserve the C11 header/library restrictions.
- Exact acceleration continuity across arbitrary S-curve replans is not claimed
  in Phase 4; the implementation preserves current state and recomputes a
  bounded profile conservatively.

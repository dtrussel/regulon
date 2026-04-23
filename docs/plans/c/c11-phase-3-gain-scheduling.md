# C11 Phase 3 Gain Scheduling Plan

Date: 2026-04-23
Status: Complete

## Objective

Implement the C11 gain-scheduling slice for `regulon-c` so Phase 3 of
`docs/plans/c11-roadmap.md` is vertically complete and can be closed with the
same evidence bar already used for PID, filters, and feed-forward.

This plan is intended to be a living record. While Phase 3 is in progress it
tracks scope and execution order. After Phase 3 is finished, this same file
should be updated with the completion evidence, residual gaps, and final status.

## Completion Update

Phase 3 is now implemented and locally closed.

Implemented files:

- `regulon-c/include/ron/ron_gain_sched.h`
- `regulon-c/src/ron_gain_sched.c`
- `regulon-c/include/ron/ron_pid.h`
- `regulon-c/src/ron_pid_api.c`
- `regulon-c/CMakeLists.txt`
- `regulon-c/test/CMakeLists.txt`
- `regulon-c/test/unit/test_ron_gain_sched.c`
- `regulon-c/test/unit/test_ron_pid_api.c`
- `regulon-c/scripts/verify_pid.ps1`
- `docs/plans/c/c11-rollout.md`
- `docs/plans/c11-roadmap.md`
- `CHANGELOG.rst`

Implemented outcome:

- Added the public `ron_gain_sched` C11 header and enabled the real
  implementation in the default build.
- Added `ron_pid_set_config()` as the public atomic full-config PID update path
  and used it from the runtime setter APIs.
- Implemented bounded table validation, binary-search breakpoint selection,
  hard-switch scheduling, linear interpolation, and optional integral reset on
  successful hard switches.
- Adopted the conservative interpolation rule planned for this phase:
  interpolate `Kp`, `Ki`, and `Kd` only; require all other
  `ron_pid_config_t` fields to match across adjacent interpolation entries.
- Added `RON-TC-GS-001` through `RON-TC-GS-008` coverage plus direct
  `RON-TC-PID-033` regressions for the new full-config PID setter.

Exact local verification results:

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`: passes
- `cmake --build regulon-c/build --config Debug`: passes
- `ctest --test-dir regulon-c/build -C Debug --output-on-failure`: passes
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`: passes
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`: passes
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`: passes with 100% statement and branch coverage for the active source set
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`: passes with `cross-arm-clang` local evidence, `cross-arm` skipped because `arm-none-eabi-gcc` is unavailable, and `cbmc` skipped because CBMC is unavailable

Residual gaps:

- No local `arm-none-eabi-gcc` evidence on this host; CI remains the ARM GCC
  smoke-build evidence path.
- No local CBMC execution evidence on this host; CI remains the formal proof
  execution path.

Deliberate implementation choices:

- `ron_gs_init()` takes `const ron_gs_table_t *` and performs validation only;
  no mutable scheduler state was introduced in this phase.
- `ron_gs_update()` now relies on prior table validation to guarantee that the
  candidate config is safe to apply, so the post-validation `ron_pid_set_config`
  call is treated as non-failing in the scheduler path.

## Scope

Requirement scope:

- `RON-FR-300` through `RON-FR-306`

Primary test scope:

- `RON-TC-GS-001` through `RON-TC-GS-008`

Secondary regression scope:

- Existing `RON-TC-PID-*` tests that cover runtime update behaviour,
  multi-instance independence, and unchanged default PID behaviour when
  scheduling is inactive.

Out of scope for this phase:

- Cascade controller, trajectory generators, or any later roadmap module.
- Public/default-build enablement of any future module beyond gain scheduling.
- New TP IDs that are not already defined in `docs/specs/TP_ControlLib.rst`.

## Design Constraints That Matter Here

- Keep the slice bounded to `ron_gain_sched` plus the minimum PID API/support
  extension required to apply a full candidate configuration atomically.
- Do not rely on dynamic allocation, recursion, global mutable state, or
  unbounded loops.
- Keep PID default behaviour unchanged when no scheduler instance is used.
- Preserve the existing feed-forward and filter fields inside
  `ron_pid_config_t` during scheduler-driven updates; Phase 3 must not narrow
  the already-implemented PID configuration surface.

## Current Starting Point

- `regulon-c/src/ron_gain_sched.c` exists only as a stub and is not part of the
  default build.
- No public `regulon-c/include/ron/ron_gain_sched.h` header exists yet.
- The PID API has per-field runtime setters, but gain scheduling needs a
  coherent full-config apply path to satisfy `RON-FR-303` for both hard-switch
  and interpolated updates.
- `docs/specs/SADS_ControlLib.rst` already fixes the intended algorithm shape:
  bounded breakpoint table, binary search lookup, hard switch or linear
  interpolation, and optional integral reset on hard switch.

## Planned Implementation Shape

### Story GS-01: Open the public module surface

Deliverables:

- Add `regulon-c/include/ron/ron_gain_sched.h`.
- Replace the stub in `regulon-c/src/ron_gain_sched.c` with the real
  implementation.
- Add the new source/header to the active CMake build only together with the
  Phase 3 tests.

Planned public types and operations:

- A bounded scheduler mode enum for hard switch vs. linear interpolation.
- A table/config structure holding:
  - breakpoint count
  - breakpoint array
  - per-breakpoint `ron_pid_config_t` records
  - mode
  - `reset_on_switch`
- Public operations for:
  - table validation / initialisation
  - scheduler update against a PID instance
  - state/diagnostic query if needed for `RON-FR-304` and Phase 3 diagnostics

Notes:

- Keep gain-scheduling types in `ron_gain_sched.h` rather than expanding
  `ron_pid_types.h` unless a shared PID type becomes unavoidable.
- The header must follow the same traceability/comment pattern as the existing
  C11 public headers.

### Story GS-02: Add the enabling PID runtime update path

Reason:

- `RON-FR-303` requires atomic application of a full scheduled configuration.
- Hard switch applies a full stored `ron_pid_config_t`.
- Linear interpolation yields a candidate config record that still has to be
  validated and committed as one coherent update.

Planned changes:

- Promote the current internal candidate-apply logic into a reusable PID helper,
  or add a public `ron_pid_set_config()` API if that is the cleaner way to make
  the atomic contract explicit.
- Keep validation single-source-of-truth in `ron_pid_config_validate()`.
- Ensure failed scheduler updates leave the PID instance completely unchanged.

Regression expectations:

- Existing runtime-update PID tests continue to pass.
- Add at least one direct regression for atomic no-change-on-invalid-candidate
  behaviour if the current PID suite does not already prove it strongly enough.

### Story GS-03: Breakpoint-table validation and lookup

Requirement coverage:

- `RON-FR-300`
- `RON-FR-301`
- `RON-FR-306`

Planned behaviour:

- Validate `n_points` against `RON_GS_MAX_BREAKPOINTS`.
- Reject null pointers and zero-point tables.
- Enforce strictly increasing breakpoints.
- Validate every associated `ron_pid_config_t` via
  `ron_pid_config_validate()`.
- Use bounded binary search to locate the active region.
- Clamp below-range and above-range scheduling inputs to the first/last region.

Verification focus:

- Boundary selection at lower bound, upper bound, exact breakpoint, and outside
  the configured range.
- Rejection of duplicate, unsorted, or invalid-config table entries.

### Story GS-04: Hard-switch update path

Requirement coverage:

- `RON-FR-302` hard-switch mode
- `RON-FR-303`
- `RON-FR-304`
- `RON-FR-305`

Planned behaviour:

- `ron_gs_update()` remains separate from `ron_pid_step()`.
- In hard-switch mode, apply the selected stored config atomically.
- If `reset_on_switch` is true, reset only the integral state after a
  successful config update.
- Do not reset integral state when the selected breakpoint region does not
  change, unless the final API contract explicitly requires it.

Verification focus:

- Correct breakpoint selection.
- Correct next-step PID output after a scheduler update.
- Optional integral reset only on successful hard-switch updates.

### Story GS-05: Linear interpolation path

Requirement coverage:

- `RON-FR-302` linear interpolation mode
- `RON-FR-303`

Planned behaviour:

- Interpolate the schedule-dependent PID tuning/config fields between the two
  bracketing breakpoints.
- Preserve non-interpolated configuration fields in a deterministic, documented
  way:
  - copy invariant/common fields from the lower breakpoint only when all table
    entries are required to match; or
  - explicitly enforce equality for fields that are not safe to interpolate.

Planned conservative rule:

- Interpolate only the continuous scalar fields that have a clear physical
  meaning under interpolation.
- Treat mode/enum/boolean fields and callback pointers as invariant across the
  table and reject tables that vary them in interpolation mode.

Reason:

- The SRS only requires interpolated gains, but the stored unit is a full
  `ron_pid_config_t`. The plan should keep the implementation conservative and
  deterministic rather than silently inventing semantics for enum or callback
  transitions.

Verification focus:

- Exact lower breakpoint, exact upper breakpoint, interior interpolation, and
  last-segment clamping.
- Invalid interpolated candidate handling leaves the PID config unchanged.

### Story GS-06: Diagnostics, tests, and quality-gate closure

Requirement coverage:

- `RON-FR-304`
- Phase 3 roadmap diagnostic expectations

Planned deliverables:

- `regulon-c/test/unit/test_ron_gain_sched.c` covering
  `RON-TC-GS-001` through `RON-TC-GS-008`.
- Extend `regulon-c/test/CMakeLists.txt` and `regulon-c/CMakeLists.txt`.
- Extend `regulon-c/scripts/verify_pid.ps1` active-source/header lists so
  format, complexity, cppcheck, clang, and coverage include gain scheduling.
- Update `CHANGELOG.rst`.
- Update `docs/plans/c/c11-rollout.md` with a Phase 3 evidence section once the
  code is complete.
- Update `docs/plans/c11-roadmap.md` Phase 3 status from planned/active to
  complete only after all exit criteria are met.

## File-Level Change Plan

Expected files to add:

- `regulon-c/include/ron/ron_gain_sched.h`
- `regulon-c/test/unit/test_ron_gain_sched.c`

Expected files to change:

- `regulon-c/src/ron_gain_sched.c`
- `regulon-c/include/ron/ron_pid.h` if a full-config runtime setter is added
- `regulon-c/src/ron_pid_api.c`
- `regulon-c/src/ron_pid_internal.h` if the atomic apply helper is shared
- `regulon-c/test/CMakeLists.txt`
- `regulon-c/CMakeLists.txt`
- `regulon-c/scripts/verify_pid.ps1`
- `docs/plans/c/c11-rollout.md`
- `docs/plans/c11-roadmap.md`
- `CHANGELOG.rst`

Files that should remain untouched unless Phase 3 proves otherwise:

- Future-module public headers and sources outside the gain-scheduling slice.
- TP/SRS/SADS documents, unless implementation discovers a real spec mismatch
  that must be corrected explicitly.

## Verification Plan

Minimum local evidence expected before Phase 3 can be marked complete:

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`
- `cmake --build regulon-c/build --config Debug`
- `ctest --test-dir regulon-c/build -C Debug --output-on-failure`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`

Phase-specific pass criteria:

- `RON-TC-GS-001` through `RON-TC-GS-008` are implemented and passing.
- Active C source coverage remains at 100% statement and branch coverage after
  adding the scheduler.
- PID and feed-forward regression suites remain green.
- Default PID behaviour is unchanged when scheduling is not active.

## Execution Order

1. Add or expose the atomic full-config PID update path.
2. Add the public gain-scheduling header and bounded table types.
3. Write the Gain Scheduling Unity suite for `RON-TC-GS-001` through
   `RON-TC-GS-008`.
4. Implement table validation and binary-search lookup.
5. Implement hard-switch mode and optional integral reset.
6. Implement linear interpolation and invariant-field enforcement.
7. Wire the module into CMake and the verification script.
8. Run the full active C11 verification set.
9. Update rollout, roadmap, and changelog with completion evidence.

## Completion Update Instructions

When Phase 3 is complete, update this file in place with:

- `Status: complete`
- final implemented file list
- exact local verification results
- any environment-limited gaps that remain CI-only
- any deliberate deviations or conservative interpolation rules that were
  adopted during implementation

At the same time, update:

- `docs/plans/c/c11-rollout.md` with a `Phase 3 Gain Scheduling Opening Evidence`
  section
- `docs/plans/c11-roadmap.md` Phase 3 status and baseline summary
- `CHANGELOG.rst`

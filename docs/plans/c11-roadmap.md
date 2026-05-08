# C11 Implementation Roadmap

Date: 2026-04-23

## Purpose

This roadmap guides future `regulon-c` implementation work toward full coverage of the current specifications in `docs/specs/`. It is intentionally feature-level: each iteration should still start by reading the applicable SRS requirements, SADS design, IS API definitions, and TP test IDs before writing code.

Ground-truth inputs:

- `docs/specs/SRS_ControlLib.rst`
- `docs/specs/SADS_ControlLib.rst`
- `docs/specs/IS_ControlLib.rst`
- `docs/specs/TP_ControlLib.rst`
- `docs/plans/c/c11-rollout.md`

## Roadmap Rules

- Keep each implementation iteration vertically complete: public API, implementation, unit tests, formal/inspection evidence where applicable, CI wiring, and documentation updates.
- Do not add public headers, default-build sources, or tests for a future module until that module is the active iteration scope.
- Every test or proof implemented in C must use a test ID that already exists in `TP_ControlLib.rst`.
- Preserve the C11 constraints: no dynamic allocation, no recursion, no VLAs, no global mutable state, no unbounded loops, no hidden OS or hardware dependency.
- Every feature slice must finish with traceability annotations in source and tests, plus `CHANGELOG.rst` and plan updates.
- The active quality bar is 100% statement and branch coverage for the active C source set, MISRA/cppcheck pass, complexity pass, host tests, double-precision tests where applicable, Clang tests, and ARM cross-compile smoke evidence.

## Current Baseline

The active C11 implementation now contains the closed PID baseline, the
Phase 1 signal-conditioning filter module, the Phase 2 feed-forward PID
extension, the Phase 3 gain-scheduling slice, and the Phase 4 trajectory
generator slice. The current public surface is:

- `regulon-c/include/ron/ron_platform.h`
- `regulon-c/include/ron/ron_pid_types.h`
- `regulon-c/include/ron/ron_pid.h`
- `regulon-c/include/ron/ron_filter.h`
- `regulon-c/include/ron/ron_feedforward.h`
- `regulon-c/include/ron/ron_gain_sched.h`
- `regulon-c/include/ron/ron_trajectory.h`

The PID slice covers `RON-FR-001` through `RON-FR-071` with supporting safety, performance, quality, and diagnostics evidence. Phase 0 PID closure has been accepted for opening non-PID work. Phase 1 filters cover `RON-FR-100` through `RON-FR-131` with active unit tests and local 100% statement/branch coverage.
Phase 2 feed-forward covers `RON-FR-200` through `RON-FR-205` with active
unit tests for `RON-TC-FF-001` through `RON-TC-FF-009`. Phase 3 gain
scheduling covers `RON-FR-300` through `RON-FR-306` with active unit tests for
`RON-TC-GS-001` through `RON-TC-GS-008`. Phase 4 trajectory generators cover
`RON-FR-500` through `RON-FR-513` with active unit tests for
`RON-TC-TRAJ-001` through `RON-TC-TRAJ-008`.

## Sequencing Strategy

Implement modules in dependency order:

1. Standalone infrastructure and reusable components.
2. PID extensions that depend on those components.
3. Composition modules that orchestrate PID instances or generated signals.
4. Matrix/state-estimation modules.
5. Supervisory modules and release-level evidence.

This order keeps each slice testable in isolation and avoids coupling later modules to incomplete support libraries.

## Phase 0: PID Closure And Baseline Freeze

Status: Complete. PID closure evidence is recorded in `docs/plans/c/c11-rollout.md`; residual local ARM/CBMC gaps are tool-availability gaps covered by CI wiring.

Requirement scope:

- `RON-FR-001` through `RON-FR-071`
- `RON-SR-001` through `RON-SR-022`
- `RON-PR-001` through `RON-PR-022`
- `RON-QR-001` through `RON-QR-031` as applicable to PID

Feature scope:

- Treat PID as the reference implementation for object model, error handling, traceability annotations, build wiring, formal harness style, and CI evidence.
- Close any residual CI-only gaps in coverage artifact generation, ARM toolchain setup, CBMC execution, and formatting.
- Freeze PID public API unless a later spec update explicitly requires a compatible extension.

Exit criteria:

- PID-only CI passes on host GCC sanitizer, GCC double, Clang, format, cppcheck/MISRA, complexity, LLVM coverage 100/100, CBMC, and ARM cross-compile.
- `c11-rollout.md`, `TP_ControlLib.rst`, and `CHANGELOG.rst` reflect final PID closure evidence.
- A reusable checklist exists for opening and closing non-PID slices.

## Phase 1: Signal Conditioning Filters

Status: Complete. The C11 filter public API, implementation, unit tests, formal harness inventory, build wiring, CI wiring, changelog, and test-plan details are in place.

Requirement scope:

- `RON-FR-100` through `RON-FR-131`

SADS/IS scope:

- `ron_filter`
- Public header: `ron_filter.h`
- Source: `ron_filter.c`

Feature scope:

- Common filter instance/config/state model aligned with PID conventions.
- First-order low-pass filter with smoothing-factor and time-constant configuration.
- Moving-average FIR with caller-provided fixed storage and recursive update.
- Biquad IIR with direct-form-II-transposed state, coefficient validation, cascaded sections, and coefficient hot-swap.
- Standalone rate-of-change limiter.

Dependencies:

- `ron_platform.h`
- Shared status/fault conventions from PID types

Verification focus:

- Coefficient validation and numerical stability guards.
- Reset and state retention behavior.
- Known-signal responses for LPF, moving average, biquad, and rate limiter.
- Boundary cases: invalid coefficients, zero/negative sample time, saturation of configured section counts, NaN/Inf rejection.

Exit criteria:

- `ron_filter.h` and `ron_filter.c` are added to the default C build only when all filter tests are active.
- Filter source set reaches 100% statement and branch coverage.
- Test plan includes `RON-TC-FILT-*` entries already defined in `TP_ControlLib.rst`.

## Phase 2: Feed-Forward PID Extension

Status: Complete. The C11 feed-forward public API, implementation, PID core
integration, unit tests, build wiring, CI wiring, changelog, and test-plan
details are in place.

Requirement scope:

- `RON-FR-200` through `RON-FR-205`

SADS/IS scope:

- `ron_feedforward`
- Integration into PID core as an optional additive path
- Public header: `ron_feedforward.h`
- Source: `ron_feedforward.c`

Feature scope:

- Feed-forward modes for static gain, velocity, and acceleration terms.
- Independent derivative estimation and filtering for feed-forward.
- Addition of `u_FF` before existing output saturation and rate limiting.
- Diagnostics/status reporting of feed-forward contribution.
- Runtime enable/disable and configuration validation.

Dependencies:

- PID baseline
- Filter primitives from Phase 1

Verification focus:

- Additive output math with saturation/rate-limit interaction.
- Independence from PID derivative filter state.
- Safe handling of disabled modes and invalid feed-forward configuration.
- Regression that PID behavior is unchanged when feed-forward is disabled.

Exit criteria:

- PID coverage and formal evidence remain closed after integration.
- Feed-forward tests cover each mode and diagnostic path.

## Phase 3: Gain Scheduling

Status: Complete. Closure evidence is recorded in
`docs/plans/c/c11-phase-3-gain-scheduling.md` and
`docs/plans/c/c11-rollout.md`.

Requirement scope:

- `RON-FR-300` through `RON-FR-306`

SADS/IS scope:

- `ron_gain_sched`
- Public header: `ron_gain_sched.h`
- Source: `ron_gain_sched.c`

Feature scope:

- Breakpoint table validation.
- Hard-switch scheduling.
- Linear-interpolation scheduling.
- Atomic gain/configuration update through PID runtime APIs.
- Diagnostics for active region and invalid scheduling input.

Dependencies:

- PID runtime update APIs

Verification focus:

- Sorted breakpoint validation and boundary selection.
- Interpolation correctness at lower bound, upper bound, exact breakpoint, and between breakpoints.
- Atomicity failure handling when candidate PID config is invalid.
- Multi-instance independence.

Exit criteria:

- Gain scheduling can be built and tested independently of cascade/trajectory modules.
- PID default behavior remains unchanged when scheduling is not active.

## Phase 4: Trajectory Generators

Status: Complete. Closure evidence is recorded in
`docs/plans/c/c11-phase-4-trajectory-generators.md` and
`docs/plans/c/c11-rollout.md`.

Requirement scope:

- `RON-FR-500` through `RON-FR-513`

SADS/IS scope:

- `ron_trajectory`
- Sources: `ron_trajectory_trap.c`, `ron_trajectory_scurve.c`
- Public header: `ron_trajectory.h`

Feature scope:

- Trapezoidal velocity profile generation.
- S-curve jerk-limited profile generation.
- Position, velocity, acceleration, and jerk outputs as specified.
- Restart/replan behavior and completion status.
- Feed-forward-compatible outputs.

Dependencies:

- Platform/types baseline
- Optional later use by feed-forward, but implementation should remain standalone

Verification focus:

- Profile phase transitions.
- Limit compliance for velocity, acceleration, and jerk.
- Degenerate short moves.
- Direction changes and restart behavior.
- Deterministic output for fixed sample sequences.

Exit criteria:

- Trajectory module can be used without PID.
- No dynamic storage is required for profile execution.
- Active local gates pass, including host builds/tests, double precision,
  standalone Clang, format, cppcheck/MISRA, complexity, LLVM 100% statement and
  branch coverage, ARM cross-compile smoke builds, and `git diff --check`.

## Phase 5: Cascade Controller

Requirement scope:

- `RON-FR-400` through `RON-FR-406`

SADS/IS scope:

- `ron_cascade`
- Public header: `ron_cascade.h`
- Source: `ron_cascade.c`

Feature scope:

- Outer/inner PID instance composition.
- Inner setpoint generation from outer output.
- Coordinated saturation and limit enforcement.
- Back-calculation anti-windup propagation between loops.
- Unified status and fault reporting.
- Manual/automatic mode handling across both loops.

Dependencies:

- PID baseline
- Optional feed-forward and gain-scheduling compatibility

Verification focus:

- Two-loop nominal step behavior with deterministic fixtures.
- Inner saturation and anti-windup propagation.
- Fault propagation and safe-output behavior.
- Independence of outer and inner configurations.

Exit criteria:

- Cascade tests include interaction-level coverage, not just isolated PID calls.
- Existing PID unit and formal gates remain unchanged and passing.

## Phase 6: Kalman Filter

Requirement scope:

- `RON-FR-600` through `RON-FR-607`

SADS/IS scope:

- `ron_kalman`
- Public header: `ron_kalman.h`
- Source: `ron_kalman.c`

Feature scope:

- Discrete linear predict/update cycle.
- Caller-owned matrix/vector storage with fixed maximum dimensions.
- Cholesky-based innovation solve for multi-dimensional measurements.
- Joseph-form covariance update option.
- Steady-state mode.
- Measurement dropout handling.

Dependencies:

- A small internal fixed-size matrix helper may be introduced only if it is scoped, tested, and justified by Kalman/state-space reuse.

Verification focus:

- Known small-dimensional examples with reference numeric outputs.
- Dimension and storage validation.
- Positive-definite and near-singular innovation handling.
- Dropout path and steady-state update path.
- Numerical finite checks.

Exit criteria:

- Matrix helper, if introduced, has its own tests and does not become an unbounded generic math library.
- Kalman source coverage and branch coverage meet active gates.

## Phase 7: State-Space Controller And Luenberger Observer

Requirement scope:

- `RON-FR-700` through `RON-FR-704`
- `RON-FR-720` through `RON-FR-723`

SADS/IS scope:

- `ron_statespace`
- `ron_observer`
- Public headers: `ron_statespace.h`, `ron_observer.h`
- Sources: `ron_statespace.c`, `ron_observer.c`

Feature scope:

- State-feedback controller.
- Optional integral augmentation.
- Shared saturation and rate-limit behavior aligned with PID semantics.
- External-state input mode.
- Luenberger observer predict/update cycle.
- Compatibility path for Kalman-provided state estimates.

Dependencies:

- Fixed-size matrix helper from Phase 6, if accepted.
- PID-equivalent output limiting semantics.

Verification focus:

- Reference state-feedback outputs.
- Integral augmentation accumulation and reset.
- Observer convergence fixture.
- Saturation/rate-limit consistency with PID behavior.
- Invalid dimensions, null pointers, nonfinite input, and storage bounds.

Exit criteria:

- State-space and observer modules are independently usable.
- Shared math helpers remain deterministic and bounded.

## Phase 8: Relay Feedback Auto-Tuning

Requirement scope:

- `RON-FR-800` through `RON-FR-807`

SADS/IS scope:

- `ron_autotune`
- Public header: `ron_autotune.h`
- Source: `ron_autotune.c`

Feature scope:

- Relay excitation lifecycle: init, start, step, apply, abort, results.
- Zero-crossing based oscillation detection.
- Ultimate gain and period estimation.
- Ziegler-Nichols, Tyreus-Luyben, and conservative tuning rules.
- Safe abort on PID fault or timeout.

Dependencies:

- PID baseline
- Metrics or health modules are not required for first implementation; avoid circular coupling.

Verification focus:

- Synthetic oscillation signals with known period/amplitude.
- Timeout, insufficient oscillation, and fault abort.
- Gain-rule calculations.
- Staged apply behavior and restoration of previous output on abort.

Exit criteria:

- Auto-tune never modifies PID gains unless the apply operation succeeds.
- Fault and abort paths are covered by unit tests and, where practical, formal harnesses.

## Phase 9: Health Monitor

Requirement scope:

- `RON-FR-900` through `RON-FR-905`

SADS/IS scope:

- `ron_health`
- Public header: `ron_health.h`
- Source: `ron_health.c`

Feature scope:

- Output-stuck detection.
- Divergence detection.
- Oscillation detection.
- Sensor dropout detection.
- Setpoint-unreachable detection.
- Optional condition callback.

Dependencies:

- Platform/types baseline
- May consume PID status but must remain usable with other controller outputs.

Verification focus:

- Sliding-window behavior with bounded buffers.
- Threshold and hysteresis edge cases.
- Callback-on-new-condition behavior.
- Clear/get status behavior.

Exit criteria:

- Health monitor has no dependency on PID internals.
- Detection logic is deterministic and bounded.

## Phase 10: Runtime Performance Metrics

Requirement scope:

- `RON-FR-950` through `RON-FR-954`

SADS/IS scope:

- `ron_metrics`
- Public header: `ron_metrics.h`
- Source: `ron_metrics.c`

Feature scope:

- Error-integral metrics such as IAE, ISE, ITAE, and RMSE as specified.
- Overshoot, settling, and steady-state error tracking.
- Enable/disable/reset behavior.
- Setpoint-step restart handling.
- Fixed-window or cumulative modes as defined by IS/SADS.

Dependencies:

- Platform/types baseline
- Optional health monitor interoperability

Verification focus:

- Closed-form reference signals for each metric.
- Setpoint-step restart behavior.
- Disabled mode and reset behavior.
- Overflow and nonfinite input handling.

Exit criteria:

- Metrics module is usable as an observer with no effect on controller output.
- Deterministic reproducibility tests are added.

## Phase 11: Full-Library Integration And Release Hardening

Requirement scope:

- All functional ranges: `RON-FR-001` through `RON-FR-954`
- All safety, performance, quality, standards, diagnostics, and design-constraint requirements in the current SRS and TP.

Feature scope:

- Enable all implemented public headers and sources in the default CMake build.
- Add aggregate examples only after all required APIs are stable.
- Validate combined include topology and absence of circular dependencies.
- Revisit CMake options for module selection if full-library build time or embedded integration needs require it.

Verification focus:

- Full-library host tests.
- Full-library coverage gate, or clearly documented per-module active-source gates if full 100/100 aggregate coverage is impractical for generated/defensive paths.
- ARM cross-compile with all modules enabled.
- CBMC harness inventory across safety-critical modules.
- Traceability audit from each requirement to at least one test/proof/inspection item.

Exit criteria:

- `TP_ControlLib.rst` traceability tables are complete and internally consistent.
- `IS_ControlLib.rst` matches public headers exactly.
- `SADS_ControlLib.rst` matches implemented module boundaries and algorithms.
- `CHANGELOG.rst` records full C11 implementation completion.

## Per-Iteration Deliverables

Each feature iteration should produce:

- A short implementation plan or update under `docs/plans/`.
- Public header(s) only for the active module.
- Source files listed in CMake only when the module is ready for active build/test.
- Unit tests for every active TP ID.
- Formal or inspection harnesses for safety, bounds, memory, deterministic execution, and no-blocking claims where the TP defines them.
- CI updates for any newly active source files.
- Documentation updates to SRS/SADS/IS/TP only when requirements, design, API, proof bounds, or verification claims change.
- `CHANGELOG.rst` entry.

## Release Gate Checklist

Before declaring any roadmap phase complete:

- Requirement IDs are identified and mapped to source/test annotations.
- No future module artifacts were added opportunistically.
- Host unit tests pass.
- Double-precision regression passes when the module uses `ron_float_t`.
- Clang build passes.
- `clang-format --dry-run --Werror` passes for active sources and headers.
- cppcheck/MISRA pass for active production sources.
- Complexity is within the configured threshold.
- Coverage reaches the active target.
- ARM cross-compile passes.
- Formal/inspection evidence is current or a justified TP update records why it is not applicable.
- `git diff --check` is clean.

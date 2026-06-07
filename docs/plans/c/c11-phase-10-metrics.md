# C11 Phase 10 Runtime Performance Metrics Plan

Date: 2026-06-07 (planned), 2026-06-07 (closed)
Status: Complete

## Objective

Implement the C11 runtime performance metrics accumulator (`ron_metrics`) slice
for `regulon-c` so Phase 10 of `docs/plans/c11-roadmap.md` is vertically
complete and closed with the same evidence bar used for PID, filters,
feed-forward, gain scheduling, trajectory generators, the cascade controller,
the Kalman filter, the state-space / observer slice, the relay auto-tuner, and
the health monitor.

This file is the Phase 10 living record: it fixes scope and execution order and
is updated in place on completion with the implemented file list, verification
results, residual tool gaps, and deliberate design choices.

## Scope

Requirement scope:

- `RON-FR-950` through `RON-FR-954` (attachable accumulator; IAE / ISE / ITAE,
  peak overshoot, settling time, rise time; windowed and cumulative modes;
  runtime enable/disable with zero overhead when disabled; setpoint-step
  detection that restarts the transient metrics).

Primary test scope:

- `RON-TC-MET-001` through `RON-TC-MET-007` (registered in
  `docs/specs/TP_ControlLib.rst`; the detailed blocks for `MET-001`, `MET-003`,
  `MET-004`, `MET-005`, and `MET-007` were added in this phase to complement the
  pre-existing `MET-002` and `MET-006` blocks).
- A `metrics_no_heap_proof.c` CBMC harness (`RON-TC-MET-001-FV`) for no-heap /
  bounded-step evidence, consistent with the no-heap proofs in earlier phases.

Out of scope: the Rust `metrics.rs` port (tracked separately under
`docs/plans/rust/rust-first-rollout.md`), full-library integration (Phase 11),
any later roadmap module, and any change to the authoritative metric set.

## Metric Set (authoritative)

The SRS / IS / SADS specify exactly six metrics — IAE, ISE, ITAE, peak
overshoot (%), rise time, and settling time. The earlier "RMSE / steady-state
error" wording in the Phase 10 roadmap section is loose and **superseded**; per
`AGENTS.md` the specs are ground truth, so no spec amendment was needed and no
RMSE / steady-state field was added.

## Public API (match IS verbatim)

`regulon-c/include/ron/ron_metrics.h` matches the IS API block
(`IS_ControlLib.rst` lines 2221-2273):

- `ron_metrics_mode_t`: `RON_METRICS_CUMULATIVE` `0`, `RON_METRICS_WINDOWED` `1`.
- `ron_metrics_config_t`: `mode`, `window_steps` (uint32), `band_pct`,
  `settle_confirm`, `step_thresh`.
- `ron_metrics_result_t`: `IAE`, `ISE`, `ITAE`, `peak_overshoot`, `rise_time`,
  `settling_time`.
- `ron_metrics_t`: `cfg`, the internal running state (the IS "fields omitted for
  brevity" expanded into the concrete `MetricsState` from SADS:1817), `enabled`,
  `is_initialised`.
- Functions returning `ron_fault_t`: `ron_metrics_init(m, cfg)`,
  `ron_metrics_reset(m)`, `ron_metrics_enable(m, enable)`,
  `ron_metrics_step(m, r, y, dt)`, `ron_metrics_get(m, out)`.
- Includes `"ron/ron_pid_types.h"` for `ron_float_t` / `ron_fault_t`; the header
  is PID-, health-, and `ron_matrix`-independent. (The IS sketch shows
  `ron_platform.h`, but `ron_fault_t` lives in `ron_pid_types.h`, the same
  include every fault-returning module uses.)

## Algorithm Shape (from SADS:1812)

With `e = r - y`, per enabled step:

- Time and integrals: `t_elapsed += dt`; `IAE += |e| dt`; `ISE += e^2 dt`;
  `ITAE += t_elapsed |e| dt`.
- Rise time: `frac = (y - step_ref) / step_size`; capture `t_rise_start` at the
  first `frac >= 0.10`; latch `rise_time = t_elapsed - t_rise_start` at the first
  `frac >= 0.90`.
- Overshoot: `peak_overshoot = max(peak_overshoot, (y - step_target)/step_size *
  100)` while that fraction is positive.
- Settling: while `|e| <= |step_size| * band_pct`, accumulate the in-band dwell;
  latch `settling_time = t_elapsed` once the dwell reaches `settle_confirm`.

## Settled Design Decisions

- **Standalone passive observer.** `ron_metrics` reuses only the shared
  platform/fault conventions; it reads `(r, y, dt)` and writes only its own
  instance. It never touches the controller, satisfying the passive-observer
  intent behind RON-FR-953.
- **Disabled by default + zero-overhead disabled path.** `ron_metrics_init`
  leaves `enabled = false` (SADS:1953). `ron_metrics_step` returns immediately
  after the handle check when disabled, performing no work and changing no
  state (RON-FR-953 / `RON-TC-MET-006`).
- **Transient guard on `|step_size|`.** Rise / overshoot / settling are
  evaluated only when the captured `|step_size| > RON_METRICS_MIN_STEP`, so the
  divide by `step_size` is never reachable on a zero step and the transient
  metrics stay at their `-1` / `0` sentinels until a real step exists.
- **Sign-normalised transients.** Rise and overshoot use the step-normalised
  fraction `(y - ref)/step_size`, a refinement of the SADS positive-step form
  that also handles downward setpoint steps.
- **Setpoint-step restart.** `|Δr| >= step_thresh` (or the first sample)
  re-captures the step reference frame and clears the transient timers
  (RON-FR-954). The error integrals are not cleared by a step — their scope is
  governed by the mode.
- **Windowed = rolling restart of integrals.** Windowed mode restarts the error
  integrals and timers every `window_steps` samples while preserving the step
  reference frame and `r_prev`, using the `window_counter` field from SADS
  (no ring buffer, no dynamic allocation).
- **Nearest-sample duration rounding.** Settling confirms when
  `in_band_time + 0.5*dt >= settle_confirm`, matching the health monitor so the
  band dwell lands deterministically under single precision.
- **No new `ron_fault_t` bits.** Misuse (null `m`, uninitialised instance,
  non-positive/non-finite `dt`, non-finite `r/y`) returns
  `RON_FAULT_NULL_POINTER` / `RON_FAULT_CONFIG_INVALID`.
- **Per-metric helpers for CCN <= 10.** Validation, restart, and each metric
  update are small `static` helpers, mirroring `ron_health.c`.

## Implemented Files

Added:

- `regulon-c/include/ron/ron_metrics.h`, `regulon-c/src/ron_metrics.c`
  (replacing the previous stub `ron_metrics.c`).
- `regulon-c/test/unit/test_ron_metrics.c` (`RON-TC-MET-001`..`007`).
- `regulon-c/test/formal/metrics_no_heap_proof.c` (`RON-TC-MET-001-FV`, no-heap /
  monotone-integral CBMC harness, discovered automatically by the dynamic
  `*_proof.c` enumeration).

Modified:

- `regulon-c/CMakeLists.txt`, `regulon-c/test/CMakeLists.txt` — registered
  `src/ron_metrics.c` in `add_library(regulon STATIC ...)` (between
  `ron_matrix.c` and `ron_observer.c`) and added
  `ron_add_test(test_ron_metrics unit/test_ron_metrics.c)`.
- `regulon-c/scripts/verify_pid.ps1`, `.github/workflows/ci_c.yml` —
  `ron_metrics.c` / `ron_metrics.h` added to the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source sets.
- `docs/specs/TP_ControlLib.rst` — added the detailed `MET-001/003/004/005/007`
  blocks and the `RON-TC-MET-001-FV` formal block; registered the FV proof in
  the traceability matrix, the by-test table, and the coverage summary.
- `docs/plans/c11-roadmap.md`, `docs/plans/c/c11-rollout.md`, `CHANGELOG.rst` —
  Phase 10 status, opening evidence, and changelog entry.

## Verification Results

Collected on 2026-06-07 (Linux host, GCC + Clang + CMake + gcov):

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
  `cmake --build regulon-c/build`: pass, warning-free under
  `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
- `ctest --test-dir regulon-c/build`: 14/14 suites pass, including the new
  `test_ron_metrics` (`RON-TC-MET-001`..`007`).
- Double-precision (`RON_USE_DOUBLE=ON`), standalone Clang, and GCC
  `-fsanitize=address,undefined -fno-sanitize-recover=all` builds: 7/7
  `test_ron_metrics` cases pass on each configuration.
- `clang-format --dry-run --Werror` over every new source / header / test /
  harness, and `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
  `metrics_no_heap_proof.c`: clean.
- `gcov -b` on `ron_metrics.c`: 100% line and 100% branch coverage (both
  directions taken).
- `git diff --check`: passes.

## Residual Tool Gaps

As in Phases 6-9, this verification host lacks `cbmc`, `cppcheck`, `lizard`,
`arm-none-eabi-gcc`, and the clang LLVM coverage / ASan runtime libraries, so
the `RON-TC-MET-001-FV` CBMC proof, MISRA static analysis, lizard complexity
(CCN <= 10), the LLVM `llvm-cov` 100% gate, and the ARM cross-compile smoke
build remain CI responsibilities. The dynamic `*_proof.c` discovery picks up
`metrics_no_heap_proof.c` automatically once `cbmc` is available.

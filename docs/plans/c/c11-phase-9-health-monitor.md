# C11 Phase 9 Control Loop Health Monitor Plan

Date: 2026-06-05 (planned)
Status: Planned

## Objective

Implement the C11 control-loop health monitor (`ron_health`) slice for
`regulon-c` so Phase 9 of `docs/plans/c11-roadmap.md` is vertically complete and
closed with the same evidence bar used for PID, filters, feed-forward, gain
scheduling, trajectory generators, the cascade controller, the Kalman filter,
the state-space / observer slice, and the relay auto-tuner.

This file is the Phase 9 living record. It fixes scope and execution order and
will be updated in place on completion with the implemented file list,
verification results, residual tool gaps, and deliberate design choices.

## Scope

Requirement scope:

- `RON-FR-900` through `RON-FR-905` (attach + per-step evaluation; five health
  conditions reported via a latched bitmask; independent per-condition
  thresholds; passive / non-mutating; first-activation callback; latch until
  cleared).

Primary test scope:

- `RON-TC-HLTH-001` through `RON-TC-HLTH-010` (already defined in
  `docs/specs/TP_ControlLib.rst`).
- An optional `health_no_heap_proof.c` CBMC harness for no-heap / bounded-step
  evidence, consistent with the no-heap proofs in earlier phases.

Out of scope: the runtime metrics module (Phase 10), full-library integration
(Phase 11), any later roadmap module, and any new TP IDs or spec
(SRS/SADS/IS/TP) changes — the health-monitor specification is already complete
and frozen.

## Public API (match IS verbatim)

`regulon-c/include/ron/ron_health.h` is created to match the IS API block
(`IS_ControlLib.rst` lines 2159-2219) exactly:

- Status bitmask `ron_health_status_t` (uint8): `RON_HEALTH_OK` `0x00`,
  `RON_HEALTH_OUTPUT_STUCK` `0x01`, `RON_HEALTH_DIVERGING` `0x02`,
  `RON_HEALTH_OSCILLATING` `0x04`, `RON_HEALTH_SENSOR_DROPOUT` `0x08`,
  `RON_HEALTH_SP_UNREACHABLE` `0x10`.
- `typedef void (*ron_health_cb_t)(ron_health_status_t condition);`
- `ron_health_config_t`: `t_sat_max`, `err_diverge_thresh`, `osc_count_thresh`
  (uint8), `dead_band`, `dropout_time`, `ss_err_thresh`, `settling_time`, `cb`.
- `ron_health_state_t`: `status`, `t_saturated`, `t_dropout`, `t_since_step`,
  `osc_window[RON_HEALTH_OSC_WINDOW]` (uint8), `osc_idx` (uint8), `e_prev`,
  `y_prev`, `is_initialised` (bool).
- `ron_health_t { ron_health_config_t cfg; ron_health_state_t state; }`.
- Functions returning `ron_fault_t`: `ron_health_init(h, cfg)`,
  `ron_health_step(h, r, y, u, dt)`, `ron_health_clear(h)`,
  `ron_health_get(h, status)`.
- Includes `"ron/ron_pid_types.h"` for `ron_float_t` / `ron_fault_t`; the header
  is PID-independent (no `ron_pid.h`, no `ron_matrix.h`).

`RON_HEALTH_OSC_WINDOW` (`32U`) and its `>= 4U` static assert already exist in
`ron_platform.h` (lines 258 and 280) — no platform-header edit is required.

## Settled Design Decisions

- **Standalone passive observer (SADS DD-16).** `ron_health` reuses only the
  shared platform/fault/status conventions; it has no dependency on PID
  internals or `ron_matrix`. `ron_health_step` reads `(r, y, u, dt)` and writes
  only its own `state`, satisfying the passivity requirement RON-FR-903.
- **OUTPUT_STUCK = output value unchanged.** The IS `ron_health_config_t`
  exposes no `u_min` / `u_max`, so the SADS "output equals `u_min` or `u_max`"
  description is realised as *output-value-unchanged within an epsilon* for
  `t_sat_max`. This is the only reading consistent with the API and with
  `RON-TC-HLTH-002` (a constant `u` fed for >= 50 steps trips the condition).
- **Latch + first-activation callback.** Each condition bit latches on its first
  OK->set transition; `cfg.cb(bit)` fires exactly once per transition
  (RON-FR-904); bits clear only via `ron_health_clear`, which also resets all
  counters and the oscillation window (RON-FR-905).
- **No new `ron_fault_t` bits.** Misuse (null `h`, uninitialised instance,
  non-positive/non-finite `dt`, non-finite `r/y/u`) returns
  `RON_FAULT_NULL_POINTER` / `RON_FAULT_CONFIG_INVALID`, consistent with the
  rest of the library.
- **Per-condition helpers for CCN <= 10.** Each of the five comparators is a
  small `static` helper, mirroring the `ron_autotune.c` decomposition, to keep
  every function within the lizard complexity gate.

## Algorithm Shape (from SADS)

With `e = r - y`, each condition is an independent stateful comparator:

- **OUTPUT_STUCK** (RON-FR-901a): `t_saturated += dt` while `u` is unchanged
  within epsilon; reset on movement; trigger when `t_saturated > t_sat_max`.
- **DIVERGING** (RON-FR-901b): trigger when `|e| > err_diverge_thresh` and
  `sign(e) == sign(e - e_prev)` (large and still growing).
- **OSCILLATING** (RON-FR-901c): push `sign(e)` into the fixed
  `osc_window` ring buffer; count sign changes across the window; trigger when
  count `> osc_count_thresh`.
- **SENSOR_DROPOUT** (RON-FR-901d): `t_dropout += dt` while
  `|y - y_prev| < dead_band`; reset on movement; trigger when
  `t_dropout > dropout_time`.
- **SP_UNREACHABLE** (RON-FR-901e): a setpoint step (`r` change) resets
  `t_since_step`; otherwise `t_since_step += dt`; trigger when
  `|e| > ss_err_thresh` persists with `t_since_step > settling_time`.

## Planned Files

To add:

- `regulon-c/include/ron/ron_health.h`, `regulon-c/src/ron_health.c`
  (replacing the current stub `ron_health.c`).
- `regulon-c/test/unit/test_ron_health.c` (`RON-TC-HLTH-001`..`010`).
- `regulon-c/test/formal/health_no_heap_proof.c` (no-heap / bounded-step CBMC
  harness, discovered automatically by the dynamic `*_proof.c` enumeration).

To modify:

- `regulon-c/CMakeLists.txt`, `regulon-c/test/CMakeLists.txt` — register
  `src/ron_health.c` in `add_library(regulon STATIC ...)` (alphabetically
  between `ron_gain_sched.c` and `ron_kalman.c`) and add
  `ron_add_test(test_ron_health unit/test_ron_health.c)`.
- `regulon-c/scripts/verify_pid.ps1`, `.github/workflows/ci_c.yml` —
  `ron_health.c` / `ron_health.h` added to the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source sets.
- `docs/plans/c11-roadmap.md`, `docs/plans/c/c11-rollout.md`,
  `CHANGELOG.rst` — Phase 9 status, opening evidence, and changelog entry.

## Planned Tests (one per TP ID)

- `RON-TC-HLTH-001` (FR-900) — init / attach lifecycle and defensive
  null/uninitialised paths.
- `RON-TC-HLTH-002` (FR-901a) — OUTPUT_STUCK: `t_sat_max = 0.5 s`,
  `dt = 0.01 s`; not set at step 49, set at step 50; callback fires once.
- `RON-TC-HLTH-003`..`006` (FR-901b..e) — DIVERGING, OSCILLATING,
  SENSOR_DROPOUT, SP_UNREACHABLE, each with a deterministic stimulus.
- `RON-TC-HLTH-007` (FR-902) — independent thresholds: trip one condition
  without tripping the others.
- `RON-TC-HLTH-008` (FR-903) — passivity: 200 steps with all thresholds armed;
  the recorded controller `(r, y, u)` stream is identical to a run with no
  monitor attached (within `FLT_EPSILON`).
- `RON-TC-HLTH-009` (FR-904) — callback fires on first activation only.
- `RON-TC-HLTH-010` (FR-905) — status latches until `ron_health_clear` resets
  it and all counters / the oscillation window.

## Planned Verification

Replicate the Phase 8 evidence collection on the Linux host:

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
  `cmake --build regulon-c/build`: warning-free under the strict flag set.
- `ctest --test-dir regulon-c/build --output-on-failure`: all suites pass,
  including the new `test_ron_health`.
- Default and double-precision (`RON_USE_DOUBLE=ON`) GCC, standalone Clang, and
  GCC `-fsanitize=address,undefined -fno-sanitize-recover=all` builds: all pass.
- `clang-format --dry-run --Werror` over the new header/source/test/harness:
  clean.
- `gcov -b` on `ron_health.c`: 100% line and 100% branch (both directions).
- `gcc/clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
  `health_no_heap_proof.c`.
- `git diff --check`: clean.

## Anticipated Residual Tool Gaps

As in Phases 6-8, this verification host lacks `cbmc`, `cppcheck`, `lizard`,
`arm-none-eabi-gcc`, and the clang LLVM coverage / ASan runtime libraries, so
the `health_no_heap_proof.c` CBMC proof, MISRA static analysis, lizard
complexity (CCN <= 10), the LLVM `llvm-cov` 100% gate, and the ARM cross-compile
smoke build remain CI responsibilities. The dynamic `*_proof.c` discovery picks
up the health harness automatically once `cbmc` is available.

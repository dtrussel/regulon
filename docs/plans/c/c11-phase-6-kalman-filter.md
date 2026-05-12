# C11 Phase 6 Kalman Filter Plan

Date: 2026-05-12
Status: Planned

## Objective

Implement the C11 discrete linear Kalman filter slice for `regulon-c` so
Phase 6 of `docs/plans/c11-roadmap.md` is vertically complete and can be closed
with the same evidence bar already used for PID, filters, feed-forward, gain
scheduling, trajectory generators, and the cascade controller.

This file is the Phase 6 living record. It is created before implementation to
fix scope and execution order. When Phase 6 is complete this same file is
updated in place with the final implemented file list, exact local verification
results, residual tool gaps, and any deliberate design choices.

## Scope

Requirement scope:

- `RON-FR-600` through `RON-FR-607`

Primary test scope:

- `RON-TC-KF-001` through `RON-TC-KF-008`
- `RON-TC-KF-008-FV` (no-heap formal harness)

Supporting safety/quality regression scope (already-defined IDs only):

- `RON-TC-SAFE-001` style null-pointer / parameter validation applied to the
  Kalman API (covered by `RON-TC-KF-008` and the formal harness inventory).
- `RON-TC-QUAL-*` finite-output and determinism expectations as they apply to
  the Kalman source set.

Out of scope for this phase:

- State-space controller, Luenberger observer, relay auto-tune, health monitor,
  metrics, or any later roadmap module.
- Public/default-build enablement of any module other than `ron_kalman`.
- Any state-space / observer reuse of the matrix helper beyond what Phase 6
  itself needs and tests.
- New TP IDs that are not already present in `docs/specs/TP_ControlLib.rst`.
  Detailed *descriptions* for already-listed `RON-TC-KF-*` IDs may be added to
  the TP (the IDs `RON-TC-KF-001` through `RON-TC-KF-008` and
  `RON-TC-KF-008-FV` already exist in the traceability tables; only
  `RON-TC-KF-001` and `RON-TC-KF-006` currently have detailed sections).

## Ground-Truth Inputs

- `docs/specs/SRS_ControlLib.rst` — `RON-FR-600` … `RON-FR-607`.
- `docs/specs/SADS_ControlLib.rst` — "Module Design: ron_kalman" (data
  structures, predict pseudocode, update pseudocode, Cholesky note,
  Joseph-form note).
- `docs/specs/IS_ControlLib.rst` — `ron_kalman.h` API block: `ron_kf_config_t`,
  `ron_kf_state_t`, `ron_kf_t`, and the `ron_kf_init` / `ron_kf_reset` /
  `ron_kf_predict` / `ron_kf_update` / `ron_kf_get_state` signatures.
- `docs/specs/TP_ControlLib.rst` — `RON-TC-KF-001` … `RON-TC-KF-008`,
  `RON-TC-KF-008-FV`, and the traceability rows mapping them to
  `RON-FR-600` … `RON-FR-607` and `RON-SR-003`.
- `regulon-c/include/ron/ron_platform.h` — `RON_KF_MAX_STATES` (8),
  `RON_KF_MAX_MEASUREMENTS` (4), `RON_KF_MAX_INPUTS` (4), and the existing
  `RON_STATIC_ASSERT(RON_KF_MAX_STATES >= 1U, ...)`.

## Current Starting Point

- `regulon-c/src/ron_kalman.c` exists only as a stub and is not part of the
  default `add_library(regulon ...)` source list in `regulon-c/CMakeLists.txt`.
- No public `regulon-c/include/ron/ron_kalman.h` header exists yet; the IS
  document fixes the intended shape.
- `regulon-c/include/ron/ron_platform.h` already defines the Kalman dimension
  bounds and the `>= 1` static assert; no platform changes are expected unless
  implementation discovers a missing bound (e.g. an explicit
  `RON_KF_MAX_INPUTS >= 1` / `RON_KF_MAX_MEASUREMENTS >= 1` assert worth
  adding for symmetry with the other module bounds — to be decided during
  implementation, kept minimal).
- The active public surface is PID + filters + feed-forward + gain scheduling +
  trajectory + cascade; Phase 6 adds exactly one public header and one source
  file to that surface.

## Design Constraints That Matter Here

- Keep the slice bounded to `ron_kalman` plus a small, scoped, internal
  fixed-size matrix helper if and only if it is justified by Kalman use and is
  itself tested. Do **not** grow it into a generic unbounded math library, and
  do **not** add a separate public matrix header in this phase.
- Caller-owned, fixed-maximum storage only: all matrices/vectors live inside
  `ron_kf_config_t` / `ron_kf_state_t` with `RON_KF_MAX_*` dimensions; the
  active `n`, `m`, `p` are `uint8_t` fields validated against those bounds.
- No dynamic allocation, no recursion, no VLAs, no global mutable state, no
  unbounded loops, no hidden OS/hardware dependency.
- Avoid `<math.h>` in the Kalman production source. The Cholesky solver needs a
  square root: reuse the bounded fixed-iteration `sqrt` helper pattern already
  used by the trajectory module (or a small shared internal helper) rather than
  pulling in `<math.h>`. Use the existing `RON_ISNAN` / `RON_ISINF` /
  `RON_ISFINITE` macros from `ron_platform.h` for finite checks.
- Determinism: a fixed `(n, m, p)` and fixed inputs must produce bit-identical
  outputs across runs; loop bounds are the compile-time `RON_KF_MAX_*`
  constants with `idx < active_dim` guards.
- Keep `ron_kalman` standalone: no dependency on PID internals; only reuse the
  shared `ron_float_t`, `ron_fault_t`, and `RON_*` platform conventions
  (`ron_kalman.h` includes `ron/ron_platform.h`, not `ron/ron_pid.h`).
- Default behaviour of every already-active module must be unchanged; Phase 6 is
  purely additive.

## Algorithm Shape (from SADS)

System model (`RON-FR-600`):

- `x(k+1) = A x(k) + B u(k) + w(k)`, `z(k) = H x(k) + v(k)`.

Configuration (`RON-FR-601`, `RON-FR-607`): `A[n][n]`, `B[n][p]`, `H[m][n]`,
`Q[n][n]`, `R[m][m]`, `x0[n]`, `P0[n][n]`, `use_joseph_form`, `steady_state`,
`K_inf[n][m]` — all bounded by `RON_KF_MAX_*` compile-time constants.

`ron_kf_predict(kf, u[p])` (`RON-FR-602`):

- `x_prior = A·x_hat + B·u`
- `P_prior = A·P·Aᵀ + Q`
- Store `x_prior`, `P_prior` back into the instance state.

`ron_kf_update(kf, z[m], z_valid)` (`RON-FR-602`, `RON-FR-603`, `RON-FR-604`,
`RON-FR-605`, `RON-FR-606`):

- If `!z_valid`: return `RON_FAULT_NONE`, leave state untouched
  (measurement-dropout path, `RON-FR-605`).
- Gain:
  - `steady_state == true` → `K = K_inf` (`RON-FR-606`, no inversion).
  - else `S = H·P_prior·Hᵀ + R`; for `m == 1` use scalar division; for
    `m > 1` solve `K·S = P_prior·Hᵀ` (equivalently `K = P_prior·Hᵀ·S⁻¹`) via a
    Cholesky factorisation of `S` and forward/back substitution (`RON-FR-603`).
- Innovation update: `innov = z − H·x_hat`; `x_hat = x_hat + K·innov`.
- Covariance update:
  - `use_joseph_form == true` → `IKH = I − K·H`;
    `P = IKH·P_prior·IKHᵀ + K·R·Kᵀ` (`RON-FR-604`).
  - else → `P = (I − K·H)·P_prior`.

All matrix ops are fixed-size loops over the compile-time bounded dimensions;
no dynamic allocation.

## Planned Public Surface (`ron_kalman.h`)

Follow the IS document exactly:

- `ron_kf_config_t` — `uint8_t n, m, p;` plus `A`, `B`, `H`, `Q`, `R`, `x0`,
  `P0`, `use_joseph_form`, `steady_state`, `K_inf` with `RON_KF_MAX_*`
  dimensions.
- `ron_kf_state_t` — `x_hat[RON_KF_MAX_STATES]`, `P[RON_KF_MAX_STATES][...]`,
  `bool is_initialised;`.
- `ron_kf_t` — `{ ron_kf_config_t cfg; ron_kf_state_t state; }`.
- Operations:
  - `ron_fault_t ron_kf_init(ron_kf_t *kf, const ron_kf_config_t *cfg);`
  - `ron_fault_t ron_kf_reset(ron_kf_t *kf);`
  - `ron_fault_t ron_kf_predict(ron_kf_t *kf, const ron_float_t u[RON_KF_MAX_INPUTS]);`
  - `ron_fault_t ron_kf_update(ron_kf_t *kf, const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid);`
  - `ron_fault_t ron_kf_get_state(const ron_kf_t *kf, ron_float_t x_hat[RON_KF_MAX_STATES]);`

Header conventions: same Doxygen-style `@file`/`@module`/`@req`/`@version`
banner, traceability `@req RON-FR-600 … RON-FR-607`, include guard
`RON_KALMAN_H`, `extern "C"` block, and `#include "ron/ron_platform.h"` as in
the IS block.

Open design questions to settle during implementation (record the chosen answer
in the completion update):

- Fault taxonomy: reuse existing `ron_fault_t` bits (`RON_FAULT_NULL_POINTER`,
  `RON_FAULT_CONFIG_INVALID`, `RON_FAULT_INPUT_NAN`, `RON_FAULT_OUTPUT_NAN`) —
  conservative default — vs. introducing a dedicated bit for a non-positive-
  definite / near-singular innovation covariance. Prefer reuse
  (`RON_FAULT_CONFIG_INVALID` for an `S` that fails Cholesky,
  `RON_FAULT_INPUT_NAN` / `RON_FAULT_OUTPUT_NAN` for nonfinite inputs/results)
  unless a new bit is genuinely needed; do not change the `ron_fault_t`
  width.
- Whether `ron_kf_predict` validates `u` for finiteness, or trusts the caller —
  default: validate at the API boundary, consistent with the rest of the
  library.
- Symmetry enforcement on `P` after the standard (non-Joseph) update — default:
  do not force-symmetrise in Phase 6; document that Joseph form is the
  numerically robust option per SADS.

## Planned Internal Implementation (`ron_kalman.c` + optional helper)

- `ron_kalman.c` holds the public API, validation, predict, update, and the
  small matrix primitives it needs:
  - `mat_vec` (`y = M·x`), `mat_mat` (`C = A·B`), `mat_mat_T` (`C = A·Bᵀ`),
    `mat_add`, identity / `I − K·H`, all as `static` functions taking explicit
    `(rows, cols, ...)` and writing into caller-provided fixed-size buffers.
  - A `static` Cholesky factor + solve for the `m×m` innovation covariance,
    `m` bounded by `RON_KF_MAX_MEASUREMENTS` (4). Returns a fault if the
    diagonal pivot is not strictly positive (treats `S` as not positive
    definite).
  - A `static` bounded `sqrt` helper (fixed iteration count) so the source does
    not include `<math.h>` — mirrors the trajectory module pattern. If a single
    shared internal helper file is preferable, keep it tiny and tested, but the
    default plan keeps it `static` inside `ron_kalman.c` to avoid widening the
    build surface.
- Scratch buffers are local automatic arrays sized `RON_KF_MAX_*`; no heap, no
  VLAs, no recursion.
- `ron_kf_init` validates: non-null `kf` and `cfg`; `1 <= n <= RON_KF_MAX_STATES`,
  `1 <= m <= RON_KF_MAX_MEASUREMENTS`, `1 <= p <= RON_KF_MAX_INPUTS`; all used
  matrix/vector entries finite; `R` diagonal positive (so `m == 1` scalar
  division and the Cholesky path are well-defined); `P0` usable as an initial
  covariance. On success copies `cfg`, seeds `state.x_hat = x0`,
  `state.P = P0`, sets `is_initialised = true`.
- `ron_kf_reset` re-seeds `x_hat`/`P` from the stored `cfg.x0`/`cfg.P0` and
  requires `is_initialised`.
- `ron_kf_predict` / `ron_kf_update` require `is_initialised`, validate
  pointers and finite inputs, and follow the SADS pseudocode above. `update`
  with `z_valid == false` is a no-op returning `RON_FAULT_NONE`.
- `ron_kf_get_state` copies `state.x_hat` into the caller buffer (full
  `RON_KF_MAX_STATES`, with entries beyond `n` left as the stored values),
  requires `is_initialised`, validates pointers.

## Test Plan (`test/unit/test_ron_kalman.c`)

One Unity suite covering every active `RON-TC-KF-*` ID. The TP currently has
detailed sections only for `RON-TC-KF-001` and `RON-TC-KF-006`; the others are
named in the traceability table. The plan is to (a) add detailed TP sections for
`RON-TC-KF-002`, `RON-TC-KF-003`, `RON-TC-KF-004`, `RON-TC-KF-005`,
`RON-TC-KF-007`, `RON-TC-KF-008`, and `RON-TC-KF-008-FV` *before* writing the
tests (mirroring how Phase 4 filled in `RON-TC-TRAJ-*` descriptions), and
(b) implement the tests against those descriptions:

- `RON-TC-KF-001` — Scalar state estimation convergence. `A=1, B=0, H=1,
  Q=0.01, R=1, x0=0, P0=10`, true state `5.0`, 100 predict–update cycles with
  a deterministic pseudo-random measurement sequence. Assert `fabs(x_hat-5) <
  0.5` and `P` monotonically decreasing for the first 20 cycles. (`RON-FR-600`,
  `RON-FR-602`.)
- `RON-TC-KF-002` — Parameter-matrix configuration / `ron_kf_init` validation:
  dimension bounds, null pointers, nonfinite entries, non-positive `R`
  diagonal, and a successful multi-state/multi-measurement config.
  (`RON-FR-601`.)
- `RON-TC-KF-003` — Predict–update algorithm correctness against a hand-checked
  small example (e.g. a 2-state constant-velocity model) with reference numeric
  outputs after a fixed step sequence. (`RON-FR-602`.)
- `RON-TC-KF-004` — Cholesky-based innovation solve for `m > 1`: a 2- or
  3-measurement update with a known positive-definite `S`, compared against the
  reference gain/state; plus a near-singular / non-PD `S` that yields a fault
  and leaves the state unchanged. Also verify the `m == 1` scalar-division path
  matches the general path on a 1-measurement instance. (`RON-FR-603`.)
- `RON-TC-KF-005` — Joseph-form covariance update: same step sequence with
  `use_joseph_form = true` vs `false`, asserting both stay finite and agree to
  tolerance on a well-conditioned problem, and that Joseph form keeps `P`
  symmetric / non-negative-diagonal. (`RON-FR-604`.)
- `RON-TC-KF-006` — Measurement dropout: converge the scalar system over 50
  cycles, then 10 `ron_kf_predict` calls with `z_valid = false`; assert
  `x_hat` unchanged across the dropout, `P` increasing each dropout step, and no
  fault. (`RON-FR-605`.)
- `RON-TC-KF-007` — Steady-state (fixed-gain) mode: `steady_state = true` with a
  precomputed `K_inf`; assert no inversion path is exercised, the gain stays
  fixed, and the state/`P` follow the fixed-gain recursion. Contrast with the
  full-gain instance converging to the same neighbourhood. (`RON-FR-606`.)
- `RON-TC-KF-008` — No-dynamic-allocation / bounded-storage behaviour at the API
  level: maximum-dimension config initialises and runs; all storage is inside
  the caller-owned `ron_kf_t`; null/uninitialised-instance calls are rejected
  without side effects. (`RON-FR-607`.)

Formal harness:

- `RON-TC-KF-008-FV` — CBMC no-heap proof for the Kalman functions
  (`malloc`/`calloc`/`realloc`/`free` unreachable), mirroring the existing
  `pid_no_heap_proof.c` / `filter_no_heap_proof.c` pattern. Add the harness file
  under `regulon-c/test/formal/` and to the dynamic harness discovery used by
  the verify script and CI. (`RON-FR-607`, `RON-SR-003`.)
- Consider, but do not require beyond the TP, additional CBMC harnesses for
  null-pointer rejection, bounded execution, and bounded array access on the
  Kalman API, consistent with the filter/PID formal inventories — only if a
  corresponding TP ID already exists or is added explicitly.

## File-Level Change Plan

Files to add:

- `regulon-c/include/ron/ron_kalman.h`
- `regulon-c/test/unit/test_ron_kalman.c`
- `regulon-c/test/formal/kalman_no_heap_proof.c` (and any further
  `kalman_*_proof.c` only if their TP IDs already exist)

Files to change:

- `regulon-c/src/ron_kalman.c` — replace the stub with the real implementation.
- `regulon-c/CMakeLists.txt` — add `src/ron_kalman.c` to the `regulon` library
  source list (only together with the active Phase 6 tests).
- `regulon-c/test/CMakeLists.txt` — add `ron_add_test(test_ron_kalman
  unit/test_ron_kalman.c)`.
- `regulon-c/scripts/verify_pid.ps1` — add `ron_kalman.c` / `ron_kalman.h` to
  `$ActiveSources` and to the per-step cppcheck/MISRA, complexity, clang, and
  coverage source/header lists.
- `.github/workflows/ci_c.yml` — add `ron_kalman.c` / `ron_kalman.h` to the
  format-check, static-analysis, and coverage job file lists, consistent with
  how Phase 5 wired `ron_cascade`.
- `regulon-c/include/ron/ron_platform.h` — only if a missing `>= 1` static
  assert for `RON_KF_MAX_MEASUREMENTS` / `RON_KF_MAX_INPUTS` is worth adding
  for symmetry; otherwise untouched.
- `docs/specs/IS_ControlLib.rst` — only if implementation reveals the published
  `ron_kalman.h` block needs a documented correction (e.g. an added explicit
  Kalman-specific fault field); the API signatures should otherwise match.
- `docs/specs/TP_ControlLib.rst` — add detailed sections for the
  already-listed `RON-TC-KF-002` … `RON-TC-KF-005`, `RON-TC-KF-007`,
  `RON-TC-KF-008`, and `RON-TC-KF-008-FV` test IDs.
- `docs/plans/c/c11-rollout.md` — add a "Phase 6 Kalman Filter Opening
  Evidence" section.
- `docs/plans/c11-roadmap.md` — update the Phase 6 status from planned to
  complete and refresh the "Current Baseline" / public-surface summary.
- `CHANGELOG.rst` — add a Phase 6 entry.
- `docs/plans/c/c11-phase-6-kalman-filter.md` — this file: flip `Status` to
  `Complete` and fill in the completion update.

Files that must remain untouched unless Phase 6 proves otherwise:

- Public headers and sources for any later module (`ron_statespace`,
  `ron_observer`, `ron_autotune`, `ron_health`, `ron_metrics`).
- PID/filter/feed-forward/gain-scheduling/trajectory/cascade public APIs and
  behaviour.
- SRS and SADS, unless implementation discovers a real spec mismatch that must
  be corrected explicitly (record any such change in the completion update).

## Verification Plan

Minimum local evidence expected before Phase 6 can be marked complete (same set
used to close Phases 1–5):

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON`
- `cmake --build regulon-c/build --config Debug`
- `ctest --test-dir regulon-c/build -C Debug --output-on-failure`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps format,complexity,cppcheck`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps msvc,double,clang`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps coverage`
- `powershell -NoProfile -ExecutionPolicy Bypass -File regulon-c/scripts/verify_pid.ps1 -Steps probe,cross-arm,cross-arm-clang,cbmc`
- `clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over the new
  `regulon-c/test/formal/kalman_*_proof.c`
- `git diff --check`

Phase-specific pass criteria:

- `RON-TC-KF-001` through `RON-TC-KF-008` implemented and passing; the
  `RON-TC-KF-008-FV` harness present and discovered by the formal-proof
  tooling.
- Active C source coverage stays at 100% statement and branch coverage with
  `ron_kalman.c` added to the active set.
- PID, filter, feed-forward, gain-scheduling, trajectory, and cascade regression
  suites remain green; their public behaviour is unchanged.
- cppcheck/MISRA clean, lizard complexity `CCN <= 10` for `ron_kalman.c`,
  clang-format clean, MSVC + GCC ASan/UBSan + double-precision builds clean,
  standalone Clang build clean, ARM / ARMv7 cross-compile smoke builds pass
  (freestanding-header fallback acceptable on hosts without Newlib).

## Execution Order

1. Add detailed TP descriptions for `RON-TC-KF-002` … `RON-TC-KF-005`,
   `RON-TC-KF-007`, `RON-TC-KF-008`, and `RON-TC-KF-008-FV`.
2. Add the public `regulon-c/include/ron/ron_kalman.h` matching the IS block.
3. Write `regulon-c/test/unit/test_ron_kalman.c` with the `RON-TC-KF-001` …
   `RON-TC-KF-008` cases (red).
4. Implement `regulon-c/src/ron_kalman.c`: validation + bounded matrix
   primitives + bounded `sqrt` + Cholesky solve + predict + update + reset +
   get_state.
5. Wire `ron_kalman.c` into `regulon-c/CMakeLists.txt` and `test_ron_kalman`
   into `regulon-c/test/CMakeLists.txt`; build and run host tests green.
6. Add the `kalman_no_heap_proof.c` (and any further already-defined-ID
   harnesses) under `regulon-c/test/formal/`.
7. Extend `regulon-c/scripts/verify_pid.ps1` and `.github/workflows/ci_c.yml`
   active-source/header lists to include `ron_kalman`.
8. Run the full active C11 verification set (format, complexity, cppcheck,
   msvc/double/clang, coverage, cross-arm/cross-arm-clang, cbmc-discovery,
   `git diff --check`).
9. Update `docs/plans/c/c11-rollout.md`, `docs/plans/c11-roadmap.md`,
   `CHANGELOG.rst`, and this file with completion evidence and any deliberate
   design choices.

## Completion Update Instructions

When Phase 6 is complete, update this file in place with:

- `Status: Complete`
- final implemented file list
- exact local verification results
- any environment-limited gaps that remain CI-only (expected: local CBMC if
  `cbmc` is unavailable; local `arm-none-eabi-gcc` if unavailable)
- any deliberate deviations: the chosen fault taxonomy for non-PD innovation
  covariance, whether a shared internal math helper file was introduced, the
  bounded `sqrt`/Cholesky iteration bounds, and the symmetry-handling decision

At the same time update:

- `docs/plans/c/c11-rollout.md` with a "Phase 6 Kalman Filter Opening Evidence"
  section
- `docs/plans/c11-roadmap.md` Phase 6 status and the "Current Baseline" summary
- `CHANGELOG.rst`

## Completion Update

(To be filled in when Phase 6 closes.)

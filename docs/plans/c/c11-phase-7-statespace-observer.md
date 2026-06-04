# C11 Phase 7 State-Space Controller And Luenberger Observer Plan

Date: 2026-06-04 (planned), 2026-06-04 (closed)
Status: Complete

## Objective

Implement the C11 discrete-time state-feedback controller (`ron_statespace`)
and Luenberger observer (`ron_observer`) slices for `regulon-c` so Phase 7 of
`docs/plans/c11-roadmap.md` is vertically complete and closed with the same
evidence bar used for PID, filters, feed-forward, gain scheduling, trajectory
generators, the cascade controller, and the Kalman filter.

This file is the Phase 7 living record. It fixes scope and execution order and
is updated in place on completion with the implemented file list, verification
results, residual tool gaps, and deliberate design choices.

## Scope

Requirement scope:

- `RON-FR-700` through `RON-FR-704` (state-feedback controller)
- `RON-FR-720` through `RON-FR-723` (Luenberger observer)

Primary test scope:

- `RON-TC-SS-001` through `RON-TC-SS-009`
- `RON-TC-SS-004-FV` (CBMC saturation-bound + no-heap harness)

Out of scope: relay auto-tune, health monitor, metrics, or any later roadmap
module; new TP IDs not already present in `docs/specs/TP_ControlLib.rst`.

## Settled Design Decisions

- **State source binding:** `ron_statespace` embeds both a `ron_obs_t` and a
  `ron_kf_t`, realising the full SADS `EXTERNAL / LUENBERGER / KALMAN`
  selection. `ron_statespace.h` includes `ron_observer.h` and `ron_kalman.h`.
- **Shared math:** the bounded fixed-size matrix/vector primitives were
  factored out of `ron_kalman.c` into a new internal, non-public
  `ron_matrix` unit reused by Kalman, state-space, and observer.

## Implemented Files

Added:

- `regulon-c/include/ron/ron_observer.h`, `regulon-c/src/ron_observer.c`
- `regulon-c/include/ron/ron_statespace.h`, `regulon-c/src/ron_statespace.c`
- `regulon-c/src/ron_matrix_internal.h`, `regulon-c/src/ron_matrix.c`
- `regulon-c/test/unit/test_ron_observer.c`,
  `regulon-c/test/unit/test_ron_statespace.c`
- `regulon-c/test/formal/statespace_sat_proof.c` (`RON-TC-SS-004-FV`)

Modified:

- `regulon-c/src/ron_kalman.c` — refactored onto the shared `ron_matrix`
  helper (pure extraction; public API, numerics, and `RON-TC-KF-*` suite
  unchanged).
- `regulon-c/include/ron/ron_platform.h` — `RON_SS_MAX_*` minimum-bound
  static asserts; `RON_MAT_MAX_DIM` definition and coverage asserts;
  `RON_KF_MAX_MEASUREMENTS` / `RON_KF_MAX_INPUTS` `>= 1` asserts.
- `regulon-c/CMakeLists.txt`, `regulon-c/test/CMakeLists.txt` — new sources
  and test executables registered.
- `regulon-c/scripts/verify_pid.ps1`, `.github/workflows/ci_c.yml` — new
  sources / headers added to the format, cppcheck/MISRA, complexity,
  coverage, and CBMC sets; `-I .../src` added to the cppcheck and CBMC
  invocations so the internal helper header resolves.
- `docs/specs/IS_ControlLib.rst` — `ron_observer.h` / `ron_statespace.h` API
  blocks. `docs/specs/TP_ControlLib.rst` — detailed `RON-TC-SS-001`..`009`
  and `RON-TC-SS-004-FV` entries.
- `docs/plans/c11-roadmap.md`, `docs/plans/c/c11-rollout.md`,
  `CHANGELOG.rst` — Phase 7 status, opening evidence, and changelog entry.

## Algorithm Shape (from SADS)

State-space step (`RON-FR-700`..`703`): obtain `x_hat` from the selected
source; `u_fb = -K·x_hat`; if integral augmentation is enabled,
`e_reg = r - C_out·x_hat`, `integral += Ki_aug·dt·e_reg` clamped to
`[i_min, i_max]`, `u_raw = u_fb + Kr·r + integral` (else `u_raw = u_fb +
Kr·r`); saturate to `[u_min, u_max]`; rate-limit by `du_max` — identical to the
PID pipeline.

Luenberger step (`RON-FR-720`): `innovation = y - C·x_hat`;
`x_hat = A·x_hat + B·u + L·innovation`.

## Verification Results

Collected on 2026-06-04 (Linux host, GCC + Clang + Ninja + CMake + gcov):

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
  `cmake --build regulon-c/build`: pass, warning-free under
  `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
- `ctest --test-dir regulon-c/build --output-on-failure`: 11/11 suites pass,
  including new `test_ron_observer` / `test_ron_statespace` and the unchanged
  `test_ron_kalman`.
- Double-precision (`RON_USE_DOUBLE=ON`), standalone Clang/Ninja, and GCC
  `-fsanitize=address,undefined -fno-sanitize-recover=all` builds: 11/11 pass
  each.
- `clang-format --dry-run --Werror` over every new source/header and
  `clang -std=c11 -Wall -Wextra -Werror -fsyntax-only` over
  `statespace_sat_proof.c`: clean.
- `gcov -b`: 100% line and 100% branch coverage (both directions taken) on
  `ron_matrix.c`, `ron_observer.c`, `ron_statespace.c`, and `ron_kalman.c`.
- `git diff --check`: passes.

## Residual Tool Gaps

`cbmc`, `cppcheck`, `lizard`, `arm-none-eabi-gcc`, and the clang LLVM
coverage / ASan runtime libraries are not installed on this host, so the
`RON-TC-SS-004-FV` proof, MISRA static analysis, lizard complexity
(CCN <= 10), the LLVM `llvm-cov` 100% gate, and the ARM cross-compile smoke
build remain CI responsibilities. The dynamic `*_proof.c` discovery picks up
`statespace_sat_proof.c` automatically once `cbmc` is available.

## Deliberate Design Choices

- **Shared `ron_matrix` helper, no separate unit test ID.** The factored-out
  primitives are exercised — and held at 100% line/branch coverage — by the
  aggregate Kalman, observer, and state-space suites (which feed `+inf`,
  `-inf`, NaN, and finite values through every primitive). No `RON-TC-MAT-*`
  IDs were invented, honouring the roadmap rule that every implemented test
  uses a pre-existing TP ID.
- **Observer `dt` dropped.** The SADS pseudocode lists a `dt` argument, but
  the discrete observer recursion does not use a sample period, so
  `ron_obs_step` takes only `y` and `u`. Recorded as a deliberate
  spec-vs-implementation note.
- **Embedded-estimator advance API.** The embedded observer / Kalman are
  advanced via thin wrappers (`ron_ss_observer_step`,
  `ron_ss_kalman_predict`, `ron_ss_kalman_update`) that enforce the matching
  source; `ron_ss_step` then consumes the latest estimate. Cross-source calls
  return `RON_FAULT_CONFIG_INVALID`.
- **Output-NaN detection on the raw control.** Finiteness is checked on
  `u_raw` before limiting (an overflowing `K·x_hat` is reachable and yields
  `RON_FAULT_OUTPUT_NAN`); after clamping with finite bounds the output is
  necessarily finite, so a post-limit check would be dead code.
- **External-pointer null check deferred to step time.** `ron_ss_init` does
  not require `x_ext` at init; `ron_ss_step` returns `RON_FAULT_NULL_POINTER`
  if the external source pointer is null, keeping that guard reachable.
- **Scalar finiteness via the shared helper.** `ron_statespace.c` routes
  scalar finiteness through `ron_mat_vec_finite(&v, 1U)` (helper `ss_finite`)
  rather than inline `RON_ISFINITE`, so the unit carries no per-call-site
  macro-expansion branches and inherits the helper's full coverage.
- **Uniform `RON_MAT_MAX_DIM` stride.** Defined as
  `max(RON_KF_MAX_STATES, RON_SS_MAX_STATES)` so all three modules share one
  scratch-matrix stride, with static asserts proving every module bound fits.
- **No new `ron_fault_t` bits.** Reuses `RON_FAULT_NULL_POINTER`,
  `RON_FAULT_CONFIG_INVALID`, `RON_FAULT_INPUT_NAN`, `RON_FAULT_OUTPUT_NAN`,
  consistent with the rest of the library.

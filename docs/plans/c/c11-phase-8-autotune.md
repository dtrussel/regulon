# C11 Phase 8 Relay Feedback Auto-Tuning Plan

Date: 2026-06-05 (planned), 2026-06-05 (closed)
Status: Complete

## Objective

Implement the C11 relay-feedback PID auto-tuner (`ron_autotune`) slice for
`regulon-c` so Phase 8 of `docs/plans/c11-roadmap.md` is vertically complete and
closed with the same evidence bar used for PID, filters, feed-forward, gain
scheduling, trajectory generators, the cascade controller, the Kalman filter,
and the state-space / observer slice.

This file is the Phase 8 living record. It fixes scope and execution order and
is updated in place on completion with the implemented file list, verification
results, residual tool gaps, and deliberate design choices.

## Scope

Requirement scope:

- `RON-FR-800` through `RON-FR-807` (relay excitation, Ku/Tu estimation,
  tuning rules, staged apply, raw Ku/Tu, relay bound, abort).

Primary test scope:

- `RON-TC-AT-001` through `RON-TC-AT-008`
- `RON-TC-AT-007-FV` (CBMC relay-bound + no-heap harness)

Out of scope: the health monitor, runtime metrics, or any later roadmap module;
new TP IDs not already present in `docs/specs/TP_ControlLib.rst`.

## Settled Design Decisions

- **Standalone scalar module.** Relay excitation and zero-crossing period
  estimation are O(1)/step scalar math (SADS DD-15 — no FFT, no buffers), so
  `ron_autotune` pulls in no `ron_matrix` dependency and keeps all bookkeeping in
  fixed `ron_at_state_t` fields.
- **PID touched only at start/apply/abort.** `ron_autotune_step` is standalone
  (no PID argument). `start` snapshots gains + mode and parks the PID in manual;
  `apply` is the ONLY path that writes gains (`ron_pid_set_gains`, RON-FR-804);
  `abort` and timeout restore the snapshot (RON-FR-807).
- **Rule constants as lookup tables.** The four tuning rules are factor tables
  indexed by `ron_at_rule_t`, avoiding a `switch` default that would be dead
  code under 100% coverage.

## Implemented Files

Added:

- `regulon-c/include/ron/ron_autotune.h`, `regulon-c/src/ron_autotune.c`
- `regulon-c/test/unit/test_ron_autotune.c`
- `regulon-c/test/formal/autotune_relay_bound_proof.c` (`RON-TC-AT-007-FV`)

Modified:

- `regulon-c/CMakeLists.txt`, `regulon-c/test/CMakeLists.txt` — new source and
  `test_ron_autotune` executable registered (replacing the `ron_autotune.c`
  stub in the default build).
- `regulon-c/scripts/verify_pid.ps1`, `.github/workflows/ci_c.yml` —
  `ron_autotune.c` / `ron_autotune.h` added to the format, cppcheck/MISRA,
  complexity, coverage, and CBMC source sets.
- `docs/plans/c11-roadmap.md`, `docs/plans/c/c11-rollout.md`,
  `CHANGELOG.rst` — Phase 8 status, opening evidence, and changelog entry.

## Algorithm Shape (from SADS)

Lifecycle phases `RON_AT_IDLE → RON_AT_SETTLING → RON_AT_RELAY →
RON_AT_ESTIMATING → RON_AT_DONE`, with `RON_AT_ABORTED` reachable from
settling/relay on timeout or caller abort.

Relay law (`RON-FR-806`): `e = r - y`; `e > +eps ⇒ u = u_bias + d`;
`e < -eps ⇒ u = u_bias - d`; else hold the previous output. The result is
always in `[u_bias - d, u_bias + d]`.

Estimation (`RON-FR-802`): zero crossings of `e` bound half-periods; `Tu = 2 ×`
the average half-period; amplitude `A = (pv_max - pv_min) / 2`;
`Ku = 4d / (pi · A)`. Rules (`RON-FR-803`): `Kp = kp·Ku`, `Ti = ti·Tu`,
`Td = td·Tu`, with `Ki = Kp / Ti`, `Kd = Kp · Td`.

## Verification Results

Collected on 2026-06-05 (Linux host, GCC + Clang + CMake + gcov):

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
  `cmake --build regulon-c/build`: pass, warning-free under
  `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
- `ctest --test-dir regulon-c/build`: 12/12 suites pass, including the new
  `test_ron_autotune` (`RON-TC-AT-001`..`008`).
- Double-precision (`RON_USE_DOUBLE=ON`), standalone Clang, and GCC
  `-fsanitize=address,undefined -fno-sanitize-recover=all` builds: 12/12 pass
  each.
- `clang-format --dry-run --Werror` over every new source / header / test /
  harness, and `gcc -fsyntax-only` over `autotune_relay_bound_proof.c`: clean.
- `gcov -b` on `ron_autotune.c`: 100% line, 100% branch (both directions
  taken), 100% call coverage.
- `git diff --check`: passes.

## Residual Tool Gaps

`cbmc`, `cppcheck`, `lizard`, `arm-none-eabi-gcc`, and the clang LLVM
coverage / ASan runtime libraries are not installed on this host, so the
`RON-TC-AT-007-FV` proof, MISRA static analysis, lizard complexity (CCN <= 10),
the LLVM `llvm-cov` 100% gate, and the ARM cross-compile smoke build remain CI
responsibilities. The dynamic `*_proof.c` discovery picks up
`autotune_relay_bound_proof.c` automatically once `cbmc` is available.

## Deliberate Design Choices

- **`RON_AT_SETTLING` before `RON_AT_RELAY`.** A definite initial relay drive
  (`u_bias + d`) is applied, and period/peak measurement only begins after the
  first zero crossing, so transient start-up samples do not bias `Tu` / `A`.
- **`RON_AT_ESTIMATING` is a synchronous marker.** Estimation runs to
  `RON_AT_DONE` (or `RON_AT_ABORTED` on a sub-measurable oscillation) within the
  same `step`; the phase value is assigned for SADS fidelity but is not a
  dwelling state.
- **Insufficient-excitation guard.** If the measured half-amplitude is below
  `RON_AT_MIN_AMPLITUDE` the run aborts rather than dividing through a
  near-zero amplitude, keeping `Ku` finite.
- **`step` returns faults only for misuse.** Null pointers, an un-started
  instance, a non-positive / non-finite `dt`, and non-finite `r` / `y` return
  `RON_FAULT_*`; timeout and abort are reported through `state.aborted` /
  `state.phase`, not the return code.
- **Public `ron_at_phase_t` convenience enum.** The IS specifies `phase` as a
  `uint8_t`; the header adds a named enum for the SADS phase values and
  documents the remaining `ron_at_state_t` fields (which the IS abbreviates) as
  opaque oscillation-tracking bookkeeping. No public function or struct field
  named in the IS was altered.
- **No new `ron_fault_t` bits.** Reuses `RON_FAULT_NULL_POINTER` and
  `RON_FAULT_CONFIG_INVALID`, consistent with the rest of the library.
- **Deterministic test fixtures.** `RON-TC-AT-003` / `-004` inject an open-loop
  synthetic sine (`A = 0.5/pi`, `Tu = 0.5 s` → `Ku = 4.0`) so the estimator is
  checked against exact reference values; `RON-TC-AT-001` closes the loop around
  a first-order plant to confirm the relay excites a real limit cycle.

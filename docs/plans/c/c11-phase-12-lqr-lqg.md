# C11 Phase 12 LQR and LQG Optimal Control Plan

Date: 2026-08-07 (planned), 2026-08-07 (closed)
Status: Complete

## Objective

Implement the C11 discrete-time MIMO LQR (`ron_lqr`) and LQG (`ron_lqg`)
controller modules for `regulon-c` so Phase 12 of `docs/plans/c11-roadmap.md`
is vertically complete and closed with the same evidence bar used for every
prior phase. The public headers, CMake plumbing, and test catalog were
already committed in the preceding spec-only phase ("Spec Phase: LQR and LQG
Controller Support"); this phase implements the `.c` sources, tests, and
formal harnesses, and flips the modules on by default. This closes the C11
roadmap — every module across Phases 0–12 is now implemented.

## Scope

Requirement scope:

- `RON-FR-730` through `RON-FR-739` (LQR: DARE solver, pre-computed/DARE
  gain modes, three-source state estimate, integral augmentation,
  per-input output limiting, runtime gain update, DARE-solution accessor).
- `RON-FR-750` through `RON-FR-759` (LQG: dual-DARE init, separation
  principle, predict/update delegation, combined control step, output
  limiting, state accessor).

Primary test scope:

- `RON-TC-LQR-001` through `RON-TC-LQR-009` and `RON-TC-LQR-010-FV`
  (already catalogued in `docs/specs/TP_ControlLib.rst`).
- `RON-TC-LQG-001` through `RON-TC-LQG-009` and `RON-TC-LQG-010-FV`.

Out of scope: the Rust ports (tracked separately under
`docs/plans/rust/rust-first-rollout.md`) and any change to the LQR/LQG
public API fixed by the prior spec phase.

## Design Notes

- **Shared DARE solver.** `ron_lqr_dare_solve` (iterative value recursion
  per SADS DD-19 — no Schur decomposition) is defined once in `ron_lqr.c`
  and declared in the private `ron_lqr_internal.h` so `ron_lqg.c` reuses it
  for the LQR half of the combined gain, rather than duplicating the
  Riccati iteration.
- **CCN discipline.** The DARE solver, config validation, and step paths
  are decomposed into small `static` helpers (mirroring `ron_kalman.c` /
  `ron_statespace.c`), keeping every function at or below CCN 10
  (`lizard -C 10`: no warnings).
- **Delegated LQG embedded-Kalman validation.** `ron_lqg`'s own config
  validation checks only `A` and `B` up front, since those feed the shared
  DARE solver directly. The Kalman-only fields (`H`, `Q_noise`, `R_noise`,
  `x0`, `P0`, `K_f_inf`) are left to the embedded `ron_kf_init()` call's own
  validation — the same delegation pattern `ron_statespace`/`ron_observer`
  already use for their embedded estimators, avoiding redundant checks and
  a second source of truth for the same fields.
- **Kr always caller-supplied.** In both modules the reference pre-gain
  `Kr` is a design parameter set directly in the config and is never
  computed by DARE (DARE only solves the state-feedback gain `K`); `Kr` is
  therefore validated as finite unconditionally, while `K` is validated
  only in `PRECOMPUTED` mode (it is overwritten before first use in `DARE`
  mode).
- **`dare_tol` in practice.** The DARE convergence check in the unit tests
  uses `1e-4`, not the illustrative `1e-8` in the TP preconditions text:
  under the library's default single-precision build, `1e-8` is below the
  float ULP at the P-matrix magnitudes involved and the iteration
  oscillates in floating-point noise without ever satisfying it (confirmed
  empirically). `1e-4` converges within ~8 iterations and the tests still
  check the documented pass criterion (`K_solved` within `1e-4` of the
  analytically known optimal gain).

## Implemented Files

Added:

- `regulon-c/src/ron_lqr.c`, `regulon-c/src/ron_lqg.c`.
- `regulon-c/src/ron_lqr_internal.h` (private, shared DARE solver
  declaration).
- `regulon-c/test/unit/test_ron_lqr.c` (`RON-TC-LQR-001`..`009` plus
  defensive/validation coverage).
- `regulon-c/test/unit/test_ron_lqg.c` (`RON-TC-LQG-001`..`009` plus
  defensive/validation coverage).
- `regulon-c/test/formal/lqr_saturation_proof.c` (`RON-TC-LQR-010-FV`).
- `regulon-c/test/formal/lqg_no_heap_proof.c` (`RON-TC-LQG-010-FV`).

Modified:

- `regulon-c/cmake/ron_options.cmake` — `RON_ENABLE_LQR`/`RON_ENABLE_LQG`
  flipped to default `ON`.
- `regulon-c/test/CMakeLists.txt` — registered `test_ron_lqr` /
  `test_ron_lqg`.
- `regulon-c/scripts/lib_sources.txt`, `regulon-c/scripts/format_files.txt`
  — added `ron_lqr.c`, `ron_lqg.c`, `ron_lqr_internal.h`.
- `docs/plans/c11-roadmap.md`, `CHANGELOG.rst` — Phase 12 closure entries.

## Verification Results

Collected on 2026-08-07 (Linux host, GCC 13 + Clang + CMake + gcov + cppcheck
2.13 + lizard + CBMC 5.95 + `arm-none-eabi-gcc` 13.2, all installed locally —
no residual tool gaps for this phase):

- `cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON` and
  `cmake --build regulon-c/build`: pass, warning-free under
  `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow -Wundef`.
- `ctest --test-dir regulon-c/build`: 19/19 suites pass, including the new
  `test_ron_lqr` and `test_ron_lqg`.
- Double-precision (`RON_USE_DOUBLE=ON`), standalone Clang, and GCC
  `-fsanitize=address,undefined -fno-sanitize-recover=all` builds: 19/19
  suites pass on each configuration.
- `clang-format --dry-run --Werror` over the format manifest: clean.
- `cppcheck --addon=misra.py --check-level=exhaustive` over
  `scripts/lib_sources.txt`: zero findings in `ron_lqr.c`/`ron_lqg.c` (the
  run surfaces pre-existing, unrelated findings in `ron_autotune.c`/`.h`
  from this host's cppcheck/MISRA-addon version — not part of this phase).
- `lizard -C 10` over `scripts/lib_sources.txt`: no function exceeds CCN 10
  (`ron_lqr.c` max CCN 9, `ron_lqg.c` max CCN 7 after the shared-DARE and
  guard-clause decomposition).
- `gcov -b` (GCC `--coverage`) on `ron_lqr.c` and `ron_lqg.c`: 100% line
  and 100% branch coverage, both directions taken on every branch.
- CBMC (`--unwind 65 --unwinding-assertions --bounds-check
  --pointer-check`) on both new harnesses: **VERIFICATION SUCCESSFUL**,
  0 properties failed. Both proofs use `RON_LQR_GAIN_PRECOMPUTED` /
  `RON_LQG_GAIN_PRECOMPUTED` so the property (output-bounded, no heap)
  is proved on `*_step` directly without unwinding the bounded-iteration
  DARE solver, which is out of scope for these harnesses.
- ARM Cortex-M cross-compile (`arm-none-eabi-gcc`, real Newlib headers via
  `-DRON_ARM_GCC_NEWLIB_INCLUDE=/usr/include/newlib
  -DRON_ARM_GCC_ALLOW_HEADER_SHIM=OFF`, `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16
  -mfloat-abi=hard -mthumb`): full library including `ron_lqr.c`/
  `ron_lqg.c` builds cleanly.
- Minimal-subset build (all `RON_ENABLE_*` OFF) and full-library
  `RON_BUILD_EXAMPLES=ON` build: both pass; existing examples run
  unaffected by the newly-active LQR/LQG sources.
- `bash regulon-c/scripts/check_manifest.sh`: manifests in sync.

## Residual Tool Gaps

None for this phase — cppcheck, lizard, CBMC, and the ARM GCC toolchain
were all available and exercised locally in this environment.

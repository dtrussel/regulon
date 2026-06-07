# C11 Phase 11 Full-Library Integration And Release Hardening Plan

Date: 2026-06-07 (planned), 2026-06-07 (closed)
Status: Complete

## Objective

Close Phase 11 of `docs/plans/c11-roadmap.md` by hardening the `regulon-c`
library for release. Phases 0–10 already added every module source to the
default build, so the literal "enable all sources" step was already done
incrementally; Phase 11 instead delivers full-library integration validation,
build/CI release hardening, optional per-module selection for constrained
targets, and a traceability / spec-consistency audit.

This file is the Phase 11 living record: it fixes scope and execution order and
is updated in place on completion with the implemented file list, verification
results, residual tool gaps, and deliberate design choices.

## Scope

- Aggregate header `regulon-c/include/ron/ron.h` including every public header in
  dependency order, guarded by `RON_HAVE_<MODULE>` macros.
- Per-module CMake options `RON_ENABLE_<MODULE>` (default ON) with a generated
  `ron/ron_modules.h`; PID core + integrated feed-forward are the mandatory
  baseline; `RON_ENABLE_STATESPACE` forces `RON_ENABLE_KALMAN`.
- Integration suite `RON-TC-INT-001` .. `RON-TC-INT-005` (registered in
  `TP_ControlLib.rst` first, per the TP-first rule).
- Host examples gated by `RON_BUILD_EXAMPLES`.
- Centralized CI source manifests + drift check, replacing the five parallel
  hard-coded lists in `ci_c.yml` (and the duplicates in `verify_pid.ps1`).
- Traceability / spec-consistency audit: TP matrices, CBMC harness inventory,
  `IS_ControlLib.rst` vs headers, `SADS_ControlLib.rst` vs modules.

Out of scope: the Rust port (tracked under `docs/plans/rust/`), any new
controller algorithm, and any change to the authoritative requirement set.

## Settled Design Decisions

- **Feed-forward is part of the baseline, not an optional module.**
  `ron_pid_config.c` calls `ron_feedforward_config_validate()`, so the
  feed-forward source is always compiled; `RON_HAVE_FEEDFORWARD` is always 1.
- **State-space depends on Kalman.** `ron_statespace.c` calls `ron_kf_*`, so
  `RON_ENABLE_STATESPACE` force-enables `RON_ENABLE_KALMAN`; Kalman / state-space
  / observer pull in the internal `ron_matrix.c`.
- **Generated header named `ron_modules.h`, not `ron_config.h`.** Avoids a
  collision with the optional user-supplied `ron_config.h` platform override
  referenced (in a comment) by `ron_platform.h`.
- **Aggregate header adds no symbols** and is header-only: it is not in the
  library source list nor any production gate (cppcheck/lizard/coverage/cbmc);
  it is in the clang-format gate.
- **Integration tests only add covered paths** and never enter the coverage
  `-sources` list, so the 100/100 line/branch gate is unaffected.
- **Examples are host-only** (use `printf`/`<stdio.h>`), excluded from every
  production gate and from the cross-compile builds; built on GCC + Clang under
  `-Werror` in CI to catch public-API and warning regressions.
- **Manifest as the single source of truth.** `scripts/lib_sources.txt` and
  `scripts/format_files.txt` feed every gate; `scripts/check_manifest.sh` fails
  if a production source/header is missing from a manifest, converting the
  "silently dropped file" failure mode into a hard error.
- **Subset builds keep their own test set.** Unit-test registration is guarded
  by the same `RON_ENABLE_*` options; the integration suite registers only in
  the full default build. The full build remains the 100/100 coverage gate; the
  PID-only build is a compile/link smoke check.

## Implemented Files

Added:

- `regulon-c/include/ron/ron.h` (aggregate header).
- `regulon-c/cmake/ron_modules.h.in` (generated → `ron/ron_modules.h`).
- `regulon-c/test/integration/test_ron_integration.c` (`RON-TC-INT-001`..`005`).
- `regulon-c/examples/pid_quickstart.c`, `regulon-c/examples/cascade_control_loop.c`,
  `regulon-c/examples/CMakeLists.txt`.
- `regulon-c/scripts/lib_sources.txt`, `regulon-c/scripts/format_files.txt`,
  `regulon-c/scripts/check_manifest.sh`.
- `docs/plans/c/c11-phase-11-integration.md` (this file).

Modified:

- `regulon-c/cmake/ron_options.cmake` — `RON_BUILD_EXAMPLES` + `RON_ENABLE_*`.
- `regulon-c/CMakeLists.txt` — dependency resolution, conditional source list,
  `configure_file` for `ron_modules.h`, generated include dir + install, examples
  subdirectory.
- `regulon-c/test/CMakeLists.txt` — per-module guards + integration suite.
- `.github/workflows/ci_c.yml` — manifest-driven format/cppcheck/lizard/coverage/
  cbmc lists; new `manifest-check`, `build-subset`, and `build-examples` jobs.
- `regulon-c/scripts/verify_pid.ps1` — read the manifests instead of inline lists.
- `docs/specs/TP_ControlLib.rst` — `INT` module code, `RON-TC-INT-001`..`005`
  blocks, both traceability matrices, CBMC harness inventory, OI-TP-06.
- `docs/specs/IS_ControlLib.rst` — `ron.h` / `ron_modules.h`, header-inclusion
  model, directory layout, CMake module options.
- `docs/specs/SADS_ControlLib.rst` — aggregate header + build-time module
  selection.
- `docs/plans/c11-roadmap.md`, `docs/plans/c/c11-rollout.md`, `CHANGELOG.rst`.

## Audit Findings (closed)

The manifest drift check and CBMC inventory audit surfaced and closed real gaps:

- `ron_matrix_internal.h` was absent from the CI clang-format list — now added.
- `verify_pid.ps1` omitted `ron_matrix.c` / `ron_metrics.c` / `ron_observer.c` /
  `ron_statespace.c` from its CBMC source set — now manifest-driven.
- The Formal Verification Summary lists `RON-TC-CASC-004-FV` but no
  `cascade_*_proof.c` harness exists; recorded as open item OI-TP-06 (interim
  coverage via `pid_backcalc_proof.c`).

## Verification Results

Collected on 2026-06-07 (Linux host, GCC 13 + Clang 18 + CMake 3.28 + gcov):

- Full default build (GCC): `ctest` 15/15 suites pass, including
  `test_ron_integration` (`RON-TC-INT-001`..`005`).
- Clang full build: 15/15. Double-precision (`RON_USE_DOUBLE=ON`): 15/15.
- GCC `-fsanitize=address,undefined -fno-sanitize-recover=all`: 15/15.
- Minimal-subset build (all `RON_ENABLE_*` OFF, tests off): PID-only library
  compiles and links (5 baseline objects).
- Partial subset (filter+health+metrics on): 7/7 suites; integration suite
  correctly not registered.
- `RON_ENABLE_STATESPACE=ON RON_ENABLE_KALMAN=OFF`: Kalman force-enabled;
  archive contains matrix/kalman/statespace/observer objects.
- Single-include compile of `ron/ron.h` under strict flags: clean.
- `RON_BUILD_EXAMPLES=ON` (GCC): both examples build and run; tracking to target.
- `clang-format --dry-run --Werror` over the format manifest (incl. `ron.h`):
  clean. `bash scripts/check_manifest.sh`: manifests in sync.
- YAML lint of `ci_c.yml`: valid.

## Residual Tool Gaps

As in Phases 6–10, this verification host lacks `cbmc`, `cppcheck`, `lizard`,
`arm-none-eabi-gcc`, and the clang LLVM coverage / ASan runtime libraries, so the
CBMC proofs, MISRA static analysis, lizard complexity (CCN ≤ 10), the LLVM
`llvm-cov` 100% gate, and the ARM cross-compile smoke builds remain CI
responsibilities. The new `manifest-check`, `build-subset`, and `build-examples`
jobs run on stock Ubuntu runners with no extra tooling.

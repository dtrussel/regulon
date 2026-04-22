.. ============================================================
.. CHANGELOG — Regulon Control Systems Library
.. ============================================================

CHANGELOG
=========

All notable changes to the **Regulon Control Systems Library** are documented
here following `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
conventions.  Version numbers follow `Semantic Versioning <https://semver.org/>`_.

------------------------------------------------------------------------

Unreleased — Rust-First PID Kickoff
-----------------------------------

Added
~~~~~
- ``regulon-rs/Cargo.toml`` and ``regulon-rs/.cargo/config.toml``: Rust
  workspace scaffold and cross-target build configuration for the new
  Rust-first implementation track.

- ``regulon-rs/regulon/Cargo.toml`` and ``regulon-rs/regulon/src/``:
  ``#![no_std]`` Rust crate with ``platform``, ``error``, and ``pid``
  modules implementing the project kickoff PID baseline.

- Rust PID public API centered on ``Pid`` and ``PidConfig`` with runtime
  state encapsulation, configuration validation, manual/automatic mode
  switching, normalization, saturation, rate limiting, anti-windup, fault
  latching, and state snapshots.

- Traceable Rust unit tests and Kani proof scaffolding for the PID and
  safety baseline using ``RON-TC-PID-*`` and ``RON-TC-SAFE-*`` identifiers.

- ``docs/plans/rust/rust-first-rollout.md``: rollout plan/status record for
  the Rust-first implementation, iteration sequence, and current completion
  state.

- ``regulon-rs/regulon/src/filter/``: first reusable filter slice with a
  first-order low-pass filter and standalone asymmetric rate limiter plus
  traceable Rust tests.

- Static-gain feed-forward support in the Rust PID module, including a
  bounded feed-forward configuration surface, feed-forward diagnostics in
  PID state/status, and traceable Rust tests for ``RON-TC-FF-002`` and
  ``RON-TC-FF-008``.

------------------------------------------------------------------------

Unreleased C11 PID Vertical Slice
------------------------------------

Added
~~~~~
- ``regulon-c/include/ron/ron_filter.h`` and ``regulon-c/src/ron_filter.c``:
  added the complete C11 signal-conditioning filter slice with LP1,
  moving-average FIR, cascaded biquad IIR, coefficient helpers, notch
  hot-swap, and asymmetric rate-limiter APIs.

- ``regulon-c/test/unit/test_ron_filter.c``: added traceable Unity tests for
  ``RON-TC-FILT-001`` through ``RON-TC-FILT-017``.

- ``regulon-c/test/formal/filter_*_proof.c``: added focused CBMC harnesses
  for filter null-pointer handling, bounded array access, bounded execution,
  and no-heap evidence.

- ``regulon-c/src/ron_pid_api.c``, ``regulon-c/src/ron_pid_core.c``,
  ``regulon-c/src/ron_pid_fault.c``, and ``regulon-c/src/ron_pid_internal.h``:
  implemented the active C11 PID slice behind the frozen public API.

- ``regulon-c/test/unit/test_ron_pid_core.c``,
  ``regulon-c/test/unit/test_ron_pid_api.c``, and
  ``regulon-c/test/unit/test_ron_pid_common.h``: added traceable PID and
  safety unit suites for the kickoff sprints.

- ``regulon-c/test/formal/*.c``: added the first PID-focused CBMC harness set
  for saturation, back-calculation, integral clamp, multi-instance
  independence, and null-pointer validation.

- ``docs/plans/c/c11-rollout.md``: C-track rollout/status note with local
  evidence and remaining verification gaps.

- ``regulon-c/scripts/verify_pid.ps1``: repo-owned Windows verification
  entrypoint for the active PID slice, including tool probing, MSVC builds,
  double-precision regression, formatting, cppcheck, and complexity checks.

- Additional C CBMC harnesses for PID verification closure, covering API
  validation, denominator guards, no-heap/no-recursion/no-blocking claims,
  array-bounds smoke coverage, NaN/Inf fault handling, integral overflow,
  bounded step execution, and no global-state dependency.

Changed
~~~~~~~
- ``regulon-c/CMakeLists.txt`` and ``regulon-c/test/CMakeLists.txt``: enabled
  the Phase 1 filter source and unit suite in the active C11 build without
  enabling later placeholder modules.

- ``regulon-c/scripts/verify_pid.ps1`` and ``.github/workflows/ci_c.yml``:
  extended active-source verification from PID-only to PID plus filters for
  format, static analysis, complexity, coverage, and CBMC source lists.

- ``docs/specs/IS_ControlLib.rst`` and ``docs/specs/TP_ControlLib.rst``:
  reconciled the filter API with ``get_state`` operations, shared
  fault/status types, band-pass coefficient generation, and detailed
  ``RON-TC-FILT-*`` test scenarios.

- ``docs/plans/c/c11-rollout.md`` and ``docs/plans/c11-roadmap.md``:
  recorded Phase 0 PID closure acceptance and Phase 1 filter completion, with
  Phase 2 feed-forward identified as the next C11 roadmap item.

- ``regulon-c/CMakeLists.txt``: keeps the active C build limited to accepted
  slices, now PID plus filters, so later placeholder modules do not pollute
  host builds and quality evidence.

- ``regulon-c/test/CMakeLists.txt``: fixed the Windows/MSVC host-test build by
  making ``m`` linkage conditional and enabling the PID core/API suites.

- ``docs/specs/SRS_ControlLib.rst``, ``docs/specs/IS_ControlLib.rst``, and
  ``docs/specs/TP_ControlLib.rst``: reconciled the PID API summary and updated
  stale ``c/`` path references to ``regulon-c/`` where the C11 track is
  described concretely.

- ``regulon-c/src/ron_pid_config.c`` and ``regulon-c/src/ron_pid_core.c``:
  refactored the active PID validation and step pipeline to keep cyclomatic
  complexity within the ``RON-QR-011`` limit.

- ``regulon-c/test/unit/test_ron_pid_api.c``: tightened the PID verification
  slice with open-loop anti-windup recovery contrast, expanded null-pointer
  checks, fault-register coverage, safe-output clamping, and deterministic
  reproducibility testing.

- ``regulon-c/scripts/verify_pid.ps1``: added Clang/LLVM coverage
  enforcement, dynamic CBMC harness discovery, CBMC unwinding assertions,
  and cleaner skip reporting for unavailable Clang, coverage, cross, and
  formal tools.

- ``.github/workflows/ci_c.yml``: now enforces 100% statement and branch
  coverage with Clang/LLVM coverage, uploads raw/rendered coverage plus CBMC
  artifacts, and adds the ARM Cortex-M cross-compile smoke build for the
  active PID slice.

- ``docs/specs/TP_ControlLib.rst`` and ``docs/plans/c/c11-rollout.md``:
  updated the C verification-closure claims, evidence, and remaining local
  tool gaps.

- ``docs/deviations/MISRA_C_deviations.rst``: recorded approved deviations and
  observations for the active PID verification closure.

------------------------------------------------------------------------

Unreleased — Sprint 1 (C11 Platform + Type Headers)
----------------------------------------------------

Added
~~~~~
- ``regulon-c/include/ron/ron_platform.h``: Platform portability layer —
  ``ron_float_t``, ``RON_FLOAT_*`` macros, ``RON_STATIC_ASSERT``,
  ``RON_ASSERT``, ``RON_ISNAN``/``RON_ISINF``/``RON_ISFINITE``,
  ``ron_clamp()``, ``ron_fabs()``, dimension constants for all modules.
  Satisfies: RON-PR-010, RON-PR-011, RON-DC-001/002.

- ``regulon-c/include/ron/ron_pid_types.h``: All public enumerations
  (``ron_aw_mode_t``, ``ron_deriv_mode_t``, ``ron_op_mode_t``,
  ``ron_safe_policy_t``, ``ron_integ_method_t``), fault/status bitmask
  types, ``ron_pid_config_t``, ``ron_pid_state_t``, ``ron_pid_instance_t``.
  Compile-time size assertions included.
  Satisfies: RON-FR-001–071 (declarations), RON-SR-010–013, RON-PR-021.

- ``regulon-c/include/ron/ron_pid.h``: Complete public API declarations
  (lifecycle, runtime, configuration update, state inspection, fault management).
  Satisfies: RON-FR-050–071, RON-SR-001/002/012.

- ``regulon-c/test/unit/test_ron_pid_types.c``: Unity test suite covering
  RON-TC-PERF-005, RON-TC-PERF-006, RON-TC-QUAL-005, RON-TC-QUAL-007.

------------------------------------------------------------------------

[Sprint 0] 2026-04-11 — C11 Build Scaffold
-------------------------------------------

Added
~~~~~
- ``regulon-c/CMakeLists.txt``: Main CMake 3.21+ build file for the C11
  implementation track.  Builds the ``regulon`` static library.
  Satisfies: RON-QR-004, RON-DC-005.

- ``regulon-c/cmake/ron_options.cmake``: Build options
  ``RON_USE_DOUBLE``, ``RON_BUILD_TESTS``, ``RON_ENABLE_ASSERT``.

- ``regulon-c/cmake/toolchains/arm-none-eabi.cmake``: ARM Cortex-M
  cross-compile toolchain file.

- ``regulon-c/cmake/toolchains/riscv32-unknown-elf.cmake``: RISC-V 32-bit
  cross-compile toolchain file.

- ``regulon-c/cmake/toolchains/host-x86_64.cmake``: Host x86-64 toolchain
  reference file.

- ``regulon-c/test/CMakeLists.txt``: Test sub-directory build.  Registers
  Unity-based test executables with CTest.

- ``regulon-c/test/framework/unity/``: Vendored Unity v2.6.0 (MIT) — the
  lightweight C unit test framework used for all ``UT`` and ``IT`` test
  levels.

- ``.clang-format``: Project-wide code formatting configuration.
  Satisfies: RON-SR-030.

- ``.github/workflows/ci_c.yml``: GitHub Actions CI pipeline — build,
  test, static analysis (cppcheck), coverage (lcov), and artefact upload.
  Satisfies: RON-QR-004, RON-TC-QUAL-008.

- ``docs/deviations/MISRA_C_deviations.rst``: MISRA C:2023 deviation
  records (initially empty — no deviations at baseline).

- ``CHANGELOG.rst``: This file.

------------------------------------------------------------------------

Specification Baseline — 2026-04-11
-------------------------------------

Added
~~~~~
- ``docs/specs/SRS_ControlLib.rst`` v1.1.0 — Software Requirements
  Specification (RON-SRS-001).
- ``docs/specs/SADS_ControlLib.rst`` v1.1.0 — Software Architecture and
  Design Specification (RON-SADS-001).
- ``docs/specs/IS_ControlLib.rst`` v1.1.0 — Implementation Specification
  (RON-IS-001).
- ``docs/specs/TP_ControlLib.rst`` v1.0.0 — Test Plan and Specification
  (RON-TP-001).
- ``AGENTS.md`` — Developer guidelines and coding standards.
- ``LICENSE`` — MIT License.

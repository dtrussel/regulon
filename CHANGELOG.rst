.. ============================================================
.. CHANGELOG — Regulon Control Systems Library
.. ============================================================

CHANGELOG
=========

All notable changes to the **Regulon Control Systems Library** are documented
here following `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
conventions.  Version numbers follow `Semantic Versioning <https://semver.org/>`_.

------------------------------------------------------------------------

Unreleased — Sprint 2 (PID Core Algorithm + Fault Manager)
-----------------------------------------------------------

Added
~~~~~
- ``regulon-c/src/ron_pid_internal.h``: Private (intra-library) header
  declaring ``ron_pid_core_step``, ``ron_pid_fault_set``, and
  ``ron_pid_fault_safe_output``.  Not installed with the public API.

- ``regulon-c/src/ron_pid_core.c``: Full 17-stage PID computation pipeline
  per SADS §"Module Design: ron_pid_core" — input NaN/Inf guard,
  manual-mode passthrough, input normalisation, setpoint filter,
  setpoint-change integral reset, 2DOF error signals, proportional term,
  derivative (DoE/DoM with Tustin-discretised LP filter), integral
  (Euler/trapezoidal, back-calculation or conditional-clamping AW, hard
  clamp), denormalisation, output NaN/Inf guard, saturation, rate limit,
  integral overflow guard, state writeback, AW-active status bit.
  Satisfies: RON-FR-001–007, RON-FR-010–013, RON-FR-020–022, RON-FR-025–027,
  RON-FR-030–035, RON-FR-054, RON-FR-070.

- ``regulon-c/src/ron_pid_fault.c``: Fault bit-latch manager.
  ``ron_pid_fault_set`` ORs new fault bits into the sticky register,
  asserts ``RON_STATUS_FAULT``, and invokes the optional user callback
  once per newly-set bit (previously-latched bits do NOT re-fire).
  ``ron_pid_fault_safe_output`` resolves the configured safe-state policy
  (HOLD_LAST / ZERO / CONSTANT) with ``safe_value`` clamped to
  ``[u_min, u_max]``.  Satisfies: RON-SR-010, RON-SR-011, RON-SR-013.

- ``regulon-c/test/unit/test_ron_pid_core.c``: 14 Unity test cases —
  RON-TC-PID-001 parallel form, RON-TC-PID-002 ISA equivalence,
  RON-TC-PID-003 P-only, RON-TC-PID-004/005 Euler/trapezoidal integration,
  RON-TC-PID-006/007 DoE/DoM derivative kick,
  RON-TC-PID-008/009 derivative filter on/off,
  RON-TC-PID-010 2DOF weights, RON-TC-PID-011/012/013/014 input
  normalisation, scaling, output denormalisation, raw-domain bypass.

- ``regulon-c/test/unit/test_ron_pid_fault.c``: 5 Unity test cases —
  RON-TC-SAFE-007 callback single-shot, RON-TC-SAFE-008 safe-state
  policies, RON-TC-SAFE-009 ``ron_pid_fault_clear``,
  RON-TC-SAFE-011 NaN/Inf and invalid-``dt`` guards,
  RON-TC-SAFE-013 integral overflow threshold trip.

Changed
~~~~~~~
- ``regulon-c/src/ron_pid_api.c``: ``ron_pid_step`` no longer ships the
  P-only stub.  After the NULL/init/fault-latch guards and a new
  ``dt`` finiteness/positivity check (header precondition at
  ``ron_pid.h:131``), execution delegates to ``ron_pid_core_step``.
  Safe-state resolution moved into ``ron_pid_fault_safe_output``.

- ``regulon-c/test/CMakeLists.txt``: Enabled the ``test_ron_pid_core`` and
  ``test_ron_pid_fault`` Sprint 2 suites.

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

.. ============================================================
.. Test Plan and Specification
.. Regulon Control Systems Library
.. ============================================================

.. meta::
   :description: Test Plan and Specification for the Regulon Control Systems Library.
   :keywords: regulon, PID, control systems, test plan, test specification, traceability

########################################################################
Test Plan and Specification
########################################################################

**Document Title:** Test Plan and Specification — Regulon Control Systems Library

**Document ID:** RON-TP-001

**Version:** 1.0.0

**Status:** Draft

**Date:** 2025-04-10

.. contents:: Table of Contents
   :depth: 3
   :local:
   :backlinks: none

.. sectnum::

------------------------------------------------------------------------

Revision History
================

.. list-table::
   :header-rows: 1
   :widths: 10 10 60 20

   * - Version
     - Date
     - Description
     - Author
   * - 1.0.0
     - 2025-04-10
     - Initial baseline. Covers all RON-SRS-001 v1.1.0 requirements.
     - TBD

------------------------------------------------------------------------

Introduction
============

Purpose
-------

This document specifies the test strategy, test cases, and requirement
traceability for the **Regulon Control Systems Library**. It ensures that
every requirement in RON-SRS-001 v1.1.0 is verified by at least one test
case, and provides the test IDs that are embedded directly into C and Rust
unit test source code to enable automated, traceable test reports in CI.

Scope
-----

This document covers:

- Test strategy: levels, environments, tools, CI integration.
- A complete **Requirement → Test** traceability matrix.
- A complete **Test → Requirement** traceability matrix.
- Detailed test case specifications for every RON-SRS-001 requirement.
- Code embedding conventions for test IDs (both C and Rust).
- Acceptance criteria and coverage targets.

Both implementation tracks (C11 and Rust Edition 2021) share the same test
IDs and test case specifications; the only difference is the test framework
and syntax used to implement each case.

Parent Documents
----------------

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Document ID
     - Title
   * - RON-SRS-001 v1.1.0
     - Software Requirements Specification
   * - RON-SADS-001 v1.1.0
     - Software Architecture and Design Specification
   * - RON-IS-001 v1.1.0
     - Implementation Specification

References
----------

.. list-table::
   :header-rows: 1
   :widths: 10 90

   * - ID
     - Reference
   * - [REF-01]
     - Unity Test Framework (MIT). https://github.com/ThrowTheSwitch/Unity
   * - [REF-02]
     - ``cargo test`` documentation. https://doc.rust-lang.org/cargo/commands/cargo-test.html
   * - [REF-03]
     - ``cargo-llvm-cov`` (Apache 2.0). https://github.com/taiki-e/cargo-llvm-cov
   * - [REF-04]
     - ``gcov``/``lcov`` documentation. https://gcc.gnu.org/onlinedocs/gcc/Gcov.html
   * - [REF-05]
     - Kani Rust Verifier (Apache 2.0). https://model-checking.github.io/kani/
   * - [REF-06]
     - CBMC C Bounded Model Checker (BSD-4-Clause). https://github.com/diffblue/cbmc
   * - [REF-07]
     - JUnit XML format specification. https://www.ibm.com/docs/en/developer-for-zos/14.1?topic=formats-junit-xml-format
   * - [REF-08]
     - GitHub Actions documentation. https://docs.github.com/en/actions
   * - [REF-09]
     - ``proptest`` property-based testing (MIT). https://github.com/proptest-rs/proptest

------------------------------------------------------------------------

Test Strategy
=============

Test Levels
-----------

.. list-table::
   :header-rows: 1
   :widths: 12 18 70

   * - Level Code
     - Level Name
     - Description
   * - ``UT``
     - Unit Test
     - Tests a single module function or operation in isolation. All
       dependencies outside the unit under test are either absent (pure
       computation) or replaced by stubs. Runs on the host (x86-64) without
       any embedded hardware.
   * - ``IT``
     - Integration Test
     - Tests interactions between two or more modules (e.g., cascade
       controller exercising both PID instances and the anti-windup
       propagation path). Runs on host.
   * - ``PT``
     - Performance Test
     - Measures execution time (WCET), stack usage, and binary size on the
       embedded target or under emulation. Requires a build for an embedded
       target or a cycle-accurate simulator.
   * - ``FV``
     - Formal Verification
     - Bounded model checking via CBMC (C) or Kani (Rust). Proves safety
       properties over all bounded inputs without enumeration.

Test Environments
-----------------

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Environment ID
     - Description
   * - ``ENV-HOST``
     - Host x86-64 PC. Used for all ``UT`` and ``IT`` tests. Compiled with
       GCC or Clang (C) / ``rustc`` stable (Rust) with AddressSanitizer and
       UBSan enabled. No embedded hardware required.
   * - ``ENV-TARGET``
     - Target embedded board or cycle-accurate emulator (QEMU). Used for
       ``PT`` tests. Specific target defined in the System Integration Record
       (RON-SIR-001).
   * - ``ENV-FV``
     - Formal verification environment. CBMC (C) or Kani (Rust) run on the
       host with bounded loop unwinding.

Test Frameworks
---------------

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - Track
     - Framework
     - Notes
   * - C11
     - Unity [REF-01]
     - Lightweight, ``no_stdlib``-compatible, produces JUnit XML via
       ``unity_xml_runner``. Test functions named after their test ID
       (see Section 1.5). ``CMake`` ``enable_testing()`` / ``ctest``
       collects and runs all suites.
   * - Rust
     - ``cargo test`` [REF-02]
     - Built-in. Test functions named after their test ID. JUnit XML output
       via ``cargo test -- -Z unstable-options --format json | cargo2junit``
       or ``cargo-nextest`` (MIT) with built-in JUnit XML export.
   * - C (formal)
     - CBMC [REF-06]
     - Harness files in ``c/test/formal/``. Run as part of CI via
       ``cbmc --unwind N harness.c``.
   * - Rust (formal)
     - Kani [REF-05]
     - Harness functions annotated ``#[kani::proof]``. Run via
       ``cargo kani``.

Test ID Naming Scheme
---------------------

Every test case is assigned a unique **Test Case ID** of the form:

.. code-block:: none

   RON-TC-<MODULE>-<NNN>

where ``<MODULE>`` is a 2–5 character module code and ``<NNN>`` is a
zero-padded three-digit sequence number.

.. list-table::
   :header-rows: 1
   :widths: 12 20 68

   * - Module Code
     - Module
     - Requirement Groups Covered
   * - ``PID``
     - PID controller
     - RON-FR-001 – FR-071
   * - ``FILT``
     - Signal conditioning filters
     - RON-FR-100 – FR-131
   * - ``FF``
     - Feed-forward control
     - RON-FR-200 – FR-205
   * - ``GS``
     - Gain scheduling
     - RON-FR-300 – FR-306
   * - ``CASC``
     - Cascade controller
     - RON-FR-400 – FR-406
   * - ``TRAJ``
     - Trajectory generators
     - RON-FR-500 – FR-513
   * - ``KF``
     - Kalman filter
     - RON-FR-600 – FR-607
   * - ``SS``
     - State-space + observer
     - RON-FR-700 – FR-723
   * - ``AT``
     - Auto-tuning
     - RON-FR-800 – FR-807
   * - ``HLTH``
     - Health monitor
     - RON-FR-900 – FR-905
   * - ``MET``
     - Performance metrics
     - RON-FR-950 – FR-954
   * - ``SAFE``
     - Safety, fault, defensive programming
     - RON-SR-001 – SR-022
   * - ``PERF``
     - Performance (timing, memory, precision)
     - RON-PR-001 – PR-022
   * - ``QUAL``
     - Quality attributes, standards, portability
     - RON-SR-030 – SR-033, RON-QR-001 – QR-031, RON-DC-001 – DC-005

Embedding Test IDs in Code
---------------------------

Test IDs appear in source code so that CI tooling can correlate test
results back to this specification.

**C11 / Unity convention:**

Each test function is named ``test_<lowercased_id>`` (hyphens replaced by
underscores) and preceded by a ``/* RON-TC-xxx-NNN */`` comment:

.. code-block:: c

   /* RON-TC-PID-001 | RON-FR-001 */
   void test_ron_tc_pid_001(void)
   {
       /* Verify parallel PID form: known step response of a first-order plant */
       ron_pid_instance_t pid;
       ron_pid_config_t   cfg = make_default_pid_config();
       TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_pid_init(&pid, &cfg));

       ron_float_t u;
       ron_status_t status;
       ron_pid_step(&pid, 1.0F, 0.0F, 0.01F, &u, &status);
       TEST_ASSERT_FLOAT_WITHIN(0.01F, cfg.Kp, u); /* first step ≈ Kp·e */
   }

   /* In the Unity test runner file: */
   RUN_TEST(test_ron_tc_pid_001);

**Unity test suite registration macro** — a helper macro ensures the test
ID also appears in the XML output as the test name:

.. code-block:: c

   #define RON_RUN_TC(id, fn)  \
       Unity.TestFile = __FILE__; \
       UnityDefaultTestRun(fn, #id, __LINE__)

   RON_RUN_TC(RON_TC_PID_001, test_ron_tc_pid_001);

**Rust / cargo test convention:**

Each test function is named with the lowercased test ID (hyphens → underscores).
The ``#[doc]`` attribute embeds the test ID string so it appears in ``cargo
test -- --format json`` output and in ``cargo-nextest`` reports:

.. code-block:: rust

   /// RON-TC-PID-001 | RON-FR-001
   /// Verify parallel PID form: known step response of a first-order plant.
   #[test]
   fn ron_tc_pid_001() {
       let cfg = PidConfig::default_with_gains(1.5, 0.3, 0.05);
       let mut pid = Pid::new(cfg).unwrap();
       let (u, _status) = pid.step(1.0, 0.0, 0.01).unwrap();
       assert!((u - cfg.Kp).abs() < 0.01, "first step ≈ Kp·e");
   }

**Kani proof harness convention:**

.. code-block:: rust

   /// RON-TC-PID-015-FV | RON-FR-020, RON-SR-010
   /// Formal: output is always within [u_min, u_max] for any finite input.
   #[cfg(kani)]
   #[kani::proof]
   fn ron_tc_pid_015_fv() {
       let r: f32 = kani::any();
       let y: f32 = kani::any();
       kani::assume(r.is_finite() && y.is_finite());
       let cfg = PidConfig::default_with_gains(1.0, 0.1, 0.0);
       let mut pid = Pid::new(cfg.clone()).unwrap();
       let (u, _) = pid.step(r, y, 0.01).unwrap();
       assert!(u >= cfg.u_min && u <= cfg.u_max);
   }

CI Integration and Report Generation
--------------------------------------

Both tracks emit JUnit-compatible XML, which is consumed by CI dashboards
(GitHub Actions ``junit-reporter``, GitLab ``artifacts: reports: junit``).

**C track CI pipeline** (``ci_c.yml`` excerpt):

.. code-block:: yaml

   - name: Build and run tests (host)
     run: |
       cmake -B build_host -S c/ -DRON_BUILD_TESTS=ON \
             -DCMAKE_C_FLAGS="-fsanitize=address,undefined"
       cmake --build build_host
       cd build_host && ctest --output-junit test_results_c.xml

   - name: Static analysis
     run: |
       cppcheck --addon=misra.py --xml --xml-version=2 \
         c/src/ 2> cppcheck_results.xml

   - name: Formal verification (CBMC)
     run: |
       for h in c/test/formal/*.c; do
         cbmc --unwind 32 --xml-ui "$h" >> cbmc_results.xml
       done

   - name: Coverage report
     run: |
       cmake -B build_cov -S c/ -DRON_BUILD_TESTS=ON \
             -DCMAKE_C_FLAGS="--coverage"
       cmake --build build_cov
       cd build_cov && ctest
       lcov --capture --directory . --output-file coverage.info
       lcov --remove coverage.info '/usr/*' --output-file coverage.info
       genhtml coverage.info --output-directory coverage_html

   - uses: actions/upload-artifact@v4
     with:
       name: test-reports-c
       path: |
         build_host/test_results_c.xml
         coverage_html/
         cppcheck_results.xml

**Rust track CI pipeline** (``ci_rust.yml`` excerpt):

.. code-block:: yaml

   - name: Run tests with nextest (JUnit XML)
     run: |
       cargo nextest run --workspace \
         --profile ci \
         --test-output immediate-final
     working-directory: rust/

   - name: Kani formal verification
     run: cargo kani --workspace
     working-directory: rust/

   - name: Coverage (llvm-cov)
     run: |
       cargo llvm-cov nextest --workspace \
         --lcov --output-path coverage.lcov \
         --mcdc
     working-directory: rust/

   - uses: actions/upload-artifact@v4
     with:
       name: test-reports-rust
       path: |
         rust/target/nextest/ci/junit.xml
         rust/coverage.lcov

Coverage Requirements
---------------------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Metric
     - Target
     - Measurement Tool
   * - Statement coverage
     - 100%
     - ``lcov`` (C), ``cargo-llvm-cov`` (Rust)
   * - Branch (decision) coverage
     - 100%
     - ``lcov`` (C), ``cargo-llvm-cov`` (Rust)
   * - MC/DC coverage (safety paths)
     - 100% of RON-SR-010 – SR-013 conditions
     - ``llvm-cov --mcdc`` (both tracks, LLVM 18+)
   * - Requirement coverage
     - 100% of RON-SRS-001 requirements
     - Traceability matrix in this document
   * - Formal verification
     - All saturation, fault-detection, and anti-windup functions
     - CBMC / Kani (bounded, ``--unwind 32``)

------------------------------------------------------------------------

Requirement-to-Test Traceability Matrix
========================================

This matrix maps every RON-SRS-001 v1.1.0 requirement to the test case(s)
that verify it. Every requirement **shall** appear in at least one row.

.. list-table::
   :header-rows: 1
   :widths: 18 42 10 30

   * - Requirement ID
     - Requirement Summary
     - Level
     - Test Case ID(s)
   * - RON-FR-001
     - Parallel PID form
     - UT
     - RON-TC-PID-001
   * - RON-FR-002
     - ISA form
     - UT
     - RON-TC-PID-002
   * - RON-FR-003
     - Proportional term
     - UT
     - RON-TC-PID-003
   * - RON-FR-004
     - Euler / trapezoidal integration
     - UT
     - RON-TC-PID-004, RON-TC-PID-005
   * - RON-FR-005
     - Derivative on error / measurement modes
     - UT
     - RON-TC-PID-006, RON-TC-PID-007
   * - RON-FR-006
     - Derivative LP filter
     - UT
     - RON-TC-PID-008, RON-TC-PID-009
   * - RON-FR-007
     - 2DOF setpoint weights
     - UT
     - RON-TC-PID-010
   * - RON-FR-010
     - Input scaling / normalisation
     - UT
     - RON-TC-PID-011
   * - RON-FR-011
     - Linear normalisation map
     - UT
     - RON-TC-PID-012
   * - RON-FR-012
     - Output de-normalisation
     - UT
     - RON-TC-PID-013
   * - RON-FR-013
     - Normalisation optional
     - UT
     - RON-TC-PID-014
   * - RON-FR-020
     - Output hard saturation
     - UT, FV
     - RON-TC-PID-015, RON-TC-PID-015-FV
   * - RON-FR-021
     - Runtime limit update
     - UT
     - RON-TC-PID-016
   * - RON-FR-022
     - SATURATED status flag
     - UT
     - RON-TC-PID-017
   * - RON-FR-025
     - Output rate limiting
     - UT
     - RON-TC-PID-018
   * - RON-FR-026
     - Rate limit disable (≤ 0)
     - UT
     - RON-TC-PID-019
   * - RON-FR-027
     - Rate limit after saturation
     - UT
     - RON-TC-PID-020
   * - RON-FR-030
     - Anti-windup required
     - UT
     - RON-TC-PID-021
   * - RON-FR-031
     - Back-calculation AW
     - UT, FV
     - RON-TC-PID-022, RON-TC-PID-022-FV
   * - RON-FR-032
     - Conditional clamping AW
     - UT
     - RON-TC-PID-023
   * - RON-FR-033
     - AW mode selection
     - UT
     - RON-TC-PID-024
   * - RON-FR-034
     - AW disable option
     - UT
     - RON-TC-PID-025
   * - RON-FR-035
     - Integral accumulator clamp
     - UT, FV
     - RON-TC-PID-026, RON-TC-PID-026-FV
   * - RON-FR-040
     - Manual override mode
     - UT
     - RON-TC-PID-027
   * - RON-FR-041
     - Bumpless manual→auto transfer
     - UT
     - RON-TC-PID-028
   * - RON-FR-042
     - Integral freeze on auto→manual
     - UT
     - RON-TC-PID-029
   * - RON-FR-050
     - Explicit initialisation
     - UT
     - RON-TC-PID-030
   * - RON-FR-051
     - State reset (config preserved)
     - UT
     - RON-TC-PID-031
   * - RON-FR-052
     - Integral pre-load (warm start)
     - UT
     - RON-TC-PID-032
   * - RON-FR-053
     - Atomic runtime parameter update
     - UT
     - RON-TC-PID-033
   * - RON-FR-054
     - Integral reset on SP change
     - UT
     - RON-TC-PID-034
   * - RON-FR-060
     - Multiple simultaneous instances
     - IT
     - RON-TC-PID-035
   * - RON-FR-061
     - No global mutable state
     - UT, FV
     - RON-TC-PID-036, RON-TC-PID-036-FV
   * - RON-FR-062
     - Instance independence
     - IT
     - RON-TC-PID-037
   * - RON-FR-070
     - Status word bitfields
     - UT
     - RON-TC-PID-038
   * - RON-FR-071
     - State getter functions
     - UT
     - RON-TC-PID-039
   * - RON-FR-100
     - Filters as first-class components
     - UT
     - RON-TC-FILT-001
   * - RON-FR-101
     - Filter instance/config/state model
     - UT
     - RON-TC-FILT-002
   * - RON-FR-102
     - Filter init/reset/step/get_state API
     - UT
     - RON-TC-FILT-003
   * - RON-FR-103
     - Filter coefficient validation
     - UT
     - RON-TC-FILT-004
   * - RON-FR-110
     - First-order IIR LP filter
     - UT
     - RON-TC-FILT-005
   * - RON-FR-111
     - LP1 alpha / fc configuration
     - UT
     - RON-TC-FILT-006, RON-TC-FILT-007
   * - RON-FR-115
     - Moving average filter
     - UT
     - RON-TC-FILT-008
   * - RON-FR-116
     - MA static buffer (compile-time bound)
     - UT
     - RON-TC-FILT-009
   * - RON-FR-117
     - MA sliding-sum efficiency
     - UT
     - RON-TC-FILT-010
   * - RON-FR-120
     - Biquad IIR (Direct Form II-T)
     - UT
     - RON-TC-FILT-011
   * - RON-FR-121
     - Cascaded biquad sections
     - UT
     - RON-TC-FILT-012
   * - RON-FR-122
     - Biquad coefficient generation (LP/HP/BP/notch)
     - UT
     - RON-TC-FILT-013, RON-TC-FILT-014
   * - RON-FR-123
     - Notch runtime update (no state reset)
     - UT
     - RON-TC-FILT-015
   * - RON-FR-130
     - Standalone rate limiter
     - UT
     - RON-TC-FILT-016
   * - RON-FR-131
     - Asymmetric rise/fall rate limits
     - UT
     - RON-TC-FILT-017
   * - RON-FR-200
     - Additive feed-forward path
     - UT
     - RON-TC-FF-001
   * - RON-FR-201
     - FF modes: static/velocity/accel/external
     - UT
     - RON-TC-FF-002, RON-TC-FF-003, RON-TC-FF-004, RON-TC-FF-005
   * - RON-FR-202
     - FF derivative estimation with LP filter
     - UT
     - RON-TC-FF-006
   * - RON-FR-203
     - FF subject to saturation/rate limits
     - UT
     - RON-TC-FF-007
   * - RON-FR-204
     - FF disabled by default (zero overhead)
     - UT
     - RON-TC-FF-008
   * - RON-FR-205
     - FF term in status/diagnostics
     - UT
     - RON-TC-FF-009
   * - RON-FR-300
     - Gain scheduling breakpoint table
     - UT
     - RON-TC-GS-001
   * - RON-FR-301
     - Static breakpoint table (compile-time bound)
     - UT
     - RON-TC-GS-002
   * - RON-FR-302
     - Hard-switch and linear-interpolation modes
     - UT
     - RON-TC-GS-003, RON-TC-GS-004
   * - RON-FR-303
     - Atomic gain update via runtime API
     - UT
     - RON-TC-GS-005
   * - RON-FR-304
     - Decoupled scheduling update rate
     - UT
     - RON-TC-GS-006
   * - RON-FR-305
     - Integral reset on hard switch (optional)
     - UT
     - RON-TC-GS-007
   * - RON-FR-306
     - Breakpoint table validation
     - UT
     - RON-TC-GS-008
   * - RON-FR-400
     - Cascade controller structure
     - IT
     - RON-TC-CASC-001
   * - RON-FR-401
     - Cascade step: outer→inner setpoint
     - IT
     - RON-TC-CASC-002
   * - RON-FR-402
     - Inner-loop output range constraint
     - IT
     - RON-TC-CASC-003
   * - RON-FR-403
     - AW back-propagation outer↔inner
     - IT, FV
     - RON-TC-CASC-004, RON-TC-CASC-004-FV
   * - RON-FR-404
     - Coordinated mode transitions
     - IT
     - RON-TC-CASC-005
   * - RON-FR-405
     - Full single-PID feature support in cascade
     - IT
     - RON-TC-CASC-006
   * - RON-FR-406
     - Unified status word (outer/inner bits)
     - IT
     - RON-TC-CASC-007
   * - RON-FR-500
     - Trapezoidal velocity profile
     - UT
     - RON-TC-TRAJ-001
   * - RON-FR-501
     - Short-move case
     - UT
     - RON-TC-TRAJ-002
   * - RON-FR-502
     - Kinematic outputs (pos/vel/acc)
     - UT
     - RON-TC-TRAJ-003
   * - RON-FR-503
     - Online goal update mid-profile
     - UT
     - RON-TC-TRAJ-004
   * - RON-FR-510
     - S-curve jerk-limited profile
     - UT
     - RON-TC-TRAJ-005
   * - RON-FR-511
     - S-curve exposes jerk output
     - UT
     - RON-TC-TRAJ-006
   * - RON-FR-512
     - ``finished`` flag
     - UT
     - RON-TC-TRAJ-007
   * - RON-FR-513
     - Hold mode
     - UT
     - RON-TC-TRAJ-008
   * - RON-FR-600
     - Kalman predict–update cycle
     - UT
     - RON-TC-KF-001
   * - RON-FR-601
     - Kalman parameter matrices
     - UT
     - RON-TC-KF-002
   * - RON-FR-602
     - Predict–update algorithm correctness
     - UT
     - RON-TC-KF-003
   * - RON-FR-603
     - Cholesky solve for innovation inversion
     - UT
     - RON-TC-KF-004
   * - RON-FR-604
     - Joseph form covariance update
     - UT
     - RON-TC-KF-005
   * - RON-FR-605
     - Measurement dropout
     - UT
     - RON-TC-KF-006
   * - RON-FR-606
     - Steady-state (fixed-gain) mode
     - UT
     - RON-TC-KF-007
   * - RON-FR-607
     - No dynamic allocation in Kalman
     - UT, FV
     - RON-TC-KF-008, RON-TC-KF-008-FV
   * - RON-FR-700
     - State-feedback controller
     - UT
     - RON-TC-SS-001
   * - RON-FR-701
     - State source selection
     - UT
     - RON-TC-SS-002
   * - RON-FR-702
     - Integral augmentation
     - UT
     - RON-TC-SS-003
   * - RON-FR-703
     - SS saturation / rate limiting / fault
     - UT, FV
     - RON-TC-SS-004, RON-TC-SS-004-FV
   * - RON-FR-704
     - Runtime gain matrix update
     - UT
     - RON-TC-SS-005
   * - RON-FR-720
     - Luenberger observer step
     - UT
     - RON-TC-SS-006
   * - RON-FR-721
     - Observer parameterisation
     - UT
     - RON-TC-SS-007
   * - RON-FR-722
     - Observer state getter
     - UT
     - RON-TC-SS-008
   * - RON-FR-723
     - Observer compile-time dimension bounds
     - UT
     - RON-TC-SS-009
   * - RON-FR-800
     - Relay feedback auto-tuning
     - IT
     - RON-TC-AT-001
   * - RON-FR-801
     - Relay configuration parameters
     - UT
     - RON-TC-AT-002
   * - RON-FR-802
     - Ku / Tu estimation
     - UT
     - RON-TC-AT-003
   * - RON-FR-803
     - Tuning rule selection (ZN/TL/OS/NOS)
     - UT
     - RON-TC-AT-004
   * - RON-FR-804
     - Gains applied only on explicit ``apply``
     - UT
     - RON-TC-AT-005
   * - RON-FR-805
     - Raw Ku/Tu exposed to caller
     - UT
     - RON-TC-AT-006
   * - RON-FR-806
     - Relay output within bias ± d
     - UT, FV
     - RON-TC-AT-007, RON-TC-AT-007-FV
   * - RON-FR-807
     - Abort on fault or explicit abort
     - UT
     - RON-TC-AT-008
   * - RON-FR-900
     - Health monitor attaches to controller
     - UT
     - RON-TC-HLTH-001
   * - RON-FR-901
     - All five health conditions detected
     - UT
     - RON-TC-HLTH-002, RON-TC-HLTH-003, RON-TC-HLTH-004, RON-TC-HLTH-005, RON-TC-HLTH-006
   * - RON-FR-902
     - Independent per-condition thresholds
     - UT
     - RON-TC-HLTH-007
   * - RON-FR-903
     - Health monitor is passive
     - UT
     - RON-TC-HLTH-008
   * - RON-FR-904
     - Health callback on first activation
     - UT
     - RON-TC-HLTH-009
   * - RON-FR-905
     - Health status latches until cleared
     - UT
     - RON-TC-HLTH-010
   * - RON-FR-950
     - Metrics accumulator
     - UT
     - RON-TC-MET-001
   * - RON-FR-951
     - IAE / ISE / ITAE / overshoot / settling / rise time
     - UT
     - RON-TC-MET-002, RON-TC-MET-003, RON-TC-MET-004
   * - RON-FR-952
     - Windowed and cumulative modes
     - UT
     - RON-TC-MET-005
   * - RON-FR-953
     - Zero overhead when disabled
     - UT, PT
     - RON-TC-MET-006, RON-TC-PERF-007
   * - RON-FR-954
     - Setpoint step detection / metric restart
     - UT
     - RON-TC-MET-007
   * - RON-SR-001
     - API input parameter validation
     - UT, FV
     - RON-TC-SAFE-001, RON-TC-SAFE-001-FV
   * - RON-SR-002
     - Division zero-divisor guard
     - UT, FV
     - RON-TC-SAFE-002, RON-TC-SAFE-002-FV
   * - RON-SR-003
     - No dynamic allocation
     - FV, PT
     - RON-TC-SAFE-003-FV, RON-TC-PERF-006
   * - RON-SR-004
     - No recursion
     - FV
     - RON-TC-SAFE-004-FV
   * - RON-SR-005
     - No unchecked array indexing
     - FV
     - RON-TC-SAFE-005-FV
   * - RON-SR-006
     - Null pointer validation
     - UT, FV
     - RON-TC-SAFE-006, RON-TC-SAFE-006-FV
   * - RON-SR-010
     - Fault conditions detected
     - UT, FV
     - RON-TC-SAFE-007, RON-TC-SAFE-007-FV
   * - RON-SR-011
     - Safe-state output policy
     - UT
     - RON-TC-SAFE-008
   * - RON-SR-012
     - Fault state latches until cleared
     - UT
     - RON-TC-SAFE-009
   * - RON-SR-013
     - Fault code register
     - UT
     - RON-TC-SAFE-010
   * - RON-SR-020
     - NaN / Inf detection in all inputs
     - UT, FV
     - RON-TC-SAFE-011, RON-TC-SAFE-011-FV
   * - RON-SR-021
     - Minimise round-off accumulation
     - UT
     - RON-TC-SAFE-012
   * - RON-SR-022
     - Integral accumulator finite clamp
     - UT, FV
     - RON-TC-SAFE-013, RON-TC-SAFE-013-FV
   * - RON-SR-030
     - MISRA C:2023 / MISRA Rust:2024 compliance
     - UT (static)
     - RON-TC-QUAL-001
   * - RON-SR-031
     - IEC 61508 SIL 2 compatible lifecycle
     - Review
     - RON-TC-QUAL-002
   * - RON-SR-032
     - Requirement traceability
     - Review
     - RON-TC-QUAL-003
   * - RON-SR-033
     - Free static analysis tool compliance
     - UT (static)
     - RON-TC-QUAL-004
   * - RON-PR-001
     - Bounded deterministic WCET
     - FV, PT
     - RON-TC-PERF-001, RON-TC-SAFE-004-FV
   * - RON-PR-002
     - O(1) execution — no loops/recursion/alloc
     - FV
     - RON-TC-PERF-002-FV
   * - RON-PR-003
     - WCET meets integration target budget
     - PT
     - RON-TC-PERF-003
   * - RON-PR-004
     - ISR-safe (no blocking operations)
     - FV
     - RON-TC-PERF-004-FV
   * - RON-PR-010
     - 32-bit float minimum
     - UT
     - RON-TC-PERF-005
   * - RON-PR-011
     - 64-bit double option
     - UT
     - RON-TC-PERF-005
   * - RON-PR-012
     - IEEE 754 special value handling
     - UT, FV
     - RON-TC-SAFE-011, RON-TC-SAFE-011-FV
   * - RON-PR-020
     - Code size minimised
     - PT
     - RON-TC-PERF-006
   * - RON-PR-021
     - RAM footprint ≤ 128 B per instance
     - UT, PT
     - RON-TC-PERF-006
   * - RON-PR-022
     - No heap allocation
     - FV, PT
     - RON-TC-SAFE-003-FV, RON-TC-PERF-006
   * - RON-QR-001
     - Portable across 32/64-bit architectures
     - UT
     - RON-TC-QUAL-005
   * - RON-QR-002
     - Hardware-specific code isolated
     - Review
     - RON-TC-QUAL-006
   * - RON-QR-003
     - Exact-width integer types
     - UT (static)
     - RON-TC-QUAL-007
   * - RON-QR-004
     - Compiles clean under two toolchains
     - UT
     - RON-TC-QUAL-008
   * - RON-QR-010
     - Single responsibility per function
     - Review
     - RON-TC-QUAL-009
   * - RON-QR-011
     - Cyclomatic complexity ≤ 10
     - UT (static)
     - RON-TC-QUAL-010
   * - RON-QR-012
     - API documentation (Doxygen / rustdoc)
     - Review
     - RON-TC-QUAL-011
   * - RON-QR-013
     - No magic numbers
     - UT (static)
     - RON-TC-QUAL-012
   * - RON-QR-020
     - Host-testable without hardware
     - UT
     - RON-TC-QUAL-013
   * - RON-QR-021
     - Internal state accessible via getters
     - UT
     - RON-TC-PID-039, RON-TC-KF-008
   * - RON-QR-022
     - 100% statement + branch coverage
     - UT
     - RON-TC-QUAL-014
   * - RON-QR-023
     - MC/DC on safety-critical paths
     - UT
     - RON-TC-QUAL-015
   * - RON-QR-030
     - No data races (single context)
     - FV
     - RON-TC-QUAL-016-FV
   * - RON-QR-031
     - Deterministic reproducible behaviour
     - UT
     - RON-TC-QUAL-017
   * - RON-DC-001
     - Language: C11 or Rust Edition 2021
     - Review
     - RON-TC-QUAL-018
   * - RON-DC-002
     - Minimal stdlib dependencies
     - UT (static)
     - RON-TC-QUAL-019
   * - RON-DC-003
     - No setjmp/longjmp/goto/exceptions
     - UT (static)
     - RON-TC-QUAL-020
   * - RON-DC-004
     - 7-bit ASCII source files
     - UT (static)
     - RON-TC-QUAL-021
   * - RON-DC-005
     - Cross-compilation build support
     - PT
     - RON-TC-QUAL-022

------------------------------------------------------------------------

Test-to-Requirement Traceability Matrix
=========================================

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Test Case ID
     - Level
     - Requirements Verified
   * - RON-TC-PID-001
     - UT
     - RON-FR-001
   * - RON-TC-PID-002
     - UT
     - RON-FR-002
   * - RON-TC-PID-003
     - UT
     - RON-FR-003
   * - RON-TC-PID-004
     - UT
     - RON-FR-004 (Euler)
   * - RON-TC-PID-005
     - UT
     - RON-FR-004 (trapezoidal)
   * - RON-TC-PID-006
     - UT
     - RON-FR-005 (DoE)
   * - RON-TC-PID-007
     - UT
     - RON-FR-005 (DoM, no kick)
   * - RON-TC-PID-008
     - UT
     - RON-FR-006 (filter enabled)
   * - RON-TC-PID-009
     - UT
     - RON-FR-006 (N=0, filter disabled)
   * - RON-TC-PID-010
     - UT
     - RON-FR-007
   * - RON-TC-PID-011
     - UT
     - RON-FR-010
   * - RON-TC-PID-012
     - UT
     - RON-FR-011
   * - RON-TC-PID-013
     - UT
     - RON-FR-012
   * - RON-TC-PID-014
     - UT
     - RON-FR-013
   * - RON-TC-PID-015
     - UT
     - RON-FR-020
   * - RON-TC-PID-015-FV
     - FV
     - RON-FR-020, RON-SR-010
   * - RON-TC-PID-016
     - UT
     - RON-FR-021
   * - RON-TC-PID-017
     - UT
     - RON-FR-022
   * - RON-TC-PID-018
     - UT
     - RON-FR-025
   * - RON-TC-PID-019
     - UT
     - RON-FR-026
   * - RON-TC-PID-020
     - UT
     - RON-FR-027
   * - RON-TC-PID-021
     - UT
     - RON-FR-030
   * - RON-TC-PID-022
     - UT
     - RON-FR-031
   * - RON-TC-PID-022-FV
     - FV
     - RON-FR-031
   * - RON-TC-PID-023
     - UT
     - RON-FR-032
   * - RON-TC-PID-024
     - UT
     - RON-FR-033
   * - RON-TC-PID-025
     - UT
     - RON-FR-034
   * - RON-TC-PID-026
     - UT
     - RON-FR-035
   * - RON-TC-PID-026-FV
     - FV
     - RON-FR-035
   * - RON-TC-PID-027
     - UT
     - RON-FR-040
   * - RON-TC-PID-028
     - UT
     - RON-FR-041
   * - RON-TC-PID-029
     - UT
     - RON-FR-042
   * - RON-TC-PID-030
     - UT
     - RON-FR-050
   * - RON-TC-PID-031
     - UT
     - RON-FR-051
   * - RON-TC-PID-032
     - UT
     - RON-FR-052
   * - RON-TC-PID-033
     - UT
     - RON-FR-053
   * - RON-TC-PID-034
     - UT
     - RON-FR-054
   * - RON-TC-PID-035
     - IT
     - RON-FR-060
   * - RON-TC-PID-036
     - UT
     - RON-FR-061
   * - RON-TC-PID-036-FV
     - FV
     - RON-FR-061
   * - RON-TC-PID-037
     - IT
     - RON-FR-062
   * - RON-TC-PID-038
     - UT
     - RON-FR-070
   * - RON-TC-PID-039
     - UT
     - RON-FR-071, RON-QR-021
   * - RON-TC-FILT-001 – FILT-017
     - UT
     - RON-FR-100 – FR-131
   * - RON-TC-FF-001 – FF-009
     - UT
     - RON-FR-200 – FR-205
   * - RON-TC-GS-001 – GS-008
     - UT
     - RON-FR-300 – FR-306
   * - RON-TC-CASC-001 – CASC-007
     - IT
     - RON-FR-400 – FR-406
   * - RON-TC-CASC-004-FV
     - FV
     - RON-FR-403
   * - RON-TC-TRAJ-001 – TRAJ-008
     - UT
     - RON-FR-500 – FR-513
   * - RON-TC-KF-001 – KF-008
     - UT
     - RON-FR-600 – FR-607
   * - RON-TC-KF-008-FV
     - FV
     - RON-FR-607, RON-SR-003
   * - RON-TC-SS-001 – SS-009
     - UT
     - RON-FR-700 – FR-723
   * - RON-TC-SS-004-FV
     - FV
     - RON-FR-703, RON-FR-020
   * - RON-TC-AT-001 – AT-008
     - UT / IT
     - RON-FR-800 – FR-807
   * - RON-TC-AT-007-FV
     - FV
     - RON-FR-806
   * - RON-TC-HLTH-001 – HLTH-010
     - UT
     - RON-FR-900 – FR-905
   * - RON-TC-MET-001 – MET-007
     - UT
     - RON-FR-950 – FR-954
   * - RON-TC-SAFE-001 – SAFE-013
     - UT / FV
     - RON-SR-001 – SR-022
   * - RON-TC-PERF-001 – PERF-008
     - PT / FV
     - RON-PR-001 – PR-022, RON-FR-953
   * - RON-TC-QUAL-001 – QUAL-022
     - UT (static) / Review / PT
     - RON-SR-030 – SR-033, RON-QR-001 – QR-031, RON-DC-001 – DC-005

------------------------------------------------------------------------

Detailed Test Case Specifications
===================================

PID Controller Tests (RON-TC-PID-xxx)
=======================================

RON-TC-PID-001 — Parallel PID Form Output Correctness
-------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-001
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Instance initialised with :math:`K_p=2.0`, :math:`K_i=0.0`,
       :math:`K_d=0.0`, no normalisation, no saturation (:math:`u_{min}=-1000`,
       :math:`u_{max}=1000`), AW disabled.
   * - **Stimulus**
     - Single step: :math:`r=1.0`, :math:`y=0.0`, :math:`dt=0.01`.
   * - **Expected Output**
     - :math:`u = K_p \cdot e = 2.0 \cdot 1.0 = 2.0`. Tolerance: ±1 ULP.
   * - **Pass Criterion**
     - ``fabs(u - 2.0) < FLT_EPSILON * 4.0``. Status = ``RON_STATUS_OK``.
       Fault = ``RON_FAULT_NONE``.

.. code-block:: c

   /* RON-TC-PID-001 | RON-FR-001 */
   void test_ron_tc_pid_001(void) {
       ron_pid_instance_t pid;
       ron_pid_config_t cfg = make_pid_cfg_kp_only(2.0F);
       TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_pid_init(&pid, &cfg));
       ron_float_t u; ron_status_t s;
       TEST_ASSERT_EQUAL(RON_FAULT_NONE,
           ron_pid_step(&pid, 1.0F, 0.0F, 0.01F, &u, &s));
       TEST_ASSERT_FLOAT_WITHIN(4.0F * FLT_EPSILON, 2.0F, u);
       TEST_ASSERT_EQUAL(RON_STATUS_OK, s);
   }

.. code-block:: rust

   /// RON-TC-PID-001 | RON-FR-001
   #[test]
   fn ron_tc_pid_001() {
       let mut pid = Pid::new(kp_only_config(2.0)).unwrap();
       let (u, s) = pid.step(1.0, 0.0, 0.01).unwrap();
       assert!((u - 2.0_f32).abs() < 4.0 * f32::EPSILON);
       assert_eq!(s, PidStatus::OK);
   }

------------------------------------------------------------------------

RON-TC-PID-002 — ISA Form Parameter Conversion
-----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-002
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Two instances: one configured in parallel form, one using ISA form
       (:math:`K_p=2.0`, :math:`T_i=5.0`, :math:`T_d=0.1`), implying
       :math:`K_i = K_p/T_i = 0.4`, :math:`K_d = K_p \cdot T_d = 0.2`.
   * - **Stimulus**
     - 50 identical steps (:math:`r=1.0`, :math:`y=0.5`, :math:`dt=0.01`).
   * - **Expected Output**
     - Both instances produce identical output sequences within ±4 ULP.
   * - **Pass Criterion**
     - ``fabs(u_parallel - u_isa) < 4 * FLT_EPSILON`` for all 50 steps.

------------------------------------------------------------------------

RON-TC-PID-007 — Derivative-on-Measurement: No Kick on SP Step
----------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-005 (DoM default)
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Instance: :math:`K_p=1.0`, :math:`K_i=0.0`, :math:`K_d=2.0`,
       :math:`N=0` (filter off), DoM mode. Running at steady state
       (:math:`r=y=0.5`).
   * - **Stimulus**
     - Instantaneous SP step: :math:`r` changes from 0.5 to 1.0 at step k.
       :math:`y` unchanged = 0.5. :math:`dt=0.01`.
   * - **Expected Output**
     - In DoM mode: derivative = :math:`-(y(k) - y(k-1))/dt = 0` (y unchanged).
       Therefore no derivative spike: :math:`u(k) = K_p \cdot 0.5 = 0.5`.
   * - **Pass Criterion**
     - Output change < :math:`2 \cdot K_p \cdot \Delta r = 1.0`. No spike
       exceeding :math:`K_p \cdot e + \epsilon`.
   * - **Contrast test**
     - RON-TC-PID-006 uses DoE mode and verifies a spike *is* present (the
       expected behaviour in that mode).

------------------------------------------------------------------------

RON-TC-PID-015 — Output Saturation Hard Limits
-----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-020
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`K_p=1000.0`, :math:`u_{min}=-10.0`, :math:`u_{max}=10.0`,
       AW disabled.
   * - **Stimulus**
     - 20 steps with :math:`r=100.0`, :math:`y=0.0`.
   * - **Expected Output**
     - All 20 outputs == ``u_max`` = 10.0 exactly.
       ``RON_STATUS_SATURATED`` flag set in all 20 status words.
   * - **Pass Criterion**
     - ``u == 10.0F`` (exact equality for clamped value).
       ``(status & RON_STATUS_SATURATED) != 0``.

RON-TC-PID-015-FV — Saturation Formal Proof
---------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-020, RON-SR-010
   * - **Level**
     - FV / ENV-FV
   * - **Property**
     - For all finite :math:`r`, :math:`y`, :math:`dt > 0`: output
       :math:`u \in [u_{min}, u_{max}]`.
   * - **Tool**
     - Kani (Rust): ``#[kani::proof]`` with ``kani::any::<f32>()`` inputs
       and ``kani::assume(r.is_finite() && y.is_finite() && dt > 0.0)``.
       CBMC (C): loop-free bounded harness with ``__CPROVER_assume``.
   * - **Pass Criterion**
     - No assertion violation reported by the model checker.

------------------------------------------------------------------------

RON-TC-PID-022 — Back-Calculation Anti-Windup Recovery
-------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-031
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`K_p=1.0`, :math:`K_i=10.0`, :math:`K_d=0.0`,
       :math:`u_{min}=-5.0`, :math:`u_{max}=5.0`, :math:`T_{aw}=0.1`,
       AW mode = ``RON_AW_BACK_CALC``.
   * - **Stimulus**
     - Phase 1 (saturation): 100 steps, :math:`r=100.0`, :math:`y=0.0`.
       Phase 2 (recovery): 100 steps, :math:`r=0.0`, :math:`y=0.0`.
   * - **Expected Output**
     - Phase 1: output clamped to 5.0; integral bounded (AW active).
       Phase 2: output returns to 0.0 within 20 steps (tracked by settling
       criterion: ``fabs(u) < 0.1``).
   * - **Pass Criterion**
     - Recovery settling step ≤ 20. Compare against a no-AW instance:
       its recovery **shall** be ≥ 2× slower (windup present).
   * - **Note**
     - Same test without AW (RON-TC-PID-025) must demonstrate the
       contrast — if AW is not beneficial on this plant/gains combination,
       tighten the gain or increase dt.

------------------------------------------------------------------------

RON-TC-PID-028 — Bumpless Manual-to-Automatic Transfer
-------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-041
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`K_p=2.0`, :math:`K_i=1.0`, :math:`K_d=0.0`. Controller in
       manual mode, outputting 3.5 for 10 steps (:math:`r=y=0.0`).
   * - **Stimulus**
     - Call ``ron_pid_set_mode(&pid, RON_MODE_AUTOMATIC, 3.5F)`` then
       immediately call ``ron_pid_step(&pid, 0.0F, 0.0F, 0.01F, &u, &s)``.
   * - **Expected Output**
     - First automatic-mode output ``u`` is within 5% of 3.5.
   * - **Pass Criterion**
     - ``fabs(u - 3.5F) / 3.5F < 0.05``.

------------------------------------------------------------------------

RON-TC-PID-035 — Multi-Instance Independence (Integration)
----------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-060, RON-FR-062
   * - **Level**
     - IT / ENV-HOST
   * - **Preconditions**
     - Instance A: :math:`K_p=1.0`, :math:`K_i=0.1`, :math:`K_d=0.0`.
       Instance B: :math:`K_p=3.0`, :math:`K_i=0.5`, :math:`K_d=0.2`.
       Both reset to zero state.
   * - **Stimulus**
     - 50 interleaved steps: alternate calling A then B with independent
       random setpoints and process variables.
   * - **Expected Output**
     - A produces the same output sequence as if B did not exist (verified
       by running A alone with the same inputs).
       B produces the same output sequence as if A did not exist.
   * - **Pass Criterion**
     - ``fabs(u_A_interleaved[k] - u_A_alone[k]) < FLT_EPSILON * 4`` for
       all k. Same for B.

------------------------------------------------------------------------

Signal Conditioning Filter Tests (RON-TC-FILT-xxx)
====================================================

RON-TC-FILT-005 — First-Order LP Filter Step Response
------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-110
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`\alpha = 0.1` (heavy smoothing).
   * - **Stimulus**
     - Unit step: 200 steps, input = 0 for first 10 then 1.0 thereafter.
   * - **Expected Output**
     - Output at step 20 ≈ :math:`1 - 0.9^{10} \approx 0.651`. Tolerance
       ±0.001.
     - Output at step 210 within 1% of 1.0.
   * - **Pass Criterion**
     - Analytical comparison at two checkpoints.

RON-TC-FILT-006 — LP1 Frequency-Domain Configuration
------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-111
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Two instances: one configured with :math:`\alpha` directly, one
       with :math:`f_c = 10` Hz and :math:`T_s = 0.001` s.
   * - **Stimulus**
     - 500 identical unit-step inputs.
   * - **Pass Criterion**
     - Outputs differ by < 0.1% across all 500 steps.

RON-TC-FILT-011 — Biquad IIR Correctness (Known Transfer Function)
-------------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-120
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - 2nd-order Butterworth LP at :math:`f_c = 100` Hz, :math:`f_s = 1000`
       Hz. Coefficients computed by ``ron_biquad_coeff_lp``.
   * - **Stimulus**
     - Sinusoidal input at 50 Hz (passband) and 300 Hz (stopband), 1000
       samples each.
   * - **Expected Output**
     - 50 Hz signal attenuated < 3 dB. 300 Hz signal attenuated > 20 dB.
   * - **Pass Criterion**
     - RMS ratio of output/input within specified dB bounds.

RON-TC-FILT-015 — Notch Runtime Update (No State Discontinuity)
----------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-123
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Biquad notch at :math:`f_0 = 60` Hz running on a 60 Hz sinusoidal
       input. Both steady-state.
   * - **Stimulus**
     - Call ``ron_biquad_update_notch`` to shift centre to 120 Hz while
       filter is running. Continue 50 more steps.
   * - **Pass Criterion**
     - No output spike > 3× the signal amplitude at the update step.
       Output converges to attenuating 120 Hz within 20 steps post-update.

------------------------------------------------------------------------

Feed-Forward Tests (RON-TC-FF-xxx)
====================================

RON-TC-FF-002 — Static Gain Feed-Forward
-----------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-201 (mode a)
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`K_p=1.0`, :math:`K_i=0.0`, :math:`K_d=0.0`,
       :math:`K_{FF}=0.5`.
   * - **Stimulus**
     - :math:`r=2.0`, :math:`y=0.0`, :math:`dt=0.01`.
   * - **Expected Output**
     - :math:`u = K_p \cdot e + K_{FF} \cdot r = 2.0 + 1.0 = 3.0`.
   * - **Pass Criterion**
     - ``fabs(u - 3.0) < 4 * FLT_EPSILON``.

RON-TC-FF-008 — Zero Overhead When FF Disabled
-----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-204
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Two identical PID configurations: one with FF disabled (all gains
       = 0), one with FF not configured (legacy path).
   * - **Stimulus**
     - 1000 steps each. Compare outputs.
   * - **Pass Criterion**
     - Identical outputs (within FLT_EPSILON). Verifies that disabled FF
       does not alter the PID output.

------------------------------------------------------------------------

Gain Scheduling Tests (RON-TC-GS-xxx)
=======================================

RON-TC-GS-003 — Hard-Switch Mode
---------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-302 (hard switch)
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - 3-point table: σ = [0.0, 1.0, 2.0] mapping to :math:`K_p` = [1.0,
       2.0, 3.0].
   * - **Stimulus**
     - Call ``ron_gs_update`` with σ = 0.5 → verify :math:`K_p` = 1.0.
       Call with σ = 1.5 → verify :math:`K_p` = 2.0.
       Call with σ = 2.5 → verify :math:`K_p` = 3.0 (clamped to last bp).
   * - **Pass Criterion**
     - Correct breakpoint selected in all three cases.
       Next ``ron_pid_step`` produces output consistent with new :math:`K_p`.

RON-TC-GS-004 — Linear Interpolation Mode
------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-302 (linear interpolation)
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - 2-point table: σ = [0.0, 1.0], :math:`K_p` = [1.0, 3.0].
   * - **Stimulus**
     - σ = 0.25 → expected :math:`K_p` = 1.5. σ = 0.75 → expected
       :math:`K_p` = 2.5.
   * - **Pass Criterion**
     - ``fabs(actual_Kp - expected_Kp) < 1e-5`` at each interpolation point.

------------------------------------------------------------------------

Cascade Controller Tests (RON-TC-CASC-xxx)
============================================

RON-TC-CASC-002 — Outer Output Becomes Inner Setpoint
------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-401
   * - **Level**
     - IT / ENV-HOST
   * - **Preconditions**
     - Outer: :math:`K_p=1.0`, :math:`K_i=0.0`, :math:`K_d=0.0`,
       :math:`u_{min}=-5.0`, :math:`u_{max}=5.0`.
       Inner: :math:`K_p=2.0`, :math:`K_i=0.0`, :math:`K_d=0.0`,
       :math:`u_{min}=-10.0`, :math:`u_{max}=10.0`.
   * - **Stimulus**
     - :math:`r_{out}=2.0`, :math:`y_{out}=0.0`, :math:`y_{in}=0.0`,
       :math:`dt=0.01`.
   * - **Expected Output**
     - Outer produces: :math:`r_{in} = K_p^{out} \cdot (r_{out} - y_{out}) = 2.0`.
       Inner produces: :math:`u = K_p^{in} \cdot (r_{in} - y_{in}) = 4.0`.
   * - **Pass Criterion**
     - ``fabs(u_final - 4.0) < 4 * FLT_EPSILON``.

RON-TC-CASC-004 — Outer Integral AW Back-Propagation
-----------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-403
   * - **Level**
     - IT / ENV-HOST
   * - **Preconditions**
     - Inner loop saturates immediately.
       Outer: :math:`K_i=10.0`, back-calc AW, :math:`T_{aw}=0.1`.
   * - **Stimulus**
     - 50 steps with inner loop fully saturated.
   * - **Pass Criterion**
     - Outer integral accumulator ``|I_outer|`` ≤ 2 × max single-step
       increment after 50 steps (AW effectively bounded it). Compare
       against outer loop running without inner saturation feedback —
       its integral must be > 10× larger.

RON-TC-CASC-004-FV — AW Propagation Formal Proof
-------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-403
   * - **Level**
     - FV / ENV-FV
   * - **Property**
     - Outer integral is bounded by ``[I_min, I_max]`` for all inputs
       when inner loop is saturated.
   * - **Pass Criterion**
     - No assertion violation from CBMC / Kani.

------------------------------------------------------------------------

Trajectory Generator Tests (RON-TC-TRAJ-xxx)
=============================================

RON-TC-TRAJ-001 — Trapezoidal Profile Reaches Target
-----------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-500
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`v_{max}=1.0`, :math:`a_{max}=2.0`. Target = 1.0. Start = 0.0.
   * - **Stimulus**
     - Run ``ron_trap_step`` until ``finished == true`` or 5000 steps.
   * - **Pass Criterion**
     - ``fabs(pos - 1.0) < 0.001`` at ``finished``.
       ``fabs(vel) < 0.001`` at ``finished``.
       All intermediate velocities ≤ :math:`v_{max}` + 1e-4.
       All intermediate accelerations ≤ :math:`a_{max}` + 1e-4.

RON-TC-TRAJ-002 — Short-Move (No Full Velocity)
------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-501
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`v_{max}=10.0`, :math:`a_{max}=2.0`. Target = 0.1 (very short
       move; impossible to reach :math:`v_{max}`).
   * - **Stimulus**
     - Run until finished.
   * - **Pass Criterion**
     - Peak velocity during profile < :math:`v_{max}`.
       ``fabs(pos - 0.1) < 0.001`` at finished.
       ``fabs(vel) < 0.001`` at finished.

RON-TC-TRAJ-004 — Online Goal Update
--------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-503
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Moving toward target = 2.0 (halfway through profile, vel ≈ 0.8).
   * - **Stimulus**
     - Call ``ron_trap_set_target(1.0)`` mid-profile. Continue stepping.
   * - **Pass Criterion**
     - No velocity discontinuity > :math:`a_{max} \cdot dt` at update step.
       Profile converges to new target 1.0 with ``fabs(pos - 1.0) < 0.001``.

------------------------------------------------------------------------

Kalman Filter Tests (RON-TC-KF-xxx)
=====================================

RON-TC-KF-001 — Scalar State Estimation Convergence
----------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-600, RON-FR-602
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Scalar system: :math:`A=1.0`, :math:`B=0.0`, :math:`H=1.0`,
       :math:`Q=0.01`, :math:`R=1.0`. :math:`\hat{x}_0=0.0`,
       :math:`P_0=10.0`. True state = 5.0 (constant).
   * - **Stimulus**
     - 100 predict–update cycles with noisy measurement
       :math:`z(k) = 5.0 + \mathcal{N}(0, R^{0.5})` (deterministic seed).
   * - **Pass Criterion**
     - ``fabs(x_hat - 5.0) < 0.5`` after 100 cycles.
       ``P`` monotonically decreasing for first 20 cycles.

RON-TC-KF-006 — Measurement Dropout
--------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-605
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Same scalar system as RON-TC-KF-001, converged after 50 cycles.
   * - **Stimulus**
     - 10 predict steps with ``z_valid = false`` (no update).
   * - **Pass Criterion**
     - State estimate unchanged (predict-only increases P).
       No fault generated.
       ``P`` increases each step during dropout.

------------------------------------------------------------------------

Safety and Fault Tests (RON-TC-SAFE-xxx)
==========================================

RON-TC-SAFE-001 — API Null Pointer Rejection
---------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-001, RON-SR-006
   * - **Level**
     - UT / ENV-HOST
   * - **Stimulus**
     - Call each public API function with NULL as every pointer parameter
       (one test variant per parameter, across all modules).
   * - **Pass Criterion**
     - Return value == ``RON_FAULT_NULL_POINTER`` in all cases.
       No crash, no memory write to address 0.

RON-TC-SAFE-001-FV — Null Pointer Proof
-----------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-001, RON-SR-006
   * - **Level**
     - FV / ENV-FV
   * - **Property**
     - For ``ron_pid_step``, ``ron_pid_init``, and all other public API entry
       points: any NULL pointer argument results in
       ``RON_FAULT_NULL_POINTER`` and no dereference of address 0.
   * - **Pass Criterion**
     - CBMC/Kani reports no null-dereference or undefined behaviour.

RON-TC-SAFE-007 — All Four Fault Conditions Trigger FAULT Flag
---------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-010
   * - **Level**
     - UT / ENV-HOST
   * - **Sub-cases**
     - (a) Input NaN: ``r = NAN``. Expect ``RON_FAULT_INPUT_NAN``.
       (b) Input +Inf: ``y = INFINITY``. Expect ``RON_FAULT_INPUT_NAN``.
       (c) Integral overflow: set :math:`K_i=1000`, very large error for
       1000 steps, ``I_overflow_thresh = 100``. Expect
       ``RON_FAULT_INTEGRAL_OVERFLOW``.
       (d) Invalid config: ``u_min = u_max``. Expect
       ``RON_FAULT_CONFIG_INVALID``.
   * - **Pass Criterion**
     - In all sub-cases: ``RON_STATUS_FAULT`` bit set in status word.
       Safe-state output applied. Fault persists after next ``ron_pid_step``
       without a clear.

RON-TC-SAFE-009 — Fault Latch and Explicit Clear
-------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-012
   * - **Level**
     - UT / ENV-HOST
   * - **Stimulus**
     - Inject NaN input → confirm fault. Inject valid input for 5 steps
       without clearing → confirm fault persists. Call
       ``ron_pid_fault_clear()`` → confirm fault cleared. Run 5 more valid
       steps → confirm normal operation.
   * - **Pass Criterion**
     - Fault persists for 5 steps without clear. Normal after clear.

RON-TC-SAFE-011 — NaN/Inf Detection in All Inputs
--------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-020, RON-PR-012
   * - **Level**
     - UT, FV / ENV-HOST, ENV-FV
   * - **Sub-cases**
     - Test ``r``, ``y``, and ``dt`` each set to NaN or ±Inf individually,
       across PID, Kalman, state-space, and observer modules.
   * - **Pass Criterion**
     - ``RON_FAULT_INPUT_NAN`` (or module equivalent) set in all cases.
       Safe-state output applied. No crash or memory corruption.

RON-TC-SAFE-012 — Integral Round-Off Drift Over Extended Run
-------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-021
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - :math:`K_i=1.0`, constant error = 0.001, :math:`dt=0.001`.
       Run for 1,000,000 steps.
   * - **Pass Criterion**
     - Accumulated integral error vs. analytical value
       (:math:`I_{true} = K_i \cdot 0.001 \cdot 0.001 \cdot 10^6 = 1.0`)
       within 0.1% of :math:`I_{true}`. Verifies round-off does not
       accumulate beyond acceptable bounds in single precision.

------------------------------------------------------------------------

Performance Tests (RON-TC-PERF-xxx)
=====================================

RON-TC-PERF-001 — WCET Analysability (No Unbounded Paths)
----------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-PR-001, RON-PR-002
   * - **Level**
     - FV / ENV-FV
   * - **Method**
     - CBMC (C): ``cbmc --unwind 1 --no-unwinding-assertions`` on
       ``ron_pid_step`` and all other ``step`` functions. Any unbounded
       loop would cause CBMC to report unwinding assertions.
       Kani (Rust): ``cargo kani`` on every ``step`` / ``predict`` /
       ``update`` function; default unwind of 1 is sufficient for O(1)
       functions.
   * - **Pass Criterion**
     - No unwinding assertions. All paths provably terminate in O(1).

RON-TC-PERF-003 — Measured WCET on Integration Target
------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-PR-003
   * - **Level**
     - PT / ENV-TARGET
   * - **Method**
     - Instrument ``ron_pid_step`` call with GPIO toggle or DWT cycle
       counter. Run 10,000 worst-case inputs (all branches exercised:
       saturation, AW, rate limit, normalisation all active). Record
       maximum cycle count. Convert to µs.
   * - **Pass Criterion**
     - Maximum measured WCET ≤ target-specific budget documented in
       RON-SIR-001. Build must include release optimisation (``-O2`` or
       equivalent).

RON-TC-PERF-005 — Single / Double Precision Correctness
---------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-PR-010, RON-PR-011
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - Build both tracks twice: once with ``RON_USE_DOUBLE=OFF`` (float),
       once with ``RON_USE_DOUBLE=ON`` (double). Run RON-TC-PID-001 through
       RON-TC-PID-039 on both. Double-precision build must show lower
       absolute error vs. analytical reference.
   * - **Pass Criterion**
     - Both builds produce outputs within stated tolerances. Double build
       error < half of float build error on RON-TC-SAFE-012.

RON-TC-PERF-006 — Memory Footprint and Zero Heap
--------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-PR-020, RON-PR-021, RON-PR-022, RON-SR-003
   * - **Level**
     - PT / ENV-TARGET or ENV-HOST
   * - **Method**
     - (a) Run test suite under Valgrind (C) / Miri (Rust) with heap
       profiling. Assert zero heap allocations.
       (b) Check ``sizeof(ron_pid_state_t) <= 128`` via static assertion
       (already in ``ron_pid_types.h``).
       (c) Measure ``.text`` section size with ``arm-none-eabi-size`` /
       ``cargo size``. Record in RON-SIR-001.
   * - **Pass Criterion**
     - Zero heap calls reported by Valgrind/Miri.
       ``sizeof(ron_pid_state_t) <= 128`` (compile-time).
       Code size within budget defined at integration.

------------------------------------------------------------------------

Quality Attribute Tests (RON-TC-QUAL-xxx)
==========================================

RON-TC-QUAL-001 — MISRA C / MISRA Rust Compliance
---------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-030
   * - **Level**
     - UT (static) / ENV-HOST
   * - **Method**
     - C: ``cppcheck --addon=misra.py --enable=all --error-exitcode=1 c/src/``.
       Rust: ``cargo clippy -- -D warnings -D clippy::all -D clippy::pedantic``.
       Both run in CI on every commit.
   * - **Pass Criterion**
     - Zero mandatory MISRA C rule violations.
       Zero ``clippy::pedantic`` violations not covered by a deviation record.
       CI step exits with code 0.

RON-TC-QUAL-004 — Free Static Analysis Clean Pass
--------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-SR-033
   * - **Level**
     - UT (static) / ENV-HOST
   * - **Method**
     - C: ``scan-build cmake --build build/`` (Clang Static Analyzer).
       ``cppcheck --bug-hunting c/src/``.
       Rust: ``cargo audit``. ``cargo clippy -D clippy::correctness``.
   * - **Pass Criterion**
     - Zero errors from all tools. Warnings reviewed; safety-relevant ones
       resolved or deviation-recorded.

RON-TC-QUAL-008 — Two-Toolchain Clean Build
---------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-004
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - C: build with GCC and Clang, ``-Wall -Wextra -Wpedantic -Werror``.
       Rust: ``rustc stable`` and ``rustc beta``.
   * - **Pass Criterion**
     - Zero errors and zero warnings under all four compiler invocations.

RON-TC-QUAL-010 — Cyclomatic Complexity ≤ 10
----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-011
   * - **Level**
     - UT (static) / ENV-HOST
   * - **Method**
     - C: ``cppcheck --enable=style --template='{file}:{line}:{severity}:{message}'
       c/src/`` and inspect complexity warnings. Alternatively, ``lizard``
       (free MIT tool) reports per-function McCabe complexity.
       Rust: ``cargo clippy -- -D clippy::cognitive_complexity``.
   * - **Pass Criterion**
     - No function has cyclomatic complexity > 10.

RON-TC-QUAL-014 — 100% Statement and Branch Coverage
------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-022
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - C: build with ``--coverage``, run full test suite, generate report
       with ``lcov --capture`` then ``lcov --remove`` to strip system
       headers, ``genhtml`` for HTML report.
       Rust: ``cargo llvm-cov nextest --workspace``.
   * - **Pass Criterion**
     - Statement coverage = 100%. Branch coverage = 100%.
       Any uncovered line or branch is a test suite defect; the suite
       must be augmented before the build is considered passing.

RON-TC-QUAL-015 — MC/DC on Safety-Critical Conditions
-------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-023
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - ``llvm-cov --mcdc`` (LLVM 18+). Targets: all ``if`` conditions
       inside RON-SR-010 – SR-013 fault detection paths, all AW clamping
       conditions, all saturation branches.
   * - **Pass Criterion**
     - MC/DC coverage = 100% on all designated safety-critical conditions.

RON-TC-QUAL-016-FV — No Data Races (Formal)
---------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-030
   * - **Level**
     - FV / ENV-FV
   * - **Method**
     - Rust: run test suite under ``cargo +nightly miri test`` with
       ``MIRIFLAGS="-Zmiri-check-number-validity"``.
       C: ThreadSanitizer is not applicable (single-context requirement).
       Verify by inspection that no global mutable variables exist in the
       library.
   * - **Pass Criterion**
     - Miri reports zero errors. Grep on library sources for global
       mutable state returns zero matches.

RON-TC-QUAL-017 — Deterministic Reproducibility
-------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-QR-031
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - Run the full test suite twice with identical seeds / inputs. Hash the
       complete output of all ``ron_pid_step`` return values with a simple
       FNV-1a accumulator.
   * - **Pass Criterion**
     - Both runs produce identical hash values.

RON-TC-QUAL-022 — Cross-Compilation Build Succeeds
----------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-DC-005
   * - **Level**
     - PT / ENV-TARGET (or cross-compile on host)
   * - **Method**
     - C: ``cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
       -DCMAKE_C_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
       -B build_arm -S c/`` and ``cmake --build build_arm``.
       Rust: ``cargo build --target thumbv7em-none-eabihf``
       and ``cargo build --target riscv32imc-unknown-none-elf``.
   * - **Pass Criterion**
     - All four builds succeed with zero errors and zero warnings.
       Build artefacts can be linked into a minimal bare-metal harness.

------------------------------------------------------------------------

Health Monitor Tests (RON-TC-HLTH-xxx)
========================================

RON-TC-HLTH-002 — Output Stuck Detection
-----------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-901 (a)
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - ``t_sat_max = 0.5 s``, ``dt = 0.01 s`` (→ 50 steps threshold).
   * - **Stimulus**
     - Feed the health monitor ``u = u_max`` for 60 steps.
   * - **Pass Criterion**
     - ``HEALTH_OUTPUT_STUCK`` not set at step 49.
       ``HEALTH_OUTPUT_STUCK`` set at step 50.
       Callback invoked exactly once.

RON-TC-HLTH-008 — Health Monitor Does Not Modify Controller
-------------------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-903
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - Run 200 steps with health monitor attached (all thresholds set to
       trigger). Record controller output sequence. Run same 200 steps
       with health monitor NOT attached. Compare outputs.
   * - **Pass Criterion**
     - Identical output sequences in both runs (within FLT_EPSILON).

------------------------------------------------------------------------

Metrics Tests (RON-TC-MET-xxx)
================================

RON-TC-MET-002 — IAE / ISE / ITAE Computation
-----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-951
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Metrics accumulator in cumulative mode.
   * - **Stimulus**
     - 100 steps with constant error :math:`e = 0.5`, :math:`dt = 0.01`.
   * - **Expected Output**
     - :math:`IAE = 0.5 \times 100 \times 0.01 = 0.5`.
       :math:`ISE = 0.25 \times 100 \times 0.01 = 0.25`.
       :math:`ITAE = 0.5 \times 0.01 \times \sum_{k=1}^{100} k \cdot 0.01
       = 0.5 \times 0.01 \times 0.01 \times 5050 = 0.2525`.
   * - **Pass Criterion**
     - All three values within 0.01% of analytical result.

RON-TC-MET-006 — Zero Overhead When Disabled
----------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-953
   * - **Level**
     - UT / ENV-HOST
   * - **Method**
     - Run 1,000 ``ron_pid_step`` calls with metrics enabled vs. disabled.
       Compare outputs.
   * - **Pass Criterion**
     - Controller outputs identical. Metrics state unchanged when disabled.

------------------------------------------------------------------------

Auto-Tuning Tests (RON-TC-AT-xxx)
====================================

RON-TC-AT-003 — Ku/Tu Estimation Accuracy
-------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-802
   * - **Level**
     - UT / ENV-HOST
   * - **Preconditions**
     - Simulated first-order plant with known :math:`K_u = 4.0`,
       :math:`T_u = 0.5 s`. Relay amplitude :math:`d=0.5`,
       hysteresis = 0.05.
   * - **Stimulus**
     - Run auto-tuner against simulated plant for 5 full cycles.
   * - **Pass Criterion**
     - Estimated :math:`K_u` within 10% of true value.
       Estimated :math:`T_u` within 10% of true value.
   * - **Note**
     - Provide a deterministic simulated plant (first-order IIR) so the
       test is reproducible without physical hardware.

RON-TC-AT-005 — Gains Applied Only on Explicit Call
-----------------------------------------------------

.. list-table::
   :widths: 20 80

   * - **Requirement**
     - RON-FR-804
   * - **Level**
     - UT / ENV-HOST
   * - **Stimulus**
     - Complete an auto-tuning run. Check PID gains BEFORE calling
       ``ron_autotune_apply``. Call ``ron_autotune_apply``. Check PID
       gains AFTER.
   * - **Pass Criterion**
     - Gains before == original gains. Gains after == computed tuning
       gains (within 0.1%).

------------------------------------------------------------------------

Formal Verification Summary
=============================

The following table lists all formal verification harnesses and their
expected property proofs.

.. list-table::
   :header-rows: 1
   :widths: 22 12 20 46

   * - Test Case ID
     - Tool
     - Bound (unwind)
     - Property Proved
   * - RON-TC-PID-015-FV
     - Kani / CBMC
     - 32
     - ``u ∈ [u_min, u_max]`` for all finite inputs
   * - RON-TC-PID-022-FV
     - Kani / CBMC
     - 32
     - Back-calc AW correction bounded; integral bounded
   * - RON-TC-PID-026-FV
     - Kani / CBMC
     - 32
     - ``|I| ≤ I_max`` at all times
   * - RON-TC-PID-036-FV
     - Kani / CBMC
     - 1
     - No writes to global mutable state in ``ron_pid_step``
   * - RON-TC-CASC-004-FV
     - Kani / CBMC
     - 32
     - Outer integral bounded when inner saturated
   * - RON-TC-KF-008-FV
     - Kani / CBMC
     - 8
     - No heap allocation in any Kalman function
   * - RON-TC-SS-004-FV
     - Kani / CBMC
     - 32
     - SS controller output ∈ [u_min, u_max]
   * - RON-TC-AT-007-FV
     - Kani / CBMC
     - 32
     - Relay output ∈ [u_bias−d, u_bias+d]
   * - RON-TC-SAFE-001-FV
     - Kani / CBMC
     - 1
     - NULL arg → ``RON_FAULT_NULL_POINTER``, no deref
   * - RON-TC-SAFE-002-FV
     - Kani / CBMC
     - 1
     - No division when denominator = 0
   * - RON-TC-SAFE-003-FV
     - Kani / CBMC
     - 1
     - No calls to ``malloc``/``free``/``alloc``
   * - RON-TC-SAFE-004-FV
     - Kani / CBMC
     - 1
     - No recursive call graph reachable from any entry point
   * - RON-TC-SAFE-005-FV
     - Kani / CBMC
     - 32
     - All array accesses within declared bounds
   * - RON-TC-SAFE-006-FV
     - Kani / CBMC
     - 1
     - All pointer dereferences preceded by NULL check
   * - RON-TC-SAFE-007-FV
     - Kani / CBMC
     - 32
     - NaN/Inf inputs → fault flag set; safe output applied
   * - RON-TC-SAFE-011-FV
     - Kani / CBMC
     - 32
     - NaN/Inf anywhere in computation → fault, no NaN in output
   * - RON-TC-SAFE-013-FV
     - Kani / CBMC
     - 32
     - ``|I| ≤ I_overflow_thresh`` or FAULT_INTEGRAL_OVERFLOW set
   * - RON-TC-PERF-002-FV
     - Kani / CBMC
     - 1
     - All ``step`` functions terminate in O(1) (no unbounded loops)
   * - RON-TC-PERF-004-FV
     - Kani / CBMC
     - 1
     - No blocking calls (no sleep/wait/yield) in any library function
   * - RON-TC-QUAL-016-FV
     - Miri (Rust)
     - N/A
     - No UB, no data races in full test suite

------------------------------------------------------------------------

Requirement Coverage Verification
===================================

The following table summarises coverage by requirement group. Every group
must reach 100% before a release is considered verified.

.. list-table::
   :header-rows: 1
   :widths: 30 15 15 40

   * - Requirement Group
     - Total Reqs
     - Test Cases
     - Status
   * - RON-FR-001 – FR-071 (PID core)
     - 34
     - 39 UT + 5 FV
     - All mapped
   * - RON-FR-100 – FR-131 (Filters)
     - 14
     - 17 UT
     - All mapped
   * - RON-FR-200 – FR-205 (Feed-forward)
     - 6
     - 9 UT
     - All mapped
   * - RON-FR-300 – FR-306 (Gain scheduling)
     - 7
     - 8 UT
     - All mapped
   * - RON-FR-400 – FR-406 (Cascade)
     - 7
     - 7 IT + 1 FV
     - All mapped
   * - RON-FR-500 – FR-513 (Trajectory)
     - 11
     - 8 UT
     - All mapped
   * - RON-FR-600 – FR-607 (Kalman)
     - 8
     - 8 UT + 1 FV
     - All mapped
   * - RON-FR-700 – FR-723 (SS + observer)
     - 9
     - 9 UT + 1 FV
     - All mapped
   * - RON-FR-800 – FR-807 (Auto-tune)
     - 8
     - 8 UT/IT + 1 FV
     - All mapped
   * - RON-FR-900 – FR-905 (Health)
     - 6
     - 10 UT
     - All mapped
   * - RON-FR-950 – FR-954 (Metrics)
     - 5
     - 7 UT
     - All mapped
   * - RON-PR-001 – PR-022 (Performance)
     - 10
     - 7 PT + 3 FV
     - All mapped
   * - RON-SR-001 – SR-022 (Safety)
     - 13
     - 13 UT + 10 FV
     - All mapped
   * - RON-SR-030 – SR-033 (Standards)
     - 4
     - 4 static/review
     - All mapped
   * - RON-QR-001 – QR-031 (Quality)
     - 10
     - 10 UT/static/review
     - All mapped
   * - RON-DC-001 – DC-005 (Constraints)
     - 5
     - 5 static/review/PT
     - All mapped
   * - **Total**
     - **157**
     - **~200**
     - **100% mapped**

------------------------------------------------------------------------

Test Execution Order and Dependencies
=======================================

Some tests depend on others passing first. The recommended CI execution
order is:

1. **Static analysis** (RON-TC-QUAL-001, QUAL-004, QUAL-007 – QUAL-012,
   QUAL-019 – QUAL-021): must pass before any functional tests are run;
   these catch undefined behaviour before execution.

2. **Formal verification** (all ``-FV`` harnesses): run early as they are
   independent of test framework.

3. **Unit tests — safety and platform layer** (RON-TC-SAFE-xxx): run before
   module-level tests; if fundamental fault detection is broken, other tests
   may give misleading results.

4. **Unit tests — PID module** (RON-TC-PID-xxx).

5. **Unit tests — all other modules** (FILT, FF, GS, TRAJ, KF, SS, AT,
   HLTH, MET): can run in parallel.

6. **Integration tests** (RON-TC-CASC-xxx, RON-TC-PID-035,
   RON-TC-PID-037, RON-TC-AT-001): require unit tests to pass first.

7. **Quality / reproducibility** (RON-TC-QUAL-013 – QUAL-018).

8. **Performance tests** (RON-TC-PERF-xxx, RON-TC-QUAL-022): run last
   and on target hardware; require all prior tests to pass.

------------------------------------------------------------------------

Open Items
==========

.. list-table::
   :header-rows: 1
   :widths: 10 60 30

   * - ID
     - Description
     - Resolution Target
   * - OI-TP-01
     - RON-TC-PERF-003 acceptance values (WCET budget in µs) TBD pending
       integration target selection. Values to be recorded in RON-SIR-001.
     - Integration phase
   * - OI-TP-02
     - Property-based test harnesses (``proptest``) to be written for
       RON-TC-PID-015 – PID-026 to supplement hand-crafted stimuli with
       random exploration.
     - Implementation phase
   * - OI-TP-03
     - Simulated plant model for RON-TC-AT-003 to be reviewed and agreed
       with a control engineer before test is locked.
     - Implementation phase
   * - OI-TP-04
     - ``cargo-nextest`` JUnit XML profile for CI integration to be
       configured in ``rust/.config/nextest.toml``.
     - Implementation phase
   * - OI-TP-05
     - OTAWA static WCET analysis to be evaluated for ARM Cortex-M target;
       results to supplement RON-TC-PERF-003 measurement.
     - Integration phase

------------------------------------------------------------------------

*End of Document — RON-TP-001 v1.0.0*

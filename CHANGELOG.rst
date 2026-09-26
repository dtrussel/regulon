.. ============================================================
.. CHANGELOG — Regulon Control Systems Library
.. ============================================================

CHANGELOG
=========

All notable changes to the **Regulon Control Systems Library** are documented
here following `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
conventions.  Version numbers follow `Semantic Versioning <https://semver.org/>`_.

------------------------------------------------------------------------

`Unreleased`_
=============

Changes on ``main`` since the ``v0.1.0`` tag.

Changed
-------
- **Breaking default:** the compile-time dimension bounds in
  ``ron_platform.h`` were lowered to cut estimator/optimal-control stack
  usage by 4.2x (largest frame 2448 B -> 576 B). State bounds
  (``RON_KF_MAX_STATES``, ``RON_SS_MAX_STATES``, ``RON_LQR_MAX_STATES``,
  ``RON_MAT_MAX_DIM``) go 8 -> 4; input and output bounds 4 -> 2
  (``RON_KF_MAX_MEASUREMENTS`` stays 4). Larger
  plants must set the macros explicitly; undersized bounds fail at compile
  time or at configuration validation, never silently.
- The DARE solver and Kalman covariance update no longer materialise
  transposes or extra scratch matrices (new internal ``ron_mat_mul_ta``).
- The biquad coefficient designers compute sine/cosine with an internal
  fixed-iteration series instead of libm. The library now includes only
  freestanding headers and needs **no C library at all** (RON-DC-002).
- Numeric helpers the modules each carried a private copy of now live once
  in the internal ``ron_util.c`` (part of the mandatory baseline): the
  finite check (15 copies, 3 implementations), the output rate limiter (4
  identical copies), square root, sign, pi, and the LQR/LQG strided matrix
  zeroing (now ``ron_mat_zero``). The trapezoidal and S-curve planners now use
  the 30-step square root the matrix module already used instead of 16/18
  steps, which is more accurate for large ``a_max * distance`` products.
  A minimum-footprint build links six baseline objects instead of five.
- ``regulon-c/scripts/verify_pid.ps1`` renamed to ``verify.ps1`` (it already
  covered the whole library) and its cppcheck suppressions synced with CI.
- The CMake package declares ``SameMinorVersion`` until 1.0, so
  ``find_package(regulon 0.1)`` no longer accepts a 0.2 that may break it.
- ``ron_autotune_apply`` takes ``const ron_at_t *`` (source-compatible).
- The API reference moved from Doxygen HTML to a Sphinx + Breathe site that
  also renders the specifications and usage guides, with requirement/test
  IDs on API entries linking into the specs.

Added
-----
- ``ron_pid_config_from_isa()``: sets a configuration's gains from the ideal
  (ISA) form (``Ki = Kp/Ti``, ``Kd = Kp*Td``; ``Ti = +Inf`` disables integral
  action). RON-FR-002 required the ISA form but the C library had no support
  for it, and RON-TC-PID-002 did the conversion in the test itself.
- Zephyr module (``zephyr/``, ``west.yml``): Kconfig options mirroring the
  CMake ``RON_ENABLE_<MODULE>`` switches with dependency ``select``\ s, a
  PID sample, and an on-target ztest suite run nightly with ``twister``
  under QEMU (Cortex-M3/M33) plus cross-builds for Cortex-M4F/M7/nRF52840.
- Optional user ``ron_config.h`` (or ``-DRON_CONFIG_HEADER``) to override
  platform defaults.
- CI enforces 100% MC/DC (``-fcoverage-mcdc``, RON-TC-QUAL-015), which the
  test plan specified but no job measured. The one uncovered condition,
  clamping anti-windup at exactly zero error, gained a test.
- ``regulon-c/scripts/check_traceability.py``, run in CI: fails when code or
  tests use a test ID the test plan does not define, cite a requirement the
  SRS does not define, or when a requirement has no test in the plan.
- CI gates: per-frame stack budget (``check_stack_usage.sh``, 768 B), no-libm
  symbol check on every cross build (``check_no_libm.sh``), and a
  ``sphinx-build -W`` documentation build.
- API documentation for the 66 public functions that had none.
- MISRA deviation records DEV-005 (Rule 20.9) and DEV-006 (Rules 2.3/2.4).

Removed
-------
- The completed C11 phase plans and roadmap under ``docs/plans/``; the
  per-phase development history they and this changelog carried remains in
  git history.
- Newlib/picolibc header discovery in the cross toolchain files and its
  ``RON_*_NEWLIB_INCLUDE`` / ``RON_*_ALLOW_HEADER_SHIM`` cache options
  (now ignored), and the ``math.h`` shims.

Fixed
-----
- ``ron_lqg.h`` cited RON-TC-LQG-010, which does not exist; the case is
  RON-TC-LQG-010-FV.
- ``regulon.pc`` still listed ``Libs.private: -lm`` after the libm removal,
  so static pkg-config consumers linked a math library the archive never
  uses.
- The cppcheck/MISRA CI step discarded cppcheck's exit status (missing
  ``pipefail``), so no finding could fail the build. The gate is now
  enforced and the findings it hid are resolved.
- ``ron_pid_core.c``: ``u_final`` is initialised explicitly (GCC
  ``-Wmaybe-uninitialized`` at ``-O2``/``-Os``; MISRA Rule 9.1). No behaviour
  change.
- Latent reStructuredText defects in the specifications surfaced by the
  first rendered build.
- Specifications: the four documents were titled and scoped as the "PID
  Controller Module"; the IS directory tree and build section described a
  PID-only ``c/`` layout, version 1.0.0, LQR/LQG off by default and the wrong
  RISC-V compiler. The build section now includes the real option and
  toolchain files. RON-TC-CASC-008 – CASC-012 ran in the C suite but were
  missing from the test plan; they are now recorded there.
- ``IS_ControlLib.rst`` API listings now match the headers: the cascade
  section used a ``ron_cascade_t`` type, a one-value ``set_mode`` and a
  16-bit status word that never shipped, and omitted ``get_state`` and
  ``fault_clear``; ``ron_pid_set_config``, ``ron_at_phase_t`` and the
  ``const`` on ``ron_gs_init`` were missing.
- ``regulon-c/AGENTS.md`` still limited work to the PID module and gave
  PID-only lint/analysis/proof commands; it now mirrors the CI gates over
  the source manifests.

------------------------------------------------------------------------

`0.1.0`_ - 2026-08-07
======================

First tagged release: the complete C11 implementation (``regulon-c/``).
The Rust implementation (``regulon-rs/``) is early-stage (PID and filters
only) and not part of this release; see ``docs/plans/rust/rust-first-rollout.md``.

- **C11 library complete**: all 14 modules (PID, filters, feed-forward,
  gain scheduling, trajectory generators, cascade control, Kalman filter,
  state-space controller + Luenberger observer, LQR, LQG, relay
  auto-tuner, health monitor, runtime metrics, aggregate header) are
  implemented with 100% statement/branch coverage, CBMC formal proofs
  where the test plan calls for them, and full MISRA C:2023 traceability.
- **Installable**: a ``find_package(regulon)``-consumable CMake package
  and a ``pkg-config`` file, in addition to the in-tree
  ``add_subdirectory`` build; per-module ``RON_ENABLE_<MODULE>`` selection.
- **Portable**: ARM Cortex-M (GCC and Clang) and RISC-V (``rv32imc``)
  cross-compile smoke builds, all exercised in CI.
- **Documented**: ``README.md``, Doxygen API reference,
  ``CONTRIBUTING.md``/``SECURITY.md``, issue/PR templates, and the
  SRS/SADS/IS/TP specification set.
- **Measured**: a host timing benchmark against the ``RON-PR-003`` 10 kHz
  design-target budget, and a MISRA deviations record covering the full
  source set.

.. _Unreleased: https://github.com/dtrussel/regulon/compare/v0.1.0...HEAD
.. _0.1.0: https://github.com/dtrussel/regulon/releases/tag/v0.1.0

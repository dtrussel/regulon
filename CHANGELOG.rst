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
- ``regulon-c/scripts/verify_pid.ps1`` renamed to ``verify.ps1`` (it already
  covered the whole library) and its cppcheck suppressions synced with CI.
- ``ron_autotune_apply`` takes ``const ron_at_t *`` (source-compatible).
- The API reference moved from Doxygen HTML to a Sphinx + Breathe site that
  also renders the specifications and usage guides, with requirement/test
  IDs on API entries linking into the specs.

Added
-----
- Zephyr module (``zephyr/``, ``west.yml``): Kconfig options mirroring the
  CMake ``RON_ENABLE_<MODULE>`` switches with dependency ``select``\ s, a
  PID sample, and an on-target ztest suite run nightly with ``twister``
  under QEMU (Cortex-M3/M33) plus cross-builds for Cortex-M4F/M7/nRF52840.
- Optional user ``ron_config.h`` (or ``-DRON_CONFIG_HEADER``) to override
  platform defaults.
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
- The cppcheck/MISRA CI step discarded cppcheck's exit status (missing
  ``pipefail``), so no finding could fail the build. The gate is now
  enforced and the findings it hid are resolved.
- ``ron_pid_core.c``: ``u_final`` is initialised explicitly (GCC
  ``-Wmaybe-uninitialized`` at ``-O2``/``-Os``; MISRA Rule 9.1). No behaviour
  change.
- Latent reStructuredText defects in the specifications surfaced by the
  first rendered build.

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

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

Changes on ``main`` since the ``v0.1.0`` tag, to be released as **0.2.0**
(``project(VERSION)`` and ``RON_VERSION_*`` already read 0.2.0). This release
breaks source compatibility; see *Migrating from 0.1* below.

Migrating from 0.1
------------------
Public names now follow one convention (IS "C Track — Naming Conventions"):
one module token across header, types, constants and functions, and
``_get_state`` / ``_get_results`` / ``_get_status`` / ``_validate`` for the
read-back and validation roles, and the state-space controller and LQR share
one state-estimator component. Behaviour is unchanged; applying this table is
the whole migration.

.. list-table::
   :header-rows: 1

   * - 0.1
     - 0.2
   * - ``ron_pid_instance_t``
     - ``ron_pid_t``
   * - ``ron_cascade_instance_t``
     - ``ron_cascade_t``
   * - ``ron_at_t``, ``ron_at_config_t``, ``ron_at_state_t``,
       ``ron_at_rule_t``, ``ron_at_phase_t``
     - ``ron_autotune_t``, ``ron_autotune_config_t``,
       ``ron_autotune_state_t``, ``ron_autotune_rule_t``,
       ``ron_autotune_phase_t``
   * - ``RON_AT_*`` (e.g. ``RON_AT_RULE_ZN``, ``RON_AT_DONE``)
     - ``RON_AUTOTUNE_*`` (``RON_AUTOTUNE_RULE_ZN``, ``RON_AUTOTUNE_DONE``)
   * - ``ron_autotune_results()``
     - ``ron_autotune_get_results()``
   * - ``ron_metrics_get()``
     - ``ron_metrics_get_results()``
   * - ``ron_health_get()``
     - ``ron_health_get_status()``
   * - ``ron_gs_init()`` (it only ever validated the table)
     - ``ron_gs_table_validate()``
   * - ``ron_ss_source_t`` / ``RON_SS_SOURCE_*``, ``ron_lqr_source_t`` /
       ``RON_LQR_SOURCE_*``
     - ``ron_estimator_source_t`` / ``RON_ESTIMATOR_*``
   * - ``cfg.source``, ``cfg.x_ext``, ``cfg.obs_cfg``, ``cfg.kf_cfg`` of
       ``ron_ss_config_t`` / ``ron_lqr_config_t``
     - ``cfg.est.source``, ``cfg.est.x_ext``, ``cfg.est.obs_cfg``,
       ``cfg.est.kf_cfg``
   * - ``ss.observer`` / ``ss.kalman`` (likewise ``lqr.``)
     - ``ss.est.observer`` / ``ss.est.kalman``
   * - ``ron_ss_observer_step(&ss, …)``, ``ron_ss_kalman_predict(&ss, …)``,
       ``ron_ss_kalman_update(&ss, …)`` (likewise ``ron_lqr_*``)
     - ``ron_estimator_observer_step(&ss.est, …)``,
       ``ron_estimator_kalman_predict(&ss.est, …)``,
       ``ron_estimator_kalman_update(&ss.est, …)``

The lowered default dimension bounds (below) are the other breaking change:
plants with more than 4 states or 2 inputs/outputs must now set the
``RON_*_MAX_*`` macros.

Changed
-------
- **Breaking:** the public API renames in the migration table above.
- **Breaking:** the state-space controller and the LQR each carried their own
  copy of the state-estimate plumbing (source enum, validation, embedded
  observer/Kalman init and reset, estimate fetch, and three wrappers each).
  Both now embed one ``ron_estimator`` component (``ron_estimator.h``,
  compiled with ``RON_ENABLE_STATESPACE``); see the migration table.
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
- ``check_manifest.sh`` also fails when ``RON_VERSION_*`` in
  ``ron_platform.h`` and ``project(VERSION)`` disagree.
- ``ron_autotune_apply`` takes ``const ron_autotune_t *`` (source-compatible).
- The API reference moved from Doxygen HTML to a Sphinx + Breathe site that
  also renders the specifications and usage guides, with requirement/test
  IDs on API entries linking into the specs.

Added
-----
- ``ron_trap_reset``/``ron_trap_get_state`` and ``ron_scurve_reset``/
  ``ron_scurve_get_state`` (RON-FR-514/515): the trajectory generators were
  the only modules with no reset or state read-back.
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
- Rust: the PID feed-forward path now covers every RON-FR-201 mode: velocity
  and acceleration (filtered finite differences with an independent
  ``derivative_filter`` bandwidth, RON-FR-202) and external
  (``Pid::step_with_feed_forward``), plus ``Pid::set_feed_forward`` and
  ``Pid::last_feed_forward``. RON-TC-FF-001 – FF-009 run in the Rust suite.
  Breaking: ``FeedForwardConfig::static_gain`` is now the signed ``gain``
  (as in C) and ``FeedForwardMode::Reserved`` is gone.
- Rust: gain scheduling (``gain_sched``, RON-FR-300 – FR-306). A
  ``GainSchedule<N>`` holds up to ``GS_MAX_BREAKPOINTS`` (16) breakpoint/
  ``PidConfig`` pairs, validated once at construction; ``apply`` updates a
  ``Pid`` atomically by hard switching (optionally resetting the integrator)
  or by interpolating the gains. RON-TC-GS-001 – GS-008 run in the Rust
  suite.
- Rust: cascade control (``cascade``, RON-FR-400 – FR-406). ``Cascade``
  owns an outer and an inner ``Pid``, feeds the outer output to the inner
  setpoint, propagates inner saturation to the outer integrator by
  back-calculation, switches modes in bumpless order and reports a
  ``CascadeStatus`` whose ``bits()`` match the C 32-bit layout.
  RON-TC-CASC-001 – CASC-012 run in the Rust suite.
- Rust: trajectory generators (``trajectory``, RON-FR-500 – FR-503,
  FR-510 – FR-515): ``Trapezoidal`` and the seven-phase jerk-limited
  ``SCurve``, with mid-move re-targeting, hold/resume, reset, latched faults
  and full state read-back, using the same bounded, libm-free Newton roots
  as C (``platform::sqrt``). RON-TC-TRAJ-001 – TRAJ-010 run in the Rust
  suite.
- Rust: loop-health monitor (``health``, RON-FR-900 – FR-905).
  ``HealthMonitor`` latches output-stuck, diverging, oscillating,
  sensor-dropout and setpoint-unreachable conditions, calls an optional
  ``fn`` callback on each first activation and also returns the newly
  latched bits from ``step``. RON-TC-HLTH-001 – HLTH-010 run in the Rust
  suite.
- ``check_traceability.py`` also scans the Rust crate (``regulon-rs/``), so
  Rust tests and annotations are held to the same test-plan and SRS IDs.

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
  ``fault_clear``; ``ron_pid_set_config``, ``ron_autotune_phase_t`` and the
  ``const`` on ``ron_gs_table_validate`` were missing.
- Specification and deviation-record revision histories named no author
  ("TBD"); they now record ``dtrussel``, and the MISRA record states that
  each revision's entry is its approval. RON-FR-061 no longer calls the
  (necessarily visible) instance structs "opaque". The IS/TP Rust sections
  are marked as describing the target, since ``regulon-sys``,
  ``ci_rust.yml`` and the Rust deviation record do not exist yet.
- Blanket annotations such as ``Satisfies: RON-FR-001 – RON-FR-071`` on
  ``ron_pid_core_step`` claimed requirements the function does not implement
  (the ISA form among them); they now name what each function satisfies.
- ``verify.ps1`` measures and enforces MC/DC when clang 18+ is available, as
  CI does.
- Rust: the filter tests did not compile with the ``double_precision``
  feature (an ``f32`` literal in RON-TC-FILT-005).
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

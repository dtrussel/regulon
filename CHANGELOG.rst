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
- **Rust behaviour:** the PID fault path matches C. A latched fault is
  reported before the arguments are checked; a non-positive or non-finite
  ``dt`` returns ``RonError::InvalidArgument`` without latching (it latched
  ``INPUT_NOT_FINITE`` before); and a faulted step no longer overwrites the
  output history with the safe-state value. The new ``Pid::output()``
  returns that value, and ``Cascade`` feeds it forward (``Cascade::last_output``
  is no longer ``const``). Tests: RON-TC-SAFE-008 (Rust test added, and its
  missing TP definition), RON-TC-SAFE-011, RON-TC-CASC-010.
- **Behaviour:** ``ron_ss_step``, ``ron_lqr_step`` and ``ron_lqg_step`` now
  latch runtime faults as the PID does (RON-SR-012, SR-013). The fault is
  kept in the existing ``faults`` field, which was never set before; the
  faulting step and every later one hold the last output, report
  ``RON_STATUS_FAULT`` and return the latched fault until the new
  ``ron_ss_fault_clear()`` / ``ron_lqr_fault_clear()`` /
  ``ron_lqg_fault_clear()`` or ``_reset()`` is called. Previously a rejected
  step left ``u`` and ``status`` unwritten and the next step ran normally.
  Rust: ``StateSpace``, ``Lqr`` and ``Lqg`` latch the same way; ``step``
  returns ``RonError::Fault`` with the latched ``PidFault`` bits (instead of
  ``InvalidArgument`` / ``Numerical``) until ``clear_fault()`` or ``reset()``,
  and ``fault()`` / ``output()`` read the register and the held output.
  Tests: RON-TC-SS-010, RON-TC-LQR-011, RON-TC-LQG-011.
- The specifications no longer plan a ``regulon-sys`` C-ABI crate:
  ``regulon-rs`` is for Rust-native use only and C users use the C11 track.
  The IS drops the C-ABI naming rows and the ``#[repr(C)]`` rule, states that
  the Rust crate has no ``unsafe``, shows the real ``regulon-rs/`` layout and
  closes OI-08; the TP records the ``ci_rust.yml`` jobs.
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
- Rust: performance metrics (``metrics``, RON-FR-950 – FR-954).
  ``Metrics`` accumulates IAE, ISE, ITAE, peak overshoot, rise and settling
  time, cumulatively or per window, is created disabled and restarts the
  transient metrics on each setpoint step. Rise and settling time are
  ``Option`` rather than C's ``-1`` sentinel. RON-TC-MET-001 – MET-007 run
  in the Rust suite.
- Rust: relay auto-tuner (``autotune``, RON-FR-800 – FR-807).
  ``Autotuner`` measures ``Ku``/``Tu`` by relay feedback, derives gains by
  Ziegler-Nichols, Tyreus-Luyben, some- or no-overshoot rules, and changes
  the ``Pid`` only through ``apply``; ``abort`` restores the captured gains
  and mode. Unlike C, aborting a run that never started leaves the
  controller untouched, and ``start`` clears any previous run.
  RON-TC-AT-001 – AT-008 run in the Rust suite, with a Kani harness for
  RON-TC-AT-007-FV.
- Rust: const-generic ``Matrix<R, C>`` (products, transpose, Cholesky
  factor/solve, SPD inverse; dimensions checked by the compiler and bounded
  by ``MATRIX_MAX_DIM``) and the Luenberger ``Observer<N, M, P>``
  (RON-FR-720 – FR-723, RON-TC-SS-006 – SS-009). A step whose estimate would
  not be finite returns ``RonError::Numerical`` and keeps the previous
  estimate; C commits the non-finite estimate.
- Rust: Kalman filter ``Kalman<N, M, P>`` (RON-FR-600 – FR-607,
  RON-TC-KF-001 – KF-008): predict/update, scalar gain for one measurement
  and Cholesky solve otherwise, Joseph form, steady-state gain, and dropout
  as ``update(None)``. A non-positive-definite innovation covariance or a
  non-finite result returns ``RonError::Numerical`` and keeps the previous
  estimate and covariance; C commits the non-finite state.
- Rust: shared state estimator ``Estimator<N, M, P>`` (RON-FR-701,
  RON-FR-734, RON-TC-EST-001 – EST-003) selecting an external vector, an
  embedded ``Observer`` or an embedded ``Kalman``. The external estimate is
  supplied with ``set_external`` (rejected if not finite) instead of C's
  live pointer, so reading the state cannot fail.
- Rust: state-feedback controller ``StateSpace<N, M, P>`` (RON-FR-700 –
  FR-704, RON-TC-SS-001 – SS-005, SS-009): ``u = -K x_hat + K_r r`` with
  optional integral augmentation, saturation, rate limiting and runtime
  gain updates, over an embedded ``Estimator``. A step whose output would
  overflow leaves the integral untouched (C winds it before the check).
- Rust: MIMO LQR ``Lqr<N, U, Y>`` (RON-FR-730 – FR-739, RON-TC-LQR-001 –
  LQR-009): pre-computed or DARE-solved gain (``solve_dare``, bounded value
  iteration, reusable by LQG), per-input integral augmentation, saturation
  and rate limiting, runtime gain updates and the DARE solution ``P``. DARE
  failures return ``RonError::Numerical``. ``A``/``B`` are only required
  for the DARE solve; C also requires them whenever an embedded estimator
  is used, although the estimator carries its own model.
- Rust: LQG ``Lqg<N, U, Y>`` (RON-FR-750 – FR-759, RON-TC-LQG-001 –
  LQG-009): an LQR law (pre-computed or DARE via ``solve_dare``) driven by
  an embedded ``Kalman`` built from the noise model, designed independently
  by the separation principle. Heap freedom (RON-TC-LQG-010-FV) holds by
  construction in the ``no_std`` crate.
- Rust: moving-average (``MovingAverage<M>``, RON-FR-115 – FR-117,
  RON-TC-FILT-008 – FILT-010) and cascaded biquad filters (``Biquad<S>``,
  RON-FR-120 – FR-123, RON-TC-FILT-011 – FILT-015) with low-pass,
  high-pass, band-pass and notch design helpers (libm-free sine/cosine, as
  in C) and runtime notch retuning. Kani harnesses cover RON-TC-FILT-009-FV
  and FILT-012-FV. A step whose output would not be finite leaves the
  window/section state untouched (C updates it first).
- ``.github/workflows/ci_rust.yml``: Rust track CI for ``regulon-rs/`` —
  rustfmt, pedantic clippy and tests in single and double precision (tests
  also on beta), a ``thumbv7em-none-eabihf`` release build, Kani proofs,
  ``cargo audit``, a ``cargo-llvm-cov`` coverage report (not yet enforced)
  and the traceability check.
- Safe-state output policy for the state-space, LQR and LQG controllers
  (RON-SR-011), as the PID has: ``safe_policy`` (``RON_SAFE_HOLD_LAST``, the
  zero-initialised default, ``RON_SAFE_ZERO`` or ``RON_SAFE_CONSTANT``) and
  ``safe_value`` (per input for LQR/LQG) select what a latched step writes to
  ``u``, clamped to the output limits; the output history is kept. Rust:
  ``safe_policy`` / ``safe_value`` on ``StateSpaceConfig``, ``LqrConfig`` and
  ``LqgConfig``, and ``output()`` returns the safe-state output while
  faulted. Tests: RON-TC-SS-011, RON-TC-LQR-012, RON-TC-LQG-012.
- LQG: ``RON_LQG_GAIN_DARE_BOTH`` (Rust: ``LqgGain::DareBoth``) solves the
  steady-state Kalman gain at init as well as the LQR gain, as RON-FR-756
  requires: the dual DARE in ``(A^T, H^T, Q_noise, R_noise)`` gives the
  a-priori covariance ``P``, and the filter runs on the fixed
  ``K_f = P H^T (H P H^T + R_noise)^-1``. Previously no mode solved the
  estimator gain; the existing modes are unchanged. The C DARE solver gains a
  matrix-level entry point, ``ron_lqr_dare_solve_mat()`` (private), for the
  dual problem's measurement-sized operands. RON-TC-LQG-006 checks both
  gains against reference values and against the gain a time-varying filter
  converges to. The ``lqr_lqg_control`` example now uses this mode and prints
  both solved gains.
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
- Rust: clippy 1.98's pedantic ``manual_midpoint`` lint failed the biquad
  high-pass design and the libm-free square root; both use
  ``RonFloat::midpoint`` (overflow-safe; stable since Rust 1.85).
- Rust: the biquad design helpers failed pedantic clippy in
  ``double_precision`` builds (``useless_conversion`` on the ``f64`` widening).
- ``ron_lqr_init`` / ``ron_lqg_init`` documented a failed DARE as
  ``RON_FAULT_OUTPUT_NAN``; it returns ``RON_FAULT_CONFIG_INVALID``.
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
- Rust PID: an integral increment that overflowed (finite error, extreme
  ``ki * dt``) left ``inf - inf = NaN`` in the compensated-sum carry, so the
  next ordinary step faulted with ``OUTPUT_NOT_FINITE``. The carry is now
  dropped when a term is not finite (regression test under RON-TC-SAFE-012).
  Found by the first Kani run of ``ron_tc_pid_015_fv``, whose input bounds
  now also match RON-TC-PID-015-FV and the C harness.
- Rust: three rate-limiter tests carried RON-TC-FILT-008 – FILT-010, which
  the test plan defines as moving-average tests; they are now filed under
  FILT-003/-004.
- Rust: the filter tests did not compile with the ``double_precision``
  feature (an ``f32`` literal in RON-TC-FILT-005).
- ``regulon-c/AGENTS.md`` still limited work to the PID module and gave
  PID-only lint/analysis/proof commands; it now mirrors the CI gates over
  the source manifests.
- ``ron_autotune_abort()`` on a tuner that was never started restored the
  zeroed placeholder snapshot, zeroing the PID's gains. The restore now runs
  only when ``ron_autotune_start()`` captured a snapshot (new opaque
  ``pid_saved`` state flag); the run is still marked aborted.
- ``ron_autotune_start()`` did not clear the previous run's oscillation
  tracking, flags and results, so a second run started with stale crossing
  counts, peaks and timers. It now reseeds the run state (configuration and
  initialised guard kept) before taking the PID snapshot.
- ``ron_trap_step()`` / ``ron_scurve_step()`` were documented to return and
  latch ``RON_FAULT_OUTPUT_NAN`` for a non-finite setpoint but never checked,
  so an overflowing move emitted ``inf``/``NaN`` setpoints (and could report
  ``finished``). The kinematic setpoints are now checked after integration;
  on failure the fault latches, the status becomes ``RON_STATUS_FAULT`` and
  the state and outputs keep the last finite setpoints.
- ``ron_obs_step()``, ``ron_kf_predict()`` and ``ron_kf_update()`` wrote a
  non-finite estimate / covariance into the instance before returning
  ``RON_FAULT_OUTPUT_NAN``, so the filter kept ``NaN``/``inf`` state and every
  later call failed too. They now compute into locals and commit only a
  finite result; a rejected step leaves the state unchanged. The headers no
  longer claim the fault "latches" (these modules have no fault register).
- ``ron_ss_step()`` and ``ron_lqr_step()`` advanced the integral accumulator
  before the output finiteness check, so a step rejected with
  ``RON_FAULT_OUTPUT_NAN`` still wound the integrator. The integral is now
  computed into a local and committed only with a finite output. The
  ``ron_statespace.h`` / ``ron_lqr.h`` / ``ron_lqg.h`` step docs said faults
  "latch", but nothing ever sets ``faults``: they now say a rejected step
  leaves the state unchanged, and list ``dt`` rejection under
  ``RON_FAULT_INPUT_NAN`` (the code's actual return) rather than
  ``RON_FAULT_CONFIG_INVALID``.
- ``ron_ma_step()`` and ``ron_biquad_step()`` stored the new sample in the
  ring buffer / running sum and shifted each section's ``w1``/``w2`` before
  the ``RON_FAULT_OUTPUT_NAN`` check, so the rejected sample was committed
  (and the running sum could hold ``inf``). They now compute on locals and
  commit only a finite output; the fault still latches as before.
- Header documentation that did not match the code (API unchanged):
  ``ron_kf_init`` claimed to require a positive-definite ``R`` (only
  finiteness is checked; a non-PD innovation covariance is rejected by
  ``ron_kf_update`` with ``RON_FAULT_CONFIG_INVALID``, which its ``@retval``
  list now says instead of ``RON_FAULT_OUTPUT_NAN``); ``ron_lqg.h`` said both
  gains are solved via DARE at init (only the LQR gain is; the Kalman filter
  runs its time-varying gain or the supplied ``K_f_inf``), and
  ``ron_lqg_reset`` claimed to clear an integral accumulator LQG does not
  have; ``ron_ma_init`` said the filter "averages only what it has" before
  ``M`` samples (it divides by ``M`` over a zero-filled window).

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

.. ============================================================
.. Software Architecture and Design Specification
.. Regulon — PID Controller Module
.. ============================================================

.. meta::
   :description: Software Architecture and Design Specification for the Regulon, PID Controller Module.
   :keywords: PID, control systems, embedded, SADS, architecture, design, safety-critical

########################################################################
Software Architecture and Design Specification
########################################################################

**Document Title:** Software Architecture and Design Specification — Regulon, PID Controller Module

**Document ID:** RON-SADS-001

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
   * - 0.1
     - 2025-03-15
     - Initial draft skeleton
     - TBD
   * - 1.0.0
     - 2025-04-10
     - First baseline release
     - TBD
   * - 1.1.0
     - 2025-04-10
     - Added architecture and detailed design for: signal conditioning filters,
       feed-forward, gain scheduling, cascade controller, trajectory generators,
       Kalman filter, state-space controller, Luenberger observer, relay
       auto-tuning, health monitor, performance metrics. Updated module
       decomposition and dependency diagram.
     - TBD

------------------------------------------------------------------------

Introduction
============

Purpose
-------

This document describes the software architecture and detailed design of the **Regulon Control Systems Library PID Controller Module**. It translates the requirements of RON-SRS-001 into a structured, implementation-agnostic design that guides developers during the coding phase and provides a reference for verification and maintenance activities.

Scope
-----

This document covers:

- Architectural decisions (programming paradigm, module decomposition, data flow).
- Detailed design of every module, data structure, and operation.
- Signal flow and algorithmic description of the full PID computation pipeline.
- Interface specifications (API contracts, pre/post-conditions).
- Error handling and fault management design.
- Design patterns and constraints for safety-critical embedded deployment.

This document does **not** prescribe the programming language. Concrete language syntax is deferred to the Implementation Specification (RON-IS-001). All designs herein are expressed in a language-agnostic notation using pseudocode, structured English, and mathematical formulas.

Parent Documents
----------------

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Document ID
     - Title
   * - RON-SRS-001
     - Software Requirements Specification — PID Controller Module
   * - RON-IS-001
     - Implementation Specification (TBD — produced after language selection)
   * - RON-TP-001
     - Test Plan (TBD)

Notation Conventions
---------------------

- ``CONSTANT`` — named constant (all caps).
- ``TypeName`` — composite data type (PascalCase).
- ``operation_name()`` — operation/function (snake_case).
- ``field_name`` — field within a data type (snake_case).
- ``[in]`` / ``[out]`` / ``[in,out]`` — parameter direction annotations.
- ``[precondition]`` / ``[postcondition]`` — behavioural contracts.
- Pseudocode uses ``IF/THEN/ELSE/END``, ``FOR``, ``WHILE``, ``RETURN``, ``←`` (assignment).

------------------------------------------------------------------------

Architectural Design
====================

Design Philosophy
-----------------

The Regulon PID module is designed according to the following principles, ordered by priority:

1. **Safety first** — correctness and predictability are non-negotiable. No feature may be added at the cost of undefined behaviour, unbounded execution, or silent fault propagation.
2. **Determinism** — every execution path must be bounded, branchless where possible, and WCET-analysable.
3. **Zero dynamic allocation** — all state lives in caller-managed storage; the library never allocates or frees memory.
4. **Separation of concerns** — computation, configuration, state management, fault handling, and normalisation are distinct responsibilities implemented in distinct sub-modules.
5. **Minimal footprint** — each byte of code and data must justify its presence on a resource-constrained MCU.
6. **Testability on host** — the module is fully exercisable on a PC without hardware.

Architectural Style
--------------------

The module uses a **layered, object-based** (not object-oriented) architecture:

.. code-block:: none

   ┌─────────────────────────────────────────────────────────────────┐
   │                        Application / HAL                        │  (caller's code)
   ├─────────────────────────────────────────────────────────────────┤
   │                    Regulon Public API Layer                         │  (ron_pid_api)
   ├────────────────┬────────────────┬──────────────────────────────┤
   │ Configuration  │  Normalisation │   Fault Manager              │  (support modules)
   │ Manager        │  Module        │                               │
   ├────────────────┴────────────────┴──────────────────────────────┤
   │                    PID Core Computation Engine                   │  (ron_pid_core)
   ├─────────────────────────────────────────────────────────────────┤
   │            Platform Abstraction (type macros, assert)            │  (ron_platform)
   └─────────────────────────────────────────────────────────────────┘

Each layer depends only on layers below it. The application layer is external and not part of the library.

Module Decomposition
---------------------

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Module
     - Responsibility
   * - ``ron_platform``
     - Compile-time type definitions, floating-point selection macro (``RON_FLOAT`` = ``float`` or ``double``), ``RON_ASSERT`` macro, platform-specific math helpers (``ron_isnan``, ``ron_isinf``, ``ron_fabs``, ``ron_clamp``).
   * - ``ron_pid_types``
     - All data structure definitions: ``PidConfig``, ``PidState``, ``PidInstance``, ``PidStatus``, ``PidFaultCode``, ``AntiWindupMode``, ``DerivativeMode``, ``OperatingMode``.
   * - ``ron_pid_config``
     - Configuration validation and application: ``pid_config_validate()``, ``pid_config_apply()``.
   * - ``ron_pid_core``
     - The main computation pipeline: ``pid_core_step()``. Contains all sub-computations (normalisation, P/I/D terms, AW, saturation, rate limiting).
   * - ``ron_pid_fault``
     - Fault detection, safe-state application, fault register management.
   * - ``ron_pid_api``
     - Public API entry points; parameter validation; dispatch to core and support modules.

New modules added in v1.1.0:

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Module
     - Responsibility
   * - ``ron_filter``
     - First-order LP, moving average, biquad IIR (cascaded SOS), and standalone rate limiter. Coefficient generation helpers for LP, HP, BP, notch configurations.
   * - ``ron_feedforward``
     - Static gain, velocity, and acceleration feed-forward computation. Derivative estimation with independent LP filter. Integrated into ``ron_pid_core`` as an optional additive path.
   * - ``ron_gain_sched``
     - Breakpoint table management, hard-switch and linear-interpolation scheduling, atomic gain update via PID runtime API.
   * - ``ron_cascade``
     - Outer/inner PID instance coupling, coordinated anti-windup propagation, unified status word assembly.
   * - ``ron_trajectory``
     - Trapezoidal and S-curve profile generators. Kinematic state machine, short-move detection, online goal update, hold mode.
   * - ``ron_kalman``
     - Discrete linear Kalman filter: predict/update cycle, Cholesky solver for multi-dimensional innovation inversion, Joseph form update option, steady-state mode, measurement dropout.
   * - ``ron_statespace``
     - Discrete state-feedback controller with optional integral augmentation. Accepts state from external vector, Luenberger observer, or Kalman filter.
   * - ``ron_observer``
     - Luenberger observer: state propagation and correction step. Exposes state vector getter.
   * - ``ron_autotune``
     - Relay feedback signal generation, oscillation detection (zero-crossing counting), :math:`K_u`/:math:`T_u` estimation, tuning rule computation, staged apply/abort interface.
   * - ``ron_health``
     - Passive loop health monitor. Per-condition threshold comparators, sliding window oscillation detector, latch/clear model, health callback dispatch.
   * - ``ron_metrics``
     - IAE/ISE/ITAE accumulators, peak overshoot tracker, settling-time and rise-time state machines, step detection, windowed/cumulative modes.

Data Flow Overview
------------------

The following describes the data flow through one call to ``pid_step()``:

.. code-block:: none

   Caller
     │
     ▼  setpoint r(k), process_variable y(k), sample_time dt
   ┌────────────────────────────────────────────────────────────────┐
   │ 1. INPUT VALIDATION & FAULT CHECK                              │
   │    - Validate r, y for NaN / Inf                               │
   │    - Check operating mode (MANUAL → bypass to safe output)     │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 2. INPUT NORMALISATION (if enabled)                            │
   │    r_n ← normalise(r),  y_n ← normalise(y)                    │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 3. SETPOINT PRE-FILTER (if enabled)                            │
   │    r_f ← first_order_filter(r_n, r_f_prev, tau_sp)            │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 4. ERROR COMPUTATION                                           │
   │    e(k) ← r_f - y_n                                           │
   │    e_P(k) ← b·r_f - y_n   (2DOF proportional error)          │
   │    e_D(k) ← c·r_f - y_n   (2DOF derivative error)            │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 5. PROPORTIONAL TERM                                           │
   │    P ← Kp · e_P(k)                                            │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 6. DERIVATIVE TERM (with LP filter)                            │
   │    IF DoM: delta ← -(y_n - y_n_prev)                          │
   │    IF DoE: delta ← e_D(k) - e_D_prev                          │
   │    D_raw ← (Kd / dt) · delta                                   │
   │    D_f ← LP_filter(D_raw, D_f_prev, N, dt)   [if N > 0]       │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 7. INTEGRAL TERM + ANTI-WINDUP                                 │
   │    I_new ← I_prev + Ki·dt·e(k)                                │
   │            + AW_back_calc_correction   [if back-calc AW]       │
   │    IF clamping AW: suppress I_new update when conditions met   │
   │    I_clamped ← clamp(I_new, I_min, I_max)                     │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 8. RAW OUTPUT SUMMATION                                        │
   │    u_raw ← P + I_clamped + D_f                                 │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 9. OUTPUT NORMALISATION INVERSE (if normalisation enabled)     │
   │    u_eng ← denormalise(u_raw)                                  │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 10. OUTPUT SATURATION                                          │
   │     u_sat ← clamp(u_eng, u_min, u_max)                        │
   │     SET status.SATURATED if u_eng ≠ u_sat                      │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 11. RATE LIMITING (if enabled)                                 │
   │     delta_u ← u_sat - u_sat_prev                               │
   │     IF |delta_u| > rate_max·dt: apply rate clamp               │
   │     SET status.RATE_LIMITED if clamped                         │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 12. NaN / INF CHECK ON OUTPUT                                  │
   │     IF u_final is NaN or Inf: FAULT → safe-state output        │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
   ┌────────────────────────────────────────────────────────────────┐
   │ 13. STATE UPDATE                                               │
   │     Store: y_prev, e_D_prev, D_f_prev, I, u_sat_prev          │
   │     Update AW back-calc with (u_sat - u_raw) for next step     │
   └──────────────────────────┬─────────────────────────────────────┘
                              ▼
     Return u_final, status word

------------------------------------------------------------------------

Detailed Data Structure Design
================================

Platform Type
-------------

A single floating-point precision alias is defined at the platform level:

.. code-block:: none

   TYPE RON_FLOAT  ←  float    [default: 32-bit IEEE 754]
                   OR  double   [option: 64-bit IEEE 754, compile-time selection]

All internal state and computation uses ``RON_FLOAT`` exclusively.

AntiWindupMode Enumeration
--------------------------

.. code-block:: none

   ENUM AntiWindupMode:
     AW_NONE           = 0   -- Anti-windup disabled
     AW_BACK_CALC      = 1   -- Back-calculation (tracking)
     AW_CLAMPING       = 2   -- Conditional integration clamping

DerivativeMode Enumeration
--------------------------

.. code-block:: none

   ENUM DerivativeMode:
     DERIV_ON_ERROR       = 0   -- D term uses error difference (derivative kick on SP step)
     DERIV_ON_MEASUREMENT = 1   -- D term uses measurement difference (no derivative kick) [DEFAULT]

OperatingMode Enumeration
--------------------------

.. code-block:: none

   ENUM OperatingMode:
     MODE_AUTOMATIC = 0   -- Closed-loop PID active
     MODE_MANUAL    = 1   -- Caller supplies output directly; integral frozen

SafeStatePolicy Enumeration
-----------------------------

.. code-block:: none

   ENUM SafeStatePolicy:
     SAFE_HOLD_LAST  = 0   -- Freeze output at last valid value
     SAFE_ZERO       = 1   -- Drive output to 0.0
     SAFE_CONSTANT   = 2   -- Drive output to PidConfig.safe_value

FaultCode Enumeration
---------------------

.. code-block:: none

   ENUM FaultCode  (bitmask, may be OR-ed):
     FAULT_NONE              = 0x00
     FAULT_INPUT_NAN         = 0x01   -- r or y is NaN or Inf
     FAULT_OUTPUT_NAN        = 0x02   -- computed u is NaN or Inf
     FAULT_INTEGRAL_OVERFLOW = 0x04   -- |I| exceeded overflow threshold
     FAULT_CONFIG_INVALID    = 0x08   -- configuration inconsistency detected at runtime
     FAULT_NULL_POINTER      = 0x10   -- null pointer passed to API

PidStatus Value Type
--------------------

.. code-block:: none

   TYPE PidStatus  (bitmask word, unsigned 16-bit):
     STATUS_OK               = 0x0000
     STATUS_SATURATED        = 0x0001
     STATUS_RATE_LIMITED     = 0x0002
     STATUS_AW_ACTIVE        = 0x0004
     STATUS_MANUAL_MODE      = 0x0008
     STATUS_FAULT            = 0x0010
     STATUS_SP_FILTER_ACTIVE = 0x0020
     STATUS_NORMALISED       = 0x0040

PidConfig Structure
-------------------

Holds all configuration (tuning) parameters. Treated as **read-only during computation** — modified only by the configuration API.

.. code-block:: none

   STRUCTURE PidConfig:

     -- ── Gain parameters ──────────────────────────────────────────
     Kp            : RON_FLOAT    -- Proportional gain (≥ 0.0)
     Ki            : RON_FLOAT    -- Integral gain (parallel form) (≥ 0.0)
     Kd            : RON_FLOAT    -- Derivative gain (parallel form) (≥ 0.0)

     -- ── Derivative filter ─────────────────────────────────────────
     N             : RON_FLOAT    -- Derivative LP filter bandwidth multiplier (≥ 0; 0 = disable)

     -- ── 2DOF setpoint weights ─────────────────────────────────────
     b             : RON_FLOAT    -- Proportional setpoint weight [0, 1], default 1.0
     c             : RON_FLOAT    -- Derivative setpoint weight [0, 1], default 1.0

     -- ── Output limits ─────────────────────────────────────────────
     u_min         : RON_FLOAT    -- Minimum control variable (u_min < u_max)
     u_max         : RON_FLOAT    -- Maximum control variable

     -- ── Rate limit ────────────────────────────────────────────────
     du_max        : RON_FLOAT    -- Max rate of change per second (≤ 0 = disable)

     -- ── Integral clamp ────────────────────────────────────────────
     I_min         : RON_FLOAT    -- Integral accumulator lower clamp
     I_max         : RON_FLOAT    -- Integral accumulator upper clamp

     -- ── Anti-windup ───────────────────────────────────────────────
     aw_mode       : AntiWindupMode
     T_aw          : RON_FLOAT    -- Back-calculation tracking time constant (> 0; used if AW_BACK_CALC)

     -- ── Derivative mode ───────────────────────────────────────────
     deriv_mode    : DerivativeMode

     -- ── Setpoint filter ───────────────────────────────────────────
     tau_sp        : RON_FLOAT    -- SP first-order filter time constant (≤ 0 = disable)

     -- ── Normalisation ─────────────────────────────────────────────
     normalise     : BOOL          -- Enable input/output normalisation
     in_min        : RON_FLOAT    -- Input engineering minimum (r and y)
     in_max        : RON_FLOAT    -- Input engineering maximum
     out_min       : RON_FLOAT    -- Output engineering minimum
     out_max       : RON_FLOAT    -- Output engineering maximum

     -- ── Safety / fault ────────────────────────────────────────────
     safe_policy   : SafeStatePolicy
     safe_value    : RON_FLOAT    -- Used when safe_policy = SAFE_CONSTANT
     I_overflow_thresh : RON_FLOAT  -- |I| > this threshold triggers FAULT_INTEGRAL_OVERFLOW

     -- ── Setpoint-change integral reset ────────────────────────────
     sp_reset_threshold : RON_FLOAT  -- |Δr| > this triggers integral reset (≤ 0 = disable)

     -- ── Fault handler callback ────────────────────────────────────
     fault_callback : POINTER TO PROCEDURE(fault_code: FaultCode)   -- nullable

PidState Structure
------------------

Holds all dynamic (mutable) state per controller instance. Must be **zero-initialised** before first use.

.. code-block:: none

   STRUCTURE PidState:

     -- ── Integrator ────────────────────────────────────────────────
     integral          : RON_FLOAT    -- Accumulated integral term

     -- ── Derivative history ────────────────────────────────────────
     y_prev            : RON_FLOAT    -- Previous process variable (for DoM)
     e_D_prev          : RON_FLOAT    -- Previous derivative-error (for DoE)
     D_filt_prev       : RON_FLOAT    -- Previous filtered derivative output

     -- ── Setpoint filter state ─────────────────────────────────────
     r_filt_prev       : RON_FLOAT    -- Previous filtered setpoint

     -- ── Output tracking ───────────────────────────────────────────
     u_sat_prev        : RON_FLOAT    -- Previous saturated output (for rate limiting + AW)
     u_prev            : RON_FLOAT    -- Previous pre-saturation output (for back-calc AW)

     -- ── Operating state ───────────────────────────────────────────
     mode              : OperatingMode
     fault_code        : FaultCode
     status            : PidStatus

     -- ── Initialisation guard ──────────────────────────────────────
     is_initialised    : BOOL

PidInstance Structure
----------------------

The opaque handle exposed to the caller. Groups configuration and state. The caller allocates storage; the library never allocates.

.. code-block:: none

   STRUCTURE PidInstance:
     config  : PidConfig
     state   : PidState

------------------------------------------------------------------------

Module Design: ron_pid_config
================================

Responsibility
--------------

Validates a ``PidConfig`` record for internal consistency before it is applied to an instance. Ensures no divide-by-zero or logically inconsistent parameter combinations reach the core.

Validation Rules
-----------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Condition Checked
     - Fault / Error Returned
   * - ``Kp``, ``Ki``, ``Kd`` are finite and ≥ 0
     - ``FAULT_CONFIG_INVALID``
   * - ``u_min < u_max``
     - ``FAULT_CONFIG_INVALID``
   * - ``I_min < I_max``
     - ``FAULT_CONFIG_INVALID``
   * - If ``aw_mode = AW_BACK_CALC``: ``T_aw > 0``
     - ``FAULT_CONFIG_INVALID``
   * - If ``normalise = TRUE``: ``in_min < in_max`` AND ``out_min < out_max``
     - ``FAULT_CONFIG_INVALID``
   * - ``N ≥ 0``
     - ``FAULT_CONFIG_INVALID``
   * - ``b``, ``c`` ∈ [0.0, 1.0]
     - ``FAULT_CONFIG_INVALID``
   * - ``I_overflow_thresh > 0`` (if set)
     - ``FAULT_CONFIG_INVALID``

Operation: ``pid_config_validate``
-----------------------------------

.. code-block:: none

   OPERATION pid_config_validate(cfg: CONST POINTER TO PidConfig) → FaultCode

   [precondition]  cfg ≠ NULL

   PROCEDURE:
     IF cfg = NULL THEN RETURN FAULT_NULL_POINTER END
     IF cfg.Kp < 0 OR isnan(cfg.Kp) OR isinf(cfg.Kp)  THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.Ki < 0 OR isnan(cfg.Ki) OR isinf(cfg.Ki)  THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.Kd < 0 OR isnan(cfg.Kd) OR isinf(cfg.Kd)  THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.u_min >= cfg.u_max                          THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.I_min >= cfg.I_max                          THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.aw_mode = AW_BACK_CALC AND cfg.T_aw <= 0.0 THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.normalise THEN
       IF cfg.in_min  >= cfg.in_max                     THEN RETURN FAULT_CONFIG_INVALID END
       IF cfg.out_min >= cfg.out_max                    THEN RETURN FAULT_CONFIG_INVALID END
     END
     IF cfg.N < 0.0                                     THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.b < 0.0 OR cfg.b > 1.0                     THEN RETURN FAULT_CONFIG_INVALID END
     IF cfg.c < 0.0 OR cfg.c > 1.0                     THEN RETURN FAULT_CONFIG_INVALID END
     RETURN FAULT_NONE

------------------------------------------------------------------------

Module Design: ron_pid_core
==============================

Responsibility
--------------

Implements the full PID computation pipeline for a single time step. Called exclusively from the API layer after input validation. Never modifies configuration; reads it as constant.

The pipeline follows the data flow described in Section 2.4. Each stage is presented below as a named sub-operation for clarity. In the implementation they are inlined into a single bounded function to minimise call overhead and simplify WCET analysis.

Sub-Operation: ``compute_normalise``
--------------------------------------

.. code-block:: none

   OPERATION compute_normalise(x: RON_FLOAT,
                               x_min: RON_FLOAT,
                               x_max: RON_FLOAT) → RON_FLOAT

   RETURN (x - x_min) / (x_max - x_min)

   -- Division safety: caller (pid_config_validate) guarantees x_max > x_min.

Sub-Operation: ``compute_denormalise``
----------------------------------------

.. code-block:: none

   OPERATION compute_denormalise(x_norm: RON_FLOAT,
                                 out_min: RON_FLOAT,
                                 out_max: RON_FLOAT) → RON_FLOAT

   RETURN x_norm * (out_max - out_min) + out_min

Sub-Operation: ``compute_sp_filter``
--------------------------------------

First-order IIR setpoint filter, Euler-discretised:

.. math::

   r_f(k) = \alpha \cdot r_f(k-1) + (1 - \alpha) \cdot r(k)

where :math:`\alpha = \frac{\tau_{sp}}{\tau_{sp} + T_s}`.

.. code-block:: none

   OPERATION compute_sp_filter(r:       RON_FLOAT,
                               r_f_prev: RON_FLOAT,
                               tau_sp:  RON_FLOAT,
                               dt:      RON_FLOAT) → RON_FLOAT

   IF tau_sp <= 0.0 THEN RETURN r END   -- filter disabled
   alpha ← tau_sp / (tau_sp + dt)
   RETURN alpha * r_f_prev + (1.0 - alpha) * r

Sub-Operation: ``compute_derivative``
---------------------------------------

Implements derivative computation with optional low-pass filter.

The Tustin-discretised first-order low-pass filter applied to the derivative is:

.. math::

   D_f(k) = \frac{N \cdot K_d}{1 + N \cdot T_s} \cdot \Delta v(k)
           + \frac{1}{1 + N \cdot T_s} \cdot D_f(k-1)

where :math:`\Delta v(k)` is the raw derivative signal (error or measurement difference).

.. code-block:: none

   OPERATION compute_derivative(cfg:     CONST POINTER TO PidConfig,
                                state:   POINTER TO PidState,
                                y_n:     RON_FLOAT,
                                e_D:     RON_FLOAT,
                                dt:      RON_FLOAT) → RON_FLOAT

   -- Select delta based on derivative mode
   IF cfg.deriv_mode = DERIV_ON_MEASUREMENT THEN
     delta ← -(y_n - state.y_prev)
   ELSE
     delta ← e_D - state.e_D_prev
   END

   -- Raw derivative (no filter)
   D_raw ← (cfg.Kd * delta) / dt

   -- Apply LP filter if N > 0
   IF cfg.N > 0.0 THEN
     coeff_N ← cfg.N * dt
     D_f ← (coeff_N / (1.0 + coeff_N)) * D_raw
          + (1.0 / (1.0 + coeff_N)) * state.D_filt_prev
   ELSE
     D_f ← D_raw
   END

   RETURN D_f

Sub-Operation: ``compute_integral``
--------------------------------------

Computes the integral update including anti-windup correction. Both Euler and Trapezoidal methods are shown; the active method is selected by configuration.

.. code-block:: none

   OPERATION compute_integral(cfg:      CONST POINTER TO PidConfig,
                              state:    POINTER TO PidState,
                              e:        RON_FLOAT,
                              e_prev:   RON_FLOAT,
                              dt:       RON_FLOAT) → RON_FLOAT

   -- Euler forward (rectangular)
   IF integration_method = EULER THEN
     I_update ← cfg.Ki * dt * e
   ELSE  -- Trapezoidal (Tustin)
     I_update ← cfg.Ki * dt * 0.5 * (e + e_prev)
   END

   -- Back-calculation anti-windup correction term
   IF cfg.aw_mode = AW_BACK_CALC THEN
     aw_correction ← (dt / cfg.T_aw) * (state.u_sat_prev - state.u_prev)
   ELSE
     aw_correction ← 0.0
   END

   I_new ← state.integral + I_update + aw_correction

   -- Conditional integration clamping AW: suppress update if winding up deeper
   IF cfg.aw_mode = AW_CLAMPING THEN
     saturated ← (state.u_sat_prev < cfg.u_min + EPSILON)
                  OR (state.u_sat_prev > cfg.u_max - EPSILON)
     same_sign ← (e > 0.0 AND state.integral > 0.0)
                  OR (e < 0.0 AND state.integral < 0.0)
     IF saturated AND same_sign THEN
       I_new ← state.integral   -- suppress update
     END
   END

   -- Hard integral clamp (independent safeguard)
   I_new ← clamp(I_new, cfg.I_min, cfg.I_max)

   RETURN I_new

Sub-Operation: ``compute_saturation``
----------------------------------------

.. code-block:: none

   OPERATION compute_saturation(u:     RON_FLOAT,
                                u_min: RON_FLOAT,
                                u_max: RON_FLOAT,
                                [out] saturated: BOOL) → RON_FLOAT

   u_sat ← clamp(u, u_min, u_max)
   saturated ← (u_sat ≠ u)
   RETURN u_sat

Sub-Operation: ``compute_rate_limit``
----------------------------------------

.. code-block:: none

   OPERATION compute_rate_limit(u_sat:     RON_FLOAT,
                                u_sat_prev: RON_FLOAT,
                                du_max:    RON_FLOAT,
                                dt:        RON_FLOAT,
                                [out] rate_limited: BOOL) → RON_FLOAT

   IF du_max <= 0.0 THEN    -- disabled
     rate_limited ← FALSE
     RETURN u_sat
   END

   delta_max ← du_max * dt
   delta     ← u_sat - u_sat_prev

   IF delta > delta_max THEN
     rate_limited ← TRUE
     RETURN u_sat_prev + delta_max
   ELSE IF delta < -delta_max THEN
     rate_limited ← TRUE
     RETURN u_sat_prev - delta_max
   ELSE
     rate_limited ← FALSE
     RETURN u_sat
   END

Main Step Operation: ``pid_core_step``
----------------------------------------

.. code-block:: none

   OPERATION pid_core_step(inst: POINTER TO PidInstance,
                           r:    RON_FLOAT,
                           y:    RON_FLOAT,
                           dt:   RON_FLOAT,
                           [out] status: POINTER TO PidStatus) → RON_FLOAT

   [precondition]  inst ≠ NULL, status ≠ NULL, inst.state.is_initialised = TRUE
   [precondition]  dt > 0.0
   [postcondition] RETURN value ∈ [inst.config.u_min, inst.config.u_max]
                   OR     inst.state.fault_code ≠ FAULT_NONE

   cfg   ← CONST POINTER TO inst.config
   state ← POINTER TO inst.state

   -- ─── 1. Input NaN/Inf guard ───────────────────────────────────
   IF isnan(r) OR isinf(r) OR isnan(y) OR isinf(y) THEN
     CALL pid_fault_set(inst, FAULT_INPUT_NAN)
     RETURN pid_fault_safe_output(inst)
   END

   -- ─── 2. Manual mode passthrough ───────────────────────────────
   IF state.mode = MODE_MANUAL THEN
     status ← STATUS_MANUAL_MODE
     RETURN state.u_sat_prev    -- last manual output unchanged
   END

   -- ─── 3. Normalise inputs ──────────────────────────────────────
   IF cfg.normalise THEN
     r_n ← compute_normalise(r, cfg.in_min, cfg.in_max)
     y_n ← compute_normalise(y, cfg.in_min, cfg.in_max)
   ELSE
     r_n ← r
     y_n ← y
   END

   -- ─── 4. Setpoint filter ───────────────────────────────────────
   r_f ← compute_sp_filter(r_n, state.r_filt_prev, cfg.tau_sp, dt)

   -- ─── 5. Setpoint change integral reset ───────────────────────
   IF cfg.sp_reset_threshold > 0.0 THEN
     IF fabs(r_f - state.r_filt_prev) > cfg.sp_reset_threshold THEN
       state.integral ← 0.0
     END
   END

   -- ─── 6. Error signals ─────────────────────────────────────────
   e    ← r_f - y_n                       -- standard error
   e_P  ← cfg.b * r_f - y_n              -- 2DOF proportional error
   e_D  ← cfg.c * r_f - y_n              -- 2DOF derivative error

   -- ─── 7. Proportional term ─────────────────────────────────────
   P ← cfg.Kp * e_P

   -- ─── 8. Derivative term ───────────────────────────────────────
   D_f ← compute_derivative(cfg, state, y_n, e_D, dt)

   -- ─── 9. Integral term ─────────────────────────────────────────
   I_new ← compute_integral(cfg, state, e, e_prev_from_state, dt)

   -- ─── 10. Raw output ───────────────────────────────────────────
   u_raw ← P + I_new + D_f

   -- ─── 11. Denormalise output ───────────────────────────────────
   IF cfg.normalise THEN
     u_eng ← compute_denormalise(u_raw, cfg.out_min, cfg.out_max)
   ELSE
     u_eng ← u_raw
   END

   -- ─── 12. Output NaN/Inf guard ─────────────────────────────────
   IF isnan(u_eng) OR isinf(u_eng) THEN
     CALL pid_fault_set(inst, FAULT_OUTPUT_NAN)
     RETURN pid_fault_safe_output(inst)
   END

   -- ─── 13. Output saturation ────────────────────────────────────
   u_sat ← compute_saturation(u_eng, cfg.u_min, cfg.u_max, saturated)
   IF saturated THEN status ← status OR STATUS_SATURATED END

   -- ─── 14. Rate limiting ────────────────────────────────────────
   u_final ← compute_rate_limit(u_sat, state.u_sat_prev,
                                 cfg.du_max, dt, rate_limited)
   IF rate_limited THEN status ← status OR STATUS_RATE_LIMITED END

   -- ─── 15. Integral overflow check ──────────────────────────────
   IF cfg.I_overflow_thresh > 0.0 THEN
     IF fabs(I_new) > cfg.I_overflow_thresh THEN
       CALL pid_fault_set(inst, FAULT_INTEGRAL_OVERFLOW)
       RETURN pid_fault_safe_output(inst)
     END
   END

   -- ─── 16. State update ─────────────────────────────────────────
   state.integral    ← I_new
   state.y_prev      ← y_n
   state.e_D_prev    ← e_D
   state.D_filt_prev ← D_f
   state.r_filt_prev ← r_f
   state.u_sat_prev  ← u_final
   state.u_prev      ← u_eng      -- pre-saturation, for AW back-calc
   state.status      ← status OR STATUS_OK

   -- ─── 17. AW active flag ───────────────────────────────────────
   IF cfg.aw_mode ≠ AW_NONE AND saturated THEN
     state.status ← state.status OR STATUS_AW_ACTIVE
   END

   RETURN u_final

------------------------------------------------------------------------

Module Design: ron_pid_fault
================================

Responsibility
--------------

Manages fault detection, safe-state enforcement, and fault register maintenance.

Operation: ``pid_fault_set``
-------------------------------

.. code-block:: none

   OPERATION pid_fault_set(inst: POINTER TO PidInstance,
                           code: FaultCode) → VOID

   inst.state.fault_code ← inst.state.fault_code OR code
   inst.state.status     ← inst.state.status OR STATUS_FAULT

   IF inst.config.fault_callback ≠ NULL THEN
     CALL inst.config.fault_callback(code)
   END

Operation: ``pid_fault_safe_output``
--------------------------------------

Returns the safe-state output according to the configured policy.

.. code-block:: none

   OPERATION pid_fault_safe_output(inst: POINTER TO PidInstance) → RON_FLOAT

   SWITCH inst.config.safe_policy:
     CASE SAFE_HOLD_LAST:  RETURN inst.state.u_sat_prev
     CASE SAFE_ZERO:       RETURN 0.0
     CASE SAFE_CONSTANT:   RETURN inst.config.safe_value
   END

Operation: ``pid_fault_clear``
--------------------------------

.. code-block:: none

   OPERATION pid_fault_clear(inst: POINTER TO PidInstance) → VOID

   inst.state.fault_code ← FAULT_NONE
   inst.state.status     ← inst.state.status AND (NOT STATUS_FAULT)

   -- NOTE: State (integral, history) is NOT automatically reset on fault clear.
   -- The caller must decide whether to call pid_reset() as well.

------------------------------------------------------------------------

Module Design: ron_pid_api (Public Interface)
===============================================

All operations in this module are entry points. They perform null-pointer checks and delegate to core or support modules.

Operation: ``pid_init``
------------------------

Initialises a ``PidInstance`` with a given configuration.

.. code-block:: none

   OPERATION pid_init(inst: POINTER TO PidInstance,
                      cfg:  CONST POINTER TO PidConfig) → FaultCode

   [precondition]  inst ≠ NULL, cfg ≠ NULL
   [postcondition] inst.state.is_initialised = TRUE  iff RETURN = FAULT_NONE

   IF inst = NULL OR cfg = NULL THEN RETURN FAULT_NULL_POINTER END

   fault ← pid_config_validate(cfg)
   IF fault ≠ FAULT_NONE THEN RETURN fault END

   inst.config ← *cfg              -- copy configuration (by value)
   ZERO-FILL inst.state            -- clear all state to 0 / FALSE
   inst.state.is_initialised ← TRUE
   inst.state.mode ← MODE_AUTOMATIC

   RETURN FAULT_NONE

Operation: ``pid_reset``
-------------------------

Resets dynamic state to zero without modifying configuration.

.. code-block:: none

   OPERATION pid_reset(inst: POINTER TO PidInstance) → FaultCode

   IF inst = NULL            THEN RETURN FAULT_NULL_POINTER END
   IF NOT inst.state.is_initialised THEN RETURN FAULT_CONFIG_INVALID END

   inst.state.integral    ← 0.0
   inst.state.y_prev      ← 0.0
   inst.state.e_D_prev    ← 0.0
   inst.state.D_filt_prev ← 0.0
   inst.state.r_filt_prev ← 0.0
   inst.state.u_sat_prev  ← 0.0
   inst.state.u_prev      ← 0.0
   inst.state.fault_code  ← FAULT_NONE
   inst.state.status      ← STATUS_OK
   -- mode is preserved

   RETURN FAULT_NONE

Operation: ``pid_step``
------------------------

Executes one control cycle. Primary runtime interface.

.. code-block:: none

   OPERATION pid_step(inst:    POINTER TO PidInstance,
                      r:       RON_FLOAT,
                      y:       RON_FLOAT,
                      dt:      RON_FLOAT,
                      [out] u: POINTER TO RON_FLOAT,
                      [out] status: POINTER TO PidStatus) → FaultCode

   [precondition]  inst ≠ NULL, u ≠ NULL, status ≠ NULL
   [precondition]  inst.state.is_initialised = TRUE
   [precondition]  dt > 0.0
   [postcondition] *u ∈ [cfg.u_min, cfg.u_max]  OR  fault set

   IF inst = NULL OR u = NULL OR status = NULL THEN RETURN FAULT_NULL_POINTER END
   IF NOT inst.state.is_initialised            THEN RETURN FAULT_CONFIG_INVALID END
   IF dt <= 0.0 OR isnan(dt) OR isinf(dt)     THEN RETURN FAULT_CONFIG_INVALID END

   -- Reject step if in latched fault state
   IF inst.state.fault_code ≠ FAULT_NONE THEN
     *u     ← pid_fault_safe_output(inst)
     *status ← inst.state.status
     RETURN inst.state.fault_code
   END

   *u ← pid_core_step(inst, r, y, dt, status)
   RETURN inst.state.fault_code

Operation: ``pid_set_gains``
------------------------------

Atomically updates all gain parameters.

.. code-block:: none

   OPERATION pid_set_gains(inst: POINTER TO PidInstance,
                           Kp:   RON_FLOAT,
                           Ki:   RON_FLOAT,
                           Kd:   RON_FLOAT) → FaultCode

   IF inst = NULL              THEN RETURN FAULT_NULL_POINTER END
   IF NOT inst.state.is_initialised THEN RETURN FAULT_CONFIG_INVALID END

   -- Build candidate config, validate, then apply atomically
   candidate ← inst.config
   candidate.Kp ← Kp
   candidate.Ki ← Ki
   candidate.Kd ← Kd

   fault ← pid_config_validate(POINTER TO candidate)
   IF fault ≠ FAULT_NONE THEN RETURN fault END

   inst.config.Kp ← Kp
   inst.config.Ki ← Ki
   inst.config.Kd ← Kd

   RETURN FAULT_NONE

Operation: ``pid_set_limits``
-------------------------------

.. code-block:: none

   OPERATION pid_set_limits(inst:  POINTER TO PidInstance,
                            u_min: RON_FLOAT,
                            u_max: RON_FLOAT) → FaultCode

   -- (analogous pattern to pid_set_gains: validate, then apply atomically)

Operation: ``pid_set_mode``
----------------------------

.. code-block:: none

   OPERATION pid_set_mode(inst:       POINTER TO PidInstance,
                          new_mode:   OperatingMode,
                          manual_out: RON_FLOAT) → FaultCode

   IF inst = NULL THEN RETURN FAULT_NULL_POINTER END

   IF inst.state.mode = MODE_MANUAL AND new_mode = MODE_AUTOMATIC THEN
     -- Bumpless transfer: pre-load integral so that P + I + D = manual_out
     -- Assuming D ≈ 0 and P ≈ 0 at transfer instant (output-tracking method):
     manual_clamped ← clamp(manual_out, inst.config.u_min, inst.config.u_max)
     inst.state.integral   ← manual_clamped
     inst.state.u_sat_prev ← manual_clamped
     inst.state.u_prev     ← manual_clamped
   END

   IF inst.state.mode = MODE_AUTOMATIC AND new_mode = MODE_MANUAL THEN
     inst.state.u_sat_prev ← manual_out   -- track manual value from now
   END

   inst.state.mode ← new_mode
   RETURN FAULT_NONE

Operation: ``pid_get_state``
------------------------------

.. code-block:: none

   OPERATION pid_get_state(inst:    CONST POINTER TO PidInstance,
                           [out] integral: POINTER TO RON_FLOAT,
                           [out] last_u:   POINTER TO RON_FLOAT,
                           [out] last_D:   POINTER TO RON_FLOAT,
                           [out] status:   POINTER TO PidStatus,
                           [out] fault:    POINTER TO FaultCode) → FaultCode

   -- Copies state fields to caller-supplied pointers (null pointers for fields
   -- the caller does not need are silently ignored, not treated as errors).

------------------------------------------------------------------------

Bumpless Transfer Design
=========================

Automatic → Manual
-------------------

When switching from automatic to manual mode, the integral accumulator is frozen. The manual output supplied by the caller becomes the tracked output. No computation occurs during manual mode; ``u_sat_prev`` is updated to ``manual_out`` each step so that the back-calculation AW term remains consistent on return to automatic.

Manual → Automatic (Output-Tracking Method)
---------------------------------------------

The integral pre-load value is computed such that the controller output at the first automatic step equals the last manual output, provided the proportional and derivative terms are small (valid assumption when setpoint and process variable are close at transfer):

.. math::

   I_{pre} = u_{manual} - K_p \cdot e_{current} - D_{current}

For the general case (when :math:`e` and :math:`D` may not be negligible), the caller should:

1. Call ``pid_step()`` once immediately after switching to automatic.
2. The back-calculation AW will drive the output toward the manual value within one to two settling time constants.

The simpler approximation used in ``pid_set_mode`` (``I_pre = u_manual``) is correct when :math:`K_p \cdot e \approx 0` and is a safe starting point for most applications.

------------------------------------------------------------------------

Numerical Design Considerations
================================

Derivative Filter Stability
----------------------------

The first-order LP filter applied to the derivative has a pole at:

.. math::

   z_{pole} = \frac{1}{1 + N \cdot T_s}

For the filter to be stable and causal, :math:`N > 0` and :math:`T_s > 0` are required. The pole magnitude is always less than 1 for finite positive :math:`N` and :math:`T_s`, guaranteeing BIBO stability.

A typical range of :math:`5 \leq N \leq 20` provides adequate noise attenuation while preserving derivative response bandwidth. Very large :math:`N` (e.g., > 100) reduces the filter to a pure derivative and risks noise amplification.

Integral Accumulator Drift
---------------------------

Over extended operation (thousands of steps), the Euler integration method may exhibit slight drift relative to a higher-order integrator. The trapezoidal (Tustin) method reduces this error and is recommended for slow-sample-rate applications (:math:`T_s > 10` ms) or plants with long time constants.

For 32-bit float arithmetic, the integral accumulator should be reset (or normalised) if it approaches the limits of single-precision representation (>±1×10⁷). The ``I_overflow_thresh`` parameter provides this safeguard.

Back-Calculation Gain Selection
---------------------------------

The tracking time constant :math:`T_{aw}` governs how quickly the integrator is unwound after saturation recovery. A commonly recommended starting value is:

.. math::

   T_{aw} = \sqrt{T_i \cdot T_d} \quad \text{(geometric mean)}

or, in the parallel form:

.. math::

   T_{aw} = \frac{1}{\omega_{gc}}

where :math:`\omega_{gc}` is the gain crossover frequency of the open-loop system. For applications where :math:`T_d = 0` (PI controller), :math:`T_{aw} = T_i / 2` is a common heuristic.

------------------------------------------------------------------------

Safety-Critical Design Patterns
=================================

No Dynamic Allocation
----------------------

All state is encapsulated in ``PidInstance``, which is allocated by the caller (typically as a static variable or on a task stack). The library contains zero calls to any heap allocation function. This is verified by static analysis and linker map inspection during testing.

Bounded Execution
-----------------

The ``pid_core_step`` function contains:

- No loops.
- No recursion.
- No function pointers in the computational path (the fault callback is invoked outside the hot path).
- A fixed, bounded number of arithmetic operations.

This guarantees constant WCET, which is required for scheduling analysis in hard real-time systems.

Defensive Null-Pointer Policy
------------------------------

Every public API function tests all pointer parameters as its first action, before any state access. This pattern eliminates an entire class of memory-fault defects at the cost of a few compare instructions.

Fault Latch and Explicit Clear
-------------------------------

The fault register uses an OR-accumulation model: once set, a fault bit persists until explicitly cleared by ``pid_fault_clear()``. This prevents the controller from silently recovering from a transient fault condition that may indicate a deeper system problem. The caller is responsible for deciding the appropriate recovery action (reset, safe mode, shutdown).

Configuration Immutability During Computation
----------------------------------------------

Configuration parameters are copied into ``PidInstance.config`` at initialisation. All gain-update operations perform copy-validate-apply, ensuring the active configuration is always a validated snapshot. This eliminates tearing between a concurrent configuration write and a computation step.

Static Assertions
-----------------

The platform module shall include compile-time assertions verifying:

- ``sizeof(RON_FLOAT) == 4`` (or 8 in double mode).
- ``sizeof(uint32_t) == 4``.
- ``sizeof(PidState) <= 128`` (RAM footprint requirement).
- ``sizeof(PidConfig) <= 128``.

------------------------------------------------------------------------

Module Dependency Diagram
==========================

.. code-block:: none

   ron_pid_api
       │
       ├──► ron_pid_config    (validation)
       ├──► ron_pid_core      (computation)
       ├──► ron_pid_fault     (fault management)
       └──► ron_pid_types     (data structures)

   ron_pid_core
       ├──► ron_pid_types
       ├──► ron_pid_fault
       └──► ron_platform      (clamp, isnan, isinf, fabs)

   ron_pid_config
       ├──► ron_pid_types
       └──► ron_platform

   ron_pid_fault
       └──► ron_pid_types

   ron_platform              (no library dependencies beyond <stdint.h>, <float.h>)

All dependency arrows point downward. There are no circular dependencies.

------------------------------------------------------------------------

API Summary Table
=================

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - Operation
     - Returns
     - Description
   * - ``pid_init(inst, cfg)``
     - ``FaultCode``
     - Initialise instance; copy and validate config; zero state.
   * - ``pid_reset(inst)``
     - ``FaultCode``
     - Zero dynamic state; preserve config and mode.
   * - ``pid_step(inst, r, y, dt, u, status)``
     - ``FaultCode``
     - Execute one PID cycle; update state; return output and status.
   * - ``pid_set_gains(inst, Kp, Ki, Kd)``
     - ``FaultCode``
     - Atomically update gain parameters.
   * - ``pid_set_limits(inst, u_min, u_max)``
     - ``FaultCode``
     - Atomically update output limits.
   * - ``pid_set_filter(inst, N)``
     - ``FaultCode``
     - Update derivative filter coefficient.
   * - ``pid_set_antiwindup(inst, mode, T_aw)``
     - ``FaultCode``
     - Select and configure anti-windup scheme.
   * - ``pid_set_mode(inst, mode, manual_out)``
     - ``FaultCode``
     - Switch operating mode; perform bumpless transfer.
   * - ``pid_set_integral(inst, value)``
     - ``FaultCode``
     - Pre-load integrator (warm start).
   * - ``pid_get_state(inst, ...)``
     - ``FaultCode``
     - Read all internal state fields.
   * - ``pid_fault_clear(inst)``
     - ``FaultCode``
     - Clear fault register (latch released).
   * - ``pid_config_validate(cfg)``
     - ``FaultCode``
     - Validate a config record without applying it.

------------------------------------------------------------------------

Testing Architecture
====================

Test Levels
-----------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Level
     - Scope
     - Environment
   * - Unit Tests
     - Individual operations (each sub-operation in ron_pid_core, config validation rules, fault manager)
     - Native host (x86-64), no hardware required
   * - Integration Tests
     - Full pipeline through ``pid_step()`` for various plant models (simulated)
     - Native host; optionally also cross-compiled and run on target via JTAG
   * - Regression Tests
     - Full test suite re-run on every code change (CI/CD)
     - Native host
   * - Target Validation
     - WCET measurement, memory footprint, floating-point behaviour verification
     - Embedded hardware target or instruction-set accurate emulator (platform determined at integration)

Key Test Cases
--------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Test Case
     - Verification Goal
   * - Step response: first-order plant
     - Verify proportional, integral, derivative action individually; check steady-state error = 0.
   * - Saturation: output clamp
     - Confirm output never exceeds :math:`[u_{min}, u_{max}]` for any input magnitude.
   * - Anti-windup: large step with saturation
     - Compare recovery overshoot with AW enabled vs. disabled; assert improvement.
   * - Derivative kick: setpoint step with DoM
     - Assert no spike in output on setpoint step in DoM mode.
   * - Bumpless transfer: manual→auto
     - Assert output continuity (delta < 1%) at switchover instant.
   * - NaN/Inf input injection
     - Assert FAULT_INPUT_NAN set; safe-state output applied; fault latches.
   * - Zero division guard (T_aw = 0)
     - Assert ``pid_init`` returns ``FAULT_CONFIG_INVALID``.
   * - Multi-instance independence
     - Two instances with different configs run concurrently; verify no cross-contamination.
   * - Rate limit enforcement
     - Assert :math:`|\Delta u| \leq \dot{u}_{max} \cdot T_s` for all output transitions.
   * - Integral pre-load warm start
     - Assert first output after pre-load matches expected value.
   * - Integral accumulator clamp
     - Assert ``|I| ≤ I_max`` at all times regardless of error magnitude.

------------------------------------------------------------------------

Module Design: ron_filter
============================

Responsibility
--------------

Provides all standalone signal conditioning components. Every filter type
follows the same lifecycle pattern as ``ron_pid_api``: ``init``, ``reset``,
``step``, ``get_state``, all returning ``ron_fault_t``.

Common Filter State Model
--------------------------

Each filter instance holds:

- A configuration record (copied by value at init, validated before application).
- A state record (dynamic, zero-initialised at init/reset).
- An ``is_initialised`` guard.

First-Order IIR Low-Pass Filter
---------------------------------

**State:** ``y_prev`` (:math:`y(k-1)`).

**Step pseudocode:**

.. code-block:: none

   OPERATION lp1_step(inst, x, [out] y) → FaultCode
     IF NOT RON_ISFINITE(x)  THEN FAULT → safe output END
     y_new ← inst.cfg.alpha * x + (1.0 - inst.cfg.alpha) * inst.state.y_prev
     inst.state.y_prev ← y_new
     *y ← y_new
     RETURN FAULT_NONE

**Coefficient conversion from** :math:`f_c`, :math:`T_s`:

.. math::

   \omega_c = 2\pi f_c T_s, \quad \alpha = \frac{\omega_c}{1 + \omega_c}

**Validation:** :math:`0 < \alpha \leq 1`.

Moving Average Filter
----------------------

**State:** circular sample buffer of length ``M``, running sum, write index.

**Step pseudocode:**

.. code-block:: none

   OPERATION ma_step(inst, x, [out] y) → FaultCode
     oldest ← inst.state.buf[inst.state.idx]
     inst.state.sum ← inst.state.sum + x - oldest
     inst.state.buf[inst.state.idx] ← x
     inst.state.idx ← (inst.state.idx + 1) MOD inst.cfg.M
     *y ← inst.state.sum / inst.cfg.M
     RETURN FAULT_NONE

Buffer is statically sized to ``RON_MA_MAX_WINDOW`` samples; ``M`` is a
runtime parameter bounded by that constant.

Biquad IIR Filter (Direct Form II Transposed)
-----------------------------------------------

**Per-section state:** ``w1``, ``w2`` (two delay registers).

**Single-section step:**

.. code-block:: none

   w0  ← x - a1*w1 - a2*w2
   y   ← b0*w0 + b1*w1 + b2*w2
   w2  ← w1
   w1  ← w0

Direct Form II Transposed is used because it exhibits lower coefficient
sensitivity than Direct Form I under finite-precision arithmetic, making it
more suitable for single-precision floating-point embedded targets.

For cascaded sections, each section's output is the next section's input.
The maximum number of sections is ``RON_BIQUAD_MAX_SECTIONS`` (compile-time).

**Coefficient generation — notch filter:**

For a notch at :math:`f_0` with quality factor :math:`Q`:

.. math::

   \omega_0 = 2\pi f_0 T_s, \quad
   b_0 = b_2 = \frac{1}{1 + \omega_0/Q}, \quad
   b_1 = a_1 = \frac{-2\cos\omega_0}{1 + \omega_0/Q}, \quad
   a_2 = \frac{1 - \omega_0/Q}{1 + \omega_0/Q}

Runtime notch update recomputes only the five coefficients and replaces them
atomically without clearing ``w1``/``w2``, avoiding a transient discontinuity.

Standalone Rate Limiter
------------------------

**State:** ``y_prev``.

**Step pseudocode:**

.. code-block:: none

   delta_max_rise ←  inst.cfg.rate_rise * dt
   delta_max_fall ← -inst.cfg.rate_fall * dt
   delta ← x - inst.state.y_prev
   delta ← clamp(delta, delta_max_fall, delta_max_rise)
   *y ← inst.state.y_prev + delta
   inst.state.y_prev ← *y

------------------------------------------------------------------------

Module Design: ron_feedforward
=================================

Responsibility
--------------

Computes the feed-forward contribution :math:`u_{FF}(k)` and adds it to the
PID core output. Integrated into the ``ron_pid_core`` pipeline as a final
additive stage occurring before saturation.

Feed-Forward Derivative Estimation
------------------------------------

Velocity and acceleration FF require :math:`\dot{r}` and :math:`\ddot{r}`.
These are estimated using the same backward-difference + LP filter mechanism
as the PID derivative term, with an independently configurable filter
coefficient :math:`N_{FF}`:

.. code-block:: none

   r_dot_raw ← (r - r_prev) / dt
   r_dot_f   ← LP_filter(r_dot_raw, r_dot_prev, N_FF, dt)

   r_ddot_raw ← (r_dot_f - r_dot_f_prev) / dt
   r_ddot_f   ← LP_filter(r_ddot_raw, r_ddot_prev, N_FF, dt)

**Pipeline position:** FF is summed with ``u_raw`` *before* denormalisation,
saturation, and rate limiting, so all output constraints apply to the total
signal :math:`u_{PID} + u_{FF}`.

**Zero-overhead disabled path:** when all FF gains are zero and external FF is
not provided, the FF addition reduces to ``u_raw + 0.0``, which an optimising
compiler may eliminate entirely.

------------------------------------------------------------------------

Module Design: ron_gain_sched
================================

Data Structure
--------------

.. code-block:: none

   STRUCTURE GainSchedTable:
     n_points    : uint8_t                               -- number of breakpoints
     sigma       : RON_FLOAT[RON_GS_MAX_BREAKPOINTS]  -- scheduling variable values (ascending)
     configs     : ron_pid_config_t[RON_GS_MAX_BREAKPOINTS]  -- corresponding configs
     interp_mode : GsInterpMode    -- HARD_SWITCH or LINEAR_INTERP
     reset_on_switch : bool        -- integral reset on hard switch

Operation: ``ron_gs_update``
------------------------------

.. code-block:: none

   OPERATION ron_gs_update(table, pid_inst, sigma_val) → FaultCode

   -- Find bracketing interval
   i ← binary_search(table.sigma, sigma_val)

   IF table.interp_mode = HARD_SWITCH THEN
     apply config table.configs[i] to pid_inst atomically
     IF table.reset_on_switch THEN ron_pid_reset_integral(pid_inst) END

   ELSE  -- LINEAR_INTERP
     t ← (sigma_val - table.sigma[i]) / (table.sigma[i+1] - table.sigma[i])
     Kp_new ← lerp(table.configs[i].Kp, table.configs[i+1].Kp, t)
     Ki_new ← lerp(table.configs[i].Ki, table.configs[i+1].Ki, t)
     Kd_new ← lerp(table.configs[i].Kd, table.configs[i+1].Kd, t)
     ron_pid_set_gains(pid_inst, Kp_new, Ki_new, Kd_new)
   END

Binary search is used (O(log n)) for the lookup; table is validated as
strictly monotonically increasing at init, so binary search is always valid.

------------------------------------------------------------------------

Module Design: ron_cascade
=============================

Data Structure
--------------

.. code-block:: none

   STRUCTURE CascadeInstance:
     outer : ron_pid_instance_t
     inner : ron_pid_instance_t

Operation: ``ron_cascade_step``
---------------------------------

.. code-block:: none

   OPERATION ron_cascade_step(inst, r_out, y_out, y_in, dt,
                                [out] u_final, [out] status) → FaultCode

   -- 1. Run outer loop; its output becomes the inner setpoint
   fault_out ← ron_pid_step(&inst.outer, r_out, y_out, dt, &r_in, &st_out)

   -- 2. Run inner loop
   fault_in  ← ron_pid_step(&inst.inner, r_in,  y_in,  dt, &u_final, &st_in)

   -- 3. Anti-windup back-propagation: if inner saturated, reduce outer integral
   IF (st_in AND STATUS_SATURATED) THEN
     aw_correction ← (dt / outer.config.T_aw)
                      * (inner.state.u_sat_prev - inner.state.u_prev)
     outer.state.integral ← outer.state.integral + aw_correction
     outer.state.integral ← clamp(outer.state.integral,
                                   outer.config.I_min, outer.config.I_max)
   END

   -- 4. Merge status words (outer bits in high byte, inner in low byte)
   *status ← (ron_status_t)((st_out << 8U) | st_in)
   RETURN fault_out | fault_in

------------------------------------------------------------------------

Module Design: ron_trajectory
================================

Trapezoidal Profile State Machine
-----------------------------------

.. code-block:: none

   ENUM TrapPhase: ACCEL | CONST_VEL | DECEL | HOLD | DONE

   STRUCTURE TrapState:
     pos, vel, acc : RON_FLOAT
     phase         : TrapPhase
     target        : RON_FLOAT
     v_peak        : RON_FLOAT   -- computed for short-move case

**Phase transitions per step** (simplified):

.. code-block:: none

   ACCEL:
     vel ← vel + a_max * dt  (clamped to v_peak)
     pos ← pos + vel * dt
     IF deceleration must begin (distance to target ≤ v²/2a): → DECEL
     IF vel = v_peak: → CONST_VEL

   CONST_VEL:
     pos ← pos + v_max * dt
     IF distance to target ≤ v²/2a: → DECEL

   DECEL:
     vel ← vel - a_max * dt  (clamped to 0)
     pos ← pos + vel * dt
     IF vel = 0 AND pos ≈ target: vel ← 0, → DONE

**Short-move** peak velocity:
:math:`v_{peak} = \sqrt{a_{max} \cdot |target - pos_0|}`, clamped to
:math:`v_{max}`.

**Online goal update:** when target changes, the state machine recomputes
:math:`v_{peak}` from the current kinematic state without zeroing velocity,
ensuring a smooth continuous trajectory.

S-Curve Profile
----------------

The S-curve extends the trapezoidal with seven phases. Each phase duration is
computed analytically from the current kinematic state and the constraints
:math:`(v_{max}, a_{max}, j_{max})`. The per-step integration is:

.. code-block:: none

   jerk ← phase_jerk(phase)          -- ±j_max or 0
   acc  ← acc  + jerk * dt
   vel  ← vel  + acc  * dt
   pos  ← pos  + vel  * dt

Phase switching is by elapsed-time comparison against the analytically
precomputed phase durations. All seven durations are recomputed on goal update.

------------------------------------------------------------------------

Module Design: ron_kalman
============================

Data Structures
---------------

.. code-block:: none

   STRUCTURE KalmanConfig:
     n, m, p           : uint8_t      -- state, measurement, input dims
     A[n][n], B[n][p]  : RON_FLOAT   -- system matrices
     H[m][n]           : RON_FLOAT   -- measurement matrix
     Q[n][n]           : RON_FLOAT   -- process noise covariance
     R[m][m]           : RON_FLOAT   -- measurement noise covariance
     x0[n], P0[n][n]   : RON_FLOAT   -- initial estimate and covariance
     use_joseph_form   : bool
     steady_state      : bool         -- if true, use fixed K_inf
     K_inf[n][m]       : RON_FLOAT   -- steady-state gain (used if steady_state)

   STRUCTURE KalmanState:
     x_hat[n]          : RON_FLOAT   -- current state estimate
     P[n][n]           : RON_FLOAT   -- current error covariance

Predict–Update Pseudocode
--------------------------

.. code-block:: none

   OPERATION ron_kf_predict(inst, u[p]) → FaultCode
     x_prior ← A * inst.state.x_hat + B * u
     P_prior ← A * inst.state.P * A_T + Q
     inst.state.x_hat ← x_prior
     inst.state.P     ← P_prior

   OPERATION ron_kf_update(inst, z[m], z_valid: bool) → FaultCode
     IF NOT z_valid THEN RETURN FAULT_NONE END   -- measurement dropout

     IF inst.cfg.steady_state THEN
       K ← inst.cfg.K_inf
     ELSE
       S   ← H * P_prior * H_T + R
       K   ← P_prior * H_T * inv(S)   -- Cholesky solve for m>1
     END

     innov ← z - H * inst.state.x_hat
     inst.state.x_hat ← inst.state.x_hat + K * innov

     IF inst.cfg.use_joseph_form THEN
       IKH ← I - K*H
       inst.state.P ← IKH * P_prior * IKH_T + K * R * K_T
     ELSE
       inst.state.P ← (I - K*H) * P_prior
     END

All matrix operations are implemented as fixed-size loops over compile-time
bounded dimensions; no dynamic allocation occurs.

------------------------------------------------------------------------

Module Design: ron_statespace + ron_observer
================================================

State-Space Controller Pseudocode
-----------------------------------

.. code-block:: none

   OPERATION ron_ss_step(inst, r, u_obs[n], dt, [out] u, [out] status) → FaultCode

   -- 1. Obtain state estimate
   IF inst.cfg.state_source = EXTERNAL THEN
     x_hat ← inst.cfg.x_ext_ptr[0..n-1]
   ELSE IF inst.cfg.state_source = LUENBERGER THEN
     x_hat ← inst.observer.state.x_hat
   ELSE  -- KALMAN
     x_hat ← inst.kalman.state.x_hat
   END

   -- 2. Compute state-feedback term
   u_fb ← -K * x_hat    -- matrix-vector product

   -- 3. Integral augmentation (if enabled)
   IF inst.cfg.use_integral THEN
     e_reg ← r - C_out * x_hat
     inst.state.integral ← inst.state.integral + Ki_aug * dt * e_reg
     inst.state.integral ← clamp(inst.state.integral, I_min, I_max)
     u_raw ← u_fb + Kr * r + inst.state.integral
   ELSE
     u_raw ← u_fb + Kr * r
   END

   -- 4. Saturation + rate limit (identical to PID pipeline)
   u_sat   ← clamp(u_raw, u_min, u_max)
   u_final ← rate_limit(u_sat, u_sat_prev, du_max, dt)
   *u ← u_final

Luenberger Observer Pseudocode
--------------------------------

.. code-block:: none

   OPERATION ron_obs_step(inst, y[m], u[p], dt) → FaultCode
     innovation ← y - C * inst.state.x_hat
     x_new      ← A * inst.state.x_hat + B * u + L * innovation
     inst.state.x_hat ← x_new

------------------------------------------------------------------------

Module Design: ron_autotune
==============================

State Machine
-------------

.. code-block:: none

   ENUM AutotunePhase:
     AT_IDLE | AT_SETTLING | AT_RELAY | AT_ESTIMATING | AT_DONE | AT_ABORTED

Relay Operation
----------------

During ``AT_RELAY`` phase, the output is:

.. code-block:: none

   IF e(k) > +epsilon THEN u_relay ← u_bias + d
   IF e(k) < -epsilon THEN u_relay ← u_bias - d
   ELSE                    u_relay ← u_relay_prev  -- hysteresis hold

Zero-crossing events (sign changes of the error signal) are counted. The
oscillation period :math:`T_u` is estimated as twice the average half-period
measured between consecutive zero-crossings. Amplitude :math:`A` is the
peak-to-peak PV excursion divided by two, tracked over the last
``min_cycles`` complete cycles.

Estimation and Tuning
----------------------

.. code-block:: none

   K_u ← (4.0 * d) / (PI * A)
   T_u ← T_osc   -- measured period

   SWITCH tuning_rule:
     ZN:        Kp = 0.60*K_u, Ti = 0.50*T_u, Td = 0.125*T_u
     TL:        Kp = 0.45*K_u, Ti = 2.20*T_u, Td = 0.158*T_u
     SOME_OS:   Kp = 0.33*K_u, Ti = 0.50*T_u, Td = 0.333*T_u
     NO_OS:     Kp = 0.20*K_u, Ti = 0.50*T_u, Td = 0.333*T_u

Convert :math:`T_i`, :math:`T_d` to parallel gains:
:math:`K_i = K_p / T_i`,  :math:`K_d = K_p \cdot T_d`.

------------------------------------------------------------------------

Module Design: ron_health
============================

Each health condition is implemented as an independent, stateful comparator:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Condition
     - Detection Logic
   * - ``OUTPUT_STUCK``
     - Increment saturation counter each step the output equals ``u_min`` or
       ``u_max``; reset on unsaturation. Trigger when counter × ``dt`` >
       ``t_sat_max``.
   * - ``DIVERGING``
     - Trigger when ``|e(k)| > err_threshold`` AND ``sign(e(k)) == sign(e(k)-e(k-1))``,
       i.e., error is large and still growing.
   * - ``OSCILLATING``
     - Maintain a sliding window of ``window_size`` steps; count sign changes of
       ``e(k)``. Trigger when count > ``osc_threshold``.
   * - ``SENSOR_DROPOUT``
     - Trigger when ``|y(k) - y_prev|`` < ``dead_band`` for more than
       ``dropout_time`` seconds continuously.
   * - ``SP_UNREACHABLE``
     - Trigger when ``|e(k)| > ss_err_threshold`` persists beyond
       ``settling_time`` after a setpoint step is detected.

All counters and window states are reset on explicit ``ron_health_clear()``.

------------------------------------------------------------------------

Module Design: ron_metrics
============================

.. code-block:: none

   STRUCTURE MetricsState:
     IAE, ISE, ITAE    : RON_FLOAT   -- running integrals
     peak_overshoot    : RON_FLOAT
     rise_time         : RON_FLOAT   -- -1.0 = not yet reached
     settling_time     : RON_FLOAT   -- -1.0 = not yet settled
     step_ref          : RON_FLOAT   -- PV at step detection
     step_target       : RON_FLOAT   -- r at step detection
     step_size         : RON_FLOAT
     t_elapsed         : RON_FLOAT
     rise_10pct_crossed : bool
     in_band           : bool
     in_band_time      : RON_FLOAT
     window_counter    : uint32_t

Per-step update (when enabled):

.. code-block:: none

   t_elapsed ← t_elapsed + dt
   e_abs ← fabs(e)
   IAE  ← IAE  + e_abs * dt
   ISE  ← ISE  + e*e  * dt
   ITAE ← ITAE + t_elapsed * e_abs * dt

   -- Rise time
   pv_pct ← (y - step_ref) / step_size
   IF NOT rise_10pct_crossed AND pv_pct >= 0.10: t_rise_start ← t_elapsed
   IF     rise_10pct_crossed AND pv_pct >= 0.90 AND rise_time < 0:
     rise_time ← t_elapsed - t_rise_start

   -- Overshoot
   IF step_size > 0 AND y > step_target + EPSILON:
     peak_overshoot ← MAX(peak_overshoot, (y - step_target) / step_size * 100)

   -- Settling (2% band default)
   IF e_abs <= fabs(step_size) * band_pct:
     in_band_time ← in_band_time + dt
     IF in_band_time >= settle_confirm_time AND settling_time < 0:
       settling_time ← t_elapsed
   ELSE:
     in_band_time ← 0.0

------------------------------------------------------------------------

Updated Module Dependency Diagram
===================================

.. code-block:: none

   Application / HAL
       │
   ┌───┴──────────────────────────────────────────────────────────────┐
   │  Public API Layer                                                │
   │  ron_pid_api  ron_cascade  ron_gs  ron_autotune             │
   │  ron_filter   ron_trajectory  ron_ss  ron_health  ron_metrics│
   └───┬──────────────────────────────────────────────────────────────┘
       │
   ┌───┴──────────────────────────────────────────────────────────────┐
   │  Core Computation Layer                                          │
   │  ron_pid_core   ron_feedforward   ron_observer   ron_kalman  │
   └───┬──────────────────────────────────────────────────────────────┘
       │
   ┌───┴──────────────────────────────────────────────────────────────┐
   │  Support Layer                                                   │
   │  ron_pid_config   ron_pid_fault   ron_pid_types               │
   └───┬──────────────────────────────────────────────────────────────┘
       │
   ┌───┴──────────────────────────────────────────────────────────────┐
   │  Platform Abstraction                                            │
   │  ron_platform  (ron_float_t, clamp, isnan, static_assert)     │
   └──────────────────────────────────────────────────────────────────┘

All dependency arrows point downward. No circular dependencies.

------------------------------------------------------------------------

Appendix A: Design Decision Log
=================================

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - ID
     - Decision
     - Rationale
   * - DD-01
     - Derivative-on-measurement as default.
     - Eliminates derivative kick on setpoint step, which is the most common source of actuator stress in embedded control loops. DoE available for applications where fast response to setpoint changes is required.
   * - DD-02
     - Back-calculation AW as default.
     - Provides smoother, parametric recovery compared to clamping. Clamping is provided as an alternative for simpler implementations or legacy compatibility.
   * - DD-03
     - Parallel (non-interacting) PID form as primary form.
     - Allows independent tuning of P, I, D without gain coupling, which simplifies iterative tuning in embedded applications. ISA form available via trivial parameter transformation.
   * - DD-04
     - Fault latch model (requires explicit clear).
     - Prevents silent fault recovery. The application must explicitly acknowledge and handle faults, consistent with safety-critical practice in IEC 61508 and DO-178C.
   * - DD-05
     - Configuration copied by value into PidInstance.
     - Avoids aliasing and pointer lifetime issues. Ensures config is always self-consistent and locally owned by the instance. Slight RAM overhead justified by safety benefit.
   * - DD-06
     - No floating-point exceptions (trapping disabled).
     - Many embedded processors raise hardware traps or faults on floating-point anomalies (NaN, Inf, divide-by-zero). Handling such traps safely in an ISR context is complex and highly platform-specific. The design instead uses explicit NaN/Inf checks at defined points in the pipeline to handle floating-point anomalies predictably, portably, and without reliance on any hardware exception mechanism.
   * - DD-07
     - Tustin derivative filter discretisation over bilinear transform at full.
     - The first-order LP filter discretised by Euler backward is sufficient for derivative filtering and avoids the frequency warping present in general bilinear transform at the frequencies of interest (well below Nyquist/2).
   * - DD-08
     - Language-agnostic design in this document.
     - Separates the algorithmic and safety specification from implementation concerns. Allows language selection (C99, C11, C++14) to be driven by project toolchain and certification requirements without re-specifying the architecture.
   * - DD-09
     - Biquad filter uses Direct Form II Transposed.
     - Lower coefficient sensitivity than Direct Form I under single-precision arithmetic, reducing the risk of instability or quantisation noise on targets without double-precision FPU.
   * - DD-10
     - Gain scheduling uses binary search on breakpoint table.
     - O(log n) lookup is deterministic and WCET-analysable for a bounded table. Linear scan would also be acceptable for small tables (< 8 points) but binary search generalises cleanly.
   * - DD-11
     - Cascade anti-windup propagation operates on the outer integrator directly.
     - The standard cascade AW approach feeds the inner saturation error back through the outer loop's tracking time constant, which is the same back-calculation mechanism already validated for single PID. This avoids a separate AW mechanism and reuses tested code paths.
     - Prevents outer integrator from winding against inner loop saturation without introducing new state or parameters.
   * - DD-12
     - Kalman filter uses Cholesky decomposition for the innovation covariance inversion (m > 1).
     - Explicit matrix inversion is numerically unstable for ill-conditioned covariance matrices. Cholesky is O(m³/3) and requires the matrix to be positive-definite, which is guaranteed by the R matrix validity check. No general-purpose linear solver is required.
   * - DD-13
     - Kalman Joseph form update is opt-in.
     - The Joseph form is more expensive (two extra matrix multiplications) but preserves positive-definiteness of P under finite-precision arithmetic. Made opt-in to avoid paying the cost on targets where standard update is numerically sufficient.
   * - DD-14
     - Trajectory generators compute phase durations analytically at goal-set time, not per-step.
     - Analytical precomputation is O(1) at goal set and makes the per-step integration trivially cheap (one addition per state variable). This is preferred over online phase detection which requires floating-point comparisons subject to accumulated error.
   * - DD-15
     - Auto-tuner uses zero-crossing counting for period estimation rather than FFT.
     - FFT requires a buffer of N samples and O(N log N) computation, impractical for a constrained embedded target with unknown oscillation frequency. Zero-crossing counting is O(1) per step and sufficiently accurate for relay feedback, which produces near-sinusoidal oscillations.
   * - DD-16
     - Health monitor is entirely passive (read-only).
     - Mixing diagnostic logic with control logic creates coupling that complicates safety analysis and testing. A passive observer can be added, removed, or replaced without any effect on the controller's certified computational path.
   * - DD-17
     - Metrics accumulator is disabled by default and zero-overhead when off.
     - Runtime metrics add floating-point operations per step. In production embedded systems these are often not needed. A runtime enable/disable flag (rather than compile-time) allows the same binary to be used for commissioning (metrics on) and production (metrics off).
   * - DD-18
     - Dual-language implementation: C11 and Rust Edition 2021, both first-class.
     - C11 maximises toolchain coverage (every embedded target, every certified compiler) and interoperability (C ABI is the universal FFI boundary). Rust provides structural memory safety via the borrow checker, richer type-level abstractions (const generics for matrix dimensions, typestate for mode transitions), and superior free static analysis and formal verification via Kani. Neither benefit can be fully replicated in the other language without significant effort. Maintaining both tracks provides the project with maximum deployment flexibility and allows safety cases to be constructed under both the established MISRA C certification path and the emerging MISRA Rust / Ferrocene path.

------------------------------------------------------------------------

Appendix B: Glossary
=====================

See Glossary in RON-SRS-001. Additional terms:

.. glossary::

   Back-calculation anti-windup
      An anti-windup strategy in which a feedback signal proportional to the difference between the saturated and unsaturated controller output is fed back to the integrator input, gradually winding down the excess integral accumulation.

   Bumpless transfer
      A control technique that ensures continuity of the controller output when switching between manual and automatic modes, preventing abrupt actuator commands.

   Conditional integration clamping
      An anti-windup strategy in which the integrator update is suspended whenever the controller output is saturated and the current error would cause further windup.

   Derivative kick
      An undesired spike in the derivative term caused by an instantaneous change in the setpoint, amplified when derivative-on-error mode is used. Eliminated by switching to derivative-on-measurement.

   Two-degree-of-freedom (2DOF) PID
      A PID structure with separate setpoint weighting factors for the proportional and derivative terms, allowing independent tuning of setpoint tracking and disturbance rejection performance.

------------------------------------------------------------------------

*End of Document — RON-SADS-001 v1.1.0*

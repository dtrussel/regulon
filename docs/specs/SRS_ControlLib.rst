.. ============================================================
.. Software Requirements Specification
.. Regulon — PID Controller Module
.. ============================================================

.. meta::
   :description: Software Requirements Specification for the Regulon, PID Controller Module.
   :keywords: PID, control systems, embedded, SRS, requirements, safety-critical

########################################################################
Software Requirements Specification
########################################################################

**Document Title:** Software Requirements Specification — Regulon, PID Controller Module

**Document ID:** RON-SRS-001

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
     - 2025-03-01
     - Initial draft
     - TBD
   * - 1.0.0
     - 2025-04-10
     - First baseline release
     - TBD
   * - 1.1.0
     - 2025-04-10
     - Added requirements for: signal conditioning filters, feed-forward,
       gain scheduling, cascade controller, trajectory generators, Kalman
       filter, state-space controller, Luenberger observer, relay auto-tuning,
       health monitor, performance metrics (RON-FR-100 – FR-954).
       Updated traceability matrix.
     - TBD

------------------------------------------------------------------------

Introduction
============

Purpose
-------

This document specifies the software requirements for the **Regulon Control Systems Library**, focusing on its first deliverable module: the **PID (Proportional–Integral–Derivative) Controller**. It defines functional, performance, safety, and quality requirements that the library shall satisfy, serving as the contractual baseline between stakeholders and the development team.

Scope
-----

The Regulon PID module provides a reusable, configurable, and safety-aware PID controller implementation suitable for deployment on resource-constrained embedded processors, including (but not limited to) 32-bit and 16-bit microcontrollers. The library is intended for use in closed-loop control applications including, but not limited to:

- Motor speed and position control
- Temperature regulation
- Pressure control
- Attitude and stabilisation systems (robotics, UAVs)
- Industrial process control

The library **does not** include:

- Hardware abstraction layers (HAL) or driver code
- Sensor acquisition or signal conditioning hardware
- Communication protocol stacks
- Real-time operating system (RTOS) integration (compatibility is required, but integration is out of scope)

Definitions, Acronyms, and Abbreviations
-----------------------------------------

.. glossary::

   Regulon
      Regulon — the software library described in this document.

   PID
      Proportional–Integral–Derivative controller.

   SRS
      Software Requirements Specification.

   SADS
      Software Architecture and Design Specification.

   MCU
      Microcontroller Unit.

   ISR
      Interrupt Service Routine.

   RTOS
      Real-Time Operating System.

   SIL
      Safety Integrity Level (as defined by IEC 61508).

   ASIL
      Automotive Safety Integrity Level (as defined by ISO 26262).

   WCET
      Worst-Case Execution Time.

   API
      Application Programming Interface.

   AW
      Anti-Windup.

   LP
      Low-Pass (filter).

   SP
      Setpoint.

   PV
      Process Variable (measured output).

   CV
      Control Variable (controller output, command to actuator).

   NaN
      Not-a-Number (IEEE 754 floating-point special value).

   IEC 61508
      International standard: *Functional Safety of Electrical/Electronic/Programmable Electronic Safety-related Systems*.

   ISO 26262
      International standard: *Road Vehicles — Functional Safety*.

   IEC 62304
      International standard: *Medical Device Software — Software Life Cycle Processes*.

   DO-178C
      Aerospace standard: *Software Considerations in Airborne Systems and Equipment Certification*.

References
----------

.. list-table::
   :header-rows: 1
   :widths: 10 90

   * - ID
     - Reference
   * - [REF-01]
     - IEC 61508:2010, *Functional Safety of E/E/PE Safety-related Systems*, Parts 1–7. IEC.
   * - [REF-02]
     - ISO 26262:2018, *Road Vehicles — Functional Safety*, Parts 1–12. ISO.
   * - [REF-03]
     - IEC 62304:2015, *Medical Device Software — Software Life Cycle Processes*. IEC.
   * - [REF-04]
     - Åström, K.J. & Hägglund, T. (2006). *Advanced PID Control*. ISA.
   * - [REF-05]
     - Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2019). *Feedback Control of Dynamic Systems*, 8th ed. Pearson.
   * - [REF-06]
     - Bohn, C. & Atherton, D.P. (1995). "An analysis package comparing PID anti-windup strategies." *IEEE Control Systems*, 15(2), 34–40.
   * - [REF-07]
     - Peng, Y., Vrančić, D., & Hanus, R. (1996). "Anti-windup, bumpless, and conditioned transfer techniques for PID controllers." *IEEE Control Systems*, 16(4), 48–57.
   * - [REF-08]
     - IEEE 754:2019, *Standard for Floating-Point Arithmetic*. IEEE.
   * - [REF-09]
     - DO-178C:2011, *Software Considerations in Airborne Systems and Equipment Certification*. RTCA.

Overview
--------

This document is organised as follows:

- **Section 2** — Overall description: product perspective, user characteristics, assumptions, and constraints.
- **Section 3** — Functional requirements.
- **Section 4** — Performance requirements.
- **Section 5** — Safety and dependability requirements.
- **Section 6** — Quality attribute requirements.
- **Section 7** — Interface requirements.
- **Section 8** — Design constraints.
- **Section 9** — Verification requirements.

------------------------------------------------------------------------

Overall Description
===================

Product Perspective
-------------------

The Regulon PID module is a standalone, self-contained software component with no dependencies on operating system services, dynamic memory allocation, or external libraries. It is designed to be integrated into larger application firmware either directly or through a thin RTOS task wrapper.

The module exposes a pure functional interface: the caller is responsible for periodic invocation (scheduling), sensor reading (providing the process variable), and actuator driving (consuming the control variable). The library performs all control calculations internally.

.. figure:: _static/context_diagram.png
   :alt: Context diagram showing Regulon PID module boundaries
   :align: center

   *Figure 1 — Regulon PID Module Context Diagram (see SADS for detailed diagrams)*

Product Functions — Summary
-----------------------------

The PID module shall provide the following high-level capabilities:

1. Discrete-time PID computation (ideal, standard, and parallel forms).
2. Configurable proportional, integral, and derivative gains.
3. Derivative-on-measurement (DoM) and derivative-on-error (DoE) modes.
4. First-order low-pass filter on the derivative term.
5. Output saturation (hard limits on the control variable).
6. Input normalisation / scaling.
7. Anti-windup mechanisms: back-calculation and conditional integration clamping.
8. Output rate-of-change limiting.
9. Bumpless transfer (manual-to-automatic mode switching).
10. Setpoint pre-filtering (setpoint weighting or first-order SP filter).
11. Fault detection and safe-state enforcement.
12. Multi-instance support (multiple independent controller instances).
13. Runtime parameter update with consistency checks.

User Characteristics and Stakeholders
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Stakeholder
     - Role / Interest
   * - Firmware / Embedded Engineer
     - Integrates and configures the library in application firmware; primary API consumer.
   * - Control Engineer
     - Tunes controller parameters; requires mathematical fidelity and adequate feature set.
   * - System Safety Engineer
     - Verifies compliance with functional safety requirements (IEC 61508, ISO 26262, etc.).
   * - Software Quality / Validation Engineer
     - Reviews, tests, and certifies the library against this SRS.
   * - System Integrator
     - Deploys the library on specific hardware targets; concerned with portability and WCET.

Assumptions and Dependencies
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 10 90

   * - ID
     - Assumption / Dependency
   * - RON-ASM-01
     - The host processor supports IEEE 754 single-precision (32-bit) floating-point arithmetic in hardware or via a software library.
   * - RON-ASM-02
     - The controller ``step`` function is called at a nominally periodic interval with a known, bounded sample period ``dt``. Jitter shall be managed externally.
   * - RON-ASM-03
     - The caller guarantees that the process variable input is a valid, bounded, finite floating-point value before each call.
   * - RON-ASM-04
     - The target system provides sufficient stack space for at least one nested function call frame per controller instance.
   * - RON-ASM-05
     - No dynamic memory allocation (heap) is available or required; all state is statically allocated.
   * - RON-ASM-06
     - The library may be used in a bare-metal or RTOS environment; mutual-exclusion around shared controller state is the caller's responsibility.
   * - RON-ASM-07
     - The programming language and toolchain to be selected during the design phase are capable of producing deterministic, WCET-analysable machine code.

------------------------------------------------------------------------

Functional Requirements
=======================

Requirement Notation
---------------------

Each requirement is identified by a unique ID of the form ``RON-FR-NNN``. The **shall** keyword denotes a mandatory requirement; **should** denotes a strong recommendation; **may** denotes a permitted option.

PID Computation — Core Algorithm
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-001
     - The library **shall** implement a discrete-time PID controller based on the **parallel (non-interacting) form**:

       .. math::

          u(k) = K_p \cdot e(k)
                 + K_i \cdot T_s \cdot \sum_{j=0}^{k} e(j)
                 + K_d \cdot \frac{e(k) - e(k-1)}{T_s}

       where :math:`e(k) = r(k) - y(k)` is the control error, :math:`r(k)` the setpoint, :math:`y(k)` the process variable, and :math:`T_s` the sample period.

   * - RON-FR-002
     - The library **shall** support the **ideal (ISA) form** as an alternative, where the integral and derivative gains are expressed as:

       :math:`K_i = K_p / T_i` and :math:`K_d = K_p \cdot T_d`

       with :math:`T_i` (integral time) and :math:`T_d` (derivative time) as configuration parameters.

   * - RON-FR-003
     - The controller **shall** compute the proportional term as:

       :math:`P(k) = K_p \cdot e(k)`

   * - RON-FR-004
     - The controller **shall** compute the integral term using the **rectangular (Euler forward) integration** method as the default, with **trapezoidal (Tustin)** integration available as a selectable option.

   * - RON-FR-005
     - The controller **shall** support two derivative modes, selectable at configuration time:

       a. **Derivative on Error (DoE):** :math:`D(k) = K_d \cdot \frac{e(k) - e(k-1)}{T_s}`
       b. **Derivative on Measurement (DoM):** :math:`D(k) = -K_d \cdot \frac{y(k) - y(k-1)}{T_s}`

       DoM **shall** be the default mode to eliminate derivative kick on setpoint steps.

   * - RON-FR-006
     - The library **shall** apply a configurable first-order low-pass (Tustin-discretised) filter to the derivative term to attenuate high-frequency noise:

       :math:`D_f(k) = \frac{N \cdot K_d}{T_s + N^{-1}} \cdot \Delta y(k)`

       where :math:`N` is the derivative filter coefficient (bandwidth multiplier, typically :math:`5 \leq N \leq 20`). When :math:`N = 0`, the filter **shall** be disabled.

   * - RON-FR-007
     - The library **shall** support a **setpoint weight** :math:`b` on the proportional term and :math:`c` on the derivative term (two-degree-of-freedom PID), modifying the error terms as:

       :math:`e_P(k) = b \cdot r(k) - y(k)` and :math:`e_D(k) = c \cdot r(k) - y(k)`

       Both weights **shall** default to 1.0 (standard PID behaviour).

Input / Output Normalisation
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-010
     - The library **shall** provide configurable input scaling such that both the setpoint :math:`r` and process variable :math:`y` can be mapped from engineering units to a normalised range :math:`[-1, +1]` or :math:`[0, 1]` prior to error computation.
   * - RON-FR-011
     - The scaling transformation **shall** be defined by a minimum (:math:`x_{min}`) and maximum (:math:`x_{max}`) value per signal, applying the linear map:

       :math:`x_{norm} = \frac{x - x_{min}}{x_{max} - x_{min}}`

   * - RON-FR-012
     - The library **shall** provide configurable output scaling to map the computed control variable from the normalised domain back to actuator engineering units.
   * - RON-FR-013
     - Normalisation **shall** be an optional feature; when disabled, all computations proceed in the raw engineering-unit domain.

Output Saturation
-----------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-020
     - The library **shall** enforce configurable hard limits on the control variable output:

       :math:`u_{sat}(k) = \text{clamp}(u(k),\ u_{min},\ u_{max})`

   * - RON-FR-021
     - The output limits :math:`u_{min}` and :math:`u_{max}` **shall** be configurable at initialisation and **shall** be updatable at runtime without reinitialisation, provided the new limits are consistent (:math:`u_{min} < u_{max}`).
   * - RON-FR-022
     - When saturation is active (:math:`u(k) \neq u_{sat}(k)`), the library **shall** set a ``SATURATED`` status flag visible to the caller.

Output Rate Limiting
---------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-025
     - The library **shall** support an optional rate-of-change limit on the control variable:

       :math:`|\Delta u(k)| = |u_{sat}(k) - u_{sat}(k-1)| \leq \dot{u}_{max} \cdot T_s`

   * - RON-FR-026
     - When :math:`\dot{u}_{max} \leq 0`, rate limiting **shall** be disabled.
   * - RON-FR-027
     - Rate limiting **shall** be applied **after** output saturation.

Anti-Windup
-----------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-030
     - The library **shall** implement anti-windup to prevent unbounded growth of the integral accumulator when the output is saturated.
   * - RON-FR-031
     - The library **shall** support the **back-calculation (tracking)** anti-windup scheme. In this method, a correction term proportional to the difference between the unsaturated and saturated outputs is fed back to the integrator:

       :math:`I(k+1) = I(k) + K_i \cdot T_s \cdot e(k) + \frac{T_s}{T_{aw}} \cdot (u_{sat}(k) - u(k))`

       where :math:`T_{aw}` is the anti-windup tracking time constant (a tunable parameter).

   * - RON-FR-032
     - The library **shall** support **conditional integration clamping** as an alternative anti-windup scheme: the integrator update **shall** be suppressed when both of the following conditions hold simultaneously:

       a. The output is saturated.
       b. The current error has the same sign as the integrator output (i.e., continued integration would deepen the windup).

   * - RON-FR-033
     - The anti-windup scheme **shall** be selectable at configuration time. The default **shall** be back-calculation.
   * - RON-FR-034
     - It **shall** be possible to disable anti-windup entirely (e.g., for plants that never saturate their actuators).
   * - RON-FR-035
     - The library **shall** allow setting a symmetric or asymmetric **integral accumulator clamp** as an independent safeguard:

       :math:`I(k) = \text{clamp}(I(k),\ I_{min},\ I_{max})`

Bumpless Transfer
-----------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-040
     - The library **shall** support a **manual override mode** in which the caller supplies the desired output directly, bypassing PID computation.
   * - RON-FR-041
     - When transitioning from manual mode to automatic (closed-loop) mode, the library **shall** perform a **bumpless transfer** by pre-loading the integral accumulator to a value that makes the controller output equal to the last manual output at the moment of switchover.
   * - RON-FR-042
     - When transitioning from automatic mode to manual mode, the library **shall** freeze the integral accumulator at its current value.

Controller State Management
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-050
     - The library **shall** support explicit **initialisation** of each controller instance to a defined, deterministic state.
   * - RON-FR-051
     - The library **shall** support **reset** of a controller instance, returning all dynamic state (integral accumulator, derivative history, filter state) to zero without modifying configuration parameters.
   * - RON-FR-052
     - The library **shall** support **pre-loading** the integral accumulator with an externally supplied value to facilitate warm-start scenarios (e.g., after power cycling with state restoration from non-volatile memory).
   * - RON-FR-053
     - The library **shall** support **runtime update** of all tuning parameters (:math:`K_p`, :math:`K_i`, :math:`K_d`, filter coefficient, limits) with an atomicity guarantee: parameters **shall** be updated as a coherent set, not individually in a potentially inconsistent sequence.
   * - RON-FR-054
     - Upon a setpoint change exceeding a configurable threshold, the library **shall** optionally reset the integral term to avoid initial windup transients.

Multi-Instance Support
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-060
     - The library **shall** support simultaneous independent instances of PID controllers, limited only by available memory.
   * - RON-FR-061
     - All instance state **shall** be encapsulated in an opaque data structure; no global mutable state **shall** be used.
   * - RON-FR-062
     - Each instance **shall** be fully independent: configuration, state, and execution of one instance **shall** not affect any other.

Status and Diagnostics
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-070
     - After each computation step, the library **shall** return a **status word** containing the following bitfield flags:

       - ``OK`` — computation completed normally.
       - ``SATURATED`` — output clipped by min/max limits.
       - ``RATE_LIMITED`` — output change was clipped by the rate limiter.
       - ``ANTI_WINDUP_ACTIVE`` — anti-windup correction was applied.
       - ``MANUAL_MODE`` — controller is in manual override mode.
       - ``FAULT`` — a fault condition was detected (see safety requirements).

   * - RON-FR-071
     - The library **shall** expose getter functions to retrieve each internal state variable (integral accumulator, last derivative, last output) for monitoring, logging, or external state estimation.

------------------------------------------------------------------------

Performance Requirements
========================

Timing and Real-Time Constraints
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-PR-001
     - The ``step`` function (single controller iteration) **shall** have a **bounded, deterministic WCET** that is analysable by static or measurement-based timing analysis tools (e.g., OTAWA, or instrumented cycle-counter profiling).
   * - RON-PR-002
     - The implementation **shall** contain **no unbounded loops**, **no recursion**, and **no dynamic memory allocation**, ensuring O(1) time complexity per call.
   * - RON-PR-003
     - The ``step`` function **shall** complete within a WCET budget that permits the library to be used at sample rates of at least **10 kHz** on the target platform for which it is deployed. The specific WCET budget **shall** be established and documented during target-specific integration, using static timing analysis or measured worst-case profiling.
   * - RON-PR-004
     - The library **shall** be safe to call from a periodic interrupt service routine (ISR) context. It **shall** not perform any operation that could block, yield, or execute for an unbounded duration.

Numerical Precision
--------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-PR-010
     - The library **shall** use at minimum **32-bit single-precision IEEE 754** floating-point arithmetic for all PID state and computation.
   * - RON-PR-011
     - A **64-bit double-precision** configuration option **shall** be available for targets that support it, selectable at compile time.
   * - RON-PR-012
     - The library **shall** detect and handle IEEE 754 special values (NaN, ±Inf) in inputs and intermediate results according to RON-SR-020.

Memory
------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-PR-020
     - The **code size** of the compiled PID module (all functions, no debug symbols) **shall** be minimised to the extent required by the target platform. The specific code-size budget **shall** be established during target-specific integration. As a design target, the module **should** compile to no more than a few kilobytes of machine code on a typical 32-bit embedded target.
   * - RON-PR-021
     - The **RAM footprint** of a single controller instance **shall** not exceed **128 bytes** (single-precision configuration).
   * - RON-PR-022
     - The library **shall** use **no heap memory** (``malloc``, ``free``, or equivalent). All state is stack- or statically allocated by the caller.

------------------------------------------------------------------------

Safety and Dependability Requirements
======================================

Defensive Programming
----------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-SR-001
     - All public API entry points **shall** validate their input parameters and return a defined error code upon detecting invalid arguments, rather than invoking undefined behaviour.
   * - RON-SR-002
     - Division operations **shall** be guarded against zero-divisors. Any configuration parameter used as a denominator (e.g., :math:`T_s`, :math:`T_i`, normalisation range) **shall** be checked for zero or near-zero values at configuration time; the library **shall** return a configuration error if violated.
   * - RON-SR-003
     - The library **shall not** use dynamic memory allocation at any point in its implementation.
   * - RON-SR-004
     - The library **shall not** use recursion.
   * - RON-SR-005
     - The library **shall not** access arrays with variable or unchecked indices.
   * - RON-SR-006
     - All pointer parameters in the API **shall** be validated as non-null before dereferencing.

Fault Detection and Safe-State
-------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-SR-010
     - The library **shall** detect the following fault conditions and set the ``FAULT`` status flag:

       - Input process variable is NaN or infinite.
       - Computed control variable is NaN or infinite before saturation.
       - Integral accumulator has grown beyond a configurable overflow threshold.
       - Configuration parameters are inconsistent (e.g., :math:`u_{min} \geq u_{max}`).

   * - RON-SR-011
     - Upon fault detection, the library **shall** apply a **safe-state output policy**, configurable to one of:

       a. **Hold last valid output** — output is frozen at the last non-faulted value.
       b. **Drive to zero** — output is forced to 0.0.
       c. **Drive to configured safe value** — output is forced to a caller-specified safe constant.

   * - RON-SR-012
     - Once a fault is detected, the controller **shall** remain in the fault state until explicitly cleared by the caller via a dedicated reset/clear function, preventing silent recovery.
   * - RON-SR-013
     - The library **shall** maintain a **fault code register** identifying the category and source of the most recent fault.

Numerical Integrity
--------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-SR-020
     - The library **shall** detect IEEE 754 NaN and infinite values in all floating-point inputs and computed intermediates before use, using portable compile-time or runtime checks.
   * - RON-SR-021
     - All arithmetic operations **shall** be structured to minimise catastrophic cancellation and accumulation of round-off error in the integral term over extended operation periods.
   * - RON-SR-022
     - The integral accumulator **shall** be independently clamped to a finite range (RON-FR-035) as an additional safeguard against numerical overflow, separate from the anti-windup mechanism.

Standards Alignment
--------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-SR-030
     - The library source code **shall** be written in conformance with an
       appropriate, recognised safety coding standard for the selected
       implementation language. For the C11 implementation this is
       **MISRA C:2023**; for the Rust implementation this is **MISRA Rust:2024**
       supplemented by ``cargo clippy`` mandatory-warning compliance. The
       applicable standard shall be mandated in RON-IS-001 and all deviations
       **shall** be documented and justified.
   * - RON-SR-031
     - The library **shall** be developed following a software life-cycle process
       compatible with **IEC 61508 SIL 2** requirements as a baseline. Higher SIL
       applicability (SIL 3/4, ASIL C/D) shall be achievable through supplementary
       verification activities performed by the integrator.
   * - RON-SR-032
     - The library **shall** provide traceability from each source code unit to the
       requirements in this SRS, suitable for safety case arguments.
   * - RON-SR-033
     - The library code **shall** be analysable by freely available static analysis
       tools without suppressing safety-relevant findings. For the C11 track:
       ``cppcheck`` (MISRA addon), Clang Static Analyzer, and CBMC. For the Rust
       track: ``cargo clippy`` (pedantic mode) and the Kani bounded model checker.

------------------------------------------------------------------------

Quality Attribute Requirements
================================

Portability
-----------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-QR-001
     - The library **shall** be portable across 32-bit and 64-bit architectures without modification to its core algorithmic logic.
   * - RON-QR-002
     - The library **shall** avoid architecture-specific intrinsics or compiler extensions in its portable core; hardware-specific optimisations (e.g., FPU intrinsics) **shall** be isolated in platform-specific build-configuration files.
   * - RON-QR-003
     - Numeric type widths **shall** be specified using exact-width types (e.g., ``uint32_t``, ``int16_t``) rather than implementation-defined types (e.g., ``int``, ``long``).
   * - RON-QR-004
     - The library **shall** compile without errors or warnings under at minimum two independent toolchains (e.g., GCC ARM and LLVM/Clang) with warnings set to maximum severity.

Maintainability
---------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-QR-010
     - Each function **shall** have a single, clearly defined responsibility (Single Responsibility Principle).
   * - RON-QR-011
     - Cyclomatic complexity of any single function **shall** not exceed **10**.
   * - RON-QR-012
     - All public API symbols **shall** be accompanied by documentation comments in a format compatible with Doxygen or a similar tool, covering: purpose, parameters, return values, preconditions, postconditions, and side effects.
   * - RON-QR-013
     - Magic numbers **shall** not appear in source code; all constants **shall** be named and documented.

Testability
-----------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-QR-020
     - The library **shall** be testable on a host PC (native target) without requiring target hardware, enabling CI/CD integration.
   * - RON-QR-021
     - All internal state **shall** be readable through getter functions to enable white-box unit testing.
   * - RON-QR-022
     - The library **shall** achieve **100% statement coverage** and **100% branch (decision) coverage** in its unit test suite.
   * - RON-QR-023
     - For safety-relevant conditional logic, **MC/DC (Modified Condition/Decision Coverage)** shall be achieved, as required by IEC 61508 SIL 3+ and DO-178C Level A.

Reliability
-----------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-QR-030
     - The library **shall** be free from data races when called from a single execution context (single task or ISR). Thread-safe multi-instance use across multiple concurrent contexts is the caller's responsibility via external locking.
   * - RON-QR-031
     - The library **shall** exhibit deterministic, reproducible behaviour given identical inputs and configuration — no random or time-dependent behaviour unless explicitly documented.

------------------------------------------------------------------------

Interface Requirements
======================

API Overview
------------

The library shall expose the following categories of functions. Concrete signatures are defined in the SADS.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Category
     - Description
   * - Lifecycle
     - ``init`` and ``reset`` — initialise a caller-allocated controller instance and reset its dynamic state.
   * - Configuration
     - ``set_gains``, ``set_limits``, ``set_filter``, ``set_antiwindup``, and ``config_validate`` — validate and update the PID configuration supported by ``ron_pid.h``.
   * - Runtime
     - ``step`` — execute one control computation cycle, given :math:`r(k)` and :math:`y(k)`, returning :math:`u_{sat}(k)` and a status word.
   * - State Access
     - ``get_state`` — read the observable integral, last output, last derivative, status word, and fault code in one call.
   * - State Mutation
     - ``set_integral`` and ``set_mode`` — preload internal state and switch between manual and automatic operation.
   * - Diagnostics
     - ``fault_clear`` — clear the latched PID fault register so normal operation can resume.

Hardware / Platform Interface
------------------------------

The library has no direct hardware interface. All platform dependencies are resolved through:

- **Floating-point type selection:** a compile-time macro selects ``float`` (32-bit) or ``double`` (64-bit).
- **Assertion mechanism:** a compile-time macro maps ``RON_ASSERT`` to the platform's assertion handler.
- **Fault handler callback:** an optional function pointer registered at initialisation to invoke a platform fault notification upon ``FAULT`` detection.

------------------------------------------------------------------------

Design Constraints
==================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Constraint
   * - RON-DC-001
     - The implementation language shall be selected during the architecture phase and documented in the SADS. Candidates are C (ISO C99/C11) and C++ (ISO C++14/C++17 with a bare-metal subset).
   * - RON-DC-002
     - The library **shall not** depend on the C standard library beyond ``<stdint.h>``, ``<stdbool.h>``, and ``<float.h>`` (or their C++ equivalents). Use of ``<math.h>`` functions is permitted only if they are bounded and WCET-analysable on the target.
   * - RON-DC-003
     - No use of ``setjmp``/``longjmp``, ``goto``, or exception handling (if C++).
   * - RON-DC-004
     - All source files shall be encodable in 7-bit ASCII to avoid character-encoding portability issues.
   * - RON-DC-005
     - The build system **shall** support out-of-tree builds and cross-compilation for at least one embedded target architecture and x86-64 (for host-based testing). The specific embedded target(s) shall be defined during integration.

------------------------------------------------------------------------

Verification Requirements
==========================

.. list-table::
   :header-rows: 1
   :widths: 12 60 28

   * - Req. ID
     - Verification Method
     - Acceptance Criterion
   * - RON-FR-001 – FR-007
     - Unit test: stimulus with known analytical solutions; numerical comparison.
     - Output error < 0.01% vs. reference.
   * - RON-FR-020 – FR-027
     - Unit test: sweep outputs beyond limits; verify clamping and rate.
     - All outputs within :math:`[u_{min}, u_{max}]`; all deltas ≤ rate limit × :math:`T_s`.
   * - RON-FR-030 – FR-035
     - Unit test: sustained large error forcing saturation; measure overshoot recovery.
     - Recovery time ≤ reference PID without AW; no runaway integral.
   * - RON-FR-040 – FR-042
     - Unit test: manual→automatic switchover; verify output continuity.
     - Output step at switchover < 1 LSB of actuator resolution.
   * - RON-SR-001 – SR-006
     - Code review + static analysis (``cppcheck --addon=misra.py`` for C;
       ``cargo clippy -D warnings`` for Rust).
     - Zero mandatory rule violations; all deviations documented.
   * - RON-SR-010 – SR-013
     - Unit test: inject NaN/Inf inputs; verify fault flag and safe output.
       Kani proof harness (Rust) / CBMC harness (C) for bounded verification.
     - ``FAULT`` flag set; safe-state output applied; fault persists until cleared.
   * - RON-PR-001 – PR-004
     - Measurement-based WCET profiling on integration target + OTAWA static
       analysis where supported.
     - No unbounded paths; WCET budget met per integration specification.
   * - RON-PR-020 – PR-022
     - Binary size measurement; heap profiler (no allocation).
     - Code ≤ 4 KB; RAM/instance ≤ 128 B; zero heap allocations.
   * - RON-QR-022 – QR-023
     - Coverage: ``gcov``/``lcov`` or ``llvm-cov`` (C); ``cargo-llvm-cov``
       (Rust). Both support MC/DC via ``--mcdc`` (LLVM 18+).
     - 100% statement + branch coverage; MC/DC where mandated.

------------------------------------------------------------------------

Signal Conditioning Filters
============================

These requirements define a family of **standalone, reusable discrete-time filter
components**. Filters are independent of the PID module but share the same platform
abstraction, instance model, and safety conventions. Each filter type is a
separately instantiable component; multiple instances of the same type may
coexist simultaneously.

General Filter Requirements
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-100
     - The library **shall** provide discrete-time signal conditioning filters as
       first-class, reusable components independent of any controller module.
   * - RON-FR-101
     - Each filter type **shall** follow the same instance/configuration/state
       model as the PID module: caller-allocated storage, no dynamic allocation,
       bounded O(1) execution per call, and a status/fault return value.
   * - RON-FR-102
     - Every filter **shall** expose: ``init``, ``reset``, ``step`` (apply one
       sample), and ``get_state`` operations with identical contract conventions
       to the PID API.
   * - RON-FR-103
     - Filter coefficients **shall** be validated at initialisation; any
       coefficient set that would produce an unstable or degenerate filter
       **shall** be rejected with ``FAULT_CONFIG_INVALID``.

First-Order Low-Pass Filter (IIR)
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-110
     - The library **shall** provide a configurable first-order IIR (exponential)
       low-pass filter:

       .. math::

          y(k) = \alpha \cdot x(k) + (1 - \alpha) \cdot y(k-1)

       where :math:`\alpha \in (0, 1]` is the smoothing factor.
   * - RON-FR-111
     - The filter **shall** accept configuration as either the smoothing factor
       :math:`\alpha` directly, or as a cut-off frequency :math:`f_c` and sample
       period :math:`T_s`, converting via:
       :math:`\alpha = 1 - e^{-2\pi f_c T_s}` (or the Euler approximation
       :math:`\alpha = 2\pi f_c T_s / (1 + 2\pi f_c T_s)` for fixed-point targets).

Moving Average Filter (FIR)
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-115
     - The library **shall** provide a causal moving-average (boxcar) FIR filter
       of configurable window length :math:`M`:

       .. math::

          y(k) = \frac{1}{M} \sum_{i=0}^{M-1} x(k-i)

   * - RON-FR-116
     - The window length :math:`M` **shall** be a compile-time constant or a
       value bounded by a compile-time maximum ``RON_MA_MAX_WINDOW``, so that
       the circular sample buffer is statically allocated with no heap use.
   * - RON-FR-117
     - The moving-average filter **shall** use an efficient recursive
       (sliding-sum) implementation: :math:`y(k) = y(k-1) + (x(k) - x(k-M))/M`,
       avoiding a full-window summation on every step.

Biquad IIR Filter
-----------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-120
     - The library **shall** provide a second-order IIR (biquad) filter using the
       Direct Form II Transposed structure:

       .. math::

          y(k) = b_0 x(k) + b_1 x(k-1) + b_2 x(k-2)
                           - a_1 y(k-1) - a_2 y(k-2)

       with coefficients :math:`(b_0, b_1, b_2, a_1, a_2)` supplied directly by
       the caller.
   * - RON-FR-121
     - The biquad filter **shall** support cascaded sections: a chain of up to
       ``RON_BIQUAD_MAX_SECTIONS`` biquad stages applied in series, enabling
       higher-order Butterworth, Chebyshev, or notch filters to be expressed as
       second-order-section (SOS) arrays.
   * - RON-FR-122
     - The library **shall** provide coefficient-generation helper operations for
       the following common biquad configurations, given cut-off frequency, sample
       rate, and quality factor :math:`Q`:

       a. **Low-pass** (Butterworth, 2nd order).
       b. **High-pass** (Butterworth, 2nd order).
       c. **Band-pass**.
       d. **Notch (band-reject)** — for structural resonance suppression.

   * - RON-FR-123
     - For the notch configuration, the centre frequency :math:`f_0` and
       bandwidth :math:`\Delta f` (or equivalently, :math:`Q = f_0 / \Delta f`)
       **shall** be configurable at initialisation and updatable at runtime
       without resetting the filter state, to support adaptive resonance
       cancellation.

Standalone Rate-of-Change Limiter
-----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-130
     - The library **shall** provide a standalone rate-of-change (slew-rate)
       limiter as an independent component, usable on any signal (setpoint,
       actuator command, filter output):

       .. math::

          y(k) = \text{clamp}\bigl(x(k),\ y(k-1) - \dot{r}_{max} T_s,\
                                              y(k-1) + \dot{r}_{max} T_s\bigr)

   * - RON-FR-131
     - The rate limiter **shall** support independent positive and negative rate
       limits (:math:`\dot{r}_{rise}` and :math:`\dot{r}_{fall}`) to handle
       asymmetric actuator dynamics.

------------------------------------------------------------------------

Feed-Forward Control
=====================

These requirements cover the addition of a configurable feed-forward (FF) term
to complement closed-loop feedback. Feed-forward operates on the setpoint or a
known disturbance signal and does not depend on the error.

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-200
     - The PID module **shall** support an additive feed-forward path that
       contributes to the controller output independently of the P, I, and D
       terms:

       .. math::

          u(k) = u_{PID}(k) + u_{FF}(k)

   * - RON-FR-201
     - The library **shall** support the following feed-forward modes, selectable
       per instance at configuration time:

       a. **Static gain FF:** :math:`u_{FF}(k) = K_{FF} \cdot r(k)`.
       b. **Velocity FF:** :math:`u_{FF}(k) = K_{FF,v} \cdot \dot{r}(k)`.
       c. **Acceleration FF:** :math:`u_{FF}(k) = K_{FF,a} \cdot \ddot{r}(k)`.
       d. **External FF:** the caller supplies :math:`u_{FF}(k)` directly as an
          additional input to ``pid_step()``.

   * - RON-FR-202
     - Feed-forward derivatives (:math:`\dot{r}`, :math:`\ddot{r}`) **shall** be
       computed by the library using finite differences with a configurable LP
       filter coefficient independent of the feedback derivative filter.
   * - RON-FR-203
     - The feed-forward output **shall** be subject to the same output saturation
       and rate limits as the full controller output (applied after summation).
   * - RON-FR-204
     - Feed-forward **shall** be disabled by default (all FF gains = 0).  When
       disabled, the computational overhead of the FF path **shall** be zero.
   * - RON-FR-205
     - The feed-forward term **shall** be reported in the status/diagnostic
       output so the caller can monitor its contribution independently.

------------------------------------------------------------------------

Gain Scheduling
================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-300
     - The library **shall** provide a gain scheduling mechanism that associates
       a set of operating points (scheduling variable breakpoints) with a
       corresponding ``ron_pid_config_t`` record per breakpoint.
   * - RON-FR-301
     - The number of breakpoints **shall** be bounded by a compile-time constant
       ``RON_GS_MAX_BREAKPOINTS`` and stored in a statically allocated table.
   * - RON-FR-302
     - The library **shall** support two scheduling transition modes:

       a. **Hard switching** — the parameter set at the nearest breakpoint below
          the scheduling variable is applied immediately.
       b. **Linear interpolation** — gains are linearly interpolated between the
          two bracketing breakpoints.

   * - RON-FR-303
     - The gain scheduling module **shall** apply gain updates atomically via the
       runtime update API, ensuring no partial-update inconsistency.
   * - RON-FR-304
     - The scheduling variable **shall** be passed to a dedicated
       ``ron_gs_update()`` operation, decoupled from ``ron_pid_step()``.
   * - RON-FR-305
     - Parameter switching under hard-switching mode **shall** optionally trigger
       an integral reset to avoid windup artefacts from the previous operating
       point.
   * - RON-FR-306
     - The breakpoint table **shall** be validated at initialisation: breakpoints
       must be strictly monotonically increasing; all associated configs must
       pass ``ron_pid_config_validate()``.

------------------------------------------------------------------------

Cascade Controller
==================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-400
     - The library **shall** provide a cascade controller structure that
       encapsulates two ``ron_pid_instance_t`` objects (outer and inner) and
       manages their interconnection.
   * - RON-FR-401
     - The cascade ``step`` operation **shall** accept the outer setpoint
       :math:`r_{out}`, outer PV :math:`y_{out}`, inner PV :math:`y_{in}`, and
       sample period ``dt``; it **shall** compute the inner setpoint as the
       clamped output of the outer controller automatically.
   * - RON-FR-402
     - The cascade module **shall** enforce an inner-loop output limit range that
       is consistent with the outer-loop output range.
   * - RON-FR-403
     - The cascade module **shall** implement **back-calculation anti-windup
       propagation**: when the inner loop saturates, a correction signal
       **shall** be fed back into the outer loop's integrator.
   * - RON-FR-404
     - Mode transitions (automatic ↔ manual) **shall** be coordinated across
       both loops with bumpless transfer applied in the correct order.
   * - RON-FR-405
     - Both inner and outer controllers **shall** independently support all
       single-PID features with no restrictions.
   * - RON-FR-406
     - The cascade controller **shall** expose a unified status word ORing both
       loops' status bits, with distinct bit ranges assigned per loop.

------------------------------------------------------------------------

Trajectory Generators
======================

Trapezoidal Velocity Profile
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-500
     - The library **shall** provide a trapezoidal velocity profile generator
       with bounded velocity :math:`v_{max}` and acceleration :math:`a_{max}`.
   * - RON-FR-501
     - The generator **shall** handle the short-move case where the target is
       reached before full velocity is attained.
   * - RON-FR-502
     - The generator **shall** expose reference position :math:`r(k)`, velocity
       :math:`\dot{r}(k)`, and acceleration :math:`\ddot{r}(k)` per step for
       direct use as feed-forward inputs.
   * - RON-FR-503
     - The generator **shall** support online goal updates mid-profile with
       smooth velocity-continuous transitions.

S-Curve (Jerk-Limited) Profile
--------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-510
     - The library **shall** provide a jerk-limited (S-curve) trajectory
       generator extending the trapezoidal profile with bounded jerk
       :math:`j_{max}` and a seven-phase profile.
   * - RON-FR-511
     - The S-curve generator **shall** expose position, velocity, acceleration,
       and jerk at each step.
   * - RON-FR-512
     - Both generators **shall** provide a ``finished`` flag indicating the
       target has been reached.
   * - RON-FR-513
     - Both generators **shall** support a ``hold`` mode that freezes output
       without resetting kinematic state.

------------------------------------------------------------------------

Kalman Filter (Discrete Linear)
=================================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-600
     - The library **shall** provide a discrete-time linear Kalman filter for
       systems :math:`x(k+1) = Ax(k) + Bu(k) + w(k)`,
       :math:`z(k) = Hx(k) + v(k)`.
   * - RON-FR-601
     - The filter **shall** be parameterised by matrices :math:`A`, :math:`B`,
       :math:`H`, :math:`Q`, :math:`R`, initial state :math:`\hat{x}_0` and
       covariance :math:`P_0`, with dimensions bounded by compile-time constants.
   * - RON-FR-602
     - The filter **shall** implement the standard predict–update two-step
       algorithm.
   * - RON-FR-603
     - Matrix inversion for the Kalman gain **shall** use scalar division for
       :math:`m=1` and a Cholesky-based solver for :math:`m > 1`.
   * - RON-FR-604
     - The Joseph form covariance update **shall** be available as a
       configuration option for enhanced numerical stability.
   * - RON-FR-605
     - The filter **shall** support measurement dropout by skipping the update
       step when flagged.
   * - RON-FR-606
     - A steady-state (fixed-gain) mode **shall** be available for
       reduced per-step computation.
   * - RON-FR-607
     - All matrix dimensions are fixed at compile time; no dynamic allocation
       is used.

------------------------------------------------------------------------

State-Space Controller and Luenberger Observer
===============================================

State-Space Controller
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-700
     - The library **shall** provide a discrete-time state-feedback controller
       :math:`u(k) = -K\hat{x}(k) + K_r r(k)`.
   * - RON-FR-701
     - The controller **shall** accept state estimates from an external vector,
       an internal Luenberger observer, or an internal Kalman filter.
   * - RON-FR-702
     - Integral action for output regulation **shall** be supported via
       augmented state.
   * - RON-FR-703
     - Output saturation, rate limiting, and fault detection **shall** apply
       as defined for the PID module.
   * - RON-FR-704
     - The gain matrix :math:`K` and pre-gain :math:`K_r` **shall** be
       updatable at runtime.

Luenberger Observer
--------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-720
     - The library **shall** provide a discrete-time Luenberger observer:
       :math:`\hat{x}(k+1) = A\hat{x}(k) + Bu(k) + L(y(k) - C\hat{x}(k))`.
   * - RON-FR-721
     - The observer **shall** be parameterised by caller-supplied :math:`A`,
       :math:`B`, :math:`C`, :math:`L`.
   * - RON-FR-722
     - The full state estimate :math:`\hat{x}(k)` **shall** be accessible via
       getter.
   * - RON-FR-723
     - Matrix dimensions **shall** be bounded by compile-time constants.

------------------------------------------------------------------------

Auto-Tuning (Relay Feedback)
==============================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-800
     - The library **shall** provide a relay feedback auto-tuning module that
       temporarily replaces the PID output with a relay signal to excite
       sustained plant oscillation.
   * - RON-FR-801
     - The relay **shall** be configurable: amplitude :math:`d`, hysteresis
       band :math:`\epsilon`, minimum oscillation cycles, and timeout duration.
   * - RON-FR-802
     - The module **shall** estimate ultimate gain
       :math:`K_u = 4d / (\pi A)` and ultimate period :math:`T_u` from the
       measured oscillation.
   * - RON-FR-803
     - The module **shall** support Ziegler–Nichols, Tyreus–Luyben,
       some-overshoot, and no-overshoot tuning rules, selectable at
       configuration time.
   * - RON-FR-804
     - Computed gains **shall** be applied to the target PID instance only
       on explicit ``ron_autotune_apply()`` call.
   * - RON-FR-805
     - Raw :math:`K_u` and :math:`T_u` **shall** be returned for custom
       tuning rule application.
   * - RON-FR-806
     - During tuning, the output remains within
       :math:`[u_{bias} - d, u_{bias} + d]` while PID output limits still apply.
   * - RON-FR-807
     - Auto-tuning **shall** be aborted and output restored on PID fault or
       explicit ``ron_autotune_abort()`` call.

------------------------------------------------------------------------

Control Loop Health Monitor
============================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-900
     - The library **shall** provide a control loop health monitor that attaches
       to any controller instance and evaluates loop health each step.
   * - RON-FR-901
     - The health monitor **shall** detect and report via a bitmask:

       a. ``HEALTH_OUTPUT_STUCK`` — output saturated beyond threshold duration.
       b. ``HEALTH_DIVERGING`` — error magnitude growing continuously.
       c. ``HEALTH_OSCILLATING`` — error sign changes exceed threshold in window.
       d. ``HEALTH_SENSOR_DROPOUT`` — PV has not changed within dead-band for
          threshold duration.
       e. ``HEALTH_SP_UNREACHABLE`` — steady-state error persists beyond settling
          time.

   * - RON-FR-902
     - Each condition **shall** have independently configurable thresholds and
       time constants.
   * - RON-FR-903
     - The health monitor **shall** be passive: it **shall not** modify the
       controller output, state, or configuration.
   * - RON-FR-904
     - A callback ``ron_health_cb_t`` **shall** be invoked when a health
       condition first becomes active.
   * - RON-FR-905
     - Health monitor status **shall** latch until explicitly cleared by the
       caller.

------------------------------------------------------------------------

Runtime Performance Metrics
=============================

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - ID
     - Requirement
   * - RON-FR-950
     - The library **shall** provide a performance metrics accumulator
       attachable to any controller instance.
   * - RON-FR-951
     - The following metrics **shall** be computed:

       a. **IAE**: :math:`\sum |e(k)| T_s`
       b. **ISE**: :math:`\sum e(k)^2 T_s`
       c. **ITAE**: :math:`\sum k T_s |e(k)| T_s`
       d. **Peak overshoot** (%) relative to step size.
       e. **Settling time** to within a configurable band.
       f. **Rise time** from 10% to 90% of step.

   * - RON-FR-952
     - The accumulator **shall** support windowed (rolling) and cumulative
       modes.
   * - RON-FR-953
     - Metrics collection **shall** be enable/disable at runtime with zero
       overhead when disabled.
   * - RON-FR-954
     - The module **shall** detect a setpoint step and automatically restart
       transient metrics from that event.

------------------------------------------------------------------------

Appendix A: PID Forms Reference
================================

For clarity, the three PID forms referenced in this document are summarised here.

**Parallel (non-interacting) form:**

.. math::

   u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}

**Ideal (ISA / standard) form:**

.. math::

   u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int_0^t e(\tau) d\tau + T_d \frac{de(t)}{dt} \right]

**Two-degree-of-freedom (2DOF) form:**

.. math::

   u(t) = K_p \left[ b \cdot r(t) - y(t) \right]
        + K_i \int_0^t e(\tau) d\tau
        + K_d \frac{d}{dt}\left[ c \cdot r(t) - y(t) \right]

Appendix B: Anti-Windup Reference
====================================

**Back-calculation (tracking)** — recommended for smooth recovery:

.. math::

   \dot{I}(t) = K_i \cdot e(t) + \frac{1}{T_{aw}} \cdot \left[ u_{sat}(t) - u(t) \right]

**Conditional integration** — simpler, zero recovery parameter:

Integrator update suppressed when: :math:`\text{sat}(u) \neq 0` AND :math:`\text{sign}(e) = \text{sign}(I)`

Appendix C: Traceability Matrix (Summary)
==========================================

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Feature Area
     - Requirement IDs
   * - Core PID algorithm
     - RON-FR-001 – FR-007
   * - Normalisation
     - RON-FR-010 – FR-013
   * - Output saturation
     - RON-FR-020 – FR-022
   * - Rate limiting
     - RON-FR-025 – FR-027
   * - Anti-windup
     - RON-FR-030 – FR-035
   * - Bumpless transfer
     - RON-FR-040 – FR-042
   * - PID state management
     - RON-FR-050 – FR-054
   * - Multi-instance
     - RON-FR-060 – FR-062
   * - Status / diagnostics
     - RON-FR-070 – FR-071
   * - Signal conditioning filters
     - RON-FR-100 – FR-131
   * - Feed-forward control
     - RON-FR-200 – FR-205
   * - Gain scheduling
     - RON-FR-300 – FR-306
   * - Cascade controller
     - RON-FR-400 – FR-406
   * - Trajectory generators
     - RON-FR-500 – FR-513
   * - Kalman filter
     - RON-FR-600 – FR-607
   * - State-space controller
     - RON-FR-700 – FR-704
   * - Luenberger observer
     - RON-FR-720 – FR-723
   * - Auto-tuning (relay feedback)
     - RON-FR-800 – FR-807
   * - Control loop health monitor
     - RON-FR-900 – FR-905
   * - Runtime performance metrics
     - RON-FR-950 – FR-954
   * - Timing / WCET
     - RON-PR-001 – PR-004
   * - Numerical precision
     - RON-PR-010 – PR-012
   * - Memory
     - RON-PR-020 – PR-022
   * - Defensive programming
     - RON-SR-001 – SR-006
   * - Fault detection
     - RON-SR-010 – SR-013
   * - Numerical integrity
     - RON-SR-020 – SR-022
   * - Standards alignment
     - RON-SR-030 – SR-033
   * - Portability
     - RON-QR-001 – QR-004
   * - Maintainability
     - RON-QR-010 – QR-013
   * - Testability
     - RON-QR-020 – QR-023
   * - Reliability
     - RON-QR-030 – QR-031

------------------------------------------------------------------------

*End of Document — RON-SRS-001 v1.1.0*

.. ============================================================
.. MISRA C:2023 Deviation Records — Regulon C11 Library
.. ============================================================

MISRA C:2023 Deviation Records
================================

**Document ID:** RON-DEV-C-001

**Version:** 0.3.0 (PID verification closure)

**Status:** Draft

**Date:** 2026-04-11

Introduction
------------

This document records all approved deviations from MISRA C:2023 guidelines
in the Regulon C11 library (``regulon-c/``).

Each record contains:

- The MISRA C:2023 rule number and title
- The file(s) and line range(s) where the deviation applies
- Justification
- Compensating measures
- Reviewer sign-off

A deviation is required for every violation of a **Required** rule.
**Mandatory** rules are never deviated.  **Advisory** guidelines that cannot
be satisfied are noted as observations but do not require a formal record.

Deviation Table
---------------

.. list-table::
   :header-rows: 1
   :widths: 8 30 22 20 20

   * - ID
     - MISRA C:2023 Rule
     - Location
     - Justification
     - Compensating Measures
   * - DEV-001
     - Rule 15.5 (Single point of exit)
     - ``regulon-c/src/ron_pid_api.c``, ``regulon-c/src/ron_pid_core.c``,
       ``regulon-c/src/ron_pid_fault.c``
     - Guard-clause exits are retained in the active PID slice to enforce the
       required defensive call order (null-check → init-check → fault latch →
       validation → computation) and to avoid deeper nesting in the safety
       paths.
     - Unity safety tests ``RON-TC-SAFE-001``, ``SAFE-006``, ``SAFE-007``,
       ``SAFE-008``, ``SAFE-009``, ``SAFE-010``, and ``SAFE-011`` exercise
       the guarded exits. Complexity is constrained by ``lizard -C 10``.
   * - DEV-002
     - Rule 20.10 (``#`` / ``##`` operators in function-like macros)
     - ``regulon-c/include/ron/ron_platform.h`` — ``RON_FLOAT_C``,
       ``RON_STATIC_ASSERT``
     - Token-pasting is required to preserve precision-independent literal
       construction and a C99/C11-compatible static-assert fallback in the
       platform layer.
     - Usage is limited to the platform header, reviewed manually, and covered
       by the PID type and precision tests.
   * - DEV-003
     - Rule 8.7 (Objects/functions with external linkage should be declared in
       one file only)
     - ``regulon-c/src/ron_pid_api.c``
     - ``cppcheck`` reports the public PID API as a Rule 8.7 issue when the
       library sources are analysed without their consumer translation units.
       These functions are intentionally public and declared in
       ``regulon-c/include/ron/ron_pid.h``.
     - The exported API is link-verified by the Unity test executables and
       reviewed against ``RON-IS-001``.

Pending Review
--------------

*None at the current PID verification-closure baseline.*

Advisory Guidelines — Non-Conformances
-----------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 8 30 22 40

   * - ID
     - MISRA C:2023 Guideline
     - Location
     - Observation
   * - OBS-001
     - Dir 4.9 (Advisory): A function should be used in preference to a
       function-like macro where they are interchangeable.
     - ``include/ron/ron_platform.h`` — ``RON_ISNAN``, ``RON_ISINF``,
       ``RON_ISFINITE``
     - These three macros cannot be replaced by ``static inline`` functions
       because they must work for both ``float`` and ``double`` operands
       without casting.  The implementation is simple (single comparison)
       and well-reviewed.
   * - OBS-002
     - Rule 2.5 (Advisory): Unused macro declarations
     - ``include/ron/ron_platform.h`` — ``RON_NORETURN``, ``RON_INLINE``,
       ``RON_VERSION_*``
     - These macros are part of the public platform contract and are retained
       for future C modules even though the active PID-only slice does not yet
       consume every one of them.

Revision History
-----------------

.. list-table::
   :header-rows: 1
   :widths: 10 10 50 30

   * - Version
     - Date
     - Description
     - Author
   * - 0.1.0
     - 2026-04-11
     - Sprint 0 baseline (no deviations).
     - TBD
   * - 0.2.0
     - 2026-04-12
     - PID kickoff updated; still no approved deviations.
     - TBD
   * - 0.3.0
     - 2026-04-12
     - Added approved PID verification-closure deviations for Rules 15.5,
       20.10, and 8.7; recorded the active-slice Rule 2.5 observation.
     - TBD

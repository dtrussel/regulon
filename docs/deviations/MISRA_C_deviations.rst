.. ============================================================
.. MISRA C:2023 Deviation Records — Regulon C11 Library
.. ============================================================

MISRA C:2023 Deviation Records
================================

**Document ID:** RON-DEV-C-001

**Version:** 0.4.0 (full-library audit — Phases 0-12)

**Status:** Draft

**Date:** 2026-08-07

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
     - Every active production source in ``regulon-c/src/`` (all PID, filter,
       feed-forward, gain-scheduling, trajectory, cascade, Kalman, state-space,
       observer, LQR, LQG, auto-tune, health, and metrics sources — verified
       by ``cppcheck --addon=misra.py`` against the full
       ``scripts/lib_sources.txt`` manifest).
     - Guard-clause exits are used throughout the library to enforce the
       mandated defensive call order (null-check → init-check → fault latch →
       validation → computation, ``AGENTS.md``/``regulon-c/AGENTS.md``) and to
       avoid deeper nesting in safety-relevant paths. This is a deliberate,
       library-wide coding convention, not a per-module exception.
     - Unity safety tests (``RON-TC-SAFE-*`` and the defensive/null/
       uninitialised-path cases in every module's own unit suite) exercise the
       guarded exits. Complexity is constrained by ``lizard -C 10`` for every
       function, so guard-clause nesting cannot substitute for real branch
       reduction.
   * - DEV-002
     - Rule 20.10 (``#`` / ``##`` operators in function-like macros)
     - ``regulon-c/include/ron/ron_platform.h`` — ``RON_FLOAT_C``,
       ``RON_STATIC_ASSERT``
     - Token-pasting is required to preserve precision-independent literal
       construction and a C99/C11-compatible static-assert fallback in the
       platform layer.
     - Usage is limited to the platform header, reviewed manually, and covered
       by the PID type and precision tests. Re-verified against the full
       source manifest: no other header or source introduces ``#``/``##``.
   * - DEV-003
     - Rule 8.7 (Objects/functions with external linkage should be declared in
       one file only)
     - Public API functions across the active module set; concretely surfaced
       by the current toolchain on ``regulon-c/src/ron_filter.c`` and
       ``regulon-c/src/ron_matrix.c`` (the exact file set ``cppcheck`` flags
       is version-/addon-dependent, since it is a whole-program-unaware,
       single-translation-unit analysis).
     - ``cppcheck`` reports every module's public API as a Rule 8.7 issue when
       the library sources are analysed without their consumer translation
       units (the Unity test executables and, for ``ron_matrix``, every module
       that shares the internal fixed-size matrix helper). These functions and
       the internal-but-cross-TU ``ron_matrix``/``ron_lqr_dare_solve`` helpers
       are intentionally declared once in a header (public or private) and
       used from other translation units by design.
     - The exported and cross-TU-internal APIs are link-verified by the Unity
       test executables and reviewed against ``RON-IS-001``.
   * - DEV-004
     - Rule 15.7 (Missing ``else`` after the final ``else if``)
     - ``regulon-c/src/ron_feedforward.c``, ``regulon-c/src/ron_pid_api.c``,
       ``regulon-c/src/ron_pid_config.c``, ``regulon-c/src/ron_pid_core.c``,
       ``regulon-c/src/ron_trajectory_scurve.c``,
       ``regulon-c/src/ron_trajectory_trap.c``.
     - Enumerated-mode ``if``/``else if`` chains (e.g. anti-windup mode,
       feed-forward mode, trajectory phase) are exhaustively validated by the
       config-validation layer before the chain is reached, so a trailing
       ``else`` would be dead defensive code the coverage gate could never
       exercise; this was already an active CI suppression
       (``--suppress=misra-c2012-15.7``) that this revision formally records.
     - Enum values are validated by each module's own config-validation
       function (rejecting any value outside the enumerated range with
       ``RON_FAULT_CONFIG_INVALID``) before the ``if``/``else if`` chain runs,
       and every branch is covered by that module's own Unity suite.

Pending Review
--------------

*None at the current full-library audit baseline (Phases 0-12 complete).*

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
     - These macros are part of the public platform contract for API
       consumers (fault-handler declarations, performance-critical inline
       hints, and library version reporting). Re-verified against the full
       Phase 0-12 active source set: no library source consumes them
       internally, which is expected — they exist for callers, not for the
       library's own translation units.

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
   * - 0.4.0
     - 2026-08-07
     - Full-library audit after Phase 12 (LQR/LQG) closed the C11 roadmap.
       Re-ran ``cppcheck --addon=misra.py`` over the complete active source
       manifest (``scripts/lib_sources.txt``): widened DEV-001 (Rule 15.5)
       from the four original PID files to the whole active source set,
       since the guard-clause convention is library-wide by design;
       reconciled DEV-003 (Rule 8.7) with the file set the current
       toolchain actually flags; added DEV-004 (Rule 15.7, missing final
       ``else``) to formally record a deviation that was already an active
       CI suppression but had never been documented; updated OBS-002's
       wording now that the "active slice" is the full library, not
       PID-only.
     - TBD

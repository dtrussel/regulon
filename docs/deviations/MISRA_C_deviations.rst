.. ============================================================
.. MISRA C:2023 Deviation Records — Regulon C11 Library
.. ============================================================

MISRA C:2023 Deviation Records
================================

**Document ID:** RON-DEV-C-001

**Version:** 0.1.0 (Sprint 0 Baseline)

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

*No approved deviations at Sprint 0 baseline.*

.. list-table::
   :header-rows: 1
   :widths: 8 30 22 20 20

   * - ID
     - MISRA C:2023 Rule
     - Location
     - Justification
     - Compensating Measures
   * - (none)
     - —
     - —
     - —
     - —

Pending Review
--------------

*None at Sprint 0 baseline.*

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

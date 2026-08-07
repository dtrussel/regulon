Specifications
==============

Regulon is written specification-first. Each document below is the ground
truth for a different question, and they are chained: a requirement in the
SRS is realised by a design in the SADS, exposed through an interface in the
IS, and demonstrated by a case in the TP.

Nothing is implemented without a requirement ID, and no requirement is
considered met without a test ID covering it. The
:doc:`API reference <../api/index>` links back into these documents from
every declaration.

.. list-table::
   :header-rows: 1
   :widths: 14 22 64

   * - ID
     - Document
     - Answers
   * - ``RON-SRS-001``
     - :doc:`SRS_ControlLib`
     - What the library must do. Functional, safety, performance and
       interface requirements (``RON-FR-*``, ``RON-SR-*``, ``RON-PR-*``).
   * - ``RON-SADS-001``
     - :doc:`SADS_ControlLib`
     - How it is built. Architecture, module decomposition, algorithm
       pseudocode, and the numbered design decisions (``DD-*``) behind them.
   * - ``RON-IS-001``
     - :doc:`IS_ControlLib`
     - What the interface looks like. Header layout, type definitions,
       function signatures, build system and toolchain integration.
   * - ``RON-TP-001``
     - :doc:`TP_ControlLib`
     - How it is proven. Test strategy, per-case specifications
       (``RON-TC-*``), coverage targets and verification gates.

.. toctree::
   :maxdepth: 2
   :hidden:

   SRS_ControlLib
   SADS_ControlLib
   IS_ControlLib
   TP_ControlLib

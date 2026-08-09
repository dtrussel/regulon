Verification and Compliance
===========================

This page explains what Regulon's correctness claims actually rest on, so
you can judge whether that basis is sufficient for your application rather
than taking a summary on trust.

Everything described here runs on every push and pull request. None of it is
a periodic audit or a release-time activity.

The chain of evidence
---------------------

Regulon is specification-driven, which in practice means one rule: **no code
without a requirement, no requirement without a test.**

.. code-block:: text

   SRS  requirement  ──▶  SADS  design  ──▶  IS  interface  ──▶  code
    │   RON-FR-730         DD-19               ron_lqr.h        ron_lqr.c
    │                                                              │
    └──────────────────  TP  test case  ◀─────────────────────────┘
                         RON-TC-LQR-001

Each declaration in the headers carries the requirements it satisfies and the
tests covering it, and the :doc:`../api/index` renders those as links into
the :doc:`specifications <../specs/index>`. The traceability is therefore
checkable by reading, not just asserted.

What each gate proves
---------------------

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Gate
     - What it establishes
   * - Unit and integration tests
     - Behaviour matches the cases specified in the test plan. Run under
       GCC and Clang, in both single and double precision.
   * - AddressSanitizer / UBSan
     - No out-of-bounds access, use-after-scope, or undefined behaviour on
       any path the tests reach.
   * - Coverage
     - 100% of statements **and branches** are exercised. Branch coverage
       is the meaningful figure: it is what forces every guard clause and
       error path to be tested, not merely compiled.
   * - CBMC formal proofs
     - Selected properties hold for *all* inputs in the bounded model, not
       only tested ones — chiefly that outputs stay within configured
       limits and that no path allocates.
   * - cppcheck / MISRA C:2023
     - Conformance to the coding standard, with every deviation recorded.
   * - Complexity (``lizard -C 10``)
     - No function exceeds cyclomatic complexity 10, keeping every function
       small enough to review and to cover exhaustively.
   * - Cross-compile builds
     - The library is clean for ARM Cortex-M and RISC-V, not only the host.
   * - Package install smoke test
     - The exported CMake package resolves and links from a separate
       consumer project.
   * - Manifest drift check
     - Source files are registered in the build and format manifests, so a
       new file cannot silently escape the other gates.

Structural guarantees
---------------------

Some properties are guaranteed by construction rather than by testing, which
is what makes the library usable from an interrupt handler:

* **No dynamic allocation.** No ``malloc``/``free`` anywhere. All state is
  caller-owned and fixed-size, so there is no heap to exhaust or fragment,
  and memory use is known at link time.
* **No recursion and no unbounded loops.** Stack depth is bounded, and every
  iteration count is bounded by a compile-time constant. Iterative
  algorithms — notably the DARE solver — take an explicit iteration cap and
  report non-convergence as a fault rather than spinning.
* **No global mutable state.** Instances are independent, so multiple loops
  do not interact through the library.
* **No VLAs, no ``goto``, no ``setjmp``.**

Reproducing the gates locally
-----------------------------

.. code-block:: bash

   # Tests, sanitizers on
   cmake -B build -S regulon-c -DRON_BUILD_TESTS=ON
   cmake --build build && ctest --test-dir build --output-on-failure

   # Formatting and manifest consistency
   bash regulon-c/scripts/check_format.sh
   bash regulon-c/scripts/check_manifest.sh

   # Static analysis and complexity
   bash regulon-c/scripts/run_cppcheck.sh
   lizard -C 10 regulon-c/src regulon-c/include

The CBMC proofs and coverage run take longer; see
``.github/workflows/ci_c.yml`` for the exact invocations CI uses.

Limits of these claims
----------------------

Being explicit about what is *not* covered:

* The formal proofs are **bounded**. They establish properties within a
  fixed unwinding depth and for bounded dimensions, not for arbitrary
  configurations.
* Coverage is measured on the **host**. The cross-compiled builds are
  compile-and-link checks; they do not execute the suite on target
  hardware.
* Timing figures from the benchmark are **host** measurements against the
  performance budgets in the SRS. They indicate the shape of the cost, not
  what your MCU will do.
* Regulon has **not** been certified against IEC 61508, ISO 26262, or
  DO-178C. It is built to be a plausible starting point for such an effort
  — the traceability and deviation records exist for that reason — but the
  qualification work itself is the integrator's.

MISRA deviations
----------------

Where the library does not follow a MISRA C:2023 guideline, the deviation is
recorded with its rationale and scope. See :doc:`../deviations/index`.

.. Landing page for the Regulon documentation site.

Regulon
=======

A specification-driven control systems library for embedded targets.

Regulon implements the classical embedded control toolkit — PID, filtering,
feed-forward, gain scheduling, trajectory generation, cascade control, state
estimation, and optimal control — as a C11 static library with **no dynamic
allocation, no recursion, and no unbounded loops**, suitable for
safety-critical and hard-real-time use.

Every function traces back to a numbered requirement, and every requirement
traces forward to a test. The specifications those IDs point at are published
here alongside the API reference, and the two are cross-linked.

.. grid:: 2
   :gutter: 3

   .. grid-item-card:: Get started
      :link: guides/quickstart
      :link-type: doc

      Build the library and run a first control loop in a few minutes.

   .. grid-item-card:: API reference
      :link: api/index
      :link-type: doc

      Every public type and function across the 14 modules, generated from
      the headers.

   .. grid-item-card:: Integrate it
      :link: guides/installation
      :link-type: doc

      ``find_package``, ``pkg-config``, ``add_subdirectory``, and
      cross-compiling to ARM and RISC-V.

   .. grid-item-card:: Specifications
      :link: specs/index
      :link-type: doc

      Requirements, architecture, interfaces, and the test plan.

   .. grid-item-card:: Zephyr RTOS
      :link: guides/zephyr
      :link-type: doc

      Use Regulon as a Zephyr module: west, Kconfig, and a runnable
      control-loop sample.


At a glance
-----------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Property
     - Value
   * - Language
     - C11, freestanding-friendly
   * - Allocation
     - None. All state is caller-owned and fixed-size.
   * - Precision
     - ``float`` by default, ``double`` via ``RON_USE_DOUBLE``
   * - Coding standard
     - MISRA C:2023, with :doc:`recorded deviations <deviations/index>`
   * - Verification
     - 100% statement and branch coverage, CBMC proofs on safety properties
   * - Targets
     - Host (x86-64), ARM Cortex-M, RISC-V ``rv32imc``
   * - Licence
     - MIT


Modules
-------

Each module is independently selectable at configure time via
``RON_ENABLE_<MODULE>``; only the PID core and feed-forward path are
mandatory. See :doc:`guides/module-selection`.

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Module
     - Purpose
   * - :doc:`api/pid`
     - Parallel and ISA-form PID with anti-windup, derivative filtering,
       bumpless transfer, and fault latching.
   * - :doc:`api/feedforward`
     - Static, velocity, and acceleration feed-forward terms.
   * - :doc:`api/filter`
     - First-order low/high-pass, biquad, moving-average, and median filters.
   * - :doc:`api/gain_sched`
     - Operating-point interpolation of controller gains.
   * - :doc:`api/trajectory`
     - Trapezoidal and S-curve motion profile generation.
   * - :doc:`api/cascade`
     - Nested inner/outer loop control with rate separation.
   * - :doc:`api/kalman`
     - Discrete-time linear Kalman filter.
   * - :doc:`api/statespace`
     - State-space controller with reference tracking.
   * - :doc:`api/observer`
     - Luenberger state observer.
   * - :doc:`api/lqr`
     - MIMO linear quadratic regulator with a DARE solver.
   * - :doc:`api/lqg`
     - Linear quadratic Gaussian control (LQR plus Kalman estimator).
   * - :doc:`api/autotune`
     - Relay-feedback automatic PID tuning.
   * - :doc:`api/health`
     - Loop health monitoring and diagnostics.
   * - :doc:`api/metrics`
     - Runtime performance metrics collection.


.. toctree::
   :hidden:
   :caption: Guides

   guides/quickstart
   guides/installation
   guides/module-selection
   guides/zephyr
   guides/cross-compiling
   guides/verification

.. toctree::
   :hidden:
   :caption: Reference

   api/index
   changelog

.. toctree::
   :hidden:
   :caption: Specifications

   specs/index
   deviations/index

.. toctree::
   :hidden:
   :caption: Project

   contributing
   security

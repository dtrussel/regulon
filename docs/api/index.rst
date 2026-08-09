API Reference
=============

Generated from the public headers in ``regulon-c/include/ron/``.

Every declaration carries the requirements it satisfies and the tests that
cover it. Those IDs are links: follow one to reach the exact requirement in
the :doc:`specifications <../specs/index>` or the test case in the
:doc:`test plan <../specs/TP_ControlLib>`.

Conventions
-----------

The API is uniform across modules, so learning one is close to learning all
of them.

* **Instances are caller-owned.** You declare a ``ron_<module>_t``, typically
  as a static object, and pass its address in. The library never allocates.
* **Every module follows init / step / reset.** ``ron_<module>_init``
  validates a configuration struct and populates the instance;
  ``ron_<module>_step`` advances it by one sample; ``ron_<module>_reset``
  returns it to its post-init state.
* **Errors are returned, never signalled.** Functions return
  :c:enum:`ron_fault_t`. ``RON_FAULT_NONE`` is success; anything else is a
  bitmask of faults. Faults latch until explicitly cleared.
* **Sample period is explicit.** ``dt`` is passed to each step call rather
  than fixed at init, so a loop may run at a variable rate.
* **Dimensions are bounded at compile time.** Matrix-based modules size their
  storage from ``RON_*_MAX_*`` macros; see :doc:`platform`.

.. toctree::
   :maxdepth: 1
   :caption: Core

   pid
   feedforward
   filter
   gain_sched
   trajectory
   cascade

.. toctree::
   :maxdepth: 1
   :caption: State estimation and optimal control

   kalman
   observer
   statespace
   lqr
   lqg

.. toctree::
   :maxdepth: 1
   :caption: Tuning and supervision

   autotune
   health
   metrics

.. toctree::
   :maxdepth: 1
   :caption: Foundation

   platform

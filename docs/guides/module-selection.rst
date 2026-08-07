Selecting Modules
=================

Every module except the PID core is optional. On a target where flash is the
binding constraint, disabling what you do not call is the most direct saving
available.

The mandatory baseline
----------------------

The PID core and its integrated feed-forward path are always built. Feed-forward
is not separately optional because PID configuration validation depends on it.

Everything else is controlled by a ``RON_ENABLE_<MODULE>`` option, each
defaulting to ``ON`` so that a plain build gives the complete library.

.. list-table::
   :header-rows: 1
   :widths: 32 38 30

   * - Option
     - Module
     - Forces on
   * - ``RON_ENABLE_FILTER``
     - :doc:`../api/filter`
     - —
   * - ``RON_ENABLE_GAIN_SCHED``
     - :doc:`../api/gain_sched`
     - —
   * - ``RON_ENABLE_TRAJECTORY``
     - :doc:`../api/trajectory`
     - —
   * - ``RON_ENABLE_CASCADE``
     - :doc:`../api/cascade`
     - —
   * - ``RON_ENABLE_KALMAN``
     - :doc:`../api/kalman`
     - —
   * - ``RON_ENABLE_STATESPACE``
     - :doc:`../api/statespace`, :doc:`../api/observer`
     - ``KALMAN``
   * - ``RON_ENABLE_LQR``
     - :doc:`../api/lqr`
     - ``STATESPACE``, ``KALMAN``
   * - ``RON_ENABLE_LQG``
     - :doc:`../api/lqg`
     - ``LQR``, ``STATESPACE``, ``KALMAN``
   * - ``RON_ENABLE_AUTOTUNE``
     - :doc:`../api/autotune`
     - —
   * - ``RON_ENABLE_HEALTH``
     - :doc:`../api/health`
     - —
   * - ``RON_ENABLE_METRICS``
     - :doc:`../api/metrics`
     - —

Dependencies resolve automatically and report themselves at configure time,
so requesting LQG without Kalman is not an error — it turns Kalman on and
says so.

A minimal build
---------------

To build the PID core alone:

.. code-block:: bash

   cmake -B build -S regulon-c \
         -DRON_ENABLE_FILTER=OFF \
         -DRON_ENABLE_GAIN_SCHED=OFF \
         -DRON_ENABLE_TRAJECTORY=OFF \
         -DRON_ENABLE_CASCADE=OFF \
         -DRON_ENABLE_KALMAN=OFF \
         -DRON_ENABLE_STATESPACE=OFF \
         -DRON_ENABLE_LQR=OFF \
         -DRON_ENABLE_LQG=OFF \
         -DRON_ENABLE_AUTOTUNE=OFF \
         -DRON_ENABLE_HEALTH=OFF \
         -DRON_ENABLE_METRICS=OFF \
         -DRON_BUILD_TESTS=OFF

This configuration is built on every push, so it cannot rot unnoticed.

Detecting what was built
------------------------

The build generates ``ron/ron_modules.h``, which records the selection as
``RON_HAVE_<MODULE>`` macros. The aggregate ``ron/ron.h`` uses it to include
only the headers whose modules are present, so it is always safe to include.

Library code that must adapt to the selection can test the macros directly:

.. code-block:: c

   #include "ron/ron.h"

   #if RON_HAVE_KALMAN
       /* Estimator-based path. */
   #else
       /* Direct-measurement fallback. */
   #endif

Prefer including ``ron/ron.h`` over individual module headers unless you have
a reason to be specific — it keeps the include list stable as the module
selection changes.

Other build options
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Option
     - Default
     - Effect
   * - ``RON_USE_DOUBLE``
     - ``OFF``
     - Compute in ``double`` rather than ``float``. See
       :doc:`installation`.
   * - ``RON_BUILD_TESTS``
     - ``ON``
     - Build the unit and integration suites. Skipped when
       cross-compiling.
   * - ``RON_BUILD_EXAMPLES``
     - ``OFF``
     - Build the host example programs.
   * - ``RON_BUILD_BENCHMARKS``
     - ``OFF``
     - Build the host timing benchmark.
   * - ``RON_ENABLE_ASSERT``
     - ``OFF``
     - Turn ``RON_ASSERT`` into a trapping runtime check. Useful during
       bring-up; it aborts rather than returning a fault, so it is not
       intended for production images.

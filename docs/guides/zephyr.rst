Using Regulon with Zephyr
=========================

Regulon ships as a `Zephyr module
<https://docs.zephyrproject.org/latest/develop/modules.html>`_, so a Zephyr
application can pull it in through west and enable it with Kconfig — no
vendoring, no hand-written build glue.

The fit is a natural one: Regulon performs no dynamic allocation, uses no
recursion or unbounded loops, and keeps all controller state in caller-owned
fixed-size objects. Nothing in it needs a heap, and nothing in it blocks.

.. note::

   Everything on this page was verified against **Zephyr 4.1** by building
   and running :ref:`the bundled sample <zephyr-sample>` on
   ``native_sim``.

Adding the module
-----------------

Add Regulon to your workspace's west manifest (``west.yml``):

.. code-block:: yaml

   manifest:
     remotes:
       - name: regulon
         url-base: https://github.com/dtrussel

     projects:
       - name: regulon
         remote: regulon
         revision: main
         path: modules/lib/regulon

Then fetch it:

.. code-block:: bash

   west update

West finds ``zephyr/module.yml`` at the repository root and wires in the
module's Kconfig and CMake automatically. There is nothing to add to your
application's ``CMakeLists.txt``.

Pin ``revision`` to a released tag rather than ``main`` for anything you
intend to ship.

Without a manifest entry
~~~~~~~~~~~~~~~~~~~~~~~~

To try the module against a checkout you already have, point Zephyr at it
directly instead of editing the manifest:

.. code-block:: bash

   west build -b <board> . -- -DZEPHYR_EXTRA_MODULES=/path/to/regulon

Enabling it
-----------

One option turns the library on:

.. code-block:: cfg

   CONFIG_REGULON=y

That builds the complete library. Every optional module defaults to ``y``,
so this is the "everything" configuration; see
:ref:`zephyr-trimming` to cut it down.

Then include the aggregate header and use the API exactly as anywhere else:

.. code-block:: c

   #include <zephyr/kernel.h>
   #include <ron/ron.h>

   static ron_pid_instance_t pid;

.. _zephyr-trimming:

Choosing modules
----------------

Each module has a ``CONFIG_REGULON_<MODULE>`` option mirroring the
``RON_ENABLE_<MODULE>`` CMake option used by the standalone build. The PID
core and its feed-forward path are the mandatory baseline and have no
option — they are built whenever ``CONFIG_REGULON`` is set.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Option
     - Module
   * - ``CONFIG_REGULON_FILTER``
     - :doc:`../api/filter`
   * - ``CONFIG_REGULON_GAIN_SCHED``
     - :doc:`../api/gain_sched`
   * - ``CONFIG_REGULON_TRAJECTORY``
     - :doc:`../api/trajectory`
   * - ``CONFIG_REGULON_CASCADE``
     - :doc:`../api/cascade`
   * - ``CONFIG_REGULON_KALMAN``
     - :doc:`../api/kalman`
   * - ``CONFIG_REGULON_STATESPACE``
     - :doc:`../api/statespace`, :doc:`../api/observer`
   * - ``CONFIG_REGULON_LQR``
     - :doc:`../api/lqr`
   * - ``CONFIG_REGULON_LQG``
     - :doc:`../api/lqg`
   * - ``CONFIG_REGULON_AUTOTUNE``
     - :doc:`../api/autotune`
   * - ``CONFIG_REGULON_HEALTH``
     - :doc:`../api/health`
   * - ``CONFIG_REGULON_METRICS``
     - :doc:`../api/metrics`

Dependencies resolve through Kconfig ``select``, so you ask for what you
want and the chain follows. Requesting LQG alone:

.. code-block:: cfg

   CONFIG_REGULON=y
   CONFIG_REGULON_LQG=y

also enables ``REGULON_LQR``, ``REGULON_STATESPACE`` and
``REGULON_KALMAN``, and compiles the shared matrix helper and observer
alongside them.

To build the PID core alone, switch the rest off explicitly — they default
to ``y``:

.. code-block:: cfg

   CONFIG_REGULON=y
   CONFIG_REGULON_FILTER=n
   CONFIG_REGULON_GAIN_SCHED=n
   CONFIG_REGULON_TRAJECTORY=n
   CONFIG_REGULON_CASCADE=n
   CONFIG_REGULON_KALMAN=n
   CONFIG_REGULON_STATESPACE=n
   CONFIG_REGULON_LQR=n
   CONFIG_REGULON_LQG=n
   CONFIG_REGULON_AUTOTUNE=n
   CONFIG_REGULON_HEALTH=n
   CONFIG_REGULON_METRICS=n

The selection is recorded in a generated ``ron/ron_modules.h`` as
``RON_HAVE_<MODULE>`` macros, and ``<ron/ron.h>`` uses them to include only
the headers whose implementations were compiled in. Application code can
test them too:

.. code-block:: c

   #if RON_HAVE_KALMAN
       /* Estimator-based path. */
   #endif

Precision and floating point
----------------------------

``CONFIG_REGULON_DOUBLE_PRECISION`` makes ``ron_float_t`` a ``double``
instead of a ``float``. It is applied application-wide rather than only to
the library, because the type appears in every structure the application
declares — the library and its callers must agree.

Leave it off on Cortex-M4F/M7 and similar parts whose FPU is
single-precision only: every ``double`` operation would be emulated in
software, costing cycles and flash. Turn it on when the plant is stiff, when
covariances span many orders of magnitude, or when :doc:`../api/lqr`'s DARE
solver struggles to converge.

If your target has an FPU, enable it — Regulon is floating-point throughout:

.. code-block:: cfg

   CONFIG_FPU=y

Threads, ISRs and shared state
------------------------------

Regulon has no global mutable state. Separate instances are fully
independent and need no locking between them.

A single instance is **not** internally synchronised. The library treats
mutual exclusion as the caller's concern, which for Zephyr means:

* **One owner per instance.** The usual arrangement is a dedicated control
  thread, or one ISR, that exclusively owns each controller. This needs no
  locking at all and is the arrangement to prefer.
* **If a second context must touch the instance** — a shell command
  retuning gains, a logger reading metrics — guard it with a
  ``k_mutex`` from thread context, or ``k_spinlock`` if an ISR is involved.
* **Calling from an ISR is supported.** Every ``_step()`` function has
  bounded, allocation-free execution and never blocks. Nothing in the
  library calls into the kernel.

Note that config-mutating calls like ``ron_pid_set_config()`` or
``ron_lqr_set_gains()`` are the ones that need care: a step running
concurrently with a gain change is the race worth guarding, not two reads.

Getting the sample period right
-------------------------------

``dt`` is passed to each step call rather than fixed at init, and it must
match the time that actually elapsed. Deriving both the scheduler period and
``dt`` from one constant keeps them from drifting apart:

.. code-block:: c

   #define LOOP_PERIOD_MS 10
   #define LOOP_DT_S      ((ron_float_t) LOOP_PERIOD_MS / 1000.0F)

For a periodic thread, compute the next deadline *before* the work so
execution time does not accumulate into the period:

.. code-block:: c

   k_timepoint_t next = sys_timepoint_calc(K_MSEC(LOOP_PERIOD_MS));

   fault = ron_pid_step(&pid, setpoint, measurement, LOOP_DT_S, &u, &status);
   /* ... drive the actuator ... */

   k_sleep(sys_timepoint_timeout(next));

If your loop jitters significantly — because it is driven by sensor
completion rather than a timer, say — measure the elapsed interval and pass
the measured value instead of the nominal constant. A ``dt`` that disagrees
with reality mistunes the integral and derivative terms.

Handling faults
---------------

Every step returns a :c:enum:`ron_fault_t` bitmask. ``RON_FAULT_NONE`` is
success; anything else means the output carries the configured safe-state
value rather than a computed one.

.. code-block:: c

   ron_fault_t fault = ron_pid_step(&pid, setpoint, measurement, LOOP_DT_S,
                                    &u, &status);
   if (fault != RON_FAULT_NONE) {
       LOG_ERR("control fault 0x%x", (unsigned int)fault);
       actuator_disable();
   }

Faults **latch**. The controller keeps returning its safe-state output until
``ron_pid_fault_clear()`` is called, deliberately: a loop that silently
recovers from a transient is a loop whose faults nobody investigates.

.. _zephyr-sample:

The bundled sample
------------------

``zephyr/samples/pid_loop/`` is a complete Zephyr application running a PID
loop in a periodic thread against a simulated first-order plant. It builds
and runs without hardware:

.. code-block:: bash

   cd zephyr/samples/pid_loop
   west build -b native_sim/native/64 . -- -DZEPHYR_EXTRA_MODULES=$(pwd)/../../..
   ./build/zephyr/zephyr.exe

Expected output — the loop converging on its setpoint, with the overshoot a
PI controller on a first-order lag should show:

.. code-block:: text

   regulon PID loop sample, 10 ms period
   step   0: y = 20 milli, u = 2050 milli
   step  50: y = 806 milli, u = 1836 milli
   step 100: y = 1091 milli, u = 1298 milli
   step 150: y = 1099 milli, u = 1005 milli
   step 200: y = 1043 milli, u = 938 milli
   step 250: y = 1005 milli, u = 960 milli
   regulon: final y = 994 milli (setpoint 1000 milli)

Its ``prj.conf`` disables every optional module, so it also demonstrates the
minimum-footprint configuration.

Troubleshooting
---------------

**``ron/ron.h: No such file or directory``**
   ``CONFIG_REGULON`` is not set. The module contributes its include
   directories only when enabled, so the header is invisible until it is.

**A module's functions fail to link, or ``RON_HAVE_*`` is 0**
   That module is switched off in Kconfig. Check
   ``build/zephyr/.config`` for the resolved values rather than reading
   ``prj.conf``, since ``select`` can turn things on that you did not ask
   for and board defconfigs can turn things off.

**``undefined reference to sin``/``cos``**
   The biquad coefficient designers in :doc:`../api/filter` are the only
   part of the library that needs libm. ``CONFIG_REGULON_FILTER`` selects
   ``REQUIRES_FULL_LIBC`` for that reason; if you have forced the minimal
   libc, either allow a fuller one or set ``CONFIG_REGULON_FILTER=n``.

**``'u_final' may be used uninitialized`` in ``ron_pid_core.c``**
   A known false positive from GCC at ``-Os`` and ``-O2``, which Zephyr
   uses by default. The variable is written on every path where the caller
   reads it; the compiler cannot see that the helper writing it returns a
   non-``NONE`` fault whenever it does not. Harmless unless you build with
   ``CONFIG_COMPILER_WARNINGS_AS_ERRORS=y``.

**Values look wrong after switching precision**
   ``CONFIG_REGULON_DOUBLE_PRECISION`` changes ``ron_float_t`` for the whole
   application. Rebuild from clean (``west build -p always``) so nothing
   stale is linked against the other type.

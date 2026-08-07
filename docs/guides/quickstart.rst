Quickstart
==========

This walks through building the library, running its tests, and writing a
first control loop. It assumes CMake 3.21 or newer and a C11 compiler.

Build and test
--------------

.. code-block:: bash

   git clone https://github.com/dtrussel/regulon.git
   cd regulon

   cmake -B build -S regulon-c -DRON_BUILD_TESTS=ON
   cmake --build build
   ctest --test-dir build --output-on-failure

All tests should pass. This builds the complete library; see
:doc:`module-selection` for trimming it down.

Run an example
--------------

The repository ships runnable demonstrations of each major module.

.. code-block:: bash

   cmake -B build -S regulon-c -DRON_BUILD_TESTS=OFF -DRON_BUILD_EXAMPLES=ON
   cmake --build build
   ./build/examples/pid_quickstart

The other examples are ``cascade_control_loop``, ``kalman_estimation``,
``statespace_observer``, ``autotune_relay``, ``trajectory_motion`` and
``lqr_lqg_control``. Their sources under ``regulon-c/examples/`` are written
to be read as much as run.

Your first control loop
-----------------------

Every module follows the same shape: declare an instance, fill in a
configuration struct, initialise it once, then call a step function each
sample period.

**1. Declare the instance.** The library never allocates, so the instance is
yours. At file scope, so it outlives the loop:

.. code-block:: c

   #include "ron/ron.h"

   static ron_pid_instance_t pid;

**2. Configure and initialise.** Every field is explicit — there are no
hidden defaults to discover later:

.. code-block:: c

   void init(void) {
       ron_pid_config_t cfg = {
           .Kp = 2.0F, .Ki = 5.0F, .Kd = 0.0F,
           .b = 1.0F, .c = 1.0F,
           .u_min = -10.0F, .u_max = 10.0F,
           .I_min = -100.0F, .I_max = 100.0F,
           .aw_mode = RON_AW_BACK_CALC, .T_aw = 0.05F,
       };
       (void)ron_pid_init(&pid, &cfg);
   }

``u_min``/``u_max`` bound the output to what the actuator can actually
deliver, and ``aw_mode`` decides what the integrator does once that bound is
reached. Setting them is not optional polish: an unbounded integrator winding
up against a saturated actuator is the classic way a PID loop misbehaves.

**3. Step it.** ``dt`` is passed per call rather than fixed at init, so a
variable-rate loop is supported directly:

.. code-block:: c

   void control_isr(ron_float_t setpoint, ron_float_t measurement, ron_float_t dt) {
       ron_float_t u;
       ron_status_t status;
       (void)ron_pid_step(&pid, setpoint, measurement, dt, &u, &status);
       actuator_set(u);
   }

Handling faults
---------------

The casts to ``void`` above keep the example short. Real code should check
the return value, which is a bitmask rather than a single code:

.. code-block:: c

   ron_fault_t fault = ron_pid_step(&pid, setpoint, measurement, dt, &u, &status);
   if (fault != RON_FAULT_NONE) {
       /* u still holds the configured safe-state output. */
       handle_fault(fault);
   }

Faults **latch**. Once set, a fault stays set until
``ron_pid_fault_clear()`` is called, and the controller keeps returning its
safe-state output in the meantime. This is deliberate: a loop must not
silently resume after a transient, because a fault that heals itself is a
fault nobody investigates.

Where to go next
----------------

* :doc:`installation` — consuming the library from another project.
* :doc:`module-selection` — building only what you need.
* :doc:`../api/index` — the full reference.
* :doc:`verification` — what the library's correctness claims rest on.

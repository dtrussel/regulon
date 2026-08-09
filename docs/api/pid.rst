PID Controller
==============

The mandatory core of the library. Provides parallel and ISA-form PID with
derivative-on-measurement, back-calculation and clamping anti-windup,
bumpless transfer, output saturation and rate limiting, and a latching
fault register that must be explicitly cleared.

.. doxygenfile:: ron_pid.h
   :project: regulon

.. doxygenfile:: ron_pid_types.h
   :project: regulon

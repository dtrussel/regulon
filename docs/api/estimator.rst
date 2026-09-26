State Estimator
===============

The state-estimate source shared by the state-space controller and the LQR:
an external vector, an embedded Luenberger observer, or an embedded Kalman
filter. A controller embeds it as ``cfg.est`` / ``est``; advance the embedded
estimator with these functions on ``&ctrl.est`` before stepping the
controller.

.. doxygenfile:: ron_estimator.h
   :project: regulon

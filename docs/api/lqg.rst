Linear Quadratic Gaussian
=========================

Combines an LQR control law with a Kalman state estimator, following the
separation principle: both gains are solved independently at init time.

.. doxygenfile:: ron_lqg.h
   :project: regulon

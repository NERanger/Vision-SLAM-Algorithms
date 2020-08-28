# Nonlinear Optimization

Motion equation and observation equation will not fully hold because of the noise. This section is about using nonlinear optimization to perform state estimation and the usage of g2o and Ceres.

## Practice

* [Curve Fitting with Ceres](./ceres_curve_fitting)
* [Curve Fitting with g2o](./g2o_curve_fitting)
* [Gauss Newton by hand (not using Ceres or g2o)](./gauss_newton)

## Principle

2-norm (also called Euclid norm)
$$
||x||_2=\sqrt{\sum^N_{i=1}x_i^2}
$$

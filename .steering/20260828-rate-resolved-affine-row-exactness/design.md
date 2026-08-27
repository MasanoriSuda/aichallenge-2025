# Design

The canonical transition contains four rows that are exactly affine for a
constant input over one stage:

- `v+ = v + a dt`
- `theta+ = theta + v_theta dt`
- `delta+ = delta + delta_rate dt`
- the first-order yaw-response state under a steering ramp

The generic finite-difference Jacobian introduced tiny coefficient errors in
these rows. OSQP solved the numerical tangent it received, while the immutable
artifact validator checked the intended exact equations. This split made a
valid solve unpublishable.

Keep numerical differentiation for the nonlinear Frenet lateral, lag, and
heading rows. After numerical differentiation, overwrite the four affine rows
with their exact coefficients, then recompute the equality offset from the
same canonical nonlinear transition. No tolerance is changed.

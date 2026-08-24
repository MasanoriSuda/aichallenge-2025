# Design

OSQP uses `0.5 z' P z + q' z`. A reference cost contributes `-w*r` to
`q`, while progress maximization contributes a separate negative coefficient.
Folding the reward into a shifted reference is only equivalent when a positive
quadratic weight exists and obscures ownership. The QP request therefore gains
one optional full-size additional linear vector. Assembly starts from that
vector and then adds reference terms.

The semantic adapter carries per-stage legacy linear terms and maps them to the
same first-five state and acceleration/virtual-progress input coordinates.
There is no legacy curvature linear term in current Track/Cruise construction,
but the adapter deliberately does not reinterpret such a term as steering:
nonlinear unit conversion of an independent curvature linear objective needs a
separate approved contract.

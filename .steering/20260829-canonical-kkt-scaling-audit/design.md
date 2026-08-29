# Design: canonical KKT scaling audit

The previous Slice proved that a physically certified ShiftOut iterate is
discarded at OSQP's KKT termination boundary.  A fixed number of additional
explicit Ruiz passes is not canonical: three and ten passes solve ShiftOut but
regress the frozen Follow sequence 5575, while one pass has the opposite
behavior.

This Slice therefore compares complete numerical ownership choices rather
than tuning an iteration count:

1. current explicit physical-box and row-tolerance transform with OSQP
   internal scaling disabled;
2. the same explicit transform with OSQP's modified Ruiz scaling;
3. raw physical coordinates with OSQP owning scaling;
4. available independent active-set/proximal QP backends.

The offline backend tool now accepts `bucket=none`, which means the recorded
QP is unchanged.  Output primals remain audit evidence and are independently
checked by `mpcc_architecture_compare --external-primal`.

The comparison follows OSQP's published scaling relation
`x_bar = D^-1 x`, `P_bar = c D P D`, `q_bar = c D q`, and
`A_bar = E A D`.  This audit does not introduce a second production solver.

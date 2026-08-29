# Design: globalized wall-pose SQP audit

The frozen D oracle proves a physically valid trajectory exists, while the
normal QP is blocked by hard lag/heading buckets.  Removing both buckets from
one QP is not itself safe: its exact nonlinear rollout can still cross the
wall corridor by millimetres.

The first observation arm therefore copies the existing bucket audit and
omits both artificial pose boxes together.  The normal Phase-I projection,
racing objective and complete exact proof chain remain unchanged.  No output
can reach a production Store.

If that racing iterate is not certified, the second arm will rebuild dynamics,
dynamic supports and wall evidence around the latest pose-relaxed iterate at
each bounded depth.  Each depth is independently proof checked.  This models
the missing feasible-QP/globalization entry without changing the production
solver until frozen evidence supports it.

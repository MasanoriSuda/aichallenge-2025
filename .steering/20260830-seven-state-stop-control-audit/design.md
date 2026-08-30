# Design

Extend only the architecture-comparison result with a compact immutable
control diagnostic derived from `ExecutionArtifact::control_stages`.

The diagnostic belongs to an audit arm and cannot be converted to a command.
The existing CLI prints it beside the accepted seven-state Stop result.  No
production header, Store or publisher consumes it.

The result distinguishes two candidate families:

1. low-complexity maximum-braking plus a small number of steering-rate arcs;
2. coupled acceleration/steering/progress optimization requiring the existing
   seven-state solver off the control callback.

The distinction is evidence-based: a lattice is considered plausible only if
the accepted sequence has few sign changes and mostly saturating/constant
controls.

The comparison also fixes the longitudinal contract.  The first version of
the causal Stop arm allowed the normal velocity objective to accelerate before
braking, so it was not comparable to the production maximum-braking Stop.  In
this Slice every future velocity node is fixed to the solver-safe
maximum-braking schedule.  Acceleration remains inside the original physical
input interval; only the state schedule is fixed, avoiding an empty
solver-inset input equality.

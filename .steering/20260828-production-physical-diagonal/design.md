# Design: one physical dynamic-obstacle contract

## Root cause and repair mapping

Cause X is the axis-only candidate representation.  It cannot describe the
continuous behind-to-side body separation which exists in the frozen current
world.  The downstream `partial_side_escape` branch then substitutes a
wall-only reachability witness and can create a trajectory rejected by the
unchanged exact dynamic certificate.

Change Y is one physical separating-row representation derived from the same
asymmetric ego body and peer circle used by final certification.  It replaces,
rather than supplements, `partial_side_escape`.

## Provenance contract

The physical geometry is not read from mutable controller state inside the
solver.  A submission contains an immutable world payload with:

- observation generation and observation time;
- exact target identity and target observation generation;
- asymmetric ego footprint and configured margin;
- current peer radius including prediction/covariance margin;
- canonical control prefix and wall-grid fingerprint.

The payload is sealed into the solver snapshot fingerprint. Synchronous
pre-entry, asynchronous pre-entry and normal Track/Cruise submission must use
the same binding helper. A missing payload cannot authorize physical diagonal
refinement. A present but mismatched observation generation rejects assembly;
neither case is replaced with default dimensions.

## Topology selection

The frozen `start=1/full=3` result is evidence, not policy. Production derives
the earliest causal receding schedule from the first valid obstacle stage:
`start = first_valid + 1`, `full_side = start + 2`. This leaves the immutable
current state untouched and connects the complete behind and side disjuncts
over two mutable stages. If the current horizon cannot contain the schedule,
no diagonal candidate is emitted. The candidate remains subject to the
existing seven-state solver and unchanged certificates; an uncertified affine
support row cannot publish.

The derivation belongs to candidate generation, not to a new lifecycle or
fallback. It is bounded, deterministic and latest-world-only. Existing
left/right homotopy identity and no-return state remain supervisory inputs.

## Deletion boundary

In the same promotion change, remove:

- `partial_side_escape` decision branches;
- partial escape row construction;
- partial escape counters and diagnostic fields;
- comments/tests which describe wall-only witness separation as safe.

Experiment-only candidate flags may remain only for frozen architecture
replay if they cannot enter live authority.  Production may not branch on
them.

## Proof boundary

The physical support plane is an SQP convexification evaluated at a witness
heading.  Therefore final authority still requires:

1. seven-state dynamics/artifact adaptation;
2. dense swept physical wall proof;
3. timed all-obstacle nonlinear body proof;
4. terminal successor viability;
5. current-world identity/revalidation.

## Production entry audit

Normal Track/Cruise submission already bound `ReplayWorld`. The synchronous
and asynchronous pre-entry evaluation paths bound the physical wall snapshot
but omitted `bind_rate_resolved_replay_world`; this was an upstream integration
defect which split behavior by entry path. Both paths now invoke the same
binding helper before the common solver. If immutable world construction
fails, the solver receives no physical diagonal authorization and remains on
complete axis-aligned disjuncts.

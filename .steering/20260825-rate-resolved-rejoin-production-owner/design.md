# Design

## Causal chain

```text
OvertakeLine Recovery
  -> canonical intent Rejoin
  -> private VelocityProgress5State solve
  -> fresh-only physical proof
  -> private canonical plan publication
```

Every other normal intent now follows:

```text
semantic MpcProblem
  -> shared VelocitySteeringProgress6State request
  -> immutable six-state artifact
  -> current-world wall + dynamic-obstacle revalidation
  -> canonical normal command or Emergency
```

The formulation switch is not required by Rejoin semantics. Recovery already
constructs the base-line lateral reference and bounds before either solver is
called. The private five-state lifecycle therefore duplicates mathematical
and authority ownership rather than providing a distinct safety function.

## Selected change

1. Extend the shared intent capability and semantic-scope resolver with a
   typed Rejoin scope.
2. Use `MpcProblem::rejoin_shadow_requested` only as semantic availability;
   it no longer selects a different formulation.
3. Build, solve, certify and retain Rejoin through the existing six-state
   producer and current-world proof.
4. Treat Rejoin as targetless and side-less. Its reference is the Recovery
   line already embedded in the semantic problem.
5. Delete the private five-state Rejoin lifecycle and dispatch atomically.

## Retained proof

Rejoin does not need Follow's target-gap extension or Overtake's mission-side
identity. It does require the generic retained proof already used by the
shared owner:

- identical six-state formulation and semantic intent;
- continuous course progress and reachable command;
- current measured-to-control pose path;
- current physical wall grid and yawed vehicle footprint;
- all currently observed V2X vehicles as dynamic obstacles.

Thus retained Rejoin is neither a plan-age exception nor Track/Cruise state
borrowing. It is the same immutable artifact re-certified against the current
world.

## Rejected alternatives

- Keep five-state as a Rejoin fallback: preserves dual normal authority.
- Reuse the old fresh-only evaluator: retains the formulation switch and loses
  the common retained proof.
- Add Rejoin to Track/Cruise by aliasing its boolean: obscures semantic
  provenance. A separate typed scope input is used instead.
- Tune wall or solver parameters: does not repair the authority split.

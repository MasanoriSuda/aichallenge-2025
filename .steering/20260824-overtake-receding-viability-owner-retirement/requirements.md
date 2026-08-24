# Requirements

Retire the remaining legacy receding-horizon Mission-viability authority from
canonical Overtake production.

## Evidence boundary

`output/20260824-130017` showed one generation-1 ShiftOut where:

- canonical MPCC had already selected explicit Emergency because no retained
  current-world-certified plan was available;
- a fresh generation-1 DP reference was active;
- the independent legacy receding validator then invalidated the Mission and
  forced DynamicWait/Recovery.

The second transition owner prevents the canonical producer from recovering
within the same tactical Mission.

## In scope

- Make legacy receding physical validation advisory for canonical
  ShiftOut/Pass/Return when a complete reference/corridor contract is still
  available.
- Preserve the Mission generation while canonical normal authority or the
  explicit Emergency supervisor owns actuation.
- Keep legacy revalidation telemetry and classify its demotion in the central
  decision trace.
- Add deterministic tests which prove the demotion cannot bypass an actual
  wall/contact/Emergency fault or an incomplete reference contract.
- Update the migration map and patch ledger when the old transition owner is
  physically unreachable in canonical Overtake scope.

## Out of scope

- Wall, vehicle-clearance, acceleration, solver or timing parameter changes.
- A grace period, retry counter, lease, cooldown or new fallback command.
- Removing the DP/reference builder itself.
- Rejoin, Stuck/Reverse Recovery or the external Emergency supervisor.
- Tuning Overtake performance.

## Definition of done

- A legacy reference-profile rejection cannot invalidate a canonical
  ShiftOut/Pass/Return Mission when its typed reference/corridor input remains
  complete and no external hard fault exists.
- The same rejection still reaches structured telemetry.
- Missing/invalid reference geometry and external hard faults remain
  fail-closed.
- Static/package tests and a bounded dynamic run pass.
- The dynamic run shows no
  `optimized horizon failed physical revalidation -> DynamicWait/Recovery`
  transition under canonical Overtake production.

# Requirements

## Objective

Remove the self-invalidating publication edge exposed after the certified Stop-lattice production bridge selected source 1629 at decision 2257.

The correction must preserve immutable solver provenance. It must not add a lease, grace period, timeout, fallback owner, solver-tolerance change or clearance change.

## Frozen evidence

Run: `output/20260831-001629/d1/autoware.log`

1. decision 2257: ordinary Pass authority failed with `terminal-contingency-unavailable`;
2. the Stop-lattice alternate for source 1629 passed current-world evaluation and became canonical normal authority;
3. final publication then logged `Published Stop lattice source rejected` with `plan=1, artifact=1, source=0`;
4. the rejection invalidated the published-source identity and current-world alternate;
5. decision 2259 had no lattice alternate and emitted external Stop;
6. authority subsequently alternated between normal Pass and external Stop.

## Required invariant

Every certified plan admitted to normal production authority must carry the immutable solver source snapshot which actually generated its execution artifact.

For a Stop-lattice plan, this is the publisher-boundary-rebased Stop candidate, not the pre-rebase normal snapshot and not a null pointer.

## Non-goals

- Do not weaken `update_published_stop_lattice_observation()` source validation.
- Do not retain an unproven alternate after source validation fails.
- Do not make the Stop-lattice plan a second Store or publisher.
- Do not change wall, dynamic-obstacle, solver or vehicle parameters.
- Do not claim that this change fixes the later actual wall contact; classify it independently after provenance continuity is restored.

## Definition of Done

- A failure-first unit test proves the accepted Stop-lattice certified plan owns a non-null, matching solver source snapshot.
- That snapshot contains the publisher-boundary-rebased Stop problem actually submitted to the private solver.
- Source mismatch and supersession behavior remain fail-closed.
- Static build/tests pass.
- Dynamic `make dev2` shows a selected Stop-lattice bridge is not immediately rejected as `source=0`.
- Any remaining Stop or wall failure is recorded as a new upstream/downstream classification rather than hidden by this Slice.

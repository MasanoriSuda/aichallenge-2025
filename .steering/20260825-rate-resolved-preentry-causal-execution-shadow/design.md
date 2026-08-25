# Design

## Responsibility split

```text
5 Hz tactical worker
  left/right candidate search + homotopy selection
                 |
                 | side + Mission geometry only
                 v
40 Hz live callback
  current isolated deep-owned snapshot
  current normal command commit
  bind draft to committed steering predecessor
                 |
                 v
latest-only execution worker
  prospective-problem build from the frozen snapshot
  selected-side six-state solve
  exact swept-wall proof
                 |
                 v
live tactical-identity classification
live current-world join (shadow only)
```

The old tactical six-state trajectory is retained only as diagnostic evidence.
The new execution producer never consumes it.

## Causal ordering

The callback freezes the current model, V2X world and selected Mission geometry
before the current normal command is chosen. It does not build the prospective
problem there. After Track/Follow commits its command, the draft is bound to
that exact steering value and submitted; the latest-only worker builds and
solves the prospective problem. This preserves the canonical
Track/Cruise/Follow causal ordering without spending the 31--45 ms problem
build inside the 25 ms callback.

## Refactor boundary

Extract the prospective branch setup from `evaluate_extended_mpcc_branch()`:

1. resolve the selected candidate;
2. apply it only to an isolated `MPC` snapshot;
3. build current `MpcProblem` and extended six-state request;
4. seal prospective intent and target identity.

The existing five-state tactical evaluator and the new execution draft builder
must call the same setup helper. This removes duplicated Mission-to-problem
translation instead of adding another special path.

## Publication boundary

The new worker has a private mailbox and private solver context. It receives a
null certified-plan store. The live consumer records feasibility and applies
the existing current-world revalidator for observation, but cannot store,
select or publish the result.

Tactical identity and physical safety are observed separately. A current
opposite-side selection, target mismatch, Mission-generation mismatch or
sequence regression blocks even shadow current-world proof. If the tactical
selection is temporarily unavailable, the result may still be checked against
the current wall and every current V2X obstacle, but it remains
`tactical_authority_current=false`. A current same-side selection is required
before the observation can be counted `authority_ready`.

## Rejected alternatives

- Reuse the old tactical trajectory: predecessor causality is false.
- Re-run full tactical search in the callback: measured 88--115 ms.
- Build even only the selected prospective problem in the callback: measured
  31--43 ms and caused 25 ms callback overruns.
- Run the full tactical search in another worker: it recreates the same stale
  trajectory problem.
- Relax continuity thresholds: hides the causal mismatch.

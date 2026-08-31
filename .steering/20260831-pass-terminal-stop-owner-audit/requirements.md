# Requirements: Pass terminal Stop owner audit

## Objective

Determine why a certified Pass continuation loses normal authority before the
later observed actual-footprint wall violation, and identify the earliest
current-world state at which another certified maneuver was still available.

## Frozen evidence

- Run: `output/20260831-081212`, domain 2, episode 3.
- Pass authority joined atomically at decision 3379.
- At decision 3418 the current Pass publisher interval and continuation wall
  checks were clear, but the independently generated `track-reference-path`
  terminal Stop collided with the wall at exact sample 15.
- Emergency Stop interrupted the normal execution ledger at decision 3418.
- The actual-footprint wall violation was observed later at decision 3424.
- Snapshot:
  `d2/mpcc_architecture_snapshots/000000003418-3e0daa84383066b3-`
  `pass-side-positive-physical-proof-terminal-contingency-unavailable/`
  `snapshot.yaml`.
- Earlier actionable snapshot:
  `d2/mpcc_architecture_snapshots/000000002782-1418eb976741a6cc-`
  `pass-side-positive-wall-refinement-coupled-solve-rejected/snapshot.yaml`.

## Constraints

- Audit only until the earliest violated invariant is demonstrated.
- Do not change clearance, solver tolerance, speed, timing, lease, grace,
  fallback, retry, Mission lifecycle or production authority during the
  comparison.
- Do not suppress recursive terminal Stop proof.
- Compare candidate representation under the same physical and dynamic proof.

## Acceptance

- Classify the frozen snapshot through the A/B/C/D architecture comparison.
- Compare fixed/path-profile Stop with bounded causal seven-state Stop.
- Identify whether the defect is physical infeasibility, candidate generation,
  solver limitation, model/certificate mismatch or lifecycle/scheduling.
- Record a production repair only if one cause is supported and directly
  falsifiable.
- Re-run the earlier actionable snapshot after the repair; do not infer
  physical infeasibility from the later already-unavoidable state.

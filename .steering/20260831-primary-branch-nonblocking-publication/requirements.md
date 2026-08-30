# Requirements: nonblocking primary normal branch publication

## Objective

Prevent a fully certified Cruise/Follow dynamic-obstacle branch from being
withheld while the same-epoch sibling homotopy is still solving.

## Frozen evidence

- Baseline: `a3ad1649`
- Run: `output/20260831-031354/d1`
- Snapshot:
  `mpcc_architecture_snapshots/000000000983-6291ad64378e923e-follow-side-neutral-physical-proof-terminal-contingency-unavailable/snapshot.yaml`
- Visible failure: decision 983 published Stop after the retained/candidate
  terminal contingency became unavailable.
- The next fresh stateless normal candidate was not admitted until decision
  1001, about 0.575 s later.

## Architecture classification

The frozen snapshot was evaluated with the repository A/B/C/D tool.

- A, persistent Mission + seven-state SQP: terminal successor rejected.
- B-left, stateless receding Bundle + the same seven-state SQP: accepted.
- B-right, stateless receding Bundle + the same seven-state SQP: accepted.

Classification: **A fails, B succeeds: lifecycle/scheduling defect**.

The upper-run log in `.steering/ano/` independently shows that its main GMPCC
continues while asynchronous left/right tactical candidates complete or fail.
A sibling candidate failure does not block the current command path.

## Invariants

- A branch reaches the candidate Store only after solver, exact physical wall,
  current-world dynamic-obstacle, and certified-plan checks all pass.
- The preferred/primary branch must not wait for sibling completion before it
  becomes available to the existing current-world consumption proof.
- The sibling is evidence only. It may join the branch bank only under the
  exact immutable source epoch.
- If the primary fails and a certified sibling completes, the sibling may
  become the candidate through the same Store and current-world proof.
- A late sibling from an older epoch must not modify a newer branch bank or
  candidate.
- No production authority, solver setting, timeout, lease, grace period,
  fallback, velocity policy, wall clearance, or safety margin is changed.

## Definition of done

- Normal avoidance no longer creates a thread per evaluation.
- Primary certification is exposed without waiting for sibling completion.
- Same-epoch branch evidence supports safe late sibling attachment.
- Unit tests cover attachment order and stale late completion.
- Static architecture contracts, build, and package tests pass.
- Dynamic acceptance shows reduced Stop-to-fresh-candidate latency without
  stale epoch adoption or authority regression.

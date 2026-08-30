# Requirements

## Objective

Restore normal authority when a terminal Stop is physically clear in the
exact occupancy grid but the progress-aligned approximate lateral support
rejects any sample before the physical proof is reached.

## Frozen evidence

- Run: `output/20260831-040106/d2`
- Snapshot: decision 889, Cruise, `terminal-contingency-unavailable`
- Normal continuation: nonlinear rollout accepted; exact current-wall path
  accepted.
- Terminal Stop synthesis: rejected at sample 0 as
  `invalid-lateral-bounds`.
- Initial lateral state: `-2.837629 m`; approximate lower support:
  `-2.797560 m`.
- The same current state is already held by the external Emergency Stop.
- Follow-up run: `output/20260831-041927`.
- D1 rejected a terminal Stop at sample 254 and D3 at sample 66 although the
  corresponding continuation publisher interval passed exact wall proof.
- D3 then remained in Stop for about 24 seconds before normal authority was
  reconstructed.

## Root-cause classification

`solve succeeds but proof fails: model/certificate mismatch`.

The progress-aligned lateral profile is a conservative planner support, while
the occupancy-grid swept-footprint proof is the physical wall authority. The
former was incorrectly acting as a second hard wall certificate. It can
exclude exact-grid-clear states at the initial point or later in the Stop
rollout and prevent the authoritative proof from running.

## Constraints

- Do not change production authority, solver settings, timeouts, leases,
  fallback rules, wall clearance, or lateral margin.
- Do not accept a Stop solely because it fits an approximate support.
- Keep approximate-support mismatch as diagnostics; never use it to bypass
  exact occupancy-grid or timed dynamic-obstacle proof.
- Preserve exact wall and dynamic-obstacle proof as final authority.

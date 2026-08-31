# Requirements

## Objective

Remove the production scheduling defect which made a physically certified
seven-state terminal Stop result obsolete before canonical normal authority
could evaluate it.

## Observed failure

Run `output/20260831-105057/d1` entered ShiftOut and lost ordinary authority
with `terminal-contingency-unavailable`.  The Stop worker eventually produced
an exact wall/dynamic-certified seven-state Stop, but the result was about
1.1 seconds old and current-world join rejected it as
`steering-unreachable`.  Emergency Stop then preceded Recovery.

## Constraints

- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not change solver tolerances, wall/vehicle clearance, weights or horizon.
- Keep one canonical normal command publisher.
- Preserve exact wall, timed dynamic-obstacle and terminal-rest proofs.
- Keep broad control-lattice exploration available for offline comparison,
  but do not allow it to block production observations.
- Retain a certified Stop artifact only within the same target, Mission
  generation, side and intent; re-prove it against the current world before
  selection.

## Definition of Done

- The live worker executes only one direct seven-state Stop solve per source.
- A running source is not cancelled by a newer source; only the newest pending
  source is coalesced.
- A result may cross producer epoch changes within the same tactical scope,
  but target/side/generation/intent changes invalidate it.
- Build and complete package tests pass.
- A dynamic run demonstrates bounded one-candidate worker behavior and records
  the next failure without hiding it.
- The next failure is classified from one immutable snapshot before another
  production change is proposed.

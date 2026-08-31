# Requirements: ShiftOut wall-proof escape audit

## Objective

Capture the immutable current-world boundary where a solved ShiftOut/Pass
seven-state QP is rejected by the final exact physical wall proof, then use
that frozen input for the architecture A/B/C/D comparison.

## Frozen evidence

Use `output/20260831-163404/d1/autoware.log`, especially sequence 1073.  The
QP and its internal wall refinement are accepted, while the final exact proof
rejects stage 40 with `hard-wall-contact`.  The previously published artifact
then ages out and Stop becomes the only certified authority.

## Constraints

- do not change production authority;
- do not change a clearance, solver tolerance, timeout, lease, grace or
  fallback;
- do not weaken Stop selection when no normal trajectory is certified;
- persist only immutable observation data, never a retained Mission path;
- keep architecture evidence work off the control callback.

## Acceptance

- the first complete ShiftOut/Pass exact-wall rejection per homotopy is
  persisted beneath `mpcc_architecture_snapshots`;
- the snapshot loads in `mpcc_architecture_compare` and can run A/B/C/D;
- existing solver-rejection and terminal-contingency recording remains valid;
- production commands and authority decisions are unchanged.

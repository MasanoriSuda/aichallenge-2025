# Design: late exact-disjunction candidate

## Evidence and alternatives

1. Relax wall or obstacle proof: rejected. The accepted audit arm passes the
   existing proofs unchanged.
2. Add SQP iterations or change solver tolerances: rejected. The accepted arm
   succeeds in one direct solve and reports `continuation=0`.
3. Copy the exhaustive 210-schedule audit lattice into production: rejected.
   It costs about two minutes per snapshot and violates the bounded async
   worker contract.
4. Append a fourth candidate: rejected. It preserves the obsolete producer
   and increases live compute.
5. Replace the late coupled-diagonal member with the evidenced complete
   disjunction member: selected.

## Bounded population

The population remains:

1. direct selected-side candidate;
2. mid-horizon physical-diagonal candidate;
3. late exact-disjunction candidate.

For a horizon of `H` control stages, the late candidate uses the final three
dynamic-obstacle stages as the complete selected-side suffix:

- stages `[0, H-4]`: complete stay-behind disjunct;
- stages `[H-3, H-1]`: complete selected-side disjunct;
- first-ahead stage `H`: no ahead row in this finite horizon; terminal
  successor proof owns safe continuation.

The three-stage suffix is the latest bounded topology member represented by
the frozen accepted schedule `(17, 20)` for `H=20`. It is a topology sample,
not a physical clearance threshold. The candidate retains the normal
selected-side soft reference, so the optimizer may begin a smooth lateral
movement before the side row becomes mandatory.

## Atomic replacement

Delete the `LatePhysicalDiagonal` enum/log identity and the normalized
last-third coupled-diagonal construction. Replace them at the same population
index with `LateExactDisjunction`. No new Store, publisher or authority route
is introduced.

## Falsifiers

- The frozen accepted arm ceases to pass the unchanged exact proofs after it is
  generated through the production population.
- Dynamic execution repeatedly postpones lateral movement instead of following
  the side reference.
- Candidate compute or callback tail increases despite unchanged population
  size.
- A failed proof reaches normal authority.

Any falsifier rejects this production Slice; it does not justify restoring the
old candidate as a fallback.

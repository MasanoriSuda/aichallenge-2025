# Design

## 1. One semantic owner for Return

`Return` means rejoining the racing line. It is not an alias for passing on the
Mission side. The immutable current-world producer may shape speed and dynamic
constraints to keep the rejoin physically safe, but it may not replace the
Return lateral reference with a new pass-side target.

ShiftOut and Pass continue to use the bounded pass-side population. Return
uses a dedicated stateless producer and the same seven-state SQP and proof
pipeline.

## 2. Current-world Return relation

The producer projects the selected target into the same course frame as the
source and classifies its current physical relation using the asymmetric ego
footprint plus opponent radius:

- `TargetAhead`: ego is behind by the complete longitudinal separation;
- `TargetRear`: ego is ahead by the complete longitudinal separation;
- `SidePositive` / `SideNegative`: bodies are separated laterally;
- `OverlapOrAmbiguous`: no complete disjunct is currently proven.

No Mission clock, retained geometry or previous candidate participates.

The relation selects a complete convex topology for the current horizon:

- TargetAhead -> stay-behind rows;
- TargetRear -> ahead rows;
- SidePositive/Negative -> keep the proven current side until Return terminal
  viability is reached;
- ambiguous -> reject.

## 3. Preserve Return geometry

The Return builder starts from the current canonical Return source, rebuilds
only the current target tube and dynamic topology, and preserves:

- state references;
- state/input bounds;
- stage timing;
- wall geometry;
- Return terminal reference.

It reseals the exact problem identity after adding the current-world target
topology. Unlike the generic pass builder, it never writes
`target_lateral + side * separation` into the Return reference.

## 4. Solved-terminal semantic certificate

Add a data-only terminal-intent evaluation after SQP artifact construction and
before Store admission.

For Return, the terminal predicted state must be inside the canonical Return
successor interval derived from the source terminal reference and the existing
Return handoff contract. The check consumes the solved artifact terminal
lateral and heading states and records:

- intent;
- terminal lateral and heading;
- required interval;
- accepted/rejected reason.

The certificate is evaluated again by the architecture comparison. A source
declaration or terminal bound merely containing zero is not sufficient.

ShiftOut and Pass keep their existing successor behavior in this Slice. Stop
contingency certification remains unchanged.

## 5. Atomic producer replacement

Replace the generic `build_bounded_candidates(source, execution_side_sign)`
edge for canonical Return with the dedicated Return population. Do not leave
both producers reachable.

Every candidate still traverses:

1. seven-state SQP;
2. nonlinear row certificate;
3. exact swept-wall proof;
4. exact dynamic-obstacle proof;
5. solved-terminal semantic proof;
6. exact terminal Stop contingency;
7. existing Store/admission/publisher path.

## 6. Diagnostics

Candidate and decision diagnostics distinguish:

- `return-target-ahead-stay-behind`;
- `return-target-rear-stay-ahead`;
- `return-side-positive-hold`;
- `return-side-negative-hold`;
- `return-relation-ambiguous`;
- `return-terminal-semantic-rejected`.

This allows a future failure to be classified before Emergency or wall contact
becomes the visible symptom.

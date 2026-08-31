# Results: ShiftOut wall-proof escape audit

## Validation

- Source-contract test: `98 passed`.
- `make autoware-build`: 25 packages completed successfully.
- Dynamic run: `output/20260831-164618`.
- Production authority was not changed by this Slice.

The run persisted the previously missing exact-proof boundaries, including:

- ShiftOut positive, sequence 770;
- Pass negative, sequence 901;
- Pass positive, sequence 1027.

## Frozen comparison 1: ShiftOut sequence 770

Snapshot:

`output/20260831-164618/d1/mpcc_architecture_snapshots/000000000770-d2880bdc2f32fca2-shiftout-side-positive-physical-proof-outer-exact-physical-wall-rejected/snapshot.yaml`

Key results from the same interaction fingerprint
`15170388385110097058`:

| Arm | Result |
|---|---|
| persistent A | wall proof rejected, hard contact at stage 106 |
| stateless left B | same wall-proof rejection |
| stateless right B | accepted ManeuverBundle |
| production left G | same wall-proof rejection |
| production right G | accepted ManeuverBundle |

The opposite current-world homotopy was physically and terminally viable.

## Frozen comparison 2: Pass sequence 901

Snapshot:

`output/20260831-164618/d1/mpcc_architecture_snapshots/000000000901-eeac747e851de20d-pass-side-negative-physical-proof-outer-exact-physical-wall-rejected/snapshot.yaml`

Key results from interaction fingerprint `17198249163769111053`:

| Arm | Result |
|---|---|
| persistent A | wall proof rejected, hard contact at stage 210 |
| stateless right B | same wall-proof rejection |
| stateless left B | accepted ManeuverBundle |
| production right G | same wall-proof rejection |
| production left G | accepted ManeuverBundle |

Again, the opposite current-world homotopy was viable at the exact instant the
committed side lost its physical certificate.

## Exit classification

`A fails, B succeeds`: **persistent Mission/homotopy lifecycle defect**.

This is not physical infeasibility: an exact opposite-side Bundle exists on
both frozen failures.  It is not primarily candidate generation: the
production stateless population can generate and certify that Bundle.  It is
not a single-SQP limitation: the ordinary opposite-side seven-state solve is
already sufficient.

The unresolved production defect is that commit/no-return and branch-bank
lifecycle prevent the live controller from adopting the certified sibling
when the selected side loses its exact wall certificate.  Stop then correctly
wins because the selected normal authority has no proof.

## Next Slice boundary

Audit the selected-side failure to sibling adoption join.  A repair may only
promote a sibling from the same immutable interaction fingerprint and must
atomically replace trajectory, physical/dynamic certificates, terminal
successor and tactical side.  Do not preserve old Mission path samples and do
not weaken Stop when no such complete sibling exists.

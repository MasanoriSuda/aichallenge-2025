# Results

## Frozen evidence

- Run: `output/20260828-174825`
- Snapshot: sequence 1755, Follow dynamic-obstacle refinement rejection
- Source interaction fingerprint: `18187803820907329878`
- Initial speed: `3.68481 m/s`
- Approximate initial longitudinal reserve after body overlap: `1.89 m`
- Maximum-braking stopping distance: approximately `2.29 m`

The persistent Follow stay-behind disjunct was therefore physically late. The
question was whether the whole world was infeasible or only the selected
longitudinal branch.

## Same-snapshot comparison

| Arm | SQP/proof result | Terminal progress | Terminal speed | Minimum wall reserve |
|---|---|---:|---:|---:|
| persistent Follow A | solver rejected | N/A | N/A | N/A |
| stateless positive-side B | certified bundle | 9.707 m | 0.332 m/s | 0.528 m |
| stateless negative-side B | certified bundle | 14.362 m | 2.956 m/s | 0.528 m |

Both B candidates consumed the same immutable current world, seven-state SQP,
solver policy, wall model, target tube and exact physical proof as A. Neither
candidate changed production authority.

## Classification

`A fails, B succeeds`: **normal-intent tactical/candidate ownership defect**.

The world was not physically blocked. Follow allowed dynamic refinement to add
only the stay-behind branch because it owned no tactical side. The first
production response to an already-late longitudinal branch was therefore
Emergency/fallback even though two fully certified lateral escape artifacts
existed.

This is not a solver-tolerance, clearance, braking or Mission-resume problem.
The next Slice may connect a certified stateless Follow escape to the existing
single seven-state authority, but it must preserve current-world fingerprint,
exact proof, successor and publication checks. It must not make the audit
builder itself authoritative.

## Verification

- `make autoware-build`: passed, 25 packages.
- focused stateless and architecture comparison tests: 2/2 targets passed.
- frozen architecture replay: persistent A rejected; both B arms accepted.

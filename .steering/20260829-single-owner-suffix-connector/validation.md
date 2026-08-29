# Validation

## Static result

- package build: passed for 25 packages;
- connector ownership tests: 4/4 passed;
- rate-resolved shadow tests: 42/42 passed;
- execution-contract tests: 75/75 passed.

The static result proved identity and scheduling mechanics only. It did not
prove that the direct old-suffix candidate was a numerically valid connection
from the latest published steering state.

## Dynamic result

### Ordinary `make dev2`

Artifact: `output/20260829-185133`

AWSIM remained in `spawned`, so this run is not movement evidence. It did show
that D1 made 317 connector claims, consumed 272 results and admitted 31 plans
to the canonical Store. Nine connector sequences were later observed as
canonical published plan identities. The connector therefore reached the
existing authority chain, but this run could not validate vehicle behavior.

### Autoware-first bounded two-domain run

Artifact: `output/20260829-185730`

Autoware domains were started before the simulator so the race reached
`ready`. D2 drove normally. D1 briefly reached about 0.44 m/s and then stayed
at zero. D1 made 452 connector claims, consumed 395 results and admitted 37
plans to the Store, but normal authority was not sustained.

Near the terminal failure, both the normal raw QP and the connector's direct
time-aligned suffix QP rejected at `steering-rate-prefix` around stage 5--7.
OSQP reached 4000 iterations with small primal violations but unresolved dual
residuals. Repeating the same formulation through a single-owner connector did
not restore production authority.

At the same snapshots, the existing observation-only latest-state physical
projection reported an available connected continuation and accepted roughly
59--65 of 80--81 proof samples. That is evidence that a connection often
exists, but the direct retained suffix is a poor SQP tangent/candidate.

## Classification

Dynamic acceptance failed. The Slice is rejected and no production connector
code is retained.

The evidence changes the next comparison arm from lifecycle scheduling to
candidate generation:

```text
latest exact state
  + old suffix states/inputs joined directly
  -> discontinuous tangent
  -> steering-rate-prefix numerical rejection

latest exact state
  + bounded reachable bridge
  + surviving semantic suffix
  -> same seven-state refinement and unchanged proof chain
```

This is currently classified as a candidate-generation defect. It is not
evidence for changing solver tolerances, wall clearance, steering limits,
leases, retries or authority rules.

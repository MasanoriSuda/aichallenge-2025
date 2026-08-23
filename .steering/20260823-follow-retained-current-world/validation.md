# Follow retained current-world validation

## Static result

- The focused current-world retained suite passes all 6 cases.
- After rebuilding every package test target, CTest passes all 38 tests.
- `make autoware-build` passes all 25 packages.
- After the final helper-name cleanup, the `mpc_controller_cpp` target recompiles and links.
- `git diff --check` passes.

## Covered evidence

The pure tests prove that a retained Follow plan is accepted only when the current target identity,
observation generation, deterministic target-tube fingerprint, complete target forecast, physical
hard gap and wall path all agree. Target substitution, tube mutation, malformed target progress,
current hard-gap violation and future-stage hard-gap violation fail closed.

Runtime code stores a fresh Follow plan only after the complete canonical command chain succeeds.
When a later eligible cycle has no fresh canonical command, the prior store is evaluated through the
typed current-world proof, canonical retained-candidate builder, normal-authority selector and exact
actuation/prediction reconstruction. The result remains `authority=shadow, selected=0`.

## Commands

```bash
make autoware-build

docker compose run -T --rm --no-deps autoware-build bash -lc \
  'cmake --build /aichallenge/build/multi_purpose_mpc_ros -j2 --target <all-test-targets> && \
   ctest --test-dir /aichallenge/build/multi_purpose_mpc_ros --output-on-failure'

docker compose run -T --rm --no-deps autoware-build bash -lc \
  'cmake --build /aichallenge/build/multi_purpose_mpc_ros --target mpc_controller_cpp -j2'
```

## Dynamic gate

Pending. A useful run must contain all of the following in order:

1. a complete fresh Follow canonical plan is stored;
2. a later eligible Follow cycle has a typed fresh canonical miss;
3. the same target is still current and its complete forecast covers the retained remainder;
4. `Follow retained MPCC shadow` reports `command > 0` and `selected=0`.

If the run contains no such fresh-miss event, it is evidence of neither acceptance nor rejection and
will be recorded as inconclusive. Follow production authority is not promoted by this Slice.

### Attempt `output/20260823-175836`

Result: infrastructure-inconclusive; it is not a Follow retained rejection.

- `make dev2` started AWSIM and both Autoware domains.
- Both Domain 1 and Domain 2 exposed an `awsim_dN` publisher for `/awsim/state` and accepted the
  normal initial-pose/control-mode requests.
- Both orchestrators remained in `WAIT_START` waiting for `Grounded, Ready, Start`.
- Domain 1 observed only `state=spawned`, received one odometry recovery, and returned to stale
  odometry after 0.5 seconds. Domain 2 observed no AWSIM state or odometry recovery.
- Neither log entered `race session changed: active=1`; no Follow fresh or retained shadow event was
  generated.

The containers were stopped with `make down`. This run cannot satisfy or falsify the dynamic gate.
The next valid run must first demonstrate a normally advancing AWSIM race before its Follow evidence
is interpreted.

### Accepted attempt `output/20260823-181103`

Before this rerun, `make autoware-build` installed the committed runtime into the workspace used by
`make dev2`. Both vehicles then advanced through `grounded`, `ready` and `start`, so this run is
valid dynamic evidence.

Domain 1 produced the required causal sequence:

1. decision 2865 completed and stored a fresh canonical Follow plan;
2. later fresh solves became unavailable or failed their physical certificate;
3. the retained evaluator used a current observation of the same `d2` target;
4. retained world proof, candidate, selector, actuation and command all completed;
5. every result remained `authority=shadow, selected=0`.

The first retained telemetry window reports:

```text
attempted=8, world=5, candidate=5, selector=5,
actuation=5, command=5, fresh_stored=32,
world_reason=accepted, proof_reason=accepted,
selector_reason=retained-certified/retained-certified,
min_gap=2.633 m, authority=shadow, selected=0
```

A later window supplied one further complete retained command. Fail-closed outcomes were also
observed: course-frame unavailable, progress discontinuity, no stored plan and certificate expiry.
None was converted into an age lease, legacy fallback or executable shadow command.

## Gate result

Static proof and the requested dynamic retained-selection event pass. This validates the retained
Follow mechanism in shadow; it does **not** approve production Follow authority. The same run also
contains long fresh-solver-unavailable intervals after the retained certificate expired, plus
control callback overruns. Production promotion therefore requires a separate coverage/latency
gate and must not be inferred from this Slice.

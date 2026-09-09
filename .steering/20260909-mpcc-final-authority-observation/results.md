# Final-authority observation results

Control baseline and rollback: `e5b8c455`. This slice repairs observation ownership;
it does not change a solver, objective, limit, physical proof or publisher.
All simulators and diagnostic containers are stopped. Protected user JSON files
were restored and verified. Full MPCC acceptance remains open.

## Demonstrated recording defect and repair

Old D1 terminal949 recorded normal365, but final951/Stop379 was missing.
The controller suppressed the final category after submitting a terminal job;
the later terminal record was then deduplicated against949. Native recorder calls
confirm this bucket behavior, and a blocked LatestOnlyWorker deterministically
replaces the first accepted pending final job. These observations do not diagnose
the old control failure or reconstruct missing951/379data.

An isolated FirstAuthorityFailureRecorder now reserves finite intent/side/boundary
buckets at enqueue time and drains distinct events in FIFO order. Terminal and
final admission boundaries are both observed. Planning workers are unchanged.
The exact rejected retained::evaluate Request and its inspected source/artifact
are recorded separately from the latest publication. Invalid/missing supplemental
evidence is explicit and does not discard a valid current world. Grid admission
validation still executes on submit; full interaction hashing, serialization and
I/O execute on the private worker. No new control authority is introduced.

## Verification

- Native scheduling probe reproduces both old observation primitives.
- 16snapshot native cases and104source-contract cases pass.
- Final `make autoware-build`:25packages in3min41s, no compiler warning match.
- Package `colcon test --packages-select multi_purpose_mpc_ros` and scoped
  `colcon test-result --verbose`:60CTestgroups/2371records,0errors/failures/skips.
- Actual old365original artifact passes original wall/dynamic proof with zero
  solves (minimum peer clearance0.2508901557m, terminal3.5688619844m/s, not rest).
- New exact945/361and later1137/575replays reproduce retained rejection
  without solving. Helper r1 had only missing retained-library link dependencies;
  r2 linked the existing libraries. Both attempts are preserved.
- Difference review confirms no changed ROS/evaluation/submission contract;
  invalid supplemental association and artifact checks remain strict. The native
  blocked-worker fixture verifies off-callback I/O and retained first ownership.

`git diff --check`, helper Python AST, registry uniqueness/references and local
Markdown links pass. The host has no `pre-commit` command (exit127); equivalent
conflict/private-key/newline checks pass. No hook installation was required.

## Fixed dev2 observation, not race acceptance

Run `20260909-final-authority-dev2-r1`, standard6laps/600s simulator settings,
stop at first active moving override. No finished result or lap is present.
The first moving Emergency is D1decision945 at wall1788919066.858519760,
exact measured speed1.5802064876844846m/s. Its first final snapshot is present:
worldfingerprint7865628471136372292, YAML SHA256
`d540fc8a61b8f4c85a422910990b71ab361519d8155734e8798619d6d5b79534`.
Both actual published and inspected sequence are361. The full rejected Request,
own grids and publication metadata are present. Recording completed asynchronously.
This meets the first-moving-final observation criterion. Old missing379is not
silently replaced by this new artifact. Separate1137/575is later than the first Emergency, before shutdown began at
1788919073.4911518; it is a downstream observation, not the earliest failure.

Zero-solve replay reproduces `steering-unreachable`: previous command
-0.042629476636648178rad, expected-0.067140458958915808rad,
reachable interval[-0.064547900780632009,-0.020711052492664347]rad over0.02s.
Measured physical steering is+0.051989892216781061rad. PublishedPlan cursor is
0.8239746129003791s; predicted speed2.1972726771742126m/s versus control-origin
1.6756152093457606m/s; pose error0.2119030691m. Original361wall/dynamic pass
(minimum0.2980002972m); terminal3.5130026654m/s is not rest.
Current continuation wall is clear. Terminal current-peer proof fails againstd2
at-0.003647424709m; independent Stop is wall-clear and peer-blocked at
-0.001124094799m. The correct rejection has now been reproduced; the upstream
cause of the lost viable candidate is not yet established.

D1/D2reported callback windows9each,364/365cycles, maxima18.836/21.067ms and
zero overruns. Command receipt is40Hz with no gap>50ms. Source timestamps and
odometry/clock delivery still contain pauses; see sealed topic timing JSON.
No timing pause is attributed to945without its paired timeline. No fresh single
run was required by this observation-only slice; the prior baseline single pass
is historical evidence, not a fresh run of these binaries.

## Next causal work and remaining acceptance

Use exact945/361to align publisher clock/reachability with serialized commands,
then compare normal A/B/C/D and complete-rest candidate formulations on this
sealed world. Preserve the previous923/1629positive witnesses and4264Unknown.
Do not promote an offline candidate or relax rate/peer proof from this result.
The current alternate Stop371was rejected, not the inspected retained361;
its artifact is not supplied by this observation. Capture it only if required
by an unresolved causal boundary rather than substituting a new solve.
After causal repair: fresh coupled race, moving multi-tick Stop/restart,
Follow/both Pass/Return directions, Recovery/Rejoin/Boost/async, dev3/dev4,
gate1/2/3 and artifact-linked submission evaluation remain open.

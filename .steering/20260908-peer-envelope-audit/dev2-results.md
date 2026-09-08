# Complete peer body: local validation and first coupled trial

Baseline `5cfc50bc685d9ce52c489d7d9962e42ae43ec9c5` plus the peer producer patch.
This is a rejected integrated trial. Complete-body enclosure passes locally;
coupled race feasibility and the overall completion plan remain open.

## Producer repair

The four local AWSIM bodies share the extracted mesh. Their maximum distance
from their own V2X/GNSS origin is1.8754249470067432m. The independently extracted
202-vertex convex hull retains the support of all705source vertices. Both YAML
configurations declare nominal peer radius1.876m. Existing uncertainty is added
once. Ego footprint support is added by the verifier, not subtracted from the
peer radius. Recovery now uses that same nominal body source.

The package witness has an actual body overlap at relative ego base(2.2,0).
Before repair its old nominal circle plus uncertainty reports+0.822mclearance;
the new regression fails. After repair the complete circle rejects the overlap.
`make autoware-build`:25packages pass. Container `colcon test --packages-select
multi_purpose_mpc_ros` and `colcon test-result --verbose`:60CTestgroups,
2338colconrecords,0errors/failures/skips. Logs are retained under
`output/20260909-peer-envelope-validation/`. No C++ compiler warnings.

## Coupled trial

`output/20260909-peer-envelope-dev2-r1` uses actual `make dev2`, standard spawn
and Domains0/1/2, six-lap/600s generated dev overlay. Source/config/binaries,
image IDs and patch are preserved in its manifest. D1 and D2 each finish only
one lap before deliberate interruption; both `finished=false`. D1lap128.895157s,
D2lap124.181389s. Domain result-details files are absent, so penalty acceptance
is unavailable. Callback D1max26.284ms/1overrun, D2max20.811ms/0overruns.

D1 first moving Emergency is decision941/942 in AWSIMReady at1.58/1.62m/s.
D1later1609/1610/1611loses normal authority at5.49/5.48m/s. D2first moving
Emergency1552is4.45m/s. Final Recovery overrides occur independently of the
tactical phase, which staysIdle. The monitor watched only tactical
`OvertakeLine -> Recovery` and missed these overrides. An explicitSIGINT
stopped the owned monitor and its normal cleanup removed only its three
Compose projects. The historical `840s host deadline` label is incorrect on
that interruption; `external-stop-reason.json` records the actual cause.
Preserve both records. No unchanged rerun is justified.

The run-generated shared result JSON is preserved separately. The preexisting
user result-summary and safety-gate JSON bytes were restored/verified, with
`workspace-artifact-restoration.json` recording the hashes. Neither belongs
in the peer repair commit.

## Earliest causal boundary

At D1decision941, observation10.094999774s/control origin10.224999774s,
retained source394has a valid27-point delay path and clear36-point normal
continuation. Its terminal Stop dynamic proof rejectsD2 with
clearance−0.000823947m. Wall proof is not attempted after that rejection;
the default `invalid_rollout` field is not evidence of a wall collision.
This source used the complete D2circle radius1.9309999998882412m.

Yet solver sources394/396and the current941snapshot have
`dynamic_obstacle_refinement_active=false`, `dynamic_obstacle_stages=[]`.
The optimizer therefore never receives the peer checked by final physical
proof. The producer selects a side target but calls
`set_target_execution_prediction(has_front_vehicle, ...)`; a side-only
observation cannot populate the current target tube. This is a demonstrated
producer omission, not evidence that the full body is physically infeasible.

Sealed original-source comparison gives:

| Source | A: same seven-state SQP | B/C/D/G alternatives |
|---|---|---|
|394|solved; D2dynamic rejection at0.577741s,−0.00104195m|unevaluated: canonical target tube absent|
|396|solved; D2dynamic rejection at0.458410s,−0.000959349m|unevaluated: canonical target tube absent|
|941|post-refinement physical adapter rejects stage0|unevaluated: canonical target tube absent|

These outcomes do not prove physical impossibility. Compare a same-world
target producer before choosing a production repair. New candidate identities
must be resealed; original snapshots remain immutable.

Source396also has certified raw primal initialprogress−2.1804138714481612e−9m
while semantic initialprogress is0. Its course window begins exactly at the
origin, and the new sampler accepts only1e−9moutside support. This is a
separate initial-state hypothesis requiring direct native transition evidence.
The initial external-primal oracle experiment does not test this hypothesis:
that oracle already initializes physical integration from the semantic source,
so changing primalx0leaves its output unchanged. Both variants still reject
D2dynamically. Do not mislabel that result as a successful physical repair.

The subsequent native probe in`world-target-comparison-r2/396/report.yaml`
does test the production transition directly: the raw initial course sample
and first physical step both fail; the semantic initial sample and identical
first control step both pass. Its producer repair is tracked separately in
`../20260909-mpcc-semantic-physical-initial/`.

Same-world target tubes were also generated using the physical predicate's
constant-velocity peer law, semantic absolute arrival times and original
course projection. Full circle, ego body, wall, actuator and bounds stay fixed.
New interaction fingerprints:394→7517972879573529579,
396→15271378681343111376,941→3716950170988289801. Existing A/B/C/D/G
architectures now receive the peer but all reject dynamic QP rows at4000
iterations. For396A the active stage7lateral row requires0.440038mwhile its
failed iterate gives0.356407m. This rejects that candidate comparison, not
all possible motion. The next comparison must consider complete physical
corner/stay-behind/Stop homotopies; repeating the same rows is unjustified.

After the independent semantic-initial repair, a native full-circle maximum-
braking Stop with historical objective and complete physical corner support
passes the same world's dynamic/wall/terminal proof. Current zero-objective
Stop still fails. See[semantic-initial results](../20260909-mpcc-semantic-physical-initial/results.md).
The complete envelope therefore does not make every motion in this scene
impossible; preserve the successful controls and isolate support/conditioning.

## Remaining acceptance

Resolve producer omission and initial-state boundary with fixed-input evidence,
then focused/package regression, build and bounded coupled acceptance. Preserve
full physical geometry, hard reserves, semantic clock and single normal
authority. Full left/right passing, Stop/restart, async, dev3/dev4, gates and
submission evaluation remain unaccepted.

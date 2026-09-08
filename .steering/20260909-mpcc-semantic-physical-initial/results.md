# Semantic physical initial-state result

Validation baseline `5cfc50bc` plus complete peer-body and semantic-initial
patches. Local initial-state acceptance passes; coupled race acceptance is open.

## Change and verification

The required `ExecutionArtifact::semantic_initial_state` contains all seven
immutable problem initial coordinates. The unmodified affine prediction array
keeps its numerical dynamics certificate. Physical replay, retained sampling,
on-trajectory joining and terminal Stop use the semantic field. Original solve,
latest-state feedback, Stop successor and external-primal comparison populate
it. Missing/nonfinite fields reject; steering must match the command origin.
Certified-plan and publication-evidence boundaries reject a solver-source
mismatch. No new authority or raw-primal fallback exists. Evidence records both
the physical seed and raw prediction array.

The original strengthened solve/feedback tests fail 2 of 3 before repair.
The first implementation overwrote affine state0 and failed 35 of 2338 test
records; it was rejected and removed. Its logs and test executable remain in
`output/20260909-semantic-initial-rejected-overwrite/`.
The separate-field implementation passes 132 focused tests in four groups.
Final `make autoware-build`: 25 packages pass. Container `colcon test
--packages-select multi_purpose_mpc_ros` and `colcon test-result --verbose`:
60 CTest groups / 2339 records, 0 errors/failures/skips. Only existing setuptools
deprecation output; no C++ compiler warnings. The source396 regression preserves
raw progress −2.1804138714481612e−9 m and accepts physical replay from semantic0;
missing and actually out-of-course semantic seeds remain rejected.

Original941 replay previously rejected physical stage0 after three corrections.
It now solves and reaches independent D2 dynamic proof, which rejects at
0.468363 s / −0.00494577 m. The source still lacks peer constraints. This
repairs the initial-state producer, not the missing-peer problem.

## Dynamic single-vehicle acceptance

`output/20260909-semantic-initial-single-r1`, Domain1, standard single eval
launch, one predeclared trial with sealed source/config/binaries/image.
Same-run result-details: six laps 253.94403076171875 s, penalty0, finishedtrue.
Active Ready/Start moving Emergency/Recovery0; no active Stuck recovery.
Callback 13093 reported cycles, max18.913 ms, 0 overruns. Bag has 10650 commands:
receipt mean24.996694 ms, p9526.423216 ms, p9927.314577 ms, max35.070181 ms;
no nonpositive receipt gap and no receipt gap above50 ms.

The runner's state regex retained ANSI reset codes. An independent lightweight
monitor normalized them during the trial, identified only owned runnerPID241726,
and confirmed no active override before finish. Its result and original runner
are preserved. The future helper now normalizes ANSI before parsing; no running
production code was edited.

ROS source time has22 duplicates and4 gaps above50 ms (max115 ms), all inStart,
around source16.8 s and136.7 s. The same bag shows clock/sensor publication
pauses: the largest clock receipt pause is298.415 ms while simulation advances
5 ms; odom304.736 ms/70 ms and velocity316.227 ms/35 ms occur in that window.
The second cluster includes92.772 ms and189.689 ms clock pauses, each advancing
5 ms simulation. Command receipts continue at the periods above. The underlying
simulator/publication pause cause is unassigned; do not label it a controller
receive gap or claim all source timestamps are unique.

Both preexisting user result JSONs remain byte-identical to their pre-run hashes;
the verification receipt records this. Run outputs stay uncommitted. This run
does not provide moving certified Stop or multivehicle coverage.

## Coupled feasibility comparison

The resealed full-circle396 world has a native certified maximum-braking Stop
witness with inherited objective plus complete physical corner support:
comparison fingerprint9052539256127508737, terminal velocity1.62878e−9 m/s,
reported lateral reserve0.0791488 m, bundle accepted. This is an offline proof
without publisher/store authority. At least one modeled safe Stop exists with
the complete peer body.

Current zero-objective Stop fails its dynamic QP at4000 iterations; the ordinary
axis branch also fails. Outer physical SQP cannot repair the first failed dynamic
solve. Objective-dependent tangent/support selection and numerical conditioning
must be compared on the actual fixed QP. Do not promote the observation arm or
change weights/tolerances merely because one comparison passed.

Next: preserve exact successful Stop controls and the failed zero-objective QP;
isolate conditioning versus physical separating planes; repair the demonstrated
producer and complete side-peer inputs. Then resume bounded dev2 and P3/P4.

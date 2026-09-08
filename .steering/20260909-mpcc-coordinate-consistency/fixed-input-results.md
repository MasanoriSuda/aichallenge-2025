# Coordinate repair: local validation and fixed inputs

Baseline d54822c982c705098cc2fbfab4160f15e41865b0 plus the preserved source patch.
This is a local repair, not completed integrated acceptance.

`make autoware-build` passes all25packages. The first focused invocation failed
because Compose's nested quoting treated the regex pipe as shell syntax; no
test result is inferred from that invocation. The corrected full package command
passes all60CTest groups. `colcon test-result --test-result-base
build/multi_purpose_mpc_ros --verbose` reports2333records with0errors/failures/skips.
Logs are preserved under `output/20260909-coordinate-native/`.
One compiler warning identified a test-only aggregate missing the new optional
course field. The later lineage slice includes it explicitly; no C++ compiler
warning remains in that build.

The native analytical invariance probe now passes24/24cases, previously14failures.
Maximum position error is3.51e-16m. New package tests also cover the actual
source6782first segment, piecewise knot crossings, virtualspeed changes,
finite Jacobians, missing/out-of-window geometry, and old model rejection.

`compare_saved.cpp` uses the real production loader and model. It preserves old
source fingerprints, changes only the model schema, reseals the problem and
interaction identities, discards old QP/warm starts, and saves a new candidate
input. Its separate fixed-control integration is explicitly conditional and
has no execution certificate. Original source YAML remains unchanged.

| Input | Original interaction | New model interaction | Result |
|---|---|---|---|
|6782|5246716225714339212|3065195133545278477|conditional motion agrees with independent Cartesian integration; new wall-refined solve rejected|
|7405|14329803062406037027|1093205162327722224|new wall-refined solve rejected|

Source6782SHA256 remains
`de387af1db6704d89b3d0218d949f592d7ec73faa81af9c5d21e4c5041fa2d47`.
Maximum position difference against the independently integrated Cartesian ODE:
0.012789mm through55ms,0.022522mm through100ms,0.036545mm through200ms,
and0.231333mm over the conditional1.794944s horizon. Previously the full-horizon
difference was651.9642mm. The remaining integration error is not a field
calibration result or a proof of real vehicle motion.

For both sources, armAand bounded wall-restorationHreach the wall-refined QP,
then hit the unchanged4000iterationlimit. Reported violated wall rows are
378and376respectively. No solved/certified bundle is produced. B/C/D/G
overtake generators reject absent peer target inputs; they do not test alternate
single-vehicle architectures. All-method physical infeasibility remains Unknown.
The older conditional Cartesian path already entered the hard wall reserve at
4.512ms, so a newly rejected solve from that state does not disprove the model
repair. Do not claim this observation establishes physical impossibility.

Raw comparisons: `output/20260909-coordinate-fixed-input/`.
The initial bounded single-vehicle run passes; see [single-r1-results.md](single-r1-results.md).
The following geometry-lineage slice reproduces two additional refusal gaps
before fixing them: recorder and certified-plan assembly accepted missing or
changed model geometry. Both now require matching course origin and allknots
for world-backed sources. Normal/continuation/Stop binding is also tested with
a physically straight actuator command and a changing virtual reference.
Final25-package build and60CTestgroups/2335colconrecords pass,0errors/failures/skips.
All evidence is sealed in [evidence.json](evidence.json). Attribution of the
original live delay-prefix failure beyond the proved coordinate defect remains
incomplete; no delay/actuator retuning was applied.

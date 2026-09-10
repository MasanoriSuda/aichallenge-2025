# Published Stop schedule closure

Authorization: existing autonomous M1–M6 implementation, simulation and local-commit scope. No new approval gate.

## Earliest invariant and failing evidence

HEAD e710056d, dev2-r13 D2 first moving Emergency decision 1401 during ShiftOut; source 828 was a complete Stop published on 1400. No fresh pose/velocity/IMU/tire observation separates the captured 1399 and 1401 inputs. 1399 is historical, not the missing immediate accepted 1400 revalidation.

The saved Stop is wall-clear at its initial model state. `stop_recursion.cpp` with the current production archives proves that exactly predicted bodies at 25/35/50/75/100/150 ms have zero pose mismatch but lose the wall/full-rest obligation. The deliberate, non-publishable continuous-rate counterfactual changes only the first-publication hold in continuation: all eight 0–200 ms samples pass the complete original suffix. This isolates a certificate/execution schedule inconsistency independently of actual transport or empirical model error.

Producer: `build_stop_contingency` integrates continuous future steering rates after the first held publication. Materialization preserves that rate sequence. Consumer: `build_continuation` holds the sampled angle for a new first publisher interval. Every reuse inserts a pause into the certified steering sequence. The adapter/normal Stop fallback subsequently rejects wall contact; Emergency detects the failure correctly. Removing the publication hold is not a valid production fix.

Actual publication was 35 ms later than its nominal decision time. Coherently moving just that history event back, including the exact waypoint 59 tangent progress projection, still rejects the current-world continuation and fresh Stop. Publication/application timing therefore remains a contributor/open contract, not the sole cause.

## Architecture gate

Full A–G comparison timed out at 180 s before emitting results: inconclusive. The replacement observation streams individual results and fixes six coarse C/D schedules (three per side) before running. A and left B solve then fail the hard wall proof; right B and all six C plus six bounded D candidates reject in the unchanged solver. Neither this nor two failed complete-rest QPs establishes physical infeasibility. No weights, limits, iterations, world or production authority changed.

## Proposed producer repair

Generate terminal Stop as actual float32 steering-angle publications held for each declared publication interval. Preserve nine body/Frenet state coordinates, the same native body model, physical rate/angle/acceleration bounds, full-rest endpoint, exact wall and peer tests, and final packet binding. The first command remains the already-proposed packet; subsequent bounded steering increments become serialized angle changes at publication boundaries.

Use a distinct sealed input-schema marker for materialized Stop schedules. Its stage rates encode command increments divided by the publication interval; physical integration holds the stage endpoint steering angle. Artifact validation, actuation extraction, exact rollout and retained continuation must agree on this representation. Older captured continuous artifacts retain their recorded meaning. No legacy/fallback normal publisher is added.

Delete the continuous future-rate interpretation from the terminal Stop producer. Keep the ordinary SQP representation and its strict publication checks; this slice does not claim all future normal packets or simulator receipt times are known.

## Acceptance and rollback

Before/after native test: each Stop publisher interval has exactly one serialized float32 steering value, rate-limited boundary increments, full body rest, same bundle/extraction/rollout and current-world continuation at publication boundaries. Reject altered schema, invalid bounds and changed wall/peer context. Replay r13 fresh Stop and synthetic closure; retain r12 peer rejection unless the actual corrected packet schedule changes its exact proof. Then build, package tests, live dev2, remaining M4–M6. Record failures without changing acceptance criteria.

Rollback: e710056d (or revert the eventual dedicated Stop-publication commit). No existing user output is reverted or committed.

## Local result (not integrated acceptance)

Materialized Stop uses the sealed input schema `accel-published-steering-increment-progress-rate-v1`. Stage endpoint steering is the held serialized angle; stage rate is its bounded increment divided by duration. No separate execution-mode flag can mutate that meaning outside the problem fingerprint. All generated Stop samples have one angle per interval; bundle construction rejects a changing angle inside an interval.

Before: regression fails at the second 5 ms sample of publisher interval 1 (0.110 vs held 0.105). Initial ad-hoc test compilation used the host gtest headers and failed to link; retry with the repository gtest vendor headers establishes the actual pre-fix test failure. Production buildr30 passes 26 packages/4m49s. Testsr25 has 2409 records and two actual test failures (four colcon failure records including aggregates): a final-angle monotonicity assumption is false after the physical schedule change, and removing terminal-rest permission now rejects earlier as InvalidCertificate. Test checks were updated to require the first corrective turn toward the line, and the exact earlier rejection classification; all six negative rest mutations remain rejected. Buildr31 passes 26 packages/14.6s; testsr26 all2409 records/62groups pass in28.69s.

Toolr37: r13 historical1399/source821 own Stop prefix and exact waypoint59 tangent projection produce a serialized Stop with330samples, materialization/rejoin accepted; model closure0/25/35/50/75/100/150/200ms is full wall/peer/rest suffix in every case. The later exact1401/source828 historical body still rejects wall contact with335samples and no bundle. Do not claim this repairs that already-recorded failure state or proves infeasibility. R12 exact932/source424 accepts its new Stop with+0.012870124661m;933 remains dynamic-path-blocked at-0.000891864256m with115samples/no bundle. At synthetic932+35ms a different certified terminal successor is used; all other sampled times preserve the solved suffix. These are model closure observations, not actual motion or exact1400 predecessor capture.

The full-comparison compiler's generic manifest text incorrectly says zero solves for toolr36; bounded_architecture.cpp and its streamed output explicitly invoke A/B/C/D solves. This labeling error has been corrected for future compiler manifests; the original recorded manifest is preserved with this correction attached. The native counterfactual has no production promotion path.

Next: local commit, fresh dev2-r14 at that commit/config; retain publication/application uncertainty and all remaining M4–M6 acceptance.

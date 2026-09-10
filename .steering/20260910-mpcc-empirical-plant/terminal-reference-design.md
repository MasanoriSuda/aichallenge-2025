# Make the terminal reference cover its complete stopping obligation

Baseline `ae2137c6`, frozen `20260910-nine-state-dev2-r11`. D2 completes negative
ShiftOut→Pass→Return→Idle. D1 first moving Emergency2588 at
wall1789036269.124476253, 8.76m/s, is preserved with exact Accepted2587 and
rejected2588 Requests, both source2047. The new moving recorder works. This
run remains failed; these observations do not close M4–M6.

Earliest broken invariant: a terminal-reference generator uses the end of the
normal solved horizon as the end of the stopping obligation, although the
immutable Stop map already extends farther. `build_normal_path_stop_profile`
copies the solved lateral profile through local progress15.34538226077145;
the physical Stop geometry extends through43.73213889153885. Course origin is
331.07877539677395. At2588 the normal-reference Stop still moves when its query
crosses absolute346.4241576575454. It returns InvalidLateralPolicy before the
wall check. The subsequent independent centerline hypothesis hits the wall,
masking the earlier reference-domain failure in the final reason.

Evidence: native toolr25/stop-reference-r1 reproduces Accepted2587 normal
695samples, while2588 normal aborts after650samples. Both centerline attempts
hit the wall. Same-world A solves15.7801ms then centerline terminal wall rejects;
B/C/D/G cannot form their target tube and remain inconclusive. Both full-rest
SQP clocks reject at4000iterations. This is not a physical infeasibility proof.

One-dimensional candidate comparison (toolr26/stop-reference-r2): preserve
every solved profile knot, append a constant-lateral course tail at the sealed
Stop geometry endpoint. This is an explicitly generated terminal reference,
not a claim that the solver produced the tail. Normal source, first packet,
body inputs, peers, grid, footprint, model, budgets and all proofs stay fixed.
2587 is numerically unchanged. 2588 completes700samples/3.50s with u/vy/r all
zero and full wall/peer proof accepted. Final progress346.52611284969021 is
0.101955192m beyond the old profile. At first crossing (3.205s), speed is
0.677543395m/s. No tolerance, clamp, timer or safety margin is changed.

Repair `build_terminal_stop_reference(execution, geometry)` as the explicit
composition producer. Keep the existing solved-profile copier and strict
sampler intact for exact-source diagnostics. The new producer appends only
inside the already sealed Stop map, preserving the solved prefix and endpoint
lateral value. Production retained proof and the normal-reference architecture
arm consume the same composition. The existing solved-only observation keeps
its original domain and explicitly reports that profile's failure. Remove the
old use of a short solved-only profile as the full terminal reference.

Every generated tail still receives the native complete-rest, full swept wall,
peer and packet proof. A missing map, query beyond declared support, collision
or invalid actuation still rejects. An ordinary command does not gain a second
authority; a selected terminal Stop still materializes the exact certified
input sequence and rejoins it before publication. The independent centerline
hypothesis remains subject to its complete proof.

Validation: old implementation fails the new native regression on reference
ownership/attempt count. Add producer boundary checks; run package build/tests,
exact2587/2588 production replay and a fresh fixed-commit dev2 attempt. Changed
profiles do not justify accepting r10/r11 or reusing earlier race passes.
Rollback is this slice's local commit based onae2137c6. The r10 backdated
publication history and isolated/adjacent timing costs remain separate open
issues until causal repairs and integrated timing acceptance establish closure.

Buildr26 passes26packages/4min48s. Testsr20 runs2404entries; the two added
regressions pass, but one existing diagnostic regression fails (reported twice
by colcon). It detects that the solved-only observation was also changed to the
composed reference. Restore that diagnostic's original strict profile; update
only its expected production result to the explicitly generated tail. Keep its
InvalidLateralPolicy and before/after immutability assertions. Rebuild and rerun
the package; the failed preparation remains evidence, not a passing check.

Final local validation: buildr27 passes26packages in17.0s; testsr21 passes2404
records/62groups/0errors/failures/skips in28.78s. Toolr27 confirms production
native2587/2588Accepted; toolr28 also builds actualproductionadapter authority
and checks its serialized acceleration/steering against each exact proposed
packet. Original2588packet remains1.3295959593141273m/s² and
0.18719810862825503physical steering rad. Native1668fromr10 still rejects
peer under originalnormal/track/composedreference, so this is a separate failure
fromr11. Its productionadapter remains unavailable. The independently rebound
braking proposal's positiveclearance is not claimed as actual1668latticepacket.

R11fullmetrics include postfailure/shutdown; beforefirstfailure, onlyisolated
observed overruns occur (D1two/D2four, eachincludingstartup). Lateradjacent
pairs cannot be assigned as precursor causes. Freshdynamic acceptance and
allM4–M6 conditions remain required; no historicalattempt is relabelledpass.

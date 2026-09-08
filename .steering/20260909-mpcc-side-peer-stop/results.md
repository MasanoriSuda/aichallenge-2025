# Fixed-source findings

Baseline b530883997e80cd78aa7e70bcd53bf2b6bcf93fc. No dynamic run has been
repeated. Source is the resealed full-body world396 from the rejected dev2 run.
All findings below are offline; full MPCC acceptance remains open.

## Independent defects and comparisons

The zero-objective maximum-braking Stop now has its own exact failed QP:
`output/20260909-side-peer-stop-audit/zero/mpcc_architecture_snapshots/`.
Its interaction fingerprint is5013267340109276277. A separate process preserves
the historical accepted witness, full-precision primal, raw states and controls.
Neither artifact is labelled as published or sent to an authority store.

| Comparison | Result |
|---|---|
| Same QP, normalized cold / warm |25iterations solved /4000iterations rejected|
| Same QP, internal equilibration cold / warm |50solved /4000rejected|
| Converged original-geometry candidates |Exact dynamic proof rejects; numerical convergence alone is insufficient|
| Common Cartesian supporting plane, zero objective |All4numerical arms still reject|
| Existing hard-equality residual objective, original rows |All4solve, all4fail dynamic proof, including8bounded SQP steps|
| Equality-equivalent objective and common Cartesian plane |All4pass the native solved-Stop proof at first QP,75/100iterations|

For the accepted combined arm, candidate QP fingerprint is17755198930602195165.
Minimum dynamic clearances by normalized cold/warm, equilibrated cold/warm:
0.0067560893,0.0085273973,0.0060510152,0.0066507330m. Native wall proof,
row certificate, exact actuator rollout, rest boundary and certified-plan builder
all pass. The actual solved-Stop rest tolerance is approximately0.003959596m/s;
the resulting terminal velocities are between-4.72e-9and4.47e-8m/s. No native
tolerance was changed. These are single-source feasibility witnesses, not a
robustness margin or runtime timing benchmark.

The generic external-primal comparator asks for another racing-line Stop even
when its input is already a full solved Stop. Its repeated terminal failure
(-0.00313693m) therefore does not evaluate this candidate's actual contingency.
The later `native-stop-proof-r2` uses the existing solved-Stop contract directly.
The first native-stop-proof helper failed to link the certified-plan library;
the link dependency was added and that build log is retained.

## Earliest coordinate inconsistency

The dynamic-row producer subtracts a target's own course progress/lateral from
an ego pose expressed in a different course frame. On the saved affine witness,
relative Cartesian displacement differs by up to0.327005666m. At stage6,
the old geometric clearance is+0.006447637m while common-frame clearance is
-0.014206719m. Stage7similarlychanges+0.004920014to-0.017842788m.
This is separate from the earlier raw-primal initial-state issue and from the
missing side-target tube. The final physical verifier correctly rejects overlap.

The replacement plane uses the actual peer world point at the same semantic
time, ego Cartesian pose from the exact course frame, and asymmetric body support.
It differentiates lateral, lag, heading and virtual progress; a frozen heading
or treatingtheta+lagas global Cartesian distance cannot represent that plane.
The comparison retains the original stage durations, dynamics, wall rows, body,
uncertainty, actuator bounds and zero-objective feasible set.

## Numerical representation

For a zero objective, add `||S_eq(A_eq x - b_eq)||²` using only existing hard
equalities and their existing row normalization. Its value is identically zero
on the original exact feasible set. No racing preference or relaxed slack is
introduced. Keep the original QP for physical certification and return duals as
`y_original = y_augmented + 2 S_eq²(A_eq x-b_eq)`; invert this map on warm start.
The solver's recorded objective remains the original zero objective.

The frozen native regression fails both warm cases before the change. The solver
suite now passes 28 tests, including original dual transport and an inconsistent
hard-row case. Cartesian analytical gradients match independent finite differences.
The actual selected-target dispatch passes 6 native cases after the side-peer
producer repair; side-only failed before it.

Actual production libraries certify the current zero-objective Stop from the same
world396 source (approximately 8.2 ms offline). Normal A/B/C/D/G remain rejected.
The first normal A boundary was an unbounded numerical tangent: stage-one progress
was -2.215132146736506e-9 m even though its declared lower bound is zero. A native
test reproduces rejection. The existing adapter's bounded tangent selection is
now shared with the peer plane; raw primal, semantic physical x0 and frame-window
tolerance are unchanged. A semantically permitted but unavailable frame remains
invalid. Dynamic/problem/adapter/snapshot suites pass 28/17/17/11 tests.

The next A-only replay (`output/20260909-side-peer-normal-tangent-after`) reaches
the actual QP and rejects at 4000 iterations, primal residual 0.000281339, dual
residual 0.0851127. This is not a physical infeasibility certificate. Do not repeat
this unchanged arm. Full package build and changed-input dev2 remain unperformed.

## Current-world Stop availability

The controller independently filters Stop submission and adoption to ShiftOut/Pass,
and retains results only after a normal artifact has already published in that
tactical scope. The saved failing source is Cruise with intent generation zero
and an independently bound dynamic peer. Thus its certified Stop cannot enter
the existing current-world join. The native Stop certifier already supports this
source; numerical success cannot repair the omitted producer/adoption path.

Use the same seven-intent capability as canonical normal control. Candidate scope
belongs to the accepted immutable current-world submission, not to successful
normal publication. Compare intent/generation, tactical and dynamic target/side,
formulation/schema/horizon; observation epochs may advance only with the existing
current-world wall/peer/model/command proof before authority. Retire the published
scope updater. Stop publication and external overrides still invalidate pending
candidate work; certified execution remains in the single publication ledger.
Prove the missing intent and incompatible-scope cases before editing production.

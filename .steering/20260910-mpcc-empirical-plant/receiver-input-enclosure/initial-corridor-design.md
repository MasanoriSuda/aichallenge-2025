# Initial physical corridor at the semantic source pose

2026-09-12 JST. Baseline/rollback0bd08706. Authorized causal producer fix;
M4–M6 and race acceptance remain open.

The initial road-width entry feeds both semantic wall profiles and the terminal
Stop map. `mpc_controller_cpp.cpp:22388` currently obtains it from the nominal
waypoint with zero lag and a heading-bucket enclosure. The nine-state source is
instead projected from the model's actual temporal pose onto that fixed waypoint.
These are different physical queries. The artifact validator correctly rejects
an initial state outside the supplied corridor; changing its tolerance is not a fix.

Runr75 D1source747/decision1237/time19.624999561, geometry7297210173200458920:
semantic ey0.8717846581278238, lag−0.4015398691692103, heading0.11711843027052682.
Old initial upper0.8609313490270555 excludes it. Original actual body is clear.
R472 passes the original saved QP/primal/residuals through the actual production
artifact builder: old initial map rejects invalid-lateral-corridor, actual-pose
map accepts. Its original physical controls pass all external wall/peer/terminal
proofs in both cases, unchanged row residual2.5676097725746416e−7. Actual-pose
scan with original hard footprint/guard gives[-3.222320869958597,0.9794605575659187].
R470 generic external artifact creation bypasses this particular production map
assignment and accepts both variants; it proves physical feasibility only, not
production construction parity. R471 diagnostic compile failure is preserved;
R472 uses the correct identity.snapshot_sec field. No diagnostic publishes.

Change the initial width producer to use the same fixed-waypoint projection and
same conditional lag meaning as build_extended_progress_problem. Scan the actual
heading with existing find_clear_lateral_runs_with_heading, inside the existing
scalar map bounds, and select using the existing0.001m boundary guard. Retire the
initial cached nominal-pose call. Keep future profile construction, full exact
wall/peer/Stop/current proofs, control limits, solver policies and all timing gates.
No fixed steering preparation, new authority, fallback, retiming or relaxed test.

Validation: frozen cropped same-map native regression for actual versus cached
pose; source ownership assertion; focused and full package tests/build; exact
saved artifact replay; fresh standard dynamic run. Measure the extra uncached
initial query cost. R468's free-input native seed/pose-wall solver candidate is
separate diagnostic work, not part of this initial metadata correction.

r75 D2first active failure is separate: dispatch2274 immediately after AWSIM Start,
update_v_max invalidation (normal15→start20km/h), same geometry10523682073518823266,
source1784/job2259/index12. Prior2273publishes;2274context rejects before physics.
Later2285 publication deadline violation is not its cause. A current speed-limit
handoff repair needs its own evidence and cannot discard context validation.

- [x] Same original source/solved QP artifact failure reproduced with R472.
- [x] Replace initial physical-width producer; preserve all downstream validation.
- [x] Native473 same-grid crop regression passes; source106 ownership checks pass.
- [x] Build116 all26packages and package103 all2598records pass, zero failures/skips.
- [x] R474 current API query and original saved artifact/physical proof pass.
  Representative scalar bounds[-4,2] yield initial interval[-3.199,0.949].
  Query median0.607659ms, p950.611306ms, max0.659247ms over100unloaded calls;
  this is not a runtime bound or an end-to-end controller replay.
- [ ] Fresh standard dev2-r76 dynamic validation on the committed source.
- [ ] Continue Start speed handoff, future wall/candidate/timing and full M4–M6.

[r75 and initial-width validation evidence](r75-initial-corridor-evidence.json)
seals708 payloads including build116/package103 and R474.
The cropped fixture preserves the original grid coordinates and cell bytes;
all cells outside its captured region are Unknown. It demonstrates the earlier
bucket excludes x0 while the actual-pose query includes it with the same hard
footprint and0.001m guard. Full actual-world proof is separate R472 evidence.

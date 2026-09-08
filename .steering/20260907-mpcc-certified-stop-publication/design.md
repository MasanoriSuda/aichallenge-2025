# Earliest boundary

Run 20260907-mpcc-stop-feasibility-dev2, D1:
- decision 1904: certified ShiftOut artifact 1267 is published.
- decision 1905: retained terminal proof loses wall clearance; independently
  proved published Stop successor bundle 1284 joins with authority=certified-stop.
- decision 1906: source is missing; atomic admission treats previous Stop as
  external and emits canonical-normal-emergency-stop, not the certified suffix.
- decision 1957: actual footprint margin triggers Recovery (wall_contact=0).

The zero-objective Stop producer now certifies candidates, but no Stop-lattice
alternate is selected in this run. Do not attribute later wall-margin events to
execution of those unselected Stop candidates. This run does not establish race
acceptance or an actual physical wall collision.

Producer/consumer defect: rate_resolved_track_cruise_control gates plan recording
on normal_execution_evidence, which excludes published Stop. Independently,
record_final_published_authority clears the executed store for every Stop,
including a successfully serialized certified contingency. On the next tick,
previous_stop_authority short-circuits ordinary joining into the external Stop
branch. This discards a certified finite suffix merely because its authority
label is Stop. Wall Recovery and steering-unreachable are later consequences to
trace; they are not the first lifecycle boundary.

Planned repair: record certified Stop in the existing execution ledger after
exact serialization, distinguish that commit from external Stop when clearing,
and retain its Stop authority label when that exact executed artifact is joined
again. New normal candidate acceptance can replace it through the existing join.
Keep cancellation of secondary Stop generation after Stop publication. Do not
create a second live store, reset a publication clock, or label Stop as ordinary
normal tactical progress. Actual failed revalidation still goes to Emergency.

Reproduction: failing two-tick publication lifecycle test plus source deletion
wiring guard. Existing run has immutable source/world snapshot at decision1905;
its historical accepted artifact is not serialized, so no cold solver result is
claimed as replay of that live accepted bundle. Root contract is independently
reproduced at publication. Rollback: revert this lifecycle slice only, preserving
the previously accepted target-time and Stop-formulation work.

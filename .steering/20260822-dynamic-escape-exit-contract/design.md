# Design

## Observed failure

The 20260822-034808 run showed a Dynamic Escape exit followed immediately by a
racing-line horizon with zero wall clearance. Because the active wall admission
monitor only recognized ShiftOut/Pass/Return phases, Follow-owned Dynamic Escape
and its exit were outside the enforcement scope. The vehicle subsequently made
physical wall contact and entered Stuck Recovery.

## Change

1. Cache the most recent successfully solved, physically prevalidated Dynamic
   Escape control horizon.
2. Maintain a 0.35 s progress-contouring formulation lease after the last
   successful Dynamic Escape solve.
3. Add a dedicated Dynamic Escape wall admission gate:
   - monitor an active escape;
   - on exit, hold until two consecutive outgoing horizons are physically clear;
   - if the outgoing horizon is unsafe, restore the recent escape horizon and
     apply the existing non-accelerating wall hold;
   - if the active/retained escape is wall-invalid, invalidate that target-side
     tracking context so the ordinary planner can evaluate the alternate side.
4. Add a `dynamic-escape-exit` wall-admission log scope and a compact bridge log
   containing retained-solution availability, age, and replan result.

## Safety behavior

The retained solution is never used beyond 0.35 s and never accelerates under a
wall hold. Current footprint contact still requires stop/recovery. If no retained
solution is available, the existing decelerating hold remains authoritative.

## Non-goals

- No wall-margin reduction.
- No follow/overtake distance tuning.
- No change to Recovery strategy.

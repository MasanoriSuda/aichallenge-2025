# Design

## Frozen-snapshot result

The dynamic-obstacle refiner solves wall geometry first and uses that result
to choose one convex branch of the obstacle disjunction.  For an explicit
Pass side it currently waits for the wall-only trajectory to obtain full
lateral separation.  Until then it emits `effective_progress <= target -
overlap` at every stage.

The frozen sequence-2187 witness remains on the selected negative side but
does not reach full separation.  Its forward motion exceeds the stay-behind
boundary at stage 2, so the emitted longitudinal rows make the exact QP
infeasible.  An LP can make the late QP feasible by accepting the weak
wall-only lateral witness, but no transition stage can satisfy full lateral
separation.  Therefore that late partial candidate is not promoted: it has no
physical opponent certificate and would merely hide that the usable homotopy
was lost earlier.

## Earliest causal defect

At Pass entry the measured lateral separation reached 1.77 m and the
front-overlap exclusion latched.  It later decayed to about 0.95 m while the
obstacle-free wall witness returned toward the racing line.  For an explicit
tactical side, the refiner previously preserved the side only when the whole
future wall-only suffix stayed separated.  It therefore discarded the
already acquired physical homotopy and reverted future rows to stay-behind.
The bad late QP and subsequent SafetyBrake are downstream symptoms.

## Repair

Treat stage-zero separation as physical state and the wall-only result only as
an obstacle-free dynamics witness:

1. if the current physical state is fully separated on the explicit selected
   side, retain that side from the first valid prediction stage;
2. require full lateral separation throughout that valid suffix, even if the
   obstacle-free wall witness later crosses back;
3. do not infer ownership from one separated middle prediction;
4. keep the existing partial-escape contract only for initial-overlap cases
   where state bounds prove stay-behind impossible;
5. leave exact Cartesian footprint and wall certification unchanged.

This repairs candidate generation before the failure becomes irreversible. It
does not alter production authority, add a fallback, or weaken proof.

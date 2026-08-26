# Design: falsification of synchronous current-cycle retry

## Experimental change

The transition-only synchronous six-state admission transaction was
temporarily generalized to every callback where retained authority was absent.
It retained the full physical and current-world proof chain and never
published a raw solver result.

## What it proved

The missing authority is not one homogeneous async race:

- a small number of newly solved plans can join immediately;
- many plans remain blocked by the current dynamic world;
- when the solver itself does not converge, retrying synchronously consumes
  more than two control periods and repeats on the next callback.

Thus a callback-local solve cannot be the normal continuity mechanism.

## Production decision

Keep the existing synchronous transaction limited to atomic intent changes.
Keep normal trajectory generation asynchronous. Classify the first retained
rejection and repair the corresponding semantic boundary instead of retrying
the same optimization in the publisher callback.

The next work must distinguish true forward dynamic blockage from irrelevant
rear/side obstacles and must validate the measured-to-control wall prefix. A
new fallback or a solver/timing parameter change is explicitly out of scope.

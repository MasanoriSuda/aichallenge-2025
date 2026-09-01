# Requirements

## Objective

Obtain an independent `executed_teacher_success` validation run for the exact
precontact teacher and frozen base checkpoint.

## Constraints

- use unseen random seed 2032 in the same one-ego/two-NPC gate;
- do not modify teacher logic, thresholds, launch defaults or checkpoint;
- require three laps, Finish, zero penalty and zero post-start stall;
- a failed run rejects this validation source and must not trigger parameter
  tuning;
- keep train seed 2031 and validation seed 2032 physically disjoint.

## Definition of Done

- runtime mode and checkpoint identity are proven;
- motion and competition analyses pass fail-closed;
- successful evidence is extracted as split `val` under the strict outcome
  certificate contract;
- failure is recorded as evidence against the current teacher distribution.

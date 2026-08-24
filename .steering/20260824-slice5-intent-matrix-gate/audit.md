# Audit

## Result

`output/20260824-194340` is rejected as a Slice 5 dynamic-intent Gate.
The run did not enter the Overtake line FSM, so Pass and Return remain
`NOT EXERCISED` rather than passed.

## Observed boundary

- local behavior briefly requested Overtake four times;
- every atomic FSM entry was rejected because no selected canonical plan and
  execution certificate had yet reached the live callback;
- the asynchronous dual MPCC later produced feasible, physically validated
  left or right branches;
- 13 asynchronous progressive entries reached common admission:
  - 2 were rejected as `minimum-speed-insufficient` at 9.04 m and 8.47 m;
  - 11 were rejected as `completion-proof-rejected` after the gap fell below
    8.0 m;
- no `Idle -> ShiftOut`, Pass, Return, DynamicWait or DynamicEscape occurred.

## Root-cause chain

1. The geometric/kinematic Mission supplies
   `predicted_minimum_ego_speed_mps`.
2. The isolated five-state MPCC then solves an exact velocity/progress
   trajectory and binds it to the selected physical execution certificate.
3. Fresh-entry admission nevertheless compares the old Mission rollout speed,
   not the selected exact five-state velocity sequence.
4. The first two certified candidates are therefore rejected on stale-formulation
   speed evidence while the completion gate still has distance reserve.
5. Once that false rejection disappears, the remaining candidates occur below
   the deliberately conservative 8 m unproven-completion reserve and are
   rejected for a different reason.

The two gates therefore form an empty observed admission window, but the root
defect is not the configured 8 m threshold. It is that the selected canonical
branch does not own its own minimum-speed proof at final admission.

## Existing-patch relationship

Commit `a0270d5` added the completion-proof gate after close progressive
entries became squeezed before rear-clear. That protection remains valid and
must not be weakened to hide this failure. Commit `096a10f` correctly unified
all fresh progressive producers behind the gate, but also made the older
Mission-level minimum-speed evidence the common admission input even after an
exact five-state branch had been selected.

## Decision

Do not tune follow distance, the 8 m completion reserve, solver settings or
wall margins. Open a bounded repair Slice which gives a complete, current
five-state execution certificate precedence for minimum-speed evidence while
retaining every physical, completion, freshness and current-world gate.

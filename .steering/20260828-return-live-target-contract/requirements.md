# Requirements

## Objective

Let a physically certified `Return` keep producing fresh seven-state MPCC
problems after the completed overtake target has legitimately left the active
target set.

## Frozen evidence

- baseline: `39010cb0`
- run: `output/20260828-133920/d1/autoware.log`
- episode 3 reached `ShiftOut -> Pass -> Return`;
- the first Return artifact (`sequence=4119`) was solved and physically
  certified;
- after rear-clear, behavior correctly changed to Cruise and active target
  provenance became unavailable;
- every subsequent Return submission was rejected as
  `rate-resolved source context incomplete`;
- the seven-control Return artifact exhausted, canonical Return authority
  disappeared, and Emergency Stop led to external stuck Recovery.

## Constraints

- Do not change solver tolerance, horizon, wall/vehicle clearance, speed,
  lease, timeout or fallback policy.
- Preserve the completed encounter identity, Mission generation and selected
  side through Return.
- Do not require a live target-observation generation after rear-clear.
- Continue to prove the current wall and all currently observed dynamic
  obstacles before publishing each fresh or retained Return artifact.
- Do not change production-authority arbitration.

## Definition of done

- The execution contract distinguishes encounter identity from a live target
  obstacle observation.
- Follow, ShiftOut and Pass still require a current target observation.
- Return requires its encounter identity and side, but accepts a zero target
  obstacle generation after rear-clear.
- Fresh Return submissions continue after behavior no longer selects the
  completed target.
- Unit tests and the package build pass.
- A dynamic run reaches a clean `Return -> Idle` without cursor exhaustion or
  external Recovery for this failure path.

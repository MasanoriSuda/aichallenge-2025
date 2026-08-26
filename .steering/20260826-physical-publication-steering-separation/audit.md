# Root-cause audit

## Observed phenomenon

`output/20260826-113945` proves that publication-time sampling is now explicit,
but domain 1 repeatedly loses normal authority under steering-rate saturation.
At decision 1195, the last desired command is `-0.132350 rad`, the physical
prediction origin is the same value for that cycle, and extraction requests
`-0.183706 rad`; the 25 ms desired-command envelope ends at `-0.155251 rad`.

## Causal graph

```text
observed steering
  -> latency-compensated physical origin
  -> six-state physical trajectory and wall proof
last published desired steering
  -> used only by downstream reachability check
physical origin
  -> incorrectly reused as desired sequence predecessor
certified steering-rate sequence + artifact age + duplicated publication advance
  -> desired jump larger than one publication step
  -> retained current-world proof rejects steering-unreachable
  -> canonical normal authority unavailable
  -> explicit Emergency / later Recovery

first repair run output/20260826-121217
  -> semantic steering extraction reject becomes zero
  -> state-box translation is still not an exact cumulative-rate certificate
  -> strict desired sequence occasionally exceeds the angle limit

direct cumulative-rate prefix rows
  -> semantic extraction reject becomes zero in output/20260826-122709
  -> publisher join rejects almost every candidate because float32 ROS wire
     steering is compared with the double solver value
  -> executed plan remains sequence 307 while certified candidates advance
  -> a fresh-result gap cannot use retained evidence and causes Emergency

wire-exact publication join
  -> output/20260826-124000 records joined=3324/3722 and rejected=0
  -> executed sequences advance to 2942/2998
  -> the former stale-executed-plan authority loss is removed
```

## Competing hypotheses

- Tight wall/clearance: falsified for the typed steering rejection; wall proof
  is accepted before the join.
- Solver failure: falsified in the selected examples; solve is accepted.
- Revalidation too strict: rejected because the candidate command is physically
  outside its declared desired-command rate envelope.
- One value owns physical and desired origins: directly supported by artifact
  layout, the failure-first extraction test and the live decision trace;
  highest confidence.

## Earliest violated invariant

A normal artifact must not give one scalar both vehicle-state and command-history
semantics.  Physical wall proof starts from the latency-compensated measured
steering state; desired publication continuity starts from the last published
desired steering.  They may differ under actuator lag.

## Repair correspondence

- Cause: one steering origin had two meanings. Repair: seal two origins.
- Cause: desired extraction integrated from the physical origin. Repair:
  integrate the exact certified rate sequence from the publication origin.
- Cause: desired angle feasibility was absent from the QP. Repair: append an
  exact cumulative steering-rate prefix bound for every stage, intersecting
  the admissible physical-origin and publication-origin angle intervals.
- Cause: the publisher boundary compared a float32 ROS field with its original
  double value. Repair: retain exact double equality before serialization and
  use exact float32-wire equality only after publication, before `mark_executed`.
- Mask retained: current-world reachability remains fail-closed and is not
  relaxed. No replacement fallback or clamp is added.

## Remaining evidence outside this Slice

`output/20260826-124000` still has repeated Follow -> ShiftOut requests whose
synchronous six-state physical proof reports `stage-wall-rejected`.  Those
requests correctly fail closed, but the supervisor proposes the rejected
intent repeatedly and temporarily loses normal authority.  This is the next
all-intent dynamic-admission/transition defect; it is not repaired by changing
the steering publication contract.

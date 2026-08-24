# Validation

## Dynamic run

- Command: clean `make dev2`, no manual pose or control publication
- Output: `output/20260824-194340`
- Duration: more than two laps, then stopped after the same entry rejection
  repeated around the course

## Counts

- `Idle -> ShiftOut`: 0
- Pass: 0
- Return: 0
- Recovery: 0
- DynamicEscape active: 0
- legacy Overtake normal authority: 0
- async entry admission rejections: 13
  - minimum speed: 2
  - completion proof: 11
- completion gate warnings: 53

## Earliest decisive sequence

1. 9.04 m: canonical progressive entry rejected for minimum speed; completion
   proof itself passes with 3.52 s before no-return.
2. 8.90 m: local behavior requests Overtake, but atomic commit correctly keeps
   canonical Cruise because certificate/plan are absent.
3. 8.47 m: the next asynchronous candidate is again rejected for minimum speed;
   completion proof still passes with 3.23 s.
4. 7.97 m and below: otherwise viable progressive prefixes are rejected by the
   unchanged 8 m completion-proof reserve.

This run is sufficient to reject the intent-matrix Gate and justify a separate
source repair. It is not evidence that Pass/Return production authority works.

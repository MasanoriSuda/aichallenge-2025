# Results: Stop producer/consumer latency

## Dynamic evidence

Two bounded `make dev2` runs were compared without changing production
authority, candidate selection or any control parameter.

- `output/20260831-142607/d1`
  - consumer decision 1494 saw Stop source sequence/decision `838/1490`;
  - source age was `0.105 s` and its control origin was still `0.025 s` in
    the future;
  - both ordinary retained and alternate evaluation reported
    `intent-mismatch` at the ShiftOut-to-Pass boundary;
  - the fresh Pass proposal was accepted, so this observation did not cause
    a global authority loss.
- `output/20260831-143756/d1`
  - consumer decision 1528 saw Stop source `886/1520/shiftout/generation:1`;
  - source age was `0.200 s` and its control origin was already `0.070 s` in
    the past;
  - current-world retained validation still returned `accepted` and joined
    the Stop candidate.

## Classification

Raw worker or mailbox age is not a sufficient cause of Stop rejection.  A
candidate older than the failed transition example joined successfully after
current-world revalidation.  The first observed rejection was a semantic
phase/intent boundary, not steering reachability or age.

The frozen decision 1563 remains a different case: direct current-world Stop
and all 68 rate schedules fail in the unchanged offline audit, and the best
rejected nonlinear rollout reaches exact wall contact about 5.624 m ahead.
Therefore the certified cached Stop being `steering-unreachable` is a symptom
of arriving at the failure decision after terminal viability was already
lost, not proof that producer latency alone caused the failure.

The next root-cause boundary is upstream: determine why a normal Pass artifact
was publishable while its terminal Stop viability later disappeared.  Do not
add a Stop resume rule, age grace, worker timeout or clearance adjustment.

## Verification

- `make autoware-build`: 25 packages passed.
- Package CTest: 59/59 passed.
- Aggregate test result: 2321 tests, zero errors and failures.  The existing
  stale `joycon_contract_guard/package.xml` entry still emits a skipped-file
  diagnostic.
- Dynamic telemetry proves producer decision, intent, generation, snapshot
  age and control-origin age are joined at the consumer.

# E2E spatial authority output-alignment requirements

## Objective

Remove the training/runtime output-support mismatch found by the closed-loop
DAgger audit.

## Frozen contracts

- candidate3 and all shipped runtime settings remain unchanged;
- spatial authority remains default-off;
- the authority limit remains exactly plus/minus 0.12 rad;
- use the same frozen conv5 representation, data split, signed-mixture head,
  optimizer and seed as DAgger v3;
- the failed NPC run remains audit-only;
- do not tune clearance, acceleration, braking or authority thresholds;
- no candidate is promoted in this slice.

## Acceptance

- train a candidate whose model support itself is plus/minus 0.12 rad;
- evaluate the exact candidate output and runtime-clipped output on the same v4
  audit set;
- improve held-out failure and aggregate attainable-improvement utilization;
- preserve peer direction and production-normal leakage gates;
- reject the candidate if output alignment merely hides direction or leakage
  regressions.

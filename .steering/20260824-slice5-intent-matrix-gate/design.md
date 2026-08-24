# Design

## Evidence sequence

1. Run the exact committed baseline with `make dev2`.
2. Join line-FSM transitions to `Overtake control decision` identity fields.
3. Count each canonical intent by fresh, retained, Emergency, legacy and
   contract-join result.
4. If Pass/Return are absent, trace the last accepted ShiftOut decision and the
   first blocking/recovery reason. Do not tune the scenario in this Slice.
5. If a deterministic source defect is proven, close this evidence Slice as
   rejected and create one bounded root-cause Slice for the repair.

## Acceptance oracle

The final decision trace, not a planner candidate or shadow solve, owns the
result. A phase is structurally accepted only when its published decision has:

- `formulation=velocity-progress-5state`;
- complete problem/solution/plan/execution-certificate identity;
- `contract_join=1`;
- canonical fresh/retained source or explicit Emergency;
- no `legacy-normal-bypass`.

The Gate measures architecture and integration. It does not certify lap time,
collision-free race quality or 2026 real-vehicle safety.

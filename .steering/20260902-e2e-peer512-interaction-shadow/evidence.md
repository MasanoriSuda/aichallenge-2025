# Evidence

## Frozen inputs

- Run: `output/20260902-e2e-peer512-interaction-shadow-v2`
- Scenario: domains 1 and 2 MPC peers, domain 3 TinyLidarNet production plus
  authority-disabled recurrent shadow
- Recurrent artifact:
  `conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016/candidate.npy`
- Recurrent SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- Production TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- Recurrent authority: disabled for every reported interval and applied zero
  times

The first attempted run was invalid because TinyLidar experiment environment
variables leaked into the MPC peer containers.  Commit `aab15ac5` removes
those variables from the peer process environments.  Only the corrected `v2`
run is evidence for this Slice.

## Competition and motion

Domain 3 completed the required three laps and finished first:

| Metric | Result |
|---|---:|
| Lap 1 | 87.5965 s |
| Lap 2 | 85.0229 s |
| Lap 3 | 85.3227 s |
| Total | 257.9422 s |
| Mean forward speed | 3.7440 m/s |
| Maximum speed | 4.4480 m/s |
| Longest low-speed interval | 0.0000 s |
| Positive-acceleration stall | 0.0000 s |

The strict competition report fails because `d3-result-details.json` contains
one wall event at race time 412.562 s on lap 4.  Domain 3 had already completed
its three laps; its rosbag ends at Finish while the session continued for the
unfinished MPC peers.  The report generated with a one-penalty allowance is
retained only as a post-Finish diagnostic and is not promotion evidence.

Domains 1 and 2 did not finish.  Their motion analyzers also reject the run:
domain 1 includes a positive-acceleration stall and domain 2 has a 10.948 s
low-speed interval.  These peer-controller failures do not authorize relaxing
the domain-3 recurrent Gate.

## Recurrent runtime Gate

The strict domain-3 report is
`output/20260902-e2e-peer512-interaction-shadow-v2/recurrent-shadow-analysis-d3.json`.

| Metric | Required | Observed |
|---|---:|---:|
| Coverage | >= 99% | 95.7445% |
| Errors | 0 | 0 |
| Non-ok intervals | 0 | 6 |
| Minimum scan frequency | >= 19 Hz | 18.09 Hz |
| Hidden-state resets | <= 1 | 401 |
| Authority applications | 0 | 0 |
| Mean inference time | diagnostic | 30.908 ms |
| Maximum inference time | diagnostic | 151.060 ms |

All six non-ok intervals report `missing-or-stale-speed`.  The recurrent
candidate declares `use_speed=false`, but the current controller executes
recurrent shadow synchronously inside the production scan callback and shares
the production spatial freshness admission.  Under three-process MPC load,
the observation-only work therefore increases callback latency and can help
make the shared wheel-speed sample stale.  A stale sample then resets the
recurrent state, even though recurrent authority is disabled.

This evidence does not prove that model inference alone caused every stale
speed sample.  It does prove that the current shadow architecture permits a
non-authoritative diagnostic to consume 30--151 ms on the production command
path, so the run cannot be treated as an unperturbed production baseline.

## Gate decision

**Reject** the peer512 artifact for bounded-authority A/B at this Slice.

- Do not enable recurrent authority.
- Do not package this recurrent artifact.
- Do not loosen speed freshness, coverage, reset or scan-rate thresholds.
- First move authority-disabled recurrent observation off the production
  command critical path with bounded, latest-wins work and explicit
  submitted/completed/dropped/stale/error telemetry.
- Re-run single-vehicle and three-vehicle shadow Gates before reconsidering
  authority.

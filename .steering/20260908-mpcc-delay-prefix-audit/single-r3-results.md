# Diagnostic 3: actual published source captured

Run `20260908-execution-evidence-single-r3`, Domain 1, baseline
`a8b968ac2e4a86a28432a52d132774459d355cb5` plus sealed working changes.
Controller `18df8ab763bffa3a0e79ecb8b6e620a894476b7af5423e7928b618829f69cabc`.
Formal result: six laps, 259.3001708984375 s, penalty 0. Integrated acceptance
is **rejected**: active-race Emergency at decisions 7405/7406, speed 7.53 m/s.
No Recovery transition. Finish-phase overrides 11575/11576 are separate.

Observation acceptance succeeds: at wall-clock 1788806751.870682257 the
existing failure worker reports `source=6782,kind=exact-executed,status=written`.
This is the actual retained artifact, not a new solve of its original input.
The three-trial observation campaign is closed; no unchanged repeat is planned.

## Identity and time

| Field | Value |
|---|---|
| Current decision / interaction fingerprint | 7405 / 14329803062406037027 |
| Source sequence / interaction fingerprint | 6782 / 5246716225714339212 |
| Source problem fingerprint | 13257275253882479858 |
| Source observation / prediction origin | 169.889996202 / 170.019996202 s |
| First publication origin / artifact cursor | 170.034996202 / 0.01500000000001478 s |
| Latest publication decision | 7404 |
| Failure observation / control origin | 169.939996201 / 170.069996201 s |
| Artifact cursor at failure | approximately 0.05 s |

The original and current snapshot pair is 6782/7405 in this run's
`d1/mpcc_architecture_snapshots/`. The source YAML hash is
`de387af1db6704d89b3d0218d949f592d7ec73faa81af9c5d21e4c5041fa2d47`.
Do not interchange interaction and problem fingerprints.

## First failure and timing

The delay prefix rejects at index 24 of 25, elapsed 0.120 s, pose about
(89667.776, 43169.696, yaw 0.016). Normal continuation and terminal Stop proof
are not reached. Normal authority returns around decision 7514 / 172.654996 s
at speed 0.004948 m/s. This braking sequence is not evidence of body contact.

Callback telemetry: 13,457 reported cycles, maximum 21.207 ms, zero overruns.
Control receive stream: 10,859 messages over 271.406884909 s, mean interval
24.996029187 ms, p95 26.485693455 ms, p99 27.422435284 ms, maximum
35.177230835 ms; zero nonpositive intervals or intervals above 50 ms.
Source clock separately has 21 nonpositive intervals and two above 50 ms,
maximum 114.999997 ms. Receive continuity does not establish source-clock quality.

## Offline replay and its limit

The actual production physical adapter rebuilds the recorded artifact with
zero solver calls. It succeeds and saves the dense native trajectory.
The optional external-primal architecture CLI returns 4: its exact-QP input
is absent at this source-only capture boundary. This is **not** a physical
infeasibility result, and no QP was invented to make it pass.

Independent coordinate comparison finds a physical-model inconsistency;
see [frame-model-audit.md](frame-model-audit.md). The saved full horizon is
a conditional continuation of the original controls, not proof that the
vehicle executed every future control after Emergency superseded the plan.
All raw files and preserved inputs are indexed by
[single-r3-evidence.json](single-r3-evidence.json).

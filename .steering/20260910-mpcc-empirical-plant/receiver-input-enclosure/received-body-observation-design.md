# Received body observations at the decision boundary

2026-09-13 JST. Baseline/rollback8d83511f; dynamic controller8d9810e0/build120.
Observation only, within the ongoing M4 causal investigation. No new authority,
model, reconstruction, control parameter, physical or timing gate.

R556/R557 show that force-r2 source velocity is30ms older than its pose. R558
common-past reconstruction reduces some errors but regresses current freshness
and crosses a rest/launch boundary. The next unknown is whether newer public
values were already received and discarded by `latest_at`'s pose cutoff. MCAP
receipt is not the controller callback's receipt, so bag inference cannot close
this boundary. Existing snapshots retain selected samples only.

Capture the three existing bounded body/tire queues after the ordinary prediction
is assembled. Retain every resident source stamp, steady receipt timestamp and
value, with selected pose/components, decision ID and capture steady timestamp.
Keep pose-newer and ROS-clock-newer received values; observations do not make
them legal control inputs. Source/semantic/current captures are separate immutable
objects, copied by shared ownership into the existing asynchronous failure writer.
The source snapshot and scheduled worker must carry their original captured
object rather than borrowing the later current queues.

Use optional diagnostic fields outside model/observation/interaction fingerprints.
Do not add queue serialization to hot-path fingerprint construction. Missing or
invalid diagnostic metadata must not discard valid original proof evidence.
Record validity and selected-observation association explicitly; no diagnostic
field may influence solve, adoption, physical proof or publication. Existing
256-entry queues remain the cap; do not introduce a retention or control knob.
The normal component selection and output values remain unchanged in this slice.

Verification: meaningful writer regression for newer-than-pose values and exact
source/receipt preservation; current/source separation and immutable ownership;
unchanged interaction fingerprint and old replay compatibility; absent/invalid
metadata does not reject valid evidence. Check actual node wiring and full
build121/package108/source tests. Then commit and run a bounded single observation
to the first moving failure, inspect captured queues/selection and callback costs,
restore protected artifacts and update the registry. This does not complete M4–M6
or promote interpolation/contact candidates. A subsequent reconstruction needs
causal raw support, reset/rest negatives and all original safety gates.

## Verification before the new observation

R559 exposed and fixed the direct header dependency. R560 retained a standalone
fixture path failure. R561 keeps that history and corrects only fixture layout:
old writer3fail/new writer3pass for actual queue persistence and source/current
separation. Received int64 times above2^53 round-trip exactly. Missing/invalid
optional data leaves the original interaction fingerprint and replay intact.
No selection, authority, model or physical/clock guard consumes this payload.

`make autoware-build` build121:26packages pass,4min58s. Existing setuptools
setup.py deprecation warnings remain; no compiler errors. Package108 via
`colcon test --packages-select multi_purpose_mpc_ros` and `colcon test-result`
passes2613records/66groups with0errors/failures/skips,41.47s. Source contract106
also passes. R562 recompiles the diagnostic driver for the changed Snapshot
layout and reproduces all3 exact historical source/domain/history/last-actual/
check/rejection-time identities. Force-r2 and single-r6 retain their wall rejection;
single-r5 still physically accepts but rejects its actual before-publication
clock. Individual current checks3.714/3.279/0.920ms are unloaded observations,
not live bounds. [Payload hashes and results](received-body-observation-evidence.json).

Next is committed standard single-r7 with original DLL, unchanged controller
settings and a120s host cap, preserving the first moving failure. Inspect semantic,
programme and current capture associations and excluded received values. In
particular, post-publication prediction may have a later `now_sec` than the
original captured decision while retaining its raw components; compare those
explicitly instead of rewriting either epoch. Restore protected results and
measure callback costs. Physical reconstruction/contact repair and allM4–M6
remain open; this observation change does not claim race acceptance.

R7 observation completed with20unique valid decision captures, all decision
associations exact, no callback above25ms and no observed send-window violation.
It stopped inReady without restart/laps. R563–R566 distinguish receiver availability
from physical accuracy and rule out initial desired as a nominal prediction cause.
[Run audit and next physical comparison](received-single-r7-audit.md).

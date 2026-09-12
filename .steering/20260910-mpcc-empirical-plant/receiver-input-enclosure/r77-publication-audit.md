# r77 final publication boundary and current proof cost

2026-09-12 JST. Baseline/rollback `1241e0cf`, native nine-state numerical owner.
Build117 all26 packages, tests104 all2599 records pass. Authorized M4–M6 remains
active; no push or routine confirmation. No production change in this audit.

Standard dev2-r77 ends automatically at D1 Ready decision934, about0.99m/s, wp30.
Neither vehicle reaches Start or a lap. Source389/job930/index1 and geometry
15741568466272881844 are exact. Index0 at933 publishes;934's complete current
physical proof accepts, then the before-publication clock guard correctly rejects.
The original DLL and both user result JSON are restored; all containers stopped.

| Boundary | Simulator seconds |
|---|---:|
| Nominal packet | 11.399999813 |
| Callback entry / current request | 11.404999745 |
| Original deadline | 11.424999813 |
| Before-publication clock | 11.429999744 |

The callback takes20.817ms wall /11.341289ms CPU. Current proof takes16.309253ms
wall /9.186057ms CPU; phase instrumentation records6 involuntary switches during
the proof. Problem initialization takes3.633ms. ROS advances25ms after an entry
already about5ms past nominal. Average Hz or a callback below25ms cannot establish
this packet's original deadline. The final guard is not the producer defect and
must not be relaxed. Source/current geometry and intent match; this is distinct
from r75 Start speed-limit invalidation and r76's correctly rejected wall world.

## Exact current replay and measured cost

R498 replays the full source, original0.63s retained history, original source input
13124620287640035622 and domain9729460009769520190. It reproduces current physics
acceptance for151 samples, entry guard true and actual before-time guard false in
all3 replays. Unloaded5.279–5.314ms is not live timing. The generic final-guard YAML
contains a default/stale current_check array; use the accepted934 admission record
and regenerated exact dispatch, not that array as the final physics result.

R499 unchanged numerical instrumentation: total5.622ms, full fresh reproof4.960ms,
domain check0.657ms, partitioned propagation4.031ms, world checks1.362ms. These are
nested costs, not additive independent totals. Compiled equations hit268 times;
204 moving steps recompute natively. Force coefficients alone take0.260ms.
This does not justify repeating previously rejected generic SIMD/force packing,
coarsening, relative-current recomposition, affinity or deadline-grace variants.

R500 changes only which existing compiled source equations are searched, retaining
exact duration/model/input/body containment and all current/physical checks.
All-row search lowers this replay from5.315–5.397ms to4.760–4.808ms. R76current3999
still rejects the wall (98.614997805 versus original98.619997805), one existing5ms
step earlier due to the different conservative numerical enclosure. All-row lookup
is diagnostic, not promoted; this modest gain does not establish live acceptance.

R501 measures misses separately, retaining original results. Original-row hits/
misses268/204 become376/96 over all existing rows. Of the original misses with
map candidates,80 have yaw as their closest missing component and106 speed;
all-row misses have54 yaw and24 speed. Another18 calls have no map view (native
prefix). Timing with these extra counters is not production cost.

## Coverage and next causal work

Whole-run D1/D2 source command Hz38.8263/38.5424; max gaps45/60ms, no duplicate,
backward or nonfinite commands. Measured speed maxima1.1211/1.3383m/s. Callback
samples1330/1278, maxima22.626601/22.293306ms, none over25ms; neither result negates
the actual ROS deadline failure. D1's later1110 before-window failure is separate.
Publication is not simulator application acknowledgement.

Next compare numerical equation representations against these exact worlds.
Yaw-dependent lookup misses suggest local rotation covariance of the native map,
not a change to the current population's physical frame or its certified world.
This is a hypothesis only: any prototype requires exact containment, independent
high-precision body/corner oracle, unchanged rejected physical/clock cases and a
material cost improvement before a production proposal. Existing all-row lookup
alone is unpromoted. No rerun of unchanged failed experiments for acceptance.

All intents/Mission/sibling/Store, actual Stop/rest/restart, Recovery/Rejoin/Boost/
async and faults, same-final-source single/dev2 repeats, dev3/dev4 six laps, gates
and same-artifact submission/image/eval remain open. [Evidence](r77-evidence.json).

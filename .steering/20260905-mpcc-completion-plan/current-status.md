# Current status and remaining completion work

2026-09-09JST. Working baseline d54822c982c705098cc2fbfab4160f15e41865b0.
The complete MPCC acceptance is NOT achieved. Preserve previous failed trials.
The user authorised autonomous execution through completion on2026-09-09.
Local commits bc01ff2f,ca38bf18,d54822c9preserve the previous completed work.

## Completed work

- Astra harness: root/package AGENTS,7Skills,project Astra/high default;
  difficult MPCC audits use an explicit xhigh override. Frontmatter/TOML/links
  and effective configuration checked. No reasoning quality/cost benchmark claimed.
- Semantic target time, Stop feasibility/reference and certified publication,
  stateless successor provenance: local causal repairs and regressions recorded
  in their steering/central experiment registry. Integrated coverage remains open.
- Local AWSIM calibration: UNKNOWN GNSS covariance explicitly configured,
  nominal ego body enclosed by measured footprint, omitted static wall cells
  added and native/editor preservation aligned.25packages build and regression
  evidence retained. None of these alone establishes race acceptance.
- New bounded observation preserves actual published solver source, artifact
  and publication clock before Emergency clears the ledger. No new authority.
  Latest25-package build passes;60CTest groups/2270package-local test records,
  0errors/failures/skips. All7normal intents preserve original solver source.
  Final bounded diagnostic captures actual published source6782at failure7405.
- Coordinate repair now integrates physical Cartesian motion and projects into
  the exact immutable reference frame, retaining the7-state solver order and
  unchanged actuator model. Native invariants24/24pass;25packages build and
  all60CTestgroups pass. Old model artifact identities are rejected. Fixed-control
  source6782error against independent Cartesian ODE is0.231333mmmaximum,
  previously651.9642mm. New solves of6782/7405stillreject wall rows; no
  physical infeasibility claim. Initial6laps252.253662s/penalty0, no active-race
  moving override or Recovery. Callbackmax20.390ms/0overruns, control receipt
  max39.862633ms/0gaps>50ms. Source duplicates coincide with simulator clock
  and sensor publication pause. Later geometry-lineage refusal checks and
  normal/continuation/Stop binding regressions pass; final build25packages,
  all60CTestgroups/2335colconrecords,0errors/failures/skips. The integrated
  campaign with those final checks and peer geometry remains open.

## Current evidence

Corrected-wall trial20260908-wall-map-single-r1:6laps253.198868s/penalty0,
but moving Emergency10699/10700at7.53/7.55m/s. First rejection is delay-prefix
collision at0.105s, before continuation or Stop-terminal proof. The old exact
published10073artifact was not recorded. Fixed-input comparisons do not find
a feasible alternative or prove physical impossibility. Contacted grid cells
contain wall faces at body height, not just overhead geometry.

Observation trial20260908-execution-evidence-single-r1:6laps250.508286s/
penalty0, no active-race moving override, callback20.701ms/0overruns,
control receive max35.215855ms/0gaps>50ms. Failure did not recur, so published
artifact capture was not triggered. The protocol permits at most2additional
same-settings diagnostic trials, solely to obtain the missing failure data.
The earlier rejected result is retained. No pass can replace its explanation.

Observation trial2finishes6laps252.718750s/penalty0 but loses normal authority
at9005/9006at7.65m/s. Here delay prefix and20-stage continuation are clear,
while both terminal Stop references fail. Original source is absent because
the source producer only retained ShiftOut/Pass inputs. A native probe confirms
5of7normal intents missing. The observation producer now retains all7; its
new build/tests and final planned diagnostic now pass observation acceptance.
This does not fix the earlier delay-prefix or terminal failure by itself.

Observation trial3finishes6laps259.300171s/penalty0, but Emergency7405/7406
at7.53m/s. Source6782and actual artifact/publication clock are saved; first
rejection is delay-prefix collision at0.120s, before continuation or terminal
proof. Callback21.207ms/0overruns;receive max35.177231ms/0gaps>50ms.
The three-trial observation campaign is closed, with all outcomes retained.

The actual native transition fails physical coordinate invariance: stationary
body, curvature0.2/m,lag-0.3m,virtualspeed4m/s moves39.9899mm in0.1s.
14of24native analytical cases fail. With captured6782controls, native world
position differs from independent Cartesian motion by47.0783mm at45.123ms.
Same source grid/native sweep: native path clears hard reserve; Cartesian
path rejects at the first sampled4.512msboundary while body margin stays
clear. Correct arc-length equations alone still leave16.0719mm at45.123ms,
because stored curvature and piecewise reference-frame derivatives disagree.
Complete actual-frame equations agree with Cartesian integration offline.
This was a proved model/certificate defect; the current local repair is above.
It is not yet a complete attribution of the live delay-prefix failure. Full-horizon rollout
is conditional: Emergency supersedes actual publication aroundcursor0.05s.

## Order to finish

1. Physical-model repair: compare complete coordinate-consistent seven-state
   Frenet and Cartesian formulations on the sealed6782/7405world and earlier
   failures. Align SQP, artifact, normal/Stop/retained rollout and wall/peer
   reconstruction; version model identities and reject incompatible warm
   starts. Native failing invariants and same-input wall-verdict comparison
   are already available. Also finish the command/sensor/time join for the
   live delay prefix; its linear steering interpolation remains a separate
   hypothesis. Do not adjust margins, delay, weights or suppress Emergency.
   Detailed scope and acceptance: [frame-model audit](../20260908-mpcc-delay-prefix-audit/frame-model-audit.md).
2. Peer geometry: V2X position is the GNSS antenna, not base_link. All4local
   kart bodies need a1.876m nominal spherical enclosure about that point when
   no heading is provided. Current peer-circle predicate permits a2D body-
   overlap counterexample with positive0.822m reported clearance. Separate
   complete body geometry from lateral spacing/uncertainty; update every
   current/serialized/recovery consumer consistently and check pass feasibility.
   A tighter oriented representation requires an observed orientation contract.
3. Fresh fixed campaign: single six laps, Follow, both Pass/Return directions,
   certified multi-tick Stop/restart and Recovery/Rejoin. Then async identity,
   dev3/dev4 and gate1/2/3. Seal all inputs/binaries and all repeated results;
   check final authority, wall/peer proof, callback/publish timing and every vehicle.
4. Submission: create tar.gz, bake eval image from that artifact, run make eval;
   verify top-level archive/interface/schema and same-run outputs. Update
   registry and P0-P4 tasklist with actual evidence before declaring completion.

Detailed current work: ../20260908-mpcc-delay-prefix-audit/ and
../20260909-mpcc-coordinate-consistency/ and ../20260908-peer-envelope-audit/.
Full acceptance remains P2partial/P3/P4open.

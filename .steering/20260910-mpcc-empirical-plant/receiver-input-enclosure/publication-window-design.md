# Certified publication window

2026-09-11 JST. Autonomous implementation is authorized by the active user request.
Baseline/rollback91d798b7; control baseline50486fcf. M4–M6 remain open.

## Earliest broken invariant and causal replay

An applied-input proof must include the actual publication epoch of its first
serialized packet. The prior918/source137 certificate assumes9.919999778;
the next919observation records the identical packet at9.934999777. The final
packet check in`mpc_controller_cpp.cpp::applied_program_matches_final_packet`
compares only the nominal message stamp. Recording actual time after publication
is already correct and must not be backdated.

R180 authenticates both source identity and first packet bytes. The original918
programme clears. Shifting its publication schedule by the measured14.999999ms,
while retaining its original initial state/history/world/profile, rejects the
wall at10.644999777 with minimum directional gap−.018548189m. Conversely,
retaining the exact919state and all packet bytes but replacing only the last
actual historical epoch with the prior nominal epoch clears the wall again.
That latter history is explicitly counterfactual and must never reach production.

The old certificate excludes positive acceleration over10.174999778–10.184999778,
while the actual history still admits+1.3295964002609253m/s² in two original
substeps. This establishes a publication/proof binding defect before the later
wall rejection. It does not prove actual contact or all-method infeasibility.
Refined numerical boxes did not address this missing timing population.

R181isolates a future-publication window equal to the original25ms period.
The prior normal word rejects before publication and admits native contact
witnesses under that larger, now explicitly represented timing population.
The immediate minimum-braking word at the same prior state clears all original
wall checks and reaches complete rest; the later919word still rejects.
These are physical input-word comparisons, not nominal MPCC/production authority.

## Change and ownership

- Add a finite nonnegative maximum publication delay to the common input
  programme, bounded by its existing publisher period. Canonical certification
  uses exactly that period. This is the existing callback deadline, not a new
  configurable timeout, a larger receiver age, or permission to miss a deadline.
- Future packet values remain possible from nominal publication through the
  latest certified publication plus the original receive/mechanical ages.
  Actual historical packet epochs are unchanged. Coverage must also hold when
  each future publication occurs at its latest allowed time; no future packet
  may fill an earlier gap. Complete-rest timing covers the latest positive input.
- Bind the window into context/provenance hashes, Stop preparation/materialization,
  remaining programmes and serialized internal evidence. Zero-window legacy
  v1/v2 fingerprints remain stable; new window provenance is explicitly v3.
  An absent/malformed new window or ceiling discriminator is rejected.
- Keep nominal first-packet time, immutable source/decision/problem and exact
  float values checked. Before publication, compare a fresh causal ROS epoch
  with the certified first-packet window. After publication, compare the existing
  recorded epoch as an upper bracket and report any crossing as a violation;
  an after-publication check is detection, never retroactive authorization.
  Bracket telemetry is required for empirical acceptance.
- Retire the nominal-stamp-only final guard. Keep the existing Emergency path,
  original nominal solver/physical/full-rest/peer/Follow gates, and exact
  same-world materialization reuse only when its full window context matches.

The public ROS topics, Domain separation, submission layout and result schemas
remain unchanged. Internal evidence gains optional programme metadata and v3
provenance; old replay evidence remains readable and retains its old semantics.
Actual vehicle/transport guarantees are still outside the empirical scope.

## Validation and remaining work

Required: original15mscausal replay; channel and coverage boundaries, delayed
positive/steering inputs, unchanged actual history, native response inclusion,
complete rest, invalid windows, hash/schema/legacy and materialized-Stop roundtrip,
and final actual-clock guards. Then make autoware-build, package tests and the
original history/wall/peer negatives before a committed live trial.

The first long Stop proof still takes~101ms. IsolatedAVX2r179reduces it to~71ms
with complete tube bit equality; that prototype is not promoted. It lacks a
portable dispatch and does not solve deadline acceptance. A valid publication
window may expose these old long callbacks earlier, so timing remains a separate
required repair. Do not suppress its errors or label the campaign complete.

## Local validation checkpoint

Buildr60 succeeds26packages/4min56s; testsr49 succeeds2459records/62groups,
0errors/failures/skips. Buildr57 failed Docker socket access before compilation;
buildr58 passed, testsr47 reported one new oracle timestamp-generation failure
twice. The generator selected adjacent nominal windows with a one-representable-
step backwards actual epoch. The diagnostic generator now takes the previous
actual epoch as a lower bound and checks it remains inside the exact window;
no time tolerance, physics or test assertion is relaxed. Buildr59/testsr48 passed
2458records. Review then found raw-clock regression could be hidden by the
causal nominal floor; the final sixth behavior test covers that case inr60/r49.

Fresh ABI native tools182–184 run the15sealed cases. Twelve keep their prior
classifications/materializations/joins. R24D2current990 andr19D2previous1070/
current1071 change from accepted to peer-rejected under the added timing
population; these rejected old positives remain explicit in the comparison.
The actualr18history-gap joins still reject reason5. R185reproduces the exact
15msfactorization with the current production library and rejects the previous
918/source137 normal command at the applied wall gate. Its source-only137
substitution into the inspected131prefix is a deliberate unbound identity
negative, not the fully rebound actual-source test from the earlier audit.
The immediate-braking word clears the physical wall but remains a diagnostic
without a newly solved nominal authority.

[Sealed evidence](publication-window-evidence.json) records290files, all attempts,
source/binary inputs and native comparisons. Originalzero-window words and
backdated-history counterfactuals are diagnostic only. Stop materialization,
remaining programmes and hash/schema tests cover the nonzero window; actual
multi-tick execution, live publication brackets and reset/async remain open.
Next is a local commit and dev2-r29 under the same original campaign conditions.

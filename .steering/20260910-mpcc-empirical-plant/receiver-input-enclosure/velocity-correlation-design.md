# Preserve velocity correlation in applied-input enclosure

2026-09-13, baseline `f96325e6`, production diagnostics `b368ecab`, model `8d9810e0`.
The user authorizes implementation, verification and local commits without another approval.
M4–M6 and race acceptance remain open. Rollback baseline: `f96325e6`.

## Broken invariant and scope

A shared acceleration creates a coupled change in forward velocity, lateral velocity
and yaw rate. The existing interval producer repeatedly replaces that correlated
set by independent component intervals before the next nonlinear map. This can
invent positive yaw/heading responses near rest and fail a wall certificate even
when a tighter enclosure of the original input population is clear.

The frozen witness is single-r9 source1986/control1630 at29.914999331. Its cold
source solve and physical plan certify, but the unchanged scheduled program
`(+1.3296284675598145,-0.5010858774185181)` followed by
`(-3,-0.5259549617767334)` fails at30.209999331. The source fingerprint is exact;
this is a counterfactual fresh-source evaluation, not exact live1343/source1402.
R621 identifies original occupied cell474070(row631,column189), centered at
(89627.593870164274,43129.85165326744), with the unchanged0.1m grid and full
wall footprint(1.615,0.51,0.968,0.968,0.05). The unit fixture isolates this cell;
the full-world R616/R617 replay separately checks all walls and full Stop.

Producer: `detail/mpcc_vehicle_enclosure.hpp::centered_step/advance_partitioned_inputs`,
consumed by `mpcc_applied_input_prediction.cpp::predict_inputs_to_rest`.
The wall checker correctly refuses an insufficient enclosure; do not remove it.
The earlier observation/body/contact mismatch remains separate and unresolved.
No evidence here proves every persistent stop shares this cause.

## Bounded comparisons

- R607: checking every pre-merge native branch's paired body/corners still rejects
  at30.209999331. Last-stage subdivision alone is rejected.
- R608–R613: read-only CIL establishes actual Domain routing through
  MultiDomainROS2Manager.Update → DomainContext.SpinOnce → MultiDomainSubscription.Take
  (up to64sequential takes/callbacks) → native md_take → rcl_take. The receiver
  keeps only the latest pending longitudinal value under its lock. General ROS2cs
  SpinOnce also serializes callbacks but is a different routing path.
- The official CycloneDDS0.9.1 user-data reorder mode discards old sequence numbers;
  ALWAYS_DELIVER is reserved for a special built-in stateless participant writer.
  This supports conditional single-writer ordering, but the bundled version filename
  and container default do not establish the library actually loaded in single-r9.
  Reset/other-writer scope remains unproven. No FIFO assumption is promoted.
  [Official reorder implementation](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/0.9.1/src/core/ddsi/src/q_radmin.c),
  [official mode selection](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/0.9.1/src/core/ddsi/src/q_entity.c).
- R610: three monotone acceleration phases still reject at30.254999331. Ordering
  alone is insufficient. Original steering intervals remain independent.
- R612: retain every allowed ordered acceleration history;1641cohorts/59442updates
  certify the full world.0.715604s is the whole diagnostic, not isolated prediction.
  This conditional offline witness exposes history-merging loss and cannot execute.
- R614/R615 are failed diagnostic builds (field name, unsupported interval division),
  with no physical results. R616 fixes setup and uses one equation-derived velocity
  frame, all original arbitrary input sequences and unchanged speed bins. Full
  scheduled wall/dynamic/rest certification succeeds, without assuming ordering.
- R617 retains that exact map.256native traces include arbitrary acceleration
  switching, endpoint and interior steering.153600body and614400corner endpoint
  comparisons all pass without added tolerance.75steps/4partitions reach complete
  rest at30.274999331. Isolated prediction7.175861ms is diagnostic evidence;
  it does not establish total callback or live timing.
- R618 cannot compare old domains after rebuilding them with a new enclosure.
  R619 rebuilds the historical immutable source with its original producer and
  enables the new representation at current dispatch. Force-r2 andsingle-r6 still
  wall-reject; single-r5 still fails its real before-publication deadline; single-r8
  remains RestExpired. The two wall rejection epochs change by one5ms step and are
  not claimed bit-identical. R619/R620r76 setup fails on missing newer metadata;
  R622 uses the existing original-schema reader and preserves r76D1current3999
  wall rejection at98.614997805. Source/domain/history identities remain exact.

## Integration counterexample and paired enclosures

Build124passes26packages with fixed sources. Host source/architecture tests pass112
with the CMake-declared PYTHONPATH (initial host collection failure is retained).
Package111reports2621records/9reportedfailures: four actual existing moving-Stop
positive tests fail, each repeated in result formats. A single actuation frame is
therefore insufficient and is not promoted despite R624's53native tests passing.
No original positive expectation, wall geometry or threshold is removed.

The next structural representation propagates both original physical axes and the
fixed actuation axes independently through every identical input and native step.
If the true reachable set S is contained in both Ephysical and Eactuation, then it
is contained in their intersection. Intersect corresponding physical publication,
swept and endpoint body boxes and world-corner boxes before the one unchanged
validator. Do not feed a newly intersected endpoint into an unrelated swept range,
choose a successful validator, change inputs, or replace either propagation with
a point witness. Rest is tested on the common physical endpoint after the original
full-program/input-expiry floor. Partitions retain the original speed thresholds;
maximum_body_partitions records the maximum within one coordinate frame.

This retains the original representation's information in moving scenes and the
actuation representation's correlation at launch. Both must remain numerically
valid; an empty intersection or failed propagation rejects. Timing, the four old
moving positives, new isolated-cell regression, native containment and five
negative worlds must pass before any commit or live run.

R625paired propagation restores all four moving-Stop positives and passes all53
native vehicle tests. Two prefix-parity tests then expose a missing consumer:
`mpcc_starting_domain.cpp::predict_pending_input_prefix` still used only physical
axes. Preserve the exact equality contract by using one AppliedInputPopulation
for short and full predictions. Remove the duplicate physical-only short-prefix
loop; retain its original clock/input/identity handling and the independent
starting-domain producer. Do not relax equality assertions or adopt a wider prefix.

## Numerical repair

Let `(Gu,Gv,Gr)` be the native acceleration-force coefficients at the initial
physical tire angle. Define fixed binary64 coefficients `kv=Gv/Gu`, `kr=Gr/Gu` and
an invertible numerical frame

```
w = (u, vy - kv*u, r - kr*u)
(u,vy,r) = (w0, w1 + kv*w0, w2 + kr*w0).
```

Propagate the same native map as `T Phi(T^-1 w,a)`, with outward arithmetic and
the existing mean-value Jacobian enclosure. Keep this frame fixed throughout
one source-to-rest prediction. Gu/Gv/Gr are used only to select numerical axes;
no physical parameter, force, observed state or command is fitted or changed.
Forward velocity remains an axis, so every existing rest/reverse/speed partition
retains its original threshold and complete population. Cartesian body pose and
world-oriented footprint corners keep their original meaning.

Decode every public BodyRanges at publication, swept/endpoint and terminal output.
Keep explicit per-call frame values; remove diagnostic thread-local switching.
Compiled Jacobian maps must carry and match their numerical frame before reuse.
Physical-coordinate maps from the independent starting-domain producer remain
usable by their original consumers and cannot be mistaken for transformed maps.
The public tube/profile/model/schema/clock/fingerprint contracts remain unchanged.
The single certified nine-state authority and native scalar trajectory stay intact.

The paired implementation augments the original physical representation with
an independent actuation-coordinate enclosure in applied-input propagation. Original source-domain prediction and all hard validators
remain. All diagnostic-only maps/readers stay outside production. No margins,
thresholds, weights, solver budgets, rates, timing windows, hold/retry or normal
fallback change. DDS ordering investigation needs no further promotion work for
this chosen repair because the complete original input population is retained.

## Acceptance and remaining work

1. Add the exact isolated-cell regression and demonstrate failure against the old
   producer. Add native containment over rest/launch/reverse/full steering and an
   occupied-interior negative; numerical frame/cache mismatch must fail reuse.
2. Implement the typed fixed frame and physical output decoding. Review each
   caller/cache, build all packages and run meaningful package/source tests.
3. Re-run the frozen positive world with production code and the five negative
   cases, preserving historical-source identities. Check timing and all final
   authority/packet/deadline proofs. Past builds are not evidence for new code.
4. Local commit, then fresh bounded single-r10 with protected artifacts restored.
   Inspect restart, Start/laps, final authority, topics and callback timing before
   remaining intent/async/multicar/gates/submission M4–M6. No completion claim yet.

[Sealed diagnostics and hashes](velocity-correlation-evidence.json).

## Final pre-live validation

R626passes53vehicle and162retained tests after unifying the short/full population.
Build125passes all26packages with fixed source hashes; package112passes2621records
with zero errors/failures/skips. Source/architecture112pass with declared PYTHONPATH.
R627uses the current production numerical predictor in the original full-world
source1986counterfactual. Original nominal/dynamic/wall/input/program certificates
all pass;75steps reach rest30.274999331.256native traces/768000endpoint checks have
zero violations without added tolerance. Prediction alone takes14.047511ms, which is
not a live callback bound. R628/R629preserve all five historical negatives and exact
source/domain/current-history identities. Historical source reconstruction alone
uses its saved original producer; current dispatch uses current production code.

The implementation removes the duplicate physical-only short-prefix loop. The
physical numerical frame remains an independent complete enclosure and compiled
source-domain consumer; the actuation frame is explicit and immutable per call.
Both must succeed before the unchanged one validator. No extra normal authority.
Next: local commit and bounded single-r10, then remaining M4–M6 acceptance.

# Current observation contract (baseline28a8a660)

Continue the authorized M1–M6 work without a new approval. This checkpoint is
a structural implementation following the sealed comparison. The observation
membership API and empirical profile remain unchanged; the dispatcher may mint
a distinct, independently recomputed current physical proof as described below. Rollback base is28a8a660. All physical margins, model parameters,
receiver250ms/publication25ms/nominal130ms/steering100ms and budgets stay fixed.

R45 D1dispatch556 andD2dispatch536, replayr283, prove the current numerical
certificate is tied to a point-valued original observation. The original native
rest tube accepts; the next measured pose is outside it. Its exact membership
rejection is correct. A new sensor observation is not evidence of a mathematical
integration rounding defect. The independently fixed progress/ns representation
bugs do not solve this model/observation contract.

Public-message analysisr1 selects Ready-to-teardown using same-run wall receipt
times. D1/D2kinematic pose spans include3.21cm/0.087cmX and0.2853/0.00098rad yaw;
raw GNSS pose is separately retained. IMU reports zero angular covariance, while
velocity and steering reports have no covariance field. Kinematic yaw variance
is about0.47–0.51rad²; GNSS pose covariance is a configured0.1/0.1/1.0. These
values do not supply a deterministic bounded sensor model. Do not turn an
arbitrary multiple of covariance, or this failure's largest delta, into a hard
physical error guarantee. No new numerical tolerance follows from this analysis.

Compare these observation/proof representations on the sealed original inputs:

1. Existing proof plus strict current measurement membership: reproduced reject.
2. A completely independent current physical proof of the exact original common
   packet programme. Use fresh original component stamps, actual history and
   current world, unchanged packet epochs/values/margins and full rest. Give the
   resulting population its own fingerprint; never relabel it as the old tube.
   Measure its complete cost. This is a diagnostic, not permission to revive
   r193/r194's reanchored cache or to skip original prefix/intent/serialization.
3. An explicitly set-valued observation model whose full population is propagated
   and physically certified. It needs independently justified sensor/process
   bounds and treatment of sensor epochs; reported covariance alone is insufficient.
   This arm is currently inconclusive, not an accepted uncertainty profile.

For arm2, a programme whose first nominal epoch is already in the past needs a
typed unissued-packet window restricted by actual now and the original deadline.
Backdating an observation, retiming later packets, or extending its window is
invalid. Start the comparison only on captured cases where the original first
epoch is still future, and report that domain explicitly. Both old and fresh
world checks must preserve source horizon speed limits and full-rest conditions.
Nominal identity and physical population identity must remain distinct.

Next detection obligations: capture the first Ready/Start admission failure,
because r45's bounded recorders used their stationary bucket before Ready; capture
current-observation-unavailable with its failing producer. Do not delete the
existing startup evidence. Mission/geometry/sibling/Store/positive-source-horizon
Stop and full M4–M6 are still open. Local tests are not integrated acceptance.


## Implemented boundary

The only normal dispatcher, `prepare_dispatch`, authenticates one current
request and the complete actual prefix. The old strict measurement API still
returns the original mismatch. Only pose/velocity/yaw-rate/tire membership
mismatches may enter a new whole-population proof. Invalid observations, clock
regression/expiry, input-delay changes, issued-steering mismatch, regressing
component stamps, or changed values at identical component stamps reject.
Generation, problem semantics and geometry, model, physical bounds, footprint,
Follow source branch and fresh hard gap, current walls and every peer stay gated.

`PendingInputTube` is explicitly distinct from ordinary/scheduled populations.
Its exact unissued original suffix retains every original packet value, epoch,
period and deadline. Actual sends are removed from the future suffix only after
ledger authentication, and remain in actual input history. For a callback which
has already entered the original publication window, retaining the entire old
window is a conservative overapproximation of possible responses; it does not
claim a hypothetical packet actually left ROS. No observation is backdated and
no new grace or deadline is introduced. The first original deadline still binds.
The original receiver profile and numerical body/corner/rest kernel are shared.

`CurrentPhysicalProof` has a private constructor. The sole dispatcher owns its
fresh request, independent tube/hash and original suffix index. The ledger's
original job/programme fingerprint remains stable across actual suffix sends;
the new physical population has a separate hash in the dispatch candidate and
publication telemetry. Actual raw before/after clocks must fit the original
window and both old and newly proved rest horizons. Revocation, exact wire and
actual predecessor slew remain mandatory. This is not r193/r194 cache reuse:
there is no reanchored old population and no partial physical checking.

Startup and active-session admission/proof recorders have separate bounded
slots. Unavailable current-input producers save the raw observation and the
failing producer; final-boundary snapshots identify any independent proof and
its exact suffix. Evaluation JSON/schema/contracts are unchanged.

## Causal evidence and acceptance

R45's first point-membership failures remain reproducible in the strict API.
Observation-analysis r1 cannot derive a deterministic error bound from reported
covariances. R288 independently proved the unchanged original future programme
in both saved cases (about6.8ms each); this alone granted no production authority.
R289 exercised108 scalar publication schedules including callbacks inside the
original window, with every native body/rigid-corner endpoint enclosed through
full rest. R290 had136/137 tests passing and one invalid test setup; r292 passed
137/137 after correcting that setup. R291 failed diagnostic compilation on a
new observer member-access typo, subsequently corrected. These failures are
retained and not reported as passes. Final replay/build/test/runtime results
belong in the companion evidence and active tasklist once actually observed.

The replaced restriction was unconditional membership in the old point-valued
population at current dispatch. Original source proof, strict compatibility API,
actual ledger and single normal authority remain. No alternative fallback,
positive hold, tolerance/profile/parameter/budget/rate change was added.
All M4–M6 integrated acceptance remains open until actual simulation and
submission results pass. Rollback is local commit28a8a660.

Final local validation: r293 production-dispatch reconstruction accepts both saved
point-mismatch cases in6.872/6.867ms with distinct physical hashes; original
appointments accepted and outside-deadline sends rejected. R294 passes138native
tests including fresh Follow hard-gap rejection. Buildr86passes26packages;
testsr74passes2530records/64groups, zero errors/failures/skips. Dev2-r46 is next.

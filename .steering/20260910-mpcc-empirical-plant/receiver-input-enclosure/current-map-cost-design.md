# Current proof cost after restored moving source supply

2026-09-12 JST. Baseline/rollback `11b93527`. Authorized autonomous M1–M6;
local commits only. Production slice implemented and locally validated; integrated acceptance remains open.

Standard dev2-r71 starts both cars. D2 source admission and motion are restored:
215 positive publications, maximum measured speed1.319033861m/s, longest72
consecutive positive packets spanning1.775s. D1 reaches1.065380335m/s and57
consecutive positive packets/1.4s. These are producer/bag observations, not actual
receiver application or Stop/rest/restart acceptance. No laps complete.

D1's first moving final-publication failure is decision1070. Original nominal
14.819999702, dispatch14.824999668, actual before14.854999667, original deadline
14.844999702. The proposed packet is braking−3m/s², wire steering0.2277864665rad.
The controller correctly rejects before sending that expired scheduled packet;
its distinct failsafe publication retains the upstream failure identity.

The producer of the late decision is synchronous current proof work:
- Due job1067/source514/index0 rejects after13.788687ms wall/11.605790ms CPU.
- Active job1064/source514/index3 accepts after15.229523ms wall/12.219044ms CPU.
- Both first reject their independently precomputed starting domain against the
  current wall, then perform the existing full current physical reproof.
- Callback30.280551ms/CPU24.958556ms; normal-primary29.503ms. Request construction,
  problem initialization, admission logging and Recovery are small here.

The earlier r70D1decision1529 has one accepted current proof20.471354ms wall /
16.829101ms CPU and a21.360712ms callback. It still exceeds its original remaining
ROS-clock window. Its later stationary1858 overrun is separate evidence. The
original integer25ms window,250ms receiver,130ms control origin,100ms steering
and strict pre/post-send checks remain unchanged. No deadline extension, old
point-future reanchor/reuse, resource/affinity change or alternate normal command
is proposed. Off-CPU time contributes; these observations do not identify its
entire cause.

R437's first driver abort is a diagnostic YAML nesting error, retained. R438
corrects it and reconstructs both complete original sources, exact retained
actual histories, current contexts and physical proofs. Both accept at the
recorded callback-entry clock and reject at the actual pre-publication clock.
Repeated isolated current proofs take approximately9.39/7.35ms including copied
observer code. Original/current samples are173/148; world checks account for
approximately2.74/1.34ms across all attempted domains/current samples. Isolated
latency is not the loaded runtime bound. R439 profiles the copied numerical
kernel: centered native state propagation dominates; input-bound lookup and
force-coefficient construction alone cannot explain the total.

R440 independently propagates six original-input domain representations on
both saved scenes: full, early time/full body, early time/early body, reserved
multi-window body/time bin, single original window with raw-inclusive body, and
single window with model-only body. Broad member populations still hit the
current wall. Single-window alternatives omit measured body components and
cannot authorize either prefix. No domain-width change is promoted. Previous
SIMD, force-column packing, coarsening and linkage experiments remain rejected
under their recorded conditions; they are not repeated as production fixes.

R441 tests a distinct numerical representation: store each mean-value native
step map computed for a freshly propagated independent starting domain. Its
parent interval Jacobian, interval-valued center image and branch information
can evaluate a contained current state/input box without repeating derivative
construction. Cartesian translation is still exact; all other state components,
input, step duration and model must match or be contained. A discontinuous parent
keeps its complete natural enclosure. Uncovered states use the unchanged fresh
map. These local equations are not old point trajectories or dispatch authority.

R441 reduces isolated prediction about7.34→3.76ms and6.73→3.39ms. However its
additional current-prefix/relative-body recomposition fails r71's wall check even
without compiled maps; that representation is rejected for promotion. R442
instead compares local-map evaluation inside the existing complete current
observation/history predictor, retaining its original frame, native prefix,
rigid-corner propagation, full rest, current-world and final publication checks.
No production implementation is justified until this comparison and independent
numerical containment/negative checks succeed.

Required next evidence: original-point/current proof parity or explicitly
conservative new enclosure with every physical/clock negative retained; strict
model/state/input/branch/map-owner checks; complete current body/corner/rest
oracle tests; bounded worker memory/time and main-thread cost; build/package
checks and committed standard dev2. M4 intent/Mission/sibling/Store, actual
Stop/rest/restart, Recovery/Rejoin/Boost/async, same-final-HEAD single/dev2×3,
dev3/dev4six laps, gate1–3 and same tar/image/eval remain open.

R442 completes both exact current-history cases. Unmodified complete reproof and
compiled-map reproof both accept current physics/world and retain entry-pass /
actual-before-fail. Isolated observed totals are about10.70→7.03ms and8.53→5.59ms;
these diagnostic copies are not a production timing guarantee. Map hits/misses
are350/177 and278/196; uncovered branches use the full unchanged calculation.
R70's conservative enclosure reaches rest one native5ms step later; the old
original source rest and packet deadline are not moved. Current-world checks
cover every resulting sample. R441's additional relative-current recomposition
remains rejected. R443 now checks restricted maps against the independent
100-decimal-digit shared-kernel oracle, including body and rigid-corner values.

Candidate implementation ownership, conditional on numerical checks:
- Explicit per-build recorder and immutable per-domain map storage; no diagnostic
  global state or mutable cross-thread cache enters production.
- Store maps while computing the existing independent full starting domain;
  avoid a second future propagation. Early child and Follow proof policies stay.
- A local map binds the model, native duration, moving/rest branch, every
  non-translational body coordinate and acceleration interval. Restriction uses
  an enclosing parent Jacobian and its interval-valued center; discontinuous
  parent maps keep their full natural image. Rigid-corner displacement still
  uses the current corner population and its actual coordinate frame.
- Query only after original source/context/actual-history authentication. An
  incompatible/missing map uses the existing full fresh native computation;
  it cannot waive any physical/current-world/rest/clock check.
- Bound optional stored equations to4096 records (about6.6MB of equation data),
  preserving complete parent propagation after storage is full. This limits
  optimization memory, not the physical proof horizon or solver budget.
- Expose an opaque immutable owner at the numerical API; keep derivative records
  internal. Do not store parent plan/source chains or use process-global maps.
- Replace repeated derivative construction for covered native steps only. The
  current proof and sole scheduled dispatcher retain authority; every original
  failure path and exact complete calculation for uncovered steps remains.


Implementation and final local validation: explicit immutable map owners replace
the diagnostic globals; bounded4096flat records are collected during the existing
full independent-domain propagation. The existing fresh-history predictor and
its coordinate/corner ownership remain. Focused444passes12domain+3Jacobian tests.
Build113passes26packages/5m; package99passes2592records/66groups,source105/C5 161,
40.36s. Built-library445preserves six original source/history/current decisions:
r70D1median9.831020→6.709019ms,r71D1median7.797523→5.188506ms,r63D2median
6.579902→3.396571ms; early/full domain-admitted cases retain their sub2ms timings.
Original recorded late guards still reject. Source/domain owners and all recorded
actual histories match. [Evidence](current-map-cost-evidence.json).
Standard committedr72 and all remaining M4-M6 work follow; this is not race acceptance.

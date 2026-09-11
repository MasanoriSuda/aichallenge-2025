# Scheduled certificate and pending C4–C5 integration

Baseline/rollback 6cf3b202. The scheduled nominal forecast passed buildr71 (26 packages) and testsr59 (2483 records/64 groups).
The normal authority is STILL synchronous. No deadline repair claimed.

1. Add exact programme composition: inherited old first index and count plus new
   terminal programme. Require equal interval/window/clock kind and exact new
   first == old publication_epoch(start+count). Preserve old values/source indices,
   allow its declared repeat tail only, cap total10000. Never retime any packet.
   Original actual history remains separate, authenticated by ledger cursor.
2. Introduce opaque ScheduledRetained/ScheduledCertificate type, holding original
   Request/Observation, explicit committed prefix, typed forecast/new-first index,
   nominal solved-source proof, composite full ScheduledInputTube. It must NOT
   convert to old Certificate or be executable through existing final adapters.
3. Reuse nominal retained engine through a private scheduled-evaluation parameter,
   built only by the new factory. Private future view has now=planned first and
   forecast state/path, but NO fabricated ObservationProvenance/publication_prefix.
   Generalize the few prefix-dependent reads to private typed access. Keep required
   old prefix/applied flags TRUE and old prefix/applied pointers ABSENT in any
   private nominal Proof; old adapters must reject it if it ever escapes.
   Do not simply set required=false and promote nominal-only proofs.
4. Private scheduled engine needs select_publication_actuation at future control
   origin, prior steering from last declared prefix (or actual ledger predecessor),
   nominal predecessor duration. Actual dispatcher later checks actual predecessor
   duration, may wait within original window for slew, never clip packet or add grace.
   Build future candidate first from old prefix with last held, then predict its
   proposed packet and re-run selection/binding. Return mismatch if it changes.
5. Progress projection needs immutable exact reference context. Live builder currently
   calls model->physical_course_progress(pose); capture its reference/association
   before worker. Do NOT reuse r229 diagnostic 1e-9 matching-frame association as
   production authority. Follow target absolute-progress semantics must survive
   shifting query time and control-origin reference. Original Follow world still
   owns actual observed_sec and forecast; future view is NOT a fresh observation.
6. Extract existing applied world checker once (same arithmetic/order) in its cpp:
   input immutable original origin, wall/footprint, source hard bounds, optional
   source velocity ceiling, peers, Follow absolute-progress origin/query epoch.
   Stats separate from Certificate private fields. Reuse for scheduled prediction
   from ORIGINAL now through waiting prefix and full rest. Retain all existing
   Follow/state/wall/dynamic checks. Existing r232 copy explicitly excludes Follow
   and is NOT production implementation.
7. Fresh adoption/dispatch world validation can recheck remaining saved numerical
   ranges under fresh peers/wall/Follow without repropagating or reanchoring them.
   Check full context/target/intent/geometry/schema/horizon/model/bounds/cost.
   Need genuine compatibility logic, not source age or target equality alone.
8. Fresh raw measurement compatibility is per sensor epoch: pose x/y/yaw at
   initial.source_sec; u/vy at velocity_source_sec; yaw-rate at yaw_rate_source_sec;
   tire at tire_source_sec. initial.state.desired_steering is last ISSUED command,
   NOT measured receiver desired state. Bind that via ledger, not tube-state[6].
   Holding components to common pose epoch is current empirical reconstruction;
   do not assert a fresh reconstructed population is contained from point tests.
   The old original tube remains its original population; reject inconsistent
   actual measurements rather than create a new population/cache.
9. Single dispatcher consumes ready scheduled proof, keeps publishing previously
   certified programme including its Stop while worker computes. Account bootstrap
   Emergency prefix explicitly with sourceNone; absence of a new result never
   grants positive hold. Keep old authority until promotion, then retire old sync
   authority in that same slice. LatestOnlyWorker supports one-running/one-pending
   and generation cancellation; exact prefix/context/reset must gate adoption.
10. Every actual send remains one ledger transaction with before/after raw clocks.
    Strict future before>=nominal, after<=latest. ROS ticks can be1ns before a
    nominal25ms boundary: wait for real admissible time, no1ns tolerance. Clock
    reset/admin/session/shutdown invalidates jobs/cursors. Historical ledger only
    detects observed raw regressions; it cannot claim unseen jumps are detected.

Useful locations:
- mpc_controller_cpp.cpp build_rate_resolved_current_world_request at26636
- retained cpp publication_prefix_consistent695, evaluate_with_stop_profile780,
  constant programme1470, applied certificate1752, evaluate_stop_candidates1771
- applied cpp certify_terminal_stop260, shared physical check350 onward
- node raw measurement assembly~55000, all final pubs~55080/55155/59375


Current implementation status:
- Exact prefix composition implemented. r240 passed60tests; it preserves inherited
  wire values and epochs, rejects clock/phase/window changes and undeclared tails.
- Shared AppliedWorldCheck preserves ordinary world arithmetic/order. r241/r242
  passed96existing retained/Stop/Follow tests. No ordinary type layout changed.
- Opaque ScheduledNominal/ScheduledCertificate implemented. The nominal engine
  receives a private typed forecast, then the original-observation full body and
  corner proof checks waiting inputs plus the complete new terminal programme.
  It never supplies an ordinary Certificate; the diagnostic proof is removed.
- r243/r244 exposed noncanonical timestamps in the test fixture (multiplying by
  1e-9 instead of division by1e9). The fixture now emits exact integer-ns seconds;
  strict production clock validation was not relaxed. r245 passed100tests.
- r246/r247 reproduced two new-factory defects before runtime promotion:
  shifting the Follow gap erased an original hard-gap violation; nominal peer
  queries omitted waiting time and recorded-0.055m for an already departed peer.
  Original Follow validity/hard-gap checks now run before shifting. Scheduled
  nominal dynamic queries add now-observed_sec without changing obstacle epochs;
  ordinary queries/arithmetic remain unchanged. Original full-body world checks
  already used absolute elapsed time and remain the final physical gate.
- r248 passed103tests/508ms: Track/Cruise/Follow, exact old Stop prefix,
  waiting-only contact, moving peer timing, original Follow violation, identity,
  circular progress, history/clock failures and rejection by ordinary adapters.
- r249 diagnostic mistakenly used the100ms steering application delay as nominal
  prediction duration. Future12arms accepted, zero-lead4moving arms rejected.
  This is not the captured130ms control-origin configuration and is not accepted
  as its replay evidence. No production/config change resulted.
- r250 reads the captured130ms state_prediction_delay_sec and verifies its exact
  original origin arithmetic. Six r40inputs x0/1/3old packets =18accepted full
  proofs,9.19–14.31ms. Every input had exactly one matching captured course frame.
  This exact-match frame is a diagnostic hypothesis only; live code must capture
  model.s/current_waypoint directly. Actual old sends and fresh context are not
  established by offline evaluation. No timing/authority acceptance is claimed.
- Buildr72 passed26packages/4m59s; testsr60 passed2493records/64groups,0errors/
  failures/skips,32.1s. Quality hooks passed; C++warnings absent, existing setuptools
  deprecations only. r251–253 preserved all21legacy classifications, including the
  r18history gap rejection. C4fresh sensor/world and
  C5single async dispatch remain unimplemented. The existing synchronous normal
  authority must retire in the same slice as scheduled authority promotion.

Live progress projection uses model.s + (cos(frame.yaw)*dx + sin(frame.yaw)*dy)
from current_waypoint. Capture its exact frame, including parenthesization. No
old Request layout extension or inferred live association is needed.

[471ファイルの保存証拠](scheduled-certificate-evidence.json)。


最終レビューで、normal-path Stop参照が作れない分岐のscheduled context引数漏れを修正。
R254の最初のfixtureはvirtual-progress dynamics不整合で拒否され、採用しない。
R255は整合する0/4mpsの仮想進行速度で参照を作れない事例を用い、修正前は
publication-prefix-unavailable、修正後は元の待機中peer違反として拒否されることを確認。
全104native test合格。専用回帰をpackageへ追加した最終buildr74は26packages、
testsr62は2494記録/64groupでエラー・失敗・skip0。r72/r60とr73/r61は中間検証。
R250の18証明・r251–253の21判定は、このscheduled分岐の引数修正前の証拠として保持。
この修正はordinary呼出しの引数・演算を変えない。

# Pending C4 and C5 after scheduled full certificate

C3 compiled/native/replay; do not infer runtime authorization. Existing synchronous authority remains.

C4 sensor verification against original ScheduledInputTube:
- Keep original population; this is observation consistency/falsification, NOT a
  fresh population cache or proof reanchor (r193/r194 remain rejected).
- Observation.component time equal original -> require exact same raw component.
  This permits repeated unchanged older velocity/IMU/tire samples which precede
  the old common pose epoch. A different earlier epoch must reject.
- Newer component -> find existing native swept interval at its source epoch;
  exact endpoint can use endpoint range. Pose x/y/yaw at pose_source; u/vy at
  velocity_source; r at yaw_source; tire at tire_source. No single common time.
- Desired state is last ISSUED physical angle, not measured receipt: compare its
  exact serialization with the actual ledger predecessor. Node keeps pre-wire
  physical command, so comparing raw desired to wire/gain directly is incorrect.
- Observation actual history must match current ledger. Worker snapshot must
  match original history exactly; ledger.matches_prefix must authenticate every
  expected old packet/source/index/raw bracket. Missing/extra send/reset reject.
- No post-rest or uncovered source time accepted. Before-send strict raw window
  and actual steering predecessor duration must still gate every dispatch.

C4 current world:
- Reuse original numerical swept ranges and rigid corner ranges. At partial
  interval use begin=max(sample.begin, fresh.now), unchanged whole swept box;
  this conservatively bounds remaining states. Never query a fresh peer at a
  negative elapsed time. Recheck through original complete rest.
- Fresh Follow forecast must use its actual observed epoch and current source
  course reference; prove circular/branch association. Old lap offset alone is
  insufficient across a wrap. Do not infer fresh control-origin predicted
  population containment from raw point consistency.
- Require full context compatibility: same model/profile/static course/intent/
  target/side/mission/schema/horizon and current bounds/cost policy, not target
  or age alone. Current MpccProblemContext string schema IDs are fixed strings;
  they do NOT by themselves hash changing cost/bound values.
- Existing make_owned_tactical_snapshot owns reference/model/gap/planner for
  worker use. Existing mpcc_lite_async_context_epoch is a tactical epoch; inspect
  actual invalidation coverage before reusing it as normal dispatch context.

C5 integration:
- Queue full scheduled factory off control callback with explicit prior Stop
  prefix. Continue old proved programme, including its Stop. No new result never
  permits a positive hold; bootstrap Emergency prefix is explicit sourceNone.
- Node default state_prediction_delay_sec is .13; nominal steering application
  is .1. Never confuse them. Capture configured .13 as an integer duration and
  derive future origin with ns addition, retaining all actual observed stamps.
  Same-epoch original origin retains old double arithmetic.
- Preserve immutable source/job decision identity separate from current dispatch
  cycle ID. New final adapter may not fabricate an ordinary Certificate or
  backdate actual execution clock. Retire old synchronous normal authority in
  this same promotion slice; do not just add a competing final writer.
- Lifecycle cancellation on clock/admin/session/target/intent/schema/course reset,
  shutdown; current ledger only detects observed raw regression, not unseen jumps.
- New actual ledger source bookkeeping carries planned packet index and full
  input context while recording before/after raw clocks; post-failure invalidates.

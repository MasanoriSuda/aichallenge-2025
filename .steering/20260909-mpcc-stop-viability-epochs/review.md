# State epoch repair review

Earliest producer inconsistency: raw pose/speed/yaw atsource were passed to an
actuator predictor beginning atnow. Exact native constant-motion invariants fail
for5/15msage. Consumers also bind canonical prefix-start to raw safety pose;
that owner must change with the producer. Preserve raw contact/Recovery owner.

MotionObservation is stamped. Existing constant-twist model spans onlysource-to-
now, holding body speed/yaw; existing committed-input/yaw-response predictor spans
now-to-control. No claimed identification of acceleration or physical steering
phase. Invalid/backward/stale input fails within the existing observation limit.
Canonical current execution pose and raw wall-monitor pose have distinct owners;
async snapshots copy both. Prefix start/end checks stay strict; physical world
current_pose joins the estimatednow prefix. Disabled prediction retains old raw
path. The diagnostic missing-steering path uses the same aligned start and gains
no authority. Historical shadow/raw wall diagnostics are not current certificates.

Native34cases and104source contracts pass. First full build had a ROSPose2D
constructor error, preserved; field-assignment repair builds26packages. Final
2381records pass. Single6laps/0penalty/max15.327ms/0overruns. Dev2still rejects
movingD1Emergency930; no callback overruns. Three recordednowposes exactly match
the declared estimator, with original source receipt ambiguity explicitly limited.
No command/model/constraint or evaluation/ROS interface was changed. Model accuracy,
steering source age, actual command sampling/application and full MPCC acceptance
remain open. Rollback4b2474da; preserve all failed and accepted evidence.

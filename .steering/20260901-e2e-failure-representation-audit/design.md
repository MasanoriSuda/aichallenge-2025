# Design

Relabel the two closed-loop authority runs with the frozen candidate3 and
`LidarPrecontactTeacher`:

```text
20260901-175609 single, completed -> train
20260901-180313 NPC, wall failure -> validation, causal pre-contact prefix
```

Append that immutable source to the existing recurrent corpus without moving
any existing sequence between splits.  Extend the diagnostic action probe with
raw-scan variants.  Raw LiDAR is normalized once from metres, while history is
reset at every sequence boundary and uses only causal lags 1 and 8.

The probe predicts left/neutral/right successor correction only.  It does not
alter ROS runtime or produce weights.  Every validation sequence includes a
last-200-sample metric (about 10 seconds at 20 Hz), and an optional focus token
selects the rejected authority failure for an explicit report.

# Design

Add `AsyncPreentryHomotopy` as an explicit Gate A tactical-input source.

For an idle execution state, tactical input resolution is:

1. a valid same-cycle current-world Overtake Mission;
2. otherwise, the accepted async branch selection and its certificate-stripped
   Mission geometry;
3. otherwise, no Gate A input.

Active ShiftOut/Pass replacement ordering is unchanged and may not consume the
new-entry async hint.

The async source sequence is carried into the draft and checked again when the
Gate A result is consumed.  Gate A remains causal: the background producer
rebuilds the seven-state problem from its owned current-world snapshot and the
serialized predecessor command, then recreates all physical certificates.

This repairs a scheduling/lifecycle defect.  It does not relax feasibility or
promote the tactical worker to production authority.


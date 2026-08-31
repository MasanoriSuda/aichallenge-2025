# Design: Return worker isolation

Add one `LatestOnlyWorker` and one private seven-state `SolverContext` owned by
the controller for `ReturnGateA` only.  Select the lane once from the immutable
draft kind before submission.

The existing mailbox and monotonically increasing sequence remain shared.
Its publication guard already prevents an older result from replacing a newer
published sequence.  Therefore this change removes only scheduling
head-of-line blocking; it does not create another authority or result store.

Deferral telemetry reads the Return lane statistics.  Consumer telemetry
reads the lane matching the completed result kind.

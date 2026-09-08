# Official information boundary — 2026-09-09

Reviewed the official [SW rules](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html),
[interface](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html),
and [simulator specification](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html).
V2X is other-vehicle position information. The simulator page confirms the
topic and variable100–200ms delay. The interface page contains no V2Xentry;
these reviewed pages do not establish a peer attitude/plan observation contract.
Local V2XVehiclePosition contains header/id/position/covariance only. Keep exact
wire/coordinate/sample-clock questions open; no other-domain raw-topic access.

Rear information may support safety checks, but must not cause obstruction or
unnecessary slowing. This reinforces the need to explain the coupled terminal
failure; it does not permit dropping the rear peer from physical proof.
Official safety gates are now recommended rather than mandatory. The existing
repository completion gate is unchanged. Scope-limited corrections appear in
V2X/open-questions/competition-rules/safety-gates docs; this is not a complete
re-audit of every older competition date, ranking or vehicle parameter entry.

One attempted /faq/URL failed to open; no finding relies on that URL. The
published fixed vehicle dimensions/GNSS table must not replace the already
measured local collision mesh and reference-point calibration without checking
the binary/version differences. No production/config change from this review.

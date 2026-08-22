# Design

## Causal hypothesis

The five-state QP constrains the vehicle centre in Frenet lateral bounds. The final certificate
samples the complete yawed kart footprint and the swept path from the actual pose. A centre-state
solution can therefore be QP-feasible while its rotated corners or its current-to-stage transition
touch a wall. The existing string-only result discards the evidence needed to prove which mechanism
caused each rejection.

## Contract

`PhysicalWallCertificateDiagnostic` records:

- a stable rejection enum;
- first failing stage and waypoint;
- cumulative horizon distance;
- solution lateral position, lower/upper bounds and bound reserve;
- derived heading offset and sampled world pose;
- map/out-of-map/contact-cell evidence;
- swept-path rejected index and checked-pose count.

The existing validator remains the only pass/fail authority. The diagnostic is an optional output
and cannot turn a failure into success. Status-change logging formats the typed diagnostic, avoiding
the previous lossy free-text boundary without increasing normal log frequency.

## Decision after replay

- Positive QP bound reserve plus physical contact indicates the QP corridor is not footprint-safe.
- Near-zero/negative reserve indicates a solver/bound-tolerance problem.
- Swept-only failure at the first path segment indicates current-state stitching/reachability.
- Invalid/out-of-map samples indicate a map or coordinate-contract problem.

Only the evidenced category may define the following structural slice.

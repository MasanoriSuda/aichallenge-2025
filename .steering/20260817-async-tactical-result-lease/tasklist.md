# Tasklist

- [x] Add pure lease validation and adaptive duration helpers.
- [x] Cache and reuse an accepted async tactical result across control callbacks.
- [x] Clear the cache when the asynchronous planning context is invalidated.
- [x] Add unit tests for freshness, context and hard-fault revocation.
- [x] Build the ROS 2 package.
- [x] Run the package tests.
- [x] Commit only implementation, tests and steering documents.

## Definition of Done

- A result accepted at 5 Hz remains available to intervening 40 Hz callbacks.
- A due side/replan evaluation can consume the leased result instead of empty
  async-owned assessments.
- Target/context/phase/side mismatch, excessive age and hard faults revoke the
  result.
- The execution lease covers normal worker cadence but never exceeds the
  configured tactical-result maximum age.
- Existing tests and new focused tests pass.

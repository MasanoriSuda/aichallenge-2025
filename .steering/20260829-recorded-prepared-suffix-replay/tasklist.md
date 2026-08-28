# Tasklist

- [x] Round-trip the recorded `assembly_request` through the public v2 loader.
- [x] Reject malformed assembly payloads without weakening v1 boundaries.
- [x] Add observation-only prepared-suffix replay CLI.
- [x] Run every replay-ready v2 frozen snapshot. The current corpus contains
  seven ShiftOut failures and no Pass snapshot; this evidence gap is explicit.
- [x] Compare build/solve time with full semantic pipeline compute.
- [x] Run full build and package tests.
- [x] Decide live-shadow eligibility from the recorded evidence.

Decision: do not add a live shadow or change production authority. Time-aligned
prepared-QP construction is computationally cheap, but it does not rescue any
frozen failure. Continue with candidate-generation/single-SQP classification,
not another connector policy.

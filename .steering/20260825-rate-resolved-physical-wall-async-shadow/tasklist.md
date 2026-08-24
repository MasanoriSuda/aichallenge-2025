# Task list

- [x] Extract a pure rate-resolved physical wall proof request/result.
- [x] Give the immutable wall grid shared lifetime ownership.
- [x] Reject the separate-worker design after dynamic scheduling regression.
- [x] Serialize solver, adapter and wall proof in one latest-only worker.
- [x] Add full-identity provenance mailbox and stale/superseded rejection.
- [x] Remove synchronous proof execution from result consumption.
- [x] Add deterministic unit and source-contract tests.
- [x] Run full build and package tests (25-package build; 46/46 package tests).
- [x] Commit the implementation.
- [x] Run bounded `make dev2` dynamic Gate (`output/20260825-050500`).
- [x] Record acceptance or the next typed blocker.

## Dynamic Gate evidence

- D1: 405 submitted, 310 consumed/current-semantic, zero rejection, zero
  worker rejection and zero callback overrun; maximum proof 2.372 ms.
- D2: 6313 submitted, 5109 consumed, 4914 current-semantic, zero submission or
  worker rejection; maximum proof 19.258 ms in the worker.
- One D2 callback overrun occurred at decision 6419 (30.326 ms). Its region was
  `mpc=29.207 ms`; the production Track/Cruise report in the same window shows
  solve max 23.282 ms and synchronous production certificate max 13.338 ms.
  This is not execution of the new shadow proof on the callback and is retained
  as a separate production timing blocker.
- No additional normal authority or command path was introduced; the new
  certificate remained `authority=shadow, selected=0`.

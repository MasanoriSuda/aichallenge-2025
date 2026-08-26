# Tasklist

- [x] Freeze HEAD, run, domain and first post-start decision.
- [x] Prove solver, physical and current-world admission precede the failure.
- [x] Add and demonstrate a failure-first production-adapter test.
- [x] Implement certified physical projection at the single adapter boundary.
- [x] Verify values outside the sealed certificate still fail closed.
- [x] Run focused, package, source-contract and workspace tests.
- [ ] Run moving dynamic acceptance.
- [x] Update audit and migration documentation.
- [x] Commit without staging the user-owned result JSON.

## Static evidence

- Failure-first focused test: 2 expected failures before implementation.
- Focused production-adapter test: passed after implementation.
- `make autoware-build`: 25 packages passed.
- Complete package CTest suite: 51/51 passed after rebuilding stale test
  binaries against the current physical-trajectory ABI.
- Source authority contract: 56/56 passed as part of the package suite.
- No parameter, fallback, feature flag, timeout, lease or normal authority was
  added.

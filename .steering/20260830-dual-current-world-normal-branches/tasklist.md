# Tasklist

- [x] Freeze baseline, run, decision and immutable comparison result.
- [x] Trace producer, Store, retained proof and publisher ownership.
- [x] Add failing tests for an atomic same-epoch dual branch bank.
- [x] Implement and test the branch bank.
- [x] Evaluate both Cruise/Follow branches before selection.
- [x] Remove the preferred-first early-return producer edge.
- [x] Revalidate an untried bank branch only after ordinary sources fail.
- [x] Add authority/fingerprint/side diagnostics.
- [x] Run focused tests and source-contract checks.
- [x] Run the complete package test suite.
- [x] Run `make autoware-build`.
- [x] Run a dynamic multi-vehicle trial and compare with the frozen failure.
- [x] Compare qualitative behavior and timing with `.steering/ano`.
- [ ] Observe a natural dynamic run where the ordinary source fails and the
  same-epoch alternate branch passes exact production revalidation.
- [x] Record results and commit only intended files.

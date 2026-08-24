# Task list

- [x] Classify the previous dynamic rejects and identify duplicate ownership.
- [x] Trace semantic current steering, QP rows and solver certificate.
- [x] Add the explicit certified-sample contract.
- [x] Connect the shadow evaluator without authority change.
- [x] Add certificate, timing and sampled-bound tests.
- [x] Run build, full package tests and authority audit.
- [x] Commit the static Slice (`9a5428f`).
- [x] Run `make dev2` and compare the rejection distribution.
- [x] Record the next root-cause decision.

## Definition of Done

- One component owns each actuator invariant.
- No clamp, tuning, fallback or new authority path is added.
- A non-certified QP can never use the certified sampler.
- The ten minority timing rejects are not hidden.

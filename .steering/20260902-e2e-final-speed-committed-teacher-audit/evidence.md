# Evidence

Generated run (not committed):
`output/20260902-e2e-final-speed-committed-teacher-all-v2`.

## Immutable setup

- world: four peer vehicles, no NPC (`e2e-final`);
- runtime mode: `speed_committed_teacher` in domains 1 through 4;
- acceleration: `0.8 m/s2`;
- fixed-mode speed governor: explicitly disabled with `0.0 m/s` because the
  teacher owns its causal speed policy;
- production v11, its checkpoint and all production launch defaults remained
  unchanged.

The first launch attempt was rejected before motion because the participant
launch forwarded the production-only `4.6 m/s` governor to teacher modes while
the shell wrapper rejected the controller's documented diagnostic value
`0.0`.  The startup contract now accepts finite non-negative values and every
teacher Make target explicitly exports `0.0`; fixed production modes retain
the packaged `4.6 m/s` default.

## Strict race result

| Domain | Finish | Penalty | Laps [s] | Mean / max speed [m/s] | Stall [s] |
|---|---:|---:|---|---:|---:|
| d1 | 6/6 | 0 | 82.399 / 64.069 / 62.994 / 63.024 / 66.048 / 62.759 | 4.779 / 6.261 | 0 |
| d2 | 6/6 | 0 | 71.041 / 63.059 / 63.369 / 64.969 / 62.509 / 62.684 | 5.028 / 6.640 | 0 |
| d3 | 6/6 | 0 | 76.433 / 61.634 / 62.019 / 65.179 / 62.389 / 63.489 | 4.909 / 6.417 | 0 |
| d4 | 6/6 | 0 | 68.812 / 62.429 / 64.739 / 64.169 / 62.234 / 62.934 | 4.899 / 6.221 | 0 |

Every domain passed both post-start low-speed and positive-acceleration stall
gates.  The strict competition analyzer cross-checked one unambiguous result
entry per domain, runtime provenance, Finish, six laps and all three penalty
kinds.

Artifact hashes:

- result summary:
  `480d55e377eb277b4d319cc395113b925767e696385e544b5ce33ee93b2b4bc8`;
- strict competition analysis:
  `a41899daae2de204bdca07777299341a0ebccb3dfdbff1338e9d77cb7faf9703`;
- d1/d2/d3/d4 motion analyses:
  `cb77dbae295984fe5f45b8f0d30268e010fdde07911076297872ed347bd7ae82`,
  `89002fac02aeacbd91380941a712648882cce92abe58957f4cd417b8f3804013`,
  `b0f2fc140db191848fc353eeba721f2e78691308f94eb1ee46707b511bd6833f`,
  `3a35c34a6495fb4e835a555a797cb9562d1e0d93bede096c48872c29b27fbbc7`.

## Decision

The complete run is admitted as executed, outcome-qualified peer-interaction
teacher evidence.  It is a valid source for a later run-disjoint dataset Slice;
this Slice does not extract labels, train a model or promote runtime authority.

The run also exposed a recording-lifecycle issue outside teacher authority:
AWSIM reached `FinishALL` and wrote every result JSON, but the vehicle-side
orchestrators did not enter post-processing before manual `make down`.  Bags
were finalized cleanly on shutdown and all strict gates passed.  This issue
must be handled separately rather than being hidden in the teacher policy.

## Validation

- `bash -n aichallenge/run_autoware.bash`: pass;
- Make dry-run for the four-domain target: pass;
- launch/interface contract tests: 13 passed;
- motion/competition analyzer tests: 21 passed;
- complete TinyLidarNet workspace tests in the development container:
  221 passed;
- `make autoware-build`: 25 packages built successfully.

# Dynamic validation

## Run

- controller logs: `output/20260823-170154/d1/autoware.log`,
  `output/20260823-170154/d2/autoware.log`
- duration: approximately six minutes after controller startup
- production config and authority: unchanged
- user result SHA-256 before/after:
  `03e2f3935d95a550d0e1a3f2006dde08dae4a7a4c74121c430b2452daf4414e6`

## Observation

This run contained no eligible Follow shadow window. Domain 1 repeatedly changed from Cruise directly
to `LowSpeedAvoidance` when it encountered the slower vehicle and then returned through Follow with no
current relevant/front observation. Domain 2 likewise had no coherent front vehicle.

Counts from the complete logs:

- Domain 1 `Follow MPCC shadow runtime` windows: 0
- Domain 2 `Follow MPCC shadow runtime` windows: 0
- Domain 1 V2X behavior transitions: 23
- Domain 2 V2X behavior transitions: 17
- observed positive encounter action: `Cruise -> LowSpeedAvoidance`
- observed Follow shadow event: `not-eligible/no-coherent-front-observation`

No QP was attempted for an invalid semantic Follow observation. This is correct admission behavior but
provides no evidence for or against live warm-start application.

Every observed Follow shadow event remained `authority=shadow, selected=0`. The user-owned result file
was unchanged. The isolated simulator result artifact was not emitted in this run, so lap/penalty data
is not used as evidence.

## Conclusion

The run is **inconclusive**, not failed. The test scenario did not exercise the acceptance boundary under
evaluation. Production parameters must not be changed merely to manufacture a passing rate and the
Follow authority gate remains closed.

The code-level causal fix remains supported by build, unit tests and the full package suite. The next
positive dynamic evidence must use a deterministic moving/stopped-front Follow scenario or a replay that
preserves the target observation and stage geometry. Until such evidence exists:

- do not promote Follow canonical authority;
- do not delete the scalar Follow owner;
- do not tune OSQP or distance/wall parameters;
- do not interpret successful `LowSpeedAvoidance` passes as Follow evidence.

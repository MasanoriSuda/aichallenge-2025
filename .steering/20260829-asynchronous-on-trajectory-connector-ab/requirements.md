# Requirements

## Objective

Freeze production authority and determine why a freshly solved asynchronous
normal MPCC artifact cannot always join the command stream while the vehicle is
moving.

## Frozen evidence

- A time-aligned unpublished suffix can join while moving in the post-fix d2
  run.
- The first published artifact-local cursor is now preserved; a published plan
  no longer rewinds to its artifact origin.
- In the corresponding d1 startup, the asynchronous result is already roughly
  55--100 ms old.  Selecting that suffix expects steering from its skipped
  prefix, although those commands were never published.  The current publisher
  remains near zero steering and the actuator connector rejects the result.
- A synchronous current-state solve can produce a valid command, but its
  35--230 ms solve time is not suitable for the 40 Hz callback.

## Constraints

- Do not change production authority, solver tolerances, wall clearance,
  Mission leases, timeouts, or fallback policy.
- The comparison arm must be structurally unable to produce command or
  production-authority objects.
- Evaluate the same immutable candidate and current world; change only the
  unpublished execution-clock interpretation.
- Log one comparison per candidate sequence.
- Never infer that an origin-accepted candidate is safe to publish: its prefix
  did not cross the publisher.

## Exit classification

- Time-aligned suffix fails and origin arm succeeds: asynchronous connector or
  lifecycle defect.  Design a feedback/on-trajectory connector.
- Both arms fail for the same physical reason: plan/current-world infeasibility.
- Both arms fail for different reasons: inspect model/certificate identity
  before changing the solver.
- Live comparison is absent: scheduling, worker, or observability defect.

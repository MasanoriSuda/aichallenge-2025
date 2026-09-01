# Requirements

## Objective

Make an authority-disabled recurrent experiment observational: its inference
must not delay or modify the production TinyLidarNet command path.

## Constraints

- Preserve the production base, spatial authority, speed governor and
  longitudinal-safety command exactly.
- Automatically isolate recurrent execution only while recurrent authority is
  disabled.  Authority-enabled execution remains on the certified synchronous
  path until separately qualified.
- Use one bounded latest-wins worker.  Do not create an unbounded inference
  queue.
- A worker backlog may drop diagnostic samples but may not reset recurrent
  history or publish a stop command.
- Sensor watchdog and explicit controller resets remain valid recurrent reset
  boundaries.
- Report submitted, completed, dropped, stale and error counts separately.
- Do not loosen speed freshness, scan-rate, coverage, reset or race Gates.

## Definition of Done

- Unit tests prove deferred recurrent evaluation leaves the production command
  bit-identical to the recurrent-disabled controller.
- Unit tests prove a bounded worker replaces pending work and never grows an
  unbounded queue.
- Runtime logs identify the execution mode and async lifecycle counts.
- Existing synchronous authority and artifact-contract tests continue to pass.
- A single-vehicle and three-vehicle dynamic shadow rerun are specified before
  any authority reconsideration.

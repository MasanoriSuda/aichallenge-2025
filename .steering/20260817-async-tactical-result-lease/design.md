# Design

## Observed failure

The tactical worker runs at approximately 5 Hz while the controller callback
runs at 40 Hz.  `take_mpcc_lite_async_result()` consumed a result for only one
callback.  On the following callbacks both live side assessments returned
`tactical candidate generation owned by async worker`.  If a replan evaluation
became due between worker completions, it saw no current or alternate candidate,
so the last-feasible cache stayed empty.

The fixed 0.30 s receding-horizon lease was also marginal against a 0.20 s
submission interval plus worker computation and scheduling jitter.  The latest
run released otherwise valid execution authority at 0.33--0.51 s.

## Change

1. Store the latest accepted asynchronous result in the live controller.
2. Revalidate that cached result every callback against exact target, context
   epoch, Mission generation, phase, side, age and current hard faults.
3. Reuse its left/right assessments and candidate outputs until a newer result
   is accepted or the bounded lease is revoked.
4. Derive the execution-prefix lease from two asynchronous evaluation periods,
   the last measured worker compute time and two control periods.  Clamp it
   between the configured continuity lease and the tactical-result maximum age.

## Safety boundary

Caching does not make an old result executable by itself.  Existing current
curve, target, wall, target-bound, body-overlap, emergency and solver gates are
still evaluated by the live callback.  Cache reuse is disabled immediately on
context mismatch or a current hard fault.

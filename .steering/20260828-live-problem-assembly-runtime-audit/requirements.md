# Requirements

## Frozen failure

In `output/20260828-202620`, domain 1 initially publishes certified Cruise and
Follow artifacts.  Once start-grid dynamic/overtake evaluation begins, the
40 Hz control callback repeatedly takes about 58--66 ms against a 25 ms
budget.  The callback then publishes at roughly 16 Hz, while asynchronous
artifacts continue to be generated from older predecessor states.  Retained
current-world proof subsequently fails with `steering-unreachable` or
`actuator-envelope-rejected`, and canonical Emergency becomes the only owner.

The physical wall envelope cache remains above 97% hit rate, so a wall-margin
change is not evidence-based.  The upper-rank `ano` log instead keeps its main
GMPCC loop running while left/right tactical solves are sent to a non-blocking
child process.

## Required evidence

- Attribute synchronous `get_control()` time to problem setup, live behavior,
  pre-entry snapshot work, dynamic/gap planning, overtake-line update,
  remaining problem assembly, retained proof and successor-draft work.
- Emit diagnostics only for the production callback, not isolated branch
  workers.
- Preserve all control, safety, authority, solver and clearance behavior.
- Use one bounded dynamic run to identify a dominant synchronous region before
  changing scheduling ownership.

## Prohibited shortcuts

- Do not tune solver tolerance, clearance, lease, timeout, fallback or control
  frequency.
- Do not suppress the Emergency owner or bypass current-world certification.
- Do not assume asynchronous worker CPU load is the cause without measuring
  the production callback regions.

## Definition of done

- Unit/build regression gates remain green.
- A bounded `make dev2` run produces causal region timing for the frozen
  overrun.
- The next structural change is stated as one root cause and one owner move,
  rather than a new behavioral exception.

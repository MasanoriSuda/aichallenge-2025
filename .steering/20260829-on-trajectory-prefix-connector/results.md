# Results

## Static verification

- `make autoware-build`: 25 packages passed.
- package tests: 53/53 targets, 2083 assertions, 0 failures.
- single-authority source contract: 69 passed.

The connector module is observation-only.  It cannot build a command,
production authority or execution-store update, and the controller does not
consult `Result::accepted()` while selecting authority.

## Dynamic run

- Run: `output/20260829-025035`
- Mode: bounded `make dev2`
- Production authority, solver policy, wall clearance and behavior parameters
  were unchanged.

| Domain | exact state mismatch | parent cursor unavailable | connector accepted |
|---|---:|---:|---:|
| d1 | 23 | 39 | 0 |
| d2 | 64 | 0 | 0 |

For state mismatches:

| Domain | parent elapsed avg | candidate elapsed avg | abs lateral avg/max | abs progress avg/max | abs steering avg/max |
|---|---:|---:|---:|---:|---:|
| d1 | 0.1874 s | 0.0863 s | 0.0245 / 0.1278 m | 0.2691 / 0.8085 m | 0.0316 / 0.1470 rad |
| d2 | 0.0916 s | 0.0615 s | 0.0053 / 0.0258 m | 0.1737 / 0.8278 m | 0.0082 / 0.0481 rad |

The d1 unavailable parent cursor ranged from 2.420 s to 57.225 s
(average 25.059 s).  Once fresh candidate promotion stopped, the last
published certified trajectory was not an indefinitely valid parent.

## Root-cause classification

The async candidate is solved from a predicted observation while the publisher
continues another certified trajectory.  At result consumption time the two
histories do not terminate in the same seven-state physical state.  Selecting
a candidate suffix by elapsed time therefore skips unpublished controls; using
candidate cursor zero merely replays an old state.  Current-world proof is
correct to reject the unsafe cases, but repeated rejection eventually exhausts
the executable parent.

This is an **asynchronous preparation/feedback connector defect**.  It is not
evidence for a clearance, solver-tolerance, lease, timeout or Mission-lifetime
change.

## Architecture decision

A committed parent prefix is necessary for causal prediction but is not
sufficient: d2 demonstrates persistent small state differences even during
otherwise normal execution.  The chosen next architecture is an AS-RTI-style
latest-state feedback correction over the prepared seven-state problem,
followed by the existing exact physical and current-world certificates.

Before production promotion, an observation-only A/B must establish that the
feedback phase fits the callback budget.  Promotion must atomically delete the
elapsed-suffix-only adoption path; no second normal authority may remain.

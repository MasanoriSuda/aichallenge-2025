# Results

## Confirmed defects

1. Every unpublished asynchronous result was previously evaluated at artifact
   cursor zero.  While moving, that compared the current vehicle against a past
   cross-section and made the candidate joinable mainly after stopping.
2. After a time-aligned suffix first crossed the publisher, the retained clock
   restarted at elapsed zero.  The published plan therefore rewound.  The
   certified-plan store now records both the first publication control origin
   and the artifact-local cursor that actually crossed the publisher.

## Dynamic evidence

- Pre-fix: `output/20260829-013139`
- Time-aligned candidate: `output/20260829-014148`
- Time-aligned candidate plus preserved first-published cursor:
  `output/20260829-015514`

The d2 vehicle proves that both clock corrections allow moving candidates and
published successors to remain causal.  The d1 vehicle exposes the remaining
defect: the time-aligned unpublished suffix expects the steering state produced
by a skipped prefix which never crossed the publisher.

This remaining issue is not closed by another elapsed-time rule.  It is now
tracked by `20260829-asynchronous-on-trajectory-connector-ab`.

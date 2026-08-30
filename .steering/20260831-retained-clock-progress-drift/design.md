# Design: retained clock/progress drift audit

## Initial hypothesis

The persistent artifact cursor advances from publication wall clock while the
vehicle can fall materially behind the planned progress. Current-world rebase
then certifies a connection to a suffix several metres ahead without replacing
the artifact's time schedule. The same source is retained until its cursor
expires, causing an authority hole near the wall.

This was not accepted as root cause. The earlier decision-1726 snapshot showed
that the same seven-state SQP and exact proof chain can certify a different
current-world candidate. Retained cursor exhaustion is therefore downstream of
an earlier candidate-generation authority gap.

## Causal chain under test

```text
published plan clock advances
  -> vehicle speed/progress lags plan
  -> selected suffix join moves ~3.6 m ahead
  -> current-world rebase accepts retained source
  -> heading/progress error grows near wall
  -> artifact cursor unavailable
  -> normal authority unavailable
  -> Emergency Stop
  -> actual wall-margin violation
  -> Recovery
```

## Comparison

- A: persistent Mission plus retained seven-state artifact
- B: stateless current-world ManeuverBundle with the same seven-state solver
- C: rough bounded candidate plus seven-state refinement
- D: offline multi-SQP/nonlinear feasibility oracle

No comparison arm may publish or change production state.

## Classification result

At frozen sequence 992 / decision 1726:

- A persistent and B stateless direct candidates failed;
- the existing midpoint/late candidates failed;
- a rough physical diagonal from the last nominal stay-behind stage 5 to the
  canonical target-tube boundary stage 14 certified on the right side;
- the same candidate on the left was rejected by exact timed obstacle proof.

This is `A/B fail, C succeeds`: candidate generation defect. At decision 1802
all arms were already physically infeasible, so auditing only the visible wall
Recovery would have selected the wrong root cause.

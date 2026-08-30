# Design

## Observation axes

The certified Stop suffix contains the steering state produced by its exact
nonlinear integration. At the next control callback it is compared with:

1. command-control-origin steering (the current canonical request state);
2. current-time physical steering report;
3. response-control-origin steering inferred from yaw response;
4. previous published physical steering.

The first axis remains the canonical validity requirement. The other three are
diagnostics and may be unavailable independently.

## Bounded telemetry

Each one-shot join updates an in-memory window:

- reason counts;
- sampled counts by canonical intent;
- average and maximum pose, yaw and speed error;
- average absolute and maximum absolute steering error for each owner.

The controller emits one structured summary per second and clears the window.
Individual samples never affect authority and are no longer logged.

## Exit classification

- command-control-origin is the closest axis: successor model and canonical
  request share steering semantics;
- physical or response axis is much closer: current request starts from a
  different steering owner than the proof;
- previous-published is closest by one full rate step: publication-boundary
  indexing is off by one;
- every axis has a large structured error: exact Stop integration and
  serialized publisher semantics disagree.

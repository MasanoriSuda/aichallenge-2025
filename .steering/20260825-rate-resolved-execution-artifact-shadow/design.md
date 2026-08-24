# Design

## Representation boundary

The rate-resolved artifact is deliberately separate from
`CanonicalExecutionPlan`:

```text
five-state canonical plan
  state   = [e_y, e_lag, e_psi, v, theta]
  input   = [a, curvature, v_theta]

rate-resolved artifact
  state   = [e_y, e_lag, e_psi, v, theta, steering]
  input   = [a, steering_rate, v_theta]
```

No curvature control is synthesized and no solver steering state becomes a
new actuator integration origin.

## Extraction

After the common physical row certificate accepts the primal, extract every
state and control directly by the six-state/three-input layout. Store lateral
bounds from the same adapter request and stage durations from the same temporal
linearizations.

Artifact validation is fail closed. It checks identity, shape, finiteness,
limits, certificate acceptance, state-zero semantic continuity, lateral boxes,
and the complete semantic-steering piecewise sequence. It never clamps or
repairs a solver result.

## Time and actuation ownership

The prediction clock starts at the snapshot state time, not solver completion.
An exact cursor walks the immutable stage durations. Steering at any cursor is
integrated from the semantic current steering through all crossed certified
steering-rate stages. This preserves the same semantics already proven for the
40 Hz first publication sample and extends them to the whole retained horizon.

## Authority

The artifact is attached to the shadow result as
`shared_ptr<const ExecutionArtifact>`. The mailbox remains observation-only and
there is intentionally no store/admission/publisher API in this Slice.

The later production Slice must still add current-world physical wall and
dynamic-obstacle revalidation before it may feed the normal authority selector.

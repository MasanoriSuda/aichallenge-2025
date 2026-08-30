# Design

## Command versus plant response

The Ackermann command remains maximum braking until the next publication.  At
the physical lower velocity bound, the simulated plant cannot acquire negative
forward speed, so its effective acceleration becomes zero.  These are not two
serialized commands:

```text
commanded acceleration: -3 m/s2 for the whole command interval
effective acceleration: -3 m/s2, then 0 after the speed floor is reached
```

`ActuationSample::acceleration_mps2` remains the immutable command provenance
used to build the execution artifact.  Add
`effective_acceleration_mps2` for exact-model diagnostics.  The nonlinear
integrator uses the effective value, while bundle grouping compares the
commanded value.

This removes no certificate and creates no additional authority path.  It
aligns the immutable artifact with what crossed the wire and the exact
trajectory with the saturated plant response.

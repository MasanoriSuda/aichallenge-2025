# Design

The fixed Stop candidate has one lateral target for the entire braking
distance.  The accepted seven-state Stop proves that the missing degree of
freedom is a changing lateral path, not weaker braking or wall constraints.

Build an immutable profile from the normal artifact:

```text
(theta_0, e_y_0), ... (theta_N, e_y_N)
```

At each Stop publication interval, linearly sample `e_y_ref(theta)` and feed it
to the existing single Stop path-feedback law.  Acceleration remains physical
maximum braking.  The exact rollout is then checked by the unchanged wall and
current-world peer certificates.

This is a candidate-generation comparison.  The normal artifact is not
retained merely because a Mission exists; every current-world invocation
still rebuilds and proves the exact Stop rollout.


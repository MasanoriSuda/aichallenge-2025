# Design

Add a process-local launch environment to the TinyLidarNet node:

```xml
<env name="OPENBLAS_NUM_THREADS" value="1"/>
```

This value exists before Python imports NumPy, unlike assigning
`os.environ` inside the node module.  The setting therefore controls the
actual OpenBLAS pool and does not affect sibling ROS processes or MPC peers.

The node startup identity records the environment value.  The existing launch
contract test verifies the value structurally; no new ROS parameter or topic is
introduced.

Run the accepted peer-512 artifact as authority-disabled async shadow.  This
exercises production Conv5/spatial inference plus recurrent diagnostic load
without changing the published command.  A later bounded-authority retry, if
any, is a separate Slice after this resource contract is certified.

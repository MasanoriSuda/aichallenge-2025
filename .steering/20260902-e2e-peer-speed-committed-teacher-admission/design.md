# Design

The failed production run already provides a causal observation: the spatial
student steers toward a coherent side hazard before becoming physically
trapped.  The repository also contains a speed-aware, temporally committed
teacher whose policy was designed to decide earlier and avoid late cross-side
switches.  The clean experiment is therefore to execute that existing policy
in the same d3 role while keeping both MPC peers unchanged.

This separates two questions:

1. **Teacher policy feasibility:** can the proposed corrective policy actually
   finish this interaction world?
2. **Student representation/training:** only after (1) passes, can its executed
   successful sequence become a hard-label source for a future candidate?

No production authority changes.  A successful teacher run is data evidence,
not permission to ship the heuristic.  A failed teacher run falsifies this
label source for the frozen case and ends the Slice without training.

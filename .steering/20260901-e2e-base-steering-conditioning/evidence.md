# Evidence

## Candidate

- artifact: `spatial-production-wheel-base-seed2030-v10/20260901_205621/candidate.npy`
- SHA256: `8094387fd64ce8f702fb599b2153a06051fc88ea23eb75abd7e606054fe36b43`
- embedded base SHA256: `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- correction representation and runtime authority: `+/-0.12 rad`

## Passed evidence

- strict offline gate: aggregate material improvement `30.41%`, held-out
  focus `31.75%`, held-out tail `34.31%`, peer `32.06%`, bounded sign
  accuracy `95.32%`, normal-anchor MAE `0.003853 rad`, independent normal
  MAE `0.006360 rad`;
- shadow single: three laps, no penalty or stall, 100% inference coverage;
- authority single: three laps, no penalty or stall, 7048/7048 authority
  applications and no runtime clipping.

## Failed evidence

NPC seed 2026 timed out after two laps.  The final wall stall lasted about
`76.2 s`.  Frozen-bag replay shows that during the failure window:

- the candidate selected the correct positive escape direction on every
  material sample;
- its mean correction was `+0.11986 rad`, effectively pinned to its model
  limit;
- the admitted teacher required a mean correction of `+0.88240 rad`;
- the teacher reason was `side-clearance` for all 1196 focus samples;
- wheel-speed versus fused-speed substitution did not change this result.

The old saturation diagnostic tested `abs(output) > bound`, so a sigmoid
output that approached the bound without exceeding it incorrectly reported
zero saturation.  Near-bound residency is now measured separately.

## Classification

This is not a direction-classification, ROS input, checkpoint-loading, or
runtime-wiring defect.  The bounded-residual representation is physically
incapable of overriding the frozen base when the base steering points toward
the wall.  Candidate v10 is therefore rejected and must not be promoted.

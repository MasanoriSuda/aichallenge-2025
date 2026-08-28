# Requirements

## Objective

Determine which physical proof or replanning operation owns the remaining
22--28 ms `update_overtake_line()` spikes observed after start-grid tactical
generation was removed from the 40 Hz callback.

## Constraints

- Do not change production authority.
- Do not change solver tolerances, wall/vehicle clearance, timing leases,
  fallbacks or state transitions.
- Attribute work to a concrete operation before moving or deleting it.
- Preserve current physical wall and dynamic-obstacle certificates.
- Keep generated run artifacts and user result files out of commits.

## Exit criteria

- A single decision log identifies the expensive helper family and its wall
  cache request/miss/scan cost.
- Static tests pass.
- A bounded dynamic run reproduces or falsifies the hotspot.
- Any subsequent correction removes duplicated ownership rather than weakening
  a certificate.

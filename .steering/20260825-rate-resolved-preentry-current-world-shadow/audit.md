# Audit

## Finding

The new six-state prospective artifact already contains exact solver and static
physical evidence, but no live adoption proof. The existing production
rate-resolved retained evaluator already owns the required measured-to-control
connector, actuation reachability, static-wall identity and dynamic obstacle
checks. Reimplementing any subset at Gate A would create another competing
definition of executable.

## Decision

Refactor only request construction into a shared helper, then invoke the same
revalidator for the selected six-state artifact at the live adoption boundary.
Keep the result observation-only until dynamic evidence exists.

## Root cause exposed by the first dynamic run

In `output/20260825-194808`, a complete selected six-state ShiftOut plan reached
the live adoption shadow but was rejected as `static-world-mismatch`. The
source physical snapshot was produced in the async worker from an immutable
deep copy of the live wall grid. The retained evaluator nevertheless defined
static-world identity as equality of the two `shared_ptr` addresses.

The earliest violated invariant was therefore not wall feasibility: identical
grid content acquired a different allocator address across the intentional
async copy boundary. The pointer comparison masked all later current-world
checks. Replacing it with a bypass would weaken proof, so the immutable physical
snapshot now seals a deterministic content fingerprint. Same-owner requests
retain the fast pointer path; different-owner requests must match the sealed
fingerprint. A changed cell or geometry is still rejected.

## Post-fix dynamic finding

In `output/20260825-200059`, `static-world-mismatch` disappeared. Complete
selected six-state plans crossed the live join and were rejected by later,
typed conditions instead:

- sequence 1615: `steering-unreachable`;
- sequence 1779: `progress-lift-rejected`;
- later samples also exposed `velocity-unreachable`.

This confirms that static-world content identity repaired the first blocking
contract rather than suppressing current-world safety. It does not approve
production promotion: async actuation/progress connectivity and dynamic Pass
coverage remain unresolved.

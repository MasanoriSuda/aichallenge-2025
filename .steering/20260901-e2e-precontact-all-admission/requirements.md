# E2E pre-contact all-teacher admission requirements

## Objective

Determine whether the accepted pre-contact diagnostic policy is a valid
four-vehicle teacher rather than only a local escape for domain 4.

## Constraints

- Use the deterministic `e2e-final` world and synchronized start.
- Use `precontact_teacher` on all four domains.
- Keep checkpoint, physical distances, acceleration and AWSIM settings frozen.
- Do not change production defaults or extract labels before run-level evidence.
- A successful domain requires no post-start low-speed interval, no
  positive-acceleration stall, and terminal/contact evidence.

## Definition of Done

1. A reproducible Make target makes the all-teacher authority explicit.
2. Every launch log proves the expected control mode.
3. Every finalized bag is analyzed with the same admission tool.
4. Evidence either admits corrective extraction or closes this teacher branch.

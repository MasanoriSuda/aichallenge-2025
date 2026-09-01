# E2E pre-contact corrective-label requirements

## Objective

Convert the admitted all-peer pre-contact teacher bags into auditable train-only
corrective sequences without confusing them with the historical gap teacher.

## Constraints

- Preserve `gap_teacher` relabeling and its existing metadata by default.
- Require an explicit teacher mode for the new policy.
- Record teacher class, control mode, label source, configuration, checkpoint
  hash and source bag in every generated sequence.
- Do not change production authority or production weights in this slice.
- Do not place generated dataset arrays in Git.

## Definition of Done

1. Relabeling supports `precontact_teacher` with a distinct dataset identity.
2. The dataset loader admits the new identity and continues to reject unknown
   sources.
3. Unit tests cover identity selection and sequence-ID separation.
4. The four admitted bags can be extracted without provenance ambiguity.

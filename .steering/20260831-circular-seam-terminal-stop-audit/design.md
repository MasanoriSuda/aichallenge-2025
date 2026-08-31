# Design: circular-seam terminal Stop audit

The current rejection collapses five independent shape checks into one text:

- nominal path states;
- solver input stages;
- configured solver horizon;
- progress-indexed wall reference;
- lower and upper physical wall profiles.

Extend only the rejection evidence.  Record all five sizes, whether the
progress wall refinement was active, and the wall-profile construction
diagnostic already sealed into the immutable solver snapshot.  This keeps the
failure fail-closed while making a seam-specific shape or provenance defect
replayable.

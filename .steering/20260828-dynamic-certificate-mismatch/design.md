# Design: Dynamic certificate mismatch localization

`DynamicProof::Result` already owns the final acceptance decision but discards
where a dense swept sample first violates clearance.  Extend only its evidence
payload with:

- first rejection reason and obstacle;
- first rejection elapsed time and ego pose;
- minimum-clearance sample identity.

`observe_pose` is the earliest boundary that has all these values.  Capturing
them there avoids duplicating collision geometry in the comparator and does
not affect proof behavior.  The offline comparator then identifies whether the
sample belongs to the measured-to-control prefix or the candidate trajectory.

No runtime command path consumes the new fields.


# Design

## Evidence before implementation

The two-frame signed model classified the memorized hard tails accurately but
could not keep corrections closed on normal observations.  A dataset audit
showed that material correction direction persists within the preceding eight
samples:

- ordinary successful runs: approximately 65--78%;
- hard failure tails: approximately 95--96%;
- validation runs: approximately 48--89%.

The base-policy steering distributions overlap heavily between negative,
anchor and positive target classes.  Therefore adding only base steering would
not be an evidence-backed separator.

## Comparison

Add one offline input mode, `scan_history8`, containing the current normalized
LiDAR scan and the seven immediately preceding scans.  At the beginning of a
run, repeat the first real scan.  The model and loss remain the signed-mixture
architecture from the frozen previous Slice.

This is an explicit finite observation window, not a production recurrent
state.  It tests the narrower question: does recent geometry resolve the
teacher action sufficiently to justify a recurrent/runtime implementation?

## Admission

Use the existing limits unchanged:

- material MAE improvement at least 30%;
- anchor MAE at most 0.01 rad;
- independent normal leakage MAE at most 0.01 rad.

Historical anchor failure is also a hard qualitative rejection even if an
aggregate metric happens to pass.

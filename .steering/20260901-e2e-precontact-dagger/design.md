# Design

The relabeler remains one implementation because scan loading, contact cutoff,
checkpoint validation and array writing are common contracts.  A small immutable
teacher identity table owns the fields that must never drift independently:

```text
teacher mode
  -> runtime control mode
  -> dataset label source
  -> generated control message identity
  -> teacher class name
```

The mode is included in the sequence identity so running the two teachers on the
same bag and checkpoint cannot collide.  `gap_teacher` stays the command-line
default for backward compatibility.  `precontact_teacher` must be selected
explicitly for the newly admitted data.

The first all-active extraction produced 5,417 labels, but only 561 samples had
any difference from the historical teacher and 418 differed by at least 0.02
rad.  Training all active labels regressed independent validation MAE by 28.2%.
The successor extraction therefore admits only material successor-policy deltas
relative to `LidarGapTeacher`; this directly represents the clustered side
detection and directional projection introduced by `precontact_teacher`.

Even the 418-sample successor-only set materially regressed normal validation
when every layer was updated.  The next bounded candidate freezes the admitted
feature extractor and hidden policy, updating only `fc4`.  This tests whether
the existing representation already separates side-threat states without
allowing a rare correction set to rewrite normal LiDAR features.

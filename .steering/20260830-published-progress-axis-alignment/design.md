# Design: published progress-to-path alignment

## Causal chain

1. `build_published()` advances a published source from measured course
   progress minus the source course-progress origin.
2. `align_published_overtake_execution_trajectory()` passes that course
   progress displacement to the generic resampler.
3. The resampler's axis is `Source::path_distance_m`, while the advance value
   is measured course progress.  Lateral displacement and curvature make those
   coordinates differ.
4. An initial experiment changed the resampler axis to course progress.  The
   dev2 run `output/20260830-165202` falsified that approach: exact progress is
   contractually allowed to plateau during lateral motion, while the generic
   resampler requires a strictly increasing axis.
5. The publication cursor can therefore remain available while either form of
   direct coordinate substitution rejects a valid published trajectory.
6. The rolling Overtake supervisor then loses the exact lateral profile;
   downstream progress watchdog and authority failures are symptoms.

## Change

`mpcc_rate_resolved_execution_source::Source` preserves three immutable
quantities:

- `path_distance_m`: monotonic nominal path coordinate used by horizon geometry
  and the exact nonlinear proof samples;
- `progress_m`: absolute lifted course progress, which may plateau;
- `elapsed_time_sec`: exact proof time used to locate the publication cursor on
  a progress plateau.

The execution-source owner lifts measured course progress, finds every matching
segment in the exact progress samples, and selects the corresponding path point
nearest the publication-clock cursor.  The consumer receives that projected
`advanced_path_distance_m` and uses only `path_distance_m` for resampling and
remaining coverage.  Failure to project is explicit; there is no fallback.

## Classification

This is a model/certificate integration mismatch: solve and proof can succeed,
but the lifecycle consumer joined course displacement to a path-coordinate
profile.  It is not clearance tuning, physical infeasibility or a
watchdog-threshold defect.

# Investigate trial 3 physical slowdown before authority loss

The fixed-binary single campaign fails on repetition3 after two clean six-lap
trials. Preserve all three outcomes. Do not proceed to dev2 or claim overall
Stop-reference acceptance from the two successes.

Determine why vehicle-status speed falls from7.796716 to0.852230m/s between
source250.599994 and250.634994s while the published command remains7.80m/s
with+1.3296m/s2. The first Recovery10713 and authority loss10714 are later
effects, not the origin of that slowdown. No controller policy, speed cap,
braking limit, wall margin or localization parameter is changed during audit.

Use same-run timestamps, declared map/footprint, sensor provenance and the
actual simulator model. If evidence cannot reconstruct the first boundary,
obtain minimum diagnostics with a named deletion boundary before a fix.
Rollback baseline remains a8b968ac, not a race-certified commit.

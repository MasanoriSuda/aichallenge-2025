# Rejoin full wall feedback comparison

2026-09-13. Production unchanged at e6b5c740 (runtime code226e4f84).
R776binds actual single-r23 post-motion decision913 to Rejoin sources537/538/656/743.
R777cold source537reproduces wall722;538/743remain4000iteration solver rejections.
656runtimewall803 versus coldwall834 reflects absent original internal warm state;
do not claim exact original solver reconstruction. Exact captured failed QP warm/cold
both reject for538/743. Existing target-free B/C/D arms are inapplicable. Separate
restorationH743 fully certifies, but H537/538/656wallreject. No infeasibility claim.

R778537: current pose and control origin have no hard-clearance contacts. At3.615sec,
left-front cell471815overlaps the complete footprint, separation−0.000065564915m.
Original0.2mlateral clearance and all dimensions remain. The outer wall guard correctly
rejects. The source correction loop only reacts to exact state/adapter constraints;
it has no complete world-wall feedback after the nonlinear correction.

Bounded temporary-source comparison retains world/model/bounds/objective/4000solver
budget and max3corrections. R780537A stops at correction1 with wall722. Adding the
complete trajectory wall outcome to the existing correction condition produces:
correction2wallpass/statefail; correction3wallpass/statepass; full original wall/
dynamic/completeStop bundle accepted. R782656A wall834; same feedbackB wall914 at2,
then all original guards and bundle pass at3. Approximate offline solver times are
67.90ms and64.98ms; these include diagnostics and are not runtime deadline acceptance.

AlternativeC rebuilds local wall approximations from the original broad problem and
latest native trajectory:537rejects empty-trust-region;656passes. It is unnecessary
for the two B positives. 538/743 retain their pre-wall solver rejection. R779driver
construction failure and R780/R781C missing diagnostic prerequisites are preserved;
R782 explicitly records the pre-wall problem without changing production.

Next test the minimal B predicate for trajectory wall rejection only. Irreparable
current-pose/control-origin or invalid-input failures must not request extra solves.
Add saved-world negatives before production integration; preserve the independent
outer wall authority and original correction limit. Then build/package and fixed
commit live acceptance. No wall relaxation, iteration increase or new normal owner.
Later Stop envelope input, Recovery rear hazard, physical observation/current velocity/
r80actual-send timing and M4–M6remain open. [Evidence](rejoin-full-wall-feedback-evidence.json).

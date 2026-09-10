# Separate steering receipt observation

2026-09-11 JST; controller/shared arithmetic `2fc31bf7` (scalar bit parity with
`cbcda112`). No control parameter changes. This declaration precedes the new
application-dev2-r4 launch and the first steering assignment measurement.

The longitudinal 250ms empirical candidate passed independent application-r3.
It does not establish a steering receipt bound. The decoded callback writes
`Vehicle.SteerAngleInput` after releasing two longitudinal/stamp locks. Source
time and exact angle at that write are the missing observations. The previous
four-call probe only saw longitudinal receive/selection/application.

Add one observation immediately after the original steering field assignment
in the Ackermann callback, recording receiver ID, original message stamp, wire
steering, assigned float32 degrees, ROS time and monotonic time. Preserve the
original DLL and every decoded original instruction, branch and exception target;
validate the generated copy after stripping the five diagnostic calls. Do not
use the private receiver trace in the participant or submission. Instrumentation
changes timing; this run is not runtime acceptance.

Predeclared steering candidate: **100ms maximum age of the last assigned packet
in ROS source time**, separate from the existing100ms mechanical delay. It is a
conditional empirical model hypothesis, not a claimed transport guarantee.
The slower longitudinal selection keeps its already declared250ms candidate.
The100ms steering candidate is fixed before obtaining the new steering trace;
do not increase it after an observed failure without a new causal comparison.
Also report the250ms diagnostic hypothesis without treating a larger passing
number as adoption. The source-stop-r4 conditional250ms/250ms calculation passes
low-speed895 and diverges numerically in1101/1585/1620; it is not an infeasibility
certificate and must remain rejected/inconclusive as a numerical method.

Every steering write must match a received message by receiver, stamp and
float32 payload. Keep duplicate-message ambiguities explicit; do not invent a
sequence identity. Confirm the decoded radian-to-degree conversion. Test every
complete interval on each receiver, including startup and stopping: prior
source <= prior assignment <= next assignment, nondecreasing source, and next
assignment ROS time minus prior source <=100ms. Report all violations, maxima,
and initial/final open intervals as censored. Probe errors or conflicting clock
order make the profile inconclusive. Recheck all longitudinal assignments and
the250ms complete-interval condition on this same run.

Retain first moving Emergency/Recovery as a failed race run. Freeze config,
source/binary/probe hashes and restore the original DLL/protected user JSONs.
No same-HEAD race acceptance is claimed. After observation, compare the same
source-world input windows and improve the numerical enclosure if necessary;
then complete the one immutable common-control proof integration and M4–M6.

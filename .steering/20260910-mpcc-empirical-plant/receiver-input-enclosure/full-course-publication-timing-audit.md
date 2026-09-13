# Full-course attempt: actual publication boundary failure

2026-09-13. single-r26 / 549d96a7 (control38a562ee), original DLL/build140/
package127. The declared6laps/600simsec attempt stopped after an actual violation;
it did not complete the intended time or distance. No vehicleStart/laps here.
721successful normal log entries plus one additional actual normal2180 followed
immediately by failsafe. Monitor10hostsec polling permits further commands until
teardown; all retained, source/binaries/user artifacts verified and runtime stopped.
2580callbacks:mean4.359,p9510.380,p9922.419,max28.203ms;8over25,no adjacent pair.

2180/job2177/source1860/index0, Cruise/current domain8. Original nominal
46.239998985, callback46.244998966, before/guard_end/raw_end46.264998965,
after46.269998965, deadline46.264998985.20nsremain before final publication;
guard0.002194ms/raw0.005050ms/final0.082775ms. Whole callback3.461ms, current
proof1.656704ms. ROS advances25ms across this short callback. Original full
bracket rejection is correct and mandatory; no epsilon/window expansion.

R803:all10749recorded clocks match floor(n*float32(0.005)*1e9) exactly. Largest
recorder receipt gap296.983129ms spans46.239998966→46.244998966 immediately
before the failure, followed by clustered clock receipts. Recorder receipt alone
cannot distinguish Unity frame/GC/rendering, host scheduling or DDS. Older R53
does independently prove original producer multi-tick moving-frame clusters;
one added5ms safety allowance is therefore not a proven whole-call bound.

R802 lacked its read-only steering include mount;R804 rebuilt the parent domain
but incorrectly compared it to the snapshot's selected early domain. Retained
diagnostic failures, no weakened production check. R805 lets the original
dispatcher select its own early domain, then verifies original source/selected
domain/history/wire/epoch fingerprints and exact recorded decision. Current
proof accepts; both decision-clock and before-clock guards accept; original
actual after bracket rejects. Standalone current proof1.039ms is not a live bound.

Simple second checks at guard/raw end still accept this actual trace. Absolute
ROS appointment ownership already exists; its wall timer is Emergency-only.
Do not propose the old periodic-timer replacement again or freeze the ROS clock.
Future whole-call timing needs supported execution/clock evidence; safety margins,
rates, publication width, source model and all final guards remain unchanged.

Next independent environment hypothesis: single-r27 changes only Unity to
-batchmode -nographics, retaining original DLL/physics/sensors/RViz/standard
startup, current code and6laps600simsec,120hostsec observation bound. It compares
clock/receipt clustering and actual sends, not controller parameters. Previous
r67 used old control and stopped for a different source population defect; it is
not current evidence. Renderer diagnosis is not canonical acceptance or proof
of a GPU cause. Stop on actual window or launch failure. The unused interval
observer remains observation-only; neither current failing attempt reached its
declared query, and deletion/relevance is reviewed after this environment check.
All M4–M6 remain open. [Evidence](full-course-publication-timing-evidence.json).

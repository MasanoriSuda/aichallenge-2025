# Complete wall feedback live acceptance

2026-09-13. single-r24 / 1648f338 / D1, build139/package126, original DLL,
120hostsec. 4267callbacks, maximum24.299706ms;552normal sends, zero pre-send
refusals or actual deadline violations. Last1226/job1222/source957/index1:
nominal18.764999608, before/after18.769999580, deadline18.789999608.
Selected Rejoin sends1038/source599 and1177/source868. No Rejoin completion.

At decision1132/time16.409999633, mailbox latest-published/store785 and its
result record a certified steering-before-drive candidate with one wall-driven
correction, total worker99.318021ms. Source785 subsequently sends three commands,
first1152/job1149/index0 at16.914999621 within the original deadline. This validates
an actual corrected source reaching existing final authority. Other logged corrected
sources701/983 do not have observed sends; do not infer their runtime eligibility.

Public peak1.076852083mps at10.814999758;clock200Hz,velocity/steering28.571,
IMU20,pose50,trajectory1,commands36.187. GameStart yes; vehicleStart/laps absent.
All9Recovery choices Forward; configured small-steering reverse still unobserved.
No Unknown-direction capture. Source/binary hashes and protected artifacts verified;
runtime stopped. No causal performance comparison across independent r23/r24 worlds.

First saved post-motion stationary final failure949 later resumes. Saved actual
Rejoin639initial solver,717futurewall931,793post-correction solver,837wall-refined
solver failures have original inputs/QP tags inR791; these do not prove infeasibility.
Later Store983 stops accepting; final4236/time103.774997680 reports missing terminal
Stop course geometry with current physical envelope valid1/feasible1, but the
preferred-containment flag and selected interval are absent. The predicate also
requires containment and finite ordered bounds (mpc_controller_cpp.cpp:22490).
No clearance or faulty guard is inferred from that incomplete diagnostic.

Next capture this earliest missing producer query/result once after authenticated
normal motion: original map, temporal pose, reference frame, lag/heading, bounds,
hard clearance, sampling anchor, clear components and selected interval. Keep
original computation and control decisions unchanged, serialize outside control,
then replay exact inputs before any producer fix. Retain original safety/clock
contracts. Physical model/receiver/current velocity, previous r80 D2 timing and
M4–M6 remain open. [Evidence](full-wall-feedback-live-evidence.json).

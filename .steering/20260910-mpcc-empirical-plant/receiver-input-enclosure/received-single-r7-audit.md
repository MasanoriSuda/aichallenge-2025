# Actual received histories in standard single-r7

2026-09-13 JST,fe976b39/build121/package108. The observation implementation works;
control selection/model/authority/guards remain unchanged. FullM4–M6 are incomplete.
[Manifest and payload hashes](received-single-r7-evidence.json).

R7 reached Ready and public forward speed1.039849758m/s at9.869999779, then stopped.
The earlier stationary Track/Cruise context failure779 resumed with normal782.
Moving rejected candidate867 is a worker/source observation, not a final authority
loss. Actual normal sends continued through1011/job994/source593/index14,
nominal13.474999827/before=after13.479999698/deadline13.499999827. The next final
Emergency1012 was already stationary. Its exact final snapshot is unavailable:
the earlier stationary779 consumed that recorder slot. No Start/laps; termination
was the120s host cap. There is no accepted restart or overall stop/race acceptance.
All runtime containers stopped and original DLL/userJSON hashes restored.

R563 records26valid occurrences over20unique decision captures, with3missing
programme records explicitly missing. All recorded source/current decision IDs
associate correctly.17occurrences have a received velocity/tire sample newer
than pose but no newer than captured now;10have such an IMU sample. These counts
include aliases and are not26independent experiments. Source859 uses u0.897505283
from9.624999784 at pose9.644999784 despite already receiving u0.917512953 at
9.659999784 before its9.679999783decision. This establishes actual receiver
availability, which earlier bag timestamps could not establish.

R564 same-bag frequencies during Ready7.7–13.5s are velocity/tire28.5714Hz,
IMU20Hz,pose50Hz,trajectory1Hz,control40Hz andclock200Hz.35ms velocity/tire periods
explain recurring component ages without proving that every held value is wrong.
4239recorded callbacks have maximum19.778002ms,none exceed25ms.480scheduled sends
have no before/after deadline violations. This is one run's cost evidence, not a
new bound or a repair of earlier actual deadline failures.

Programme diagnostic association flags preserve changes in post-publication now
and command-derived initial desired steering. Raw pose/body/IMU/tire components
are unchanged. R566 compares7saved programme occurrences: replacing only that
initial desired value leaves every nominal current/control/intermediate state
exactly equal, because applied input history resets desired before each positive
integration interval. This flag discrepancy is not promoted as a body-error cause.

R565 freezes an offline pose-bracket reconstruction and rejects5malformed history/
ownership cases. It uses only exact already-received endpoints and retains held
channels lacking support. R7 has no private physical truth at its pose epochs,
so no accuracy/promotion claim follows. The next bounded force-r3 jointly captures
controller queues and physical truth under the [frozen protocol](received-pose-bracket-protocol.md).
Further observation/model repair, original safety/timing negatives and allM4–M6
remain required. No confirmation or push is needed.

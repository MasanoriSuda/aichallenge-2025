# Preserve moving authority loss and attribute its callback

Baseline `edad4e32`, frozen `20260910-nine-state-dev2-r10`. Both cars launch;
D2 negative ShiftOut reaches Pass. D1 first moving Emergency is decision1670,
wall1789034756.744055656, 8.41m/s. The six-lap attempt fails and remains recorded.
Shutdown starts1789034765.09014; later clock/input failures are separate.

D1decision1668 takes164.662143ms, with163.471ms inside MPC. A current-world
lattice source1119 is accepted, then its terminal Stop is materialized as1152.
The existing throttled production breakdown reports1659, losing1668's detail.
The command bag receipt gap is178.093672ms. Its source stamps are28.249999368
and28.284999367; the next packet jumps to28.434999364. This establishes delay,
but the source of its cost and actual simulator application epoch remain unknown.

Recorder detection gap: stationary Cruise833 already occupied FinalAuthority,
so moving1670 and its exact1669 predecessor are absent. Available ordinary
viability pair1667/1668 is a different event/source1148. Exact native replay
reproduces Accepted with0.461575247m terminal peer clearance, then rejection
at−0.005066955m. A separately rebound immediate-brake proposal with the
captured scalar progress anchor passes at1.212136504m on1668. This diagnostic
does not reconstruct the live waypoint projection or missing1670 input.

Same sealed1668 architecture comparison: A solves15.0498ms but terminal peer
proof rejects. B/C/D/G lack the current target tube and are inconclusive for
Cruise, not evidence of physical impossibility. Both current-world complete-rest
SQP clocks reject at4000iterations before proof (113.776/46.3936ms). Existing
native braking continuation succeeds. All evidence belongs to unchanged
production binary fromedad4e32; no new controller authority is promoted.

Observation-only change:

- Preserve TerminalContingency and FinalAuthority; add MovingFinalAuthority
  when the existing current speed is finite and above the campaign's0.1m/s
  classification threshold. No control decision reads this classification.
- Keep one immutable event per intent/side/boundary. The finite queue has
  three boundary categories; repeated decisions cannot replace the first.
- Carry already measured primary/lattice/Stop/output/snapshot times into
  each existing overrun detail, so the one-second throttle on the separate
  summary cannot hide the first event. Nested times are not added to totals.
- Extend the queue regression: startup and moving failure both survive while
  the writer is blocked, with separate first worlds and publication identities.

No model, solver budget, corridor, safety margin, normal selection, command or
deadline rule changes. The controller fix remains unproven until the earliest
production violation is captured. Rollback is the observation slice's local
commit, based onedad4e32. Build/package checks and a fresh bounded dev2 run
must follow. M4–M6 remain open; do not count diagnostic replay as race acceptance.

Validation: buildr25 passes26packages in4min36s; testsr19 passes2402test
records in62CTestgroups,0errors/failures/skips. Quality hooks pass. Public
interval extraction initially failed on a ROS Python import; preserved failure
source/log, corrected import, r2extraction passes. At1668 packet source28.284999367
arrives between clock28.434999364/28.439999364. The packet's actual command is
−2.888103485m/s² and−0.080717243wire rad; this differs from the diagnostic
immediate−3m/s² Stop. Native diagnostic must not be presented as that actual
lattice selection. See evidence JSON for hashes and comparison provenance limits.

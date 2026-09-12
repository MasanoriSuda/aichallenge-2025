# single-r11: current timing passes locally; source wall failure remains

2026-09-13, production fdee0b8f, shared native model8d9810e0.
Build126all26packages/package113all2622records/source112/native216apply to this run.
No controller, parameter, model or simulator DLL change during or after the run.
User authorizes continuing the full M1–M6plan and local commits without approval.

## Runtime evidence

Standard single/D1, original simulator DLL,120s host cap.4255callbacks,
maximum19.919364ms, no25ms callback overrun, no before-publication guard rejection,
no out-of-window normal send.509normal sends, last1047/job1031/source651/index13
at nominal14.024999828, actual14.029999686 before deadline14.049999828.
Ready1048at14.054999685selects the next Stop index14after rest expiry. No later
normal send. Store remains651/420accepted through~103.8s despite new solve attempts.
No Start/lap/finish. Publicmax0.997210741mps; this is not private body/contact truth.

Source-time whole-bag frequencies: velocity/tire28.571Hz,IMU20Hz,localization49.99Hz,
trajectory1Hz,control36.048Hz,clock200Hz. Do not relabel whole-run control frequency
as the configured40Hz or infer controller DDS receipt from bag recorder timestamps.
Original DLL and both userJSON hashes are preserved/restored; runtime stopped.

## Causal separation

R649repeats actual1048three times with exact source/domain/history/last actual
source. RestExpired is reproduced; extending it would conceal absent replenishment.
The last normal motion witness is1005/source651/index10,u0.1827897429at12.964999710.
Both initializer0/source805andinitializer1/source804capture the actual loss1048,
with matching earlier normal publication identity. Original source outcomes are
outer-exact-physical-wall-rejected at stage71. Cold A replays the same failure
class/segment, not bit-identical original solver state or exact failed QP.
Unlike single-r9, the source Store does not keep receiving fresh certified plans.

R648's existing B/C/D/G are avoidance planners requiring a dynamic target; the
current scene is target-free Cruise. Their unavailability proves no infeasibility.
The9state solution progresses physically only near30.84m while its nominal
path sample is34.77m; the exact wall checker catches the actual full footprint.
The inner sampled pose-bucket constraints were intentionally retired because of
prior false infeasibility; its diagnostic accepted flag is not the final physical
certificate. Do not restore those rejected boxes or mistake the log for a safe plan.

R650fixed6native trajectories keep original controls/model/wall/dynamic/full Stop
proof. Quarter-drive +/-steering certify but peak only0.003423mps and move~6mm;
these are not useful restart witnesses. Half/full drive fail wall or artifact
corridor. R651holds acceleration at0until the same bounded steering endpoint is
reached, then launches on the next existing stage. All4half/full +/-candidates
fail wall/corridor. No candidate executes, no tuning or infeasibility claim.

The existing Recovery detector is gated byrace_started_, which becomes true only
onStart. Ready permits normal Cruise but remains race-inactive for Recovery.
This is a separate eligibility boundary; enabling reverse cannot substitute for
repairing the source proof. Preserve all session/control-mode/gear/reset safeguards.

## Next bounded work

Locate the exact limiting occupied cell and footprint edge/corner at current and
first rejected native pose. Compare a target-free nonlinear feasibility source
under the original9state model, input bounds, complete world and terminal Stop.
Use a useful verified physical witness to test the corresponding free optimization
and repair its actual geometry/constraint producer; no raw fixed-input normal
fallback, added seed sweep, grace interval or smaller wall margin. Candidate
parameterization may change with sealed identity; all hard proofs remain.

Measurement/contact error, Recovery/Rejoin, all intents/Mission/siblings/Store/async,
repeated final-source single/dev2runs,dev3/dev4sixlaps,gates1–3and same tar/image/eval
remain open. No partial timing result is integrated MPCC completion.

[Sealed run, replays and hashes](transport-single-r11-evidence.json).

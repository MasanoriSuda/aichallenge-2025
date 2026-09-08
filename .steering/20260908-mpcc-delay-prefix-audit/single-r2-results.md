# Diagnostic2: moving Emergency and source-capture gap

6laps252.718750s,penalty0, but active-race Emergency9005/9006at7.65m/s.
This is rejected integrated acceptance. Earliest aggregate loss9005has clear
0.13sdelay prefix and20-stage normal continuation, but both Stop references
fail terminal wall proof. Terminal-only8995precedes it and is separate.
No Recovery. Callback13053reportedcycles,max19.772ms,0overruns.
Source times:9005observation210.084995/control210.214995,cursor0.050000,
firstpublish210.199995/firstcursor0.035000,published sequence8390.

No original execution source was saved: the producer kept it only for
ShiftOut/Pass. Native before/after7-intent probe confirms and corrects this
observation gap. This trial does NOT provide exact executed-artifact evidence;
do not apply the unused reconstruction helper to a different fresh problem.

Control timing (source/receive clocks kept separate):
{
  "source_sec": {
    "count": 10598,
    "duration_sec": 264.534994087,
    "gap_ms_mean": 24.963196573275454,
    "gap_ms_p95": 34.99999999999659,
    "gap_ms_p99": 35.0000000000108,
    "gap_ms_max": 94.99999799999159,
    "nonpositive_gaps": 20,
    "gaps_over_50_ms": 3
  },
  "receive_sec": {
    "count": 10598,
    "duration_sec": 264.89771366119385,
    "gap_ms_mean": 24.997425088345178,
    "gap_ms_p95": 26.520252227783203,
    "gap_ms_p99": 27.451915740966793,
    "gap_ms_max": 32.84883499145508,
    "nonpositive_gaps": 0,
    "gaps_over_50_ms": 0
  }
}

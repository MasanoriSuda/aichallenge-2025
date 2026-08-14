# Design

## Observed failure

`20260814-090647`ではrolling replanが9回動作し、従来0回だった
`FollowPrepare -> ShiftOut`が2回発生した。一方、FSM上の
`Pass -> Return`は0回だった。

代表例では、Pass中の`ey=0.79 m`から同側候補を採用した後、
`goal=1.25 m / shift=14.0 m`の新規ShiftOutとして再開し、約0.34秒後に
`optimized horizon escaped target separation bounds`で再び中断した。

`follow_prepare_origin_phase`はPassを保持しているが、Mission commitの最後が
paused originを参照せず一律にShiftOutへ遷移していることが原因である。

## Change

1. paused Mission replacementの実行modeを明示する。
   - Pass origin + same side: `ContinuePass`
   - ShiftOut originまたはcross-side: `RestartShiftOut`
2. `ContinuePass`では現在`ey`と累積Pass距離を初期状態として、候補の横補正を
   `mission_pass_lateral_replan_*`へ設定する。
3. candidateのshift距離は新規ShiftOut距離ではなく、Pass中に横補正を完了する
   前進距離として使用する。
4. rear-clearおよびReturn開始距離は、既に走ったPass距離へ残り計画を加算して
   再設定する。
5. Return確認済みの場合は既存`DynamicMissionWaitAction::Return`を優先する。

## Safety boundary

- replacement候補は従来と同じfull preflight/prefix admissionを通過済みであること。
- current body overlap、target discontinuity、wall hard fault、EmergencyBrake、
  solver recoveryではPass継続しない。
- cross-sideはPass継続に変換せず、既存のno-return admissionを維持する。
- 解がない場合にstale Missionを無期限復活させない。

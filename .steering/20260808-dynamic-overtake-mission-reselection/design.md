# Design

## 基本方針

既存の左右Mission候補生成とatomic replacementを再利用し、OvertakeLineへ
`dynamic mission wait`を追加する。これは新しい走行フェーズではなく、`FollowPrepare`中に
「現在側を再開できるか、反対側へ置換できるか」を継続評価する所有状態である。

```text
ShiftOut / early Pass
  ├─ current feasible                         -> keep
  ├─ alternate feasible before no-return     -> debounce -> atomic replace
  ├─ neither feasible, target/body observable -> FollowPrepare + dynamic wait
  └─ wall/body/Emergency/solver hard fault    -> Recovery

FollowPrepare + dynamic wait
  ├─ current feasible                         -> same-side resume
  ├─ alternate feasible before no-return     -> atomic replace -> ShiftOut
  ├─ no-return / neither feasible             -> keep following and re-evaluate
  └─ target discontinuity / timeout/hard fault -> Recovery
```

## 評価周期とno-return

既存`v2x_overtake_opponent_side_replan_evaluation_interval_sec`を使用する。
候補生成はtarget continuity、現在車体非重複、予測有効時に行う。反対側への置換は既存の
no-return前だけ許可するが、no-return後も現在側の再開可否は評価する。

これによりtargetが近い間は横断せずFollowし、targetが再び前へ離れて反対側が成立した場合に
初めて切り替えられる。

## dynamic mission wait

OvertakeLineStateへ待機flagを保持する。次の場合に有効化する。

- SafeSeparationがrear-clear前にAbort / RecoverBehindとなる
- Pass horizonの延長、refresh、holdがsoft geometry理由で成立しない
- Behavior continuityがlive corridor不成立で切れ、hard faultを伴わない

待機中は同じ側の即時`FollowPrepare -> Pass`を禁止する。最新の左右assessmentが実行され、
現側が完全Missionとして成立した周期だけ同側再開を許可する。

## atomic replacement

既存`replace_frozen_overtake_mission_for_opponent_side()`を`FollowPrepare`へ拡張する。
target ID、Mission全体のabsolute pass budget、累積距離、置換回数は保持する。
新candidateをfreeze後、現在位置から新しい`ShiftOut`を開始する。

## hard fault境界

以下はdynamic waitへ変換しない。

- actual wall footprint contact / wall sample unavailable
- current body footprint overlap
- EmergencyBrake
- target position/course progress discontinuityまたはstale
- solver recovery
- rear-clear済み

## ログ

状態変化時に以下を出す。

- dynamic wait開始・解除理由
- current / alternateの成立可否
- same-side resumeまたはalternate atomic replacement
- no-returnにより横断せず待機したこと

周期debugでは既存`opp_*`へcurrent feasibilityとdynamic waitを追加する。

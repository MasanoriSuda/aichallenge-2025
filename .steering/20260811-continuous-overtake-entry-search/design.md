# Design

## 現象

20260811-154311のd1ログでは、前方車捕捉中のdebug 100サンプルのうち59件が`ShiftOut geometry retry cooldown`だった。追い越し開始は3回で、いずれもwp_id=3〜10に集中した。

原因は、`select_overtake_mission_candidate()`が現周期で候補を選べないだけで、実行失敗と同じside retry blockをarmしていることにある。

## 方針

side retryの原因を次の2種類に分ける。

- `PlanningSearchMiss`
  - 未実行のMission候補が現周期で全棄却された。
  - cooldownをarmしない。
  - 現行のassessment reasonは保存し、次周期に再評価する。
- `PhysicalOrCommittedFailure`
  - 壁接触、壁余裕違反、実行horizon不成立、実行中Mission timeout、実行中の占有など。
  - 現行の1秒cooldownを維持する。

## 実装

1. `v2x_overtake_core`へfailure classとcooldown arm方針のpure functionを追加する。
2. `arm_overtake_line_side_retry_block()`へfailure classを必須引数として渡す。
3. Mission candidate search rejectionだけ`PlanningSearchMiss`とする。
4. その他の呼び出しは`PhysicalOrCommittedFailure`とし、従来動作を維持する。
5. 既存のtarget-scoped pre-arm validation leaseは変更しない。候補を継続評価することで、短いplanner揺れの間は既存leaseが速度履歴を保持する。

## 安全境界

- 探索を継続するだけで、不成立Missionは実行しない。
- BehaviorからOvertakeLineへの引き渡しは、従来どおり現周期のfull validated Missionが必要。
- committed executionの失敗後は同じ側へ即再進入しない。

## 動的確認

- 前方車捕捉中の`ShiftOut geometry retry cooldown`比率
- `mission candidate search rejected`後に別wpでcandidateが再出現するまでの時間
- wp_id=3〜10以外の`Follow -> Overtake`数
- `ShiftOut -> Recovery`、wall/solver failure、接触の増加有無
- control周期の悪化有無

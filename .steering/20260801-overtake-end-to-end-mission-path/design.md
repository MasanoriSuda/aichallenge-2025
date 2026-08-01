# 設計

## 1. 全区間candidate path

entry時に次の距離基準profileを生成する。

```text
0 .. shift_distance
  current_ey -> fixed_pass_goal
shift_distance .. shift_distance + pass_distance
  fixed_pass_goalを保持
残りreturn_distance
  fixed_pass_goal -> base trajectory (ey=0)
```

現行設定では`4 + 8 + 6 = 18 m`であり、既存のovertake gap lookahead 24 m内に収まる。
preflightは全profile sampleに対して、legacy wall bounds、occupancy-grid footprint、
wall margin、横加速度を確認する。全区間を観測できないhorizonは成立扱いにしない。

## 2. path ownership

採用時に次を`OvertakeLineState`へlatchする。

- target ID
- side sign
- fixed pass goal
- shift/pass/return distance
- validated total distance

実行中のwall/contact/target footprint監視は継続するが、locked targetの横位置更新を
fixed pass goalへ再適用しない。危険化した場合はpathを相手へ追従させずRecoveryへ移る。

## 3. FollowPrepare終了

`FollowPrepare`中だけ前進距離を積算する。次のORで現missionを終了する。

- 経過時間 >= 4.0秒
- 前進距離 >= 20.0 m

終了時は旧sideへ既存entry retry cooldownを適用し、side lockを解除する。
同一周期中に反対側へ横断せず、次周期のIdle entryで左右を新規評価する。

## 4. 完了target再捕捉防止

rear-clear confirmationをmission stateへlatchする。latch後はV2Xの一時的な前後判定揺れで
`Return -> Pass`しない。Return完了時にtarget IDを1.5秒保持し、その間は新規overtake
entryだけを抑止する。Follow/SafetyBrakeの衝突保護から対象車を除外しない。

## 5. interface影響

変更は`multi_purpose_mpc_ros`内部と既存YAML parameter追加だけである。
topic、message、service、launch、Domain、result schemaへの変更はない。


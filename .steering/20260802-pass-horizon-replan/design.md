# Design

## 現行の問題

現行candidateは、ShiftOutと固定長Passのcorridorを評価する一方、Returnはrear-clear観測後まで延期している。

```text
入口で ShiftOut + 8 m Pass を検証
              ↓
ShiftOut完了、Passへ移行
              ↓
rear-clear未成立なら固定横目標を維持
              ↓
8 mを超えても、対象へ少しずつ接近していればwatchdog再装填
              ↓
次のヘアピンまで同じ横位置を保持
              ↓
車体重複 / 壁 / SafetyBrake / solver failure
```

`progress_watchdog_distance=32 m` は「全く前後関係が改善しないPass」を検出する。今回必要なのは、これとは別の「入口で検証した経路の期限」である。

## 1. Rear-clear rollout

既存kinematic rolloutへ次を追加する。

- predicted side-by-side time/distance
- predicted rear-clear time/distance
- rear-clear後のReturn開始位置
- rollout終端での対象車との前後距離

rear-clear条件は実行時と同じ縦距離を使う。予測不能または最大rollout時間内にrear-clearできないcandidateは、通常の追い越しcandidateとして採用しない。

固定の8 mをそのままPass距離にせず、概念的に次でmissionを作る。

```text
predicted_pass_hold_distance =
  max(configured_pass_distance,
      predicted_rear_clear_distance - shift_distance + merge_reserve)
```

過大なmissionを防ぐため、予測Pass距離と予測時間には上限を持たせる。

## 2. End-to-end candidate validation

candidate pathは次の3区間を持つ。

```text
ShiftOut
  -> predicted rear-clearまで同じ側を保持
  -> Return
```

検証対象:

- 静的壁footprint sweep
- 動的vehicle corridor
- 曲率速度cap
- 横加速度
- body-clear / hard-distance / rear-clear deadline
- Return corridor

実際のReturn開始はrear-clear観測後のままとする。入口では「予測どおりrear-clearした場合にReturn可能か」を確認するだけで、観測前にReturnへ移らない。

## 3. Mission state

`OvertakeLineState`へ以下の概念を保持する。

```text
mission_predicted_rear_clear_time_sec
mission_predicted_rear_clear_distance_m
mission_validated_pass_distance_m
mission_validation_generation
mission_runtime_slack_distance_m
mission_runtime_slack_time_sec
mission_extension_pending
```

`phase_traveled_m` はPass内の実走距離として維持する。`mission_validated_pass_distance_m` は同じPass開始点基準とし、比較座標を混ぜない。

## 4. Runtime horizon decision

純粋関数 `resolve_committed_pass_horizon()` を追加し、毎周期の状態を次のいずれかへ分類する。

| Decision | 条件 | 動作 |
|---|---|---|
| `Keep` | 検証済み距離・時間に十分余裕がある | 現在missionを維持 |
| `Return` | rear-clear確認済み、Return corridor成立 | Returnへ移行 |
| `RefreshSameSide` | 検証終端が近い、rear-clear未成立 | 現在位置から同じ側だけを再評価 |
| `ExtendSameSide` | 新しい同じ側missionが成立 | missionを一括更新 |
| `HoldFrozenLine` | 延長不可、Returnも不可、現在の短区間は安全 | 現在横目標を保持しclosingを抑えて再評価 |
| `Abort` | 短区間の保持も不可能、またはhard limit超過 | 明示理由付きでRecovery |

判定順序:

1. hard guard
2. rear-clear済みReturn
3. horizon余裕十分ならKeep
4. 同じ側のRefresh/Extend
5. Return可能ならReturn
6. 短時間だけHoldFrozenLine
7. Abort

`HoldFrozenLine` は通常trajectoryを出さない。既存の固定横目標を短時間保持し、対象速度付近へclosingを落とす。無期限保持は禁止し、距離・時間上限を持つ。

## 5. Revalidation

再評価は検証終端に到達してからでは遅いため、lead distanceを持つ。

初期案:

- revalidation lead distance: 3.0 m
- revalidation lead time: 0.75 s
- same-side extension upper bound: 12.0 m
- absolute Pass distance upper bound: 24.0 m
- absolute Pass time upper bound: 10.0 s
- HoldFrozenLine上限: 1.0 s / 3.0 m

距離または時間の早い方で再評価を開始する。上限値はA/B対象であり、実装時にハードコードしない。

## 6. Same-side extension

Pass中の再評価は現在sideだけを探索する。

- 現在の`e_y`を開始点にする。
- 現在のfixed goalを第一候補にする。
- 必要なら同じ側corridor内で最小量だけgoalを調整する。
- 反対側candidateは生成しない。
- 更新はgoalだけでなく、closing、rear-clear予測、validated distance、deadline slackを一括反映する。

これにより対象の横揺れで左右を往復せず、壁形状に合わせた小さな同側補正だけを許可する。

## 7. Existing watchdogとの関係

既存watchdogは残す。

- 新horizon: 検証済み経路を使い切る前に再計画する
- 既存watchdog: 対象との縦関係が改善しない異常を検出する

新horizonは絶対距離・時間を監視するため、0.5 m進捗による再装填は行わない。

## 8. ログ

状態変化時だけ次を記録する。

```text
phase
target
side
pass_traveled
validated_pass_distance
predicted_rear_clear_distance/time
runtime_slack_distance/time
return_corridor_blocked
extension_result
decision
reason
mission_generation
```

40 Hzの毎周期ログにはしない。`Keep -> RefreshSameSide`、延長採用、Return、Hold、Abort時だけ出す。

## 9. 実装順序

1. rear-clear rolloutとhorizon decisionを純粋関数化する。
2. candidateにdynamic Pass距離を組み込む。
3. mission stateへ検証期限を保存する。
4. Pass runtime revalidationを接続する。
5. Hold/Abortを既存FSMへ接続する。
6. ログと実走評価を行う。

Pass runtime処理を直接さらに継ぎ足さず、純粋関数と状態更新を分ける。

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

`progress_watchdog_distance=32 m` は「前後関係が改善しないPass」を検出する。今回追加するhorizonは「missionがどこまで検証済みか」を管理する。両者は別責務とする。

## 1. Rear-clear rollout

既存kinematic rolloutへ次を追加する。

- predicted side-by-side time/distance
- predicted body-clear time/distance
- predicted rear-clear time/distance
- rear-clear時のego speed
- rear-clear後のReturn開始位置
- rollout終端での対象車との前後距離

rear-clear条件は実行時と同じ縦距離を使う。rear-clear確認用reserveは固定距離ではなく、確認時間と制御遅延から速度依存で求める。

```text
rear_clear_pass_distance =
  predicted_ego_distance_at_rear_clear - shift_distance

confirmation_reserve_distance =
  predicted_ego_speed_at_rear_clear
  * (rear_clear_confirm_sec + control_delay_sec)

predicted_pass_hold_distance =
  max(configured_min_pass_distance,
      rear_clear_pass_distance + confirmation_reserve_distance)
```

Return距離はPass距離へ混ぜず、別区間として保持する。予測Pass時間が初期budgetを超えるcandidateは通常追い越しとして採用しない。

## 2. End-to-end candidate validation

candidate pathは次の3区間を持つ。

```text
ShiftOut
  -> predicted rear-clearまで同じ側を保持
  -> Return
```

### 静的preflight

mission全体について次を成立必須とする。

- 壁footprint sweep
- map内判定
- 曲率速度cap
- 横加速度
- Return corridor

### 動的preflight

対象車と第三車両についてはV2X予測の信頼可能範囲だけを検証する。動的予測がReturnまで届かないことだけでcandidateを棄却しない。

```text
effective_valid_until_distance = min(
  static_valid_until_pass_m,
  dynamic_valid_until_pass_m)

dynamic_time_slack_sec =
  dynamic_valid_until_monotonic_sec - now_monotonic_sec
```

静的geometryの有効範囲は距離で管理する。rollout上の到達予測時間は、predicted Pass budgetとabsolute mission timeの評価値であり、静的mapの失効時刻ではない。

動的期限より前に再評価できる距離・時間slackがないcandidateは採用しない。実際のReturn開始はrear-clear観測後のままとする。

動的予測では次の時刻を分ける。

```text
planner_generated_at_sec
  planner結果を生成したcontroller monotonic時刻

prediction_epoch_monotonic_sec
  予測対象状態の基準時刻をcontroller monotonic clockへ正規化した値

prediction_horizon_sec
  prediction epochから先の予測長

dynamic_valid_until_monotonic_sec =
  prediction_epoch_monotonic_sec + prediction_horizon_sec
```

V2X source timestampがcontroller clockと異なる場合は、その値を直接比較しない。planner時点で求めたsource ageを使い、概念的に `prediction_epoch_monotonic = planner_now_monotonic - source_age` とする。planner生成時刻へ予測horizonを足し直してはならない。

## 3. Mission state

`OvertakeLineState`へ次の概念をまとめた `PassMissionValidation` を保持する。

```cpp
struct PassMissionValidation
{
  std::uint64_t generation{0};

  double pass_start_time_sec{0.0};
  double predicted_rear_clear_pass_m{0.0};
  double predicted_rear_clear_elapsed_sec{0.0};

  double static_valid_until_pass_m{0.0};
  double dynamic_valid_until_pass_m{0.0};
  double planner_generated_at_sec{0.0};
  double prediction_source_age_sec{0.0};
  double prediction_epoch_monotonic_sec{0.0};
  double prediction_horizon_sec{0.0};
  double dynamic_valid_until_monotonic_sec{0.0};

  double validated_pass_hold_distance_m{0.0};
  double return_start_pass_m{0.0};
  double return_distance_m{0.0};

  double absolute_pass_distance_limit_m{0.0};
  double absolute_pass_time_limit_sec{0.0};

  int extension_count{0};

  double hold_start_pass_m{0.0};
  double hold_start_elapsed_sec{0.0};
};
```

runtime slackは状態へ保存せず、毎周期計算する。

```text
distance_slack =
  min(static_valid_until_pass_m, dynamic_valid_until_pass_m)
  - phase_traveled_m

dynamic_time_slack =
  dynamic_valid_until_monotonic_sec - now_monotonic_sec

absolute_time_slack =
  pass_start_time_sec + absolute_pass_time_limit_sec
  - now_monotonic_sec
```

`phase_traveled_m` と全valid-until距離は最初のPass開始点基準とする。extension採用時は現在位置からのローカル値を絶対値へ変換する。

`pass_start_time_sec` は予測値ではなく、実際に最初の `ShiftOut -> Pass` 遷移が成立したcontroller monotonic時刻を一度だけ保存する。入口rolloutのpredicted Pass開始時刻との差をabsolute timerへ持ち込まない。rear-clear予測時間はfreshなPass horizon生成時点からの相対時間として再計算し、実際のPass開始時刻へ変換する。

```text
new_static_valid_until_pass_m =
  current_phase_traveled_m
  + extension.static_validated_distance_from_current_m

new_dynamic_valid_until_pass_m =
  current_phase_traveled_m
  + extension.dynamic_validated_distance_from_current_m

new_dynamic_valid_until_monotonic_sec =
  extension.prediction_epoch_monotonic_sec
  + extension.prediction_horizon_sec
```

Pass開始距離、Pass開始時刻、absolute budgetはextensionやHoldでリセットしない。

動的予測の時刻基準はplanner生成時刻ではなく、V2X source ageを反映したprediction epochとする。入口candidateの動的予測がShiftOut中に失効した場合、Pass進入時に即時再評価し、古い1秒予測を新しいPass horizonとして使い回さない。

## 4. Horizon actionとplanner結果の分離

毎周期の純粋判断は、planner処理やcommit結果を含めない。

```cpp
enum class PassHorizonAction
{
  Keep,
  Return,
  RequestSameSideExtension,
  EnterHold,
  Abort,
};
```

同側延長の探索結果は別構造体で返す。

```cpp
struct OvertakeMissionPath
{
  double fixed_goal_ey{0.0};
  double validated_pass_hold_distance_m{0.0};
  double return_start_pass_m{0.0};
  double return_distance_m{0.0};
  // Pass保持区間とReturn区間を含む、実際にpublishする置換経路。
};

struct SameSideExtensionCandidate
{
  std::uint64_t source_generation{0};
  double planner_generated_at_sec{0.0};
  double prediction_source_age_sec{0.0};
  double prediction_epoch_monotonic_sec{0.0};
  double prediction_horizon_sec{0.0};
  std::string target_id;
  int side_sign{0};

  OvertakeMissionPath replacement_path;
  double closing_speed_limit{0.0};

  double static_validated_distance_from_current_m{0.0};
  double dynamic_validated_distance_from_current_m{0.0};

  double predicted_rear_clear_distance_from_current_m{0.0};
  double predicted_rear_clear_time_from_current_sec{0.0};
};
```

採用条件は次をすべて満たすこととする。

- `source_generation == current generation`
- `now_sec < prediction_epoch_monotonic_sec + prediction_horizon_sec`
- `now_sec - planner_generated_at_sec <= extension_planner_result_max_age_sec`
- target ID一致
- side一致
- 現在phaseがPass、またはPass内部Hold
- 新しい実効valid-untilが現在値より前進
- absolute distance/time limit以内

採用時はreplacement path、fixed goal、closing、Pass保持距離、Return開始位置、Return距離、rear-clear予測、静的期限、動的期限を一括更新し、generationを増やす。pathの一部だけを先行更新してはならない。

## 5. ShiftOut / Pass境界

入口の動的予測はShiftOut中にも期限を消費する。ShiftOut中は次を毎周期確認する。

```text
remaining_dynamic_ttl =
  dynamic_valid_until_monotonic_sec - now_monotonic_sec

required_ttl =
  predicted_time_to_body_clear + revalidation_lead_time
```

`remaining_dynamic_ttl < required_ttl` になった時点でfresh horizonを要求する。stale予測のままShiftOutを継続しない。

Pass遷移条件は次へ固定する。

```text
fresh horizon成立
  -> Pass

fresh horizon未取得
かつ current footprint非重複
かつ 固定OvertakeLineの短区間が静的・動的に安全
  -> ShiftOut完了位置を固定
  -> closing speed = 0
  -> 最大1回だけ再評価

短区間も不成立
  -> side-by-sideならAbortToSafeSeparation
  -> 前後分離済みならRecovery
```

fresh horizon待機は1.0秒または3.0 mの早い方で終了し、再装填しない。古いhorizonでPassへ遷移する経路は設けない。

## 6. Runtime decision

判定順序は次とする。

1. 物理接触、map外、現在footprint検証不能
2. confirmed current overlap、Emergency
3. target continuity異常
4. solver failure
5. rear-clear済みかつReturn成立ならReturn
6. rear-clear済みのmargin-only violationならReturnを優先
7. absolute Pass distance/time limit確認
8. horizon余裕が十分ならKeep
9. 同側extensionを要求
10. extension不成立ならbounded Hold
11. Hold不能または上限超過ならAbort分類

距離と時間の両方を使う。

```text
normalized_slack = min(
  distance_slack / revalidation_lead_distance,
  dynamic_time_slack / revalidation_lead_time)

normalized_slack > 1.0  -> Keep
normalized_slack <= 1.0 -> RequestSameSideExtension
```

低速・停止時は時間、高速時は距離の期限が先に効く。

## 7. Same-side extension

Pass中の再評価は現在sideだけを探索する。

- 現在の`e_y`を開始点にする。
- 現在のfixed goalを第一候補にする。
- 必要なら同じ側corridor内で最小量だけgoalを調整する。
- 反対側candidateは生成しない。
- goal jumpには既存横加速度・壁sweep検証を適用する。
- 初期実装では1 missionにつき最大1回だけ延長する。
- 40 Hzで毎周期全探索せず、pending中は重複要求を抑止する。

mission作成時はgenerationを1とする。rear-clear、Return、Recovery、target変更時はpending extensionを無効化する。

## 8. HoldとAbort

Holdは上位Behaviorを増やさず、Pass内部substateとする。

```cpp
enum class PassHorizonMode
{
  Normal,
  Holding,
};
```

Hold中も通常/base trajectoryへ戻さず、現在の固定OvertakeLineをpublishし続ける。target ID、side、fixed lateral goal、lateral ownershipを維持し、closing speedを概ね0へ落とす。

Holdへ入れるのは次をすべて満たす場合に限る。

- 現在footprint非重複
- 現在の短区間が静的壁clear
- target continuity正常
- targetがselected sideへ侵入していない
- Emergencyではない
- solver正常

Holdは1.0秒または3.0 mの早い方で終了する。rear-clearがHold中に成立した場合は即Returnを評価する。

Abortは次へ分ける。

```text
AbortToRecovery
  対象が十分前または後ろで、通常経路へ戻っても非重複

AbortToSafeSeparation
  side-by-sideのため現在sideを維持し、closing <= 0で前後分離
```

`AbortToSafeSeparation` は新しい無期限待機状態にはしない。上限は1.0秒または3.0 mの早い方とする。Holdから連続して入る場合は同一fallback episodeとして開始点を共有し、上限を再装填しない。前後分離後にReturnまたはRecoveryへ移る。

上限に達した場合はHoldまたはSafeSeparationへ再入場せず、現在footprintとcorridorを再評価してhard abortへエスカレートする。ここでもabsolute Pass上限は継続する。

## 9. Wall guardの分離

次を混同しない。

- wall contact、map外、現在footprint検証不能：hard abort
- 追加wall margin不足：rear-clear済みならReturnを先に評価

既存の `ReturnBeforeWallMarginRecovery` と `RecoverWallMargin` の責務を維持する。

## 10. Existing watchdogとの関係

既存watchdogは残す。

- 新horizon：検証済み経路を使い切る前に再計画する
- 既存watchdog：対象との縦関係が改善しない異常を検出する
- absolute limit：extensionを含むmission全体の距離・時間を制限する

新horizonとabsolute limitは、0.5 m進捗による再装填を行わない。

## 11. 初期パラメータ

- revalidation lead distance: 3.0 m
- revalidation lead time: 0.75 s
- extension planner result max age: 0.10 s
- predicted Pass time budget: 8.0 s
- absolute Pass time limit: 10.0 s
- validated Pass soft distance: 24.0 m
- absolute Pass distance limit: 32.0 m
- same-side extension max distance: 8.0～12.0 m
- Hold上限: 1.0 s / 3.0 m
- ShiftOut fresh-horizon待機上限: 1.0 s / 3.0 m、再評価1回
- SafeSeparation上限: 1.0 s / 3.0 m（同一fallback episodeで共有）
- 初期版extension count: 1

全値を設定化する。24 m到達は即hard abortではなく、extension抑制とReturn／Hold判断を早めるsoft limitとする。

## 12. ログ

状態変化時だけ次を記録する。

```text
phase
pass_horizon_mode
target
side
pass_traveled
pass_elapsed
static_valid_until_distance
dynamic_valid_until_distance/time
effective_slack_distance/time
predicted_rear_clear_distance/time
absolute_limit_distance/time
extension_count
extension_result
decision
abort_class
reason
mission_generation
planner_generated_at
prediction_epoch
prediction_source_age
dynamic_expiry
mission_path_generation
pass_hold_distance
return_start_distance
return_distance
fallback_episode_elapsed/distance
```

40 Hzの毎周期ログにはしない。extension要求、延長採用・棄却、Return、Hold開始・終了、Abort時だけ出す。

## 13. 段階実装

### Phase 1: mission範囲の確立

- rear-clear rollout
- dynamic Pass distance
- Return静的preflight
- 静的validated distanceと、動的validated distance/time保存
- prediction epoch、source age、dynamic expiry保存
- Pass保持距離とReturn pathを含むmission path保存
- absolute budget保存

### Phase 2: 一回だけのreceding-horizon更新

- `Keep / Return / RequestSameSideExtension / Abort`
- generation付きatomic update
- same-side extension最大1回
- side-by-side safe separation
- fresh horizonなしでPassへ遷移しないShiftOut境界処理

### Phase 3: bounded fallback

- Pass内部Hold
- 複数回extension
- 動的予測不確実性への対応

Phase 1と2のログで必要性を確認するまで、Phase 3の複雑性を入れない。

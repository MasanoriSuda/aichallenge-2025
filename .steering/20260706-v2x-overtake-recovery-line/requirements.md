# V2X Overtake / Recovery Line Requirements

作成日: 2026-07-06
状態: Implemented / runtime verification pending

## 目的

V2X で前方車両を検出したときに、MPC がその場の制約だけで横へ逃げるのではなく、明示的な追い越しラインと復帰ラインを持って走れるようにする。

現状の C++ MPC は、前方リスク、到達可能 gap、停止車回避、Follow 中の preposition などを持つが、追い越しの「どこを通るか」「いつ戻るか」はまだ弱い。結果として、次の症状が残る。

- 低速車の真後ろに居座り、追い越し試行が遅れる。
- 直線追い越しで滑らかな U 字ではなく、S 字や壁へ吸われるような予測点になる。
- ヘアピンで内側へ入ろうとしてから急に外へ切り替わり、経路が破綻する。
- 追走から追い越しへ入る瞬間に横移動が大きすぎる。
- 抜いた後に戻る意思が弱く、壁側または外側へ残りやすい。

本 steering では、Pro との壁打ち結果にある全構想を一括実装せず、まず「追い越しライン・復帰ライン」に絞って設計する。

## Pro 提案との切り分け

今回扱うもの:

- 追い越し用の明示的な横オフセットライン。
- `FOLLOW_PREPARE` / `OVERTAKE_SHIFT` / `OVERTAKE_PASS` / `OVERTAKE_RETURN` / `RECOVERY` 相当のフェーズ管理。
- 追い越し開始から復帰までの横目標 `target_ey` を滑らかに生成する。
- ヘアピンや追い越し禁止区間では追い越し開始を抑制する。
- 壁側へ向かう予測点を抑える安全ガード。

今回扱わないもの:

- 本格的な `race_state_estimator` ノード分離。
- コース全体の `track_projector` パッケージ化。
- `boostRemaining` / `isBoosting` を使った boost 戦略。
- penalty 状態を使った速度計画。
- defense line、防御走行、後方車両への戦略対応。
- 2026 公式仕様未確定部分の topic 契約変更。
- `aichallenge_system/` の変更。

## 現状実装

現行 C++ MPC には次がある。

- `/v2x/vehicle_positions` の購読。
- `Cruise` / `Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` の大まかな状態。
- required decel による front risk arbitration。
- reachable gap 判定。
- `LowSpeedAvoidance` の local path planner。
- `Overtake` 中の pass-side target ramp。
- `Follow` 中の弱い preposition target。
- `domain_v_max` / `domain_a_max`。

不足しているのは、`Overtake` へ入った後の明示的な横ライン形状と復帰フェーズである。

## 必要機能

### 追い越しフェーズ

追い越しを次のフェーズに分ける。

- `Idle`: 通常走行。
- `FollowPrepare`: 前走車の真後ろを避け、追い越し候補側へ小さく寄る。
- `ShiftOut`: 追い越し側の横オフセットラインへ滑らかに移る。
- `Pass`: 前走車横を維持しながら抜く。
- `Return`: 前走車を抜き切った後、基準 trajectory へ滑らかに戻る。
- `Recovery`: 壁側・姿勢不安定・gap 喪失などで追い越しを中断し、安全側へ戻る。

既存の `V2XBehaviorState::Overtake` は大分類として維持し、その内部サブフェーズとして扱う。

### 横ライン生成

MPC horizon 上の `xr[e_y]` に対して、距離方向に滑らかな横目標列を生成する。

要求:

- step 的に横目標を切り替えない。
- `smoothstep` または quintic easing を使い、横速度・横加速度の急変を抑える。
- target は path constraints の `lb/ub` 内へ clip する。
- `Overtake` 中だけ強い横目標を使い、`Follow` 中は弱い preposition に留める。
- pass side は追い越し開始時に lock し、途中で左/右を頻繁に反転しない。

### 復帰判定

次を満たしたら `Return` へ入る。

- 対象前走車が自車後方へ一定距離以上抜けた。
- pass side に対象車両が残っていない。
- 復帰先 trajectory 側に十分な lateral clearance がある。
- カーブ入口や壁近傍で急復帰しない。

復帰開始後は、一定距離または一定時間、pass side を維持してから滑らかに戻る。

### Recovery 判定

次のときは `Recovery` へ入る。

- 追い越し target が path constraints の端に張り付く。
- 予測点が壁側へ向かい続ける。
- reachable gap が途中で失われた。
- side vehicle により復帰先が塞がれた。
- front risk が emergency になった。
- yaw error、steering rate、横加速度推定がしきい値を超える。

Recovery では新しい追い越しを開始せず、速度上限を下げ、基準 trajectory または安全側 corridor center へ戻す。

### 安全ガード

- 追い越し開始直前に curve guard / forbidden WP を確認する。
- 内側ヘアピンへの飛び込みを禁止する。
- target lateral が `lb/ub` 端に近すぎる場合は速度上限を下げる。
- 横移動の必要加速度がしきい値を超える場合は `Overtake` へ入らない。
- `SafetyBrake` 判定は追い越しより優先する。

## Config 要求

初期案:

```yaml
mpc:
  v2x_overtake_line_enabled: false
  v2x_overtake_line_shift_distance: 8.0
  v2x_overtake_line_pass_distance: 8.0
  v2x_overtake_line_return_distance: 10.0
  v2x_overtake_line_lateral_offset: 1.2
  v2x_overtake_line_target_bias: 0.8
  v2x_overtake_line_min_wall_clearance: 0.8
  v2x_overtake_line_max_lateral_accel: 2.5
  v2x_overtake_line_max_target_change: 0.25
  v2x_overtake_line_return_clear_distance: 4.0
  v2x_overtake_line_phase_hold_time: 0.3
  v2x_overtake_line_debug_log_enabled: true
```

既定値は `false` から始め、現行挙動を壊さない。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- `docs/spec/mpc-integration.md`
- 必要に応じて `.steering/20260706-v2x-front-risk-arbitration/` との整合

対象外:

- `/v2x/vehicle_positions` の topic 名・message 型変更。
- `/control/command/control_cmd` の topic 名・message 型変更。
- `aichallenge_system/` の変更。
- boost / penalty の実装。
- コース全体の複数 CSV trajectory 切り替え。

## 互換性要求

- `v2x_overtake_line_enabled=false` で現行挙動を維持する。
- 既存の `LowSpeedAvoidance` と gate2 挙動を壊さない。
- 既存の `front risk arbitration` と `SafetyBrake` を弱めない。
- `control_method=mpc` の launch 経路を維持する。
- config key が未指定でも起動できる。

## 受け入れ条件

- `make autoware-build` が成功する。
- `Overtake` 中の target lateral が step 変化せず、滑らかな U 字に近い予測点になる。
- 低速車の真後ろに居座る時間が減る。
- ヘアピン入口では無理な追い越し開始をしない。
- 前走車を抜き切った後、基準 trajectory 側へ戻るフェーズがある。
- `make gate2` の停止車追い越しを壊さない。
- `make dev2` / `make dev3` で追い越し不可理由と phase をログで追える。

## 未確定事項

- 横ライン target を既存 `xr[e_y]` に混ぜるだけで足りるか、path constraints 自体も補正するか。
- 復帰先を base trajectory にするか、corridor center にするか。
- 対象前走車 ID を V2X message から安定して保持できるか。
- section / lap 情報を使った追い越し禁止区間を入れるか。
- boost / penalty 戦略をどの steering で扱うか。

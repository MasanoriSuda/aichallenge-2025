# MPC Low Speed V2X Avoidance Design

作成日: 2026-07-05
状態: Draft

## 方針

高速追い越しとは別に、近距離停止車両を低速ですり抜ける状態を V2X behavior FSM に追加する。

現行の `V2XGapPlanner` は横方向 gap を計算できるため、新しい planner は作らず、低速回避状態では既存 gap planner を許可する。

## 状態追加

`V2XBehaviorState` に `LowSpeedAvoidance` を追加する。

優先度は次の扱いにする。

```text
Cruise < Overtake < Follow < LowSpeedAvoidance < SafetyBrake
```

## 判定

SafetyBrake 判定の直前で以下を満たす場合、`LowSpeedAvoidance` に倒す。

- 前方車両がある
- 前方距離が `v2x_low_speed_avoidance_distance` 以下
- 追い越し禁止 waypoint / 曲率条件に該当しない
- gap planner が feasible な gap を返す
- gap 幅が `v2x_low_speed_avoidance_min_gap_width` 以上

近距離停止車両がこの距離条件に入っているが、連続した安全 gap がまだ確認できない場合は、通常 `Overtake` へ落とさず `Follow` に倒す。回避ラインが確定する前に通常追い越しへ入ると、停止車両の前で左右へ振って接触しやすいためである。

`/v2x/vehicle_positions` は速度を直接持たず、C++ 側の速度は位置差分推定である。gate2 のように初期から前方車が約 7m にいるケースでは初期速度推定が不安定になりやすいため、低速回避開始条件では前方車速度を hard gate にしない。

gate2 の停止車両配置では直近車両だけを見ると、1、2台目を通過した後に3台目の認識が遅れて接触しやすい。`LowSpeedAvoidance` では停止車列を先読みするため、gap planner が見る V2X 車両を `v2x_low_speed_avoidance_lookahead_distance` 程度まで広げる。また通常レースでの不用意な車両間すり抜けを避けるため `v2x_vehicle_vehicle_gap_enabled: false` は維持しつつ、`LowSpeedAvoidance` の計算時だけ車両間 gap を候補へ戻す。

内外両方に gap がある場合、ベース trajectory の左右寄りに引っ張られると gate2 の初期位置で壁側へ吸われる。`LowSpeedAvoidance` の gap 選択では、初期 desired を現在 trajectory 偏差ではなく走行可能 corridor 中央に置き、gap 幅は安全判定と同点時の補助に留める。カーブ中も曲率だけで壁側を選ばず、各 horizon 点の通れる gap を再選択する。

ただし曲率だけで `base.lower/base.upper` を desired にすると、gate2 で壁側へ寄り続けて接触する。低速回避では gap 幅と直前 target の連続性を優先し、曲率単独では壁側を desired にしない。

V2X 車両の横方向禁止幅は、相手車両半幅だけではなく自車半幅も含める。全幅 1.45m 同士なら `v2x_vehicle_radius` は 1.45m 程度を基準にする。1.6m まで膨らませると gate2 の外側 gap が通過不能判定になりやすい。一方で前後方向まで同じ値で膨らませると停止車両が過大に長く見えるため、前後方向は `v2x_vehicle_length` と自車長で別計算する。

`v2x_self_filter_radius` は V2X に自車が混ざる場合の除外用である。gate2 の V2X は他車のみなので、至近距離の他車を誤って無視しないよう小さめにする。

`SafetyBrake` の横方向判定に通常 corridor 全体を使うと、停止車両の横を通過中でも前方距離だけでブレーキに落ちる。危険判定は `v2x_vehicle_radius + v2x_prediction_margin` の衝突幅に重なった場合に限定する。

3台目付近で曲率禁止条件に入ると `LowSpeedAvoidance` から `Follow` へ落ちて停止するため、低速回避に入った後は gap が残る限り継続を許可する。通常の新規追い越し開始は禁止条件を維持する。

3台目の横を抜ける前に前方車両判定が外れると、`LowSpeedAvoidance` から `Cruise` へ戻って速度が上がり接触しやすい。低速回避に入った後は、`v2x_low_speed_avoidance_clear_distance` 以内に V2X 車両が前方または横方向に残る間だけ徐行状態を保持し、抜け切り後半の早すぎる復帰を抑える。完全に後方へ抜けた車両は保持条件に使わず、外側 target lock を解除して通常 trajectory へ戻す。

各 horizon 点で gap を再選択し、近い点は2台目、遠い点は3台目、さらに先は通常 trajectory という目標列を作る。3台以上の停止車列では、最初に選んだ `target_ey` を絶対値として保持すると3台目で壁側へ張り付きやすい。低速回避では直前 target を連続性のヒントとしてだけ使い、対象 V2X がある horizon 点ごとに通れる gap を再選択する。`LowSpeedAvoidance` から抜けるときは低速回避で使った target 記憶もクリアする。

ただし通過側まで毎 horizon で自由に選ぶと、初期 trajectory に引っ張られて一瞬だけ壁側へ向かう。`LowSpeedAvoidance` では `v2x_low_speed_pass_side` で `auto` / `left` / `right` を選べるようにし、`auto` では最初に選んだ通過側だけを side lock として保持する。固定するのは側だけで、`target_ey` の絶対値は車両ごとに再選択する。

実制御時は、障害物と重なる horizon 点だけでなく手前の horizon 点にも side-pass target をソフト参照として入れる。これにより「最初に既存 trajectory 側へ寄ってから戻す」挙動を抑える。候補判定時にはこのランプ target を入れず、実際に障害物と重なる horizon 点だけで連続 gap を評価する。

低速回避開始条件は、単一 horizon 点だけの gap で成立させず、`v2x_low_speed_avoidance_min_gap_points` 以上の連続点で `v2x_low_speed_avoidance_min_gap_width` 以上の gap があることを要求する。これにより、瞬間的な隙間だけで停止車列へ突入することを抑える。

低速回避では壁と車に挟まれた gap の中央を基本目標にする。壁側には `v2x_wall_clearance_margin` を適用し、自車中心が壁へ張り付く経路を避ける。車両側へ寄せる bias は3台目との接触余白を削るため、gate2 向けには `v2x_wall_avoidance_bias: 0.0` を使う。

## MPC 反映

`LowSpeedAvoidance` では次を行う。

- `allow_gap_planner = true`
- `target_velocity_limit = v2x_low_speed_avoidance_velocity`
- gap planner が出した `lb/ub` と `target_ey` を既存の処理で MPC に反映

## 注意

これは一時参照軌道を生成する本格的な planner ではない。停止車両の近距離すり抜けを成立させるための最小追加である。

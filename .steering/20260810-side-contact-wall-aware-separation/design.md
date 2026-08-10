# Design

## 背景

最新走行ログでは `contact_continue=0` のみで、横接触継続処理が発動していなかった。現行は course-frame の実車体重複確定、front-cap release、forward-completion latch を同時に要求するため、映像上は接触していても V2X と制御周期の境界では処理へ入れない場合がある。

## 方針

### 1. 近接接触エンベロープ

Pass 中の locked target について、物理車体幅へ `near_gap` を加えた矩形内に入った状態を連続確認する。既定値は `near_gap=0.05 m`、`confirm=0.10 s` とする。

接触証拠は次の OR とする。

```text
actual body overlap confirmed
OR
near-contact envelope confirmed
```

その後も対象継続性、横並び形状、closing speed、横相対速度、経過時間、前進進捗で制限する。

### 2. 接触継続の所有条件

Pass phase と locked target continuity を必須とする。一方、接触後に front-cap release や forward-completion latch が落ちたことだけを理由に継続処理を無効化しない。前方衝突は `minimum_absolute_lateral` で除外する。

### 3. 壁制約付き分離バイアス

requested bias は committed side 符号付き 0.15 m。全 horizon の動的壁境界と `min_wall_clearance` から、同じ側へ移動可能な量だけを適用する。

```text
requested_goal = base_goal + signed_bias
applied_goal   = clamp(requested_goal, wall_feasible_interval)
```

現在 goal が既に壁側限界を越えている場合、接触バイアスはゼロとし、壁方向へ悪化させない。

## 影響範囲

- ROS topic/service/message 契約の変更なし
- 評価基盤の変更なし
- 通常 Follow、ShiftOut、Return の目標変更なし
- 新規 YAML parameter は後方互換の既定値を持つ

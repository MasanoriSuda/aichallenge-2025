# Design

## 原因

`evaluate_gap_plan_static_wall_preflight()` は低速停止車向けに作られた入力条件を、
通常の動的障害物回避にも適用していた。通常 GapPlanner は実行可能な
`target_ey` / `target_active` を生成する一方、低速車専用の `pass_side_sign` を
設定しないため、壁形状を検査する前に `invalid local path target` で落ちていた。

## 方針

### 1. 実行 sample 契約を純粋関数へ分離

`v2x_overtake_core` に、planner の target vector と activity mask だけを検証する
純粋関数を追加する。controller 固有の wall map 処理と入力契約を分離し、理由別の
単体テストを可能にする。

### 2. 実際に所有する経路範囲だけを wall preflight

- activity mask が空、または全 sample active: horizon 全体を検査する。
- 疎な mask: 最後の active sample に 2 sample の末尾余裕を加えた範囲まで検査する。

通常の動的回避は receding horizon で毎周期再検証されるため、未所有の遠方 horizon
全体を毎回 swept-footprint 検査しない。これにより 5 active sample に対して約1000 pose
を走査していた過剰計算も抑える。

### 3. 実行参照との整合

inactive sample は 0 m 固定ではなく、MPC と同じ corridor center reference
`clip(center_bias * center, lb, ub)` を使用する。active sample は同じ base reference と
`target_bias` で合成し、wall preflight と実際の MPC 横参照の不一致を減らす。

### 4. ログ

既存の change-aware decision trace に次を追加する。

- `preflight_samples=execution/active`
- `preflight_range=first:last`（active sample がない場合は `none`）
- `preflight_invalid_index=<index|none>`
- `preflight_poses=<swept footprint pose count>`

連続値やactive範囲だけの変化では40 Hzログを再出力しない。契約理由・異常indexの
変化は即時に出し、active範囲は状態変化時または周期サマリーの行へ併記する。

## 影響範囲

- `v2x_overtake_core`: 実行 sample 契約の型と純粋関数
- `mpc_controller_cpp.cpp`: wall preflight と診断伝播
- `overtake_decision_trace`: 一行決定ログ
- 既存テスト: 契約・表示の回帰試験

外部インターフェースと設定ファイルには影響しない。

# MPC Wall-Edge Trajectory Tracking Config Tasklist

作成日: 2026-07-04
状態: Draft

## Definition of Done

- `center_bias` と `safety_margin_scale` が `config.yaml` から設定できる。
- key 未指定時は現行挙動に戻る。
- `center_bias=1.0`, `safety_margin_scale=1.0` で既存挙動を維持できる。
- `center_bias=0.0` で左右制約中央ではなく trajectory 線を目標にできる。
- `safety_margin_scale` で path constraint の余白を段階調整できる。
- `make autoware-build` が成功する。
- `/control/command/control_cmd` の topic 契約を壊さない。
- 検証した値と結果を記録する。

## Phase 0: 現状整理

- [x] 中央寄せの原因箇所を特定する。
- [x] safety margin の原因箇所を特定する。
- [x] config 化する方針を決める。
- [x] ステアリング文書を作成する。

## Phase 1: Config 追加設計

- [x] `MpcConfig` に `center_bias` を追加する。
- [x] `MpcConfig` に `safety_margin_scale` を追加する。
- [x] YAML key 欠落時の fallback を `1.0` にする。
- [x] 範囲外値の clamp 方針を実装前に決める。
- [x] `config.yaml` に key とコメントを追加する。
- [ ] 必要なら `sim_config.yaml` にも key を追加する。

## Phase 2: C++ 実装

- [x] `load_config()` で `center_bias` を読む。
- [x] `load_config()` で `safety_margin_scale` を読む。
- [x] `BicycleModel` に `safety_margin_scale` を渡す。
- [x] `compute_safety_margin()` を scale 対応にする。
- [x] `MPC::init_problem()` の `xr` を `center_bias` 対応にする。
- [x] warning / log が必要か判断する。

## Phase 3: Build Verification

- [x] `make autoware-build` を実行する。
- [x] `center_bias` / `safety_margin_scale` key ありで起動確認する。
- [ ] key なし YAML でも fallback で起動できることを確認する。

## Phase 4: Runtime Verification

- [ ] `center_bias=1.0`, `safety_margin_scale=1.0` で baseline を確認する。
- [ ] `center_bias=0.0`, `safety_margin_scale=1.0` で中央寄せが消えるか確認する。
- [ ] `center_bias=0.0`, `safety_margin_scale=0.7` を確認する。
- [ ] `center_bias=0.0`, `safety_margin_scale=0.5` を確認する。
- [ ] 必要時のみ `safety_margin_scale=0.3` を確認する。
- [ ] `safety_margin_scale=0.0` は限界確認として扱い、接触リスクを記録する。

## Phase 5: Result Review

- [ ] `/control/command/control_cmd` の publish rate と値を確認する。
- [ ] trajectory からの横偏差を確認する。
- [ ] wall / over / crash penalty を確認する。
- [ ] lap time への影響を確認する。
- [ ] infeasible fallback が増えていないか確認する。
- [ ] 採用する暫定値を記録する。

## Phase 6: Documentation

- [x] `multi_purpose_mpc_ros/README.md` に config の意味を追記する。
- [x] 必要なら `docs/spec/mpc-integration.md` に追記する。
- [x] 評価・提出契約に影響しないことを確認する。

## 検証メモ

未実行。実装後に以下を記録する。

```text
center_bias:
safety_margin_scale:
trajectory:
command:
result:
notes:
```

## 実装メモ: 2026-07-04

- `mpc_controller_cpp.cpp` に `center_bias` と `safety_margin_scale` を追加した。
- `center_bias` は `0.0..1.0` に clamp し、未指定時は `1.0` とした。
- `safety_margin_scale` は負値を `0.0` に clamp し、未指定時は `1.0` とした。
- `config.yaml` は初回検証用に `center_bias: 0.0`, `safety_margin_scale: 1.0` とした。
- 検証済み: `make autoware-build` 成功。
- 短時間起動確認: `ros2 run multi_purpose_mpc_ros mpc_controller_cpp ...` は config 読み込み後に odometry 待ちまで進んだ。`timeout 5s` で停止したため終了コードは 124。

# MPC Python to C++ Faithful Port Tasklist

作成日: 2026-07-04
状態: Draft

## Definition of Done

- Python MPC と C++ MPC の主要中間値を比較する fixture がある。
- C++ MPC が `control_method=mpc` で起動する。
- `/control/command/control_cmd` の出力契約が維持される。
- `make autoware-build` が成功する。
- 走行確認が必要な差分では `make dev` または `make gate*` の結果を記録する。
- 仕様説明が `docs/spec/` または `docs/interface/` に反映される。

## Phase 0: 現状固定

- [x] 既存の MPC launch 経路を確認する。
- [x] Python MPC の主要対象ファイルを確認する。
- [x] ステアリング文書を作成する。
- [x] Python 実装を正本として扱うことを実装ブランチの説明に明記する。
- [x] 移植前後の `make autoware-build` 結果を記録する。
- [ ] 現行 Python MPC の起動ログ、topic list、control command 出力を記録する。

## Phase 1: Python Fixture 作成

- [ ] fixture 出力用の Python スクリプトを追加する。
- [ ] config 読み込み結果を JSON 化する。
- [ ] map の metadata、cell count、代表 `w2m` / `m2w` 結果を保存する。
- [ ] reference path の waypoint 数と代表 waypoint を保存する。
- [ ] speed profile の代表 `v_ref` を保存する。
- [ ] bicycle model の `t2s`、`s2t`、`linearize` 結果を保存する。
- [ ] MPC の `P`、`q`、`A`、`l`、`u` を比較可能な形式で保存する。
- [ ] first control の `v`、`delta`、`max_delta` を保存する。

## Phase 2: C++ 基盤追加

- [x] `multi_purpose_mpc_ros` に C++ MPC executable を追加する。
- [x] CMake と `package.xml` に必要最小限の依存を追加する。
- [x] YAML 読み込み構造体を追加する。
- [x] CSV 読み込み helper を追加する。
- [ ] fixture 比較用の C++ テストまたは検証 executable を追加する。
- [x] Python 実行系を残したまま C++ 実行系をビルドできる状態にする。

## Phase 3: Core 移植

- [x] `core/utils.py` を C++ に移植する。
- [x] `core/map.py` を C++ に移植する。
- [ ] `line_aa` 相当処理の cell sequence を Python と比較する。
- [ ] `remove_small_holes` 相当処理を Python と比較する。
- [x] `core/reference_path.py` の waypoint 構築を C++ に移植する。
- [x] path width と border cell 計算を C++ に移植する。
- [x] speed profile の OSQP 問題を C++ に移植する。
- [x] `core/spatial_bicycle_models.py` を C++ に移植する。
- [x] `core/MPC.py` の OSQP 問題生成を C++ に移植する。
- [ ] infeasible fallback と steering rate limit を Python と比較する。

## Phase 4: ROS Node 移植

- [x] `MPCController` の parameter declare / update callback を移植する。
- [x] publisher / subscriber と QoS を移植する。
- [x] odometry、trajectory、path constraints、stop request callback を移植する。
- [ ] V2X callback と obstacle filtering を移植する。
- [x] wait 処理と control loop を移植する。
- [x] Ackermann command 生成、raw publish、gain 適用順序を移植する。
- [x] marker publish と dummy visualization topic publish を移植する。
- [x] stop 処理を移植する。

## Phase 5: Parity 検証

- [ ] fixture 比較を CI または手元コマンドで実行できるようにする。
- [ ] map / reference path / speed profile の一致を確認する。
- [ ] bicycle model の一致を確認する。
- [ ] MPC matrix の一致を確認する。
- [ ] first control の一致を確認する。
- [ ] 短い odometry sequence で command 出力の一致を確認する。
- [ ] 差分が残る場合は、理由、許容誤差、評価影響を記録する。

## Phase 6: Launch 切り替え

- [x] `mpc.launch.xml` の実行先を C++ MPC に切り替える。
- [x] `control_method=mpc` の外部契約が変わっていないことを確認する。
- [x] Python venv / pip install が runtime 必須でなくなったことを確認する。
- [x] 比較用 Python 実装を残すか削除するかを判断し、理由を記録する。

## Phase 7: 評価・ドキュメント

- [x] `make autoware-build` を実行して結果を記録する。
- [ ] `make dev` または `make gate*` で起動・制御出力を確認する。
- [x] `/control/command/control_cmd` の publish を確認する。
- [ ] `/mpc/prediction` と `/mpc/ref_path` の publish を確認する。
- [x] `docs/spec/mpc-integration.md` を更新する。
- [x] `docs/interface/participant-interface.md` を更新する。
- [ ] 変更内容、検証コマンド、未検証リスクを最終報告にまとめる。

## 実装メモ: 2026-07-04

- `mpc_controller_cpp` を追加し、`control/mpc.launch.xml` の実行先を C++ に切り替えた。
- Python 実装は比較・検証用に残した。補助 Python scripts のため venv 作成はビルド時に残るが、通常の `control_method=mpc` 実行経路では必須ではない。
- 検証済み: `make autoware-build` 成功、テスト Odometry 投入で `/control/command/control_cmd` publish を確認。
- 未完了: golden fixture による Python/C++ 数値一致確認、V2X obstacle avoidance の完全移植、`make dev` / `make gate*` の走行確認。

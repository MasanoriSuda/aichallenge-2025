# Task list

## 設計

- [x] Pro の指摘を現行 HEAD と照合する。
- [x] frozen mission と early side replan の到達条件を確認する。
- [x] 最新ログで side replan 未発動と片側未評価を確認する。
- [x] shadow 評価、no-return、atomic replacement の契約を定義する。

## 実装

- [x] opponent-driven shadow replan 用の request/resolution と reason を pure core に追加する。
- [x] no-return 判定と切替ヒステリシスの単体テストを追加する。
- [x] frozen mission 中も左右の完全候補を一定周期で評価できる経路を追加する。
- [x] 現在状態から反対側の ShiftOut/Pass/rear-clear/Return を preflight する。
- [x] frozen `OvertakePassPlan` の atomic replacement を追加する。
- [x] side replacement count と absolute Pass budget の保持を追加する。
- [x] current-side failure 時の fallback 順序へ反対側評価を挿入する。
- [x] no-return 後の切替禁止を controller transition に接続する。
- [x] 状態変化ログと集約 debug field を追加する。
- [x] `make autoware-build` と対象 package tests を実行する。

## 動的確認（オペレーター）

- [ ] `make dev2` で先行車が片側へ寄る場面を収録する。
- [ ] 反対側 opportunity の検出と stable timer を確認する。
- [ ] no-return 前の atomic replacement を確認する。
- [ ] 横並び後の side switch がないことを確認する。
- [ ] 置換後の rear-clear、Return、Idle 完遂を確認する。
- [ ] 接触、wall Recovery、SafetyBrake、solver failure の増減を比較する。

## Definition of Done

- [x] 通常の frozen mission で opponent-driven side replan が到達可能である。
- [x] 反対側の一時開放では切り替えず、継続した完全経路だけを採用する。
- [ ] 反対側成立中に FollowPrepare/SafeSeparation へ直接落ちない。
- [x] 1 mission あたり最大 1 回、no-return 前だけ切り替わる。
- [x] 既存 ROS 2・評価インターフェースに変更がない。
- [ ] 静的検証と動的確認結果を tasklist に記録する。

## 実施結果

- `make autoware-build`: 成功（25 packages）。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core`: 841 tests、failure 0。
- `git diff --check`: 成功。
- `make dev2`: 未実施。動的確認項目と残りの Definition of Done は走行ログ取得後に判定する。

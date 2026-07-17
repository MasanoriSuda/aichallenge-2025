# Task List: Overtake Attack Window Experiment

- [x] Requirements / designを作成
- [x] pure coreへstage speed、path-time、completion feasibility helperを追加
- [x] MPC adapterへstage speedとcompletion guardを統合
- [x] V2X gap predictionをpath-time化
- [x] 実験用configを更新
- [x] V2X debug logへcompletion距離・speed stageを追加
- [x] pure helper単体テストを追加
- [x] `docs/spec/mpc-integration.md`を更新
- [x] 対象単体テストを実行（26 tests passed）
- [x] `make autoware-build`を実行（25 packages succeeded）
- [x] `make dev3`を実行し、追い越し・接触・停止・solverログを確認

## Definition of Done

- 対象テストとビルドが成功する。
- hard safety gateと評価インターフェースを変更していない。
- dev3の実験結果、未解決事項、次のA/B候補が記録されている。

実験結果は`results.md`を参照する。

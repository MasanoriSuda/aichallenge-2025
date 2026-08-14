# Tasklist

- [x] 最新走行のbody-clear解除とRecovery経路を特定する
- [x] rear-clear制約解除契約と単体テストを追加する
- [x] controllerからrear-clear状態をreceding horizonへ渡す
- [x] Return延期時のcurrent-side Pass holdを追加する
- [x] 診断ログをrear-clear契約へ合わせる
- [x] 対象packageの単体テストとビルドを実行する
- [x] 検証結果と残課題を記録する

## 試走確認

- rear-clear前のdebugログで `rear_release=0` が維持されること。
- targetが前方にいる間、予測ホライズンがtarget boundsを保持すること。
- Return preflight不成立時に `rear-clear Return deferred; retained current-side Pass`
  が出て、Recoveryへ直行しないこと。
- `Pass -> Return -> Idle`、target overlap、wall Recovery、ラップ時間を前回runと比較する。

## 静的・単体検証

- `make autoware-build`: 成功（25 packages）
- `colcon build --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets成功
- `colcon test-result --verbose`: 1110 tests、failure/error 0
- `test_v2x_overtake_core`: 571 tests、failure/error 0
- `git diff --check`: 成功

## 残課題

- 動的効果確認は未実施。前回runの5試行に対して正常Return数とRecovery数を比較する。
- MPCC-liteは制御callback内の同期処理のまま。非同期workerとFrenet DP corridorは
  本変更の対象外。

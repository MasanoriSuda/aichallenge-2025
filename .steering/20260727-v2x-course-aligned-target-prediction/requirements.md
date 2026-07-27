# Requirements

## 目的

dev2走行 `output/20260727-085009` で、P1の追い越しは2回完了した一方、
新規ShiftOut 27回に対して `locked target no longer executable` が22回発生した。
ヘアピン中もV2X車両をCartesian直線等速で予測しているため、相手が実際には
コースに沿って旋回した次の観測で、追い越しcorridorが急に実行不能となる可能性がある。

V2X相手車両を基準コースに沿って予測し、ShiftOut/Pass中のcorridor判定を安定させる。

## 必須要件

- 相手車両をMPCの基準コースへ投影できた場合、コース進行度と接線方向速度で将来位置を予測する。
- 相手の現在のコース横位置を予測期間中は保持する。
- 既存の `v2x_prediction_use_path_time` によるhorizon時刻計算を維持する。
- コース投影が無効、範囲外、または非有限の場合は現行のCartesian直線等速予測へ戻す。
- 壁余裕、横加速度、車体footprint、V2X timeout/jump判定を緩和しない。
- `/v2x/vehicle_positions`、`/control/command/control_cmd`、Domain構成などの
  ROS 2インターフェースを変更しない。
- 設定で新予測を無効化し、現行挙動へ戻せるようにする。

## 対象範囲

- `multi_purpose_mpc_ros` のV2X gap planner
- 純粋関数としてのコース追従予測選択ロジック
- 設定、起動ログ、単体テスト
- MPC統合仕様のV2X予測説明

## 対象外

- V2X messageへのyaw追加
- AWSIMまたは評価基盤の変更
- 壁・横加速度guardの閾値変更
- 追い越し速度、追従距離、Recovery速度の調整
- 実車での有効化確認

## Definition of Done

- 新設定が有効な場合、投影可能なV2X車両はコース進行度で予測される。
- 投影不能時のCartesian fallbackを単体テストで確認できる。
- コース追従予測の進行度・横位置計算を単体テストで確認できる。
- 対象packageの単体テストとビルドが成功する。
- dev2効果確認項目をtasklistに残す。

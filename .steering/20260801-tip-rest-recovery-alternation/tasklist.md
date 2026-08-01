# Tasklist

- [x] 最新D2ログと復帰FSMを照合する
- [x] 距離誤流用とForward無限再試行を特定する
- [x] 実駆動方向を優先する脱出方向解決を追加する
- [x] 脱出確認をmaneuver距離へ変更する
- [x] Forward連続時間切れ後のReverse強制を追加する
- [x] force-motion候補選択にReverse強制を反映する
- [x] 単体テストを追加する
- [x] パッケージビルドとテストを実行する

## 検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功
- `git diff --check`: 問題なし

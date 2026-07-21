# Tasklist

- [x] レビュー指摘を現行コードへ照合する。
- [x] 変更範囲と非対象を確定する。
- [x] 停止車回避のフェーズ別速度を実装する。
- [x] waypoint association coreを実装する。
- [x] simulation-only recovery fault retryを実装する。
- [x] 単体テストを追加する。
- [x] 正本仕様を更新する。
- [x] 対象テストとAutowareビルドを実行する。

## Verification

- `make autoware-build`: 25 packages成功。
- 今回のpure C++対象テスト: 3/3成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 22/22 CTest target成功。
- `mpc_controller_cpp`を現行`config.yaml`で3秒起動し、新規設定を含む初期化成功を確認。
- 修正版`traj_mincurv.csv`の閉路端点間隔は約0.999 mであるため、旧「重複終端を1点削除」テストをdistinct endpoint保持契約へ更新。

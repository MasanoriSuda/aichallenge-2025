# Results

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 58件成功
- package全体test: 19 test target中18件成功
  - `test_path_core` の既存1件が失敗。現在の `traj_mincurv.csv` に旧テストが期待する
    circular endpoint重複がないためで、今回の追い越し変更とは無関係。
- `make dev3`: `output/20260719-185846` で実走確認

## dev3観測

D2はWP130、前方距離9.39 mでOvertakeへ入り、5.61 mでもOvertakeを維持した。
旧実装で中断していた5.0 mを越えて4.28 mまで継続し、距離ガードでは中断しなかった。
その後WP148で `before-curve overtake blocked` によりFollowへ遷移した。

したがって、開始5.0 m / 継続2.5 mの距離ヒステリシスは意図どおり動作した。
次に追い越し成立率を上げる場合は、soft curve中のactive pass継続条件を独立して調整する。

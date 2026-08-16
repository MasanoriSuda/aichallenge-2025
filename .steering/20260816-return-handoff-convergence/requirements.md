# Requirements

## 目的

追い越しReturnが固定距離の消化だけで通常経路へ制御権を渡し、横位置・姿勢が未収束のまま通常MPCへ切り替わる不整合を解消する。

## 背景

`20260816-101204`ではrolling DP更新が5回採用され、2回の`Pass -> Return -> Idle`を確認できた。一方、5回目のReturn完了後に高速度のまま経路誤差が拡大し、OSQP不収束、waypoint association loss、Stuck Recoveryへ連鎖した。

現行のReturn完了条件は、横誤差が小さい場合に加え、Return距離を走り切った場合にも姿勢誤差を確認せずIdleへ遷移する。

## 要求

- Return完了には通常経路に対する横位置と姿勢の両方の収束を要求する。
- 収束は一周期ではなく短い連続確認時間を要求する。
- Return距離を消化しても未収束ならReturn参照を継続する。
- corridor blocked、solver recovery、無効観測中はhandoffしない。
- 距離到達時のhandoff延期と最終handoffを、イベントログだけで判別できるようにする。
- ROS 2 topic、message、launch、評価結果schemaは変更しない。

## 制約

- 変更は`aichallenge_submit/multi_purpose_mpc_ros`に閉じる。
- 既存のReturn preflight参照、壁guard、Recovery、MPC制約は緩和しない。
- 終盤に観測されたhairpinでのOSQP不収束そのものは別課題とし、今回の修正は危険なhandoffを防ぐ範囲に限定する。

## Definition of Done

- 距離到達のみではReturnが完了しない。
- 横位置・姿勢が閾値内で安定した場合だけReturnが完了する。
- 無効観測または一時的な収束逸脱で確認時計がリセットされる。
- config/config_for_cloudで同じ設定を持つ。
- 対象packageがビルドし、関連テストが成功する。
- 変更をコミットする。

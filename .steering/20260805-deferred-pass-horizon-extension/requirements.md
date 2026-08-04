# Pass horizon延長の早期発火修正

## 背景

`20260805-001505`のd1では、連続外側切替が0回の一方、`ShiftOut -> Pass` 12回に対して
`Pass -> Recovery`が11回発生した。

Pass開始直後、予測rear-clear距離がcommitted static horizonを少しでも超えると
`rear_clear_window_replan_required`が即時trueとなっていた。その結果、まだ約20 m以上の
検証済み経路が残っていても全経路の延長preflightを実行し、横加速度、将来のouter-role
反転、goal adjustment等でSafeSeparationへ移行していた。

## 要求

- rear-clear予測がcommitted horizon外という事実と、延長実行時期を分離する。
- committed static pathの残距離が既存`revalidation_lead_distance`以下になるまで、
  rear-clearを理由とする延長を要求しない。
- predicted footprint overlap、dynamic prediction expiry、absolute Pass limit等の既存hard
  triggerは変更しない。
- 延期待ちの間も、rolling outer replanを先に評価できる現在の実行順序を維持する。
- 延長が必要だがまだlead window外であることを、1 Passにつき1回だけログで確認できる。
- ROS 2 topic/service/message契約と既存設定値は変更しない。

## 完了条件

- Pass開始直後、static horizon残距離がleadより十分長ければ延長しない単体試験がある。
- lead境界到達時には延長要求へ変わる単体試験がある。
- `make autoware-build`、対象package test、`git diff --check`が成功する。


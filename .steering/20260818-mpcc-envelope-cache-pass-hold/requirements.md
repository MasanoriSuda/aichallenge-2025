# Requirements

## 目的

`20260818-084301` の試走で確認した MPCC 実行周期の悪化と、横離隔を獲得した
Pass が将来の target-wall 競合だけで Dynamic Mission Wait へ移る事象を減らす。

## 変更範囲

- 静的地図上の物理 wall envelope 探索結果を、同一幾何条件では再利用する。
- Pass の横離隔 latch 後は、現在の実車体・壁 hard guard が安全な間、将来の
  target-wall 競合を再計画トリガとして扱い、同側の last-feasible 軌道を短時間維持する。
- 実接触、壁 margin 違反、地図範囲外、EmergencyBrake、solver recovery は従来どおり
  hard fault とする。

## 制約

- ROS 2 topic/service、提出インターフェースは変更しない。
- 既存のユーザー変更 (`config.yaml`, `result-summary.json`) は変更・コミットしない。
- 追い越しの clearance や加減速度パラメータは変更しない。

## Definition of Done

- envelope cache が幾何変更時に破棄され、上限付きである。
- cache hit でも最終の物理 footprint 再検証を省略しない。
- latched Pass の保持条件と hard fault 拒否条件に単体テストがある。
- package build/test が成功する。

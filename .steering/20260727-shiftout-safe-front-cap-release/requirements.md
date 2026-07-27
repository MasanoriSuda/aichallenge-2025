# Requirements

## 目的

`output/20260727-232326/d1`で確認した、ShiftOut中に横離隔が解除閾値へ
達しただけで前走車由来速度capを解除し、壁制約中の横ラインを全速側で
継続する問題を解消する。

## 根拠

- `1785162476.850`: ShiftOut中、横離隔`1.52 m`、targetは`7.53 m`前方でcap解除
- 同区間は`lat_limited=1`、`wall_limited=1`、`static_wall_limited=1`
- `1785162478.000`: `Pass -> Recovery`、
  `reason=actual footprint wall margin violated`
- 別試行も横離隔`1.51 m`、target`5.13 m`前方で解除後、約0.18秒でRecovery

横離隔1.50 mは車体間の横重なり解除目安であり、安全な追い越し線の完成を
示さない。

## 変更範囲

- ShiftOut / Passのlocked target由来front-cap解除条件
- OverTakeLine horizon評価と速度参照の適用順序
- pure helper単体テスト
- MPC暫定仕様

## 制約

- ShiftOut中のclosing speed上限は既存設定を維持する。
- 横離隔の解除・再適用閾値`1.50 / 1.30 m`は変更しない。
- wall、static wall、横加速度、Emergency、別の前方車、curveの各制約は緩和しない。
- start-grid breakoutの専用速度方針は変更しない。
- ROS topic/service、Domain、評価成果物の契約は変更しない。

## 完了条件

- 横離隔だけではfront-capを解除しない。
- pass側の横目標へ到達し、かつ実行horizonがwall/static wall/横加速度の
  いずれにも制限されていない場合だけ解除する。
- 解除後に横離隔が再適用閾値未満へ縮むか、実行horizonが制限された場合は
  capを再適用する。
- 対象packageの単体テストとビルドが成功する。
- 実走効果はユーザーの`make dev2`で確認する。

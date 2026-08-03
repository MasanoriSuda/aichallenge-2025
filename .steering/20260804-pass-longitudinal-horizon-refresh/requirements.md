# Requirements

## 背景

`output/20260804-004137` では、前回の時計系修正後に prediction expiry は解消し、
同一側の Pass horizon 延長も 10 回成立した。一方で `Pass -> Return` は 0 回、
16 回すべてが `Recovery` に入り、そのうち 10 回は 1 回の横経路延長後に
`bounded Pass horizon exhausted` となっている。

現行実装は、横目標を変更する幾何延長回数
`mission_extension_count` と、実速度から rear-clear 距離を更新する縦方向の
再検証を同じ上限で止めている。このため、横経路が同じ側で安全なままでも
1 回延長後は SafeSeparation に落ち、追い越し速度を失う。

## 目的

- 横経路の変更回数と、同じ横経路上の縦方向 rear-clear 再予測を分離する。
- 同じ target、同じ side、同じ横目標を維持できる間は、現在実速度から
  rear-clear 必要距離を再計算して Pass を継続する。
- 縦方向の更新は 32 m / 10 s の既存絶対上限内に限定する。
- 実際の壁接触、車体重複、EmergencyBrake、静的経路不成立では従来どおり
  SafeSeparation / Recovery を許す。

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`
  - Pass horizon の縦方向更新アクションを追加する。
- `mpc_controller_cpp.cpp`
  - 同一横目標を固定した縦方向 horizon 更新を実装する。
  - 幾何延長回数と縦方向更新回数を分離して記録する。
- `test_v2x_overtake_core.cpp`
  - 幾何延長上限到達後も縦方向更新を選ぶ条件を単体テストする。

## 非対象

- `a_max: 1.0 m/s^2` の変更。
- 壁余裕、車体寸法、V2X topic 契約の変更。
- 左右 side の再選択や横目標の無制限な変更。

## 完了条件

- 幾何延長上限到達後、rear-clear 距離不足なら同じ横目標で縦方向更新する。
- 単なる fresh prediction の更新では SafeSeparation に落とさない。
- 絶対距離・時間上限または実際の短期安全不成立では更新を拒否する。
- 対象単体テストと `make autoware-build` が成功する。

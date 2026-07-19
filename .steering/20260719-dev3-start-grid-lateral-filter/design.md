# Design

## Root cause

`output/20260719-174512`では全車のMPCは同時刻にSTARTしたが、P1はP2を
`front_distance=0.99 m`, `path_lat=-2.66 m`の前方車として扱い、SafetyBrakeへ入った。
スタート地点はcurve lookahead内のため、通常の横衝突幅1.55 mへ
`v2x_front_decel_guard_curve_lateral_margin=1.5 m`が加わり、3.05 mまで拡大していた。
そのため隣グリッドのP2まで衝突対象になり、P1はP2が約3 m離れるまで発進できなかった。

## Approach

1. front lateral range計算を`start_grid_grace`のpure functionへ分離する。
2. Start grid graceのPrepared/Grace中はcurve guardの有無にかかわらず通常幅を返す。
3. grace外では従来どおり`min(corridor, danger + curve margin)`を返す。
4. invalid/non-finiteな幾何値は例外として既存controller fallbackへ倒す。
5. dev3ログから各車の最初の`ego > 0.05 m/s`時刻を比較する。

## Compatibility

- ROS topic/service/message契約は変更しない。
- `v2x_front_decel_guard_curve_lateral_margin`設定値は変更しない。
- Gate2のLowSpeedAvoidance latchには影響しない。
- 変更は参加者package内に閉じる。

## Verification result

- `make autoware-build`: 成功（25 packages）
- `test_start_grid_grace`: 14 tests全件成功
- `make dev3`: `output/20260719-175431`
  - P1 first motion: `1784451297.299307212` (`ego=0.20 m/s`)
  - P2 first motion: `1784451297.336316742` (`ego=0.30 m/s`)
  - P3 first motion: `1784451297.326814120` (`ego=0.32 m/s`)
  - 3台のfirst motion最大時刻差: 約`0.037 s`（受入基準`1.0 s`以内）

修正前の`output/20260719-174512`ではP1がP2/P3より約`3.01 s`遅れていたが、
修正後はP1が3台中最初に動き始めた。

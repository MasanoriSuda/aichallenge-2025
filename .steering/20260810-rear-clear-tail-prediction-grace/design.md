# Design

## 原因

rear-clear tailは予測sweep clearを成立条件にしている。一方、既存のprediction-only graceは、
現車体とcorridorがclearでforward progressが新鮮なら、予測だけの一時的不成立を0.25秒まで
許容する。tail側がこのgraceを所有権条件へ反映していないため、grace中でもlocal budget判定へ
戻ってAbortしていた。

## 方針

rear-clear tailを次の二つに分ける。

1. physical tail: current body、predicted sweep、corridorがclear
2. prediction-grace tail: current bodyとcorridorがclearで、既存prediction-only graceがactive

`PassShortHorizonGuardResolution`でgrace由来かを型付きで返す。どちらもSafeSeparationへは
同じrear-clear completion所有権として渡すが、grace由来をログで識別可能にする。

grace判定そのものは既存実装を再利用し、fresh progress、0.25秒上限、hard guardを変更しない。
grace終了後に予測sweepが復旧しなければ従来どおりAbortする。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: rear-clear tailとprediction graceの合成
- `mpc_controller_cpp.cpp`: current geometryに基づくtail候補の配線とsourceログ
- `test_v2x_overtake_core.cpp`: grace境界とhard faultの回帰テスト
- topic、message、launch、yaml: 変更なし

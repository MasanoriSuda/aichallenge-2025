# Requirements

## 目的

dev3の追い越しShiftOutで、遠距離の前走車には固定1.5 m/sより速く接近し、近距離の前走車には2.0 m/s固定値によるSafetyBrakeを避ける。

## 要件

- ShiftOut中の許容接近速度を1.5〜2.0 m/sで連続的に調整する。
- 前方距離、保護する最小前方距離、残りShiftOut距離から接近可能速度を求める。
- 適応上限が入口速度の下限によって無効化されないようにする。
- Pass、SafetyBrake、front risk、curve guard、wall clearance、hard capは既存動作を維持する。
- 設定で無効化でき、無効時は固定上限の既存動作を維持する。
- 単体テスト、ビルド、dev3走行で確認する。

## 比較基準

- 固定1.5 m/s: `output/20260719-194740`
- 固定2.0 m/s（不採用）: `output/20260719-200646`


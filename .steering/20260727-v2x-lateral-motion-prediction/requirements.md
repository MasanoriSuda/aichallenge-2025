# Requirements

## 目的

低速車が左右へ移動して空きを開閉する場面で、現在位置だけでなく横移動傾向を
V2X gap plannerへ反映し、閉じつつあるcorridorへのShiftOutを減らす。

## 背景

- `output/20260727-092323` のcourse-progress縦予測有効runは、solver failure、
  SafetyBrake、接触が増えた。
- `output/20260727-093318` の同予測無効runは安全性とlap timeが改善したが、
  低速車が開けた後に塞ぐ動きへの先読みはない。
- 現状の等速予測は世界座標の `vx/vy` を用いるため、カーブ上の進行方向変化と
  実際の横移動を分離できない。

## 要件

1. V2X車両の連続する2点をreference courseへ投影し、Frenet横偏差の差分から
   横速度を求める。
2. 横速度にdeadbandと上限を設け、位置ノイズや投影誤差をそのまま増幅しない。
3. 予測不能時は既存のCartesian等速予測へ戻す。
4. `v2x_prediction_use_course_progress: false` を維持し、縦方向A/Bとは分離する。
5. ShiftOut/Passの速度制限、Recovery条件は変更しない。
6. yamlの1項目で無効化できる。

## 制約

- 2025由来のシミュレータ向け暫定実験であり、2026公式仕様ではない。
- topic、message、service、評価結果schemaは変更しない。
- `aichallenge_system`は変更しない。


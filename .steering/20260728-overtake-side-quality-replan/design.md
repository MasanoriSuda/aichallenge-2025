# Design

## 1. 左右品質

左右候補を同じ評価式で比較する。

- 車両からのside clearance
- gap plannerのcorridor width
- 連続して通行可能なcorridor距離
- ShiftOut preflightの必要横加速度

内側・外側は実行許可の分類にのみ使い、内側を無条件の第一候補にしない。
品質差が小さい場合だけ、従来の幾何preferred sideをtie breakに使用する。

## 2. 選択側競合

ShiftOut中、locked targetが前方の設定距離内にあり、target相対横位置が
選択側へegoを越えている場合をselected-side conflictとする。
既存の2 m hard intrusion guardは維持し、より遠方の競合はside再計画の
入力として扱う。

## 3. 安定確認

反対側が品質優位、または選択側競合により反対側だけが成立する状態が
設定時間継続した場合だけ判断を確定する。候補sideが変わった場合は
安定時間をリセットする。

## 4. ShiftOut前半

横移動量と前進距離が設定上限内なら、現在位置を新しいShiftOut起点として
反対側のcorridor goalへ再計画する。phase時間・距離・横rampをリセットし、
古いgoalを引き継がない。

## 5. ShiftOut後半

前半window外、または安全な反対側候補がない場合はsideを直接反転しない。
旧sideをretry blockし、Recoveryへ移す。

## 6. 予測

過去A/Bで不安定だった生のcourse lateral velocityは有効化しない。
今回は現在のcourse-relative orderingを継続時間で平滑化する。

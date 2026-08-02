# Requirements

## 目的

追い越し候補の横運動、縦運動、deadlineを同じ車両速度rolloutで評価し、左右を含む候補から「実行中に抜き切れる」missionを選択する。

## 必須変更

- 現在自車速度、加減速上限、制御遅延、経路速度上限、candidate closing speedを使う時系列rolloutを追加する。
- body-clear時刻、hard-distance時刻、deadline slack、rollout中の最大必要横加速度を同じrolloutから算出する。
- 左右それぞれの最良candidateを同じ比較器へ渡し、deadlineをside preferenceより先に評価する。
- missionへdeadline checked / feasibleを分けて保存する。
- Recovery再取得時はgoalだけを差し替えず、選択済みmissionを一括反映する。

## 制約

- ROS 2 topic、message、launch、評価インターフェースは変更しない。
- 壁・footprint・Emergency・solverの既存hard guardは緩和しない。
- 加速度上限 `a_max: 1.0 m/s^2` は変更しない。
- start-grid専用corridor選択は今回のglobal candidate選択対象外とする。
- runtime deadline monitorは候補選択の動的効果確認後の別作業とする。

## 完了条件

- 高速自車から減速する候補と低速自車から加速する候補をrolloutで再現できる。
- deadline miss側よりdeadline feasible側が左右選択で優先される。
- deadline未評価missionはShiftOut ownershipを取得しない。
- Recovery再取得で旧closing/deadlineを流用しない。
- `make autoware-build` と追い越しコアテストが成功する。


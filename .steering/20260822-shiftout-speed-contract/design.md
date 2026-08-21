# 設計

## 原因

現行実装では、ShiftOut / Pass の前車 cap が未解除のときだけ
`target_velocity_reference` を生成していた。横分離が成立して cap が解除されると、
ShiftOut が継続中でも速度 reference が infinity に戻り、縦速度 authority が
レーシングラインへ移っていた。

前車 cap は「相手へ接近しすぎないための制約」であり、速度契約は
「採用した横軌道を壁・横加速度条件内で実行するための基準」である。
両者を同じ release 条件で消してはいけない。

## 変更方針

### 1. Mission に計画実行速度を保持

`OvertakeMissionCandidate` に `planned_execution_speed_mps` を追加し、
候補 rollout に用いた target speed + closing speed を保存する。
Mission freeze / rolling refresh でも同じ値を原子的に引き継ぐ。

### 2. ShiftOut speed contract を局所関数へ分離

純粋関数で以下を解決する。

- ShiftOut か
- frozen Mission か
- 有限な計画実行速度があるか
- 既存の追い越し速度 reference とどちらが低いか
- 現在速度が契約速度を何 m/s 超えているか

有効な場合、既存 reference と計画実行速度の小さい方を使う。
これは MPC の hard state bound ではなく速度 reference とし、急な QP 不成立を避ける。

### 3. authority を構造的に監視

ShiftOut + active line では longitudinal owner を追い越し系のまま保持する。
また、速度契約を期待しているのに active でない場合は
`shiftout-without-speed-contract` conflict を決定ログへ出す。

決定ログには次を追加する。

- speed contract expected / active
- contract reference
- actual overspeed

## 非対象

- 壁クリアランス値の変更
- Pass 完了後の最高速・closing speed 攻撃化
- Recovery / Reverse の変更
- MPCC solver や horizon の変更

# Design

## 原因整理

現行の`ExtendedSolverCircuitBreaker`は、失敗後0.75秒だけsolveを止める。cooldown終了後の
最初の1回が解けると、その周期からextended解を制御へ採用するため、境界条件が不安定な
ShiftOut / Passでは次の往復が起こり得る。

```text
extended failure
  -> cooldown中は3-state
  -> 最初のextended成功を即採用
  -> 再失敗
  -> 3-stateへ戻る
```

## 方針

`mpcc_progress`へ`ExtendedSolverReentryGate`を追加する。

- 通常時はextended成功を即採用する。
- failure / conversion rejectでrequalificationを要求する。
- cooldown終了後はextended QPを解くが、設定回数まで解はshadow probeとする。
- probe中は3-state MPCCが実制御を担当する。
- required success回連続で解けた周期からextended制御へatomicに戻す。
- probe中の失敗はstreakを0へ戻し、既存cooldownを再開する。

初期値は3周期とする。40 Hzでは約75 msであり、安定確認を行いつつ追い越し応答を大きく
遅らせない。velocity handoffは0.15秒から0.30秒へ延長し、再採用直後の縦指令差を緩和する。
現在周期のhard velocity boundは従来どおり即時適用する。

## テレメトリ

`Extended MPCC runtime`へ以下を追加する。

- `requalifying`: solve成功だが3-stateを継続した周期数
- `reentry_streak`: 現在の連続成功数 / 必要数

## 非目標

- MPCC参照経路の形状変更
- wall/target physical horizon判定の緩和
- OSQP weight、最大反復数の変更
- Recovery FSMの変更

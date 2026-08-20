# Design

## 方針

今回の変更は制御の意味を変えないobservability refactorとする。
既存のexecution authorityをMPC問題生成時の判断スナップショットとして内部保持し、
制御callbackでpost-processとRecovery arbitrationが終わった後に
`Overtake control decision`を生成する。
従来のpre-solver authorityログは重複出力せず、最終ログ一行へ内容を統合する。

```text
control callback decision ID
  -> MPC / behavior / corridor authority
  -> solver result or fallback
  -> acceleration and steering post-process
  -> stuck recovery arbitration
  -> final AckermannControlCommand publish
  -> change-aware final decision log
```

## decision ID

ROS制御callback開始時に単調増加IDを採番し、`MPC::get_control()`へ渡す。
branch workerの仮想問題は最終出力traceを更新しない。通常制御問題だけが同じIDを保持する。

## 最終制御源

優先順位を診断用の純粋関数で分類する。

1. failsafe
2. stuck recovery
3. control disabled
4. low-speed wall stop
5. solver crawl
6. solver fallback / forced stop
7. low-speed direct control
8. solved MPC

分類は既存の実行順序を記録するだけで、指令値を変更しない。

## Path source

最終lateral ownerに加え、RacingLine、GapPlanner、DynamicObstacleEscape、
FrozenMission、RecedingHorizon、RecedingDP、DynamicWaitPrefix、ContactEscape、
RecoveryLine、SafetyHoldを区別する。Mission系では生成時刻からpath ageも記録する。

## ログ抑制

decision IDはメッセージには含めるが、change-aware signatureには含めない。
owner/source/phase/fallback状態が変化したとき、競合時、または5秒heartbeatでのみ出力する。

## 既知の診断誤検出

DynamicObstacleEscapeはGapPlannerが生成したboundsを利用する同一の制御チェーンである。
両方がactiveでも独立した二つのlateral authorityではないため、競合から除外する。

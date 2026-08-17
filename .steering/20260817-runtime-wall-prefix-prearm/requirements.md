# Requirements

## Background

`6449803`後の実走`output/20260817-085008`と`output/20260817-085344`では、危険な
`holding=current-side`は0回になった。一方、wall-escape prefixは合計9回すべて不成立で、
採用は0回だった。

最新走行の6回の不成立理由は次のとおり。

- `ShiftOut/Pass path exceeds lateral acceleration limit`: 2回
- `no wall-feasible physical-clearance centerward goal`: 4回

警告は`ttc=0.35 s`で始まり、`fallback_delay=0.15 s`後の約0.17〜0.18秒経過時に、
設定された4.0 m ShiftOutと0.5〜1.0 m holdを要求している。これでは、警告後に実行可能な
中央寄りprefixを生成する時間・距離が残らない。

## Goal

- 実行中のMission経路に沿って壁を予測し、直進外挿より早く正確にwall preplanを開始する。
- fresh async Missionの完成待ちとは独立して、ローカルwall-escape prefixを最初の警告周期から
  評価する。
- 壁到達までの距離内で完了するprefix horizonを選び、固定5 mを無条件に要求しない。
- prefix不成立時の既存Mission handoffとhard wall guardは維持する。

## Constraints

- wall接触、hard margin、map unavailableを緩和しない。
- targetとのphysical separationと横加速度制限を緩和しない。
- ROS 2 topic/service、Domain、launch、result JSON schemaを変更しない。
- RecoveryおよびOSQP失敗対策は今回の主対象にしない。

## Definition of Done

- wall forecastはactive Frenet pathを優先し、利用不能時だけMission横目標を使う。
- 予測lookaheadをprefix全長を確保できる時間帯へ前倒しする。
- prefixのShiftOut距離は予測TTCと現在速度から上限を決める。
- prefix評価にasync候補待ちの固定遅延を要求しない。
- core unit test、package test、buildが成功する。


# Design

## 現象

最新走行では、現在車体・短期予測 footprint が非重複でも、数秒先の target-wall 制約が
不成立になると target-bound hold が開始される。ShiftOut では直前解を使わず現在横位置を
固定するため、0.35 秒の repair budget 内に新 Mission が得られないと FollowPrepare へ落ちる。

## 方針

1. 最新の solved MPCC trajectory、次に last physically validated trajectory を現在進捗へ整列する。
2. 現在の stage bounds と static-wall footprint で全点を再検証する。
3. locked target の continuity、現在 body separation、短期 predicted sweep が有効な場合だけ、
   整列済み軌道を target-bound execution prefix として採用する。
4. 採用時は ShiftOut の短い current-lateral freeze より長い独立 budget を使う。
5. 各 callback で再検証し、新しい feasible horizon が来たら既存 lifecycle の stable handoff で
   差し替える。
6. 直前解が stale / wall unsafe / prediction unsafe の場合は従来の current-lateral freeze を使う。

## 初期値

- solved trajectory lease: 0.75 s
- ShiftOut solved-prefix hold: 0.75 s / 4.0 m

40 Hz の open-loop 保持ではなく、各周期の再整列・壁再検証・相手予測guardを前提とする。

## 局所リファクタ

最新解と last physically validated 解の選択・壁検証が二重化しているため、controller 内の
単一 helper に集約する。tactical FSM や Recovery の責務は変更しない。

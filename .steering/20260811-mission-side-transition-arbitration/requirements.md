# Requirements

## Purpose

`3107313`の実走で発生した、`continuous outer`による`1 -> -1`の後に
`opponent side replan`が`-1 -> 1`へ戻し、壁余裕違反からRecoveryへ落ちる事象を解消する。

## Scope

- `continuous/scheduled outer`と`opponent side replan`でMission-wideの左右変更ラッチを共有する。
- 1つのMissionでコース全幅を横断する変更を1回に制限する。
- 反対側候補がさらに別のside transitionを必要とする場合は採用しない。
- candidate gate後、frozen/prepared Missionのmetricsでも完遂条件を再確認する。
- cross-side terminal closing speedと追従誤差用壁余裕を専用パラメータへ分離する。
- dynamic Mission waitでwall/emergency/solver等のhard faultをrear-clearより優先する。
- ego/targetのcourse-progress座標契約をコードコメントと左右対称テストで固定する。

## Constraints

- ROS 2 topic/service/message契約は変更しない。
- 評価基盤と`aichallenge_system`は変更しない。
- `aichallenge/result-summary.json`などの生成済み評価結果は変更しない。
- 同側のlongitudinal refreshとdynamic corridor refreshは従来どおり許可する。

## Definition of Done

- `continuous outer: 1 -> -1`後の`opponent replan: -1 -> 1`が拒否される。
- opponent replan後のscheduled/rolling outerによる2回目の再横断も拒否される。
- cross-side admissionがprepared Missionのtime/distance/speed/wall metricsでも成立する。
- dynamic wait中のhard faultが同一評価周期でRecoveryを選ぶ。
- 対象packageのbuild/testが成功する。

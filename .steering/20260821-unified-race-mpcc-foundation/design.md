# Unified Race MPCC foundation design

## Target architecture

```text
Gap Planner / Frenet DP / target tracker
              |
              v
      Race MPCC seed candidates
       Left Right Hold Return
              |
              v
   one geometry / constraint contract
              |
              v
      Race MPCC shadow decision

Current runtime authority remains unchanged in this steering.
```

## Stage geometry

`StageGeometry`は次を保持する。

- tracking waypoint index
- state waypoint index
- transition distance from the previous state
- cumulative distance from the current tracking state

stage 0は`tracking_wp -> tracking_wp + 1`に対応する。現在のように、dynamicsがこの区間を使う一方でexecution pathが`tracking_wp + 1 -> tracking_wp + 2`を使う状態を禁止する。

すべてのconsumerは同じimmutable vectorを利用し、独自にwaypoint間距離を再計算しない。

## Persistent branch solver contexts

workerはLeftとRightのside別contextを所有する。snapshotは問題入力だけを複製し、solver workspaceとwarm-startはcontextからbranch evaluatorへ移す。

context keyは最低限、次を含む。

- async context epoch
- target ID
- side
- horizon size
- progress/legacy mode

key変更時、progress discontinuity時、非有限解時だけresetする。Hold / Return用contextを同じ型で追加できる形にする。

## Target provenance

physical certificateをwall-only proofとして扱わず、生成時targetの次を付加する。

- source stamp
- receipt time
- course progress
- course lateral
- prediction generation

採用時はwall再検証に加え、現在targetとの縦横差と世代を検証する。値が得られない場合は新規Entryのみfail-closedとし、既存のEmergency / Recoveryには影響させない。

## Race MPCC shadow interface

`RaceMpccHomotopy`をLeft / Right / Hold / Returnで定義する。各候補は同じ結果schemaを使う。

- attempted / feasible
- objective
- terminal progress / velocity
- minimum wall and target reserve
- solver time / iterations / warm-start state
- reject reason
- source context and target provenance

本作業では既存Left / Right結果を変換し、Hold / Returnは適用条件と未評価理由を含むshadow recordとして出す。後続ステアリングで同一solver定式化へ接続する。

## Logging

通常周期ログは集約し、状態変化、候補選択変化、hard failure時だけ即時出力する。

```text
Race MPCC shadow: epoch=..., target=..., geometry=valid/stages/length,
L=attempted/feasible/warm/objective/reason,
R=...,
H=...,
Return=...,
selected=..., authority=shadow
```

certificate rejectではtarget provenance差、geometry generation、wall contractを一行で出す。

## Compatibility

ROS publisher/subscriber、message型、launch entry、Domain、評価結果schemaは変更しない。変更範囲は`multi_purpose_mpc_ros`内のpure core、controller内部、単体テスト、仕様文書に限定する。

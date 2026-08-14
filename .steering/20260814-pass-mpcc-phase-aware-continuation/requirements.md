# Requirements

## Goal

直近走行 `output/20260814-090647` で確認した、Pass中のsoft failureから
fresh same-side Missionを採用した際に、一律でShiftOutへ戻って再び経路を
失う事象を解消する。

## Scope

- `FollowPrepare`の起点がPassで、同じpass sideのhard-feasible候補を採用する
  場合は、現在状態からPassを継続する。
- 横補正、rear-clear予測、Return開始距離を残りMissionとして再構成する。
- ShiftOut起点またはcross-side候補は従来どおりShiftOutから実行する。
- Pass由来の同側継続も既存の壁、相手footprint、横加速度、時間・距離budgetの
  admissionを通す。

## Non-goals

- 通常走行、Recovery、ReverseをMPCCへ統合しない。
- no-return後のcross-side切替を緩和しない。
- 壁・車体のhard guardやSafetyBrake条件を緩和しない。
- ROS 2のtopic/service/interfaceを変更しない。

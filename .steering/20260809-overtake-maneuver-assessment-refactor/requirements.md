# Requirements

## Purpose

次段の動的maneuverランキング実装に先立ち、現行の左右Pass再評価を局所的に整理する。

## Scope

- Missionの有無、gap、実行許可、side conflict、runtime sweepからPass候補を評価する。
- 左右候補の物理余裕差と切替要求を共通の純粋関数で算出する。
- 連続安定時間のpending state更新を純粋関数へ分離する。
- コントローラは上記結果を既存`OpponentSideReplan`へ渡す。

## Non-goals

- 左右・Follow・Delayの新しいランキングは追加しない。
- 安定時間、no-return距離、切替回数などのパラメータは変更しない。
- FSM遷移、速度制御、横経路、ROS 2インターフェースは変更しない。


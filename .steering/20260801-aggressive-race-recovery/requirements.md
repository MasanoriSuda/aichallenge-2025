# Requirements

## Goal

競技用シミュレーションの復帰処理を、接触回避よりレース復帰を優先する方針へ変更する。

## Requirements

- 通常走行・通常追い越しの安全判定は変更しない。
- Stuck Recovery 中は、後方・周辺V2X車両を恒久停止理由にしない。
- 静的地図上の全候補が棄却されても、最も接触増加と経路悪化が小さい候補を選ぶ。
- 復帰候補・接触・gear由来の `SAFE_STOP` は終端状態にせず、短時間後に再評価する。
- 接触数、方向、姿勢が変化しなくても再試行を継続する。
- 前進・後退・Rejoin は既存の車両加速度上限を超えない。
- ROS topic、service、message、提出インターフェースを変更しない。
- 本変更は `simulation_only` かつ明示設定時だけ有効にする。
- 非finite入力、時刻逆行、odometry/control喪失は、移動を計算できないため既存hard stopを維持する。

## Out of scope

- 通常Overtakeの開始・左右選択ロジック
- 実車向け復帰制御
- AWSIM・評価基盤側の変更

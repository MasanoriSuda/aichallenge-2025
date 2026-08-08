# Requirements

## 目的

`output/20260808-093230` で、完全な追い越しMissionが複数回成立したにもかかわらず、
entry pre-armの0.3秒確認が候補側・closing speedの変化や短い候補欠落でリセットされ、
1周を通してOvertakeLineへ一度もhandoffできなかった事象を解消する。

## 対象事象

- P1は98.17秒で1周し、`Idle -> ShiftOut`は0回だった。
- `OvertakeLine entry retry blocked`は129回、`completion=0`は90周期だった。
- 同一target `d2`に対し、実測相対速度が0.71 m/sまで上がりながら、安定確認は
  0.21/0.30秒でリセットされた。
- 同一周期では完全なShiftOut/Pass/Return Missionが成立していても、別の粗い
  completion-distance判定がentryを棄却し得る。

## 制約

- 古い横経路を実行しない。ShiftOutへのhandoffには、その周期で再検証された完全Missionを必須とする。
- target変更、EmergencyBrake、明示的禁止waypoint、position jump、solver recoveryでは保持を即解除する。
- pre-armの保持対象は速度準備の履歴だけとし、左右・横goal・closing speedは最新Missionを使う。
- ROS 2 topic/service、評価schema、`a_max=1.0 m/s^2`は変更しない。
- 既存のPass prediction graceとactive Pass budgetは変更しない。

## 完了条件

- 同一targetでvalidated Missionの左右またはclosing speedが変わっても、速度確認時間をリセットしない。
- 短い候補欠落中はbounded validation leaseで速度確認履歴を保持するが、ShiftOutは開始しない。
- lease中もhard guardはfail-closedである。
- 最新の完全Missionが成立していれば、粗い次hard-curve距離だけで二重棄却しない。
- core unit testと対象package buildが成功する。

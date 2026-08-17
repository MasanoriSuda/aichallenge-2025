# Requirements

## 目的

通常のレーシングライン走行を基準に、全V2X車両をGapPlannerの時系列動的障害物として扱う方向へ移行する。第1段階では、停止・極低速車だけが通常の左右Mission評価より先に専用`LowSpeedAvoidance`へ分岐する所有権逆転を解消する。

## 要求

- 確認済みの停止・低速V2X車両を通常のfront tactical targetへ昇格する。
- GapPlannerが保持する全V2X footprintを用い、通常の左右MPCC-lite Mission生成を先に実行する。
- 新規遭遇ではlegacy low-speed local bypassを先に開始しない。
- 既に実行中のlegacy `LowSpeedAvoidance`は途中で所有権を切り替えない。
- 実行可能な通常Missionがない場合は、既存Follow・SafetyBrake・Recoveryのhard guardを維持する。
- ROS topic、service、launch、提出物の契約を変更しない。

## 対象外

- FSM全廃または完全な単一MPCCへの置換。
- 接触、壁ピン、Reverse Recoveryの変更。
- V2X予測モデル、車体寸法、クリアランス値の調整。

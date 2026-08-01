# Requirements

## Purpose

低速車追い越しで、直線上の十分に開いた次カーブ内側を選べず、また有効な追い越し経路へ進入後も前後距離だけで SafetyBrake へ落ちて失速する事象を改善する。

## Requirements

- 追い越し側の基本選択は、現行どおりレーシングライン維持と最小横移動を優先する。
- 直線上でも設定距離内の次カーブから内側を解決できること。
- 内側の連続開放距離と corridor 幅が設定値以上の場合だけ、許容横移動差の範囲で内側を優先すること。
- ShiftOut / Pass 中の locked target に限り、validated corridor と現在・予測の2D車体分離が成立するときは、前後距離だけを根拠とする SafetyBrake を抑制すること。
- 別車両、現在車体重複、予測sweep重複、予測不能、target jump、未検証corridor、壁・実行経路異常は従来どおり保護すること。
- `a_max: 1.0` は変更しないこと。
- ROS 2 topic / service / message 契約を変更しないこと。

## Out of scope

- Recovery、Reverse、Start Grid の方針変更
- 車体寸法、壁余裕、MPC加速度上限の変更
- 実車向け設定


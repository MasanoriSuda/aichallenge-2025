# Design

## 1. Longitudinal safety envelope

moving/stopped frontに対する停止距離、速度headway、固定floorを一つのpure functionで計算する。BehaviorのSafetyBrake判定、Overtake entry reserve、ShiftOut/Passの横分離前closing reserveは同じ結果を参照する。

これにより、追い越し速度参照だけがSafetyBrake境界の内側へ接近し続ける矛盾を禁止する。既に横分離済みの場合は従来どおりfront capを解除できる。

## 2. Extended branch admission

左右branchを次に分類する。

- robust: 設定された追加境界余裕を満たす。
- physical-boundary: QP hard bound内、かつ連続physical wall検証済みだが追加境界余裕だけ不足。
- rejected: side、attempt、solver feasibility、数値、wall validationのいずれかが不成立。

robust branchを常に優先する。robustが一つもない場合だけphysical-boundaryを採用可能にする。壁検証未実施・失敗branchはfallback対象にしない。

## 3. SafeSeparation budget

absolute budget到達後も、既にforward escapeが開始済みでlocal windowが残っている場合に限り、そのlocal windowを継続する。local windowの再延長は禁止する。forward escapeが未成立なら従来どおりabsolute limitで即Abortする。

## 4. Decision trace

最終制御ログへ以下を同一decision IDで追加する。

- front distance
- dynamic safety distance
- unseparated protected distance
- closing speed reference
- left/right MPCC branch eligibility
- solver crawl block reason

ログ量は既存change-aware emitterの周期・状態変化集約を維持する。

## Safety impact

- target/wall hard constraintは変更しない。
- physical-boundary fallbackは連続physical wall検証済みbranchに限定する。
- emergency/SafetyBrakeの優先権は維持する。
- solver fallbackの挙動は変更せず、理由の分類だけを追加する。

# Design

## 1. 実行フェーズの定義

front-cap解除helperの`Pass`限定条件を、committedな`ShiftOut / Pass`実行中へ広げる。
新規候補選択前やFollow中には適用しない。

## 2. 現在横離隔

locked targetの現在横離隔を、PassだけでなくShiftOutでも同じreference course frameで
計算する。解除は履歴だけではなく、現在値が
`max(v2x_overtake_pass_front_overlap_lateral_clearance,
v2x_vehicle_radius + v2x_prediction_margin)`以上であることを要求する。

## 3. ヒステリシス

解除成立をOvertakeLine stateへ保持し、現在横離隔が
`v2x_overtake_pass_front_cap_reapply_lateral_clearance`以上の間だけ維持する。
再適用閾値未満へ縮んだ場合は前車由来capへ戻す。

Pass継続用のfront-overlap exclusion latchは従来どおりPassでのみ確定し、
ShiftOut中の一時的な横離隔だけでcommitted Pass continuityを成立させない。

## 4. 二重所有の整合

Behavior FSMのstage speedとOvertakeLine側の補助速度参照の両方で、
同じ実行フェーズ・現在横離隔・ヒステリシスを使用する。

## 5. 安全性

解除するのはlocked target由来のfront-speed capだけである。別の前方車、Emergency、
wall/corridor、curve、MPC加速度、solver/odometryの各制約は維持する。

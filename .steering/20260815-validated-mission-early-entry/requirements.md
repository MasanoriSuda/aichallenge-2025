# Requirements

## 目的

- 実行可能な追い越しMissionがあるのに、実相対速度の確認待ちでShiftOutが4〜6 mまで遅れる現象を解消する。
- Missionが検証済みの場合、計画したclosing speedを初動から使い、追い出しが0.8 m/sへ不整合に縮む現象を解消する。

## 実走根拠

- `output/20260815-223825/d1/autoware.log`
- pre-arm timeout: 5回
- `Idle -> ShiftOut`: 14回中11回が前方距離6.21 m以下
- 初回ShiftOut closing: 15回中9回が0.8 m/s、Mission closingは2.0 m/s
- `Pass -> Return`: 1回

## 制約

- 相対速度ゲートを全面撤廃しない。
- 現在周期で実行可能なMission、body-clear、entry front reserve、hard guardを満たす場合だけ早期実行する。
- 緊急前方リスク、solver recovery、位置飛び、禁止区間は従来どおり拒否する。
- ROS 2 topic/service、評価インターフェースは変更しない。

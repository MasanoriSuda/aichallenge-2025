# Requirements

## 目的

dev3のヘアピン主体コースで、カーブを一律に追い越し中断条件として扱わず、
通過可能な外側回廊がある場合は前走車の外側へ回り込む追い越しを成立させる。

## 要件

- 曲率符号からカーブ内側と外側を判定し、外側だけを新規カーブ追い越し候補にする。
- soft curve区間では、連続した外側gapが成立すれば新規ShiftOutを許可する。
- 外側ShiftOut/Passが開始済みでgapとlocked targetが継続している場合は、hard curveでも中断しない。
- hard curve内での新規追い越し開始は許可しない。
- イン差し禁止、明示WP禁止、EmergencyBrake、curve cooldownは緩和しない。
- 外側gapが失われた場合は既存Recoveryへ戻す。
- 横クリア後は既存どおりPassを加速し、対象がrear-clearになるまでReturnしない。
- ROS 2 topic、service、message、Domain、評価インターフェースは変更しない。
- 機能はconfigで無効化でき、既定値は従来互換のfalseとする。

## 完了条件

- pure coreで外側soft-curve進入、内側拒否、hard-curve新規進入拒否、外側継続、安全guardを単体テストする。
- dev3設定で外まくり機能を有効にする。
- `make autoware-build`が成功する。
- dev3実走はユーザー側で行う。

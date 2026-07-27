# Design

## 1. 新規開始とcommit後の責務分離

Behavior FSMのgap、curve、completion判定は新規Overtake開始時に使用する。
OvertakeLineがShiftOutへcommitした後は、固定したtarget、side、corridorを
実行時hard guardが否定しない限り保持する。

## 2. commit後の継続条件

BehaviorがOvertake以外へ一時的に切り替わっても、次をすべて満たす場合は
現在のShiftOut / Passを保持する。

1. locked targetのcourse progressが連続している
2. targetがまだ前方にいる
3. targetが選択済みpass側へ侵入していない
4. live execution corridorがblockされていない
5. 明示禁止waypointではない
6. front riskがEmergencyではない

soft/hard curveの新規入口判定、completion距離、overtake cooldown、
soft forbiddenはcommit後のキャンセル条件に使用しない。

## 3. hard guard

位置ジャンプとsolver recoveryは既存continuity resolverで即Recoveryとする。
actual wall、horizon wall/static wall/横加速度不成立、target期限切れ、
live corridor不成立も従来どおりRecoveryとする。

## 4. Recovery再取得

`Recovery -> ShiftOut`は残す。今回の修正により、新規候補判定の1周期dropでは
Recovery自体へ入らなくなるため、同じ原因による再取得チャタリングを除去する。

# Design

## 1. Horizon評価の共通化

`update_overtake_line()`内で行っている次の処理を、controller内の副作用を持たない
評価関数へ切り出す。

- path boundと`min_wall_clearance`による横目標clamp
- 距離/速度から求める必要横加速度と上限制限
- static wall gridに対する実車体footprintのclamp
- 追加壁余裕が収まらない場合のphysical footprint再評価
- static clamp後の必要横加速度再評価

評価結果はtarget horizonと、物理不成立・壁余裕劣化・横加速度不成立の理由を返す。
実行時と新規ShiftOut入口の両方が同じ関数を使う。

## 2. ShiftOut入口preflight

通常の追い越し候補ごとに、候補corridor centerを最終横目標として
ShiftOut horizonを事前評価する。実行時と同じ車体footprint・static map・
`min_wall_clearance`・`max_lateral_accel`を使用する。

不成立ならその側の`gap_available`をfalseにし、反対側が成立する場合は既存の
両側選択へ委ねる。start-grid専用corridorは既存の車両inflation・操舵曲率検証を
維持し、今回の通常入口preflightを重ねない。

## 3. 幾何失敗側の再試行抑止

入口preflightまたは実行中のstatic wall系失敗を、対象ID・pass sideごとに短時間
記録する。期限内は同じ対象の同じ側を新規候補またはRecovery reacquireとして
採用しない。反対側の探索と通常Followは妨げない。

既定の抑止時間は1.0秒とし、param yamlから調整可能にする。

## 4. 安全性

actual footprintの物理接触とmargin違反、Emergency、solver failureの既存遷移は
維持する。入口preflightは安全guardを無効化するものではなく、同じ失敗を
ShiftOut開始前へ前倒しする。

## 5. ログ

入口preflightの棄却と再試行抑止の開始を状態変化時だけ出す。周期デバッグログは
増やさない。

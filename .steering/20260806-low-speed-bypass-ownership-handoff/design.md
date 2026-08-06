# Design

## 所有権規則

横経路の優先順位を次のようにする。

1. 同一targetに対する凍結済み `OvertakeLine ShiftOut/Pass`
2. 既に開始済みの停止車用direct control
3. 新規 `LowSpeedAvoidance` 候補
4. 新規通常Overtake候補

成立済み `OvertakeLine` は全体経路preflight後のmissionであるため、単に対象速度が0.2 m/s以下へ変わっただけでは別plannerへ移管しない。EmergencyBrake、壁接触、target discontinuity、solver failureなど既存のhard guardは維持する。

## 通常追い越しの保持

`LowSpeedBypassCandidateRequest` に `committed_overtake_execution_active` を追加する。同一targetの凍結済み `ShiftOut` / `Pass` の場合、停止車用候補を開始しない。

Behaviorが`LowSpeedAvoidance`へ変わらないため、`update_overtake_line()`内のstopped-bypass handoffも発生せず、既存missionとopponent-driven side replanが継続する。

## 低速direct controlの反対側再計画

新規低速回避として開始したmissionでは、locked sideを第一候補として毎周期評価する。第一候補がV2X corridorまたはstatic wall preflightで不成立となった場合、`auto`設定なら反対側を現在poseから評価する。

反対側採用時は次を原子的に更新する。

- `low_speed_locked_side_sign`
- pass/rejoin target lateral
- direct control side
- direct control phaseを`Shift`へ戻す

反対側も不成立なら従来のlive safety guardを維持し、速度0とする。

## ログ

反対側が実際に採用されたときだけ、旧side、新side、target lateralをINFOで出す。両側不成立の毎周期ログは追加しない。既存のguard立上がりログを維持する。

## 影響範囲

- `multi_purpose_mpc_ros`内部のみ。
- 設定値と外部ROS契約に変更なし。
- 低速direct control未使用時のMPC制御に変更なし。
- stuck recoveryは変更せず、誤った入口となったplanner競合を上流で除去する。


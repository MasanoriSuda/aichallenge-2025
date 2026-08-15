# Requirements

## 背景

`output/20260815-194555` では runtime completion tactical replan が15回起動し、
4回は現在側の hard-feasible progressive prefix を生成できた。しかし全ケースが
`prefix=1/0/safe-separation, authority=none` となり、atomic Mission replacement は
一度も実行されなかった。

SafeSeparation は通常の未完成prefixを拒否すべきだが、今回のprefixは
runtime rear-clear予算不足を検出した後、現在状態から再生成・物理再検証された
同側候補である。この限定ケースまで拒否すると、古いMissionを保持したまま
壁・距離上限へ到達する。

## 要求

- SafeSeparationの通常fail-closed契約は維持する。
- runtime completion replan pending中に限り、厳格なtactical re-arm条件を満たす
  同側progressive prefixを実行候補として認める。
- 反対側progressive prefixはSafeSeparation中に認めない。
- target discontinuity、予測sweep重複、壁・EmergencyBrake・solver hard fault、
  side-by-side/no-return後は従来どおり拒否する。
- 採用は既存のtransactional Mission replacement経由とし、成功時のみ
  SafeSeparation budgetとruntime pendingをリセットする。
- ROS topic/service、設定値、評価インターフェースは変更しない。

## 制約

- `aichallenge/result-summary.json` のユーザー変更は変更・コミットしない。
- Recoveryやcross-side安全条件を緩和しない。
- boolean aggregate initializerの追加事故を避けるため、対象prefix request生成は
  明示的なfield assignmentへ局所リファクタする。

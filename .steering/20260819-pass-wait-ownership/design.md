# Design

## 観測した失敗経路

```text
Pass (forward escape latched, rear-clear未成立)
  -> target/wall bounds conflict
  -> DynamicMissionWait
  -> forward prefixなし
  -> short wait timeout
  -> Idle
  -> SafetyBrake / wall contact / Stuck Recovery
```

DynamicMissionWaitへ入る際、`dynamic_mission_wait_pass_forward_completion_latched`
にはPassのcommit状態が保存されている。しかし期限延長判定はforward prefixと
full-closing/continuous-DP authorityだけを見ており、このsnapshotを使用していない。

## 修正方針

`DynamicMissionWaitRetentionRequest`へ
`pass_forward_completion_latched`を追加する。

保持を許可する条件は次のいずれかとする。

1. 従来どおり、wall-validated forward prefixが有効で、full-closingまたは
   continuous-DP authorityを持つ。
2. Pass-originかつforward-completionが既にlatchされ、target progressが新しく、
   runtime hard faultがない。

2は軌道を強制実行する条件ではなく、Mission/target/sideの所有権を保持して再計画と
rear-clear判定を継続する条件である。実行可能prefixがなければ従来どおり
FollowPrepare側の安全出力を使う。

## 安全境界

- pre-commit ShiftOutはlatchを持たないため対象外。
- actual wall contact、wall margin block、EmergencyBrake、solver Recovery、
  forbidden waypointでは保持しない。
- target jump、course progress discontinuity、staleは既存terminal処理へ渡す。
- Mission総時間予算は変更しない。

## 効果確認

- `forward_latched=1`のPass-origin waitが短いreselect timeoutでIdleへ落ちない。
- rear-clear成立後はReturnへ進む。
- hard faultでは保持しない。

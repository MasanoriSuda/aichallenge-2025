# 設計

## 現象

最新runでは`ShiftOut -> Pass`後、`body_separated=1`でも、Pass horizon更新時の
`static full-path preflight`が将来のReturn区間で壁・横加速度制約に失敗し、
SafeSeparationのRecoverBehindへ移行していた。

## 変更

`PassContinuationPreflightPolicyResolution`に`include_return_path`を追加する。
このpolicyは既にadmit済みのactive Pass専用なので、値は`false`とする。

controllerの抽出済みpreflightメソッドを
`evaluate_committed_pass_continuation_preflight`へ改名し、policyのscopeを
`evaluate_overtake_line_entry_preflight`へ渡す。

これにより、次を独立させる。

1. Initial admission
   - ShiftOut + Pass + Returnを評価（現行どおり`include_return_path=true`）。
2. Active Pass continuation
   - 現在位置からsame-side rear-clearまでを評価（`include_return_path=false`）。
3. Deferred Return
   - rear-clear成立後、Return phaseの実行horizonを現在位置から生成する。

## 維持する保護

- Pass中の実行horizonは毎周期、壁と横加速度を確認する。
- actual footprint wall contactは即Recovery/停止対象のままとする。
- target footprint、prediction、position jump、intrusion、EmergencyBrake、solver guardを維持する。
- Pass horizon absolute time/distanceとSafeSeparation自体は削除しない。

## ログ変更

active Passの失敗prefixを`static Pass-continuation preflight`へ変更する。
初回admissionのfull missionログは変更しない。

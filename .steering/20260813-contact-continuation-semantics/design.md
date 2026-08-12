# Design

## 状態の分離

```text
NearContactPrearm
  - near envelopeを0.10秒確認
  - target、side、接触前ego速度を短時間保持
  - 安全制約は緩和しない

RecoverableContactAdmission
  - 確認済みactual overlap
    OR
  - Prearm後0.20秒以内にego速度が1.0 m/s以上急落
  - 相対ヨー20度以内
  - 前周期の壁余裕0.25 m以上

ContactContinuation
  - 最大0.8秒
  - 証拠欠落0.20秒まで保持
  - 相対横速度は開始0.5 m/s、保持0.8 m/s
  - 初期0.25秒後は前進進捗を要求
  - 相手から離れる0.15 m lateral biasを壁区間内へclamp
```

## 姿勢推定

V2X messageにはtarget yawがない。target速度が0.5 m/s以上で観測有効な場合、`atan2(vy, vx)`をtarget headingとし、egoの現在yawとの差を正規化する。低速または観測不能時はContactContinuationを開始しない。

## 壁guard

OvertakeLineのactual footprint wall monitorで、ContactContinuation専用の0.25 m拡張footprintを評価する。その結果を次周期のbehavior評価へ渡す。初期値と不明時はfalseとし、実壁接触は既存hard abortを維持する。

## 証拠欠落保持

開始済みContactContinuationに限り、actual overlap/impact証拠の最終観測から0.20秒以内を保持する。target不連続、相対ヨー、壁、closing、低速、時間、進捗のguardは毎周期再評価する。

## 動的確認

- `evidence=near`だけのContactContinuation開始が0回
- `impact`または`overlap`で開始すること
- `evidence_hold=1`で短い境界揺れを越えること
- ContactContinuationとcrashペナルティの時刻
- `Pass -> Return -> Idle`完遂率
- wall / crash / SafeSeparation回数


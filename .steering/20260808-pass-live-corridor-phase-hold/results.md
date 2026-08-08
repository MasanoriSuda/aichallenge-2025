# Results

## 実装結果

`ShiftOut`のlive execution corridor dropoutが消費したhold時間を、物理的に車体非重複な
`Pass`へ持ち越さないようにした。Pass中の基準時刻は、Pass開始時刻とその後に確認した
live corridor最終成立時刻の新しい方である。同一Pass内では開始時刻が固定されるため、
hold自身による期限延長は発生しない。

debug logの`pass_phase_ref=1`は、Pass開始時刻をhold基準に採用したことを示す。

## 静的検証

- build: 25 packages successful
- package tests: 25/25 successful
- aggregate test result: 923 tests, 0 failures

## 次回走行の確認点

- `ShiftOut -> Pass`後のcorridor chatterで`pass_phase_ref=1`が記録されること
- 旧runの約1.45秒地点で`Pass -> Recovery: live overtake corridor unavailable`へ
  落ちないこと
- `Pass -> Return -> Idle`完遂数と98秒級外れ周の減少
- actual wall、target overlap、EmergencyBrake、solver異常による保護が従来どおり残ること
- corridorがPass開始後2秒を超えて復帰しない場合は、従来どおりRecoveryへ移ること


# Design

## 1. Attempt-scoped Dynamic Escape exit contract

`DynamicEscapeExitGate` に attempt ID を渡し、replacement 採用を契約終了ではなく同一契約内の `ReplacementExecuting` として扱う。

- target が blocking の間は gate を閉じない。
- replacement が一時的に消えた場合は `ReplacementLost` とし、同じ契約内で一度だけ再計画する。
- target resolved + wall admitted の連続確認、または Recovery override だけを通常の release とする。
- attempt ID が本当に変わった場合だけ契約を再初期化する。

## 2. Publish-aware ShiftOut rollback

Mission の走行距離ではなく、その generation の追い越し指令を実際に publish 済みかを記録する。

- 未 publish の ShiftOut が壁判定で落ちた場合: `EntryRollback -> Idle`。
- publish 済みの ShiftOut/Pass: lateral authority を失わず `DynamicReplan`。
- final postprocessed prediction の壁判定にも同じ判定を適用する。

## 3. Explicit boundary tolerance

active overtake の最終 physical footprint 距離に限り 0.01 m の境界許容を与える。

- contact/out-of-map は常に拒否。
- `physical_min + tolerance >= required` のときだけ受理。
- ログへ raw clearance、required、tolerance、shortfall、boundary acceptance を出す。

0.397 m / 0.400 m は受理対象だが、0.256 m や 0.114 m は従来どおり拒否する。

## 4. Traceability

- Dynamic Escape: request/latched attempt、replacement active/admitted、replacement execution状態。
- Wall admission: tolerance、raw/effective shortfall、boundary acceptance。
- Wall replan: generation が publish 済みか、rollback/replan の選択根拠。

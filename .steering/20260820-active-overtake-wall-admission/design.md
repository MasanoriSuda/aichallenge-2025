# Design

## 1. 共通 wall path admission

既存の solver recovery 専用 gate を、明示的な activation request と observation freshness を受け取る汎用 gateへ局所リファクタする。同じ実装を次の2系統で独立運用する。

- `solver-handoff`: bounded continuation から通常解へ戻る境界
- `active-overtake`: ShiftOut / Pass / Return の通常 MPC 解

active-overtake は 10 Hz で `current_prediction` を物理 footprint scanする。接触、out-of-map、無効予測、required clearance未満を検出した場合にgateへ入り、前周期の操舵と制御減速を維持する。保留中は新しい安全観測が2回連続するまで解除しない。

## 2. DP tracking release confirmation

DP authorityのhard fault条件は変更しない。tracking errorのみ、0.10秒連続して不成立になった場合にreleaseする。単発のtracking境界越えは既存の検証済みprefixを保持する。

## 3. 決定ログ

- wall admissionログに `scope`、phase、path sourceを追加する。
- DP authorityログに `reason`、raw/effective tracking、tracking-loss elapsedを追加する。
- final control sourceへ `overtake-wall-admission-hold` を追加し、solver handoff保留と区別する。

## 非目標

- MPCC cost、左右戦術、壁clearance設定値の調整
- Recoveryロジックの変更
- 全車両予測モデルの変更

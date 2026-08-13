# Tasklist

- [x] `20260813-121813`のshadow不一致を集計する
- [x] Mission→shadow候補変換を純粋関数へ分離する
- [x] shadow-only SideAssessmentを実装する
- [x] left/rightを同一shadow周期で評価する
- [x] 実行中Returnの誤棄却を修正する
- [x] Mission/SafeSeparation残予算を候補判定へ反映する
- [x] 棄却理由と残予算ログを追加する
- [x] 単体テストを追加・更新する
- [x] `make autoware-build`と対象テストを実行する
- [x] 非介入境界とユーザー変更を含まない差分を確認する

## Definition of Done

- shadow周期ではleft/right双方がplanner評価済みとしてログへ現れる
- 実行中Returnがlive corridor成立時にfeasibleとなる
- 予算不足がgeneric hard constraintではなく専用理由になる
- shadow評価がside/FSM/速度/操舵を変更しない
- buildと既存・追加単体テストが成功する

## Verification

- `make autoware-build`: 25 packages成功
- `test_v2x_overtake_core`: 全540テスト成功
- `git diff --check`: 成功
- `authority=none`を維持し、制御出力への接続は次段階へ持ち越し

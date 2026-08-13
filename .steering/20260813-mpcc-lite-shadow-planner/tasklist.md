# Tasklist

- [x] 現行の左右Mission candidateとrollout構造を確認する
- [x] Phase 1の非介入境界を定義する
- [x] typed shadow evaluatorを実装する
- [x] 左右・現在側・Return candidateをcontrollerで構成する
- [x] 8 Hz評価、1 Hz/変化時ログ、last-feasible診断を実装する
- [x] config/config_for_cloudへshadow-only設定を追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] ユーザー変更を含めず差分を確認する

## Definition of Done

- feature有効時も既存のside/FSM/速度/操舵出力が変化しない
- 4戦術を共通指標で比較できる
- infeasible候補は推奨されない
- last-feasibleはログ上だけで保持される
- 走行ログからFSMとshadow推奨の一致率を集計できる

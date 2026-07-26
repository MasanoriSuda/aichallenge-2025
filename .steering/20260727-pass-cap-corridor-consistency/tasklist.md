# Task List

- [x] 最新ログと現行ロジックを照合する
- [x] Pass 速度 cap ヒステリシスを実装する
- [x] corridor goal と最低横離隔の共通範囲を実装する
- [x] 対象車後方時の Return 優先を実装する
- [x] 状態変化ログを追加する
- [x] 単体テストを追加して実行する
- [x] 対象パッケージをビルドする
- [x] 試走確認項目をまとめる

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 178 tests、失敗なし
- `multi_purpose_mpc_ros` 全テスト: 665 tests、失敗なし

## 試走確認

- `OvertakeLine pass front cap: Released` 後、1.30〜1.50 mの微小揺れで `Reapplied` にならないこと
- 1.30 m未満では `Reapplied` となり、再解除には1.50 mが必要なこと
- 抜き終わり付近の壁余裕事象が `Pass -> Return` となり、不要なRecovery速度制限へ入らないこと
- 物理接触時は従来どおりRecoveryへ入ること

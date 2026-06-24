# Codex 向けリポジトリ指示

- 変更前にこのファイルを読むこと
- 作業は PR 単位の小さな変更に分けること
- 可能な限り `aichallenge_submit/` 配下で完結させること
- 明示的な指示がない限り evaluator は変更しないこと
- feature flag で既存ベースライン挙動を維持すること
- 実装前に既存 launch、topic、message 型の契約を確認すること
- 変更後は少なくとも以下を実行すること
  - `colcon build`
  - 関連する unit test
- 最後に以下を報告すること
  - 変更ファイル一覧
  - 実行コマンド
  - 残タスク
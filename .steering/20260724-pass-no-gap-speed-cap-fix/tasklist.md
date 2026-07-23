# Pass中 no-gap 速度制限修正 タスクリスト

## 設計

- [x] 急失速のコード経路とログ時系列を特定する
- [x] 変更範囲、安全境界、Definition of Doneを記録する

## 実装

- [x] no-gap速度制限の適用条件を純粋関数化する
- [x] committed Pass corridor bypass中だけ適用を抑止する
- [x] 状態別単体テストを追加する

## 検証

- [x] 対象パッケージの全testを実行する（600 tests、失敗0）
- [x] `make autoware-build`相当のDocker buildを実行する（25 packages成功）
- [x] dev2でcorridor bypass後の速度推移を確認する
- [x] `git diff --check` とinterface差分を確認する

## 完了条件

- [x] 隠れた2.0 m/s capによる急失速が再現しない
- [x] 通常Follow・未分離Pass・Emergency系の既存条件を維持する
- [x] 検証結果と残課題を記録する

# Tasklist

- [x] 最新ログとattack/danger/contact経路を照合する
- [x] 要件、非対象、完了条件を固定する
- [x] pre-contact squeeze分類をcoreへ追加する
- [x] wall-bounded lateral escapeをPass実行へ接続する
- [x] front capとdanger suppressionの競合を解消する
- [x] config、起動ログ、イベント/debugログを更新する
- [x] core単体テストを追加・更新する
- [x] package build/testを実行する
- [x] `git diff --check`と契約差分を確認する
- [x] ユーザー変更を除外してコミットする

## 動的確認

- confirmed predicted overlap時に`squeeze response entered`が1回だけ出ること
- `attack_hold=0`、`danger_suppress=0`、front cap Reappliedになること
- applied biasがrequested bias以下かつ壁境界内であること
- biasがwall-limitedの場合にspeed-cap fallbackが記録されること
- ContactContinuation、Pass -> Return -> Idle、壁接触回数を併記すること

## Verification

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 29/29 test sets pass
- `colcon test-result --verbose`: 1393 tests、0 errors、0 failures
  - 既存`build/joycon_contract_guard/package.xml`欠損の集約警告は今回の対象外
- `git diff --check`: 問題なし
- `pre-commit`: ホストに実行ファイルがなく未実行
- 動的効果確認: 次回`make dev2`試走で実施

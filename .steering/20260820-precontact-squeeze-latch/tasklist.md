# Tasklist

- [x] 最新走行のrecurrenceログを照合する
- [x] 自己終了する条件を特定する
- [x] 取得条件と継続条件を分離する
- [x] 継続reasonと回帰テストを追加する
- [x] package build/testを実行する
- [x] ユーザ変更を除外してコミットする

## 動的確認

- `entered`後に`front-cap-not-released`で即終了しないこと
- 周期debugが`active-held-after-front-cap-reapply`を示すこと
- active中は`pass_owner=1`を維持すること
- `bias=0.15/0.15`またはwall-boundedな適用値を維持すること
- predicted sweep clearまたはContactContinuationへの引き渡しで終了すること

## Verification

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: success
- `colcon test-result --verbose`: 1393 tests、0 errors、0 failures
  - 既存`build/joycon_contract_guard/package.xml`欠損の集約警告は今回の対象外
- 動的効果確認: 次回`make dev2`試走で実施

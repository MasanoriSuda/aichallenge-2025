# Tasklist

- [x] `20260815-194555` の未接続点を特定
- [x] SafeSeparation tactical re-arm付きprefix admissionを追加
- [x] shadow same-side authorityへre-armを接続
- [x] transactional replacement commitへre-armを接続
- [x] prefix request生成を局所リファクタ
- [x] unit testを追加・実行
- [x] package build/testを実行
- [x] ユーザー変更を除外してコミット

## Definition of Done

- 通常のSafeSeparation prefixは拒否される。
- 厳格にre-armされたPass中の同側prefixだけがadmitされる。
- no-return、非Pass、hard constraint違反はre-arm時も拒否される。
- runtimeログで次回 `prefix=1/1/admitted, authority=replace` を判定できる。

## Validation

- `make autoware-build`: 成功（25 packages）
- `ctest --test-dir .../multi_purpose_mpc_ros --output-on-failure`: 25/25成功
- 追加したruntime re-arm / prefix authorityテスト: 2/2成功
- `git diff --check`: 成功

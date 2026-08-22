# Tasklist

- [x] 最新試走と直前試走を比較する
- [x] attempt lifecycle、authority、wall admission、solver fallback を時系列照合する
- [x] 関連コードと git blame から導入経緯を確認する
- [x] 根本原因と反証条件を整理する
- [x] execution lease resolver と単体テストを追加する
- [x] fresh/effective execution 状態を分離する
- [x] retained control/prediction を壁判定前に復元する
- [x] wall replan を prediction ownership で制限する
- [x] decision/wall trace に ownership を追加する
- [x] build と関連テストを実行する
- [x] 不要になった exit/handoff 分岐を確認する
- [x] コミットする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 targets 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1496 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功

## 分岐整理

- wall/exit hold 中だけ遅れて retained prediction を復元していた処理を、実行権解決直後へ移動した。
- 後段の復元処理は、fresh replacement 自体が壁不成立になった場合の安全holdに必要なため残した。
- exit gate、wall admission gate、0.35秒leaseは実際の終了・安全監視に必要であり削除していない。
- パラメータ、magic number、ケース固有ifは追加していない。

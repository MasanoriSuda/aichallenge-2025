# Tasklist

- [x] 最新runの速度・Mission・SafeSeparation時系列を現行entry gateと照合する
- [x] requirements/designを記録する
- [x] entry admissionをpure resolutionへ局所リファクタリングする
- [x] validated Missionをgate迂回ではなくpre-arm条件へ変更する
- [x] pre-armの縦速度ownershipと基準線維持を接続する
- [x] 相対速度閾値とログを更新する
- [x] core unit testを追加・更新する
- [x] `docs/spec/mpc-integration.md`を更新する
- [x] package testと`make autoware-build`を実行する
- [x] `make dev2`向け動的確認項目を記録する

## 動的確認項目

- 速度不足時に`prearm=1`かつOvertakeLineがIdleを維持すること
- pre-arm中にFollow cap、follow gap planner、follow prepositionが入らないこと
- 相対速度+0.3 m/sを0.3秒確認後に1回でShiftOut/Passへ入ること
- `target clear ahead` SafeSeparation、`short horizon unsafe` Recoveryの件数
- ShiftOut/Passからrear-clearまでの所要時間と成功率
- Emergency、壁接触、target jumpでは従来guardが働くこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 CTest targets）
- `colcon test-result --verbose`: 896 tests、0 errors、0 failures、0 skipped
- `colcon test-result`は既存stale artifact
  `build/joycon_contract_guard/package.xml`欠損を警告したが、今回対象の試験結果に失敗はない
- `git diff --check`: 成功
- `make dev2`: 未実施。上記の動的確認項目を次回走行ログで照合する

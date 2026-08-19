# Tasklist

- [x] 最新ログと現行コードの中断経路を照合
- [x] preview/actionの純粋判定をcoreへ追加
- [x] controller・config・起動ログへaction TTCを接続
- [x] prediction距離を使う可変shift horizonを追加
- [x] READMEを更新
- [x] core unit testを追加・更新
- [x] Docker内build/testとYAML整合を確認
- [x] ユーザー所有変更を除外してコミット

## Definition of Done

- prediction previewだけでは`ExitCurrentMission`を返さない。
- action帯とhard guardは従来の安全動作を維持する。
- long preview時は退避shiftを設定上限まで延長できる。
- build/testが成功し、`aichallenge/result-summary.json`をコミットしない。

## Verification

- `make autoware-build`: 25 packages success
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 CTest success
- `colcon test-result --verbose`: 1362 tests, 0 errors, 0 failures
- YAML parse / `git diff --check`: success

# Results

## 実装結果

- 非同期のMPCC-lite候補へ、rollout生成時に使用した最低速度要求値を保存した。
- prefix/cross-side Mission採用時は、予測最低速度と同一snapshotの要求値を比較する共通helperへ統一した。
- snapshot metadataを持たない旧候補は、従来どおりlive要求値へfail-closedでfallbackする。
- 数値誤差用の許容値を `0.02 m/s` として設定化した。
- 拒否ログへ `predicted / planned / live` を出し、真の速度不足とsnapshotずれを区別できるようにした。
- freshness、target継続性、壁余裕、時間・距離budget、no-return、phaseなどの現時点hard gateは変更していない。

## 静的検証

### Build

```text
make autoware-build
[build_autoware] Build successful.
Summary: 25 packages finished
```

### Test

```text
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
Summary: 1333 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result` は別packageの古い生成物
`build/joycon_contract_guard/package.xml` が存在しないという警告を表示したが、
テスト結果自体は全件成功している。

### Diff check

```text
git diff --check
# no output
```

## 動的確認

未実施。次回 `make dev2` で以下を確認する。

- `minimum-speed-insufficient` が40 Hzで反復しないこと。
- dual MPCCで選択されたprefixが実行Missionへ採用されること。
- 真に計画時要求速度を下回る候補は引き続き棄却されること。
- Pass/Return完遂数、Recovery数、壁・車両接触を前回走行と比較すること。

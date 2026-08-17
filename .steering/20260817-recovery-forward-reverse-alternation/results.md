# Results

## 静的確認

- Forward失敗は`ForwardManeuver`から離れる時点で記録し、後続の
  `StopAndReassess` / `CheckClearance` / `SafeStop`によるreason上書きの影響を受けない。
- `collision_worsening`と`forward_duration_limit`を失敗として扱う。
- 同一aggressive retry cycleの複数失敗stepは1回だけ累積する。
- 設定閾値到達時は既存の`forced_reverse_retry`を使い、Forward probeを抑止する。
- Reverse実行、Forward escape成功、Recovery終了、新規episodeでtrackerをresetする。

## 実行結果

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

```text
colcon test --packages-select multi_purpose_mpc_ros
100% tests passed, 0 tests failed out of 28
Summary: 1260 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result --verbose`は、今回と無関係な既存の
`build/joycon_contract_guard/package.xml`欠損を警告したが、テスト集計は失敗0だった。

## 未確認

実走で次の順序を確認する。

1. Forward失敗を含むaggressive retryが2 cycle発生する。
2. `Stuck recovery alternating after failed Forward`が出る。
3. 次のmaneuverが`direction=Reverse`になる。
4. 壁contact数が減り、通常走行へ復帰する。

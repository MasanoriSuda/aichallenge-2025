# Results

## 実装結果

- 旧 `Q/QN * 0.001` を廃止し、拡張MPCC専用のstage/terminal横・姿勢重みへ分離した。
- 拡張MPCCと3-state MPCCの速度指令を0.15秒で連続化した。
- mode handoffは現在周期の速度上下限内でのみ補間する。front-risk capが低下した場合は即時に新しい上限へclipする。
- ローカルtheta、warm-start再基準化、failure circuit breakerは維持した。
- 起動ログへ実効追従重み、failure cooldown、handoff時間を追加した。

## 静的検証

### Build

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

### Test

```text
colcon test --packages-select multi_purpose_mpc_ros
Summary: 1261 tests, 0 errors, 0 failures, 0 skipped
```

初回試験では0.20秒ちょうどが浮動小数点誤差で遷移中と判定される境界不具合を検出した。終了判定へ1 ns toleranceを追加し、全試験成功を確認した。

## 動的確認項目

次回 `make dev2` では以下を前run `20260818-190144` と比較する。

- `Pass -> Return` 成功数（前runは0）
- `ShiftOut -> FollowPrepare` と `Pass -> FollowPrepare` の回数
- `Stuck detector` に出る `abs(e_y)` と `abs(e_psi)`
- `optimized horizon escaped wall bounds` とwall contact回数
- Extended MPCCのsuccess/fallback/circuit_skipとsolve時間
- 追い越し開始後の急な速度指令低下

動的試走はユーザー実施待ちであり、追従性能改善の最終判定は未実施。

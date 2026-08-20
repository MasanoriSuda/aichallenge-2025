# Design

## 原因

`solved_mpcc_execution_path_wall_safe()` は既定で離散 stage の footprint のみを
確認していた。`dynamic_margin_escape_solution_wall_safe()` だけは swept path を
明示的に検証する一方、通常の rolling MPCC source と stitch 後の handoff は
離散検証のままだった。

この bool 既定値は呼び出し側から検証範囲が分かりにくく、将来も同じ欠陥を
再導入しやすい。

## 変更方針

1. 検証範囲を `SolvedExecutionWallValidationScope` で明示し、既定引数をなくす。
2. solved-MPCC の latest、last-feasible、stitch 後 handoff はすべて
   `SweptFromCurrentPose` を指定する。
3. swept path は現在の `actual_wall_monitor_pose_` を始点として、各 stage 間を
   static wall grid の解像度以下で補間する。
4. 新解が不合格なら既存の DP execution path を変更しない。既存の
   `resolve_solved_execution_source_handoff()` の atomic promote 契約を維持する。
5. promote / pending ログへ `wall_validation=swept-current-to-horizon` を追加する。

## 影響範囲

- `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- controller の内部検証とデバッグログのみ
- 設定、topic、外部インターフェースの変更なし

## 動的確認項目

- `MPCC solved execution source promoted` が
  `wall_validation=swept-current-to-horizon` を出す。
- 接続区間が壁を横切る場合は `retained as pending` に
  `solution swept wall path ...` が出る。
- `Overtake wall path invalidated` の
  `execution_contract_mismatch=1` と実壁接触が減る。
- Pass 完遂率、callback overrun、solver failure が悪化しない。

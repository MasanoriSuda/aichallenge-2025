# Requirements

## 背景

`output/20260820-233247` では solved-MPCC handoff の連続壁検証により、危険な
新解の promote と実壁接触は抑制できた。一方、追い越し開始後の wall path
invalidation は5回残り、すべて `path_source=receding-dp` かつ
`execution_contract_mismatch=1` だった。

左右候補の幾何・DP preflightは通過しても、左右MPCC branchが実際に出した軌跡を
物理footprintで採用前検証していない。このため、壁側候補を一度実行してから棄却し、
FollowPrepare / Recoveryへ移ることがある。

## 目的

- 左右MPCC branchの解を、現在の実測姿勢から全horizonまで連続壁検証する。
- 選択側が物理壁契約を満たさず逆側が満たす場合、逆側を採用する。
- 両側が不成立なら追い越しMissionを開始せず、現在の安全な走行authorityを維持する。
- ログだけで左右のsolver成否、物理壁検証、最終採用側と棄却理由を追跡できる。

## 制約

- no-return後の全幅切り返し条件は緩和しない。
- 壁clearance、速度、Recovery等のconfig値は変更しない。
- ROS 2 topic、message、service、launch、提出物契約を変更しない。
- `output/`、result JSON、crash artifactを変更・コミットしない。

## Definition of Done

- 左右extended MPCC branchが物理footprintの連続壁検証を通った場合だけfeasibleになる。
- 片側のみwall-safeなら既存のdual branch選択がその側を選ぶ。
- 新規entryで両側不成立なら幾何候補をfallback採用しない。
- periodic logに左右のwall validation状態、required clearance、理由、選択結果が出る。
- package buildと単体テストが成功する。

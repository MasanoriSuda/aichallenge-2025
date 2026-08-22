---
name: mpcc-root-cause-auditor
description: Automotive AI ChallengeのMPC/MPCC制御回帰を、authority競合、定式化切替、solver/fallback、wall/corridor証明、async provenanceの因果関係から監査する。「MPCCを総点検」「根本原因を調べる」「MPCとMPCCの切替」「パッチが泥団子」「堂々巡りを止めたい」で使用する。単純なパラメータ説明や、根本原因が承認済みの小規模実装だけには使用しない。
---

# MPCC Root Cause Auditor

MPC/MPCCの最後の警告を消すのではなく、観測された失敗までの因果連鎖で最初に破られた
不変条件を特定する。

## Start

1. `git status --short`でユーザー変更を確認する。
2. リポジトリrootと対象packageの`AGENTS.md`を読む。
3. `docs/spec/mpc-integration.md`、関連interface、現在のsteeringを読む。
4. 監査基準commit、run ID、Domain、対象時刻またはdecision IDを固定する。
5. [references/audit-workflow.md](references/audit-workflow.md)を読み、該当する監査を行う。

## Default mode

根本原因が未確定の回帰は`AUDIT_ONLY`とする。production code、config、test expectationを
変更しない。観測不足なら、適用前の計測案だけを示す。

ユーザーが監査済みのimplementation sliceを明示的に承認した場合だけ実装へ進める。その場合も、
修正前に失敗するtest/replay、修復するinvariant、削除するmask/bypassを1対1で対応させる。

## Required distinctions

必ず次を分離する。

- Root cause: 最初の不変条件違反。
- Contributing cause: 発生確率または影響を増幅する条件。
- Mask: fallback、hold、retry、Recoveryなど表出を遅らせる処理。
- Detection gap: 発生源で拒否できず、後段まで流れた理由。
- Recovery behavior: 根本修正ではなく安全確保を担当する処理。

OSQP failure、wall reject、急減速、Recoveryは、それ以前のproblem/certificateが正しいと証明されるまで
root causeと断定しない。

## Output gate

実装提案の前に最低限示す。

1. 期待挙動と実挙動。
2. 最初に破られたinvariant。
3. 不正状態のproducer。
4. downstream mask/bypass。
5. 修正前に失敗するdeterministic test/replay。
6. 修正で削除できるproduction branch/configuration。
7. 代替案と副作用。
8. 未確定事項。

根拠は`file:line`、commit、run/log/replayのいずれかで示す。根拠不足を推測で埋めない。

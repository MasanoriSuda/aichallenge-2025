---
name: mpcc-root-cause-auditor
description: MPCCの未解明回帰をauthority、solver、wall/dynamic proof、async時系列から監査する。原因確定済みの小変更には使わない。
---

# MPCC causal audit

対象packageのAGENTS.mdがnormal authorityと方式比較の契約を定義する。
関連する`docs/spec/mpc-integration.md`、現在のsteeringと実験registryを確認し、
[audit-workflow.md](references/audit-workflow.md)の必要な観測・比較を行う。

監査・計画のみの依頼は`AUDIT_ONLY`。修正依頼では、根拠を揃えてから
承認済み範囲の実装・検証へ続ける。監査完了だけを理由に再承認を求めない。
根拠不足なら、最初の不明境界の観測・再現を先に行う。

実装前に、期待と実際、最初の不変条件違反、producer、後段mask、
失敗するtest/replay、削除可能な経路、代替案・副作用、Unknownを示す。
codeはfile:line、実行はcommit/run/Domain/time/decisionで裏付ける。

OSQP failure、wall reject、急減速、Recoveryを、それ以前の入力・問題・証明が
正しいと確認する前に根本原因と断定しない。root、contributor、mask、検出漏れ、
安全確保のRecoveryを分ける。出力は課題に必要な長さにする。

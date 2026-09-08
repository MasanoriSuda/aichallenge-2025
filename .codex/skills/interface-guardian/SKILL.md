---
name: interface-guardian
description: ROS 2のtopic/service/Domain、提出物、評価JSONの変更による互換性と移行範囲を調べる。
---

# Interface compatibility

変更する契約に応じて`docs/interface/participant-interface.md`または
`docs/interface/evaluation-interface.md`を読み、producerとconsumerの両方を追う。

- 名前・型・QoS・remap・Domainと、launch/serviceの責務を確認。
- 提出entry launch、control_method、tar.gz最上位、result schema、latestリンクと所有者を確認。
- 2026公式の変更と2025由来の暫定を分け、依存側への移行手順を明らかにする。

互換／移行必要／契約違反を根拠のfile:lineとともに報告する。
契約変更を実装する依頼では、先に契約文書へ影響・移行を反映し、依存側と合わせて検証する。

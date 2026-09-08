# Requirements

- 依頼: AGENTS.md、Skills、reasoning設定をGPT-5.6 Sol Extra HighからGPT-6 Astra向けに見直す。
- root/package AGENTSと7 local Skillsを簡素化し、重複・過剰な手順・再承認待ちを減らす。
- 依頼範囲、自律実行、必要な検証を明示。監査依頼を実装依頼に拡張しない。
- ROS/評価/実車契約、MPCC single authorityと原因証拠・方式比較gateを保持。
- project内のモデル/effortを設定。global設定、権限、provider、共有Skillsは変更しない。
- 制御code/config、過去のMPCC計画、ユーザー変更、生成物は今回の変更範囲外。
- 完了条件: 新指示の整合、7 Skillsの検証、設定読取確認、変更・未計測事項の報告。

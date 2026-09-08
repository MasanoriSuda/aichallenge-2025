# Design

- 共通ルールはroot AGENTS、MPCC固有契約はpackage AGENTS、観測方法はSkillへ集約。
- rootの15互換契約を意味を保って7項へ整理。command一覧はREADME/正本へのリンクと検証表へ。
- 承認ルールは「監査のみ」と「修正まで依頼済み」を分ける。
  MPCC修正前の再現・不変条件・producer・削除対象・受入れ・rollbackは維持。
- 7 Skillsの名前とpathを維持。固定レポート雛形と独自保存先、重複した一般手順を削除。
- latestのrun混在、時刻原点、snapshot payloadの対応は解析の判断に必要なので残す。
- project defaultはAstra high。難しいMPCCはCLI/UIでxhigh、軽微な作業はmedium。
  highはローカル運用判断で、従来Sol xhighとの同等性能を主張しない。
- 設定変更を実行中会話のreasoning変更とは扱わない。
- 正本は[docs/spec/codex-harness.md](../../docs/spec/codex-harness.md)。

読み取り確認: Codex CLI 0.92.0、global Astra/xhigh、Astra catalog default medium、
current project trusted。公式Astra guidance、config basics/reference、Skills説明を確認。

保護された`.codex/`への書込は、完成した差分を一時ディレクトリで検証してから適用する。
適用前hashを照合し、並行する変更を上書きしない。

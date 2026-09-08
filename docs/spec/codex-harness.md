# Codexハーネス

2026-09-07確認。GPT-5.6 Sol Extra HighからGPT-6 Astraへ移行するための、
本リポジトリのagent指示・Skills・reasoning運用。制御器や大会仕様は変更しない。

## 指示の配置

| 配置 | 所有する内容 |
|---|---|
| [root AGENTS.md](../../AGENTS.md) | 依頼範囲、自律実行、repo境界、互換契約、必要な検証 |
| [MPCC AGENTS.md](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/AGENTS.md) | normal authority、不変条件、原因修正・方式比較gate |
| `.codex/skills/*/SKILL.md` | 各作業に固有の観測・判断。共通ルールや固定出力雛形を重複させない |
| `.steering/` | 目的、根拠、設計、進捗。過去の承認待ちは現在の権限を決めない |
| [.codex/config.toml](../../.codex/config.toml) | このプロジェクトのモデルとreasoning既定値 |

AstraはSkillsやAGENTSの指示によって作業を止めたり、軽微な変更にも広い検証を行う場合がある。
依頼済みの可逆的な作業を継続し、必要な検証が通ったら完了へ進む指示を明示する。
[OpenAIのAstra移行・prompting guidance](https://developers.openai.com/api/docs/guides/latest-model)

「監査だけ」と「調査して直す」を区別する。後者は原因を確認してから実装へ進み、
調査完了だけを理由に追加承認を要求しない。権限外操作、実車確認、物理証明、
未認証指令の禁止、契約移行手順は維持する。

## Skills

既存の7つの呼出名と配置を維持し、役割の重複と雛形を削減した。
`web-summarizer`の配置は互換性のため`web-summarized/`のまま。
本sessionが検出している`.codex/skills/`を使い、別の探索pathへ重複コピーしない。

| Skill | 主な入口 |
|---|---|
| `code-reviewer` | 変更差分の不具合レビュー |
| `interface-guardian` | 契約と依存側の移行影響 |
| `error-analyzer` | build、launch、DDS、評価起動の失敗 |
| `evaluation-analyzer` | 完走、lap、penalty、結果欠落 |
| `data-analyzer` | MCAP時系列、topic欠損、制御と自己位置の相関 |
| `mpcc-root-cause-auditor` | 未解明MPCC回帰、authority/proof、方式比較 |
| `web-summarizer` | 大会・Autoware等の外部公式仕様 |

必要な観点から開始し、追加の観測が必要になった場合だけ別Skillへ進む。
Skillは選択時に本文が読み込まれるため、descriptionを短く具体的にする。
[公式Skills説明](https://learn.chatgpt.com/docs/build-skills)

## Reasoningの選択

確認時のユーザー設定は既に`gpt-6-astra / xhigh`。本プロジェクトの通常開発既定を
`gpt-6-astra / high`とし、難しいMPCC監査では`xhigh`を明示選択する。
共有の`~/.codex/config.toml`や他プロジェクトは変更しない。

| 作業 | 選択 | 判断 |
|---|---|---|
| 通常の実装・修正・レビュー | `high` | 本repoの既定。調査と実装をまとめて扱う |
| 原因不明MPCC回帰、定式化・非同期・物理証明の比較 | `xhigh` | 従来と同じeffortを移行基準として維持 |
| 文言・リンク修正、既知コマンドや結果の整理 | `medium` | 原因推論を伴わない作業で明示選択 |

`high`で従来のSol `xhigh`と同等品質になることは未検証。
公式の移行基準は既存effortの維持であり、通常作業`high`はユーザーの再検討依頼に
基づくローカル運用判断。MPCCの難しい監査はまずAstra `xhigh`で基準を取り、
同じ課題・受入れ条件で`high`を比較してから下げる。
[Astra移行ガイド](https://developers.openai.com/api/docs/guides/latest-model)

利用例（repo rootから。reasoningはAGENTSの文章だけでは切り替わらない）:

```bash
# 通常開発: project configのAstra high
codex
# MPCC根本原因監査: Astra xhigh
codex -m gpt-6-astra -c 'model_reasoning_effort="xhigh"'
# 軽微な作業
codex -m gpt-6-astra -c 'model_reasoning_effort="medium"'
```

UIから開始する場合もモデルとreasoningの選択を確認する。
既存会話の選択をファイル編集で遡って変更できるとは扱わない。
project設定はtrusted projectで読み込まれ、明示したCLI overrideが優先される。
project内の`profiles`は現行公式仕様では無視されるため追加していない。
[設定の優先順位](https://learn.chatgpt.com/docs/config-file/config-basic)、
[設定reference](https://learn.chatgpt.com/docs/config-file/config-reference)

ローカルmodel catalogのAstra既定は`medium`。catalogには`max/ultra`もあるが、
CLI 0.92.0と公式config referenceで共通に扱える`medium/high/xhigh`を本設定で使う。
長い思考や自動委譲を一律に有効化する設定は追加しない。

## 検証と見直し

静的検証はSkill frontmatter、参照先、TOML、CLIによる設定読取、契約の保存を対象にする。
このハーネス変更だけでROS buildやAWSIMを再実行する必要はない。

代表課題は、文書の小修正、原因確定済みのnode修正、decision 1187のMPCC監査。
依頼範囲の完了、不要な確認回数、反復した検証、原因と根拠の正確さ、所要時間を比較する。
同じ凍結入力と同じ受入れ条件を使い、token削減だけで成功と判定しない。
今回モデルへの課題再実行は行っておらず、reasoning別の品質・速度・費用差は未計測。

| 依頼例 | 指示の整合確認で期待する扱い |
|---|---|
| READMEのリンクを直す | 修正とリンク確認で完了。ROS buildを追加しない |
| launch不具合を直す | 調査・修正・必要なbuildへ進む。調査後の再承認は不要 |
| MPCCの原因を監査して | 証拠・比較・結論まで。production変更はしない |
| MPCCの回帰を調べて直す | 再現→原因→修正→削除→検証。根拠不足をparameter調整で埋めない |
| もう一つleaseを追加する案 | packageの方式比較gateと一時例外の要件を適用 |
| 実車の操舵を変更する | シミュレータ確認と既存の実車・権限境界を適用 |

この表は指示の整合レビューであり、モデルの実行評価結果ではない。

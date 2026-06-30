---
name: web-summarizer
description: Automotive AI Challenge 2026、Autoware、AWSIM、ROS 2、Docker などの外部・公式情報を調査して構造化レポートにする。最新仕様、公式ドキュメント、リリース情報、規約確認が必要なとき、「調べて要約」「公式情報を確認」「2026仕様を調査」などで呼び出される。
---

# Web Summarizer

外部情報を調査し、2026 ベース化に使える形で要点、根拠、未確定事項を整理する。

## 情報源の優先順位

1. Automotive AI Challenge 公式ドキュメント
2. 公式 GitHub repository / release / issue / discussion
3. Autoware / ROS 2 / AWSIM / Docker の公式ドキュメント
4. 信頼できる技術記事や発表資料

公式情報と推測を混ぜない。2026 仕様として確証がない内容は「未確認」または「2025 由来の暫定」と明記する。

## 手順

1. ユーザーの調査対象を 1〜3 個の検索クエリに分解する。
2. 公式ソースを優先して検索する。
3. 日付、対象バージョン、リポジトリ名を確認する。
4. このリポジトリへ影響する点を `docs/spec/`、`docs/interface/`、AGENTS 方針に結び付けて整理する。

## 出力

```markdown
# Research Report

## Summary
- 主要結論

## Confirmed Facts
- 根拠 URL 付きの確定事項

## Unconfirmed / TBD
- 2026 仕様として未確認の事項

## Impact on This Repository
- 変更が必要な docs / code / workflow

## Sources
- [title](url)

## Checked At
YYYY-MM-DD
```

保存が必要な場合は `reports/YYYY-MM-DD_topic.md` に書く。

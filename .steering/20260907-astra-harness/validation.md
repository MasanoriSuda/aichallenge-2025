# Validation

## 変更量

| 指示 | 変更前 | 変更後 |
|---|---:|---:|
| root AGENTS | 291行 | 106行 |
| MPCC AGENTS | 141行 | 79行 |
| 7 SkillsのSKILL.md | 447行 | 126行 |
| MPCC監査reference | 164行 | 58行 |
| 合計 | 1043行 | 369行 |

行数は読み込む指示の整理量であり、tokenや品質の実測ではない。
モデル別説明と運用判断は`docs/spec/codex-harness.md`へ置いた。

## 実施した確認

- `git diff --check`: 成功。
- skill-creatorの`quick_validate.py`相当のvalidatorで既存7 Skillsを確認: 全件成功。
- Python `tomli`でproject configをparse: `gpt-6-astra / high`。
- Codex CLI 0.92.0の`app-server`でinitialize→`config/read(cwd=repo)`:
  model=`gpt-6-astra`、model_reasoning_effort=`high`。
  両値のoriginは本repoの`.codex`を指すproject layer。モデル推論は実行していない。
- ローカルMarkdownリンク19件、code fence、適用した11ファイルのhash: 成功。
- rootの旧15互換契約を新7項へ対応確認: Domain/V2X/admin/AWSIM/launch/topic/service/
  提出JSON/latest/UID-GIDを保持。
- MPCCのsingle authority、同一solution/proof、async/warm-start、Emergency/Recovery、
  原因証拠、A/B/C/D比較、例外の明示承認と失効条件を差分確認。
- 監査依頼はAUDIT_ONLY、修正依頼は原因確認後に依頼範囲で実装、というroot/package/Skill整合を確認。

## 未実施・影響範囲

- モデル別・reasoning別の代表課題評価は未実施。Astra highとSol xhighの同等性は未検証。
- ROS build、制御test、AWSIMは未実施。変更はagent指示・モデル設定・文書のみ。
- global config、provider、権限設定、共有Skills、制御code/config、過去MPCC計画は変更していない。
- 作業開始時の結果JSON 2件と未追跡生成物を保持。commitは作成していない。
- 新規sessionのproject既定を変更したもので、進行中会話の選択を変更した証拠ではない。

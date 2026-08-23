# Follow canonical shadow dynamic evidence

## Purpose

Follow fresh canonical shadowが実走中にどの段階まで到達するかを計測し、production authority昇格の
判断材料を得る。

## Constraints

- Follow canonical commandはshadowのまま維持する。
- パラメータ調整や新しいfallbackを同時に行わない。
- ユーザー所有の`aichallenge/result-summary.json`を上書きしない。
- 失敗時は最初のreject境界をログから特定し、観測なしに修正しない。

## Acceptance

- `Follow MPCC shadow runtime`のcanonical-ready率と段階別件数を取得する。
- `authority=shadow, selected=0`を確認する。
- wall/effective-gap/canonical-chainの主要reject理由を集計する。
- callbackまたはshadow total時間に明確な過負荷がないか確認する。

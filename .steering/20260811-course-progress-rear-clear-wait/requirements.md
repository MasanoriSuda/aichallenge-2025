# Requirements

## Objective

`output/20260811-094132` で確認した次の追い越し失敗を局所修正する。

1. dynamic Mission wait へ入るために現Missionをinvalidateした直後、再計画期限を待たずRecoveryへ落ちる
2. ヘアピン外側の物理走行距離をそのままcourse progressとして扱い、rear-clear距離を過小評価する

## Constraints

- hard fault、target continuity喪失、実車体重複は従来どおりRecoveryとする
- invalidate済みMissionをそのまま再開しない
- 新しいcurrent/alternate Missionは従来どおり完全preflight後に置換する
- 速度、加速度、壁余裕、車体寸法は変更しない
- ユーザー生成物 `aichallenge/result-summary.json` は変更しない

## Acceptance criteria

- invalidate済みgenerationでも、replacementが未成立ならdynamic wait期限までHoldする
- replacementが成立すればcurrent/alternateへtransactionalに置換する
- rear-clear rolloutが `s_dot = v / (1 - kappa * e_y)` のcourse progress係数を使う
- outer offsetではrear-clear予測が遅く、inner offsetでは速くなる
- build/testが成功する

# Design

## Wall-escape prefix execution ownership

wall-escape prefix採用時に、そのMission generationがローカルwall prefixであることを状態へ記録する。
この状態かつDP execution authorityが有効な間だけ、次を適用する。

- 通常DP pathの2 m級terminal distanceではなく、専用の0.25 mを使う。
- Pass entry physical gateへ渡すwarningを抑制し、事前検証済みDP prefixを実行する。

hard wall fault、target continuity喪失、予測footprint重複などはDP authorityを従来どおり即時に
失効させる。その場合はPass entry gateやMission再探索へ戻るため、prefix flagだけで安全guardを
迂回しない。

## Return phase goal

pass lateral goal policyはShiftOut/Passだけに適用する。ReturnとRecoveryでは既存の
`overtake_line_goal_ey()`を使い、基準線0 mを選ぶ。

Return preflight referenceが有効な間はそのdistance-domain pathを使い、coverage終了後は0 mへの
legacy Return profileへ連続的に引き継ぐ。これにより凍結済みpass goalへの逆戻りを防ぐ。

## Configuration

```yaml
v2x_overtake_runtime_wall_prefix_terminal_distance: 0.25
```

この値はwall-escape prefixにだけ適用する。通常の
`v2x_overtake_mpcc_lite_prefix_terminal_distance`は変更しない。

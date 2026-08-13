# Design

## 1. Runtime wall fallback

壁warning中の優先順位を次にする。

```text
fresh robust same-side Missionあり
  -> 原子的に置換

fresh Missionなし、warning継続
  -> nominal target separationを守る中央寄りgoalを生成
  -> ShiftOut/Pass/Return全体をrobust wall clearanceでpreflight
  -> 成立時だけ原子的に置換

中央寄りgoalも不成立
  -> 対象が十分前方かつReturn corridor clearならReturn
  -> それ以外は既存hard guardを維持
```

中央寄り縮退はロバスト車間を無条件に捨てない。通常candidateではロバスト車間を維持し、
wall warning後のfallbackだけ、実寸幅以上の既存名目車間を下限にする。壁hard guardは常に維持する。

## 2. Front-cap lease

初回解除と解除後保持を分離する。

- acquire: robust current footprint + robust predicted sweep
- hold: physical current footprint + physical predicted sweep
- predicted physical overlap: 既存confirm時間後に再適用
- current physical overlap: 既存confirm/ContactContinuation規則
- wall contact、path infeasible、target discontinuity: 即時失効

ロバスト余裕はcandidate採用と初回速度解放に使用するが、一度成立したPassを推奨余裕の
境界揺れだけでFollow速度へ戻さない。

## 3. 非対象

- hard wall clearance値の変更
- 車体寸法の変更
- ContactContinuation条件の攻撃化
- 加速度・最高速度の変更

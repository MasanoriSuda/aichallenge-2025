# Design

## 車間制約の段階縮退

対象車両との前後 footprint が重なる horizon sample では、選択済み side を維持したまま次の順で lateral bound を解く。

1. robust center separation（速度・曲率余裕込み）
2. configured center separation（現設定 1.55 m）
3. physical center separation（現設定 1.45 m）
4. physical separation が trust region だけで拒否された場合、robust wall-feasible bounds 内で trust region を拡張

4 でも成立しない場合は選択 side を継続不能として Recovery を許可する。縮退後の軌道も既存の robust wall・lateral acceleration・static wall 再検証を通す。

## エラー分類

robust margin の縮退成功は `active + fallback` とし、Recovery の原因にはしない。hard failure は receding-horizon が返した具体的な理由を phase 遷移へ渡す。

## shadow 負荷

MPCC-lite shadow はまだ authority を持たないため、評価周期を 0.125 s から 1.0 s へ変更する。機能は無効化せず診断データを残す。

## 影響範囲

- `v2x_overtake_core`: bound 解決を純粋関数として追加
- `mpc_controller_cpp`: live horizon の制約生成と理由伝播
- `config*.yaml`: shadow 評価周期
- unit test: robust/nominal/physical/trust expansion の境界

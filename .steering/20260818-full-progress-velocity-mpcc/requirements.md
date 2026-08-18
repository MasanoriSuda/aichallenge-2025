# Requirements

## 目的

現行の進捗MPCCを、速度を直接入力とする3状態モデルから、速度状態・加速度入力・仮想進捗入力を持つ追い越し実行モデルへ段階的に拡張する。

## 必須要件

- 拡張状態は `[e_y, e_lag, e_psi, v, theta]` とする。
- 拡張入力は `[a, kappa, v_theta]` とする。
- 既存のstage corridorを横位置のhard constraintとして維持する。
- stageごとの速度referenceとhard capを分離する。
- Pass/rear-clear中は終端速度目標を強め、並走後の不要な失速を抑える。
- 拡張QPが解けない周期は、現行3状態MPCCへフォールバックする。
- 現行のROS 2 topic、launch、評価インターフェースを変更しない。
- `aichallenge/result-summary.json` を変更・コミットしない。

## 検証

- 拡張Frenet線形化、速度horizon、解のlegacy形式変換の単体テスト。
- `multi_purpose_mpc_ros` のbuild/test。
- 実走効果は `make dev2` でユーザーが確認する。

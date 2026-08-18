# Design

## 構成

既存の3状態QPは削除せず、同じcorridor・速度上限・横参照から5状態QPを組み立てる。

### 拡張モデル

状態:

- `e_y`: contour error
- `e_lag`: 実進捗率と仮想進捗率の積分差
- `e_psi`: heading error
- `v`: 車速
- `theta`: 仮想コース進捗

入力:

- `a`: 加速度
- `kappa`: 曲率
- `v_theta`: 仮想進捗速度

連続時間モデルは次をEuler離散化してRTI用に線形化する。

```text
e_y_dot   = v sin(e_psi)
e_lag_dot = v cos(e_psi) / (1 - kappa_path e_y) - v_theta
e_psi_dot = v kappa - kappa_path v_theta
v_dot     = a
theta_dot = v_theta
```

### 速度目的

- 各stageで `v_ref[k]` と `v_hard_cap[k]` を別に保持する。
- 通常stageは `q_v (v-v_ref)^2` を使う。
- committed Passではstage速度weightを増やす。
- horizon終端には `q_T (v_N-v_terminal)^2` を追加する。
- 安全上限は従来どおりhard boundが優先し、速度floorはreferenceにだけ反映する。

### フォールバック

拡張QP専用のpersistent OSQPとwarm startを持つ。拡張QPの準備またはsolveが失敗した場合、その周期は既存3状態QPを解く。拡張解は既存後段が扱える `[e_y,e_psi,s] / [v,kappa]` 配列へ変換してから、予測表示・壁再検証・制御出力へ渡す。

## 局所リファクタリング

- 拡張モデルの数式、速度horizon、解変換を `mpcc_progress` に閉じ込める。
- controller本体は既存問題から拡張問題を構築し、solver選択だけを担当する。
- 5x3固有indexを名前付き定数にし、magic numberを拡散させない。

## 非対象

- Recovery/Reverse FSMの再設計。
- 左右branchの並列非同期solve。
- 非線形solverへの全面置換。
- 2026未確定インターフェースの変更。

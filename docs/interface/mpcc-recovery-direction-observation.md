# Recovery方向候補の内部観測形式

`mpcc-recovery-direction-observation/v1`は、CheckClearanceで方向を選べなかった最初の
1件を保存する内部診断。`authority: false`を固定し、通常MPCCのsnapshot・ROS・評価JSONは
変更しない。解析ではrun manifestのcommit・source/binary hashと対応させる。

| 項目 | 意味・並び |
|---|---|
| `decision_id`, `ros_sec`, `steady_sec` | 同一制御判断のID・ROS時計・停止検出側の単調時計 |
| `pose` | 地図上のx[m], y[m], yaw[rad] |
| `footprint` | front, rear, left, right, margin（すべてm） |
| `course` | lateral[m], heading[rad], activation lateral[m], worsening tolerance[m] |
| `grid` | 寸法・原点・解像度・行方向・セル指紋・`wall-grid.bin`への相対参照 |
| `trials[].rollout_parameters` | distance[m], rollout step[m], swept step[m], wheelbase[m], steering magnitude[rad] |
| `trials[].contact_counts` | initial, maximum, final, reduction |

地図payloadはrow-majorのint8でUnknown=-1、Free=0、Occupied=1。積分画像キャッシュを
保存せず、元セルの指紋を照合して再生成できる。enum値は保存版の`recovery_footprint.hpp`と
`stuck_recovery_core.hpp`へ対応させ、未知値を別の意味へ読み替えない。

各trialは実際に呼ばれた候補評価1回分で、`phase`が用途を示す。`feasible`と`reason`は
その物理評価の結果。`course_preview`は元のコース条件を同じ終点へ適用した診断であり、
最終採用の意味を持たない。物理拒否で軌道が空ならコース診断も無効とする。
V2Xは最終clear/情報完全性の要約のみで、相手車による拒否の完全再生には別入力が必要。

保存上限256件を超えた場合、元の候補評価は継続し、`attempted_trial_count`へ全呼出し数を
残して`complete: false`とする。不完全な記録を全候補失敗・走行不能の根拠にしない。
記録失敗や既存ディレクトリを上書きせず、失敗した一時ディレクトリも保持する。

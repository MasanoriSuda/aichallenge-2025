# Design

MPC が使う waypoint index を次の2種類に分離する。

- `tracking_wp_id`: 現在位置に最も近い waypoint。状態変換、モデル線形化、経路制約、V2X、予測軌跡に使用する。
- `preview_wp_id`: `tracking_wp_id + effective_offset`。速度・曲率の入力参照候補にだけ使用する。

モデル線形化の affine 項は tracking waypoint の速度・曲率を operating point として計算する。入力コストは future waypoint が低速なら早めに減速し、同方向でより大きな曲率なら早めに切り増す。future waypoint が高速または小曲率の場合は tracking 値を維持し、カーブ出口での早期加速・早期切り戻しを禁止する。offset を変えても状態方程式と状態座標系は一致したままになる。

offset の選択と index 解決は ROS 非依存の `mpc_waypoint_preview` core に分離し、範囲、低速切替、circular wrap、non-circular clamp を単体テストする。許可範囲は `0..2` とする。

# Design

## 横目標

各sideの新規Mission entryで、現在カーブのinner sideを用いて次の分類を行う。

- inner sideが未定義: straight
- 選択sideとinner sideが異なる: outer
- 選択sideとinner sideが一致: inner

straight/outerでは既存buffer 0.20 mに
`v2x_overtake_minimum_motion_straight_outer_extra_clearance`を加えた横目標と、
既存bufferだけの横目標を同時に生成する。innerでは既存横目標だけを生成する。

## 二段階選択

追加横目標は従来と同じbody-clear、rear-clear、Return、動的corridor、横加速度の
全検証を受ける。さらに、全Missionのminimum wall clearanceが

`line_min_wall_clearance + 実際に適用した追加量`

以上の場合だけ追加clearance tierへ入れる。

追加tierに一つでも完全なMissionがあれば、そのside内では追加tierだけを従来の
性能scoreで比較する。存在しなければ通常tierを比較する。最終的な左右比較では
clearance tierを優先条件にしないため、戦略sideを強制的に外へ変更しない。

## ログ

Mission選択理由へ `clearance_bias=requested/applied_m/fallback` を加える。
起動ログには設定された追加量を表示する。

## 影響範囲

- `config.yaml`, `config_for_cloud.yaml`: 追加量0.10 m
- `mpc_controller_cpp.cpp`: 分類、二段階候補、fallback、ログ
- `v2x_overtake_core.hpp`: 選択済み候補の診断metadata
- `test_v2x_overtake_core.cpp`: buffer増加・狭幅capの単体確認
- package README: 設定契約

ROS topic/service、評価結果schema、Recovery、速度制御は変更しない。

# Design

## 方針

入口で見える瞬間的な gap 幅だけでなく、既存の kinematic rollout と同じ時間軸で相手との横 separation を評価する。

### 1. Pass interaction reserve

rollout 中、ShiftOut 完了後から rear-clear までの

`abs(target_ey(t) - ego_path_ey(t)) - physical_center_separation`

の最小値を `predicted_minimum_pass_target_surface_clearance_m` として Mission 候補へ保存する。

候補順位では次を interaction reserve とする。

`min(predicted target surface clearance, path wall clearance, return wall clearance, corridor half width)`

差が設定閾値以上の場合だけ interaction reserve を優先し、僅差では従来の rear-clear 進捗順位を維持する。

### 2. 動的 body-clear margin

相手の filtered lateral velocity のうち、選択した pass side へ向かう成分だけを intrusion speed とする。

`effective_margin = base_margin + min(max_extra, gain * intrusion_speed)`

相手が反対へ離れている場合と停止中は追加 margin を 0 とする。したがって、停止車両の緊急回避を固定の大きな front-distance で塞がない。

### 3. 再選択

既存の opponent-side replan は actual body overlap と no-return latch を hard guard のまま保持する。将来 overlap が見えた場合の debounce を短くし、alternate の complete Mission が成立した場合だけ atomic replacement を行う。

## 設定初期値

- future interaction reserve advantage: 0.10 m
- intrusion margin gain: 1.0 s / (m/s)
- intrusion margin maximum extra: 0.50 s
- opponent side replan stable time: 0.10 s

## 影響範囲

- `v2x_overtake_core`: rollout 診断値、deadline margin resolver、候補順位
- `mpc_controller_cpp`: 設定読込、候補への診断値伝播、ログ
- `config.yaml`, `config_for_cloud.yaml`: 同一設定を追加
- ROS インターフェースおよび評価基盤は変更しない

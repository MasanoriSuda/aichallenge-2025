# Design

## 観測と実行の分離

- `v2x_front_progress_detection_distance: 24.0 m`: 前方車の早期捕捉。
- `v2x_overtake_gap_lookahead_distance: 30.0 m`: 左右候補と将来経路の評価。
- `v2x_overtake_entry_commit_max_front_distance: 15.0 m`: 新規ShiftOut/Passへ横方向実行権限を渡す境界。
- `v2x_overtake_entry_prearm_max_distance: 15.0 m`: 同一対象に対する速度準備の走行距離上限。

15 mより遠方ではFollow表示になっても、完全Missionまたはsetup候補を継続評価し、前車速度への不要な追従を避ける既存pre-armを利用する。

## 共通Mission契約

新規entryで実行可能とする候補は次を全て満たす。

1. progressive entryではない。
2. body-clear deadlineが確認済みかつ成立。
3. rear-clear predictionが確認済みかつ成立。
4. rear-clear時刻と自車走行距離が有限かつ非負。

MPCC-liteの局所prefixはshadow比較には残すが、上記を満たさない限り`SelectEntry`権限を持たせない。これによりFSMの`rear_clear=inf`とMPCC-liteの有限terminal penaltyが別物のまま実行へ混入する経路を閉じる。

## 継続Mission

距離境界は新規entryだけに適用する。既にShiftOut/Pass/Return中のMissionは、従来どおりlive wall/target/solver guardと再計画で継続・置換する。

## 効果確認

- `V2X behavior: Follow -> Overtake` の前方距離が原則15 m以下であること。
- 新規entryログに `progressive_entry=1` または `rear_clear_t=inf` が出ないこと。
- 15 mより遠方でMPCC-lite shadow評価は続くが、authorityが`entry`にならないこと。
- `ShiftOut/Pass -> Recovery`、SafetyBrake、MPC failure、停止時間が増えていないこと。


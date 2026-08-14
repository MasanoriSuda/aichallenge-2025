# Design

## 方針

Pass由来のFollowPrepareは、新しい追い越しentryではなく、既に獲得した横位置から
rear-clearを完了するための短い戦術pauseとして扱う。

候補生成時に現在の `e_y` をgoalへ追加する。動的corridor境界外にいる場合は、既存の
`add_goal` と同じく最も近い境界へclampする。そのうえで、現在位置からの横補正量が
`v2x_overtake_mpcc_lite_same_side_max_lateral_adjustment` を超える候補を、entry
preflightより前に棄却する。

これにより、現在位置を保持できる場合は横移動0 m、corridorへ少し戻す必要がある場合
だけ設定上限までの補正となる。全候補は従来どおりwall、target sweep、body-clear、
kinematic rolloutで検証する。

## 設定

初期値は `0.35 m` とする。直近の失敗で観測した約1.5 mの切り込みは拒否しつつ、
推定揺れやcorridor境界への小さな復帰は許容する値である。

## 計算時間

専用threadから現在のcontroller/model状態へ直接触れる変更は、snapshot境界がないため
この局所修正では行わない。代わりに、横補正上限でgoalを重いpreflight前に棄却し、
同側rolling replanの同期計算量を削減する。非同期化はimmutable planner snapshotを
先に設計してから行う。

## 影響範囲

- `v2x_overtake_core`: 横補正admissionの純粋関数
- `mpc_controller_cpp.cpp`: 現在横位置候補、早期棄却、診断ログ
- `config*.yaml`: 横補正上限
- `test_v2x_overtake_core.cpp`: admissionの境界値・左右対称テスト


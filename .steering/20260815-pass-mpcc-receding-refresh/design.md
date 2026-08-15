# Design

## 方針

既存の Frenet DP 左右枝生成と receding-horizon lateral optimizer を再利用する。新しい solver を並設せず、既に生成されている same-target/same-side rolling candidate の実行接続と検証を修正する。

## 1. Prefix stitching

rolling candidate の distance-domain path をそのまま現在状態へ差し替えると、最初の数点が不連続になり横加速度制限へ入りやすい。そこで次の参照を生成する。

- 先頭は measured `current_ey`。
- 短い固定 prefix は、現在実行中の feasible DP path を現在進捗へ再サンプルした値を使う。
- その後を smoothstep で新 candidate へ接続する。
- 旧 path が利用不能なら measured `current_ey` を anchor とする。

この処理は参照生成だけであり、実行可否は次段の horizon validator が決める。

## 2. Elastic atomic promotion

候補は次の順で検証する。

1. robust planning wall clearance
2. 1 が補正・不成立なら、既存の physical minimum wall clearance

2 であっても、target bounds、wall physical bounds、横加速度、V2X continuity、hard fault は必須とする。physical fallback を使ったことはログへ残す。

## 3. Atomic state replacement

検証済みの stitched path のみを `mission_frenet_dp_*` へ一括反映し、進捗を 0 m へrebaseする。棄却時は active path と runtime-validation lease を変更しない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: stitching の純粋関数
- `mpc_controller_cpp.cpp`: rolling candidate 検証と atomic promotion
- `config.yaml`, `config_for_cloud.yaml`: stitching 距離
- `test_v2x_overtake_core.cpp`: stitching と棄却条件の単体テスト

## 試走時の判定

- `DP execution rolling refresh` が発生すること。
- DP `remaining=0` の継続時間がなくなること。
- `refresh_count` が同一Pass中に増えること。
- Pass -> Return が増え、Pass -> FollowPrepare/Recovery が増えないこと。
- physical fallback 使用時もwall接触・横加速度違反が発生しないこと。

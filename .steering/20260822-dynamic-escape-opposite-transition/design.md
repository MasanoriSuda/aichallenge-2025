# Design

## 1. Forced-side transition prefix

GapPlannerへ任意の`forced_pass_side_transition_distance_m`を追加する。既定値は0で、
既存callerの挙動は変えない。DynamicEscapeの反対側speculative branchだけが正値を渡す。

指定側へまだ連続到達できないstageでは、現在の`desired_ey`を含むfree intervalを選び、
prefixとして保持する。次を満たすfree intervalをgatewayとする。

- 現在の`desired_ey`を含む
- course centerを越えて指定sideへ延びている

gateway stageではhard boundを物理free intervalのまま維持し、targetだけを指定sideへ置く。
次stage以降は従来どおり指定side候補を強制する。これにより障害物を含む不連続な
intervalを一つへ結合せず、接続可能になったstageでのみ横断を開始する。

## 2. Deadline

gatewayが設定距離までに現れなければ`forced-side-transition-expired`で棄却する。
開始点から指定sideへ瞬間移動させず、ホライズン全体を無条件に緩和もしない。

初期値は6 mとする。直近ログの3.98--4.00 m棄却をprefixへ取り込みつつ、20 mの
全ホライズンを待ち続けない値である。通常設定とcloud設定を同値にする。

## 3. Unsafe primary suppression

主候補が`wall-margin-escape`で、threat distanceが開始点の場合、
反対側が採用できなければ主候補をplanner-infeasibleへ落とす。これはwall guardの
緩和ではなく、直近ログで発生した
`危険primary採用 -> OSQP破綻 -> wall handoff -> stuck`
を入口で止めるfail-closed処理である。

## 4. Decision log

candidate traceへ次を追加する。

- `side_transition`: requested / gateway_found
- `transition_prefix_samples`
- `transition_gateway_index/distance`
- `transition_deadline`
- `transition_reason`

decision traceへ`primary_suppressed`と理由を追加する。

branch比較ではalternateがunusableかつresolved side=0の場合、side validityより
usabilityを先に判定し、`alternate-unusable`を出す。

## 5. 非対象

- active Pass中の全幅切替
- Recoveryの後退距離・速度
- OSQPの重み・iteration設定
- 全車両を同一QPへ追加する変更

# Design

## Bounded soft-abort replan lease

SafeSeparationのsoft abort後は、物理hard faultがなく、target continuityとPass側の
現行車体分離が成立する場合だけ、同側再計画へ短いリースを与える。

- 初回soft abortでリースを一度だけ開始する。
- 初期値は`0.25 s / 1.5 m`。
- リース中にfresh same-side Missionまたはlast-feasible maneuverを採用できれば置換する。
- 不成立、期限切れ、hard fault発生時はリースを再armせず、既存のdynamic wait、
  speed-preserving Follow、またはRecoveryへ一度だけhandoffする。
- control周期ごとのWARNは出さず、開始・期限切れだけを記録する。

## Forward-motion stall

SafeSeparation開始後の最高速度と低速継続時間を状態として保持する。次をすべて満たす場合を
一般的なforward-motion stallとする。

- forward escapeを要求中
- SafeSeparation中に一度`3.0 m/s`以上へ到達済み
- 実速度が`1.6 m/s`以下の状態が`0.75 s`継続
- targetに対する前進進捗もfreshではない

固定速度値そのものには依存しない。検出時はsoft abortとして再計画/handoffへ進めるが、
壁接触やemergencyなどのhard fault分類は従来どおり優先する。

## Target execution floor

receding-horizonのtarget bound縮退下限を、

`physical center separation + minimum execution surface reserve`

とする。初期値は`0.10 m`。現在車体の物理overlap guard自体は変更せず、MPCCが車体境界
ぴったりの軌道を実行目標にしないための余裕としてのみ使用する。

## 検証

- soft abortリースの開始、継続、期限切れ、再arm禁止の単体テスト
- forward-motion stallがrear-clearより優先されず、通常低速開始を誤検出しない単体テスト
- target boundが10 cmのexecution floorより狭い候補を拒否する単体テスト
- YAML整合、package build、既存core test

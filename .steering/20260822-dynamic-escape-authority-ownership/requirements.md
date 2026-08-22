# Requirements

## 目的

2026-08-22 の試走で観測された DynamicEscape 中の経路・速度指令チャタリングを、
閾値変更やケース別の例外追加ではなく、実行権と経路所有権の不整合として修正する。

## 観測された現象

- 前回修正後、DynamicEscape attempt は 25 回から 4 回へ減少した。
- それでも同一 attempt 内で `dynamic-escape` と `follow/racing-line` が約 70 回往復した。
- `dynamic-escape` 周期では extended MPCC 解を発行しているが、直後の周期で
  `racing-line` と壁 handoff hold へ切り替わり、最大減速度 `-3.0 m/s^2` が発行された。
- DynamicEscape wall replan 31 回のうち、評価対象が実際の DynamicEscape 経路だった
  のは 4 回だけだった。残り 27 回は racing-line または safety-hold を評価していた。
- 非 DynamicEscape 経路の壁不成立が、保持中の DynamicEscape target/side の失敗として
  backoff/quarantine へ記録されていた。

## 期待する挙動

- 新しい候補が一周期生成できなくても、物理的に承認済みの保持解が有効な間は
  DynamicEscape の実行権・経路所有権・速度所有権を維持する。
- DynamicEscape の「終了」は fresh candidate の欠落ではなく、fresh/retained の両方が
  利用不能になった時だけ検出する。
- 壁判定による DynamicEscape side の失敗記録は、評価した予測経路が fresh または
  retained DynamicEscape に所有されている場合に限る。
- racing-line の壁不成立を DynamicEscape side の solver backoff に変換しない。
- SafetyBrake などの hard safety authority は従来どおり優先する。

## 制約

- クリアランス、速度、加速度などの調整値は変更しない。
- ROS 2 topic/service/message と提出インターフェースは変更しない。
- 既存の物理壁判定を弱めない。
- retained solution は既存の 0.35 秒 lease と horizon cursor の範囲だけで使用する。
- `aichallenge/result-summary.json` の既存ユーザー変更には触れない。

## Definition of Done

- fresh candidate と effective execution authority が別の状態として表現される。
- retained solution の同一性を attempt/target/side で検証する純粋関数とテストがある。
- retained execution 中の authority trace が racing-line へ戻らない。
- retained execution の予測を壁判定前に復元する。
- outgoing racing-line の壁判定を DynamicEscape side failure として記録しない。
- package build と関連単体テストが成功する。


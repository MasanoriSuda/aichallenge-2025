# Requirements

## 背景

`d78cba9` で Dynamic Escape の実行軌道を物理壁判定へ通すようにした結果、
壁へ向かう軌道自体は棄却できるようになった。一方、棄却直後に Dynamic
Escape が終了すると、対象車がまだ前方にいるにもかかわらず RacingLine / Follow
へ横方向の実行権が戻る。このため、壁衝突を避けた代わりに前車後方へ収束し、
SafetyBrake と solver fallback を連鎖させるケースが残っている。

## 目的

- Dynamic Escape 終了時に、壁安全性だけでなく前方動的障害物の未解消も確認する。
- 壁に安全な再計画解が採用されるまで、直前に物理検証済みだった横回避解を短時間保持する。
- 再計画解への引継ぎは、壁判定を通過してから原子的に行う。
- 保持解が失効した場合は古い操舵を無期限に使わない。
- 終了、保持、再計画、新解採用の理由を一行ログで追跡可能にする。

## 制約

- ROS 2 topic / service / message 契約は変更しない。
- 速度安全制御、Emergency Brake、Recovery の実行権は奪わない。
- 壁に不安全と判定された新しい Dynamic Escape 解を公開しない。
- 既存の物理検証済み解の保持時間 `0.35 s` を延長しない。
- `aichallenge/result-summary.json` のユーザー変更は変更・コミットしない。

## Definition of Done

- 前方障害物未解消の Dynamic Escape 終了を独立した終了契約として判定できる。
- 実行中の新解が壁判定で拒否された周期でも、直前の物理検証済み解を復元できる。
- 新しい Dynamic Escape 解は物理壁判定合格後に終了契約を引き継ぐ。
- 対象解消、再計画解採用、Recovery override を単体テストで確認する。
- 対象 package のビルドとテストが成功する。

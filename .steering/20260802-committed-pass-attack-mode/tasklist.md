# Tasklist

- [x] 最新ログの Pass 失敗経路を特定する
- [x] committed Pass attack hold を core に追加する
- [x] locked-target front danger suppression と同じ mode を共有する
- [x] body-clear deadline を hard reject から soft ranking へ変更する
- [x] 競技用 config と起動ログを追加する
- [x] core unit test を追加・更新する
- [x] package build／test を実行する
- [ ] `make dev2` 実走で Pass -> Return、SafetyBrake、Recovery を確認する（ユーザー確認）

## Definition of Done

- 攻撃モード OFF の既存挙動を維持する。
- 攻撃モード ON では、成立済み Pass の予測重複だけで cap を再適用しない。
- 現在 footprint 重複、target 異常、壁／経路不成立では hold しない。
- deadline を満たす候補を優先しつつ、未達候補だけでも mission を生成できる。
- 対象 package の build と test が成功する。

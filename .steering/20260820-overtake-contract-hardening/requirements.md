# Requirements

## 目的

追い越し機能を凍結したまま、2026-08-20 の試走で決定ログから判明した実行契約の欠陥を修正する。

## 対象

- Pass の速度参照・上限・下限が矛盾する `invalid-speed-window` を構造的に禁止する。
- DynamicMissionWait 中に縦制御だけが残り、横制御が racing line へ抜ける状態を禁止する。
- Mission 採用時と runtime wall preplan の壁有効距離の契約を同じ尺度で追跡する。
- 通常の authority owner 揺れは集約し、異常・fallback・契約違反は即時出力する。

## 制約

- 速度、壁余裕、追い越し開始条件のパラメータは変更しない。
- ROS 2 topic、message、launch、提出インターフェースを変更しない。
- `aichallenge_system` は変更しない。
- `output/` と既存のユーザー変更は編集・コミットしない。

## 完了条件

- speed floor は同周期の有限な reference/limit を超えない。
- DynamicMissionWait の Hold は、壁成立時に明示的な横 hold path を所有する。
- 壁契約不足を決定ログの conflict/reason から判別できる。
- 通常の owner chatter は抑制数を含む定期集約になり、warning は遅延しない。
- 対象 unit test と package build が成功する。

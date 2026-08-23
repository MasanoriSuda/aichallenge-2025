# Requirements

## Objective

Overtake canonical fresh shadow の物理証明棄却を、1 秒集計の最後の周期が成功しても失わず、保持解の current-world 再証明へ進めるかを証拠で判断できるようにする。

## Constraints

- 制御挙動、authority、solver 設定、wall margin、速度設定を変更しない。
- 新しい fallback、flag、timeout、lease を追加しない。
- Track/Cruise と Overtake で同じ物理証明理由の分類を使う。
- `output/` と rosbag は編集・コミットしない。
- ユーザー変更 `aichallenge/result-summary.json` は編集・stage・revert しない。

## Acceptance

- Overtake の1秒集計に物理棄却理由別の件数が残る。
- 最後に観測した物理棄却の decision ID と診断詳細が残る。
- Track/Cruise の既存分類と意味が変わらない。
- package build/test が通る。
- 同一 bag replay で6周期の棄却理由を特定できる。

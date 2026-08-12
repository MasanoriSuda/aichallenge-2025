# Requirements

## 目的

前方車を捕捉し、車体クリアまでのShiftOutが物理的に成立しているにもかかわらず、rear-clear、Return、長い予測horizonのいずれかが未成立という理由だけで追い越しを開始しない現象を解消する。

## 要求

- hard wall clearance 0.15 m、車体footprint、横加速度6.0 m/s^2は変更しない。
- 車体クリアまで成立する候補は、全Mission未成立でも初回ShiftOutへ段階的に昇格できる。
- 昇格後は同じ側を保持し、rolling replanと既存のPass完遂処理でrear-clearを目指す。
- 全区間成立済み候補がある場合は、常に段階候補より優先する。
- active Missionの左右置換には不完全候補を使用しない。
- tactical revalidation時、前方車との距離が短い状態でレーシングラインへ早戻りしない。
- 既存ROS topic/service/launch契約は変更しない。

## 対象外

- wall/車体のhard overlapを無視する変更
- Recovery/Reverse全体の再設計
- 評価基盤およびAWSIM側の変更

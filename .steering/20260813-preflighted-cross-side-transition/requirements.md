# Requirements

## 目的

追い越し実行中に反対側の完全 Mission が優位になった場合、将来の scheduled outer transition を含むという理由だけで切り返しを棄却しない。

## 背景

`20260813-004330` では反対側候補の壁余裕が 1.21--1.31 m（要求 0.35 m）ある一方、`additional_side_transition_required` により 8 回棄却された。壁余裕ではなく、事前検証済み遷移を Admission が識別できないことが阻害要因である。

## 要件

- wall clearance 0.20 m の設定は維持する。
- scheduled outer transition を含む候補は、その遷移の壁・横加速度・残余 Pass・Return preflight が完了した場合だけ cross-side replacement に使用できる。
- 未検証の追加遷移は従来どおり棄却する。
- no-return、SafeSeparation、rear-clear、速度、壁、時間・距離 budget の既存制約は緩和しない。
- 1 Mission あたりの cross-side replacement 上限は既存設定を維持する。

## 変更範囲

- `multi_purpose_mpc_ros` の追い越し候補メタデータ、cross-side Admission、単体テスト。
- ROS topic/service/message 契約および評価基盤は変更しない。


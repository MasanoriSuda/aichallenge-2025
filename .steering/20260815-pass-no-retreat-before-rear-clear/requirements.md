# Requirements

## 目的

追い越し中、locked target がまだ自車前方にいるにもかかわらず
`TargetClearAhead` と判定して Return し、Follow / SafetyBrake へ戻る早期離脱を止める。

## 制約

- rear-clear 後の通常 Return は維持する。
- 現車体接触、壁異常、Emergency、実行 corridor 異常は緩和しない。
- 予測 sweep が clear の場合だけ前進加速を許可する。
- 予測だけが一時的に重なる場合は、減速して target を逃がさず、同じ側で再計画を待つ。
- Pass のローカル・絶対時間/距離 budget は維持する。
- ROS 2 topic / service / message 契約は変更しない。

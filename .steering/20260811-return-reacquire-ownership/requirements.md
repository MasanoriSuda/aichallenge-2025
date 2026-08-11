# Requirements

## 目的

20260811-160756 の走行で9回発生した、`Pass -> Return -> Pass` の即時反転を止める。
SafeSeparationが前方へ離れた同一targetを先行させるために選んだReturnは、Return完了までその判断を保持する。

## 対象事象

- `SafeSeparation target clear ahead; speed-preserving Return` の直後、同じ制御周期に
  `same target reacquired during early return` が成立する。
- Return開始前に作られたbehavior出力を再帰呼び出しでも使用するため、意図的な
  tactical disengagementが一般的な早期再捕捉規則で取り消される。
- この反転でPassの時間・進捗状態が繰り返し初期化され、状態遷移ログと速度所有権が
  不安定になる。

## 制約

- rear-clear前のReturnを新しい障害物や再開可能な同一targetから取り消す既存機能は残す。
- wall contact、EmergencyBrake、solver recovery、target continuityのhard faultは変更しない。
- 追い越し余裕、加減速度、横加速度などの性能パラメータは変更しない。
- ROS 2 topic/service、launch、評価インターフェースは変更しない。
- ユーザーの `aichallenge/result-summary.json` 変更は編集しない。

## 完了条件

- Returnが「早期再捕捉可」か「Return完遂」のどちらかを明示的に所有する。
- tactical revalidationから始めたReturnでは同一targetを再捕捉しない。
- 従来のreacquirable Returnは、同一target・同一sideなど既存条件を満たす場合だけ再捕捉できる。
- core単体テストと対象packageのビルドが成功する。

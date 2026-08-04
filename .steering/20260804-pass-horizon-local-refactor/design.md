# 設計

## 方針

巨大な Pass horizon 更新ラムダ全体は移動せず、次の性能修正で変更する境界を
先に抽出する。これにより機械的な大規模移動を避け、挙動等価性を保つ。

## 抽出する責務

1. `evaluate_committed_pass_full_path_preflight`
   - 現行どおり ShiftOut/Pass/Return を含む preflight を実行する。
   - 次工程で Pass 継続と Return 検証を分ける交換点とする。
2. `build_pass_horizon_decision_request`
   - FSM、予測有効期限、距離・時間上限から pure core への入力を構築する。
3. `begin_pass_safe_separation`
   - SafeSeparation の開始可否、状態初期化、既存警告ログを一か所へ集約する。
4. 短期安全スナップショット
   - 1制御周期に一度だけ評価し、preflight policy と SafeSeparation の双方で共有する。

## 非対象

- Pass 継続時に Return を除外する性能変更
- Hard abort / Soft hold の分類変更
- SafeSeparation の速度・時間・距離設定変更
- OvertakeArmed、複数 lateral knot、左右候補 scoring の変更

## 互換性

外部インターフェース、設定ファイル、状態遷移理由、ログ文言に変更はない。

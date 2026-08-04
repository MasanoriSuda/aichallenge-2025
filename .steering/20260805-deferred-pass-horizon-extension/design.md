# 設計

## 原因

従来の判定は次だった。

```text
required_rear_clear_pass > static_valid_until_pass
  -> rear_clear_window_replan_required = true
  -> Pass開始直後にRequestSameSideExtension
```

これは`revalidation_lead_distance: 3.0 m`を無視している。ログではtargetが約5.2 m前方、
static pathが約21.7 m残っている時点で延長し、約23.9 m先のouter-role反転を理由に
SafeSeparationへ入っていた。

## 新しい判定

純粋関数で次を分離する。

```text
beyond_committed_horizon = required_rear_clear_pass > static_valid_until_pass
remaining_committed_path = max(0, static_valid_until_pass - pass_traveled)
replan_due = beyond_committed_horizon
          && remaining_committed_path <= revalidation_lead_distance
```

`beyond_committed_horizon && !replan_due`では現在の検証済みPassを継続する。rolling outer
判定は通常のPass horizon decisionより前にあるため、将来の内外反転が12 m lookaheadへ
入った時点で先にside transitionを試せる。

## 維持する即時trigger

- confirmed predicted footprint overlap
- static/dynamic horizon自体のlead window到達
- dynamic prediction TTLのlead window到達
- absolute Pass distance/time limit
- rear-clear確定後のReturn
- wall/contact/Emergency/target continuity guard

## ログ

1 Passにつき最初の延期時だけ、`Pass rear-clear extension deferred`としてrequired、
static limit、remaining、leadを出す。周期ログは追加しない。


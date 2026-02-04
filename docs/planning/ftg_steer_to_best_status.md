# FTG: B方向への操舵移行 状態管理

## 目的
- 進行方向の判断を FTG（best角）基準に統一する
- `B`（best_angle_deg）と操舵判断の乖離を解消する
- 影響範囲は `Process::proc()` の操舵角算出のみ

## 状態一覧
- [完了] 現状ロジックの整理（best/min/steer の関係）
- [完了] `steerSourceAngle` を best 角ベースに変更
- [完了] 影響確認（符号/ゲイン/安全側挙動）

## メモ
- 参照: `docs/planning/process_decision_telemetry_status.md`
- `PROCESS_MIN_ANGLE_SIGN` を `+1.0f` に変更し、Bと操舵の符号を一致させた

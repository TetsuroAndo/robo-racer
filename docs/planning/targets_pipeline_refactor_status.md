# 入力・安全・出力パイプライン分割（MANUAL削除）ステート

## 目的
- applyTargets_() の責務を分割し、AUTO入力/安全制限/出力を分離する。
- MANUAL（pad操作）を本番構成から除去する。
- TSD20 の IMU 予測マージンを維持したまま制限ロジックをクラス化する。
- ABS 制動の状態をクラス内に閉じ、global を削減する。

## 前提
- 既存の安全系（kill/TTL/TSD20 clamp/ABS）を優先する。
- TSD20 予測マージン（v_est/a_long/tau）は維持する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 仕様確認（関連ドキュメント/現行実装） | 完了 | 2026-02-06 | docs/firmware/plan.md と main.cpp を確認 |
| MANUAL分岐の削除 | 完了 | 2026-02-06 | applyTargets_ から pad 経路を除去 |
| Auto入力の分離（AutoCommandSource） | 完了 | 2026-02-06 | cmd_fresh 判定を移動 |
| TSD20制限のクラス化（Tsd20Limiter） | 完了 | 2026-02-06 | 予測マージン含む |
| ABS制動のクラス化（AbsController） | 完了 | 2026-02-06 | 内部状態を保持 |
| SafetySupervisor による配線 | 完了 | 2026-02-06 | Desired→Safe の流れを固定 |
| ログ/診断の移植 | 完了 | 2026-02-06 | diag struct 化 |
| main.cpp の簡素化 | 完了 | 2026-02-06 | 5〜10行相当へ |

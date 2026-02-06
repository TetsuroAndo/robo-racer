# 速度指令の物理化とテレメトリ100Hz化 ステート

## 目的
- speed_mm_s を物理速度目標として成立させるための速度制御（FF+PI）を導入する。
- STATUS/IMU_STATUS の送信周期を 100Hz（10ms）へ引き上げる。
- 変更内容を `docs/` に設計書として追加する。

## 前提
- 既存の安全系（kill/TTL/TSD20 clamp/ABS）を優先する。
- IMU 推定は校正状態に応じて制御ゲインを抑制する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 仕様確認（既存挙動・関連ドキュメント） | 完了 | 2026-02-06 | docs/state_management.md と firmware 実装を確認 |
| SpeedController 追加（FF+PI, アンチワインドアップ） | 完了 | 2026-02-06 | control/ に新規追加 |
| Drive の責務整理（PWM入力化） | 完了 | 2026-02-06 | setTargetPwm 追加、既存 API 互換維持 |
| main loop への速度制御統合 | 完了 | 2026-02-06 | clamp/ABS 後に速度制御を適用 |
| 送信周期 100Hz 化 | 完了 | 2026-02-06 | STATUS_INTERVAL_MS=10 と main の timer 更新 |
| 速度制御のログ/可観測性追加 | 完了 | 2026-02-06 | 100-200ms ログに追加 |
| ドキュメント追加 | 完了 | 2026-02-06 | docs/firmware, docs/interfaces に追加 |

# エンジンスルーレート低下の修正ステート

## 目的
- Driveのスルーレート設定が「per-second」で解釈される問題を正し、加速遅延を解消する。
- 既存のENGINE_SPEED_STEPを「per tick」の意図に合わせて扱う。

## 前提
- SlewRateLimiterは「単位/秒」で更新する実装。
- Drive::tick()のdt_sはループ依存で変動する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 影響範囲の確認 | 完了 | 2026-02-05 | Drive/Engine/Config/SlewRateLimiterを確認 |
| Driveスルーレート補正 | 完了 | 2026-02-05 | stepをdtでrateに変換 |
| Config注釈の確認 | 完了 | 2026-02-05 | ENGINE_SPEED_STEPの意味を明確化 |
| ビルド/テスト | 未実施 | 2026-02-05 | 実機/CIは別途 |

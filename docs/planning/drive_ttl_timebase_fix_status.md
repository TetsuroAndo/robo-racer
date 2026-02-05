# Drive TTL時間基準ズレの修正ステート

## 目的
- Drive::tick のTTL判定が「now_ms と setTarget() 内の millis() のズレ」で誤って期限切れになる問題を解消する。
- 速度指令が 0mm/s に強制される事象を防ぐ。

## 前提
- now_ms は loop() で一度だけ取得し、tick() に渡している。
- setTarget*() は millis() を使って lastUpdate を更新している。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 影響範囲の確認 | 完了 | 2026-02-05 | Drive.cpp/Drive.h を確認 |
| TTL判定の修正 | 完了 | 2026-02-05 | now_ms < lastUpdate の補正 |
| テスト/ログ確認 | 未実施 | 2026-02-05 | 実機で確認 |

# RPi標準出力にIMU/TSD20ログ追加 ステート

## 目的
- RPi の標準出力ログに IMU 値と TSD20 値を出力する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 関連資料の確認 | 完了 | 2026-02-06 | `docs/observability/logging_v1.md` / `docs/proto/protocol_logging_plan.md` を確認 |
| serialdのIMUログ追加 | 完了 | 2026-02-06 | IMU_STATUSを解釈してstdoutへ出す |
| TSD20ログのstdout出力 | 完了 | 2026-02-06 | ESP32 LOGのtsd20行をstdoutへ出す |

# HB喪失停止 / MODE解除 / TSD20バイナリ化 ステート

## 目的
- **Heartbeat(PING)喪失時に確実に停止**する（DRIVEがTTL内で届き続ける異常系でも停止）。
- **manual無効ビルドでも MODE_SET(0) を受理**し、運用上の「AUTO停止」を自然にする。
- **TSD20状態の取得をLOG文字列パースから脱却**し、専用固定長payloadで可観測性を堅牢化する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 関連資料の確認 | 完了 | 2026-02-06 | `docs/proto/overview.md` / `docs/interfaces/protocol_rpi_esp32_v1.md` / `shared/proto/include/mc_proto.h` |
| (1) HB喪失停止が制御入力生成に効いていない修正 | 完了 | 2026-02-06 | `firmware/src/control/AutoCommandSource.cpp` に hb_fresh 条件追加 |
| (2) manual無効ビルドで MODE_SET(0) を拒否する運用バグ修正 | 完了 | 2026-02-06 | `firmware/src/comm/handlers/ModeHandler.cpp` で mode=0 を “AUTO_OFF” として受理 |
| (3) TSD20 状態のLOGパース依存を解消（TSD20_STATUS追加） | 完了 | 2026-02-06 | `TSD20_STATUS` を追加し、RPi側はバイナリ優先（LOGパースは後方互換として残置） |


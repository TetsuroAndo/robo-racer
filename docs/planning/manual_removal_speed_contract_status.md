# 本番MANUAL排除と速度契約明文化 ステート

## 目的
- 本番ビルドで MANUAL（pad操作）を排除し、AUTOのみで運用できる構成にする。
- `speed_mm_s` の物理一致に関する契約（上限値・符号・許容誤差・slew）を明文化する。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 関連資料の確認 | 完了 | 2026-02-06 | `docs/proto/overview.md`/`docs/firmware/speed_command_v2.md`/`docs/interfaces/ipc_payloads_v1.md` を確認 |
| MANUAL排除（ビルドフラグ化） | 完了 | 2026-02-06 | `MC_ENABLE_MANUAL=0` で MODE_SET/Bluepad32 を無効化 |
| `speed_mm_s` 契約明文化 | 完了 | 2026-02-06 | 上限・符号・誤差・slew を docs に追記 |
| 100Hz運用の文書整合 | 完了 | 2026-02-06 | DRIVE/STATUS周期とTTL例を100Hz前提に統一 |

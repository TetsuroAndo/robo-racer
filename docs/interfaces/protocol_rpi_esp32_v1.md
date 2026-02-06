# RPi↔ESP32 プロトコル v1（Interface Freeze）

## 目的
RPi↔ESP32間の通信を「安全優先・低遅延・拡張可能」に保つため、v1の仕様を凍結する。
この文書は**運用上の契約**であり、変更は原則として v2 へ持ち越す。

## 正本（詳細仕様）
- `docs/proto/overview.md`
- `docs/proto/log_transport.md`

本書は上記の要点をまとめ、凍結範囲を明示する。

## 物理・フレーム仕様（v1固定）
- UART: 921600 bps / 8N1
- フレーミング: COBS + `0x00` 終端
- CRC: CRC16-CCITT-FALSE（header+payloadに対して計算）
- Header v1は9 bytes固定（MAGIC/VER/TYPE/FLAGS/SEQ/LEN）

## type一覧（v1固定）
- 0x01 `DRIVE`   : 操舵/速度/TTL
- 0x02 `KILL`    : 即時停止（ラッチ）
- 0x03 `MODE_SET`: MANUAL/AUTO 切替
- 0x04 `PING`    : 生存確認（ACK応答）
- 0x10 `LOG`     : ESP32ログ（level + text）
- 0x11 `STATUS`  : 状態通知
- 0x12 `HILS_STATE` : HILS観測（既存）
- 0x80 `ACK`     : ACK応答

> 本番ビルドでは `MC_ENABLE_MANUAL=0` を想定し、`MODE_SET` は **AUTOのみ受理** する。

## 安全ポリシー（凍結）
- **KILLは最優先**（どのモードでも即時停止、解除はローカル操作）
- **AUTO中は手動入力を無視**（介入はKILLのみ）
- **TTL失効で停止**（受理時刻基準、RPi時刻は不使用）

## 互換性ルール（v1）
- `ver != 1` のフレームは破棄する
- `len` が期待と合わない場合は破棄する
- 未知の `type` は破棄（ログのみ許可）
- 予約ビットは0固定（v1）
- 追加は **typeの拡張**で行い、既存type/サイズを変更しない

## 変更管理
- v1の変更は**原則禁止**。必要な変更は v2 として追加する
- v1の仕様差異は `docs/proto/*` を一次情報として判断する

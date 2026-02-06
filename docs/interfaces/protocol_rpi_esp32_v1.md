# RPi↔ESP32 プロトコル v1（Interface Freeze）

## 目的
RPi↔ESP32間の通信を「安全優先・低遅延・拡張可能」に保つため、v1の仕様を凍結する。
この文書は**運用上の契約**であり、変更は原則として v2 へ持ち越す。

## 正本（詳細仕様）

**wireレベル（ヘッダ/flags/type/payloadレイアウト/周期）の正本**は **1箇所のみ**とし、他は要約とします。

- **一次情報（正本）**
  - `docs/proto/overview.md`
- 補助ドキュメント（ログ/実装方針など）
  - `docs/proto/log_transport.md`
  - `docs/interfaces/protocol_rpi_esp32_v1.md`（このファイル）

> ※ wire仕様（ヘッダ構造、flagsの意味、各typeのpayload定義、送信周期など）に食い違いがある場合は、  
> **必ず `docs/proto/overview.md` を真とみなす**。

本書は `overview.md` の**要点を運用視点でまとめたもの**であり、凍結範囲を明示する。

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
- 0x13 `IMU_STATUS` : IMU推定状態
- 0x14 `TSD20_STATUS` : TSD20状態（固定長）
- 0x80 `ACK`     : ACK応答

### typeごとの送受信周期（推奨 / 契約値）

> 詳細な根拠・チューニング指針は `docs/proto/overview.md` の **3.6 タイムアウト・周波数** を参照してください。  
> ここでは「誰が / どのくらいの周期で / 何を送るか」の運用契約だけを簡潔にまとめます。

- **DRIVE (0x01, RPi→ESP32)**  
  - 周期: **100 Hz（10 ms）程度**  
  - 備考: `ttl_ms` は制御周期の 2〜3 倍（例: 20〜30 ms）
- **MODE_SET (0x03, RPi→ESP32)**  
  - 周期: イベント駆動（モード切替時のみ）
- **KILL (0x02, RPi→ESP32)**  
  - 周期: イベント駆動（非常停止時のみ、必要なら複数回送信してもよい）
- **PING (0x04, RPi→ESP32)**  
  - 周期: **10〜50 Hz**（リンク監視目的）
- **STATUS (0x11, ESP32→RPi)**  
  - 周期: **100 Hz（10 ms）前後**  
  - 内容: `seq_applied` / `auto_active` / `faults` / `speed_now_mm_s` / `steer_now_cdeg` / `age_ms`
- **IMU_STATUS (0x13, ESP32→RPi)**  
  - 周期: **100 Hz（10 ms）前後**（IMU更新周期に合わせる）
- **TSD20_STATUS (0x14, ESP32→RPi)**  
  - 周期: おおよそ **TSD20 センサ更新周期（200 Hz）を1/数に間引き**して 50〜100 Hz 程度
- **LOG (0x10, ESP32→RPi)**  
  - 周期: 非周期（イベント駆動 / ログ設定に依存）
- **ACK (0x80, ESP32→RPi)**  
  - 周期: 非周期（`ACK_REQ` を付けたフレーム受理時のみ）

### flags 使用方針（v1の凍結内容）

flags の**一次定義**は `docs/proto/overview.md` の「flags（共通）定義（v1固定）」節にあります。ここでは運用ルールをまとめます。

- **共通ルール**
  - v1 で意味を持つのは **bit0: `ACK_REQ` のみ**。
  - bit1〜bit7 は **常に0** にする（送信側）。受信側は無視する。
- **RPi→ESP32 側**
  - `DRIVE` / `MODE_SET` / `KILL` / `PING` で **必要な場合のみ `ACK_REQ` を立ててよい**。
  - それ以外のbitは 0。予約bitに意味を持たせない。
- **ESP32→RPi 側**
  - `ACK` 返信時は **`flags=0` 固定**（payloadも空）。
  - `STATUS` / `IMU_STATUS` / `TSD20_STATUS` / `LOG` では、v1 では `flags` を使用しない（常に0）。
- **PING と ACK の特例**
  - `PING` は `ACK_REQ` に関係なく、**len==0 の `PING` を受理したら必ず `ACK` を返す**。
  - 将来 `PING_EX` を追加する場合も、本ルールは v1 の後方互換として維持する。

> 本番ビルドでは `MC_ENABLE_MANUAL=0` を想定し、`MODE_SET` は **AUTO(1) と AUTO_OFF(0) を受理**する（手動操作自体は提供しないが、運用上の「AUTO停止」を許可する）。

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

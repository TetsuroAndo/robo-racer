# LOG/HILS 設計ドキュメント（Plan）

このドキュメントは、ESP32ファームウェアにおけるログ（LOG）の
設計思想、構成、データフロー、プロトコル仕様をまとめた計画書です。

## 目的

- 低遅延の制御ループを阻害しないログ出力を提供する
- RPi側で再利用できる共通フレーム（COBS+CRC）でログを送る
- 将来のプロトコル統合（Status/Command）に備えた拡張性を確保する

## 設計方針

1. 制御経路の安定性最優先
   - ログは「捨てられても良い副経路」扱い
   - レベル/トピックで出力量を制御
2. ログと制御のフレーム統一
   - `Protocol.h` の `MsgType::Log` を使用
   - UART/UDPの双方でCOBS+CRCフレームを利用
3. 拡張に強いプロトコル設計
   - ログ送出はSinkで拡張できる

## コンポーネント構成

### ログ

- `firmware/src/log/Log.h`
- `firmware/src/log/Log.cpp`
- `Logger`, `Sink`, `StreamSink`
- レベル/トピック定義、ログマクロ

### ログ出力拡張

- `firmware/src/log/LogSinks.h`
- `firmware/src/log/LogSinks.cpp`
- `ProtocolSink`（UART/COBS+CRC）
- `UdpSink`（Wi-Fi UDP/COBS+CRC）

### プロトコル

- `firmware/src/comm/protocol/Protocol.h`
  - `MsgType::Log`, `LogPayloadHeader`
- `firmware/src/comm/protocol/Codec.h`
- `firmware/src/comm/protocol/Codec.cpp`
  - COBSエンコード、CRC16-CCITT
- `firmware/src/comm/protocol/PacketWriter.h`
- `firmware/src/comm/protocol/PacketWriter.cpp`
  - 送信側のパケット組み立て

HILSの設計は `docs/firmware/hils_plan.md` を参照してください。

## データフロー設計

### テキストログ（Serial/BLE）

```
MC_LOGI(...) -> Logger -> StreamSink -> Serial/BLE
```

### UARTのCOBS+CRCログ

```
MC_LOGI(...) -> Logger -> ProtocolSink -> COBS+CRC frame -> UART
```

### UDPログ（高頻度用途）

```
MC_LOGI(...) -> Logger -> UdpSink -> COBS+CRC frame -> UDP
```

## ログプロトコル仕様（MsgType::Log）

`Protocol.h` のヘッダに `LogPayloadHeader` + message bytes が続きます。
messageはNUL終端されません（`msg_len` で長さを示します）。

```
Header:
  u8  version
  u8  msg_type = 0x90
  u8  seq
  u8  flags
  u16 payload_len
  u16 crc16_ccitt (header[version..payload] を対象)

LogPayloadHeader:
  u32 now_ms
  u8  level
  u8  topic
  u16 msg_len
  u8  msg[msg_len]
```

フレーミング:

```
COBS(frame) + 0x00 terminator
```

UART/UDPとも同一フレーム仕様です。

## 制約・注意点

- ログメッセージは固定長バッファ（128 bytes）で整形される
- COBSログ/UDPログは最大96文字に切り詰められる
- `Logger` のSink数は最大3
- COBSログとテキストログを同じUARTに同時出力すると解析が困難

## 拡張の方向性

- `PacketReader` を追加し、ESP32側の受信も同一プロトコルで統一
- `MsgType::Status` とログ送信頻度のレート制御
- HILS関連は `docs/firmware/hils_plan.md` を参照

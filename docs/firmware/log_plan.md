# LOG 設計ドキュメント（mc_proto LOG v1）

このドキュメントは、ESP32 ファームウェアにおけるログ（LOG）の
設計思想、構成、データフロー、プロトコル仕様をまとめた運用設計書です。
**wire 上の値は `shared/proto/mc_proto.h` を正とし、本書は運用・設計の補助です。**

## 目的

- 低遅延の制御ループを阻害しないログ出力を提供する
- RPi 側で再利用できる共通フレーム（COBS + CRC）でログを送る
- mc_proto の LOG(0x10) に統合して、仕様の単一正を保つ

## 設計方針

1. 制御経路の安定性最優先
   - ログは「捨てられても良い副経路」扱い
2. フレーム統一
   - LOG は mc_proto v1 の `Type::LOG(0x10)` を使用
   - UART 伝送は COBS + CRC16 で再同期性を確保
3. 実装はシンプルに
   - `AsyncLogger -> UartTx` の単一経路でベストエフォート

## 実装コンポーネント

- `firmware/src/log/AsyncLogger.*`
  - `LogLevel` とメッセージ文字列を LOG フレームへ変換
  - 送信キュー満杯時は drop（`dropped()` でカウント）
- `firmware/src/comm/UartTx.*`
  - 送信専用キュー（低優先度タスク）
- `firmware/src/comm/mc_proto.*`
  - PacketWriter / COBS / CRC16

## データフロー

```
MC_LOGI(...) -> AsyncLogger -> UartTx -> COBS+CRC frame -> UART
```

## LOG プロトコル仕様（mc_proto v1）

フレーミングは `docs/proto/log_transport.md` を参照。
ここでは LOG の payload 仕様のみ記す。

```
type = 0x10 (LOG)
payload:
  [0]   level : u8
  [1..] text  : UTF-8 bytes (NUL終端なし)
```

- `level` は `LogLevel` の 0..5 を使用
- `text` は `tag + ": " + msg` 形式（`AsyncLogger` 実装）

## 制約・注意点（現行実装に基づく）

- payload の最大長は `mc::proto::MAX_PAYLOAD`（64 bytes）
  - `text` は最大 63 bytes に切り詰められる
- 送信キューが詰まった場合は **drop** される（制御を止めない）
- LOG の送信はベストエフォートで、順序/完全性は保証しない

## 運用（推奨）

- 重要イベント（KILL/TTL_EXPIRED など）を優先し、DEBUG を抑制する
- 帯域を占有しないよう **ログ量を制限** する
  - 例: 1 秒あたりの上限 bytes / 1 ループあたりの上限件数
  - 上限は `AsyncLogger` 側で制御（将来の拡張ポイント）

## 参考

- `shared/proto/mc_proto.h`（wire 上の正）
- `docs/proto/log_transport.md`（フレーミングと UART 運用）

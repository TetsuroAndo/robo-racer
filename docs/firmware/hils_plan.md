# HILS 設計ドキュメント（Plan）

このドキュメントは、ESP32ファームウェアにおけるHILS
（Hardware-in-the-Loop Simulation）機能の設計思想、構成、
データフロー、APIをまとめた計画書です。

## 目的

- アクチュエータ出力を安全に切り替え可能にする
- 実機出力とHILS出力を同時/排他的に扱えるようにする
- 低遅延ループを壊さない軽量な観測経路を提供する
- 将来の疑似センサ注入（入力側HILS）を見据えた拡張性を確保する

## 適用範囲 / 非目的

適用範囲:

- ESP32側のアクチュエータ出力を観測/再生可能にする
- 実機制御の「最終出力」への観測点を提供する

非目的:

- 現時点では疑似センサ入力を注入する機能は持たない
- リアルタイム同期や閉ループHILSの完全再現は対象外

## 設計方針

1. 出力経路の切り替えで実現する
   - `OutputMode` によりHardware/Shadow/Hilsを統一制御
2. HILS出力はSinkで拡張可能にする
   - `hils::Sink` インターフェースで出力先を増やせる
3. 制御ループと疎結合にする
   - HILS出力は副経路として扱い、停止しても制御に影響しない

## 用語

- HILS: 実機の制御出力を外部に送出し、シミュレーションと接続する枠組み
- Hardware: 実機アクチュエータに直接出力する通常運用
- Shadow: 実機出力に加え、HILS出力も同時に送る
- Hils: 実機出力を停止し、HILS出力のみ送る

## システム全体における位置づけ

```
App -> ActuatorBus -> (MotorDriver / ServoSteering)
                     -> (HILS Sink: Serial/BLEなど)
```

- HILSはアクチュエータ出力直前に挿入される
- 出力値は安全処理やスケーリング後の最終指令値
- hostタイムアウト時の「停止値」もHILSに出力される

## コンポーネント構成

- `firmware/src/control/ActuatorBus.h`
- `firmware/src/control/ActuatorBus.cpp`
  - `OutputMode` と HILS送出
- `firmware/src/hils/Hils.h`
- `firmware/src/hils/Hils.cpp`
  - `hils::Sink`, `hils::Mux`, `hils::StreamSink`
- `firmware/src/config/Config.h`
  - `OUTPUT_MODE`, `HILS_REPORT_TO_SERIAL`, `HILS_REPORT_TO_BLE`

## データフロー設計

```
ActuatorBus::apply -> OutputMode分岐
  Hardware: 実機に出力のみ
  Shadow: 実機出力 + HILS Sinkへ送出
  Hils: 実機出力なし、HILS Sinkへ送出のみ
```

## 初期化 / 設定フロー

1. `App::begin()` で `ActuatorBus::begin()` を呼び実機初期化
2. `App::configureHils()` で `OutputMode` とHILS Sinkを設定
3. `ActuatorBus::apply()` がループ内で毎回呼ばれる

注意:

- `OutputMode` は `Config.h` の静的定数で設定する
- 変更はビルド/書き込みが必要

## データ仕様

### ActuatorCommand

```
now_ms: millis() の値（単調増加、約49日でラップ）
throttle: 最終指令値（スケーリング後）
steer: 最終指令値（スケーリング後）
```

スケーリング例:

- `throttle` は `THROTTLE_MAX` / `THROTTLE_REVERSE` を反映した値
- `steer` は `STEER_MAX` を反映した値

つまり、HILSで取得する値は「入力（-1..1）」ではなく
「実機へ出す正規化済み指令」です。

## タイミング / 性能

- 出力頻度はメインループに依存（おおむね数百Hz）
- HILS Sinkは同期呼び出しのため、重い処理は避ける
- 出力の取りこぼしは許容（制御優先）

## 安全設計 / フェイルセーフ

- `ActuatorBus::begin()` でモータ停止・サーボセンターに初期化される
- `OutputMode::Hils` は実機出力を無効化する
- hostタイムアウト時は停止値がHILSにも送出される
- HILS出力が止まっても制御ループは継続する

## API設計

### OutputMode

- `Hardware`: 実機に出力、HILS送出なし
- `Shadow`: 実機出力 + HILS送出
- `Hils`: 実機出力なし、HILS送出のみ

### ActuatorBus

- `setOutputMode(OutputMode mode)`
- `attachHilsSink(hils::Sink *sink)`
- `apply(float throttle, float steer, uint32_t now_ms)`

### hils::Sink

```
struct ActuatorCommand {
  uint32_t now_ms;
  float throttle;
  float steer;
};

class Sink {
 public:
  virtual void onActuatorCommand(const ActuatorCommand &cmd) = 0;
};
```

### hils::Mux

複数Sinkに送出するための集約クラス。
最大3つまで登録可能。

### 3.1 通信プロトコル

低遅延性と既存資産の再利用のため、`comm/mc_proto` (COBS + CRC16) を使用する。
新しいパケットタイプ `CMD_HILS_STATE (0x12)` を定義し、バイナリで転送する。

**パケット構造:**
`[Header] [Type=0x12] [Seq] [Len] [Payload] [CRC]`

**Payload (HilsStatePayload):**

| フィールド | 型 (Byte) | 説明 | スケーリング |
| :--- | :--- | :--- | :--- |
| timestamp | uint32 (4) | 内部時刻 | 1ms |
| throttle | int16 (2) | スロットル | Raw値 (-1000~1000等) |
| steer | int16 (2) | ステアリング | センチ度 (cdeg) |
| flags | uint8 (1) | 状態フラグ | Bitmask |

## 制約・注意点

- HILS出力は副経路として扱われるため、Sinkの停止は制御に影響しない
- Sink数は最大3
- `OutputMode::Hils` では実機出力が無効になる
- Shadow/Hilsは安全確認の上で使用する
- HILS出力はアクチュエータのPWM値ではなく正規化コマンド値

## 検証観点

- Shadowモードで実機挙動とHILS値の一致を確認
- Hilsモードで実機出力が止まることを確認
- HILS出力を有効にしてもループレイテンシが悪化しないことを確認

## 拡張の方向性

- HILS入力（疑似センサ値の注入）経路の追加
- バイナリHILS出力（COBS+CRC）による高頻度配信
- HILS再生用のタイムスタンプ同期機構
- 出力のレート制御（デシメーション）
- 収録データのフォーマット統一（ログ/テレメトリとの統合）

# HILS 運用ガイド（Guide）

このドキュメントは、HILSの設定・運用方法をまとめたガイドです。
運用時の注意点やトラブルシュートも含みます。

## 前提

- HILSは「アクチュエータ出力の観測経路」です
- `Config.h` の値を変更した場合は再ビルド/書き込みが必要です
- HILS出力は制御ループの同期呼び出しで発行されます

## 設定ファイル

設定は `firmware/src/config/Config.h` にあります。

### HILS設定

- `OUTPUT_MODE`
  - `0` = Hardware
  - `1` = Shadow
  - `2` = Hils
- `HILS_REPORT_TO_SERIAL`
- `HILS_REPORT_TO_BLE`

出力先の注意:

- BLE出力を使う場合は `BLE_ENABLED=true` が必要
- BLEは帯域が小さいため高頻度には向かない

## 出力値の意味

HILSで観測できる `throttle` と `steer` は、**最終指令値**です。

- `throttle`: `THROTTLE_MAX`/`THROTTLE_REVERSE` を反映済み
- `steer`: `STEER_MAX` を反映済み

入力（-1..1）の素値ではなく、実機に出す正規化済み値が出力されます。

## 運用パターン

### Shadowモード（実機+HILS）

実機を動かしながらHILS出力を同時に確認します。

```
OUTPUT_MODE = 1
HILS_REPORT_TO_SERIAL = true
```

手順:

1. 設定を変更しビルド/書き込み
2. シリアルモニタで `HILS` 行が出ることを確認
3. 実機動作とHILS出力が一致するか確認

### Hilsモード（実機出力なし）

実機出力を止め、HILS出力のみを確認します。

```
OUTPUT_MODE = 2
HILS_REPORT_TO_SERIAL = true
```

手順:

1. 設定を変更しビルド/書き込み
2. 実機が動かないことを確認
3. HILS出力のみが出ていることを確認

## 実運用フロー（詳細）

目的別に手順を分けて記載します。安全確認を優先してください。

### 事前準備

1. 配線確認
   - モータ/サーボが物理的に安全な状態か確認
   - Hilsモードの場合は車体が動かないことを確認
2. 出力経路の選択
   - Shadow: 実機+HILS観測
   - Hils: 実機停止+HILS観測
3. 出力先の選択
   - Serial: デバッグ向け
   - BLE: 近距離確認向け（帯域に注意）

### 設定（Config.h）

例: Shadow + Serial出力

```
OUTPUT_MODE = 1
HILS_REPORT_TO_SERIAL = true
HILS_REPORT_TO_BLE = false
```

例: Hils + Serial出力

```
OUTPUT_MODE = 2
HILS_REPORT_TO_SERIAL = true
HILS_REPORT_TO_BLE = false
```

### ビルド/書き込み

1. `Config.h` を更新
2. ファームウェアをビルド
3. ESP32へ書き込み

注意:

- 書き込み後に電源再投入が必要な場合がある

### シリアル接続とログ取得

1. シリアルモニタを起動
2. `HILS` 行が出ることを確認

保存例（外部ツールで保存）:

```
HILS 1234 0.120 -0.045
```

### Shadowモードの検証

1. 安全な環境で最小入力から開始
2. 実機の挙動とHILS値が一致するか確認
3. 必要に応じて記録を停止/再開

### Hilsモードの検証

1. 実機が動かないことを確認
2. HILS出力が継続して出ることを確認
3. 記録データを解析し、指令が想定通りか確認

### 解析・再生の基本

1. `HILS` 行を抽出
2. `now_ms` を基準に並べ、時系列として扱う
3. `throttle` と `steer` をグラフ化して挙動を確認

推奨:

- 高頻度運用は出力先を軽量にする（Serialログを抑制）
- 記録ファイルに実験条件（設定値）を併記する

## 出力フォーマット

### StreamSink

```
HILS <ms> <throttle> <steer>
```

例:

```
HILS 1234 0.120 -0.045
```

## 記録と解析

- `HILS ` プレフィックスで行を抽出して保存する
- `now_ms` を基準に時系列で並べ、後処理で再生する
- 高頻度化が必要ならバイナリ出力の追加を検討する

## ログとの併用

- HILS出力はテキストのため、LOGのテキスト出力と混在する
- 解析しやすくするには `HILS` 行だけをフィルタする
- 解析の精度を重視する場合はLOGテキストを減らす

## 運用上の注意点

- `OutputMode::Hils` は実機出力が無効になる
- HILS出力は副経路なので、Sink停止は制御に影響しない
- Sinkは最大3つまで
- 重い処理をHILS受信側に入れるとループに影響する
- 同期が必要なら時刻のずれを補正する仕組みを追加する

## 運用チェックリスト

- `OUTPUT_MODE` が意図通りか
- `HILS_REPORT_TO_SERIAL`/`HILS_REPORT_TO_BLE` が有効か
- シリアル/BLEが接続済みか
- 実機を動かす場合は安全確認済みか

## トラブルシュート

- HILSが出力されない
  - `OUTPUT_MODE` と `HILS_REPORT_TO_*` を再確認
  - Serial/BLEが初期化されているか確認
  - HILS出力を受け取る側が接続済みか確認
  - ログ出力で埋もれていないか確認
  - ボーレート/接続先ポートが正しいか確認

- 実機が動いてしまう
  - `OUTPUT_MODE` が `2 (Hils)` になっているか確認
  - 書き込み後に電源再投入したか確認

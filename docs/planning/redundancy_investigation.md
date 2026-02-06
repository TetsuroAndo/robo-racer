# 冗長ロジック調査レポート

本ドキュメントは現状のコードベースから冗長・重複しているロジックを列挙し、調査結果をまとめたものです。

---

## 1. 速度・ステアリングの二重・三重クランプ

### 1.1 速度のクランプ

速度制限が複数箇所で重複して適用されている。

| 層 | ファイル | 処理内容 |
|----|----------|----------|
| RPi Process | `Process.cpp` | `v_min`〜`v_max`、`v_cap_input`（ブレーキ距離ベース）で制限 |
| RPi Sender | `Sender.cpp` | `clamp_speed_input()`: `[-SPEED_INPUT_LIMIT, +SPEED_INPUT_LIMIT]` でクランプ後、`speed_mm_s` に変換 |
| ESP32 DriveHandler | `DriveHandler.cpp` | `speed_mm_s` を `[-SPEED_MAX_MM_S, +SPEED_MAX_MM_S]` で再クランプ |
| ESP32 Tsd20Limiter | `Tsd20Limiter.cpp` | TSD20 距離に応じた速度上限 |
| ESP32 SpeedController | `SpeedController.cpp` | PI 制御出力の `pwm` を `[-255, 255]` でサチュレート、積分項クランプ |
| ESP32 Drive | `Drive.cpp` | `steerCdegToDeg_` 内で角度クランプ |

**冗長性**: RPi の `clamp_speed_input` と DriveHandler のクランプは同じ物理限界（`SPEED_MAX_MM_S`）を二重に適用している。防御的プログラミングとしては妥当だが、RPi 側で既にクランプ済みであるため、ESP32 側のクランプは「不正なペイロード」対策としての意味合いが強い。

### 1.2 ステアリング角度のクランプ

ステアリング角度のクランプが 4 箇所以上で行われている。

| 層 | ファイル | 処理内容 |
|----|----------|----------|
| RPi Process | `Process.cpp` | `target_angle_f` (L230-233)、`out_angle` (L244-247) を `STEER_ANGLE_MAX_DEG` でクランプ |
| RPi Sender | `Sender.cpp` | `clamp_cdeg()`: `[-STEER_ANGLE_MAX_CDEG, +STEER_ANGLE_MAX_CDEG]` |
| ESP32 DriveHandler | `DriveHandler.cpp` | `steer_cdeg` を `[STEER_ANGLE_MIN_CDEG, STEER_ANGLE_MAX_CDEG]` で再クランプ |
| ESP32 Drive | `Drive.cpp` | `steerCdegToDeg_()` 内で `mc::clamp` |
| ESP32 Steer | `Steer.cpp` | `setAngle()` 内で同様のクランプ |

**冗長性**: Process 内で 2 回（target → applied）、Sender で 1 回、DriveHandler で 1 回、Drive/Steer で 1 回と、同じ物理限界に対するクランプが 5 段階で重複している。

---

## 2. 時刻取得の重複実装

### 2.1 `now_ms()` の独自実装

- **Sender.cpp** (L15-21): 無名名前空間内で `clock_gettime(CLOCK_MONOTONIC)` を用いた `now_ms()` を独自実装
- **mc::core::Time**: `Time::ms()` が `rpi/lib/mc_core/src/Time.cpp` に存在

**冗長性**: Sender は `mc::core::Time::us()` を他で使用している一方、`now_ms()` は自前実装。`mc::core::Time::ms()` に統一可能。

---

## 3. ユーティリティ関数の重複

### 3.1 `ensure_dir_for()`

同一ロジックが 2 箇所に存在。

- `rpi/src/main.cpp` (L33-44)
- `rpi/apps/lidar_received/main.cpp` (L244-254)

**冗長性**: 共通ユーティリティ（例: `mc/core/FileUtils.hpp`）に抽出可能。

### 3.2 `make_run_id()`

- `rpi/src/main.cpp` (L27-31): `Time::us() + "-p" + getpid()` を含む
- `rpi/src/lidar_to_esp.cpp` (L21-25): `Time::us()` のみ（pid なし）

**冗長性**: 仕様の差（pid の有無）があるため、用途に応じて共通化または明示的な分岐が望ましい。

### 3.3 `show_cursor()` / `on_sig()`

- `rpi/src/main.cpp`: `show_cursor`, `on_sig` を定義
- `rpi/src/lidar_to_esp.cpp`: 同様の `show_cursor`, `on_sig` を再定義

**冗長性**: 共通のシグナル・終了処理モジュールにまとめられる。

### 3.4 クランプ関数の重複

- **Sender.cpp**: `clamp_speed_input`, `clamp_cdeg` を手書き
- **firmware/lib/common/Math.h**: `mc::clamp<T>(v, lo, hi)` が既に存在

**冗長性**: RPi 側でも `mc::clamp` 相当の共通ユーティリティを利用可能（現状 RPi は `mc_config` のみ参照）。

---

## 4. LiDAR 受信の二重パス

### 4.1 LidarReceiver vs ShmLidarReceiver

| クラス | データソース | 使用箇所 |
|--------|--------------|----------|
| `LidarReceiver` | 物理 LiDAR (シリアル) | `lidar_to_esp.cpp` の `run_lidar_to_esp()` |
| `ShmLidarReceiver` | 共有メモリ (`/lidar_scan`) | `main.cpp` のメインループ |

**冗長性**: 両者とも `getLatestData(std::vector<LidarData>&)` を提供し、`LidarData` を返す。インターフェースは共通化されているが、データソースが異なるため、アーキテクチャ上の選択（物理直結 vs 共有メモリ経由）であり、ロジックの重複ではない。

### 4.2 `lidar_to_esp` の未使用・死コード

- **main.cpp**: `#include "lidar_to_esp.h"` しているが、`run_lidar_to_esp()` は**呼ばれていない**
- **lidar_received アプリ**: `lidar_to_esp.cpp` をリンクしているが、`main()` は自前の LiDAR スキャン＋共有メモリ書き込みのみ行い、`run_lidar_to_esp()` は**呼ばれていない**

**冗長性**: `run_lidar_to_esp` は現状デッドコード。レガシーまたは将来用のエントリポイントとして残っている可能性がある。`main.cpp` の `lidar_to_esp.h` インクルードは不要。

---

## 5. 予測マージン計算の類似ロジック

### 5.1 反応距離の計算

以下の 2 箇所で、`v * tau + 0.5 * a * tau^2` に近い式が使われている。

- **Process.cpp** (L69-73): `pred_margin_mm = v_est * tau + 0.5 * a_pos * tau^2`（FTG 用）
- **Tsd20Limiter.cpp** (L94-95): `d_travel = v_used * tau + 0.5 * a_cap * tau^2`（TSD20 用）

**冗長性**: 物理モデルは同じだが、パラメータ（`FTG_PREDICT_*` vs `TSD20_*`）が異なる。共通の「反応距離計算」関数にまとめることは可能だが、設定の分離を考えると現状のままでも許容範囲。

---

## 6. Telemetry 内のクランプ

### 6.1 角度→位置変換のクランプ

- **Telemetry.cpp** (L608-614): `posFromAngle` ラムダ内で `angle_deg` を `[-90, 90]` にクランプ

**冗長性**: 表示用の正規化であり、制御系のクランプとは目的が異なる。ただし、`std::clamp` や共通ヘルパーで置き換え可能。

---

## 7. 推奨アクション（優先度順）

| 優先度 | 項目 | アクション | 状態 |
|--------|------|------------|------|
| 高 | 未使用インクルード・デッドコード | `main.cpp` から `lidar_to_esp.h` を削除。`lidar_received` で `run_lidar_to_esp` が使われていないことを確認し、必要なら `lidar_to_esp.cpp` をビルドから外す | ✅ 完了 |
| 高 | `now_ms` の統一 | Sender の `now_ms()` を `mc::core::Time::ms()` に置き換え | ✅ 完了 |
| 中 | `ensure_dir_for` | 共通ユーティリティに抽出し、main / lidar_received から参照 | ✅ 完了 |
| 中 | シグナル・終了処理 | `show_cursor`, `on_sig` を共通モジュールに集約 | ✅ 完了 |
| 低 | ステアリングクランプの整理 | Process 内の重複クランプ（target と out_angle）を 1 回に集約することを検討 | ✅ 完了 |
| 低 | 速度クランプ | 防御的クランプは維持しつつ、RPi と ESP32 の責務をドキュメント化 | 未実施 |

---

## 8. まとめ

- **速度・ステアのクランプ**: 複数層で重複しているが、防御的プログラミングとして妥当。整理するなら Process 内の重複削減から。
- **時刻・ユーティリティ**: `now_ms`, `ensure_dir_for`, `show_cursor` 等は共通化の余地あり。
- **lidar_to_esp**: 現状デッドコード。アーキテクチャ方針に応じて削除または再利用を検討。

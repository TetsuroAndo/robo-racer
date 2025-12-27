# Firmware Plan (ESP32) — TOYOTA42自動運転ミニカーバトル

## 目的とスコープ

このファームウェアは **ESP32単体で走行可能**であることを最優先とし、RPi（高レベル制御）を後から接続しても破綻しないよう、次を提供します。

- **TSD20（1D LiDAR）** を用いた前方距離計測に基づく「減速・停止・回避」
- **MPU-6050（ジャイロ）** による **ヨー角（相対）** 推定を使った「直進保持」「右回り／左回りの回避戦略」
- **ステート管理**（Idle / Manual / Auto / Estop）と安全系（ホスト監視、緊急停止）
- **RPiとの通信プロトコル**（実装はESP32側のみ。RPi側はインターフェース定義）
- 今後の2D LiDAR（RPi）・経路計画・SLAM等の拡張を前提とした **クラス分割** と **ディレクトリ設計**

本フェーズでは「最速で走る」ではなく **確実に“安全に走る”** を優先します。

---

## 現状の最小機能（本実装が提供するもの）

### 走行（Auto）
- 通常は前進（速度はConfigで制限）
- TSD20距離が閾値を下回ると減速→停止→後退→旋回→再前進
- 旋回方向は **preferRight（右優先/左優先）** で決定
- ジャイロのヨー角変化量で **旋回完了角（例: 55°）** を判定

### 直進保持（Yaw P制御）
- Cruise中に基準方位（headingRef）を固定
- 方位誤差を `steer += Kp * yaw_error` で補正

### 安全系
- EstopがActiveなら常にスロットル0、操舵センター
- Host（RPi）からManual制御中、ハートビートが途絶えたら停止
- 1D距離が非常に近い場合は、最終段で必ずスロットル0へクランプ

---

## ディレクトリ設計（推奨方針）

PlatformIO/Arduinoの慣例に合わせます。

```
firmware/
  src/
    main.cpp
    app/         … 依存を束ねるアプリ層（App）
    comm/        … RPi/ホストと通信（Comm + protocol）
    config/      … ピン・閾値・制御ゲイン（Config.h）
    control/     … 状態・戦略・制御（State/Strategy/MotionController）
    hardware/    … デバイスドライバ（Tsd20/Imu/MotorDriver/ServoSteering）
  lib/
    common/      … 本プロジェクト共通の小物（SlewRateLimiter, clamp等）
    external/    … 外部ライブラリをsubmodule/subtreeで固定する場合の置き場
docs/
  firmware/plan.md
```

PlatformIOは `lib_dir` / `lib_extra_dirs` を使ってローカルライブラリを優先的に探索できます。 citeturn16search1turn16search7  
依存は `lib_deps` に宣言して自動インストールするのが標準運用です。 citeturn16search0turn16search12

---

## 主要クラス（責務の分離）

### app/App
- 全サブシステムを所有し、`begin()/loop()` のみを公開
- センサ更新→通信反映→状態機械→アクチュエータ出力→テレメトリ送信の順に実行

### hardware/*
- `MotorDriver`（IBT-2 / BTS7960）: PWMで前後回転を切替
- `ServoSteering`: LEDC 50Hz PWMでサーボ角（正規化 -1..1）を出力
- `Tsd20`: I2Cで距離(0x00/0x01)を読む（アドレス0x52）
- `Imu`: MPU6050_lightを使い `gyroZ` の積分で相対Yawを算出（バイアス推定あり）

### control/*
- `RuntimeState`: mode, estop, preferRight, headingRef, maneuver記録など
- `Strategy`: Auto時の挙動（Cruise/Brake/Reverse/Turnのサブステート）
- `MotionController`: 目標指令に対してスロットルのスルーレート制限と方位保持を適用

### comm/*
- `proto::Header` + payload のバイナリプロトコル（COBSでフレーミング）
- `Comm`: 受信パケットをコマンド（HostCommand）に変換し、テレメトリを送信

PacketSerialは **COBS/SLIP** によるパケット化を提供し、ペイロードに0x00等が含まれても扱えます。 citeturn12search0turn12search4  
CRC16はAceCRCなどを利用し、フレーム破損検出を行います。 citeturn13search3

---

## 右回り/左回り戦略（戦術レイヤ）

本実装の「回避」は **右優先（右手法）/左優先（左手法）** を単純に切替可能にしています。

- `preferRight = true` → 右に最大操舵して旋回（前進しながら）
- `preferRight = false` → 左に最大操舵して旋回

RPi側が将来、コース形状やLiDAR（2D）で「右へ行くべき/左へ行くべき」を判断して `SetPrefer` を送れば、ESP32はその方針に従って回避します。

---

## ジャイロ処理（Yaw推定・補正方針）

MPU-6050は磁気センサを持たないため、ヨー角は **必ずドリフト** します。  
ただし「短時間の直進保持」「一定角の旋回完了判定」には十分有効です。

- `gyroZ`（deg/s）を積分して `yawDeg` を更新
- 車体静止とみなせる期間に `gyroZ` バイアスを平均して補正
- `yawDeg` は [-180, 180) にラップ

MPU6050_lightは `getGyroZ()` や `getAngleZ()` などを提供し、実装コストを大幅に下げます。 citeturn10search6turn10search15  
より姿勢推定を強化する場合、MadgwickAHRS（加速度+ジャイロのAHRS）でロール/ピッチ安定化を行い、ヨーは相対角として扱う方針が現実的です。 citeturn14view2

---

## RPi通信プロトコル（インターフェース仕様）

### 前提
- 物理層: UART（RPi <-> ESP32）
- フレーミング: PacketSerial（COBS）
- 破損検出: CRC16（CCITT系、片側だけ変えない）

### メッセージ種別（例）
- `Heartbeat`: RPiが周期送信（ESP32はウォッチドッグに使用）
- `SetMode`: Idle/Manual/Auto
- `SetManual`: スロットル/操舵（-1000..1000）
- `Estop` / `ClearEstop`
- `Status`: ESP32→RPiのテレメトリ（距離、Yaw、指令値、ループHzなど）

プロトコルの定義は `firmware/src/comm/protocol/Protocol.h` に集約しています。

---

## 外部ライブラリ（本プロジェクトで“使える”候補）

### 通信
- PacketSerial（COBS/SLIPフレーミング） citeturn12search0turn12search4
- CRC: AceCRC（crc16ccittなど） citeturn13search3
- JSON（将来の設定配布/デバッグ用途）: ArduinoJson（MIT） citeturn15search2

### IMU/フィルタ
- MPU6050_light（簡易・軽量） citeturn10search6
- I2Cdevlib（より低レベル/柔軟、DMP等も視野） citeturn2search8turn4search0
- MadgwickAHRS（AHRSアルゴリズムのラッパ） citeturn14view2

### 組込みC++の共通化
- Embedded Template Library（ETL, MIT）: 組込み向けコンテナ・アルゴリズム citeturn14view1

### サーボ（補足）
ESP32Servo系ライブラリは複数系統があり、Arduino-ESP32のバージョン追従の差があります。  
互換性が必要なら、Arduino-ESP32 v3系を明示しているESP32Servoフォークの利用を検討してください。 citeturn15search16turn15search1  
本実装は依存を減らすため、LEDCでサーボPWMを直接生成しています。

---

## 今後の拡張（次ステップ）

1. **速度推定（車輪エンコーダ）** の追加  
   直進保持は yaw のみだと速度変動に弱いので、速度ループ（PI）と組み合わせる

2. **TSD20の複数搭載**  
   前方だけでなく左右も持てば「壁沿い走行」「ライン復帰」がやりやすい

3. **RPi側のプロトコル実装**  
   Python/C++で同一ヘッダ仕様（payload構造体）を再現し、COBS+CRCで相互運用

4. 2D LiDAR/RPi側SLAMに合わせた、ESP32側の役割固定  
   - ESP32: 低遅延のモータ/サーボ制御、安全停止、基礎センサ集約
   - RPi: 認識・地図・経路計画・戦術（右/左）決定・速度目標

---

## ビルド・導入メモ

- `firmware/PLATFORMIO_SNIPPET.ini` を参考に、プロジェクトの `platformio.ini` に `lib_deps` と `lib_extra_dirs` を追記してください。 citeturn16search0turn16search1
- 外部ライブラリを `firmware/lib/external/` に固定する場合は、PlatformIOのローカルライブラリ探索に乗せる（`lib_extra_dirs`）のが扱いやすいです。 citeturn16search1turn16search7

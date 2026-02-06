# IMU補完 + ESP32安全封筒 + RPi操舵強化 計画（熟考版）

目的は **「RPi(10Hz LiDAR)の遅さを IMU で埋めて、ESP32 側は TSD20 + IMU で“止まれる速度上限”を常時保証しつつ、RPi はステアリングを攻める」** を成立させること。

本計画は **InvenSense MPU-6000/MPU-6050 Product Specification (2013-08-19)** を参照可能な前提のもと、MPU-6500(GY-521) の実装/統合、TSD20 と DS3218 と IBT-2 の仕様反映、速度ガバナと逆転ABSを含む「高速・攻めた速度・安全停止」を両立させる設計と実装順序をまとめる。

---

## 0. 結論（責務分離を成立させる要件）

ESP32 を Safety Envelope として成立させるために、最低限以下が必要。

1. **実速度の代替**  
エンコーダ無しのため、ESP32 が IMU から **短時間の速度推定 v_est** を持つ（ドリフトは前提で抑え込む）。
2. **オンライン制動能力推定**  
IMU から **実減速度 a_brake_max** を学習し、路面・電圧変動に追従する。
3. **距離→速度上限**  
TSD20 距離 d から **停止可能速度 v_max** を毎周期計算し、`min(target, v_max)` を強制。
4. **逆転ABSブレーキ**  
断続逆転を IMU で閉ループ制御し、一定減速度へ寄せる。
5. **RPi への要約フィードバック**  
RPi の FTG が遅延補完できるよう **v_est / a_long / brake_state** を返す。

---

## 1. ハード構成（I2Cピン／バス分割）

### 1.1 GY-521(MPU-6500)配線

- GPIO22 = SCL
- GPIO21 = SDA
- 3V3 / GND

### 1.2 I2Cバス分離（必須）

既存 TSD20 は `Wire(GPIO32/33)` を使用し、初期化で `Wire.end()` -> `Wire.begin(...)` を呼ぶ。  
このため **IMU は別バス**にし、IMU には `TwoWire imuWire(1)` を使用する。

- TSD20: `Wire`（GPIO32/33）
- MPU-6500: `Wire1`（GPIO21/22）

### 1.3 I2Cクロック

- TSD20: 仕様上 400kHz、現状 100kHz  
  → **400kHz に引き上げる**
- MPU-6500: 400kHz で十分

---

## 2. センサ仕様の反映（TSD20 / サーボ / IBT-2）

### 2.1 TSD20

ユーザー提供の仕様:

- 周波数: 200/100/50/20/10/1 Hz
- 設定式: `f = 10000 / (div + 1)`
- Frequency setting コマンド: `0x0B`
- I2C: 最大 400kHz、アドレス 0x52

現状コードとの差分:

- 現在: `TSD20_I2C_HZ=100000`、`TSD20_READ_INTERVAL_MS=50`（20Hz）
- 目標: **100Hz (10ms)** から開始し、安定すれば **200Hz (5ms)** へ

### 2.2 DS3218 サーボ（6.8V）

DS3218 datasheet (6.8V) より:

- 速度: **0.14 sec / 60°**
- 受信周波数: **50–330Hz**

計算:

- 最大角速度 `w_steer_max = 60° / 0.14s ≈ 428.6 deg/s`
- 20° 移動の下限時間 `t_steer_min ≈ 46.7 ms`

この `t_steer` を FTG のコストに入れて「曲がれるまでの遅延」を明示する。

### 2.3 IBT-2（BTS7960）

仕様の最大 PWM は **25kHz**。  
キャリア PWM は **18〜20kHz** を推奨（可聴域外・応答も十分）。

逆転ABSの断続周期は **PWMキャリアとは別**に制御する。

---

## 3. 目標アーキテクチャ（制御ループ二層）

ESP32 側のループ分離:

- Fast loop (200–500Hz): IMU読み、姿勢推定、重力除去、a_long算出
- Mid loop (100–200Hz): TSD20読取、v_max計算、速度クランプ
- Brake mod loop (20–100Hz): 逆転ABS断続 + IMU減速度フィードバック

RPi 側:

- 10Hz LiDAR を IMUで補完し、操舵を 20–50Hz 相当に引き上げる

---

## 4. MPU-6500 実装プラン（ESP32）

### 4.1 最小ドライバ

MPU-6000 datasheet (2013-08-19) のレジスタ手順を踏襲。

- WHO_AM_I で検出（MPU-6500/6050混在を許容）
- Gyro: ±2000 dps
- Accel: ±4g もしくは ±8g
- DLPF: 42〜98Hz帯から開始
- 内部 1kHz サンプル -> ソフトで 200–500Hz に間引き

### 4.2 軸定義を Config 化

基板向きに依存しないため:

- `axis_map` / `axis_sign` を Config で持つ
- 実装側で吸収

### 4.3 キャリブレーション

起動時:

- 1〜2秒静止平均で `ax_bias/ay_bias/gz_bias` を求める

走行中:

- ZUPT条件（停止判定）で v_est を 0 へ寄せる

---

## 5. v_est（短時間速度推定）戦略

目的は **停止距離計算に必要な数秒レンジの速度推定**。

推定器:

```text
a_long = ax - ax_bias
v_est = clamp(v_est + a_long*dt, 0, v_cmd_max)
```

ドリフト抑制:

- ZUPTで v_est を 0 リセット
- v_est にリーク（ゆっくり 0 に戻す）
- オプション: 前方障害物が安定しているときに `v_front ≒ -d_dot` を弱く融合

---

## 6. 速度ガバナ（TSD20距離→v_max）

### 6.1 停止距離モデル

```text
d_need = v * t_latency + v^2 / (2*a_brake)
d_allow = d_meas - d_margin
v_max = solve(d_allow >= d_need)
```

`a_brake` は固定値ではなく IMU から **オンライン更新**する。

### 6.2 マージン

TSD20 はバンパー先端から **約50mm後方**。

最低条件:

- `d_margin >= 50mm + 測距誤差 + 姿勢誤差`

初期は **120〜200mm** から開始が安全。

---

## 7. 逆転ABSブレーキ（断続周期の計算）

### 7.1 キャリアPWM

- IBT-2のキャリアPWM: **18〜20kHz**

### 7.2 ABS断続周期の初期値

RCレベルで IMU から減速度が「見える」時間スケールと、
現在のスルーレート制限を考慮して **20ms** を初期推奨。

根拠（計算）:

- IMU 200Hz → 1サンプル 5ms。減速度推定に 4サンプル必要なら 20ms。
- 現状の `rate_down=1200/s` だと PWM 50 変化に 41.7ms 必要。
  → ブレーキ時だけ `rate_down` を 4000/s へ引き上げれば 12.5ms で到達。

初期値:

- **T_abs = 20ms（50Hz）**
- duty = 30〜50% から開始
- IMU で減速度を見て PI 制御で duty 調整

調整指針:

- ブレーキがガクガクする → T_abs を 30〜50ms
- 停止距離が伸びる → duty 増、または T_abs を 10〜20ms
- 発熱が厳しい → duty 減、T_abs を長く

---

## 8. RPi(FTG)側のコスト改善

### 8.1 LiDAR遅延コスト

```text
t_lidar_age ≈ 0〜100ms + 処理時間
d_age = v * t_lidar_age
```

左右クリアランスから `d_age` を差し引き、保守的に評価する。

### 8.2 サーボ追従遅延コスト

```text
t_steer = |Δdeg| / 428.6(deg/s)
t_steer *= k_load (1.3〜1.8)
d_steer_delay = v * t_steer
```

FTG のコストに `d_steer_delay` を入れる。

---

## 9. ESP32 ↔ RPi プロトコル

ESP32 → RPi（推奨最小セット）:

- `v_est_mm_s`（int16）
- `a_long_mm_s2`（int16）
- `a_brake_max_mm_s2`（uint16）
- `brake_state`（bitfield）

RPi → ESP32:

- 既存 DRIVE を継続
- `speed_mm_s` は攻めた値でOK、最終クランプは ESP32

---

## 10. 実装ステップ（現実的な順番）

### Phase 1: 計測が出るまで

1. IMUドライバ（Wire1）実装
2. 軸向き検証（静止で z≈+1g、加速で x正）
3. gyro bias 取得

### Phase 2: 速度推定

4. roll/pitch 推定 -> 重力除去
5. v_est と ZUPT
6. ブレーキ実験モードで a_brake_max 推定

### Phase 3: 安全ガバナ

7. TSD20 を 400kHz / 100Hz 以上へ
8. v_max(d) に置換
9. 逆転ABS（T_abs=20ms開始）

### Phase 4: RPi補間

10. ESP32 → RPi のテレメトリ追加
11. LiDAR補間 + FTGコスト改造

---

## 11. すぐ入れるべき設定変更（現コード）

- `TSD20_I2C_HZ: 100000 -> 400000`
- `TSD20_READ_INTERVAL_MS: 50 -> 10`
- 速度クランプ: 線形縮退 -> `v_max(d)` へ段階移行

---

## 12. フェーズ別タスクリスト（ファイル単位）

### Phase 1: 計測が出るまで（IMU Bring-up）

- `firmware/src/config/Config.h`  
  IMU定数（I2Cピン/周波数/FS/DLPF/周期）を追加
- `firmware/src/hardware/Mpu6500.h`  
  IMUドライバのIF定義
- `firmware/src/hardware/Mpu6500.cpp`  
  WHO_AM_I、レジスタ設定、14byte read 実装
- `firmware/src/main.cpp`  
  IMU初期化、周期読取、ログ出力

### Phase 2: 速度推定（v_est + ZUPT）

- `firmware/src/config/Config.h`  
  IMUキャリブレーション/しきい値/ZUPT設定を追加
- `firmware/src/hardware/ImuEstimator.h`  
  バイアス推定/重力除去/速度推定IF
- `firmware/src/hardware/ImuEstimator.cpp`  
  v_est・ZUPT・リーク実装
- `firmware/src/main.cpp`  
  ImuEstimator呼び出しとログ

### Phase 3: 安全ガバナ（v_max(d)）

- `firmware/src/config/Config.h`  
  `d_margin`, `tau_total`, `a_brake_init` など追加
- `firmware/src/main.cpp` or `firmware/src/hardware/SpeedLimiter.*`  
  `v_max(d)` 計算・速度クランプ
- `firmware/src/hardware/Tsd20.*`  
  周波数設定コマンド対応（0x0B）

### Phase 4: 逆転ABSブレーキ

- `firmware/src/config/Config.h`  
  `T_abs`, duty上限, brake rate_down 設定追加
- `firmware/src/hardware/Engine.*`  
  ブレーキモード時の rate_down 強化/デッドタイム
- `firmware/src/main.cpp`  
  ABS制御ループ

### Phase 5: RPi 補間操舵

- `rpi/src/Telemetry.*`  
  ESP32→RPi IMUテレメトリ受信追加
- `rpi/src/Process.cpp`  
  `t_lidar_age` / `t_steer` コストの導入
- `rpi/src/config/Config.h`  
  サーボ速度/コスト係数追加

---

## 13. 確認事項

- IMU の WHO_AM_I 実測値
- IMU 取り付け向き（axis_map 設計）
- `tau_total` 実測（TSD20 + 制御 + モータ応答）
- 逆転ABSの初期 duty と rate_down（ブレーキ時のみ強化するか）

# IMU安全封筒 実装ステータス

## 目的
- IMU補完と安全封筒（TSD20 + IMU）で停止可能速度を保証し、RPi側の操舵を強化する。

## 状態
- 完了: IMU軸マップ/重力除去、IMU静止キャリブレーション、brake_cap学習の目標速度整合、TSD20 UART順序（0x0B→0x1F）と周期ログ、v_max速度クランプ、逆転ABSの弱化/近距離逆転禁止、Hブリッジ方向切替デッドタイム、IMU_STATUS送信/受信、FTGサーボ遅延コスト
- 未着手: LiDAR遅延コストの導入

## タスク一覧
- [x] 関連ドキュメント確認（`docs/planning/imu_safety_plan.md` ほか）
- [x] IMUドライバ（MPU-6500）と別I2Cバス（Wire1）
- [x] IMU推定器（バイアス/ZUPT/v_est/a_brake_cap）
- [x] IMU軸マップ（axis_map）対応
- [x] 重力除去（roll/pitch相当の補正）
- [x] IMU静止キャリブレーション（静止条件サンプルのみ）
- [x] TSD20 400kHz/100Hz設定と周波数コマンド対応
- [x] TSD20 UART順序（0x0B→0x1F）と実測周期ログ
- [x] v_max(d) による速度クランプ
- [x] 逆転ABSブレーキ + rate_down強化 + 逆転抑制
- [x] brake_cap学習の目標速度整合（最終適用speedを使用）
- [x] 方向切替のデッドタイム追加
- [x] ESP32 -> RPi の IMU_STATUS テレメトリ
- [x] RPi側 IMU_STATUS 受信
- [x] FTG サーボ遅延コスト導入
- [ ] LiDAR遅延コストの導入

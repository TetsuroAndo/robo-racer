# クリティカル項目修正の精査結果

## 修正完了項目

### ✅ 1. StatusPayload ワイヤフォーマット修正
- **修正ファイル**: 5ファイル
  - `rpi/ros2_ws/src/mc_bridge/mc_bridge/mc_proto_codec.py` ✅
  - `rpi/apps/seriald/seriald_client.py` ✅
  - `tools/hils/run_local_e2e.py` ✅
  - `playground/steer_limit_test.py` ✅
  - `test/rpi/seriald/test_seriald_integration.py` ✅

- **確認**: すべて `<BBHhhHBBBB` (14バイト) に更新済み

### ✅ 2. IMU bit 2 セマンティック修正
- **修正ファイル**: 6ファイル
  - `rpi/src/MotionState.h` ✅
  - `rpi/src/Sender.cpp` ✅
  - `rpi/src/Telemetry.h` ✅
  - `rpi/src/Telemetry.cpp` ✅
  - `rpi/apps/seriald/src/main.cpp` ✅
  - `playground/decel_lab/run_decel_lab.py` ✅

- **確認**: すべて `abs_active` → `brake_mode` にリネーム済み

### ✅ 3. Telemetry表示の単位ミス修正
- **修正ファイル**: 1ファイル
  - `rpi/src/Telemetry.cpp` ✅

- **確認**: `stop_req` から "ms" を削除済み

### ✅ 4. BrakeController 初期化修正
- **修正ファイル**: 1ファイル
  - `firmware/src/control/BrakeController.cpp` ✅

- **確認**: `stop_requested` 検出時に `reset_()` を呼ぶように修正済み
  - `reset_()` が `_stop_since_ms` を0にリセットし、その後 `if (_stop_since_ms == 0)` でチェックするため、正しい動作

---

## 🔴 修正が必要な項目

### 1. ROS2 Bridge のアンパック不整合

**問題**: `bridge_node.py` で `decode_status` が10個の値を返すが、6個しか受け取っていない

**ファイル**: `rpi/ros2_ws/src/mc_bridge/mc_bridge/bridge_node.py` (行449-456)

**現状**:
```python
(
    msg.seq_applied,
    msg.auto_active,
    msg.faults,
    msg.speed_mm_s,
    msg.steer_cdeg,
    msg.age_ms,
) = dec  # dec は10個の値
```

**対応方針**:
- オプション1: 追加フィールドを無視（推奨）
  ```python
  (
      msg.seq_applied,
      msg.auto_active,
      msg.faults,
      msg.speed_mm_s,
      msg.steer_cdeg,
      msg.age_ms,
      _,  # applied_brake_duty
      _,  # stop_level
      _,  # stop_requested
      _,  # reserved
  ) = dec
  ```
- オプション2: ROS2メッセージ定義を拡張（将来対応）

### 2. seriald ログ出力の不完全性（低優先度）

**問題**: `seriald/src/main.cpp` のログ出力が10バイト分しか読み取っていない

**ファイル**: `rpi/apps/seriald/src/main.cpp` (行431-450)

**現状**: ログ出力は動作上問題ないが、新しいフィールド（`applied_brake_duty`, `stop_level`, `stop_requested`）をログに含めていない

**対応方針**:
- オプション1: ログ出力を拡張（推奨）
  ```cpp
  uint8_t applied_brake_duty = p[10];
  uint8_t stop_level = p[11];
  uint8_t stop_requested = p[12];
  // ログに追加
  ```
- オプション2: 現状維持（動作上問題なし）

---

## 精査方法

### 1. プロトコル定義の確認
```bash
# StatusPayloadの定義を確認
grep -A 12 "struct StatusPayload" shared/proto/include/mc_proto.h
```

### 2. 使用箇所の確認
```bash
# decode_statusの使用箇所を確認
grep -r "decode_status" --include="*.py" --include="*.cpp"
```

### 3. 構造体サイズの確認
```bash
# C++側のstatic_assertを確認
grep "StatusPayload.*14" firmware/src/main.cpp
grep "StatusPayload.*14" shared/proto/include/mc_proto.h
```

### 4. テスト実行
```bash
# 統合テストを実行
make test
# または
uv run pytest test/rpi/seriald/test_seriald_integration.py -v
```

### 5. ビルド確認
```bash
# C++コードのコンパイル確認
make all
# または
cd rpi/apps/seriald && g++ -c src/main.cpp
```

---

## 推奨される次のステップ

1. **即座に修正**: ROS2 Bridge のアンパック不整合（必須）
2. **任意修正**: seriald ログ出力の拡張（低優先度）
3. **テスト**: 修正後の統合テスト実行
4. **ドキュメント更新**: 必要に応じてドキュメントを更新


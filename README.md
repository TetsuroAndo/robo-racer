
## Environment
- C++23
- Python 3.13

Set up the environment by executing the following:
```sh
make init
```

## File structure
```
./
├── .clang-format
├── .coderabbit.yaml
├── .github/
├── .gitignore
├── Makefile             # 全体の管理
├── pyproject.toml       # 依存管理 (実機用と学習用をグループ分け定義)
├── platformio.ini       # ESP32用設定
├── uv.lock
├── config/              # 共通設定 (車両設定など)
│   ├── vehicle.yaml          # 車両物理パラメータ (実機・学習で共有)
│   └── training_hparams.yaml # 学習ハイパーパラメータ
├── firmware/            # [ESP32] マイコン用コード (C++)
│   ├── src/
│   └── lib/
├── models/              # 学習済みモデル
│   ├── production/      # 実機で使う最新モデル (.tflite, .onnx)
│   └── archive/         # 過去のモデル
├── src/                 # [Pi 5] 実機ランタイム用コード (Python)
│   ├── main.py          # エントリーポイント
│   ├── common/          # 共有ライブラリ (通信定義など)
│   ├── control/         # 制御ロジック
│   ├── perception/      # 推論実行・カメラ処理
│   └── driver/          # ESP32通信ドライバ
└── training/            # [PC/Cloud] 学習・解析用ワークスペース
    ├── data/            # 学習データ (通常git除外)
    │   ├── raw/
    │   └── processed/
    ├── notebooks/       # 解析・試行錯誤用 (Jupyter)
    └── src/             # 学習パイプラインのソースコード
        ├── dataset.py   # データローダー
        ├── train.py     # 学習実行スクリプト
        └── augment.py   # データ拡張ロジック
```

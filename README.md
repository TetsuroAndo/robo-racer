
## Environment
- C++23
- Python >= 3.10

Set up the environment by executing the following:
```sh
make init
```

## File structure
```
./
├── .clang-format
├── .clang-tidy
├── .coderabbit.yaml
├── .github/
├── .gitignore
├── .gitmodules
├── Makefile             # 全体の管理
├── pyproject.toml       # 依存管理 (実機用と学習用をグループ分け定義)
├── platformio.ini       # ESP32用設定
├── uv.lock
├── compile_commands.json
├── robo-racer           # ホスト側バイナリ
├── firmware/            # [ESP32] マイコン用コード (C++)
│   ├── src/
│   ├── lib/
│   └── test/
├── rpi/                 # [Pi 5] 実機・学習用コード
│   ├── apps/            # 実機ユーティリティ
│   ├── build/           # ビルド成果物
│   ├── config/          # 共通設定
│   ├── lib/             # 共有ライブラリ
│   ├── models/          # 学習済みモデル
│   │   ├── production/  # 実機で使う最新モデル
│   │   └── archive/     # 過去のモデル
│   ├── src/             # 実機ランタイム
│   └── training/        # 学習・解析用ワークスペース
│       ├── data/        # 学習データ
│       ├── notebooks/   # 解析・試行錯誤用
│       └── src/         # 学習パイプラインのソースコード
├── test/                # テスト
├── docs/
├── tools/
├── playground/
├── cad/
└── logs/
```

# Test Design

## 目的
- すべてのテストは pytest で実行する。
- 仕様追加に合わせて分類ディレクトリを増やし、テストの居場所を明確にする。
- C++のプロトコル検証は pytest からビルドして実行する。

## 方針
- ルート配下は `test/` を唯一の起点とする。
- `test/rpi` と `test/firmware` に分割し、さらに機能別に分類する。
- ビルドに含めないC++テストは pytest からコンパイルして実行する。
- Makefile にはテスト用のコンパイルルールの記述を追加しない。

## ディレクトリ構成
```
test/
  rpi/
    proto/
      cpp/
        proto_tests.cpp
      test_proto_rpi_cpp.py
    comm/
      test_placeholder.py
  firmware/
    proto/
      cpp/
        proto_tests.cpp
      test_proto_firmware_cpp.py
    comm/
      test_placeholder.py
```

## 実行方法
- `uv run pytest`

## RPiプロトコルのC++テスト
- `test/rpi/proto/test_proto_cpp.py` が g++ でビルドして実行する。
- 依存ソースは `rpi/src/proto/*.cpp` を使用する。

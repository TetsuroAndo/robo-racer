import shutil
import subprocess
from pathlib import Path

import pytest


def test_proto_cpp(tmp_path: Path):
    """
    @brief
      Build and run RPi mc_proto codec tests on host.
    """
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")

    repo_root = Path(__file__).resolve().parents[3]
    test_cpp = repo_root / "test/rpi/proto/cpp/proto_tests.cpp"
    proto_src = [
        repo_root / "shared/proto/src/mcproto.cpp",
    ]

    if not all(path.exists() for path in proto_src):
        pytest.skip("shared proto sources not present")

    output = tmp_path / "proto_tests"
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-O2",
        "-I",
        str(repo_root / "shared/proto/include"),
        str(test_cpp),
        *[str(path) for path in proto_src],
        "-o",
        str(output),
    ]

    build = subprocess.run(cmd, capture_output=True, text=True)
    print("\n[INFO] build cmd:", " ".join(cmd))
    assert build.returncode == 0, build.stdout + build.stderr

    run = subprocess.run([str(output)], capture_output=True, text=True)
    print("\n[INFO] run bin:", output)
    assert run.returncode == 0, (run.stdout or "") + (run.stderr or "")

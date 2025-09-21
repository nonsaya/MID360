# Jetson Orin Nano 環境サマリー (2025-09-21)

## 基本情報
- 機種: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- CPU: 6x ARMv8 (Cortex-A78 系)
- カーネル: 5.15.148-tegra (aarch64)
- OS: Ubuntu 22.04.5 LTS (Jammy)
- L4T/JetPack: R36.4.3 / nvidia-l4t-core 36.4.3-20250107174145
- NVIDIA ドライバ: 540.4.0
- 電源モード: NV Power Mode: MAXN_SUPER

## メモリ/ストレージ
- メモリ: 7.4 GiB (使用 ~1.7 GiB / 空き ~4.2 GiB) + Swap 3.7 GiB
- ルートFS: NVMe ORICO 953.9G (/dev/nvme0n1p1 ext4)
  - 使用 8.0G / 空き 881G (Use 1%)
  - EFI: /dev/nvme0n1p10 (vfat)

## GPU/クロックの要点
- GPU 周波数: 306 MHz ～ 1020 MHz (MAXN_SUPER 時)
- EMC 上限: 3199 MHz

## JetPack コンポーネント
- nvidia-jetpack: 6.2.1+b38
- CUDA: 12.6.68 (nvcc: /usr/local/cuda/bin/nvcc)
- TensorRT: 10.3.0（Python バインディング確認済み）
- cuDNN: 9.3.0.75（CUDA 12 系向け）

## 環境設定
- 永続 PATH 追加: ~/.bashrc に以下を追記済み
  - export PATH=/usr/local/cuda/bin:$PATH

## TensorRT ベンチ（ResNet50, FP16）
- コマンド: /usr/src/tensorrt/bin/trtexec --onnx=/usr/src/tensorrt/data/resnet50/ResNet50.onnx --fp16 --memPoolSize=workspace:2048 --warmUp=200 --duration=3 --saveEngine=/tmp/resnet50_fp16.plan
- 主な結果（短時間計測）:
  - Throughput ≈ 520.8 qps
  - Mean GPU latency ≈ 1.97 ms
  - Engine 保存先: /tmp/resnet50_fp16.plan

## カメラ/ユーティリティ
- v4l2-ctl: 未導入（v4l-utils インストールが必要）
  - 推奨: sudo apt install -y v4l-utils
- tegrastats: 実行可（--count オプションは非対応）

## 参考コマンド
- バージョン確認
  - nvcc --version
  - python3 -c "import tensorrt as trt; print(trt.__version__)"
  - dpkg -l | grep -E 'libnvinfer|TensorRT|cudnn'
- カメラ確認（導入後）
  - v4l2-ctl --list-devices

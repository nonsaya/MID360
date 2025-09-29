# MID360

本リポジトリは Jetson Orin Nano（JetPack 6 / Ubuntu 22.04）に Livox MID360 を接続し、ROS 2 Humble + GLIM で自己位置推定、MAVROS 経由で ArduPilot(EKF3) に外部航法を供給するための構成・手順をまとめたものです。

## 環境サマリー
- ハード: Jetson Orin Nano 8GB（ERDK Super）
- OS/SDK: Ubuntu 22.04.5 / L4T R36.4.3 / JetPack 6.2.1
- GPUスタック: CUDA 12.6.68 / TensorRT 10.3.0 / cuDNN 9.3.0
- ROS 2: Humble（Desktop）
- LiDAR: Livox MID360（IP: 192.168.1.3）
- Jetson NIC例: 192.168.1.50/24

詳細は `Origin-HardWare.md` を参照。

## 主要ディレクトリ
- `configs/livox/` … MID360 用設定 (`MID360_config.json`)
- `ros2_ws2/src/` … ROS2 ワークスペース（livox_ros_driver2 / glim_ros2 など）
- `jetson_GLIM_cpu-install.md` … GLIM（CPU版）＋Livox のセットアップと起動手順
- `MID360_LIVOX.md` … MID360 ドライバ手順と運用ノート

## 導入状況（要点）
- Livox SDK2: /usr/local に導入済み（`sudo ldconfig` 済）
- livox_ros_driver2: ROS2 Humble 向けビルド済（`ros2_ws2`）
- GLIM（CPU版）: `~/.local` に導入済、`glim_ros2` を `ros2_ws2` でビルド済
- MAVROS: 今後導入予定（外部航法ブリッジ用）

## 起動（要約）
- LiDARドライバ（ros2_ws2）
```bash
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args -p user_config_path:=/home/$USER/repo/mid360/configs/livox/MID360_config.json
```
- GLIM（ビューワ無し/ヘッドレス）
```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
# ~/config/config_ros.json の extension_modules を ["libmemory_monitor.so"] に設定
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```
- GLIM（ビューワ有り）
```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
# ヘッドレスでビューワ利用時の例（任意）
# sudo apt install -y xvfb
# xvfb-run -s '-screen 0 1280x800x24' \
#   ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```

## 運用方針（ArduPilot 連携）
- 目標フロー
  - GLIMで自己位置推定（map基準）
  - MAVROSで外部航法（ODOMETRY推奨）をFCUへシリアル(921600bps)送信
  - EKF3でIMU/コンパス/外部航法を統合
  - 手順: Stabilize手動離陸→安定後Loiter切替→将来的にGUIDEDで座標指令
- 高度ソース
  - 主高度: BARO（必須）
  - 低空精度: レンジファインダー併用推奨
  - GLIMのZは補助（低重み）

## 外部PCでの静止マップ作成
- Jetson: 制御系（livox + GLIM + MAVROS）
- 外部PC: GLIMのみ（MAVROSは起動しない）
  - 名前空間: `--ros-args -r __ns:=/offboard_glim`
  - フレーム分離例: map_offboard / odom_offboard（`config_ros.json`で調整）
  - 代替: rosbag2でJetson記録→外部PCでオフラインGLIM

## 今後の到達目標
- Loiter安定化（外部航法併用）
- MAVROS導入と /mavros/odometry/in 経由の外部航法供給
- GUIDEDモードでの座標移動（安全柵＋Failsafe設計）
- ログ基盤（rosbag2 + ArduPilotログ）によるチューニング

## 参考ドキュメント
- `jetson_GLIM_cpu-install.md`（詳細手順）
- `MID360_LIVOX.md`（ドライバ運用注意）
- `Origin-HardWare.md`（機材・環境サマリー）
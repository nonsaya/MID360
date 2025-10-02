## Reinstall and Startup Guide (Jetson Orin Nano, JP 6.2, Ubuntu 22.04, ROS 2 Humble)

このドキュメントは、Livox MID360 ドライバ、GLIM、MAVROS の再インストール手順と起動手順をまとめたものです。基本ドメインは ROS_DOMAIN_ID=0（未設定）で統一します。複数PC連携が必要な場合のみ同一値で設定してください。

### 前提
- Jetson Orin Nano（JetPack 6.2 / Ubuntu 22.04）
- ROS 2 Humble インストール済み
- ネットワーク例: Jetson: 192.168.1.50 / MID360: 192.168.1.3（同一L2, ping疎通可能）
- リポジトリ: `~/repo/mid360`（このドキュメントの保存先）

---

## 1) Livox MID360 ドライバ（ROS 2）

### 設定ファイル
- パス: `~/repo/MID360/configs/livox/MID360_config.json`
- 主なポイント:
  - host_ip: JetsonのIP（例: 192.168.1.50）
  - LiDAR IP: 192.168.1.3
  - `pcl_data_type`: 1=Cartesian(32bit), 2=Cartesian(16bit), 3=Spherical
  - ROS出力形式は起動パラメータ`xfer_format`で指定（0=PointCloud2, 1=Custom, 2=PCL標準）

### 起動（PointCloud2 で出力）
```bash
source /opt/ros/humble/setup.bash
source ~/repo/MID360/ros2_ws2/install/setup.bash

# Launch (推奨)
ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0

# 直接ノード（例）
# ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
#   -p user_config_path:=/home/$USER/repo/MID360/configs/livox/MID360_config.json \
#   -p xfer_format:=0
```

### 確認
```bash
ros2 topic info -v /livox/lidar   # Type: sensor_msgs/msg/PointCloud2 を確認
ros2 topic hz /livox/lidar
ros2 topic echo -n1 /livox/imu
```

### よくあるトラブル
- "bind failed":
  - ドライバ二重起動の停止: `pkill -f livox_ros_driver2`
  - 設定host_ipがJetsonの実IPと一致しているか
  - `ping 192.168.1.3`、ポート競合なし（56100-56501）
- CustomMsgを別PCで見られない: `xfer_format:=0`でPointCloud2に切替

---

## 2) GLIM（CPU版）

### 設定
- 初期設定コピー: `/usr/local/share/glim/config` → `~/config`
- `~/config/config.json`: GPU設定をCPU用に差し替え（odometry/sub_mapping/global_mapping の *_cpu.json）
- `~/config/config_sensors.json`: `T_lidar_imu` を配列形式 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]`
- `~/config/config_ros.json`:
  - `extension_modules`: `["libmemory_monitor.so", "libstandard_viewer.so"]`
  - `imu_topic`: `/livox/imu`
  - `points_topic`: `/livox/lidar`

### 起動（ディスプレイあり）
```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```

### ヘッドレス起動（必要時）
```bash
xvfb-run -a -s "-screen 0 1280x800x24" \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```

### 確認
```bash
ros2 topic hz /glim_ros/points   # 目安: 10〜20 Hz
ros2 topic hz /glim_ros/odom     # 目安: 10 Hz
```

### RViz2
- Fixed Frame: `map`（不明なら `/glim_ros/odom` の header.frame_id を参照）
- Add → PointCloud2 → Topic `/glim_ros/points`

---

## 3) MAVROS（ArduPilot, /dev/ttyTHS1 921600bps）

### インストール
```bash
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### シリアル権限・競合回避
```bash
sudo usermod -aG dialout $USER
# 再ログイン or `newgrp dialout`

# getty類の停止（存在しなくてもOK）
sudo systemctl disable --now serial-getty@ttyTHS1.service || true
sudo systemctl disable --now nvgetty || true

ls -l /dev/ttyTHS1
```

### 起動
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600 gcs_url:=udp://@
```

### 確認
```bash
ros2 topic echo -n1 /mavros/state
ros2 topic hz /mavros/imu/data
```

### 注意（ROSドメイン）
- すべての端末・ノードで同じ `ROS_DOMAIN_ID` に揃える。基本は未設定（=0）。

---

## 4) 再インストール手順（サマリ）

1. ROS 2 Humble インストール
2. リポジトリ取得・サブモジュール初期化
   ```bash
   git clone https://github.com/nonsaya/MID360.git ~/repo/MID360
   cd ~/repo/MID360
   git submodule update --init --recursive
   ```
3. 依存（例）: GTSAM / gtsam_points / Iridescence をビルド＆インストール（必要時）
4. GLIM をビルド＆インストール（`sudo make install`）→ `/usr/local/share/glim/config` を `~/config` へコピー
5. ROS 2 ワークスペースをビルド（`~/repo/MID360/ros2_ws2`）
6. Livox SDK2 / livox_ros_driver2 をセットアップ、`MID360_config.json` をJetson/MID360 IPに合わせる
7. Livox SDK サンプルで疎通確認 → ROS 2 ドライバ起動（PointCloud2にしたい場合は `xfer_format:=0`）
8. GLIM起動（`~/config` 参照）→ `/glim_ros/points`, `/glim_ros/odom` を確認
9. MAVROS インストール・権限設定 → 起動・トピック確認

---

## 5) トラブルシュート（要点）

- Livox "bind failed": 二重起動停止、host_ip/ポート、疎通確認
- 別PCからPointCloud2が見えない: `xfer_format:=0`（標準型に切替）
- RViz表示不可: Fixed Frameを`livox_frame`や`map`に手入力、Color/Size調整
- ドメイン不一致: `ROS_DOMAIN_ID` の統一（未設定=0推奨）
- シリアル接続不可: dialout権限、getty停止、配線・電源確認

---

## 6) Git 保存（このドキュメントの更新方法）

```bash
cd ~/repo/mid360
git add reinstall.md
git commit -m "docs: add reinstall and startup guide (MID360, GLIM, MAVROS)"
git push
```



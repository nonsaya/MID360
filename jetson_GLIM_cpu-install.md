## Jetson Orin（CPU版）GLIM + Livox MID360 セットアップ手順

> 本プロジェクトのSLAMはGLIM（CPU版）を採用しています。FAST-LIOは導入しません。

このドキュメントは、Jetson Orin 環境で GPU を使わずに GLIM を動作させるための最小構成セットアップ手順です。Livox MID360 のドライバ（livox_ros_driver2）と GLIM（CPUモジュールのみ）を用い、ROS 2 で点群を処理します。

### 動作確認環境
- Jetson Orin Nano 8GB / JetPack 6.x（Ubuntu 22.04 LTS）
- ROS 2 Humble
- Livox MID360（例: LiDAR IP `192.168.1.3`）
- Jetson NIC（例: `192.168.1.50`）

参考:
- GLIM リポジトリ（依存・最新情報）: [GLIM: koide3/glim](https://github.com/koide3/glim?tab=readme-ov-file)
- 解説記事（Jetson Orin Nano 事例）: [Zenn: GLIM + Livox MID360 で SLAM](https://zenn.dev/koide3/articles/144e97133234e2)

---

## 1. ネットワーク設定（Jetson ↔ MID360）
Jetson の有線 NIC を MID360 に直結し、以下の例で固定してください。

```bash
# 例（手動設定）
sudo ip addr add 192.168.1.50/24 dev <NIC名>
sudo ip link set <NIC名> up

# 接続確認（MID360 の IP は 192.168.1.3 と仮定）
ping -c 3 192.168.1.3
```

Netplan で永続化する場合の例:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    <NIC名>:
      addresses: [192.168.1.50/24]
```

---

## 2. Livox SDK2 と livox_ros_driver2 のセットアップ

### 2-1. Livox SDK2
```bash
cd ~/repo/mid360
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

### 2-2. ROS2 ワークスペース作成と livox_ros_driver2（ros2_ws2 を使用）
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/repo/mid360/ros2_ws2/src
cd ~/repo/mid360/ros2_ws2/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 付属スクリプトでROS 2 Humble向けにビルド
cd ~/repo/mid360/ros2_ws2/src/livox_ros_driver2
bash ./build.sh humble
```

### 2-3. MID360 設定
`~/repo/mid360/configs/livox/MID360_config.json` を編集し、Jetson と LiDAR の IP を設定します。

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip":   "192.168.1.50",
      "push_msg_ip":   "192.168.1.50",
      "point_data_ip": "192.168.1.50",
      "imu_data_ip":   "192.168.1.50"
    }
  },
  "lidar_configs": [
    { "ip": "192.168.1.3" }
  ]
}
```

---

## 3. 依存ライブラリ（CPU構成）

### 3-1. GTSAM（4.3a0 推奨）
```bash
cd ~/repo/mid360
git clone https://github.com/borglab/gtsam.git
cd gtsam
git fetch --tags
git checkout 4.3a0
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_WITH_TBB=OFF
make -j4
sudo make install
sudo ldconfig
```

### 3-2. gtsam_points（1.2.0）
```bash
cd ~/repo/mid360
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points
git fetch --tags
git checkout v1.2.0 || git checkout 1.2.0
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install
sudo ldconfig
```

### 3-3. Iridescence（GLIM ビューワ依存）
```bash
sudo apt update && sudo apt install -y \
  libglfw3-dev libglm-dev libgl1-mesa-dev \
  libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

cd ~/repo/mid360
git clone https://github.com/koide3/iridescence.git
cd iridescence
git submodule update --init --recursive
mkdir -p build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

---

## 4. GLIM コア（CPU版）インストール（ユーザープレフィックス）
GPU を使わないため CPU モジュールのみで十分です。`~/.local` にインストールし、ROS から参照します。

```bash
cd ~/repo/mid360
git clone https://github.com/koide3/glim.git glim_core
cd glim_core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
make -j4
make install
```

環境変数（実行時）:
```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
```

---

## 5. ROS2 側（glim_ros）ビルド（ros2_ws2 を使用）
`glim_ros2` をワークスペースに置き、GLIM の CMake を `~/.local` から解決してビルドします。

```bash
cd ~/repo/mid360/ros2_ws2/src
git clone https://github.com/koide3/glim_ros2.git

cd ~/repo/mid360/ros2_ws2
CMAKE_PREFIX_PATH=$HOME/.local colcon build \
  --packages-select glim_ros \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble --symlink-install
```

---

## 6. GLIM 設定（CPU構成）
GLIM のサンプル設定をホーム配下にコピーし、CPU 用へ切り替えます。

```bash
mkdir -p ~/config
cp -r ~/.local/share/glim/config/* ~/config/

# CPU 用に切替（config.json）
sed -i 's/config_odometry_gpu.json/config_odometry_cpu.json/'       ~/config/config.json
sed -i 's/config_sub_mapping_gpu.json/config_sub_mapping_cpu.json/' ~/config/config.json
sed -i 's/config_global_mapping_gpu.json/config_global_mapping_cpu.json/' ~/config/config.json

# ROS 設定（Livox トピック/フレーム）
sed -i -E \
  's#"imu_topic"\s*:\s*"[^"]*"#"imu_topic": "/livox/imu"#; \
    s#"points_topic"\s*:\s*"[^"]*"#"points_topic": "/livox/lidar"#; \
    s#"lidar_frame_id"\s*:\s*"[^"]*"#"lidar_frame_id": "livox_frame"#; \
    s#"imu_frame_id"\s*:\s*"[^"]*"#"imu_frame_id": "livox_frame"#; \
    s#"acc_scale"\s*:\s*[-0-9.eE]+#"acc_scale": 9.80665#' \
  ~/config/config_ros.json
```

必要に応じて `~/config/config_sensors.json` の `T_lidar_imu` を実機の取り付けに合わせて調整してください（初期確認は単位クォータニオン [0,0,0,0,0,0,1] でも可）。

---

## 7. 実行手順（ヘッドレス可）

### 7-1. Livox ドライバ起動（ros2_ws2）
```bash
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args -p user_config_path:=/home/$USER/repo/mid360/configs/livox/MID360_config.json
```

`ros2 topic list` で `/livox/lidar` `/livox/imu` が出ていることを確認してください。

### 7-2. GLIM 起動（CPU）

#### 7-2-a. ビューワ付き起動（DISPLAY あり、または仮想ディスプレイ）
```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash

# 物理ディスプレイがある場合はそのまま
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)

# ヘッドレスでビューワも使いたい場合（任意）
# sudo apt install -y xvfb
# xvfb-run -s '-screen 0 1280x800x24' \
#   ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```

#### 7-2-b. ヘッドレス環境での起動（DISPLAY無し）

**重要**: 現行のGLIMビルドでは、ROSパブリッシャーがビューワモジュール（`libstandard_viewer.so`）と結合しているため、ビューワを完全に無効にするとトピックが配信されません。ヘッドレス環境では必ず`xvfb-run`を使用してください。

```bash
# xvfbが未インストールの場合（初回のみ）
sudo apt update && sudo apt install -y xvfb

# ヘッドレス環境での起動（推奨）
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
xvfb-run -a -s "-screen 0 1280x800x24" \
  ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=$(realpath ~/config)
```

**注意**: `extension_modules`を空にしたり`libstandard_viewer.so`を除外すると、`/glim_ros/odom`や`/glim_ros/points`などのトピックが配信されなくなります。必ず`["libstandard_viewer.so", "libmemory_monitor.so"]`を含める必要があります。

---

## 8. トラブルシューティング

- bind failed / SDK 初期化失敗:
  - Jetson と LiDAR の IP が競合していないか確認（例: Jetson 192.168.1.50 / LiDAR 192.168.1.3）。
- `/livox/lidar` が出ない:
  - ドライバ設定 `MID360_config.json` の IP / ポート再確認、`ping` で疎通確認。
- `libodometry_estimation_gpu.so` 読み込み失敗:
  - GPU 版設定になっています。`config.json` を CPU 用に切替（本書 6 章）。
- ビューワは起動するが何も見えない:
  - `config_ros.json` の `/livox/lidar` `/livox/imu` に切替済みか、`T_lidar_imu` が極端でないかを確認。

---

## 9. 参考
- GLIM: [https://github.com/koide3/glim](https://github.com/koide3/glim?tab=readme-ov-file)
- 記事: [GLIM + Livox MID360 で LiDAR-IMU SLAM (Jetson Orin Nano対応)](https://zenn.dev/koide3/articles/144e97133234e2)

---

## 10. トピック一覧（型・フレーム・用途）

- /livox/lidar
  - 型: sensor_msgs/PointCloud2
  - frame_id: livox_frame
  - 内容: Livox ドライバの生点群（GLIM 前処理入力）

- /livox/imu
  - 型: sensor_msgs/Imu
  - frame_id: livox_frame（本手順の設定）
  - 内容: IMU 生データ（約 200 Hz）

- /glim_ros/points
  - 型: sensor_msgs/PointCloud2
  - frame_id: livox_frame
  - 内容: GLIM の前処理済み点群（deskew / downsample 後）

- /glim_ros/points_corrected
  - 型: sensor_msgs/PointCloud2
  - frame_id: map
  - 内容: 大域補正（ループ閉じ）を反映した前処理点群

- /glim_ros/aligned_points
  - 型: sensor_msgs/PointCloud2
  - frame_id: odom
  - 内容: 現フレームをオドメトリ座標系（odom）へ整列した点群（可視化向け）

- /glim_ros/aligned_points_corrected
  - 型: sensor_msgs/PointCloud2
  - frame_id: map
  - 内容: 大域補正を反映して整列した点群（可視化向け）

- /glim_ros/map
  - 型: sensor_msgs/PointCloud2
  - frame_id: map
  - 内容: 現在までのグローバル点群マップ

- /glim_ros/odom
  - 型: nav_msgs/Odometry
  - frame_id: odom, child_frame_id: base_frame_id（本手順では livox_frame）
  - 内容: リアルタイムのオドメトリ（補正なし・ドリフトあり）

- /glim_ros/odom_corrected
  - 型: nav_msgs/Odometry
  - frame_id: map, child_frame_id: base_frame_id
  - 内容: 大域補正を反映したオドメトリ

- /glim_ros/pose
  - 型: geometry_msgs/PoseStamped
  - frame_id: odom
  - 内容: オドメトリ座標系での位置姿勢

- /glim_ros/pose_corrected
  - 型: geometry_msgs/PoseStamped
  - frame_id: map
  - 内容: 大域補正を反映した位置姿勢

- /image
  - 型: sensor_msgs/Image
  - 内容: 拡張モジュール使用時のみ（本手順では未使用）

- /tf, /tf_static
  - 内容: map → odom → base_frame（= livox_frame）および imu → lidar の TF ツリー

RViz 表示のヒント:
- Fixed Frame=map（補正後を基準に可視化）または odom（生オドメトリ）
- 典型表示: `/glim_ros/map`（全体地図）, `/glim_ros/aligned_points_corrected`（直近スキャン）, `/glim_ros/odom_corrected`（軌跡）

---

## 11. 停止方法（ドライバ/GLIM/関連プロセス）

最も安全なのは「起動したターミナルで Ctrl+C」です。ヘッドレスで端末を閉じてしまった場合などは以下を使います。

```bash
# 1) 前景で起動中なら Ctrl + C

# 2) 実行中プロセスを確認
pgrep -fa 'livox_ros_driver2|glim_rosnode|component_container|rviz2'

# 3) 個別停止（優先: SIGTERM）
pkill -f livox_ros_driver2           || true
pkill -f glim_rosnode                || true
pkill -f component_container         || true
pkill -f rviz2                       || true

# 4) まだ残る場合（強制終了: SIGKILL）
pkill -9 -f livox_ros_driver2        || true
pkill -9 -f glim_rosnode             || true
pkill -9 -f component_container      || true
pkill -9 -f rviz2                    || true

# 5) すべてのROS2プロセスを一括停止（必要時のみ）
pkill -f ros2 || true

# 6) 最終手段（システム再起動）
# sudo reboot
```

補足:
- systemd サービスとして起動していない想定です。サービス化している場合は `systemctl stop <service>` を使用してください。
- `pkill -9` は最後の手段です。まずは Ctrl+C / `pkill`（SIGTERM）を試してください。

---

## 12. 次回以降の起動（CPU設定済み時）

一度 CPU 用に切り替えた設定は `~/config/config.json` に残るため、次回以降は起動コマンドだけで動作します（再ビルド不要）。

```bash
# 毎回の前提（環境設定とライブラリパス）
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.sh

# GLIM（CPU）を起動
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
```

GPU に戻したい場合は、設定のみ切り替えれば同じ起動コマンドで動きます。

```bash
# CPU → GPU
sed -i 's/config_odometry_cpu.json/config_odometry_gpu.json/'       ~/config/config.json
sed -i 's/config_sub_mapping_cpu.json/config_sub_mapping_gpu.json/' ~/config/config.json
sed -i 's/config_global_mapping_cpu.json/config_global_mapping_gpu.json/' ~/config/config.json

# GPU → CPU（元に戻す）
sed -i 's/config_odometry_gpu.json/config_odometry_cpu.json/'       ~/config/config.json
sed -i 's/config_sub_mapping_gpu.json/config_sub_mapping_cpu.json/' ~/config/config.json
sed -i 's/config_global_mapping_gpu.json/config_global_mapping_cpu.json/' ~/config/config.json
```



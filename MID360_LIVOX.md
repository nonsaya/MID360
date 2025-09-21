### MID360 + Livox (ROS 2 Humble) セットアップ手順（Jetson Orin Nano 含む）

このドキュメントは、Livox MID360 を ROS 2 Humble 環境で動作させ、RViz で可視化できるところまでを、Git管理前提で手順化したものです。Orange Pi 5 Max で検証済み、Jetson Orin Nano でも同手順で再現可能です。

#### 前提
- OS: Ubuntu 22.04 (JetPack 6 系は 22.04 ベース)
- ROS 2: Humble (apt でインストール済みを想定)
- ネットワーク: MID360 の IP が 192.168.1.3、ホストの有線 NIC を 192.168.1.50/24 例で使用

参考:
- Livox SDK2: `https://github.com/Livox-SDK/Livox-SDK2`
- Livox ROS Driver 2: `https://github.com/Livox-SDK/livox_ros_driver2`
- 解説記事（ROS2 + MID360 + FAST-LIO2）: `https://blog.csdn.net/2301_79618994/article/details/150475756`

---

## 1. 事前インストール（共通）

```bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-vcstool \
  ros-humble-desktop

# シェルごとに読み込むROS環境
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Jetson Orin Nano: JetPack 6 環境でも上記でOK（NVIDIA提供のROS 2を使う場合はその手順に従ってください）。

## 2. ネットワーク確認

```bash
# 例: 有線NIC enP8p1s0 を 192.168.1.50/24 に設定
sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.1.50/24 dev enP8p1s0
sudo ip link set enP8p1s0 up

# 確認と疎通
ip -br addr show
ping -c 3 192.168.1.3    # MID360 への疎通
```

## 3. リポジトリ取得と作業ブランチ（本プロジェクトをGit運用）

```bash
cd ~/repo/MID360
git status -sb || git init
current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '')
git checkout -b setup-livox-fastlio-$(date +%Y%m%d-%H%M%S)
```

## 4. Livox SDK2 を /usr/local へインストール

サブモジュールで管理しつつ、システムにもインストールします。

```bash
cd ~/repo/MID360
git submodule add https://github.com/Livox-SDK/Livox-SDK2 extern/Livox-SDK2 || true
git submodule update --init --recursive

mkdir -p extern/Livox-SDK2/build
cd extern/Livox-SDK2/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"

# 記事どおり /usr/local に配置
sudo make install
# ライブラリキャッシュ更新（必須）
sudo ldconfig

# 環境（必要なら）
grep -q PKG_CONFIG_PATH ~/.bashrc || echo 'export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> ~/.bashrc
source ~/.bashrc
```

トラブル対策: cmake の互換性警告が出たら記事の回避策（`-DCMAKE_POLICY_VERSION_MINIMUM=3.5`）を検討。

## 5. livox_ros_driver2 を取得してビルド（ROS 2）

```bash
cd ~/repo/MID360
mkdir -p ros2_ws2/src
git submodule add https://github.com/Livox-SDK/livox_ros_driver2 ros2_ws2/src/livox_ros_driver2 || true
git submodule update --init --recursive

# ビルド（付属スクリプトを使用）
source /opt/ros/humble/setup.bash
cd ~/repo/MID360/ros2_ws2/src/livox_ros_driver2
bash ./build.sh humble
```

ビルド後のインストール空間: `~/repo/MID360/ros2_ws2/install`

## 6. MID360 固定IP 用の設定ファイルを作成

`~/repo/MID360/configs/livox/MID360_config.json` を用意（Git管理）。ホスト側 IP は実機のアドレスに合わせて変更。

```json
{
  "lidar_summary_info" : { "lidar_type": 8 },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.3",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {"roll":0.0, "pitch":0.0, "yaw":0.0, "x":0, "y":0, "z":0}
    }
  ]
}
```

## 7. ドライバの起動（PointCloud2 出力で一本化）

- 注: bash で `set -u`（未定義変数でエラー）を有効にしている場合、手動で `source /opt/ros/humble/setup.bash` する前に一度 `set +u` を実行してください。

```bash
# ターミナル1（ドライバ）
source /opt/ros/humble/setup.bash
source ~/repo/MID360/ros2_ws2/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p lvx_file_path:=/dev/null \
  -p user_config_path:=/home/$USER/repo/MID360/configs/livox/MID360_config.json
```

```bash
# ターミナル2（確認）
source /opt/ros/humble/setup.bash
ros2 topic info /livox/lidar -v    # 型が sensor_msgs/PointCloud2 で1つになっていること
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 8. RViz 表示（tf 警告への対処）

- RViz の Global Options → Fixed Frame を `livox_frame` に変更
- もしくは静的TFで `map -> livox_frame` を流す

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map livox_frame
```

RViz では Add → PointCloud2 → Topic: `/livox/lidar` を選択。必要に応じて Size, Style を調整。

## 9. トラブルシュート

- bind failed（検出チャネル）: UDP 56000 などが他プロセスで占有されている可能性
  ```bash
  ss -ulpn | grep -E ':(56000|5610[0-1]|5620[0-1]|5630[0-1]|5640[0-1]|5650[0-1])' || true
  pgrep -af 'livox|ros2_driver|rviz|slam|lio'
  # ユーザー権限で停止
  pkill -f 'livox_simple_driver.py|livox_ros2_driver_node|livox_ros_driver2_node|rviz2' || true
  ```
- /livox/lidar が複数型（CustomMsg と PointCloud2）: 同名トピックの重複発行。PointCloud2 で使う場合は `xfer_format:=0` に固定し、他の同名発行元を停止。
- RViz の `No tf data` 警告: Fixed Frame を `livox_frame` に変更、または静的TFを流す。

## 10. 参考リンク

- Livox SDK2: `https://github.com/Livox-SDK/Livox-SDK2`
- Livox ROS Driver 2: `https://github.com/Livox-SDK/livox_ros_driver2`
- CSDN 記事（ROS2 + MID360 + FAST-LIO2）: `https://blog.csdn.net/2301_79618994/article/details/150475756`

（FAST-LIO2 連携は別章で加筆予定）



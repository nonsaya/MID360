## LOITER 手順（MID360 + GLIM + MAVROS + ArduPilot/EKF3）

本書は、Jetson Orin Nano 上で Livox MID360 → GLIM（ヘッドレス） → ブリッジ → MAVROS → ArduPilot(EKF3) へ外部航法を供給し、LOITER を安定動作させるための起動・確認フローをまとめたものです。

### 全体フロー（トピックと周波数）

```mermaid
flowchart LR
  A["Livox MID360"] --> L1["/livox/lidar (~10Hz)"];
  L1 --> B["GLIM (ROS2)"];
  A --> L2["/livox/imu (~200Hz)"];
  L2 --> B;
  B --> L3["/glim_ros/odom (~10Hz)"];
  B --> L4["/glim_ros/odom_corrected (~10Hz)"];
  B --> L5["/glim_ros/pose (~10Hz)"];
  L3 --> C["glim_extnav_bridge"];
  L4 --> C;
  L5 --> C;
  C --> L6["/mavros/odometry/in (10-20Hz)"];
  L6 --> D["MAVROS"];
  D --> L7["MAVLink: ODOMETRY"];
  L7 --> E["ArduPilot EKF3"];
  E --> L8["推定ローカル位置/速度/ヨー"];
  L8 --> F["LOITER制御"];
```

- 基本は `/glim_ros/odom` を外部航法として供給（`use_corrected=true` でループ閉じ込み後も可）
- タイムスタンプのズレ対策はブリッジ側パラメータで調整可能（後述）

---

### 1. LiDAR ドライバ（livox_ros_driver2）起動

```bash
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args -p user_config_path:=/home/$USER/repo/mid360/configs/livox/MID360_config.json

# 確認
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

参考: `MID360_config.json` の IP が Jetson NIC/MID360 に一致していること

---

### 2. GLIM（ヘッドレス）起動

```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/repo/mid360/ros2_ws2/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)

# 確認
ros2 topic hz /glim_ros/odom
```

- フレーム: 典型的に `odom`（親）/ `base_frame_id`（子, 本手順では `livox_frame`）
- 必要に応じて `~/config/config_ros.json` の `points_topic` / `imu_topic` / frame_id 等を調整

ヘッドレス環境の注意（DISPLAY なしでの起動）:

- 典型的なエラー例:
  ```
  glfw error 65544: X11: The DISPLAY environment variable is missing
  failed to initialize GLFW
  [ros2run]: Segmentation fault
  ```
- 推奨対処: 仮想ディスプレイでラップして起動（設定変更不要）
  ```bash
  sudo apt update && sudo apt install -y xvfb
  xvfb-run -a -s "-screen 0 1280x800x24" \
    ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/config)
  ```
- 恒久対処（GUI不要の場合）: GLIMのビューアを無効化
  - `config_ros.json` などで `standard_viewer`（viewer）相当のプラグイン読み込みを無効化（enabled=false など）。
  - ビューアを読み込まなければ GLIM は DISPLAY なしでも動作します。
- GUIが使える環境なら: X転送または実ディスプレイを使用
  - SSH の場合は `ssh -X` でログインして起動
  - ローカルXがある場合は `export DISPLAY=:0` 後に起動

---

### 3. MAVROS 起動（FCU 接続済み前提）

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyTHS1:921600 \
  fcu_protocol:=v2.0 \
  gcs_url:=udp://@

# 確認
ros2 topic echo /mavros/state
```

---

### 4. GLIM→MAVROS 外部航法ブリッジ（先にビルド→起動）

先にビルド:
```bash
source /opt/ros/humble/setup.bash
cd ~/repo/mid360/ros2_ws2
colcon build --packages-select glim_extnav_bridge
source install/setup.bash
```

起動（既定: GLIMの stamp を転送）:
```bash
ros2 launch glim_extnav_bridge bridge.launch.py \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:="" \
  restamp_source:=none \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true
```

ズレ対策（必要時）:
- `restamp_source=arrival` または `now` に切替（ノード時刻で再スタンプ）
- 再スタンプ時は通常 `reject_older_than_ms=0.0` でよい

監視:
```bash
ros2 topic hz /mavros/odometry/in
ros2 topic info /mavros/odometry/in
```

トラブルシュート: `launch` が `libexec directory ... does not exist` を出す場合は、直前に `colcon build` と `source install/setup.bash` が実行されているか確認

---

### 4.5 RViz2 で GLIM/MAVROS の Pose 整合を事前確認（実機前推奨）

- 準備（型とフレーム確認）
  ```bash
  ros2 topic info /glim_ros/pose
  ros2 topic info /mavros/local_position/pose
  # どちらも geometry_msgs/msg/PoseStamped であること、header.frame_id を確認（例: map / odom など）
  ```

- RViz2 表示手順
  - `rviz2` を起動し、Fixed Frame を両者で共通なフレーム（例: `map` または `odom`）に設定
  - Displays → Add → Pose を2つ追加し、Topic をそれぞれ `/glim_ros/pose` と `/mavros/local_position/pose` に設定
  - Displays → Add → TF を追加（座標軸の整合確認用）
  - 実機を手で動かし、2つの Pose が方向/量ともに追従するか、原点や姿勢に恒常オフセットがないか確認

- フレームが合わない/恒常オフセットがある場合の対処
  - Fixed Frame を片方の `frame_id` に合わせるか、静的 TF で橋渡し
  - 例: `map` と `glim_world` を一時的に一致させる（恒等変換）
    ```bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map glim_world
    ```
  - 恒常オフセット（並進/回転）があるなら、その値を推定して静的 TF に反映

- 実験ログ（オフライン比較用）
  ```bash
  ros2 bag record /glim_ros/pose /mavros/local_position/pose /tf /tf_static
  ```

メモ:
- MAVROS 側のトピックは ENU に変換されているため、RViz2 上では ENU で揃うのが通常です。
- 長時間のドリフト傾向を視覚化したい場合は、`nav_msgs/Path` で軌跡比較ノードを用いると見やすくなります。

### 5. ArduPilot EKF3 外部航法設定（例）

- `EK3_SRC1_POSXY = 6`（ExternalNav）
- `EK3_SRC1_VELXY = 6`（ExternalNav）
- `EK3_SRC1_YAW   = 6`（External Yaw を使う場合）
- `VISO_TYPE = 1`（MAVLink ODOMETRY 使用）
- 高度ソースは BARO 主、低空はレンジファインダー併用推奨

注意:
- フレーム整合（`frame_id` / `child_frame_id`）を一貫させる
- ENU/NED の基準差は ArduPilot 側で扱われるが、ヨー原点やロール/ピッチの符号整合に留意

---

### 6. LOITER の流れ（運用）

1) Stabilize 等で手動離陸 → 機体が安定したら Loiter へ切替
2) Loiter 安定性を確認（風下・磁気干渉・外部航法の有無で挙動変化）
3) 安定後、GUIDED で座標移動を検証（必要時）

参考コマンド（GUIDED）:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}" -r 10
```

---

### 7. 主要トピック（要点）

- LiDAR → GLIM
  - `/livox/lidar` sensor_msgs/PointCloud2（約10 Hz）
  - `/livox/imu`   sensor_msgs/Imu（約200 Hz）
- GLIM → Bridge
  - `/glim_ros/odom` nav_msgs/Odometry（約10 Hz, frame_id=odom, child=base）
  - `/glim_ros/pose` geometry_msgs/PoseStamped（約10 Hz, frame_id=map）
- Bridge → MAVROS
  - `/mavros/odometry/in` nav_msgs/Odometry（10–20 Hz）
- MAVROS → ArduPilot
  - MAVLink ODOMETRY（POSXY/VELXY/YAW を EKF3が使用）

---

### 8. ズレ対策（再掲）

- ブリッジのパラメータ
  - `restamp_source`: `none`（既定, GLIM の stamp を使用）/ `arrival` / `now`
  - `reject_older_than_ms`: `none`時の旧stampドロップ閾値（ms）
  - `publish_rate_hz`: 出力レート（10–20 Hz 推奨）
  - `publish_immediately`: コールバック到着時に即時配信（レート抑制と併用）

運用の勘所:
- まず `none` で評価し、Loiter が不安定なら `arrival` へ切替
- FCU 側 Parameter を適用後は再起動して反映


---

### 9. 各ノードの停止方法（安全手順）

最優先は「起動させたターミナルで Ctrl+C（SIGINT）」です。ヘッドレス運用で端末を閉じてしまった場合のみ、以下を使用します。

```bash
# 実行中プロセスの確認（必要なもののみ停止）
pgrep -fa 'livox_ros_driver2|glim_rosnode|glim_extnav_bridge|mavros|component_container|rviz2'

# 個別停止（優先: SIGTERM）
pkill -f livox_ros_driver2           || true   # LiDAR ドライバ
pkill -f glim_rosnode                || true   # GLIM
pkill -f glim_extnav_bridge          || true   # 外部航法ブリッジ
pkill -f mavros                      || true   # MAVROS（launch含む）
pkill -f component_container         || true   # ROS 2 コンテナ（使用時）
pkill -f rviz2                       || true   # RViz（使用時）

# なお残る場合のみ（強制終了: SIGKILL）
pkill -9 -f livox_ros_driver2        || true
pkill -9 -f glim_rosnode             || true
pkill -9 -f glim_extnav_bridge       || true
pkill -9 -f mavros                   || true
pkill -9 -f component_container      || true
pkill -9 -f rviz2                    || true

# すべてのROS2プロセスを一括停止（最後の手段）
pkill -f ros2 || true
```

補足:
- systemd サービスで起動している場合は `systemctl stop <service>` を使用。
- `pkill -9` は最後の手段。まず Ctrl+C → `pkill`（SIGTERM）→ それでも残る場合のみ実施。



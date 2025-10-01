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

## MAVROS 起動例 (ArduPilot/APM)

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyTHS1:921600 \
  fcu_protocol:=v2.0 \
  gcs_url:=udp://@
```

- FCUポートは実機に合わせて変更（例: /dev/ttyUSB0 / /dev/ttyACM0 / /dev/ttyTHS*）
- 接続確認: `ros2 topic echo /mavros/state`

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

## 外部航法ブリッジ（GLIM→MAVROS /mavros/odometry/in）

GLIMの`/glim_ros/odom`（または`/glim_ros/odom_corrected`）をMAVROSの`/mavros/odometry/in`へ10–20 Hzで転送するROS 2ノードを追加。

- パッケージ: `ros2_ws2/src/glim_extnav_bridge`
- 実行: launchで起動

ビルド（先行）/起動:
```bash
source /opt/ros/humble/setup.bash
cd ~/repo/mid360/ros2_ws2
colcon build --packages-select glim_extnav_bridge
source install/setup.bash
ros2 launch glim_extnav_bridge bridge.launch.py \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:="" \
  restamp_source:=none \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true
```

メモ:
- `use_corrected=true`でループ閉じ込み後の`/glim_ros/odom_corrected`を送る運用も可能。
- `odom_child_frame_id`で`child_frame_id`を上書き可能（ArduPilotの機体座標系名と合わせる場合に使用）。
- タイムスタンプ安定化:
  - `restamp_source`: `none`（元stamp温存）, `arrival`/`now`（ノード時刻で再スタンプ）
  - `reject_older_than_ms`: `none`利用時に古いstampを拒否する閾値（ms）
  - `publish_immediately`: コールバック到着で即時配信（指定レートに従い抑制）

トピック確認:
```bash
ros2 topic echo /mavros/odometry/in | head -n 20
```

QoS: Reliable / KeepLast(10)

期待周波数: 10–20 Hz（`publish_rate_hz`で調整）

## ArduPilot EKF3 ExternalNav 設定（例）

外部航法（ODOMETRY）を利用するための主なパラメータ（EKF3 instance1例）:

- `EK3_SRC1_POSXY = 6`（ExternalNav）
- `EK3_SRC1_POSZ  = 0`（高度はBARO/RNGFに委譲推奨）
- `EK3_SRC1_VELXY = 6`（ExternalNav）
- `EK3_SRC1_VELZ  = 0`（必要に応じ）
- `EK3_SRC1_YAW   = 6`（External Yaw、GLIMのYawを使う場合）
- `VISO_TYPE = 1`（MAVLink ODOMETRY使用）
- `VISO_DELAY` / `VISO_POS_X/Y/Z` / `VISO_YAW`（機体座標→センサ座標のオフセット設定）
- `AHRS_EKF_TYPE = 3`（EKF3）

注意:
- フレーム: MAVROS `/mavros/odometry/in`はローカルNED想定。GLIMの`odom`がENUの場合、ArduPilot側で変換されるが、ヨー原点・左右手系の整合に注意。
- 一貫した`frame_id`/`child_frame_id`（例: `odom`/`base_link`）を継続して送る。
- 高度はBARO主／低高度はレンジファインダー併用推奨。

## GUIDED 座標移動（例）

準備:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
```

ローカル座標指令（ENU）:
```bash
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}" -r 10
```

または速度指令:
```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}}" -r 10
```

安全:
- Loiterで安定確認後にGUIDEDへ移行。
- Failsafe/ジオフェンスを設定。
## 参考ドキュメント
- `jetson_GLIM_cpu-install.md`（詳細手順）
- `MID360_LIVOX.md`（ドライバ運用注意）
- `Origin-HardWare.md`（機材・環境サマリー）

## 最新の動作確認結果（2025-10-01）

✅ **GLIM正常動作確認済み**
- ヘッドレス環境での起動: `xvfb-run`使用で正常動作
- 点群データ配信: `/glim_ros/points`で20Hz安定配信
- オドメトリデータ配信: `/glim_ros/odom`で正常配信
- 点群表示: ビューワーで正常表示確認（伸び問題解決済み）

✅ **設定ファイル修正済み**
- `T_lidar_imu`: 単位変換（`[0,0,0,0,0,0,1]`）で点群伸び問題解決
- Livoxトピック設定: `/livox/imu`, `/livox/lidar`に正しく設定
- CPU用設定: GPU→CPU切り替え完了

## 運用ガイド
- `LOITER.md`（LOITER運用ガイド）

## 参考ドキュメント
- `jetson_GLIM_cpu-install.md`（詳細手順）
- `MID360_LIVOX.md`（ドライバ運用注意）
- `Origin-HardWare.md`（機材・環境サマリー）



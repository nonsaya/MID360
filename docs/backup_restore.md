## バックアップと復旧（Jetson Orin Nano / JetPack 6.2 / Ubuntu 22.04）

このページは、自動更新の封じ込み、日次バックアップ（差分）、および最短復旧のための手順をまとめます。

---

### 1) 自動更新の完全封じ込み

```bash
# APT 周期自動実行を無効化
sudo tee /etc/apt/apt.conf.d/99freeze >/dev/null <<'EOF'
APT::Periodic "0";
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
Unattended-Upgrade::Automatic-Reboot "false";
EOF

# unattended/apt-daily を停止＋mask、パッケージ削除
sudo systemctl disable --now unattended-upgrades.service || true
sudo systemctl disable --now apt-daily.timer apt-daily.service || true
sudo systemctl disable --now apt-daily-upgrade.timer apt-daily-upgrade.service || true
sudo systemctl mask apt-daily.service apt-daily-upgrade.service || true
sudo apt remove -y unattended-upgrades || true

# PackageKit 停止＋mask
sudo systemctl disable --now packagekit.service packagekit || true
sudo systemctl mask packagekit.service || true

# Snap 未使用なら削除（使用する場合は削除せず延期運用へ）
sudo apt purge -y snapd || true

# JetPack/L4T/CUDA/TensorRT/カーネルを hold（誤更新防止）
sudo apt-mark hold $(dpkg -l | awk '/nvidia-jetpack|nvidia-l4t|cuda-|tensorrt|libnvinfer|linux-image|linux-headers/ {print $2}') || true
```

検証:
```bash
systemctl list-timers --all | grep -Ei 'apt|snap|packagekit' || echo "OK: 該当タイマーなし"
apt-mark showhold | grep -E 'nvidia-jetpack|nvidia-l4t|cuda-|tensorrt|libnvinfer|linux-image|linux-headers' || echo "注意: hold 対象が空です"
apt-config dump | grep -i APT::Periodic
systemctl is-enabled snapd.service snapd.snap-repair.timer packagekit.service 2>/dev/null || true
```

---

### 2) 日次バックアップ（差分 rsync）

バックアップ先（例）: 外付けSSD（`/media/$USER/writable`）

```bash
export BKDIR=/media/$USER/writable/mid360_backup_$(date +%Y%m%d_%H%M%S)
sudo mkdir -p "$BKDIR"/{home_config,repo,dot_ros,meta}
sudo chown -R "$USER:$USER" "$BKDIR"

# データ同期
[ -d ~/config ] && rsync -aH --delete ~/config/ "$BKDIR"/home_config/ || echo "skip: ~/config"
[ -d ~/repo ]   && rsync -aH --delete --exclude '.git/' ~/repo/ "$BKDIR"/repo/ || echo "skip: ~/repo"
[ -d ~/.ros ]   && rsync -aH --delete ~/.ros/ "$BKDIR"/dot_ros/ || echo "skip: ~/.ros"

# メタ情報
apt-mark showmanual | sort > "$BKDIR"/meta/pkglist.manual.txt
apt-mark showhold > "$BKDIR"/meta/pkglist.hold.txt
pip3 freeze > "$BKDIR"/meta/pip3-freeze.txt 2>/dev/null || true
uname -a > "$BKDIR"/meta/uname.txt
cat /etc/os-release > "$BKDIR"/meta/os-release.txt
dpkg -l | awk '/nvidia-l4t|cuda-|tensorrt|libnvinfer/ {print $2, $3}' > "$BKDIR"/meta/l4t_cuda_tensorrt.txt

# /etc（必要なら）
sudo rsync -aHAX --delete /etc/ "$BKDIR"/etc/

echo "BACKUP_DIR=$BKDIR"; du -sh "$BKDIR"/* 2>/dev/null || true
```

---

### 3) 復旧手順（SDK ManagerでJetPack再インストール後）

前提: ユーザー名を同じにする（`$USER`）

1. 自動更新封じ込みを再適用（1の手順）
2. バックアップルート指定:
```bash
export BKDIR=/media/$USER/writable/mid360_backup_YYYYMMDD_HHMMSS
```
3. パッケージ復元:
```bash
sudo xargs -a "$BKDIR"/meta/pkglist.manual.txt apt install -y
xargs -a "$BKDIR"/meta/pkglist.hold.txt -r sudo apt-mark hold
pip3 install -r "$BKDIR"/meta/pip3-freeze.txt || true
```
4. データ復元:
```bash
rsync -aHAX "$BKDIR"/home_config/  ~/config/
rsync -aHAX "$BKDIR"/repo/         ~/repo/
rsync -aHAX "$BKDIR"/dot_ros/      ~/.ros/
```
5. /etc は選択的に戻す（netplan/udev/systemd等）
```bash
# 例
sudo rsync -aHAX "$BKDIR"/etc/ /etc/
```
6. シリアル等の調整:
```bash
sudo usermod -aG dialout $USER
sudo systemctl disable --now serial-getty@ttyTHS1.service || true
sudo systemctl disable --now nvgetty || true
```
7. 確認:
```bash
ros2 topic hz /livox/lidar
ros2 topic hz /glim_ros/points
ros2 topic hz /glim_ros/odom
```

---

### 4) ゴールデンイメージ（丸ごとクローン）について

差し替え即起動を狙う場合は、内蔵NVMeと同容量以上の予備NVMeに対してオフラインでクローンを作成してください。誤書き込み防止のため`lsblk`で入念に確認のうえ実施してください。


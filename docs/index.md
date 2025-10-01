---
title: MID360 + GLIM + MAVROS + ArduPilot/EKF3
---

# 概要

このサイトは、Jetson Orin Nano 上で Livox MID360 → GLIM（ヘッドレス） → ブリッジ → MAVROS → ArduPilot(EKF3) へ外部航法を供給し、LOITER を安定動作させるための手順をまとめたドキュメントです。

- 主要ガイド: [LOITER 手順](LOITER.md)

## 構成
- デバイス: Livox MID360
- 推定: GLIM (ROS 2)
- ブリッジ: glim_extnav_bridge
- FCU連携: MAVROS → MAVLink ODOMETRY → ArduPilot EKF3

## クイックリンク
- GLIM 起動: セクション「2. GLIM（ヘッドレス）起動」
- ブリッジ起動: セクション「4. GLIM→MAVROS 外部航法ブリッジ」
- RViz2 比較検証: セクション「4.5 RViz2 で GLIM/MAVROS の Pose 整合を事前確認」
# MID360 (Jetson Orin Nano) - Docs

このサイトは GitHub Pages から配信する想定のドキュメントです。

- プロジェクトREADME: [../README.md](../README.md)
- LOITER運用ガイド: [../LOITER.md](../LOITER.md)

GitHub Pages 設定:
- リポジトリ設定 → Pages → Source: `main` ブランチ / フォルダ: `/docs`
- 保存後、しばらく待つと公開されます（URLはGitHubの設定画面に表示）

# m5stack_imu_micro_ros

M5Stack **Core2**（既定）または Basic（v2.6）＋ M5Stack IMU Unit（MPU6886、Port A）の加速度を、
micro-ROS（シリアル）で ROS 2 Humble にブロック単位で配信するファームウェアとメッセージ。
粉砕の振動計測（onolab_ros2 `onolab_grinding_vibration`）用。

| ディレクトリ | 中身 |
| --- | --- |
| `firmware/` | PlatformIO プロジェクト（Arduino + M5Unified + micro_ros_platformio） |
| `m5stack_imu_msgs/` | ROS 2 メッセージ `AccelBlock`（ホストの ROS ワークスペースでビルドする。ファームウェアには `firmware/extra_packages/` のシンボリックリンク経由で入る） |

## 仕様

- **サンプリング: 4 kHz（MPU6886 の上限）**。`ACCEL_FCHOICE_B = 1` でデジタルLPFを外しているので、
  帯域は 3 dB で 1046 Hz、2 kHz（ナイキスト）より上は取れない。**10 kHz は MPU6886 では取れない。**
  実際の出力レートは個体差があるため、起動時に ESP32 のクロックで 2 秒測って `sample_rate_hz` に入れる
  （実機で 3996.6 Hz）。
- レンジ ±2 g（`ACCEL_FS_G`）、`accel_m_s2_per_lsb` で m/s² に換算。
- トピック `/m5stack_imu/accel_block`（best effort）、48 サンプル/ブロック（12 ms、約 83 ブロック/s）。
- MPU6886 の FIFO（1 KB ≒ 40 ms）を 2 ms ごとに読む専用タスク（コア0）と、micro-ROS 送信（コア1）に分けている。
  サンプルを失ったときは隠さない: FIFO あふれ → `FLAG_FIFO_OVERFLOW`、送信遅れでキューあふれ → `FLAG_QUEUE_OVERFLOW`、
  いずれも `first_sample_index` が飛ぶ。再接続後は `FLAG_RESYNC` で index 0 から。
- `header.stamp` はブロック先頭サンプルの推定ホスト時刻（micro-ROS の時刻同期 + ESP32 クロック、数 ms 精度）。
  サンプル間隔は `first_sample_index` と `sample_rate_hz` を使う。
- Port A は Core2 では GPIO32/33（Basic は内部バスと同じ 21/22）。M5Unified は `M5.begin()` で Port A の
  I2C（`Ex_I2C`）を開始しないので、ファームウェアで `M5.Ex_I2C.begin()` を呼んでいる（Core2 ではこれが無いと
  IMU Unit が見えない）。`-DIMU_INTERNAL=1` で Core2 内蔵の MPU6886 を使う。
- 画面: 接続状態、レート、送信ブロック数/s、欠落数、各軸の平均と振動 RMS（重力を除いた AC 成分、バーは 0–2 m/s²）、
  前回のリセット要因。

## ビルドと書き込み

```bash
cd firmware
pio run -e m5stack-core2 -t upload --upload-port /dev/ttyUSB1     # Core2: CP2104 (10c4:ea60)
pio run -e m5stack-basic -t upload --upload-port /dev/ttyACM1     # Basic: CH9102 (1a86:55d4)
# ROS_DOMAIN_ID はファームウェアに焼き込む（エージェントでは決まらない）。既定 0
PLATFORMIO_BUILD_FLAGS="-DROS_DOMAIN_ID=12" pio run -e m5stack-basic -t upload
```

- micro_ros_platformio の初回ビルドで micro-ROS ライブラリを作る（数分）。PlatformIO の仮想環境
  `~/.platformio/penv` が必要で、その中の empy は **3.3.4**（4.x だと `em.BUFFERED_OPT` で失敗する）。
- メッセージを変えたら `.pio/libdeps/m5stack-basic/micro_ros_platformio/libmicroros` を消して作り直す。

## ホスト側

```bash
# micro-ROS エージェント（公式イメージ。ROS_DOMAIN_ID はファームウェア側の値になる）
docker run -it --rm --net=host --device=/dev/ttyUSB1 microros/micro-ros-agent:humble \
    serial --dev /dev/ttyUSB1 -b 921600

# メッセージをビルドして確認
colcon build --packages-select m5stack_imu_msgs && source install/setup.bash
ros2 topic hz /m5stack_imu/accel_block      # 約 83 Hz
```

## 検証状況（2026-11-08、静置）

- **Core2 + IMU Unit（Port A）**: 30 秒連続で index の欠落 0、フラグ 0、再接続 0、実測 3997.0 Hz、
  静置時の大きさ 9.683 m/s²。onolab_grinding_vibration（m5_accel）経由で窓ごとの生データ CSV も確認。

Basic v2.6 + IMU Unit（この個体は画面が故障していた）:

- 120 秒連続: 9994 ブロック（83.3/s）、index の欠落 0、フラグ 0、再接続 0。index とタイムスタンプの経過が一致
  （120.018 s / 119.997 s）。静置時の平均ベクトルの大きさ 9.685 m/s²（重力 9.807 に対し −1.2 %、未校正）。
- 粉砕中の振動では未確認。加速度の校正（オフセット・感度）はしていない。

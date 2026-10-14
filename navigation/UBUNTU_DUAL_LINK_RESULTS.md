# Ubuntu ROS2 + 真实视觉双链路验收

日期：2026-10-13。已拉取 GitHub `b98e36f`，读取 Windows 双链路实测记录。
继续使用 Windows 已烧录的 C 板固件，本次没有修改或重新烧录固件。

## 实测结果

| 验收项 | 结果 |
|---|---|
| Ubuntu 双口 probe，6 秒 | BOTH_LINKS / NAV_TIMEOUT_ONLY / BOTH_TIMEOUT 全部 PASS |
| USB 反馈 ID 02 / 1E / E0 | 各 200 帧 |
| USART 姿态 | 599 帧，两路拒绝帧 0 / 0 |
| ROS2 Humble 桥接构建 | 通过，安装在本仓库 navigation/install |
| ROS2 cmd_vel → USB → C 板快照 | 速度 (0.2, -0.1, 0.3)，nav_valid=1 |
| cmd_vel 停更 | 速度清零，USB 仍有效；这是桥接的话题超时保护 |
| 海康 + standard + ROS2 桥接并行 | 两路 valid=1，真实目标 tracking=1，shoot_requested=0 |
| 蓝色装甲板跟踪 | 约 10 秒发送阶段取得 332 个同时满足导航速度和视觉跟踪的回传快照 |
| 退出 | 相机线程正常停止，USB / USART 无进程占用 |
| 退出后只读复测，3 秒 | 双路 valid=0，速度/角度/跟踪/开火全零；USB 各 100 帧、USART 300 帧，拒绝帧 0/0 |

真实跟踪回传示例（角度为弧度）：

```text
[vx, vy, wz, yaw, pitch, nav_valid, vision_valid, tracking, shoot_requested]
[0.200000003, -0.100000001, 0.300000012, 1.063321114, 0.173596010, 1, 1, 1, 0]
```

停止速度发布后，回传速度为零，视觉角度仍更新且 tracking=1。
这证明 ROS2 桥接与真实视觉已同时进入 C 板，而非只证明电脑串口写入成功。
当前 SENTRY_LINK_BENCH_ONLY=1，底盘、云台、发射机构尚未联动。

## 设备与配置

- C 板原生 USB：0483:00ca，`/dev/ttyACM0`。
- 本机稳定路径：`/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_306F35813034-if00`。
- CH340：`/dev/rm_serial` → `/dev/ttyUSB0`，921600 8N1。
- J-Link 是 `/dev/ttyACM1`，不用于导航通信。
- 海康：MV-CS016-10UC，2bdf:0001。
- `auto-aim/configs/sentry_step1.yaml` 根据用户实际亮灯装甲板改为 `enemy_color: blue`；保持 `auto_fire: false`。
- `/dev/rm_nav` 尚未安装规则，本轮使用上述 by-id 路径，无需改写系统配置。

最初红色配置下双路通信有效，但没有跟踪；改为蓝色后出现真实跟踪。
比赛时 enemy_color 应按对方阵营设置，不直接沿用本次蓝色测试配置。

## 本机复测

关闭视觉、probe、旧导航串口节点；裸板模式或电机断开后执行。
本机缺少 pyserial，本次安装到 navigation/install/python_deps（构建目录，不上传）。
更换机器后建议安装 `sudo apt install python3-serial`，再重新构建桥接包。

```bash
cd /home/xiaoli/Desktop/cool-sentry
source /opt/ros/humble/setup.bash
source navigation/install/setup.bash
export PYTHONPATH="$PWD/navigation/install/python_deps:$PYTHONPATH"
export ROS_LOG_DIR=/tmp/sentry-ros-logs

# 仅 ROS2 + C 板
python3 navigation/tests/ros_bridge_board_check.py \
  --usb /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_306F35813034-if00 \
  --send-test

# 加入真实相机视觉；先在镜头前放蓝色亮灯装甲板
python3 navigation/tests/ros_bridge_board_check.py \
  --usb /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_306F35813034-if00 \
  --send-test --vision-config auto-aim/configs/sentry_step1.yaml
```

脚本自动启动和结束程序，使用独立 bench 话题，速度为 (0.2,-0.1,0.3)。
它拒绝 auto_fire 非 false 的视觉配置。TRACKING snapshots 为 0 时，仅代表链路通过，不能说目标跟踪通过。
脚本已修正 ros2 run 包装进程退出可能残留桥接子进程的问题，向整个子进程组发送 SIGINT。

如果 navigation/install 不存在，先在仓库根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base navigation/log build \
  --base-paths navigation/ros2/sentry_cboard_bridge \
  --build-base navigation/build --install-base navigation/install \
  --symlink-install --packages-select sentry_cboard_bridge
```

## 接下来

通信接力阶段已通过。后续连接雷达和 EW- 导航环境，用真实导航输出替代测试速度。
先确认定位和 TF、最终 cmd_vel 话题及坐标系；关闭旧 standard_robot_pp_ros2 串口节点和手工速度发布器。
桥接参数 cmd_vel_topic 指向最终导航速度话题，设备使用本机上述 by-id 路径或安装规则后的 /dev/rm_nav。
导航初始自旋速度先设为 0，继续保持裸板模式观察 C 板快照。

在实车出来前可继续做 USB 热插拔、长时间并行、重复冷启动、相机标定和坐标轴验证。
当前原项目内外参、偏角和弹速仍不能视作本车实测标定；本次不证明瞄准精度、底盘运动、云台闭环或射击许可。

# Windows 调试 C 板：真实视觉与导航数据

本轮沿用已验证的双链路：视觉 USART（CH340，921600），导航原生 USB CDC。此次没有改成单 USART，也没有修改 C 板协议。

## 先区分要验证的数据

- 真实视觉：海康图像 → standard 检测/跟踪 → yaw/pitch → USART → C 板。姿态由 C 板返回视觉。
- 真实雷达定位：MID360 → Point-LIO → 点云与里程计。**这些数据不是底盘速度指令，不会自动发到 C 板。**
- 真实导航指令：定位、地图、TF、Nav2 规划与控制 → 最终 cmd_vel → sentry_cboard_bridge → USB → C 板。

目前真实视觉与 ROS2 手工测试速度并行已经通过，雷达和 Point-LIO 手持定位已通过输出测试。
完整 Nav2、地图重定位和真实导航速度尚未通过本机验证。不能把旧 dual_link_probe 的固定速度或手工 topic pub 算作真实导航结果。

## 推荐设备分工

```text
Ubuntu 小电脑：海康、MID360、视觉、导航、桥接
    CH340 → C 板 USART
    原生 USB → C 板导航 USB CDC
Windows 电脑：J-Link → 同一块 C 板 SWD，烧录和查看变量
```

两台电脑同时运行，C 板按原方式独立供电；SWD/GND 接法沿用已验证方式。
原生 USB 和 CH340 都接 Ubuntu，J-Link 接 Windows。不要让两台主机共接同一个 USB 设备，也不要在 Ubuntu 测试时再运行 Windows 串口 probe。

如果只有一台双系统电脑，切回 Windows 会停止 Ubuntu 视觉和 ROS2；不能按上述方式同时验证。可在 Windows 烧录后回 Ubuntu 用板上 USB 快照验收。原生 Windows 视觉/ROS2 或 WSL2 USB 转发需要另做移植部署，本仓库尚未验证，不能仅拉取后直接运行。

## Windows 操作

在原仓库 PowerShell：

```powershell
git status
git pull --ff-only origin main
cd ele-con
mingw32-make -j4
mingw32-make download_jlink
```

保留自己的未提交修改，不强制覆盖。工具路径沿用已通过的 Windows 设置；固件维持 SENTRY_LINK_BENCH_ONLY=1、auto_fire=false，电机和发射机构不接。
本轮主要新增 Ubuntu 配置、脚本及记录，已有 C 板固件协议没变；若重烧，必须选本次成功构建的 HEX/ELF。

烧录后关闭 JFlash，再用 Ozone 连接正确 ELF（build/basic_framework.elf），芯片 STM32F407IG。保持 CPU 运行，勿在任务中断点暂停。
观察 sentry_link_snapshot，设置 Watched Data 的周期刷新；不要观察旧工程 speed_vector。

| 变量 | 预期 |
|---|---|
| vx / vy / wz | 与最终导航 cmd_vel 一致，单位 m/s、m/s、rad/s |
| yaw_rad / pitch_rad | 真实视觉目标角度，绝对弧度 |
| nav_valid | USB 指令持续到达时为 1 |
| vision_valid | 视觉有效帧持续到达时为 1，没目标也可为 1 |
| tracking | 识别并跟踪目标时为 1；移开目标后取消 |
| shoot_requested | 本轮始终 0 |

## Ubuntu 真实视觉先跑

使用已有 Ubuntu 环境，保持镜头前蓝色亮灯装甲板。当前 `auto-aim/configs/sentry_step1.yaml` 是 enemy_color=blue、auto_fire=false。

```bash
cd /home/xiaoli/Desktop/cool-sentry
cd auto-aim
./build_clean/standard configs/sentry_step1.yaml
```

板载姿态必须持续回传，IMU 超时或目标消失应取消控制。当前相机内外参、弹速仍不是本车实测标定，角度解析通过不等于瞄准精度通过。

## Ubuntu 接入真实导航速度

先完成 EW- 定位、地图、重定位和导航工作区依赖，关闭旧 standard_robot_pp_ros2 / serial_only_launch.py USB 通信节点。
不要直接启动未经检查的完整 launch：先核对它是否自动启动旧串口节点、init_spin_speed 是否为 0、地图路径和雷达地址是否正确。
雷达电脑 IP 为 192.168.1.50，雷达为 192.168.1.3；使用本机重建 SDK 的环境，避免恢复旧 spdlog 退出异常。
部署方法见 POINTLIO_UBUNTU_RESULTS.md，重新拉取到别的电脑后需要重新构建，Git 不包含 build_mid360 或 build_clean 二进制。

真实导航开始发布后检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 topic info /red_standard_robot1/cmd_vel -v
ros2 topic echo /red_standard_robot1/cmd_vel
```

上述话题是 EW- 历史记录中的候选，必须以当前最终速度输出为准，通常应选择经过 fake_vel_transform 后准备交电控的输出；不能误接中间控制器速度或默认空 /cmd_vel。
确认发布者只有期望的导航控制节点，没有手工 publisher 或旧驱动自旋。

另一个终端启动桥接（把话题替换为实际确认的最终输出）：

```bash
cd /home/xiaoli/Desktop/cool-sentry
source /opt/ros/humble/setup.bash
source navigation/install/setup.bash
# 本机临时安装的 pyserial；正常安装 python3-serial 后不需要这一行
export PYTHONPATH="$PWD/navigation/install/python_deps:$PYTHONPATH"
ros2 run sentry_cboard_bridge bridge --ros-args \
  -p device:=/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_306F35813034-if00 \
  -p cmd_vel_topic:=/red_standard_robot1/cmd_vel
```

USB by-id 是当前 C 板的地址，换板后核对；/dev/rm_nav 规则安装成功后也可使用。
桥接日志必须显示 Connected。100 ms 话题超时会清零速度，导航的发布频率需要留出余量，不能只发布一次速度。

Ubuntu 查看实际板上回传：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /serial/link_snapshot
ros2 topic echo /serial/cboard_imu
ros2 topic echo /serial/robot_motion
```

快照顺序：[vx,vy,wz,yaw,pitch,nav_valid,vision_valid,tracking,shoot_requested]。
裸板 robot_motion 为零是正常的，它不是导航目标速度；当前没有真实电机轮速反馈。

## 验收顺序和交接记录

1. 真实导航运行且亮灯装甲板被跟踪：Windows 快照和 Ubuntu USB 回传一致，两路 valid=1、tracking=1、fire=0；速度对照最终 cmd_vel。
2. 改变可见目标位置：视觉角度变化，导航速度字段不被覆盖。
3. 停止真实导航速度发布：桥接仍运行，速度清零、nav_valid 可保持 1；视觉继续有效。这验证话题超时。
4. 停止 USB 桥接：100 ms 后 nav_valid=0、速度清零；视觉继续有效。这验证板端 USB 超时。
5. 停止视觉：100 ms 后 vision_valid=0、tracking=0、角度和开火请求清零。

100 ms 是配置阈值，用调试器观察只能证明超时最终发生，不能精确测量延迟。暂停 CPU 会破坏这些验证。
无实车时不能靠下发目标让底盘到达目标；裸板只能验证导航输出经过桥接正确送达。本轮不提供假里程计或假比赛许可来让导航强行运行。

请新增 navigation/WINDOWS_REAL_DATA_RESULTS.md，记录版本、接线、最终话题与发布者、地图/定位状态、板上快照、超时结果及是否出现真实非零导航速度。
若未运行 Nav2 或速度只是手工发布，明确标记“真实导航尚未验证”。提交推送后再回 Ubuntu 继续。

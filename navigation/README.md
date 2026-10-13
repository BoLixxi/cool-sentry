# 导航 USB + 视觉 USART 双链路验收

Windows 按步骤操作请先看 [WINDOWS_DUAL_LINK_TEST.md](WINDOWS_DUAL_LINK_TEST.md)。

本轮继续使用 `new_framework/` 的 basic_framework C 板框架，不换成 StandardRobotpp。
目标是裸 C 板收发与两个上位机程序并行运行。**不是已完成实车导航、云台闭环或自动射击。**

## 1. 接线和分工

```text
小电脑（Ubuntu 22.04 / ROS2 Humble）
  ├─ 海康摄像头 → 视觉程序 → CH340 → C板 USART1（板上 USART2）
  │                                           PA9 TX / PB7 RX，921600 8N1
  └─ 雷达 → 导航 ROS2 → sentry_cboard_bridge → C板原生 USB CDC
                                                 两路在 RobotCMDTask 汇集
```

- 原生 USB 使用 `0x5A` PolarBear 帧，CH340 USART 使用已验证的 `0xA5` SeaSky 帧。
- USART 的 CH340 TX 接 PB7，RX 接 PA9，共地，3.3V TTL，勿接 CH340 VCC；C板独立按原方式供电。
- 导航桥接只传 `vx/vy/wz`，单位 m/s、m/s、rad/s；保留原 49 字节命令封装，其余 27 字节置零。
- C板只从 USB 提取速度。USB 云台、摩擦轮、开火、tracking 字段无论是什么值，都不会覆盖视觉。
- 视觉继续发送绝对 yaw/pitch 弧度、control/shoot。`sentry_link_snapshot.shoot_requested` 只是请求记录，不代表许可或实际开火。
- 两条线路不用互相转接，两程序不用共用一个串口。它们共同接入同一块 C板即可。
- 每个设备只能一个通信进程占用。新桥接运行时，停止 `standard_robot_pp_ros2` / `serial_only_launch.py`；使用双口 probe 时，也停止视觉程序和新桥接。

## 2. 原始包核对结果

- `StandardRobotpp-master.zip`：完整另一套电控，USB协议定义的依据，不整套替换本框架。
- `chuanchaun.zip`：提取的底盘收发测试。原 main 重复初始化 USB，并发送 `(1, .2, -10086.5)` 固定测试反馈；这些做法不移入正式反馈。
- EW- 仓库的串口联调文档有过时描述：当前驱动已调整多个上行 ID；实际 CRC8 覆盖前三字节（含 SOF），CRC16 覆盖除最后两字节以外整帧。
- 原 ROS2 驱动发送线程持续发送最后收到的速度，没有本桥接的速度话题超时保护；收发线程共用命令结构也需要同步。这里新增单线程 ROS2 桥接，避免移入与我们视觉不同的自动瞄准控制依赖。
- 新帧与 `chuanchaun` 原 CRC 查表实现 256 组输入逐字节匹配。

协议：`5A | len(1) | id(1) | CRC8(1) | tick_ms(4 LE) | payload | CRC16(2 LE)`。
CRC8 init FF、反射多项式 8C；CRC16 init FFFF、反射多项式 8408。len 包含 timestamp 和 payload。
下行 id=01，len=43，总长49；速度从帧偏移8开始。

## 3. C板改了哪里

- `modules/navigation/nav_wire.h`：独立帧打包、CRC、按字节流解析，可处理分包、连续包和噪声。
- `modules/navigation/navigation.c`：USB回调收包，按本板 HAL tick 判断超时，反馈发送保护。
- `application/cmd/robot_cmd.c`：初始化导航 USB回调；每轮同时读取视觉和导航快照。
- `modules/imu/ins_task.c`：向 USB 提供真实板载 IMU 四元数和角速度；转换为 ROS ZYX 的 yaw/pitch/roll。原 INS Pitch/Roll 命名不直接用作 ROS 欧拉角。
- `Src/usbd_cdc_if.c`：先消费接收包再重启接收，未枚举时不解引用空指针；发送忙时不覆盖缓存。
- `application/robot_def.h`：默认 `SENTRY_LINK_BENCH_ONLY=1`。
- `application/robot.c`：该模式不注册/运行底盘电机，云台和发射任务仍未启用。
- `Makefile` 加入新模块；CMake 按原递归规则自动包含。
- 原 `servo_motor.h` 的 GCC 私有头改为标准 `stdint.h`，便于 Linux GCC 构建。

调试器观察（**不是旧的 RobotCmdData_t / speed_vector 变量**）：

|变量|正常表现|
|---|---|
|`sentry_link_snapshot.vx/vy/wz`|USB 导航目标速度|
|`sentry_link_snapshot.yaw_rad/pitch_rad`|USART 视觉角度|
|`sentry_link_snapshot.nav_valid`|100 ms 内收到有效USB帧，且USB已枚举|
|`sentry_link_snapshot.vision_valid`|100 ms 内收到有效视觉帧；即使 control=0，通信仍有效|
|`sentry_link_snapshot.tracking`|有效视觉帧且control=1|
|`sentry_link_snapshot.shoot_requested`|有效视觉帧且shoot=1；本轮测试始终0|
|`nav_rx_frames/nav_tx_frames`|USB有效收包/提交发送累计次数|
|`nav_invalid_commands`|CRC正确但速度不是有限值的拒收次数|
|`vision_snapshot`|保留的视觉接收快照|

USB超时只清导航速度；USART超时只清视觉角度、跟踪和开火请求。计时使用本板收包时刻，不依赖小电脑时间戳变化。
桥接节点对 `/cmd_vel` 停更100ms归零，C板对USB停更100ms归零，两层保护。
USB掉线后每秒重试；重连后必须收到新的 cmd_vel，避免重放旧速度。USART原驱动的物理断线自动重连仍未实现，断线后需重启视觉。

反馈：

|上行 ID|内容|ROS话题|
|---|---|---|
|02|真实板载IMU，6个float：yaw/pitch/roll/yaw速率/pitch速率/roll速率|`/serial/cboard_imu`|
|1E|实际底盘速度，3个float，裸板默认0|`/serial/robot_motion`|
|E0（本轮扩展）|5个float速度+视觉角度，4个uint8有效/跟踪/开火标志|`/serial/link_snapshot`|

反馈每10ms轮流提交以上包，约各33Hz，发送忙则跳过，静态缓冲保留至USB完成。实际底盘速度需将来从电机反馈调用 `NavSetMotionFeedback()`，不把目标速度当里程计。
`serial/cboard_imu` 保持独立命名，避免未校准安装坐标就直接替代导航已有IMU；没有裁判系统假数据或虚构云台关节反馈。

## 4. 先在 Windows 烧录并检查

继续使用 `new_framework/WINDOWS_STEP1_TEST.md` 的 GCC + JLink 流程。VSCode终端：

```powershell
cd <你的工程路径>\new_framework
mingw32-make -j4
```

新 ELF/HEX 在 `new_framework/build/`，调试器和烧录器都选本次构建产物，不能用旧缓存。
本次 Linux 已构建的固件在 `new_framework/build_navigation/basic_framework.hex`，匹配 ELF 在同目录；这个目录被gitignore忽略，GitHub拉取后应重新构建。
用较旧 binutils 时 `--no-warn-rwx-segments` 可能不支持；可删除这一纯警告选项或升级工具链。
**不要烧另一个压缩包的完整工程。测试时电机/发射机构断开，先确认 SENTRY_LINK_BENCH_ONLY=1。**

插上C板原生USB与CH340，在设备管理器分别确定两个COM口。可运行：

```powershell
py -3 -m pip install pyserial
cd <你的工程路径>
py -3 navigation\dual_link_probe.py --usb COM6 --vision COM5 --seconds 6 --send-test
```

COM6/COM5只是示例。USB口通过拔插C板原生USB识别，CH340口通过拔插转接器识别。
脚本发送的底盘测试速度为 `(0.2,-0.1,0.3)`，视觉 tracking=1、yaw=.25、pitch=-.5、shoot=0。
0–2秒两路都发；2–3秒只发USART；3秒后两路都停止。USB反馈继续用于观察超时。
通过标准：`BOTH_LINKS`、`NAV_TIMEOUT_ONLY`、`BOTH_TIMEOUT` 三项齐全，最后 `PASS`。
这证明USB能回传本板已经收到的两路指令，不只是上位机在写串口。
不加 `--send-test` 为只接收，仅检查USB诊断与USART四元数反馈。

## 5. Ubuntu 小电脑部署

只需添加这个桥接包到导航 ROS2 工作区，保留 EW- 的定位/建图/Nav2模块。工作区路径下面用 `~/sentry_nav_ws` 举例，可以换成导航同学现有 `EEEWW`。

```bash
sudo apt install python3-serial
mkdir -p ~/sentry_nav_ws/src
ln -s /home/xiaoli/Desktop/sp_vision_25-main/navigation/ros2/sentry_cboard_bridge ~/sentry_nav_ws/src/sentry_cboard_bridge
source /opt/ros/humble/setup.bash
cd ~/sentry_nav_ws
colcon build --symlink-install --packages-select sentry_cboard_bridge
source install/setup.bash
```

更换小电脑后修正软链接的视觉仓库路径；复制的是源码，重新colcon构建，不复用别的电脑install/build。

稳定设备名规则模板在 `deploy/99-rm-links.rules`。当前框架 USB VID:PID 是 **0483:00ca**，不同于对方StandardRobotpp的0483:5740，也不同于JLink。
因此不能仅看 `/dev/ttyACM0` 就认定是C板！先确认 `lsusb` 和拔插变化。

```bash
sudo cp /home/xiaoli/Desktop/sp_vision_25-main/navigation/deploy/99-rm-links.rules /etc/udev/rules.d/99-rm-links.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/rm_nav /dev/rm_serial
```

需要的话加入dialout组并重新登录：`sudo usermod -aG dialout "$USER"`。
多块C板或多个CH340需按设备序列号/物理端口细分规则，模板只适用于一块C板和一个CH340。

先关闭所有串口程序，验收裸板：

```bash
cd /home/xiaoli/Desktop/sp_vision_25-main
python3 navigation/dual_link_probe.py --usb /dev/rm_nav --vision /dev/rm_serial --seconds 6 --send-test
```

验收通过后停止probe，开导航桥接：

```bash
source /opt/ros/humble/setup.bash
source ~/sentry_nav_ws/install/setup.bash
ros2 run sentry_cboard_bridge bridge --ros-args -p device:=/dev/rm_nav -p cmd_vel_topic:=/cmd_vel
```

另一个终端先用手工速度验证，停止publisher后100ms应看到诊断速度归零：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: -0.1}, angular: {z: 0.3}}'
```

查看反馈：

```bash
ros2 topic echo /serial/link_snapshot
```

数组顺序：`[vx,vy,wz,yaw,pitch,nav_valid,vision_valid,tracking,shoot_requested]`。

再启视觉（配置保持 auto_fire=false）：

```bash
cd /home/xiaoli/Desktop/sp_vision_25-main
./build_clean/standard configs/sentry_step1.yaml
```

这个`standard`才有C板视觉收发；`camera_detect_test`只做图像识别，不发送视觉控制。
若要窗口测试，可用本项目已有的视觉调试参数，先执行 `./build_clean/standard --help` 查看。

最后在导航工作区运行原实机导航 launch：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py
```

前提：EW-导航源码和依赖已构建，MID360网络/地图配置正确，定位与TF完整。只添加桥接包不会自动安装或配置整个导航工程。
**不要同时启动原 `standard_robot_pp_ros2` 串口节点。**
用 `ros2 topic info /cmd_vel -v` 确認导航是publisher、新桥接是subscriber；不能再把手工publisher与Nav2同时留在同一速度话题。

## 6. 实车前还要完成的工作

1. 确认USB接到云台板还是底盘板：当前按已有ONE_BOARD单板框架实现。若改双板，需设计CAN转发归属。
2. 确认速度坐标。ROS常用x前/y左/z上。EW-配置为gimbal_yaw/gimbal_yaw_fake，最终cmd_vel经过fake_vel_transform；原电控注释写y向右，且底盘还做offset_angle转换。实际轮子和云台确定后才接入电机，不能简单三项直接赋值。
3. 当前导航默认 `init_spin_speed: 3.14`，测试前在导航实机配置设为0；新桥接默认限制线速度范数2.5m/s、角速度3rad/s，可通过max_linear/max_angular设定。
4. 确认相机、板载IMU、雷达安装坐标与云台关节TF。板载IMU若在云台上，不是底盘姿态。
5. 将实测轮速反馈接到NavSetMotionFeedback；确认底盘单位转换与姿态变换只发生一次。
6. 裁判系统比赛状态、机器人ID、热量/弹速/发射许可尚未通过新USB桥接传输，需按实际导航决策依赖逐项补齐，不返回虚构比赛开始/许可数据。
7. 将快照接到真实底盘/云台控制仲裁；开火请求必须再与比赛状态、机构就绪、热量、超时等条件合并。目前不会由通信快照驱动这些机构。

USB CDC可继续使用，无需为了这轮集成换USART。实车要做连接器固定、USB/USART同时长时间运行、负载延迟和掉线恢复测试后才能评价整车稳定性。

## 7. 已执行的软件验证（2026-10-13）

- PolarBear CRC与提供的原包查表实现，256组帧逐字节一致。
- C板USB模块主机模拟：分包、CRC错误、有限值检查、100ms超时、USB掉线、HAL tick回绕、IRQ状态恢复、忙时不覆盖TX缓冲、两路快照互不覆盖通过。
- 原视觉模块主机模拟回归通过。
- Python协议4项测试通过。
- ROS2 Humble colcon构建通过；真实ROS2节点+PTY：速度话题→49字节命令、USB反馈→ROS话题、话题断流归零、Ctrl+C退出通过。
- ARM GCC10.3.1完整固件编译链接通过，使用临时工具链；去掉旧binutils不支持的纯警告链接选项。最终FLASH和RAM占用见构建日志。
- **尚未烧录此版固件、未实板验证USB+USART同时收发、未连接实车或雷达测试自主导航。**

复现基础测试：

```bash
python3 -m unittest discover -s navigation/tests -v
cc -std=c11 -Wall -Wextra -Werror -Inew_framework/host_tests/stubs -Inew_framework/modules/master_machine new_framework/host_tests/navigation_module_test.c -lm -o /tmp/nav_module_test
/tmp/nav_module_test
```

ROS2 PTY测试需source桥接install环境、安装pyserial后运行 `python3 navigation/tests/ros_bridge_pty_check.py`。
原包CRC比对测试源码为 `tests/reference_crc_test.c`，需链接提供的CRC8_CRC16.c（其大小写include在Linux需修正；附加 `-include stddef.h`）。

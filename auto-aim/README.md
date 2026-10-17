# RoboMaster 单相机装甲板自瞄

Ubuntu 接力验证已完成真实姿态接收、海康采集与模型运行，结果和下一步见 [UBUNTU_STEP1_RESULTS.md](UBUNTU_STEP1_RESULTS.md)。本机优先使用完整 `/opt/MVS/lib/64` 海康 SDK，其他安装位置可设置 CMake 参数 `HIKROBOT_SDK_LIB_DIR`。

新电控框架位于相邻的 `../ele-con/`，学习与迁移见 [USART 重写指南](../ele-con/VISION_PROTOCOL.md)。对应视觉配置为 `configs/sentry_step1.yaml`，使用 SeaSky 帧、921600 波特率，默认关闭自动开火。下面的旧 USART 协议说明对应 `standard3.yaml` 和旧电控工程。

基于 [同济 SuperPower 开源工程](https://github.com/TongjiSuperPower/sp_vision_25) 整理，面向本队 3V3 / 1V1 使用需求：保留装甲板识别、目标跟踪、弹道预测、射击判断、单海康 USB 相机和 C 板 USART 通信。

原作者许可证保留在仓库根目录的 `../LICENSE`。本次整理沿用当前工作目录中的参数和 USART 帧格式，没有用原 ZIP 覆盖你的修改。

## 编译与运行

依赖：C++17、CMake、OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann-json、OpenVINO Runtime、libusb，以及 `io/hikrobot/` 内的海康 SDK。

```bash
cmake -S . -B build_clean -DBUILD_DEBUG_TOOLS=ON
cmake --build build_clean -j2
./build_clean/standard configs/standard3.yaml
```

以上命令在 `auto-aim/` 目录运行，模型和配置中的相对路径以该目录为基准。只编译实车主程序可以使用 `-DBUILD_DEBUG_TOOLS=OFF`。OpenVINO 通过 CMake 自动查找；找不到时传入 `-DOpenVINO_DIR=/实际安装目录/runtime/cmake`。

## 学习顺序

1. `src/standard.cpp`：完整自瞄流程，只有一个相机实例，每帧发送一次指令。
2. `io/command.hpp`、`io/cboard.*`：视觉与 C 板收发的数据。
3. `tasks/auto_aim/armor.*`、`yolo.*`、`yolos/`：装甲板类型与 YOLO 检测。三种模型都是装甲板模型，保留供比较；默认使用现有的 YOLOv5。
4. `detector.*`、`classifier.*`：传统灯条检测及数字分类，仍是 YOLO 精修/分类的依赖。
5. `solver.*`：PnP、相机/云台/世界坐标变换。
6. `tracker.*`、`target.*`：目标状态机、EKF 和旋转装甲板预测。
7. `aimer.*`、`tools/trajectory.*`、`shooter.*`：弹道补偿、提前量和射击条件。

细分目录见 [CODE_STRUCTURE.md](CODE_STRUCTURE.md)。自瞄的装甲板类别和相关目标模型沿用原工程，包括基地/前哨站类别；没有删除模型输出类别，以免破坏类别映射。

## 配置与调试

实车配置为 `configs/standard3.yaml`；`configs/camera.yaml` 只配置海康相机；`configs/calibration.yaml` 用于标定；`configs/demo.yaml` 仅用于随附视频回放，不能当作实车标定结果。

```bash
# 仅查看相机，不发送云台或开火指令
./build_clean/camera_test -d
# 相机 + 装甲板检测，不需要 C 板
./build_clean/camera_detect_test configs/standard3.yaml
# 仅查看 C 板姿态，不发送云台或开火指令
./build_clean/cboard_test configs/standard3.yaml
# 视频检测
./build_clean/detector_video_test assets/demo/demo.avi -c=configs/demo.yaml
# 带四元数记录的离线自瞄回放
./build_clean/auto_aim_test
# 没有图形桌面的环境
./build_clean/auto_aim_test --display=false -e=30
```

`tests/` 中这些程序是交互式调试工具，不是自动单元测试。`calibration/` 保留内参、手眼标定和采图工具。相机内参、手眼外参、云台轴方向和偏移需要针对自己的机器人核实；本次整理没有重新标定。

## USART 协议

已对照你提供的电控工程 `Application/ins_task.c` 核对帧格式。数值按当前支持的 x86_64/aarch64 平台的小端编码。旧电控接口问题和修正建议已归档到 [FIRMWARE_INTERFACE.md](../docs/legacy/FIRMWARE_INTERFACE.md)。

接收使用电控发送函数定义的 19 字节格式；末字节是累加校验，没有固定帧尾。`receive_frame_size: 18` 仅用于兼容其他不带校验的旧固件，本队配置保持 19。

| 偏移 | 内容 |
| --- | --- |
| 0..1 | `55 AA` |
| 2..17 | `x, y, z, w`，4 个 float32 四元数分量 |
| 18 | 字节 0..17 累加后取低 8 位，接收必须校验 |

发送共 16 字节：

| 偏移 | 内容 |
| --- | --- |
| 0..2 | `AA 55 01` |
| 3..4 | control，int16，0/1 |
| 5..6 | shoot，int16，0/1 |
| 7..10 | yaw，float32，弧度 |
| 11..14 | pitch，float32，弧度 |
| 15 | 字节 2..14 累加后取低 8 位 |

接收帧只有姿态，因此弹速来自 YAML 的 `bullet_speed`（m/s），默认 23，必须改成实际测量值。当前帧没有电控模式或敌方颜色，敌方颜色来自 `enemy_color`；主程序持续运行自瞄。`auto_fire` 决定是否发送开火请求，正式开火仍需 C 板配合。

串口配置键为 `serial_port`、`baudrate`、`serial_timeout_ms`、`receive_frame_size`、`max_imu_age_ms`。支持旧配置中的 `can_interface` 作为串口路径，但已经不使用 CAN ID。启动未收到姿态会报超时；运行中姿态断流会在等待超时后停止本帧控制并发送取消指令。超过 `max_imu_age_ms`（默认 100 ms）的旧姿态拒绝使用。四元数模长必须位于 0.8 到 1.2，再做归一化，过滤零值、NaN 和明显损坏的数据。

累加校验可过滤部分损坏帧，但没有 CRC、序号或电控时间戳，仍无法检测所有错误或准确还原采样时刻。后续增加这些字段必须同步修改电控固件。发送侧检查完整写入，但写入成功只说明字节已交给系统，不代表电控已经接收或执行。当前电控解析函数未接入云台和射击任务；接入时必须设置启用条件和指令超时保护。

无需硬件的 USART 测试：

```bash
python3 tests/cboard_serial_test.py ./build_clean/cboard_serial_probe
```

测试覆盖 18/19 字节格式、各位置分片、大批量积压、噪声、无效四元数、坏接收校验、发送字节/校验、旧姿态拒绝和断流超时。虚拟串口不模拟实际线路的干扰、USB 转串口断连和电控执行，实板仍需验证。

## 导航接口

当前正式导航接口不再放在自瞄工程内部。导航 ROS2 桥接、MID360/Point-LIO 配置和验收工具统一位于 `../navigation/`。原开源但未实际接入的 `io/ros2/` 已归档到 `../docs/legacy/io_ros2/`，不参与构建。

## 本次整理与验证

移出能量机关、全向多相机感知、无人机/哨兵入口、MPC、多线程自瞄变体、其他相机驱动、独立云台/IMU 驱动，以及旧构建和日志。保留 Git 历史和原作者许可证。

修复主程序重复发送和未接入射击判断、USART 未对齐数值读写、接收缓冲区越界、无效四元数以及姿态断流无限等待。射击判断补充 pitch 容差；弹速改为明确配置。

验证：当前机器上完整构建通过；虚拟串口验证分片/噪声接收、零四元数拒绝、发送帧字节和校验、姿态超时通过；无窗口离线回放 31 帧运行通过。回放出现过目标发散日志，跟踪参数和精度仍需后续检查。尚未完成海康实拍、实板 USART、导航或实弹验证。

整理前源码与参数备份：`/tmp/sp_vision_before_cleanup_20261010.tar.gz`。移出的文件（含旧运行产物）：`/tmp/sp_vision_removed_20261010/`。这两处位于临时目录，可能被系统清理；Git 中已跟踪内容仍可从历史恢复，用户给出的原 ZIP 保持不变。

导航 USB 与视觉 USART 双链路移植、Windows 裸板验收和小电脑部署见 [navigation/README.md](../navigation/README.md)。

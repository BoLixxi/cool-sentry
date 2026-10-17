# 步骤 1：重写视觉 USART 通信

Windows VSCode＋GCC 的编译、下载、串口接线和实板验收步骤见 [WINDOWS_STEP1_TEST.md](WINDOWS_STEP1_TEST.md)。无需安装完整视觉依赖，可使用 `host_tests/windows_serial_probe.py` 模拟视觉指令。

这是用户提供的 basic_framework 压缩包的工作副本，原压缩包未修改。旧 sentry-master 的补丁不适用于此框架。

## 先理解数据流

电控 INS_Task 解算四元数 → VisionSetQuaternion → VisionSend → USART1 TX DMA → 上位机 CBoard → imu_at → 视觉解算。

视觉 Command → CBoard::send → USART1 RX DMA/IDLE → DecodeVision → VisionGetCommand → 后续控制任务。

这一步只完成数据传输。云台、拨弹盘仍未接入视觉命令；原框架电机任务、底盘和遥控器代码也不是本队已经调好的机器人参数。不要把整个框架直接烧录到带负载的机器人上当作已经调好的固件。

## 协议：两端都以共享接口为准

协议代码统一放在仓库根目录的 `interfaces/vision_wire.h`。自瞄和电控共同引用这一份文件，修改帧格式时必须同时运行两端测试。

使用 SeaSky 的帧结构，但重新约定本队第一版载荷；不是原框架未完成的欧拉角/标志位实现。

| 偏移 | 内容 |
|---|---|
| 0 | A5 |
| 1..2 | data_length，小端 uint16，包含 flags 和 float 载荷 |
| 3 | 前 3 字节 CRC8，初值 0，多项式 0x31，无反射 |
| 4..5 | command_id，小端 uint16 |
| 6..7 | flags，小端 uint16 |
| 8 起 | float32，小端 IEEE754，4 字节/数值 |
| 最后 2 字节 | CRC16/MODBUS，初值 FFFF，多项式 A001，小端；覆盖前面的全部字节 |

电控 → 视觉：命令号 2，data_length=18，总长 26。flags 必须为 0；载荷 x/y/z/w 四元数（INS 原始顺序 w/x/y/z 在发送前转换）。

视觉 → 电控：命令号 1，data_length=10，总长 18。载荷 yaw/pitch，单位弧度，绝对瞄准角，不能直接当角度增量使用。flags bit0=control，bit1=shoot，其余为 0。允许 flags=0/1/3，禁止 control=0、shoot=1。shoot 是请求，后续电控仍需根据比赛状态、热量和弹量决定执行。

弹速和敌方颜色本轮继续使用视觉 YAML。比赛阶段、阵营、弹速字段下一步再约定，不占用当前保留位。四元数对应的安装轴、参考系和 pitch 符号仍需实车确认。

串口：USART1，921600、8N1，TX PA9、RX PB7。板上接口丝印请对照具体 C 板，不要仅凭名称猜引脚。USB 转串口使用 3.3 V TTL，TX/RX 交叉、GND 共地；适配器必须支持所选波特率。若改 115200，应同步修改 Src/usart.c、CubeMX 的 .ioc 和视觉 YAML。

## 建议学习与修改顺序

1. 看 `../interfaces/vision_wire.h`：先找到 VisionPack，再看 CRC8/CRC16，最后看 VisionFeed。DMA 回调收到的只是字节片段，不保证刚好是一整帧，因此按字节累积；长度、ID、校验错误时滑动一字节重新找 A5。
2. 看 bsp/usart/bsp_usart.h/.c：新增 recv_len 保存实际接收数，供模块读取；USARTIsReady 改为检查 HAL_UART_STATE_READY。框架为普通 DMA，每次 IDLE/满缓冲回调后重新启动，重启间隙仍可能丢字节；本协议可以重新同步，长期压力测试后再决定是否改循环 DMA。
3. 看 master_process.c：DecodeVision 只在完整有效帧后更新命令和接收时间；VisionGetCommand 在短临界区复制快照，100 ms 没有有效帧则返回全零，避免控制任务读到不同帧混合的字段。调用者必须每周期调用这个接口，不要保存 VisionInit 返回指针后一直读旧值。
4. 看 modules/imu/ins_task.c：改用 VisionSetQuaternion(INS.q)。INS 仍按原调度解算；VisionSend 最多 100 Hz，使用静态发送缓冲，只有 DMA 空闲时才打包下一帧。不要在 INS 任务用 printf 或阻塞发送二进制。
5. 看 application/cmd/robot_cmd.c：每周期调用 VisionGetCommand 更新快照。本轮没有把这些角度写进电机控制器。
6. 看 application/robot_def.h：启用 VISION_USE_UART，关闭 VISION_USE_VCP。
7. 视觉端 `../auto-aim/io/cboard.cpp` 支持 `serial_protocol: seasky_v1`；旧协议默认 legacy19，旧配置仍可使用。新配置 `../auto-aim/configs/sentry_step1.yaml` 为 921600，auto_fire=false。

## 在 Windows 迁移

可以使用本目录作为新工程副本；若迁回你已经修改过的 Windows 工程，只合并下列文件的修改：

- 新增共享协议 `../interfaces/vision_wire.h`，并保持它与 `ele-con/` 为相邻目录。
- 替换 master_process.c，合并 master_process.h 的接口和容量定义。
- 合并 bsp/usart/bsp_usart.h/.c、modules/imu/ins_task.c、application/cmd/robot_cmd.c、application/robot_def.h。
- Makefile 已移除本模块不再调用的 seasky_protocol.c 源文件条目；CMake 仍可能编译这个旧文件，但新实现不调用它。

本仓库提供的是 GCC 的 CMake/Makefile 工程。若你用 Keil，需要把 `interfaces/vision_wire.h` 加入自己的头文件搜索路径，并加入上述电控源文件；不是直接打开这里的 CMakeLists。host_tests 是 Linux 主机模拟测试，不加入 ARM 工程；主机桩不能替代 STM32 HAL。

## 验证顺序

先在 Windows 编译电控工作副本并处理编译错误；当前环境没有 arm-none-eabi-gcc，整板编译与烧录尚未完成。

板上调试先检查 IMU 初始化与四元数值，再观察 tx 缓冲：应是 26 字节，以 A5 12 00 开头，命令号 02 00。视觉指令应是 18 字节，以 A5 0A 00 开头，命令号 01 00。CRC 字节随载荷变化，不能写死为帧尾。

连接上位机后先进入相邻的自瞄目录，运行只读姿态工具（无需相机）：

```bash
cd ../auto-aim
./build_clean/cboard_test configs/sentry_step1.yaml
```

随后可用 cboard_serial_probe 对实板发一次 control=1、shoot=0、yaw=0.25、pitch=-0.5 的测试命令；它原本是自动测试程序，若板持续发姿态，它的断流测试会失败，这是预期情况。实板应通过调试器查看 VisionGetCommand 的输出；更推荐下面的模拟测试先掌握协议。

主机测试：

```bash
gcc -std=c11 -Wall -Wextra -Werror host_tests/vision_wire_test.c -o /tmp/vision_wire_test
/tmp/vision_wire_test
gcc -std=c11 -Wall -Wextra -Werror -I../interfaces -Ihost_tests/stubs -Ibsp/usart host_tests/vision_module_test.c -lm -o /tmp/vision_module_test
/tmp/vision_module_test
```

上述命令在 `ele-con/` 目录运行。视觉测试改在 `auto-aim/` 目录运行：

```bash
cd ../auto-aim
python3 tests/cboard_serial_test.py ./build_clean/cboard_serial_probe
```

验收：控制/角度精确解码；乱码、CRC 错误不会刷新有效命令时间；分片和连续帧可恢复；断流 100 ms 后调用 VisionGetCommand 获得无目标、不开火；DMA 发送期间缓冲不被改写。主机测试已通过，但不模拟 HAL IRQ 时序、真实串口干扰和转接器断连。本轮未实现串口物理断连自动重连；发生接收线程错误需要重新启动视觉程序。

## 导航 USB 双链路扩展

当前新增导航 USB CDC，与本视觉 USART 独立。裸板模式默认 `SENTRY_LINK_BENCH_ONLY=1`，不会注册或运行底盘电机。收发协议、快照变量与实板验收步骤见 [navigation/README.md](../navigation/README.md)。

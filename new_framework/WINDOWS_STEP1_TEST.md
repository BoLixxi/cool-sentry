# 3V3 哨兵：Windows 下验证第一步 USART 通信

## 1. 在哪里做

建议回到你已配置的 Windows VSCode＋GCC 环境，完成电控编译、下载和实板串口测试。Linux 继续用于视觉工程编译、离线和虚拟串口测试，不必先在 Linux 重装一套电控工具链。

第一步不需要海康相机，也不需要完整视觉依赖。Windows 上用 Python 脚本模拟视觉发送命令，用调试器核对电控解析结果。

本轮已经完成：Linux 视觉构建、两端协议模拟、电控模块主机测试。尚未完成：ARM 整板构建、烧录、真实 HAL/DMA 和线路验证。下面操作是待执行的验证流程，不能把已有模拟测试当成上板成功。

## 2. 把哪份代码带回 Windows

复制当前整个 `new_framework` 文件夹到 Windows，例如 `D:\RM\sentry_step1`。这是修改后的电控工作副本，包含隐藏的 `.vscode` 文件夹；不要重新解压原 ZIP 后误用未经修改的版本。暂不覆盖你原来已经调好的工程。

在 VSCode 中选择“文件 → 打开文件夹”，打开 `D:\RM\sentry_step1`，这层应能直接看到 `Makefile`、`Src`、`modules` 和 `.vscode`。

这是示例路径，下面命令中的路径请替换成你实际使用的路径。建议工程路径不要含空格或中文，以避开原 Makefile 的路径转义问题。

本次修改集中在：

| 文件 | 作用 |
|---|---|
| `modules/master_machine/vision_wire.h` | SeaSky 打包、CRC8/CRC16、逐字节重同步 |
| `modules/master_machine/master_process.c/.h` | 收发、100 ms 超时、命令快照、DMA 缓冲 |
| `bsp/usart/bsp_usart.c/.h` | 实际接收长度、发送空闲判断 |
| `modules/imu/ins_task.c` | 向视觉模块提供四元数 |
| `application/cmd/robot_cmd.c` | 每周期读取命令快照 |
| `application/robot_def.h` | 使用 UART，关闭 VCP |
| `host_tests/windows_serial_probe.py` | Windows 实板串口测试工具 |

`host_tests` 不加入 MCU 编译源文件；里面的桩文件只给主机模拟测试使用。旧 `master_process.md` 描述的是原框架，本轮请以 `VISION_PROTOCOL.md` 和本指南为准。

## 3. 检查已有工具链

在 VSCode 终端中选择 **Command Prompt（cmd）**，进入工程目录：

```bat
cd /d D:\RM\sentry_step1
where arm-none-eabi-gcc
arm-none-eabi-gcc --version
where arm-none-eabi-gdb
where mingw32-make
mingw32-make --version
```

这些命令能找到程序并显示版本即可。注意 MCU 编译用的是 `arm-none-eabi-gcc`，不是编译 Windows 程序的 `gcc`。若找不到，把已安装工具链的 `bin` 加入 PATH，再完全重启 VSCode。还未安装时，使用 Arm 官方 GNU Toolchain（Windows 主机、arm-none-eabi 目标）和 MinGW 的 make。

VSCode 安装 Microsoft C/C++ 和 Cortex-Debug。调试器环境按硬件二选一：

- J-Link：安装 SEGGER J-Link 软件包，确认 `JLinkGDBServerCL.exe` 能找到。
- CMSIS-DAP：安装 OpenOCD，确认 `openocd --version` 正常；本工程已有 `openocd_dap.cfg`。

本轮按已有 Makefile 构建，不需要另外配置 CMake/Ninja。若你已经用 CMake 构建成功也可沿用，但不要把它生成的 ELF 和默认 `build\basic_framework.elf` 混用。

## 4. 先编译，不下载

```bat
mingw32-make -j4
```

成功应生成：

```text
build\basic_framework.elf
build\basic_framework.hex
build\basic_framework.bin
```

遇到错误就停在这一步，保存第一条实际错误及前后上下文，不要只截最后的 `Error 1`，也不要跳过错误下载旧 ELF。框架开启了 `-Werror`，warning 也可能阻止构建；优先定位问题，不要为通过构建统一关闭告警。

常见情况：

| 报错 | 处理 |
|---|---|
| 找不到 `arm-none-eabi-gcc` | 检查 PATH，重启 VSCode |
| 找不到 `libCMSISDSP.a` | 确认复制了 `Middlewares\ST\ARM\DSP\Lib`，不是只复制源代码 |
| `unrecognized option --no-warn-rwx-segments` | 旧链接器可能不支持原框架选项；优先使用兼容的新工具链，或仅移除该告警抑制选项 |
| 视觉函数未定义/头文件找不到 | 确认使用修改后的工程，`master_process.c` 在 Makefile 源文件清单中 |
| VSCode 波浪线，但终端构建成功 | 是 IntelliSense 配置问题，先区分实际编译器错误 |

如需清理本副本构建产物，cmd 下可以执行 `mingw32-make clean`，然后重新构建；它会删除本目录的 `build`，先确认工作目录正确。

## 5. 下载与初始化检查

原框架仍包含底盘、电机和遥控器任务，并非你们整车已调好的固件。本轮采用单独的 C 板台架测试，断开电机 CAN 或电机动力连接，保留 IMU、板供电、SWD 和串口；USB 转串口不负责给整车供电。

把下载器的 SWDIO、SWCLK、GND、参考电压按其接线规范接好。参考电压检测与板供电要区分，避免多个供电源冲突。

优先按 F5 下载并调试：

1. 在“运行和调试”中选 `Jlink` 或 `DAPlink`，与实际下载器一致。
2. 配置的 ELF 为本工程 `build\basic_framework.elf`，设备是 STM32F407IG，接口为 SWD。
3. F5 执行构建并启动，默认停在 `main`；此时还没有运行 INS 或发串口数据。
4. 继续运行，确认没有卡在 `HardFault_Handler`、`Error_Handler` 或初始化循环中。

如果 Cortex-Debug 提示找不到工具，可在自己的 `launch.json` 调试项中配置 `armToolchainPath`，以及匹配下载器的 `serverpath`；填写本机真实路径，不要照搬他人安装目录。

原 `.vscode/tasks.json` 的下载任务用分号串接命令，可能编译失败后仍尝试下载。第一次验证用 F5 或手工分开执行构建和下载，并确认构建成功。DAP 也可以在成功构建后执行 `mingw32-make download_dap`；J-Link 推荐先用 F5，原 `download_jlink` 依赖本机 JFlash 和项目配置，不保证能直接使用。

检查这些运行现象：

- `INS_Task` 能持续执行，未卡在 BMI088 初始化。
- `VisionSetQuaternion` 能被调用，四元数分量有限，模长接近 1。
- `VisionSend` 能成功启动 DMA，`HAL_UART_Transmit_DMA` 返回 HAL_OK。

第一次可短暂停机看值；后续测试时必须继续运行。一直打断点会暂停收发并改变超时结果。

## 6. 接 USB 转 TTL，先只接收

本工程 USART1：921600、8N1，TX=PA9、RX=PB7。请按具体 C 板原理图确认接口引出位置，框架提到的丝印 USART2 不一定与你的板一致。

| C 板 | USB 转串口 |
|---|---|
| TX（PA9） | RX |
| RX（PB7） | TX |
| GND | GND |

使用 3.3 V TTL 转接器，不用 RS232 电平；本步骤不连接转接器的 VCC。适配器应支持 921600；若改成 115200，需要同时改电控 USART 初始化、CubeMX `.ioc` 及上位机配置。

打开 Windows 设备管理器，在“端口（COM 和 LPT）”中找到转接器，如 COM5。关闭占用此 COM 的串口助手；调试器通过 SWD 工作，可同时保留。

Windows 安装 Python 后，在工程 cmd 终端执行：

```bat
py -3 --version
py -3 -m pip install pyserial
py -3 host_tests\windows_serial_probe.py --self-test
py -3 host_tests\windows_serial_probe.py --port COM5 --seconds 10
```

没有 `py` 时用实际可用的 `python` 替换。`--self-test` 不需要硬件，应该输出 PASS；最后一条命令只接收，不发送云台或开火请求。

预期每秒打印累计有效帧数、拒绝帧数和最新 `xyzw`。静止时四元数应稳定，转动 C 板后应变化。100 Hz 是发送上限，受任务调度影响不保证恰好每秒 100 帧，但有效帧应该持续增加。

这只能证明板向电脑发送成功，尚未验证电脑向板发送。

没有有效帧时依次检查：MCU 是否还停在断点、正确 COM、TX/RX 是否交叉、是否共地、两端波特率、是否使用本轮 26 字节四元数协议。不要把二进制四元数按文本显示后看到乱码当成错误。

## 7. 再测试接收与超时

在电机隔离的台架上运行：

```bat
py -3 host_tests\windows_serial_probe.py --port COM5 --seconds 15 --send-test
```

脚本循环三个阶段，所有阶段都不请求开火：

| 阶段 | 脚本动作 | 电控预期 |
|---|---|---|
| TRACK，2 秒 | 约 50 Hz 发 control=1、shoot=0、yaw=0.25、pitch=-0.5（弧度） | AUTO_AIM、TARGET_CONVERGING，角度正确 |
| SILENCE，1 秒 | 暂停发送，但继续接收姿态 | 100 ms 后读取接口返回 NO_FIRE、NO_TARGET |
| CANCEL，2 秒 | 约 50 Hz 发 control=0、shoot=0、角度=0 | 立即取消视觉目标和开火请求 |

调试器观察 `application/cmd/robot_cmd.c` 的 `vision_snapshot`，这是控制任务读取后的快照。可用 Live Watch；若无法查看，可在 `VisionGetCommand` 调用后短暂停机观察，但暂停时不要根据时间推断实际周期性能。

关键判断：

- TRACK：`vision_snapshot.yaw` 约为 0.25，pitch 约为 -0.5；fire_mode 为 AUTO_AIM（2），target_state 为 TARGET_CONVERGING（1）。
- SILENCE：超过 100 ms 后，快照 fire_mode=NO_FIRE（0）、target_state=NO_TARGET（0），角度清零。
- CANCEL：快照立即变成无目标、不开火。
- `master_process.c` 的 `latest` 是最后一次成功接收的原始值，超时后可能仍保留旧值，不能拿它当超时保护后的输出；应看 `VisionGetCommand` 输出或 `vision_snapshot`。

拔掉串口转接器时，只要电控任务还在运行并持续调用 `VisionGetCommand`，超过 100 ms 也应返回无目标、不开火。重新接上后重新打开正确 COM，再确认新有效命令可以更新。此测试验证电控超时，不代表 Linux 视觉程序已经实现 USB 断连自动重连。

本轮视觉命令尚未接入电机和发射任务，看到解析值正确即可，不以云台运动作为第一步验收标准。

## 8. 需要保存的结果

记录以下内容，便于继续定位或进入下一步：

```text
ARM GCC 版本：
构建是否成功：
下载器型号：
USB 转串口型号 / COM：
波特率：
10 秒有效帧 / 拒绝帧数：
静止和转动时四元数表现：
TRACK 的 yaw / pitch / fire_mode / target_state：
SILENCE 超时是否清零：
CANCEL 是否立即生效：
拔线、重连结果：
第一条编译或运行错误（如有）：
```

全部通过后再回 Linux 接 C 板，运行只读姿态工具：

```bash
./build_clean/cboard_test configs/sentry_step1.yaml
```

确认 Linux 串口路径正确、波特率一致。本轮仍不需要海康相机；随后才接裁判系统比赛状态和阵营，再验证相机标定、云台轴方向与控制。

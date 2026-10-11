# 视觉通信联调交接：Windows → Ubuntu

记录日期：2026-10-11

## 1. 当前结论

Windows 裸板阶段已完成实板双向通信验证。使用 C 板、J-Link 和 USB 转 TTL，不需要摄像头。

当前工作工程：`D:\autoshot_rm\cool-sentry\new_framework`。

原工程参考：`D:\RM\New\basic_framework-master`。原工程目前开启 `HERO_CAN_AUDIT_MODE=1`，会跳过 IMU 和视觉任务初始化，不作为本次视觉通信固件使用。

本次成功验证的是当前 `new_framework` 固件，不是原工程的 CAN 观察固件。

## 2. 已完成的实板验证

### 电控 → Windows

端口：COM7；波特率：921600；格式：8N1。

- 只读测试：5 秒收到 500 个有效姿态帧，拒绝帧为 0。
- 收发并行测试：15 秒收到 1500 个有效姿态帧，拒绝帧为 0。
- 发送速率约 100 Hz，四元数通过有限值和模长检查。

### Windows → 电控

通过 J-Link 直接读取板上 `vision_snapshot`，而不是仅根据串口写入成功判断接收成功。

| 测试阶段 | 实际读取结果 | 结论 |
|---|---|---|
| TRACK | yaw=0.25、pitch=-0.5；fire_mode=2、target_state=1 | 弧度角和跟踪状态解析正确 |
| SILENCE | 停止发送后 160 ms 检查，角度和状态全零 | 配置为 100 ms 的超时保护已生效；未精确测量触发时刻 |
| TRACK restored | 恢复发送后重新得到相同角度及状态 | 超时后可恢复 |
| CANCEL | control=0 后 20 ms 检查，角度和状态全零 | 显式取消生效；未精确测量处理延迟 |

J-Link 还核对了板上向量表及视觉函数位置的字节与当前构建固件一致。

全程 `shoot=0`，没有测试开火。该结果证明通信和命令快照正常，不能等同于云台控制、相机精度或射击已验证。

## 3. 当前协议，两端必须一致

视觉配置：`configs/sentry_step1.yaml`。

```yaml
serial_protocol: seasky_v1
serial_port: "/dev/rm_serial"
baudrate: 921600
auto_fire: false
bullet_speed: 23.0
serial_timeout_ms: 1000
max_imu_age_ms: 100
```

`bullet_speed` 和 `enemy_color` 目前来自 YAML；弹速 23.0 是已有配置，正式使用前应替换为实测值。

| 方向 | 总帧长 | 帧头及长度 | 命令号 | 载荷 |
|---|---|---|---|---|
| 电控 → 视觉 | 26 字节 | A5 12 00 | 02 00 | flags=0；float32 x/y/z/w 四元数 |
| 视觉 → 电控 | 18 字节 | A5 0A 00 | 01 00 | flags；float32 yaw/pitch，弧度绝对角 |

- 小端编码，使用 SeaSky 帧结构。
- 第 3 字节是头部 CRC8；最后 2 字节是 CRC16，不是固定帧尾。
- flags bit0=control，bit1=shoot；只允许 0、1、3。
- 不允许 control=0、shoot=1。
- 电控 INS 内部顺序 w/x/y/z，发送前转换为 x/y/z/w。
- 无有效命令超过 100 ms，`VisionGetCommand()` 返回无目标、不开火、角度清零；调用方必须每周期读取该接口。
- `latest` 是原始最后接收值，不能用它判断超时输出，应该检查 `vision_snapshot`。

不要使用 `configs/standard3.yaml` 对接当前固件：它对应旧的 legacy19 协议，波特率 115200，且已有配置开启自动开火。

详细定义见 `new_framework/VISION_PROTOCOL.md`。

## 4. 接线

USART1：TX=PA9、RX=PB7。接口引出位置需按具体 C 板原理图核对。

| C 板 | USB 转 TTL |
|---|---|
| PA9 / TX | RX |
| PB7 / RX | TX |
| GND | GND |

使用支持 921600 的 3.3 V TTL 转接器，不能使用 RS232 电平。本次不连接转接器 VCC，板子单独供电。

COM7 是 Windows 串口名，在 Ubuntu 下会变为 `/dev/ttyUSB*`、`/dev/ttyACM*` 或固定设备链接，必须重新确认。

## 5. Windows 工程已修复的环境与烧录链

### 构建任务

VSCode 原 shell 任务把 CMD 的 `/d /c` 传入 MSYS2，报 `/d: Is a directory`，这是终端启动配置错误。

现任务使用 `type: process`，直接运行：

```text
D:/msys64/mingw64/bin/mingw32-make.exe -j4
```

任务包括 build task、download jlink、download dap、log。下载任务依赖 build task，构建失败不会继续下载。

### 固件构建

```powershell
cd D:\autoshot_rm\cool-sentry\new_framework
mingw32-make -j4
```

输出：`build/basic_framework.elf`、`.hex`、`.bin`。

已通过 ARM GCC 15.2.1 构建。RAM 使用 80568 B，Flash 使用 91884 B。链接器有裸机 libc 的 `_read/_write/_close/_lseek` 提示，但本次构建成功。

当前工程缺失的 `bsp/log/bsp_log.c/.h` 已依据原工程恢复 SEGGER RTT 实现，并补充必要的标准库声明。

### JFlash 烧录

安装路径：`D:\ozone+jlink\JLink_V938\JFlash.exe`。

Makefile 已使用完整路径，避免 JFlash 未加入 PATH 导致“系统找不到指定的文件”。

```powershell
mingw32-make download_jlink
```

实际调用：

```text
"D:/ozone+jlink/JLink_V938/JFlash.exe" -openprj"stm32.jflash" -open"build/basic_framework.hex",0x08000000 -auto -startapp -exit
```

当前 `stm32.jflash` 已同步原工程成功更新的 V9.38 配置，芯片为 STM32F407IG，固件路径指向当前 new_framework。原配置备份为 `stm32.jflash.before-v938.bak`。

烧录或自动内存验证前结束 Ozone 调试，释放 J-Link。

## 6. Ozone 排查经验

- 安装位置：`D:\ozone+jlink\Ozone.exe`。
- J-Link 序列号：602712356。
- 优先选择具体芯片 STM32F407IG，检查 Flash Banks 已配置。通用 Cortex-M4 和 SVD 寄存器描述不能替代具体 Flash 烧录配置。
- 当前 ELF 路径：`D:\autoshot_rm\cool-sentry\new_framework\build\basic_framework.elf`。
- 新建工程中 Initial PC、Initial Stack Pointer 都选择 Read from Base Address Vector Table。
- 使用 `Project.SetOSPlugin("FreeRTOSPlugin_Cortex-M");` 启用任务显示。插件只用于调试显示，不影响固件调度或通信。
- Watched Data 的 Refresh 为 Off 时不会自动刷新；需要为变量设置刷新率。
- 暂停 CPU 会停止姿态发送并影响超时测试，收发验证时应保持运行。
- 之前异常的四元数显示不能直接认定为算法发散：当时发现板上复位向量与加载 ELF 不一致，错误的符号地址会产生误导。
- 不再把“连接成功”当成“烧录成功”；应核对板上固件、下载结果和串口协议。

## 7. Windows 可重复验证命令

本机可用 Python：`python`（MSYS2 Python 3.14），已安装 pyserial 3.5。`py -3` 在本次环境中不可用。

只读姿态：

```powershell
python host_tests\windows_serial_probe.py --port COM7 --baud 921600 --seconds 10
```

模拟视觉命令，不请求开火：

```powershell
python host_tests\windows_serial_probe.py --port COM7 --baud 921600 --seconds 15 --send-test
```

直接读取板上快照并验证：

```powershell
python host_tests\windows_jlink_command_check.py
```

后一脚本使用当前本机 DLL 路径、COM7、当前固件结构布局，并核对当前固件特定函数位置字节。它是本次联调工具；换串口、换安装路径或重新修改固件后，需要核对或更新这些设置。运行时需要 J-Link 空闲、CPU 正在运行。

## 8. 切换 Ubuntu 后的下一步

### 8.1 同步工程

将包含本次修改的整个 cool-sentry 项目同步到 Ubuntu。不要仅拉取远端仓库后默认本地修复也在里面；本次修改尚未由本次操作提交或推送。

切换操作系统后 J-Link 不必参与只读串口测试，保留 C 板供电和 USB 转 TTL 连接。

### 8.2 找到串口

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

优先把 `configs/sentry_step1.yaml` 的 `serial_port` 设置为实际 `/dev/serial/by-id/...` 路径；若 `/dev/rm_serial` 已有正确规则，可继续使用。临时也可使用实际的 `/dev/ttyUSB0`，但设备编号可能变化。

如果报 Permission denied，检查设备权限及当前账号是否属于串口设备组；常见 Ubuntu 串口组为 dialout。添加组权限后需重新登录。

### 8.3 构建视觉工程

在 Ubuntu 的 cool-sentry 项目根目录执行：

```bash
cmake -S . -B build_clean -DBUILD_DEBUG_TOOLS=ON
cmake --build build_clean -j2
```

依赖及 OpenVINO 配置见根目录 `readme.md`。本次 Windows 联调没有重新验证 Ubuntu 上的依赖安装情况。

### 8.4 先测姿态，不接摄像头

```bash
./build_clean/cboard_test configs/sentry_step1.yaml
```

验收：持续收到有效姿态，静止时基本稳定，转动 C 板时发生合理变化。

如无数据，依次检查供电、串口路径、权限、端口占用、TX/RX/GND、921600 波特率及固件是否运行。

### 8.5 再接摄像头

串口只读验证通过后，再接海康相机：

```bash
./build_clean/camera_test -d
./build_clean/camera_detect_test configs/sentry_step1.yaml
```

相机、检测分别验证通过后，运行主程序：

```bash
./build_clean/standard configs/sentry_step1.yaml
```

保持 `auto_fire: false`。现有相机内参、手眼外参、偏移和弹速来自已有配置，尚未为当前实车重新验证。

## 9. 尚未完成，不能当作已通过

- Ubuntu 实际串口连接和视觉程序运行。
- 海康相机实拍、检测精度和相机标定。
- 云台坐标系、轴方向、零点、绝对角与电控角度单位的映射。
- 将视觉命令正式接入云台和发射任务，加入遥控启用、机械限位和急停约束。
- 比赛状态、阵营、实测弹速、热量和弹量等字段扩展。
- 实弹测试。
- 视觉程序串口物理断连后的自动重连；当前文档说明接收线程错误后需重启视觉程序。

下一阶段优先目标：在 Ubuntu 无摄像头条件下运行 cboard_test，复现 Windows 已通过的有效姿态接收。

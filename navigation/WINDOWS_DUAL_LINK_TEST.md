# Windows：烧录并验证导航 USB + 视觉 USART

适用工程：<https://github.com/BoLixxi/cool-sentry>。
继续使用我们之前的 `new_framework` C 板框架、VSCode、Arm GCC 和 JLink。
本轮验收目标是 **裸板双路通信与超时保护**，不是让实车运动或射击。

## 1. 在 Windows 拉取新代码

在 PowerShell 中进入你原来的 **cool-sentry 仓库根目录**，不是 new_framework 子目录：

```powershell
git status
git pull --ff-only origin main
```

如果本地有尚未提交的修改，先保存或提交自己的修改，再拉取；不要使用 `git reset --hard` 覆盖。
若提示分支分叉，先停下，保留错误信息，不要强制覆盖。

如果 Windows 还没有这个仓库：

```powershell
git clone https://github.com/BoLixxi/cool-sentry.git
cd cool-sentry
```

确认存在这三个文件，说明拿到了本次改动：

```text
navigation\dual_link_probe.py
new_framework\modules\navigation\navigation.c
navigation\WINDOWS_DUAL_LINK_TEST.md
```

压缩包里的 StandardRobotpp、chuanchaun 仅作为协议参考，**不烧录它们的工程**。

## 2. 检查工具与裸板模式

VSCode 打开仓库根目录，打开 PowerShell 终端：

```powershell
arm-none-eabi-gcc --version
mingw32-make --version
py -3 --version
py -3 -m pip install pyserial
```

若工具找不到，沿用此前 Windows 的工具安装目录并修正 PATH，然后重启 VSCode。
Python 只用于串口验证，不需要在 Windows 安装 ROS2、海康 SDK 或完整视觉依赖。

确认 `new_framework\application\robot_def.h` 中：

```c
#define SENTRY_LINK_BENCH_ONLY 1
```

默认值就是1。这会跳过底盘电机注册和底盘控制任务。云台、发射任务也尚未启用。
本轮用裸 C 板测试，保持电机与发射机构断开；不修改这个宏为0。

## 3. 编译新固件

```powershell
cd new_framework
mingw32-make -j4
```

必须编译成功，并生成：

```text
build\basic_framework.elf
build\basic_framework.hex
build\basic_framework.bin
```

只看到旧文件还在不代表编译成功。遇到错误先处理第一条实际错误，不要烧旧固件。
如果需要排除旧构建缓存，可在确认当前目录为 new_framework 后执行：

```powershell
mingw32-make clean
mingw32-make -j4
```

常见错误：

|现象|处理|
|---|---|
|找不到 arm-none-eabi-gcc|检查原 Windows Arm 工具链的 PATH|
|找不到 navigation.h / NavInit|确认拉取完整、使用本仓库 new_framework 的 Makefile|
|找不到 libCMSISDSP.a|检查 Middlewares\ST\ARM\DSP\Lib 文件是否完整|
|不支持 --no-warn-rwx-segments|用原已验证的新工具链，或只移除这一警告抑制选项|

GitHub 不保存 ELF/HEX 等构建产物，Windows 拉取后重新编译即可。

## 4. 用 JLink 烧录

沿用此前能正常烧录的 VSCode / Cortex-Debug 或 JFlash 设置：

1. 设备 STM32F407IG，接口 SWD；连接 C 板供电和 JLink。
2. HEX 选择本次编译的 `new_framework\build\basic_framework.hex`。
3. 调试符号选择同次构建的 `new_framework\build\basic_framework.elf`。
4. 下载完成后让 CPU **继续运行**；F5 默认停在 main 时还不会通信。
5. 确认没有停在 HardFault、Error_Handler 或 BMI088 初始化循环。

如果使用 VSCode 的旧下载任务，不要只看“任务完成”：确认前面的编译成功，且烧录路径是本次产物。不要将旧 build_navigation 或其他工程的 ELF 与新 HEX 混用。

## 5. 同时接两条通信线

|连接|作用|参数|
|---|---|---|
|C板原生USB → 电脑USB|导航 USB CDC|脚本填 --usb COMx|
|C板串口 → CH340 → 电脑USB|视觉 USART|921600、8N1；脚本填 --vision COMy|
|JLink → C板SWD|烧录/调试|不是上述两个通信口|

USART 接线：CH340 TX → C板 PB7 RX；CH340 RX → C板 PA9 TX；GND共地。使用3.3V TTL，不接CH340 VCC，C板按此前方式独立供电。

在设备管理器里通过分别拔插原生USB和CH340确定对应COM口。**JLink自身也可能有COM口，不能当作C板导航USB。**
当前 new_framework 的原生 USB VID:PID 为 **0483:00CA**，不是对方工程的0483:5740。

关闭其他占用这两个COM口的程序：串口助手、旧probe、其他测试脚本。
C板 CPU保持运行，不要持续打断点或暂停，否则收发与超时测试会受影响。

## 6. 一键验证双链路

假设原生USB为COM6，CH340为COM5。在上一步 new_framework 目录中返回仓库根目录：

```powershell
cd ..
py -3 navigation\dual_link_probe.py --usb COM6 --vision COM5 --seconds 6 --send-test
```

**COM6和COM5必须换成你设备管理器里确定的实际端口。**

脚本运行过程：

|时间|发送内容|预期C板状态|
|---|---|---|
|0–2秒|USB速度 + USART跟踪指令|nav_valid=1、vision_valid=1、tracking=1|
|2–3秒|停止USB发送，USART继续|100ms后导航速度清零，视觉仍有效|
|3秒以后|两路发送都停止|100ms后两路均无效，角度/速度/跟踪/开火请求清零|

测试速度为 `(vx,vy,wz)=(0.2,-0.1,0.3)`。
视觉角度为 `(yaw,pitch)=(0.25,-0.5)` 弧度，control=1、shoot=0；**不发送开火请求**。
这里的数值只是通信快照，当前裸板模式不会将它们接入电机控制。

成功输出应包含：

```text
Verified: ['BOTH_LINKS', 'BOTH_TIMEOUT', 'NAV_TIMEOUT_ONLY']
PASS
```

最后还会显示USB收包数和USART四元数反馈数。这次USB诊断反馈包含了C板已经收到的两路数据，可以验证链路确实到达下位机。
不加 `--send-test` 就只接收，不能完成非零指令与超时验收。

## 7. 想在调试器中看数据

主要观察 `sentry_link_snapshot`：

```text
vx / vy / wz                 导航目标速度
yaw_rad / pitch_rad          视觉角度
nav_valid / vision_valid     两路通信是否有效
tracking                     视觉是否请求跟踪
shoot_requested              视觉开火请求；本轮应始终为0
```

也可观察 `nav_rx_frames`、`nav_tx_frames`、`nav_invalid_commands`、`vision_snapshot`。
这不是对方工程里的 `ROBOT_CMD_DATA.speed_vector`；我们保留自己的框架，因此观察变量不同。
`vision_valid=1` 表示近期收到有效通信帧，是否有目标还要看 tracking。

使用动态监视时让CPU持续运行；如果暂停后查看，暂停时间会影响接下来的超时判定，应重新运行整套脚本。

## 8. 出错时怎么判断

|现象|先检查|
|---|---|
|Access denied / 端口被占用|关闭串口助手和其他测试进程|
|COM口不存在|端口编号、USB线是否支持数据、C板是否上电|
|有USART四元数，USB帧为0|是否接原生USB而非JLink；是否烧录本轮新固件|
|有USB帧，USART四元数为0|CH340接线、COM口、921600、INS初始化|
|BOTH_LINKS未通过|两个端口是否填反，CPU是否运行，固件是否与ELF一致|
|超时项未通过|是否其他程序继续发指令、CPU是否暂停|

不要运行旧 `host_tests\windows_jlink_command_check.py` 代替本轮测试；它有旧版本固定地址/端口假设，不能用于本次双链路验收。

测试结束后保留：完整终端输出、两个COM口对应关系、实际烧录文件路径。
三项PASS后回Ubuntu做“真实导航ROS2 + 真实视觉程序”同时运行，步骤见 [README.md](README.md)。

目前已完成软件模拟、ROS2虚拟串口验证和ARM完整构建；这份文档的实板步骤需要由你执行，尚不能声称实车导航与射击已打通。

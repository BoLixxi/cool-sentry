# Windows 裸板双链路实测结果

日期：2026-10-13。源码版本：fc503ea。

## 结论

导航 USB CDC 与视觉 USART 同时收发，以及独立超时保护，已在真实 C 板上通过。Windows 脚本模拟导航速度和视觉角度，不代表真实 ROS2 导航、相机自瞄或实车控制已经通过。

## 环境及端口

- 工程：D:\autoshot_rm\cool-sentry\new_framework。
- 裸板模式：SENTRY_LINK_BENCH_ONLY=1，底盘控制未启用，云台/发射任务未启用。
- 原生导航 USB：COM8，VID:PID=0483:00CA。
- 视觉 CH340：COM7，VID:PID=1A86:7523，921600、8N1。
- COM6 是 J-Link CDC，不用于这两条通信链路。
- ARM GCC 15.2.1，mingw32-make，Python/pyserial，JFlash V9.38。

## 软件与烧录验证

- Python 协议 4 项测试通过。
- navigation_module_test 主机模拟通过，包括分片、CRC、双路快照、超时、断连和发送缓冲生命周期。
- 完整 ARM 构建成功，RAM 80408 B，FLASH 90396 B。
- JFlash 对本次 build/basic_framework.hex 擦除、编程及校验成功。原始烧录日志在工作区上层 D:\autoshot_rm\dual-link-flash.log，不作为仓库源码上传。

## 实板测试

在 cool-sentry 仓库根目录运行：

```powershell
python navigation\dual_link_probe.py --usb COM8 --vision COM7 --seconds 6 --send-test
python navigation\dual_link_probe.py --usb COM8 --vision COM7 --seconds 15 --send-test
```

脚本保持 shoot=0。0–2 秒两路发送，2–3 秒只发送视觉命令，3 秒后两路都停发；测试结束发送零速度及取消视觉目标。

| 验收项 | C 板 USB 诊断回传 | 结果 |
|---|---|---|
| BOTH_LINKS | 速度=(0.2,-0.1,0.3)，yaw=0.25、pitch=-0.5；valid=1/1、tracking=1、fire=0 | 通过 |
| NAV_TIMEOUT_ONLY | 速度清零，nav_valid=0；视觉角度保留、vision_valid=1、tracking=1 | 通过 |
| BOTH_TIMEOUT | 速度和角度全零，valid=0/0、tracking=0、fire=0 | 通过 |

6 秒轮次：USB ID 02/1E/E0 分别 199/200/200 帧，USART 姿态 599 帧，拒绝帧 0/0。

15 秒轮次：USB ID 02/1E/E0 各 500 帧，USART 姿态 1500 帧，拒绝帧 0/0。

两轮均输出：

```text
Verified: ['BOTH_LINKS', 'BOTH_TIMEOUT', 'NAV_TIMEOUT_ONLY']
PASS
```

这些结果来自 C 板实际回传的命令快照，不只是电脑写串口成功。脚本在停发后留有超过 100 ms 的观察窗口；未测量精确超时触发延迟。

## 启动现象及限制

刚烧录后曾出现 COM8 消失且 USART 无输出。J-Link 诊断时无 HardFault 标志，采样 PC 位于 BMI088 初始化中的 DWT_Delay；一次连续采样中 CYCCNT 没有变化。后续诊断连接及恢复运行后 COM8 重现，两轮实板测试通过。

目前没有足够证据确认最初启动延迟的根因；不把它直接归结为 USB 协议、IMU 损坏或调试器关闭计数器。本轮未修改固件源码。

随后用户重新上电，未启动 Ozone/JFlash 调试进程，也未通过 J-Link 连接或恢复 CPU，直接运行 6 秒双口脚本。COM8/COM7 正常，USB ID 02/1E/E0 各 200 帧，USART 姿态 600 帧，拒绝帧 0/0，三项验收全部 PASS。这证明本次冷上电可以自主启动双路通信。尚未完成多次冷启动、反复复位、USB 热插拔或长时间压力验证。

## 下一步

1. 本次裸板冷上电及 6 秒复测已通过；部署前还可补充多次冷启动和热插拔验证。
2. Ubuntu 按 navigation/README.md 配置 /dev/rm_nav 和 /dev/rm_serial，先重复双口 probe。
3. 停止 probe 后运行 sentry_cboard_bridge，手工发布 cmd_vel，检查 /serial/link_snapshot。
4. 保持 auto_fire=false，运行真实 standard 视觉程序，检查两路快照同时有效。
5. 最后用真实导航发布器替代手工 cmd_vel；不要同时启动旧串口节点或多个速度发布器。

本次尚未验证雷达、定位、Nav2、相机识别、坐标系映射、底盘/云台动作或开火。

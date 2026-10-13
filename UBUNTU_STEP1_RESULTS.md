# Ubuntu 接力验证结果

日期：2026-10-11。基于 Git 提交 `97c804b` 的 Windows 交接说明执行。

## 已验证

| 项目 | 结果 |
|---|---|
| CMake 配置与完整视觉构建 | 通过，调试工具全部构建 |
| USART 真实设备 | `/dev/rm_serial` → `/dev/ttyUSB0`，CH340，账号有 dialout 权限 |
| Ubuntu C++ 只读姿态测试 | 约 10 秒，998 次姿态输出，约 100 Hz，未见 timeout/stale/串口错误 |
| 静止姿态 | yaw 约 14.26°、pitch 约 0.64°、roll 约 −1.00°，观察期内稳定；未验证转动响应及坐标轴方向 |
| SeaSky 与旧协议虚拟串口测试 | 通过，覆盖分片、噪声、坏校验、过期与断流 |
| 电控模块主机模拟 | CRC、分片、超时、快照和 DMA 缓冲生命周期测试通过 |
| 海康相机 | 系统检测到 MV-CS016-10UC；完整 SDK 路径下约 6 秒采集 505 帧，稳定段约 98 fps |
| 海康实时装甲板模型推理 | 10 秒进程测试中处理 885 帧，全部 0 armors，无异常退出；不能据此判断识别精度 |

定时测试通过 timeout 在时限后发 SIGINT，成功清理的程序也返回 124，这是计时退出码，不是业务故障。测试没有运行 standard，没有发送云台/开火命令。

## 相机故障与修正

最初链接仓库 `io/hikrobot/lib/amd64/libMvCameraControl.so` 时，枚举返回 `0x80000006`（资源申请失败），没有收到图像。仓库只有主库，其动态加载的配套 USB 等库缺失。用已安装完整 `/opt/MVS/lib/64` SDK 后采集成功。

本次没有添加 MV_CC_Initialize：仓库旧 SDK 没有该导出符号，实际运行证据表明本机完整 SDK 路径已解决问题，不能把 SDK 初始化推断作为已确认根因。

`io/CMakeLists.txt` 新增 `HIKROBOT_SDK_LIB_DIR`；x86_64 下检测到本机 `/opt/MVS/lib/64` 时优先使用，仍保留仓库库路径作为回退。自定义安装路径可传：

```bash
cmake -S . -B build_clean -DBUILD_DEBUG_TOOLS=ON -DHIKROBOT_SDK_LIB_DIR=/实际SDK库目录
cmake --build build_clean -j2
```

已用 ldd 确认相机程序实际加载 `/opt/MVS/lib/64/libMvCameraControl.so`。本机调试环境无图形桌面，相机检测工具新增 `--display=false` 并记录检测数量；窗口默认行为保留。

## 可重复执行

在工程根目录：

```bash
./build_clean/cboard_test configs/sentry_step1.yaml
./build_clean/camera_test -c=configs/sentry_step1.yaml
./build_clean/camera_detect_test configs/sentry_step1.yaml --display=false
```

本地桌面查看画面：

```bash
./build_clean/camera_test -d -c=configs/sentry_step1.yaml
./build_clean/camera_detect_test configs/sentry_step1.yaml
```

按 Ctrl+C 结束；图像窗口也可按 q。沙箱内无法直接看到真实设备，需要主机设备访问权限；主机普通终端不应把沙箱的“设备不存在”误认为串口驱动故障。

## 下一步

在镜头前放真实亮灯装甲板，确认画面中的亮度、数字和灯条能正确识别，检查 enemy_color 与目标颜色。随后重新核实相机内参、手眼外参、IMU/云台轴方向和零点；原配置还不能当作当前机器人已标定结果。保持 auto_fire=false。

Ubuntu 本次没有重做向电控发送并读板上快照的验证，Windows 已完成的双向验证见交接说明。未执行主程序、自瞄电机控制或实弹测试。目标识别和坐标映射确认前，不把“程序运行通过”当作瞄准精度通过。

详细原始日志位于临时目录：`/tmp/ubuntu_cboard_step1.log`、`/tmp/ubuntu_camera_step1.log`（最初失败）、`/tmp/ubuntu_camera_systemsdk.log`、`/tmp/ubuntu_detect_step1.log`；临时目录可能被清理，以本报告作为工程内记录。

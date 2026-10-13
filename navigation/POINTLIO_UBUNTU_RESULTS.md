# Point-LIO 裸机定位接力验证

后续已完成 RViz 手持移动与返回测试，结果及再次测量的方法见 [移动测试记录](POINTLIO_WALK_RESULTS.md)。

日期：2026-10-13。本轮没有启动 Nav2、速度发布器或 C 板机构控制。

## 已完成

1. GDB 捕获原驱动 SIGBUS 调用栈：系统 libspdlog.so.1 的 spdlog::logger 析构函数，由预编译 Livox SDK 的全局 shared_ptr 析构调用。这是退出阶段的日志符号/二进制兼容性问题线索，并非点云网络故障。
2. 下载官方 Livox SDK2 master 源码，以本机 GCC 11.4 构建共享库，链接选项加入 `-Wl,-Bsymbolic`，让 SDK 内部定义优先绑定自身符号，避免被系统另一份 spdlog 替换。没有通过跳过析构或强杀进程掩盖异常。
3. 使用新共享库完成独立驱动接收复测：104 帧点云、1790 帧 IMU，正常退出码 0。
4. 用户执行 apt update 后安装 ros-humble-pcl-ros；此前 404 是旧包索引指向已失效版本。
5. 构建 EW- 的 point_lio，使用 CustomMsg（xfer_format=1）连接 MID360，完成 30 秒定位输出测试。

| 项目 | 结果 |
|---|---|
| IMU 初始化 | 完成 |
| 有限值里程计 | 265 帧 |
| 配准点云 | 265 帧 |
| 初始化后位置最大偏离 | 0.0176 m（排除输出开始的前 5 秒） |
| 末次位置 | (-0.00601, 0.000071, -0.00827) m |
| Point-LIO / 雷达驱动退出码 | 0 / 0 |

位置变化仅是本次短时观察。脚本不能判断雷达是否确实静止，也不检查全部姿态/时间戳/TF。
没有完成移动轨迹精度、回环、已有地图重定位、Nav2 路径规划或整车运动验收。
原配置的 MID360 内部 LiDAR→IMU 外参保留；本车雷达→底盘安装外参仍需单独测量。

## 当前电脑复测

保持雷达供电和网线，检查有线 IP 192.168.1.50/24。不要同时启动另一套 Livox 驱动。
雷达放稳、周围有墙面和物体，初始化期间勿移动。

```bash
cd /home/xiaoli/Desktop/sp_vision_25-main
source navigation/deploy/setup_mid360.bash
python3 navigation/tests/mid360_stream_check.py
python3 navigation/tests/pointlio_boardless_check.py
```

两个脚本分别自动启动并停止节点。第二个运行约 30 秒，只输出定位验收，不发送 cmd_vel。
完整日志：/tmp/sentry-pointlio-test.log，临时目录可能被清理，以本记录为准。

本机的新 SDK 必须优先加载：navigation/build_mid360/sdk-build/sdk_core/liblivox_lidar_sdk_shared.so。
不要继续把旧 SDK 库目录放在 LD_LIBRARY_PATH 最前面，否则原 Bus error 可能恢复。
setup_mid360.bash 已设置正确的库路径及两个 ROS 工作区环境。

## 构建复现信息

EW- 来源提交：e3b24beff58eeb4ac823b2dc5638cd1d27cbe633。
官方 SDK 源码下载地址：https://codeload.github.com/Livox-SDK/Livox-SDK2/tar.gz/refs/heads/master。
本轮压缩包 SHA256：b5c234953dacb09f2e8871e602ed572e2e6dedc508c9a35b790d5b7befbca2dc。
master 会变化，后续部署应固定实际源码版本；此次不是只加链接选项的对照实验，不能单独证明重建或符号绑定哪一项消除了故障。

源码缓存 sdk_source_archive、SDK 构建 sdk-build、驱动 install、定位 lio-install 均在 navigation/build_mid360，目录被 Git 忽略。
新增的配置、环境脚本和验收脚本属于本仓库跟踪源码；不上传缓存或二进制。

在已下载源码的本机重建 SDK：

```bash
cmake -S navigation/build_mid360/sdk_source_archive \
  -B navigation/build_mid360/sdk-build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_SHARED_LINKER_FLAGS=-Wl,-Bsymbolic
cmake --build navigation/build_mid360/sdk-build \
  --target livox_lidar_sdk_shared -j2
```

重建 Point-LIO：

```bash
source /opt/ros/humble/setup.bash
source navigation/build_mid360/install/setup.bash
colcon --log-base navigation/build_mid360/lio-log build \
  --base-paths navigation/build_mid360/ew_source/EEEWW/src/pb2025_sentry_nav/point_lio \
  --build-base navigation/build_mid360/lio-build \
  --install-base navigation/build_mid360/lio-install \
  --packages-select point_lio --cmake-args -DBUILD_TESTING=OFF --parallel-workers 1
```

## 下一步学习和验证

先在 RViz 观察配准点云和轨迹，雷达固定在可手持的刚性支架上，再做缓慢移动、转动和回到起点测试。
这是局部里程计；不是场地已有地图上的全局重定位。
确认运动定位、坐标轴与 TF 后，再配置场地地图、重定位和 Nav2。
当前不把 Point-LIO 的 odometry 直接转换为底盘命令，也不启用裸板之外的电机控制。

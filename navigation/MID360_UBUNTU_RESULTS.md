# MID360 网络与数据接收验证

后续更新：退出异常已通过本机重建官方 SDK 并隔离其日志符号消除，Point-LIO 接力测试已完成，见 [Point-LIO 验证记录](POINTLIO_UBUNTU_RESULTS.md)。本机启动优先使用该记录中的新 SDK 环境。

日期：2026-10-13。雷达通过网线直连当前 Ubuntu 电脑，使用 EW- 仓库中的 Livox ROS Driver2 1.2.4。

## 已确认

- 有线网卡：enp7s0，物理链路 UP。
- 电脑 IP：192.168.1.50/24，用户在本机输入 sudo 密码后设置成功。
- MID360 实际响应地址：192.168.1.3，与 EW- 实机导航配置一致。
- ping 3 次全部响应，延迟 0.872–1.779 ms。
- 独立驱动收到真实 PointCloud2 和 IMU，没有启动底盘、Nav2 或定位。
- 12 秒测试（含约 3 秒驱动启动）：PointCloud2 102 帧、观察窗口平均 11.48 Hz，最大间隔 0.102 秒；IMU 1789 帧、201.25 Hz，最大间隔 0.007 秒。
- 点云每帧 19872–20064 点，抽样 XYZ 有限，例如 (4.225, -1.424, 1.638) m。
- 项目内驱动第二轮复测：102 帧点云、1786 帧 IMU，最大间隔分别 0.102/0.006 秒，每帧 19872–20160 点；首点也可能为 (0,0,0)，有限值检查不代表该点是有效距离回波。

点云启动时可能集中交付缓存数据，上述平均频率不代表稳态精确频率。驱动配置发布频率为 10 Hz。
测试只检查点数、数据长度和首点 XYZ，不等同于整幅点云质量、距离精度或定位验收。

## 发现的问题

1. EW- 的单独驱动示例 MID360_config.json 使用雷达 192.168.1.146；实机导航配置则是 192.168.1.3。新增 deploy/mid360_bench.json，明确使用当前实测地址和电脑地址。
2. 运行时缺少 liblivox_lidar_sdk_shared.so 搜索路径，需将仓库自带 amd64 SDK 库目录加入 LD_LIBRARY_PATH。
3. 两轮接收阶段正常，两轮 SIGINT 退出均出现 Bus error；第二轮脚本正确报告退出失败（ros2run exit 249）。日志先显示 SDK Deinit completely 和 lddc 析构输出，随后 ros2run 报错；没有残留驱动进程。尚无堆栈证据定位根因，不声称驱动退出稳定性已通过。

验收脚本已把非零退出码单独判为失败，避免只看到传感器接收 PASS 就忽略退出异常。下一阶段应先复测并排查退出，再启动完整导航。

## 当前电脑复测

导航源码和雷达驱动构建缓存在 navigation/build_mid360，已被 gitignore 忽略；不将另一套导航工程混入本仓库的跟踪源码。
复制来源是用户提供的 https://github.com/BoLixxi/EW- ，仅构建 livox_ros_driver2。
更换机器后须重新克隆源码并构建，不能复用本机生成的 install。

```bash
cd /home/xiaoli/Desktop/cool-sentry
source /opt/ros/humble/setup.bash
source navigation/build_mid360/install/setup.bash
export ROS_LOG_DIR=/tmp/sentry-mid360-ros-logs
export LD_LIBRARY_PATH="$PWD/navigation/build_mid360/ew_source/EEEWW/src/pb2025_sentry_nav/livox_ros_driver2/Livox-SDK2/lib/amd64:$LD_LIBRARY_PATH"
python3 navigation/tests/mid360_stream_check.py --seconds 12
```

脚本自动启动和停止独立驱动，使用独立 ROS namespace；复测前停止其他 Livox 驱动，避免抢占相同 UDP 端口。
配置使用 xfer_format=0 发布 PointCloud2，正式 Point-LIO 启动仍按其 CustomMsg 需求使用导航原配置。
未启动任何速度发布器或 C 板通信程序。

当前 IP 是临时设置，重启或网络重新连接后可能失效。再次查看：

```bash
ip -br addr show enp7s0
# 仅在没有 192.168.1.50/24 时执行；输入本机 Ubuntu 密码，不向聊天发送密码
sudo ip addr add 192.168.1.50/24 dev enp7s0
ping -c 3 192.168.1.3
```

长期部署时，在 Ubuntu 网络设置中把雷达专用有线连接设为手动 IPv4，地址 192.168.1.50、掩码 255.255.255.0，不设置网关；Wi-Fi 保持原来的联网配置。

## 下一阶段

先处理或明确驱动退出问题，再构建并单独验证 Point-LIO、地图与 TF。雷达应固定，静止初始化后再缓慢移动；安装姿态和坐标变换需按本车配置。
当前没有实车，不能验证轮速里程计、定位闭环和底盘运动。实机导航的最终速度话题须核对，EW- 记录提到 /red_standard_robot1/cmd_vel，不能默认就是 /cmd_vel。
新 sentry_cboard_bridge 负责唯一的 C 板 USB 通信，旧 standard_robot_pp_ros2 串口节点应关闭。

终端顶部曾显示旧仓库目录下的 `install/setup.bash` 不存在，是 `~/.bashrc` 里的旧 source 路径；正确桥接路径为 `navigation/install/setup.bash`。此次未修改用户的 `~/.bashrc`。
